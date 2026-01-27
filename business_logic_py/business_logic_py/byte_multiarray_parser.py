#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, String, Int8, Empty, Float64
import struct
from enum import Enum

class CommandType(Enum):
    # 根据图片中的指令码定义
    NOTIFY_STORAGE = 0x101       # 通知存放 /warehouse_start
    NOTIFY_RETRIEVAL = 0x103     # 通知取出 /outbound_start
    END_OPERATION = 0x107        # 结束作业 /warehouse_stop /outbound_stop
    AXIS_JOG = 0x10D             # 轴点动 /jog_command
    AXIS_STOP = 0x10F            # 轴停止 /jog_command
    WRITE_IO = 0x115             # 写IO /do_control
    CLEAR_AXIS_FAULT = 0x117     # 清除轴故障 /control_command
    
    # 原有的指令码定义（不在图片中的）
    SYSTEM_CONTROL = 0x01
    JOG_SPEED = 0x03
    POSITION_CONTROL = 0x04
    LAYER_COMMAND = 0x05
    BOARD_WIDTH = 0x0B

class ByteMultiArrayParser(Node):
    def __init__(self):
        super().__init__('byte_multiarray_parser')
        
        # 订阅统一的ByteMultiArray话题
        self.unified_sub = self.create_subscription(
            ByteMultiArray,
            '/integrated_control',
            self.unified_callback,
            10
        )
        
        # 创建原有的多个话题发布器
        self.control_pub = self.create_publisher(String, '/control_command', 10)
        self.jog_pub = self.create_publisher(String, '/jog_command', 10)
        self.jog_speed_pub = self.create_publisher(String, '/jog_speed_command', 10)
        self.displacement_pub = self.create_publisher(String, '/displacement_command', 10)
        self.layer_pub = self.create_publisher(Int8, '/layer_command', 10)
        self.warehouse_start_pub = self.create_publisher(Int8, '/warehouse_start', 10)
        self.warehouse_stop_pub = self.create_publisher(Empty, '/warehouse_stop', 10)
        self.outbound_start_pub = self.create_publisher(Int8, '/outbound_start', 10)
        self.outbound_stop_pub = self.create_publisher(Empty, '/outbound_stop', 10)
        self.do_control_pub = self.create_publisher(String, '/do_control', 10)
        self.board_width_pub = self.create_publisher(Float64, '/board_width_command', 10)
        
        # 添加IO状态跟踪
        self.last_io_state = 0  # 初始状态为全0
        
        self.get_logger().info('ByteMultiArray解析器已启动（支持0-13位IO控制）')

    def ensure_int(self, value):
        """确保值为整数类型"""
        if isinstance(value, int):
            return value
        elif isinstance(value, (bytes, bytearray)):
            return int.from_bytes(value, byteorder='little')
        else:
            try:
                return int(value)
            except (ValueError, TypeError):
                self.get_logger().warn(f'无法转换为整数: {value}')
                return 0

    def unified_callback(self, msg):
        """处理统一的ByteMultiArray消息"""
        if len(msg.data) == 0:
            self.get_logger().warn('收到空数据消息')
            return
            
        try:
            # 关键修复：确保所有数据都是整数类型
            data_list = []
            for item in msg.data:
                data_list.append(self.ensure_int(item))
            
            # 根据图片，指令码可能是16位（2字节）
            # 假设是小端序，先尝试解析为16位整数
            if len(data_list) >= 2:
                command_code = (data_list[1] << 8) | data_list[0]  # 小端序
                payload = data_list[2:]  # 去除命令码后的负载数据
            else:
                # 如果数据长度不足，尝试单字节命令码
                command_code = data_list[0]
                payload = data_list[1:]
            
            self.get_logger().info(f'解析命令: 0x{command_code:04X}, 负载长度: {len(payload)}')
            
            # 根据命令码分发处理
            if command_code == CommandType.NOTIFY_STORAGE.value:
                self.process_notify_storage(payload)
            elif command_code == CommandType.NOTIFY_RETRIEVAL.value:
                self.process_notify_retrieval(payload)
            elif command_code == CommandType.END_OPERATION.value:
                self.process_end_operation(payload)
            elif command_code == CommandType.AXIS_JOG.value:
                self.process_axis_jog(payload)
            elif command_code == CommandType.AXIS_STOP.value:
                self.process_axis_stop(payload)
            elif command_code == CommandType.WRITE_IO.value:
                self.process_write_io(payload)
            elif command_code == CommandType.CLEAR_AXIS_FAULT.value:
                self.process_clear_axis_fault(payload)
            # 原有的指令码处理
            elif command_code == CommandType.SYSTEM_CONTROL.value:
                self.process_system_control(payload)
            elif command_code == CommandType.JOG_SPEED.value:
                self.process_jog_speed(payload)
            elif command_code == CommandType.POSITION_CONTROL.value:
                self.process_position_control(payload)
            elif command_code == CommandType.LAYER_COMMAND.value:
                self.process_layer_command(payload)
            elif command_code == CommandType.BOARD_WIDTH.value:
                self.process_board_width(payload)
            else:
                self.get_logger().warn(f'未知命令码: 0x{command_code:04X}')
                
        except Exception as e:
            self.get_logger().error(f'消息解析错误: {e}')

    def process_write_io(self, payload):
        """处理写IO命令 (0x115) - /do_control，支持状态翻转检测"""
        # IO地址映射表（0-15位）
        io_mapping = {
            0: 800,   # 启动按钮灯
            1: 801,   # 复位按钮灯
            2: 802,   # 暂停按钮灯
            3: 803,   # 蜂鸣器
            4: 804,   # 三色红灯
            5: 805,   # 三色黄灯
            6: 806,   # 三色绿灯
            7: 807,   # 预留
            8: 808,   # 预留
            9: 809,   # 预留
            10: 810,  # 顶升气缸下降
            11: 811,  # 齿轮对接气缸伸出
            12: 812,  # 皮带正转启动
            13: 813,  # 皮带反转启动
            14: 814,  # 预留
            15: 815   # 预留
        }
        
        # 检查负载长度
        if len(payload) < 2:
            self.get_logger().warn('写IO命令负载长度不足，需要至少2字节')
            return
        
        # 解析两个字节（小端序）
        byte0 = payload[0]  # 低字节，位0-7
        byte1 = payload[1]  # 高字节，位8-15
        
        # 合并为16位整数
        current_io_state = (byte1 << 8) | byte0
        
        self.get_logger().info(f'写IO命令解析: 当前状态=0x{current_io_state:04X}, 上次状态=0x{self.last_io_state:04X}')
        
        # 遍历所有位（0-15位）
        for bit_position in range(16):
            if bit_position in io_mapping:
                do_number = io_mapping[bit_position]
                
                # 获取当前位和上一次的位状态
                current_bit = (current_io_state >> bit_position) & 0x01
                last_bit = (self.last_io_state >> bit_position) & 0x01
                
                # 检查是否发生翻转
                if current_bit != last_bit:
                    # 状态发生变化，发布命令
                    command_str = f"{do_number}:{current_bit}"
                    msg = String()
                    msg.data = command_str
                    self.do_control_pub.publish(msg)
                    
                    if current_bit == 1:
                        self.get_logger().info(f'检测到翻转: {do_number} 从0变为1')
                    else:
                        self.get_logger().info(f'检测到翻转: {do_number} 从1变为0')
                    
                    self.get_logger().info(f'发布DO控制命令: {command_str} (位{bit_position})')
                else:
                    # 状态未变化，不发布命令
                    self.get_logger().debug(f'状态未变化: {do_number} 保持{current_bit}')
        
        # 更新上一次状态
        self.last_io_state = current_io_state

    def process_notify_storage(self, payload):
        """处理通知存放命令 (0x101) - /warehouse_start"""
        if len(payload) >= 2:
            layer = payload[0] | (payload[1] << 8)  # 小端序
            msg = Int8()
            msg.data = layer
            self.warehouse_start_pub.publish(msg)
            self.get_logger().info(f'发布仓库启动命令: 第{layer}层')
        else:
            self.get_logger().warn('通知存放命令负载长度不足')

    def process_notify_retrieval(self, payload):
        """处理通知取出命令 (0x103) - /outbound_start"""
        if len(payload) >= 2:
            layer = payload[0] | (payload[1] << 8)  # 小端序
            msg = Int8()
            msg.data = layer
            self.outbound_start_pub.publish(msg)
            self.get_logger().info(f'发布出库启动命令: 第{layer}层')
        else:
            self.get_logger().warn('通知取出命令负载长度不足')

    def process_end_operation(self, payload):
        """处理结束作业命令 (0x107) - /warehouse_stop /outbound_stop"""
        # 根据表格：出入库同时停止（不管数据）
        msg = Empty()
        self.warehouse_stop_pub.publish(msg)
        self.outbound_stop_pub.publish(msg)
        self.get_logger().info('发布仓库/出库停止命令')

    def process_axis_jog(self, payload):
        """处理轴点动命令 (0x10D) - /jog_command"""
        if len(payload) >= 4:
            # 假设前2字节是轴号，第3-4字节是方向
            axis_num = payload[0] | (payload[1] << 8)  # 小端序
            direction = payload[2] | (payload[3] << 8)  # 小端序
            
            # 将轴号转换为字符串格式
            axis_name = f"axis{axis_num}"
            
            # 方向映射
            direction_map = {
                0: "stop",
                1: "forward",  # 假设1为正转
                2: "reverse"   # 假设2为反转
            }
            
            direction_str = direction_map.get(direction, "stop")
            command_str = f"{axis_name}:{direction_str}"
            
            msg = String()
            msg.data = command_str
            self.jog_pub.publish(msg)
            self.get_logger().info(f'发布点动命令: {command_str}')
        else:
            self.get_logger().warn('轴点动命令负载长度不足')

    def process_axis_stop(self, payload):
        """处理轴停止命令 (0x10F) - /jog_command"""
        # 根据表格：所有轴停止
        command_str = "all:stop"
        msg = String()
        msg.data = command_str
        self.jog_pub.publish(msg)
        self.get_logger().info('发布所有轴停止命令')

    def process_clear_axis_fault(self, payload):
        """处理清除轴故障命令 (0x117) - /control_command"""
        # 根据表格：清除轴故障
        command_str = "clear_fault"
        msg = String()
        msg.data = command_str
        self.control_pub.publish(msg)
        self.get_logger().info('发布清除轴故障命令')

    # 原有的处理函数保持不变
    def process_system_control(self, payload):
        """处理系统控制命令 (0x01)"""
        if len(payload) < 1:
            self.get_logger().warn('系统控制命令负载长度不足')
            return
            
        sub_command = payload[0]
        command_map = {
            0x01: "start_manual",
            0x02: "start_auto", 
            0x03: "stop",
            0x04: "clear_fault",
            0x05: "reset"
        }
        
        if sub_command in command_map:
            command_str = command_map[sub_command]
            msg = String()
            msg.data = command_str
            self.control_pub.publish(msg)
            self.get_logger().info(f'发布系统控制命令: {command_str}')
        else:
            self.get_logger().warn(f'未知系统控制子命令: 0x{sub_command:02X}')

    def process_jog_speed(self, payload):
        """处理点动速度命令 (0x03)"""
        if len(payload) < 5:  # 1字节轴ID + 4字节浮点数
            self.get_logger().warn('点动速度命令负载长度不足')
            return
            
        axis_id = payload[0]
        # 将4字节转换为浮点数 (小端序)
        speed_bytes = bytes(payload[1:5])
        try:
            speed = struct.unpack('<f', speed_bytes)[0]
            axis_name = f"axis{axis_id}"
            
            command_str = f"{axis_name}:{speed:.1f}"
            msg = String()
            msg.data = command_str
            self.jog_speed_pub.publish(msg)
            self.get_logger().info(f'发布点动速度命令: {command_str}')
        except Exception as e:
            self.get_logger().error(f'速度值解析错误: {e}')

    def process_position_control(self, payload):
        """处理位置控制命令 (0x04)"""
        if len(payload) < 5:  # 1字节轴ID + 4字节浮点数
            self.get_logger().warn('位置控制命令负载长度不足')
            return
            
        axis_id = payload[0]
        # 将4字节转换为浮点数 (小端序)
        position_bytes = bytes(payload[1:5])
        try:
            position = struct.unpack('<f', position_bytes)[0]
            axis_name = f"axis{axis_id}"
            
            # 发布到位移命令话题
            command_str = f"{axis_name}:{position:.1f}"
            msg = String()
            msg.data = command_str
            self.displacement_pub.publish(msg)
            self.get_logger().info(f'发布位置命令: {command_str}')
        except Exception as e:
            self.get_logger().error(f'位置值解析错误: {e}')

    def process_layer_command(self, payload):
        """处理层命令 (0x05)"""
        if len(payload) < 1:
            self.get_logger().warn('层命令负载长度不足')
            return
            
        layer = payload[0]
        msg = Int8()
        msg.data = layer
        self.layer_pub.publish(msg)
        self.get_logger().info(f'发布层命令: 第{layer}层')

    def process_board_width(self, payload):
        """处理板宽命令 (0x0B)"""
        if len(payload) < 4:  # 4字节浮点数
            self.get_logger().warn('板宽命令负载长度不足')
            return
            
        # 将4字节转换为浮点数 (小端序)
        width_bytes = bytes(payload[0:4])
        try:
            width = struct.unpack('<f', width_bytes)[0]
            msg = Float64()
            msg.data = float(width)
            self.board_width_pub.publish(msg)
            self.get_logger().info(f'发布板宽命令: {width:.1f}')
        except Exception as e:
            self.get_logger().error(f'板宽值解析错误: {e}')

def main(args=None):
    rclpy.init(args=args)
    parser_node = ByteMultiArrayParser()
    
    try:
        rclpy.spin(parser_node)
    except KeyboardInterrupt:
        parser_node.get_logger().info('ByteMultiArray解析器被用户中断')
    finally:
        parser_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()