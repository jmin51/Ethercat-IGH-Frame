#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8, Empty, Bool
import time
from enum import Enum, auto
from typing import Dict, List, Optional
import re

class WarehouseState(Enum):
    IDLE = auto()
    WAIT_FOR_ENTRY = auto()
    CONVEYOR_MOVING = auto()
    LIFT_MOVING = auto()
    POST_LIFT_PROCESSING = auto()  # 新增：层移动后处理状态
    DELAY_PROCESSING = auto()      # 新增：延迟处理状态
    COMPLETED = auto()

class OutboundState(Enum):
    IDLE = auto()
    WAIT_FOR_EXIT = auto()
    LIFT_MOVING = auto()
    POST_LIFT_PROCESSING = auto()  # 新增：层移动后处理状态
    CONVEYOR_MOVING = auto()
    COMPLETED = auto()

class CommandType(Enum):
    JOG = auto()
    LAYER = auto()
    POSITION = auto()
    STOP = auto()

class ControlAction:
    def __init__(self, cmd_type: CommandType, axis_name: str, command_value: str, 
                 target_position: float = 0.0, description: str = ""):
        self.type = cmd_type
        self.axis_name = axis_name
        self.command_value = command_value
        self.target_position = target_position
        self.description = description

class BusinessLogicProcessor(Node):
    def __init__(self):
        super().__init__('business_logic_processor')
        
        # 初始化状态变量
        self.warehouse_state = WarehouseState.IDLE
        self.outbound_state = OutboundState.IDLE
        self.current_layer = 1
        self.target_layer = 1
        self.source_layer = 1
        
        # 添加上一个状态记录
        self.previous_warehouse_state = WarehouseState.IDLE
        self.previous_outbound_state = OutboundState.IDLE
        self.previous_io_signals = {
            'buffer_in_position': False,
            'buffer_out_position': False,
            'conveyor_in_position': False,
            'conveyor_out_position': False
        }
        # 添加状态变化计数器（避免频繁打印）
        self.state_change_counter = 0

        # 控制标志
        self.warehouse_process_requested = False
        self.warehouse_process_stop_requested = False
        self.outbound_process_requested = False
        self.outbound_process_stop_requested = False
        
        # 自动模式状态
        self.auto_mode_enabled = False
        self.enabled = True
        self.initialized = False
        
        # 延迟控制
        self.delay_started = False
        self.delay_counter = 0
        self.delay_condition_triggered = False
        self.conveyor_in_detected = False
        
        # 出库延迟控制
        self.outbound_delay_started = False
        self.outbound_delay_counter = 0
        self.outbound_delay_condition_triggered = False
        self.outbound_conveyor_in_detected = False
        
        # 新增：条件二延迟控制
        self.conveyor_in_then_out_delay_started = False
        self.conveyor_in_then_out_delay_counter = 0
        # 新增：条件二延迟控制变量 - 出库流程
        self.outbound_conveyor_in_then_out_delay_started = False
        self.outbound_conveyor_in_then_out_delay_counter = 0
        # 新增：层指令状态跟踪，避免重复发送
        self.last_layer_command = None  # 存储最后发送的层指令
        self.layer_command_sent = False  # 标记层指令是否已发送

        # 常量定义
        self.DELAY_BEFORE_STOP_MS = 10000
        self.DELAY_COUNTER_MAX = self.DELAY_BEFORE_STOP_MS // 100
        self.OUTBOUND_DELAY_BEFORE_STOP_MS = 500
        self.OUTBOUND_DELAY_COUNTER_MAX = self.OUTBOUND_DELAY_BEFORE_STOP_MS // 100
        
        # 待处理命令队列
        self.pending_commands = []
        
        # 当前IO信号状态
        self.current_io_signals = {
            'buffer_in_position': False,
            'buffer_out_position': False,
            'conveyor_in_position': False,
            'conveyor_out_position': False
        }
        
        # 新增：DO控制状态跟踪，避免重复发送
        self.last_do_commands = {}  # 存储每个DO地址的最后状态
        self.do_command_sent = {}   # 标记DO命令是否已发送
        
        # 创建发布器
        self.control_pub = self.create_publisher(String, '/control_command', 10)
        self.jog_pub = self.create_publisher(String, '/jog_command', 10)
        self.displacement_pub = self.create_publisher(String, '/displacement_command', 10)
        self.do_control_pub = self.create_publisher(String, '/do_control', 10)
        self.layer_pub = self.create_publisher(UInt8, '/layer_command', 10)
        
        # 创建订阅器
        self.io_status_sub = self.create_subscription(
            String, 
            '/py_io_status', 
            self.io_status_callback, 
            10
        )
        
        self.system_status_sub = self.create_subscription(
            String,
            '/system_status',
            self.system_status_callback,
            10
        )
        
        self.warehouse_start_sub = self.create_subscription(
            UInt8,
            '/warehouse_start', 
            self.warehouse_start_callback,
            10
        )
        
        self.warehouse_stop_sub = self.create_subscription(
            Empty,
            '/warehouse_stop',
            self.warehouse_stop_callback,
            10
        )
        
        self.outbound_start_sub = self.create_subscription(
            UInt8,
            '/outbound_start',
            self.outbound_start_callback,
            10
        )
        
        self.outbound_stop_sub = self.create_subscription(
            Empty,
            '/outbound_stop',
            self.outbound_stop_callback,
            10
        )
        
        # 修改：添加层移动状态跟踪
        self.layer_motion_completed = False
        self.previous_layer_completion_state = False  # 添加上一个状态记录
        self.layer_completion_received_time = None    # 添加接收时间记录
        self.layer_completion_timeout = 30.0          # 30秒超时
        
        # 创建层移动完成订阅器
        self.layer_completion_sub = self.create_subscription(
            Bool,
            '/layer_motion_completed',
            self.layer_completion_callback,
            10
        )
        
        # 定时器 - 处理业务逻辑
        self.timer = self.create_timer(0.1, self.process_logic)  # 100ms周期
        
        self.get_logger().info('Python业务逻辑处理器已启动 - DO控制优化版')

    def system_status_callback(self, msg):
        """处理系统状态消息，检测自动模式"""
        status_text = msg.data
        
        # 检测自动模式启动命令
        if '执行命令: start_auto' in status_text:
            if not self.auto_mode_enabled:
                self.auto_mode_enabled = True
                self.get_logger().info('检测到自动模式启动命令，业务逻辑处理器进入自动模式')
        
        # 检测停止命令
        elif '执行命令: stop' in status_text:
            if self.auto_mode_enabled:
                self.auto_mode_enabled = False
                self.reset_business_logic()
                self.get_logger().info('检测到停止命令，业务逻辑处理器退出自动模式')

    def io_status_callback(self, msg):
        """处理IO状态更新"""
        try:
            io_data = self.parse_io_status(msg.data)
            self.current_io_signals.update(io_data)
            self.process_io_signals()
        except Exception as e:
            self.get_logger().error(f'IO状态解析错误: {e}')

    def parse_io_status(self, io_data: str) -> Dict[str, bool]:
        """解析IO状态字符串为字典"""
        io_signals = {}
        try:
            # 使用正则表达式匹配所有DI信号
            pattern = r'DI(\d+):(\d)'
            matches = re.findall(pattern, io_data)
            
            for di_num, value in matches:
                di_num_int = int(di_num)
                # 映射到标准信号名称
                signal_name = self.map_signal_name(di_num_int)
                if signal_name:
                    io_signals[signal_name] = (value == '1')
                    
        except Exception as e:
            self.get_logger().error(f'IO状态解析失败: {e}')
        
        return io_signals

    def map_signal_name(self, di_number: int) -> str:
        """映射DI编号到标准信号名称"""
        mapping = {
            12: 'buffer_in_position',
            13: 'buffer_out_position', 
            14: 'conveyor_in_position',
            15: 'conveyor_out_position'
        }
        return mapping.get(di_number, '')

    def warehouse_start_callback(self, msg):
        """处理入库启动命令"""
        if not self.auto_mode_enabled:
            self.get_logger().warn('自动模式未启用，忽略入库启动命令')
            return
            
        if self.warehouse_state != WarehouseState.IDLE:
            self.get_logger().warn('入库流程已在运行中，无法重复启动')
            return
        
        self.target_layer = msg.data
        self.warehouse_process_requested = True
        self.warehouse_process_stop_requested = False
        self.get_logger().info(f'收到入库流程启动请求，目标层: {self.target_layer}')

    def warehouse_stop_callback(self, msg):
        """处理入库停止命令"""
        self.warehouse_process_stop_requested = True
        self.get_logger().info('收到入库流程停止请求')

    def outbound_start_callback(self, msg):
        """处理出库启动命令"""
        if not self.auto_mode_enabled:
            self.get_logger().warn('自动模式未启用，忽略出库启动命令')
            return
            
        if self.outbound_state != OutboundState.IDLE:
            self.get_logger().warn('出库流程已在运行中，无法重复启动')
            return
        
        self.source_layer = msg.data
        self.outbound_process_requested = True
        self.outbound_process_stop_requested = False
        self.get_logger().info(f'收到出库流程启动请求，源层: {self.source_layer}')

    def outbound_stop_callback(self, msg):
        """处理出库停止命令"""
        self.outbound_process_stop_requested = True
        self.get_logger().info('收到出库流程停止请求')

    def layer_completion_callback(self, msg):
        """层移动完成回调处理 - 修复版本"""
        try:
            current_time = self.get_clock().now().nanoseconds / 1e9  # 转换为秒
            
            # 记录接收时间
            self.layer_completion_received_time = current_time
            
            # 检查状态是否从false变为true（表示移动完成）
            current_state = msg.data
            was_false_now_true = (not self.previous_layer_completion_state and current_state)
            
            if was_false_now_true:
                # 状态从false变为true，表示层移动完成
                self.layer_motion_completed = True
                self.get_logger().info('检测到层移动完成：false -> true')
                    
            elif current_state:
                # 如果已经是true状态，可能是重复消息，记录但不处理
                self.get_logger().debug('重复的层移动完成信号（已经是true状态）')
            else:
                # false状态，表示移动开始或进行中
                self.get_logger().debug('层移动进行中或刚开始')
            
            # 更新前一个状态
            self.previous_layer_completion_state = current_state
            
        except Exception as e:
            self.get_logger().error(f'层移动完成回调处理错误: {e}')

    def process_logic(self):
        """主处理逻辑 - 定时器回调"""
        if not self.auto_mode_enabled or not self.enabled:
            return
            
        # 处理入库逻辑
        self.process_warehouse_logic()
        
        # 处理出库逻辑
        self.process_outbound_logic()
        
        # 处理IO信号变化
        self.process_io_signals()

        # 执行待处理命令
        self.execute_pending_commands()

    def process_io_signals(self):
        """处理IO信号变化 - 只在变化时打印"""
        if not self.auto_mode_enabled or not self.enabled:
            return
        
        # 检查IO信号是否有变化
        io_changed = False
        for signal_name, current_value in self.current_io_signals.items():
            if self.previous_io_signals.get(signal_name, None) != current_value:
                io_changed = True
                break

        # 只在变化时打印
        if io_changed:
                self.get_logger().info(
                    f'IO信号状态变化: buffer_in={self.current_io_signals["buffer_in_position"]}, '
                    f'buffer_out={self.current_io_signals["buffer_out_position"]}, '
                    f'conveyor_in={self.current_io_signals["conveyor_in_position"]}, '
                    f'conveyor_out={self.current_io_signals["conveyor_out_position"]}'
                )
        
        # 更新前一个状态
        self.previous_io_signals = self.current_io_signals.copy()

    def process_warehouse_logic(self):
        """处理入库业务流程"""
        di = self.current_io_signals
        buffer_in = di['buffer_in_position']
        buffer_out = di['buffer_out_position']
        conveyor_in = di['conveyor_in_position']
        conveyor_out = di['conveyor_out_position']

        # 检查状态是否变化
        state_changed = (self.warehouse_state != self.previous_warehouse_state)
        
        # 只在状态变化时打印详细信息
        if state_changed:
            self.get_logger().info(
                f'入库流程状态变化: {self.previous_warehouse_state.name} -> {self.warehouse_state.name}, '
                f'当前层: {self.current_layer}, 目标层: {self.target_layer}'
            )
            self.previous_warehouse_state = self.warehouse_state
        
        # 处理停止请求
        if self.warehouse_process_stop_requested:
            self.warehouse_state = WarehouseState.IDLE
            self.warehouse_process_stop_requested = False
            self.warehouse_process_requested = False
            self.get_logger().info('入库流程已停止')
            return

        if self.warehouse_state == WarehouseState.IDLE:
            # 等待启动信号
            if (self.warehouse_process_requested and 
                not buffer_in and not buffer_out and 
                not conveyor_in and not conveyor_out):
                self.warehouse_state = WarehouseState.WAIT_FOR_ENTRY
                self.warehouse_process_requested = False
                self.get_logger().info(f'入库流程启动，进入等待入库状态，目标层: {self.target_layer}')

        elif self.warehouse_state == WarehouseState.WAIT_FOR_ENTRY:
            # 检测入库条件
            if buffer_in and not buffer_out:
                self.warehouse_state = WarehouseState.CONVEYOR_MOVING
                self.get_logger().info('检测到入库条件，开始输送')
                
                # 生成输送带启动命令
                self.add_command(ControlAction(
                    CommandType.JOG, "axis1_1", "reverse", 
                    description="启动轴1_1正转"
                ))
                self.add_command(ControlAction(
                    CommandType.JOG, "axis1_2", "forward", 
                    description="启动轴1_2正转"
                ))
                
                # 发送层指令到第1层等待
                self.send_layer_command(1)

        elif self.warehouse_state == WarehouseState.CONVEYOR_MOVING:
            # 检测出料到位
            if buffer_out:
                # 使用优化后的DO控制函数，确保只发送一次
                self.send_do_control_once("813", True)  # 激活DO14皮带反转
                # self.get_logger().info('检测到运行至缓存架出料和接驳台入料位，等待提升机运行')
            
            # 修复：先更新conveyor_in检测状态，再检查条件
            # 实时跟踪conveyor_in信号变化
            if conveyor_in and not self.conveyor_in_detected:
                # conveyor_in从False变为True
                self.conveyor_in_detected = True
            
            # 修改：扩展板子到位检测条件
            # 条件1：conveyor_out为True（直接检测到出料）
            # 条件2：conveyor_in曾经为True后又变为False（检测到货物进入后离开）
            conveyor_out_detected = conveyor_out
            conveyor_in_then_out = (self.conveyor_in_detected and not conveyor_in)
            
            # 新增：条件二触发时的延迟处理
            if conveyor_in_then_out and not self.conveyor_in_then_out_delay_started:
                self.conveyor_in_then_out_delay_started = True
                self.conveyor_in_then_out_delay_counter = 0
                self.get_logger().info('检测到条件二（conveyor_in变化），开始1秒延迟')
            
            # 处理条件二的延迟
            if self.conveyor_in_then_out_delay_started:
                self.conveyor_in_then_out_delay_counter += 1
                
                # 1秒延迟（10个周期，每周期100ms）
                if self.conveyor_in_then_out_delay_counter >= 10:
                    board_in_position = True
                    self.conveyor_in_then_out_delay_started = False
                    self.get_logger().info('条件二延迟结束，认为板子到位')
                else:
                    board_in_position = False
            else:
                # 条件一立即触发
                board_in_position = conveyor_out_detected
            
            # 记录检测状态用于调试
            if conveyor_in_then_out and not conveyor_out_detected:
                self.get_logger().info(f'检测到conveyor_in变化条件: conveyor_in_detected={self.conveyor_in_detected}, conveyor_in={conveyor_in}')

            # 继续输送直到检测到板子到位
            if board_in_position:
                self.get_logger().info(f'检测到板子到位: conveyor_out={conveyor_out}, 进料变化={conveyor_in_then_out}')
                
                # 停止输送带
                self.add_command(ControlAction(
                    CommandType.JOG, "axis1_1", "stop",
                    description="停止轴1_1"
                ))
                self.add_command(ControlAction(
                    CommandType.JOG, "axis1_2", "stop",
                    description="停止轴1_2"
                ))
                self.send_do_control_once("813", False)  # 停止DO14皮带反转
                self.conveyor_in_detected = False  # 重置conveyor_in检测标志
                self.conveyor_in_then_out_delay_started = False  # 重置延迟标志

                # 发送目标层层指令
                self.send_layer_command(self.target_layer)
                self.warehouse_state = WarehouseState.LIFT_MOVING
                # 重置层移动相关状态
                self.layer_motion_completed = False
                self.previous_layer_completion_state = False
                self.layer_completion_received_time = None

        elif self.warehouse_state == WarehouseState.LIFT_MOVING:
            # 等待层移动完成
            if not self.layer_motion_completed:
                # 每5秒打印一次等待状态（避免频繁打印）
                self.state_change_counter += 1
                if self.state_change_counter >= 50:  # 5秒打印一次
                    self.state_change_counter = 0
                    self.get_logger().info('等待层移动完成...')
                return  # 继续等待
            
            # 层移动完成后执行后续操作
            self.get_logger().info('层移动完成，继续执行入库流程')
            # 重置层移动相关状态
            self.layer_motion_completed = False
            self.previous_layer_completion_state = False
            self.layer_completion_received_time = None
            
            # 重要修改：立即转换到新的状态，避免重新进入等待
            self.warehouse_state = WarehouseState.POST_LIFT_PROCESSING
            self.get_logger().info('进入层移动后处理状态')

        elif self.warehouse_state == WarehouseState.POST_LIFT_PROCESSING:
            """新增：层移动后的处理状态，避免状态循环"""
            # 执行层移动完成后的操作
            self.get_logger().info('入库流程：层移动完成，继续执行后续操作')
            
            # 继续执行入库流程的后续步骤
            self.send_do_control_once("812", True)  # 启动DO13皮带正转
            
            # 启动轴2反转
            self.add_command(ControlAction(
                CommandType.JOG, "axis2_1", "reverse",
                description="启动轴2_1反转"
            ))
            self.add_command(ControlAction(
                CommandType.JOG, "axis2_2", "forward",
                description="启动轴2_2正转"
            ))
            self.send_do_control_once("811", True)  # 激活DO气缸伸出信号

            # 立即转换到下一个状态
            self.warehouse_state = WarehouseState.DELAY_PROCESSING
            self.get_logger().info('进入延迟处理状态')

        elif self.warehouse_state == WarehouseState.DELAY_PROCESSING:
                # 检测信号变化
                if not self.delay_started and not self.delay_condition_triggered:
                    if conveyor_in:
                        if not self.conveyor_in_detected:
                            self.conveyor_in_detected = True
                            self.get_logger().info('检测到货物进入接驳台(conveyor_in=1)')
                    elif self.conveyor_in_detected:
                        self.delay_condition_triggered = True
                        self.delay_started = True
                        self.delay_counter = 0
                        self.conveyor_in_detected = False
                        self.get_logger().info(f'检测到货物离开接驳台(conveyor_in=0)，开始{self.DELAY_BEFORE_STOP_MS//1000}秒延迟')
                
                # 处理延迟逻辑
                if self.delay_started:
                    self.delay_counter += 1
                    
                    if self.delay_counter >= self.DELAY_COUNTER_MAX:
                        # 延迟结束
                        self.warehouse_state = WarehouseState.COMPLETED
                        self.delay_started = False
                        self.delay_condition_triggered = False
                        
                        # 停止轴2
                        self.add_command(ControlAction(
                            CommandType.JOG, "axis2_1", "stop",
                            description="轴2_1停止"
                        ))
                        self.add_command(ControlAction(
                            CommandType.JOG, "axis2_2", "stop",
                            description="轴2_2停止"
                        ))
                        self.send_do_control_once("811", False)
                        self.send_do_control_once("812", False)
                        self.get_logger().info('延迟结束，停止轴2并进入完成状态')
                    else:
                        # 延迟中，每5秒记录一次
                        if self.delay_counter % 50 == 0:
                            remaining_seconds = self.DELAY_BEFORE_STOP_MS//1000 - self.delay_counter//10
                            self.get_logger().info(f'延迟剩余时间: {remaining_seconds}秒')

        elif self.warehouse_state == WarehouseState.COMPLETED:
            # 回到第1层
            self.send_layer_command(1)
            
            if (not buffer_in and not buffer_out and 
                not conveyor_in and not conveyor_out):
                self.warehouse_state = WarehouseState.IDLE
                self.get_logger().info('回到初始状态，等待下一次入库')

    def process_outbound_logic(self):
        """处理出库业务流程"""
        di = self.current_io_signals
        buffer_in = di['buffer_in_position']
        buffer_out = di['buffer_out_position']
        conveyor_in = di['conveyor_in_position']
        conveyor_out = di['conveyor_out_position']
        
        # 检查状态是否变化
        state_changed = (self.outbound_state != self.previous_outbound_state)
        
        # 只在状态变化时打印
        if state_changed:
            self.get_logger().info(
                f'出库流程状态变化: {self.previous_outbound_state.name} -> {self.outbound_state.name}, '
                f'源层: {self.source_layer}'
            )
            self.previous_outbound_state = self.outbound_state
        
        # 处理停止请求
        if self.outbound_process_stop_requested:
            self.outbound_state = OutboundState.IDLE
            self.outbound_process_stop_requested = False
            self.outbound_process_requested = False
            self.get_logger().info('出库流程已停止')
            return

        if self.outbound_state == OutboundState.IDLE:

            # 等待启动信号
            if (self.outbound_process_requested and 
                self.check_outbound_condition()):
                self.outbound_state = OutboundState.WAIT_FOR_EXIT
                self.outbound_process_requested = False
                self.get_logger().info(f'出库流程启动，进入等待出库状态，源层: {self.source_layer}')

        elif self.outbound_state == OutboundState.WAIT_FOR_EXIT:
            self.get_logger().info('检测到出库条件，开始提升机运行')
            
            # 发送源层层指令
            self.send_layer_command(self.source_layer)
            self.outbound_state = OutboundState.LIFT_MOVING
            # 重置层移动相关状态
            self.layer_motion_completed = False
            self.previous_layer_completion_state = False
            self.layer_completion_received_time = None

        elif self.outbound_state == OutboundState.LIFT_MOVING:
            # 等待层移动完成
            if not self.layer_motion_completed:
                # 每5秒打印一次等待状态
                self.state_change_counter += 1
                if self.state_change_counter >= 50:  # 5秒打印一次
                    self.state_change_counter = 0
                self.get_logger().info('等待层移动完成...')
                return  # 继续等待
            
            # 层移动完成后执行后续操作
            self.get_logger().info('层移动完成，继续执行出库流程')
            # 重置层移动相关状态
            self.layer_motion_completed = False
            self.previous_layer_completion_state = False
            self.layer_completion_received_time = None
            
            self.send_do_control_once("813", True)  # 激活DO14皮带反转
            
            # 启动轴2正转
            self.add_command(ControlAction(
                CommandType.JOG, "axis2_1", "forward",
                description="启动轴2_1正转（出库）"
            ))
            self.add_command(ControlAction(
                CommandType.JOG, "axis2_2", "reverse", 
                description="启动轴2_2反转（出库）"
            ))
            self.send_do_control_once("811", True)  # 激活DO信号

            # 立即转换到新的状态
            self.outbound_state = OutboundState.POST_LIFT_PROCESSING
            self.get_logger().info('出库流程：层移动完成，继续执行后续操作')

        elif self.outbound_state == OutboundState.POST_LIFT_PROCESSING:
            """出库流程的层移动后处理状态"""         
            # 修复：先更新outbound_conveyor_in_detected状态
            if conveyor_in and not self.outbound_conveyor_in_detected:
                # conveyor_in从False变为True
                self.outbound_conveyor_in_detected = True
                self.get_logger().info('检测到货物进入接驳台(conveyor_in=1)')
            
            # 修改：扩展板子到位检测条件
            # 条件1：conveyor_out为True（直接检测到出料）
            # 条件2：conveyor_in曾经为True后又变为False（检测到货物进入后离开）
            conveyor_out_detected = conveyor_out
            conveyor_in_then_out = (self.outbound_conveyor_in_detected and not conveyor_in)
            
            # 新增：条件二触发时的延迟处理
            if conveyor_in_then_out and not self.outbound_conveyor_in_then_out_delay_started:
                self.outbound_conveyor_in_then_out_delay_started = True
                self.outbound_conveyor_in_then_out_delay_counter = 0
                self.get_logger().info('检测到出库条件二（conveyor_in变化），开始1秒延迟')
            
            # 处理条件二的延迟
            if self.outbound_conveyor_in_then_out_delay_started:
                self.outbound_conveyor_in_then_out_delay_counter += 1
                
                # 0.5秒延迟（5个周期，每周期100ms）
                if self.outbound_conveyor_in_then_out_delay_counter >= 5:
                    board_in_position = True
                    self.outbound_conveyor_in_then_out_delay_started = False
                    self.get_logger().info('出库条件二延迟结束，认为板子到位')
                else:
                    board_in_position = False
            else:
                # 条件一立即触发
                board_in_position = conveyor_out_detected
            
            # 记录检测状态用于调试
            if conveyor_in_then_out and not conveyor_out_detected:
                self.get_logger().info(f'检测到出库conveyor_in变化条件: outbound_conveyor_in_detected={self.outbound_conveyor_in_detected}, conveyor_in={conveyor_in}')

            if board_in_position: # 检测板子到位条件
                self.get_logger().info(f'检测到板子到位: conveyor_out={conveyor_out}, 进料变化={conveyor_in_then_out}')
                
                # 停止轴2
                self.add_command(ControlAction(
                    CommandType.JOG, "axis2_1", "stop",
                    description="停止轴2_1"
                ))
                self.add_command(ControlAction(
                    CommandType.JOG, "axis2_2", "stop",
                    description="停止轴2_2"
                ))
                self.send_do_control_once("811", False)  # 停止DO信号
                self.send_do_control_once("813", False)  # 停止DO14皮带反转
                self.outbound_conveyor_in_detected = False  # 重置检测标志
                self.outbound_conveyor_in_then_out_delay_started = False  # 重置延迟标志
                
                self.outbound_state = OutboundState.CONVEYOR_MOVING
                self.get_logger().info('进入输送带运行状态')

        elif self.outbound_state == OutboundState.CONVEYOR_MOVING:
            """输送带运行状态 - 优化版本"""
            # 1. 首先发送层指令1，等待提升机到达
            self.send_layer_command(1)
            
            # 2. 等待提升机到达第1层
            if not self.layer_motion_completed:
                # 每5秒打印一次等待状态
                self.state_change_counter += 1
                if self.state_change_counter >= 50:  # 5秒打印一次
                    self.state_change_counter = 0
                    self.get_logger().info('等待提升机到达第1层...')
                return  # 继续等待，不执行后续逻辑
            
            # 3. 提升机到达第1层后，启动输送带
            if not hasattr(self, 'outbound_conveyor_started') or not self.outbound_conveyor_started:
                self.send_do_control_once("813", True)  # 启动DO14皮带正转
                self.outbound_conveyor_started = True
                self.get_logger().info('提升机已到达第1层，启动输送带')
                # 重置层移动完成标志，为下一次使用做准备
                self.layer_motion_completed = False
                self.outbound_state = OutboundState.COMPLETED

        elif self.outbound_state == OutboundState.COMPLETED:
            # 4. 检测货物完全送出
            if conveyor_out and not self.outbound_delay_started:
                self.outbound_delay_started = True
                self.outbound_delay_counter = 0
                self.get_logger().info(f'检测到货物到达出料位，开始{self.OUTBOUND_DELAY_BEFORE_STOP_MS//1000}秒延迟')
            
            # 5. 延迟处理
            if self.outbound_delay_started:
                self.outbound_delay_counter += 1
                
                if self.outbound_delay_counter >= self.OUTBOUND_DELAY_COUNTER_MAX:
                    # 停止输送带并完成流程
                    self.send_do_control_once("813", False)
                    self.outbound_delay_started = False
                    self.outbound_state = OutboundState.IDLE
                    # 重置所有相关标志
                    if hasattr(self, 'outbound_conveyor_started'):
                        self.outbound_conveyor_started = False
                    self.get_logger().info('出库流程完成')

    def check_outbound_condition(self) -> bool:
        """检查出库启动条件"""
        di = self.current_io_signals
        return (not di['buffer_in_position'] and not di['buffer_out_position'] and 
                not di['conveyor_in_position'] and not di['conveyor_out_position'])

    def check_outbound_completion_condition(self) -> bool:
        """检查出库完成条件"""
        di = self.current_io_signals
        return (not di['buffer_in_position'] and not di['buffer_out_position'] and 
                not di['conveyor_in_position'] and not di['conveyor_out_position'])

    def add_command(self, action: ControlAction):
        """添加控制命令到待处理队列"""
        self.pending_commands.append(action)
        self.get_logger().info(f'生成控制命令: 类型={action.type.name}, 轴={action.axis_name}, '
                               f'值={action.command_value}, 描述={action.description}')

    def execute_pending_commands(self):
        """执行待处理命令"""
        for command in self.pending_commands:
            if command.type == CommandType.JOG:
                self.send_jog_command(f"{command.axis_name}:{command.command_value}")
            elif command.type == CommandType.LAYER:
                # 层指令已经通过send_layer_command发送，这里不需要重复发送
                pass
        
        # 清空已执行的命令
        self.pending_commands.clear()

    def send_jog_command(self, command: str):
        """发送点动命令"""
        msg = String()
        msg.data = command
        self.jog_pub.publish(msg)
        self.get_logger().info(f'已发送点动命令: {command}')

    def send_layer_command(self, layer: int):
        """发送层指令 - 优化版本，确保只发送一次"""
        # 检查是否已经发送过相同的层指令
        if self.last_layer_command == layer and self.layer_command_sent:
            self.get_logger().debug(f'层指令已发送过，跳过: 第{layer}层')
            return
        
        # 发送新命令
        msg = UInt8()
        msg.data = layer
        self.layer_pub.publish(msg)
        
        # 记录发送状态
        self.last_layer_command = layer
        self.layer_command_sent = True
        
        self.get_logger().info(f'已发送层指令: 第{layer}层')

    def send_do_control_once(self, do_address: str, state: bool):
        """发送DO控制命令 - 优化版本，确保只发送一次"""
        command_str = f'{do_address}:{1 if state else 0}'
        
        # 检查是否已经发送过相同的命令
        if do_address in self.last_do_commands and self.last_do_commands[do_address] == state:
            if do_address not in self.do_command_sent or not self.do_command_sent[do_address]:
                # 命令相同但未标记为已发送，可能是状态重置后的第一次发送
                self.do_command_sent[do_address] = True
            else:
                # 已经发送过相同的命令，跳过
                self.get_logger().debug(f'DO控制命令已发送过，跳过: {command_str}')
                return
        
        # 发送新命令
        msg = String()
        msg.data = command_str
        self.do_control_pub.publish(msg)
        
        # 记录发送状态
        self.last_do_commands[do_address] = state
        self.do_command_sent[do_address] = True
        
        self.get_logger().info(f'已发送DO控制: {msg.data}')

    def reset_do_command_state(self, do_address: str = None):
        """重置DO命令发送状态，允许重新发送"""
        if do_address is None:
            # 重置所有DO命令状态
            for addr in self.do_command_sent:
                self.do_command_sent[addr] = False
            self.get_logger().info('重置所有DO命令发送状态')
        elif do_address in self.do_command_sent:
            # 重置特定DO命令状态
            self.do_command_sent[do_address] = False
            self.get_logger().info(f'重置DO命令发送状态: {do_address}')

    def reset_business_logic(self):
        """重置业务逻辑状态"""
        # 重置入库流程状态
        self.warehouse_state = WarehouseState.IDLE
        self.warehouse_process_requested = False
        self.warehouse_process_stop_requested = False
        
        # 重置出库流程状态
        self.outbound_state = OutboundState.IDLE
        self.outbound_process_requested = False
        self.outbound_process_stop_requested = False
        
        # 重置延迟计数器
        self.delay_started = False
        self.delay_condition_triggered = False
        self.delay_counter = 0
        
        # 重置信号检测状态
        self.conveyor_in_detected = False
        self.outbound_conveyor_in_detected = False
        self.outbound_delay_started = False
        self.outbound_delay_condition_triggered = False
        self.outbound_delay_counter = 0
        
        # 新增：重置条件二延迟控制变量
        self.conveyor_in_then_out_delay_started = False
        self.conveyor_in_then_out_delay_counter = 0
        self.outbound_conveyor_in_then_out_delay_started = False
        self.outbound_conveyor_in_then_out_delay_counter = 0

        # 重置层指令发送状态，允许重新发送
        self.layer_command_sent = False
        self.last_layer_command = None

        # 重置DO命令发送状态，允许重新发送
        self.reset_do_command_state()
        
        # 清空待处理命令
        self.pending_commands.clear()
        
        self.get_logger().info('业务逻辑处理器状态已重置')

def main(args=None):
    rclpy.init(args=args)
    
    processor = BusinessLogicProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        processor.get_logger().info('业务逻辑处理器被用户中断')
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()