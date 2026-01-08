#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, UInt8, Empty
import threading
import select
import sys
import tty
import termios
import time

class ControlCommander(Node):
    def __init__(self):
        # 1. 降低ROS2日志级别，减少控制台输出
        import rclpy.logging
        rclpy.logging.set_logger_level('control_commander', rclpy.logging.LoggingSeverity.WARN)
        
        super().__init__('control_commander')
        
        # 创建发布器
        self.control_pub = self.create_publisher(String, '/control_command', 10)
        self.displacement_pub = self.create_publisher(String, '/displacement_command', 10)  # 改为String类型
        self.jog_pub = self.create_publisher(String, '/jog_command', 10)
        # 新增IO控制发布器
        self.io_control_pub = self.create_publisher(String, '/do_control', 10)
        # 新增仓库控制发布器
        self.warehouse_start_pub = self.create_publisher(UInt8, '/warehouse_start', 10)
        self.warehouse_stop_pub = self.create_publisher(Empty, '/warehouse_stop', 10)
        # 新增出库控制发布器
        self.outbound_start_pub = self.create_publisher(UInt8, '/outbound_start', 10)
        self.outbound_stop_pub = self.create_publisher(Empty, '/outbound_stop', 10)
        
        # 设置非阻塞输入
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        # 菜单状态变量
        self.current_menu = "main"  # 当前菜单状态：main, manual_mode, auto_mode
        self.manual_mode_active = False
        self.auto_mode_active = False
        
        # 位移控制参数
        self.axis4_position = 0.0
        self.axis5_position = 0.0
        self.position_step = 10.0  # 默认步长10mm
        
        # 按键缓冲区
        self.key_buffer = ""
        
        # 简化输出标志
        self.simplified_output = True
        
        # IO状态跟踪
        self.io_status = {
            '800': 0,  # 启动按钮灯
            '801': 0,  # 复位按钮灯
            '802': 0,  # 暂停按钮灯
            '803': 0,  # 蜂鸣器
            '804': 0,  # 三色红灯
            '805': 0,  # 三色黄灯
            '806': 0,  # 三色绿灯
            '810': 0,  # 顶升气缸下降
            '811': 0,  # 齿轮对接气缸伸出
            '812': 0,  # 皮带正转启动
            '813': 0,  # 皮带反转启动
        }
        
        self.get_logger().info('控制命令节点已启动')
        self.print_main_menu()
    
    def print_main_menu(self):
        """打印主菜单"""
        print("\n" + "="*40)
        print("   ROS2 机械臂控制主菜单   ")
        print("="*40)
        print("01: 进入手动模式")
        print("02: 进入自动模式") 
        print("03: 停止所有轴")
        print("04: 清除故障")
        print("05: 重置轴")
        print("06: 发送轨迹动作")
        print("q: 退出程序")
        print("="*40)
        print("请输入两位数字或字母选择命令:")
    
    def print_manual_menu(self):
        """打印手动控制菜单"""
        print("\n" + "="*40)
        print("手动模式 - 轴控制与IO控制菜单")
        print("="*40)
        print("轴控制:")
        print("  01: 轴1正转点动")
        print("  02: 轴1反转点动") 
        print("  03: 轴2正转点动")
        print("  04: 轴2反转点动")
        print("  05: 轴3正转点动")
        print("  06: 轴3反转点动")
        print("  07: 轴4正转点动")
        print("  08: 轴4反转点动")
        print("  09: 轴5正转点动")
        print("  10: 轴5反转点动")
        print("IO控制 (当前状态):")
        print("  11: M800启动按钮灯 [{}]".format("ON" if self.io_status['800'] else "OFF"))
        print("  12: M801复位按钮灯 [{}]".format("ON" if self.io_status['801'] else "OFF"))
        print("  13: M802暂停按钮灯 [{}]".format("ON" if self.io_status['802'] else "OFF"))
        print("  14: M803蜂鸣器 [{}]".format("ON" if self.io_status['803'] else "OFF"))
        print("  15: M804三色红灯 [{}]".format("ON" if self.io_status['804'] else "OFF"))
        print("  16: M805三色黄灯 [{}]".format("ON" if self.io_status['805'] else "OFF"))
        print("  17: M806三色绿灯 [{}]".format("ON" if self.io_status['806'] else "OFF"))
        print("  18: M810顶升气缸下降 [{}]".format("ON" if self.io_status['810'] else "OFF"))
        print("  19: M811齿轮对接气缸伸出 [{}]".format("ON" if self.io_status['811'] else "OFF"))
        print("  20: M812皮带正转启动 [{}]".format("ON" if self.io_status['812'] else "OFF"))
        print("  21: M813皮带反转启动 [{}]".format("ON" if self.io_status['813'] else "OFF"))
        print("其他控制:")
        print("  s: 停止所有轴点动")
        print("  r: 复位所有IO")
        print("  b: 返回主菜单")
        print("="*40)
        print("请输入命令:")
    
    def print_auto_menu(self):
        """打印自动控制菜单"""
        print("\n" + "="*40)
        print("自动模式 - 位移控制与IO控制菜单")
        print("="*40)
        print("点动控制:")
        print("  01: 轴1_1正转点动")
        print("  02: 轴1_2反转点动")
        print("  03: 轴2_1正转点动")
        print("  04: 轴2_2反转点动")
        print("轴4位移控制 (当前: {:.1f}mm):".format(self.axis4_position))
        print("  05: 轴4移动到原点 (0.0mm)")
        print("  06: 轴4移动到10.0mm位置")
        print("  07: 轴4正向移动10.0mm")
        print("  08: 轴4反向移动10.0mm")
        print("轴5位移控制 (当前: {:.1f}mm):".format(self.axis5_position))
        print("  09: 轴5移动到原点 (0.0mm)")
        print("  10: 轴5移动到10.0mm位置")
        print("  11: 轴5正向移动10.0mm")
        print("  12: 轴5反向移动10.0mm")
        print("IO控制 (当前状态):")
        print("  13: M800启动按钮灯 [{}]".format("ON" if self.io_status['800'] else "OFF"))
        print("  14: M801复位按钮灯 [{}]".format("ON" if self.io_status['801'] else "OFF"))
        print("  15: M802暂停按钮灯 [{}]".format("ON" if self.io_status['802'] else "OFF"))
        print("  16: M803蜂鸣器 [{}]".format("ON" if self.io_status['803'] else "OFF"))
        print("  17: M804三色红灯 [{}]".format("ON" if self.io_status['804'] else "OFF"))
        print("  18: M805三色黄灯 [{}]".format("ON" if self.io_status['805'] else "OFF"))
        print("  19: M806三色绿灯 [{}]".format("ON" if self.io_status['806'] else "OFF"))
        print("  20: M810顶升气缸下降 [{}]".format("ON" if self.io_status['810'] else "OFF"))
        print("  21: M811齿轮对接气缸伸出 [{}]".format("ON" if self.io_status['811'] else "OFF"))
        print("  22: M812皮带正转启动 [{}]".format("ON" if self.io_status['812'] else "OFF"))
        print("  23: M813皮带反转启动 [{}]".format("ON" if self.io_status['813'] else "OFF"))
        print("仓库控制:")
        print("  24: 启动入库")
        print("  25: 停止入库")
        print("  26: 启动出库")
        print("  27: 停止出库")
        print("其他控制:")
        print("  s: 停止所有轴")
        print("  r: 复位所有IO")
        print("  b: 返回主菜单")
        print("="*40)
        print("请输入命令:")
    
    def send_control_command(self, command_data):
        """发送控制命令"""
        msg = String()
        msg.data = command_data
        self.control_pub.publish(msg)
        if not self.simplified_output:
            self.get_logger().info('已发送控制命令: {}'.format(command_data))
    
    def send_displacement_command(self, axis_name, position):
        """发送位移命令 - 格式: 'axis4:0.0' 或 'axis5:10.0'"""
        msg = String()
        msg.data = '{}:{:.1f}'.format(axis_name, position)
        self.displacement_pub.publish(msg)
        if not self.simplified_output:
            self.get_logger().info('已发送位移命令: {}'.format(msg.data))
    
    def send_jog_command(self, command):
        """发送点动命令"""
        msg = String()
        msg.data = command
        self.jog_pub.publish(msg)
        if not self.simplified_output:
            self.get_logger().info('已发送点动命令: {}'.format(command))
    
    def send_io_control(self, io_address, value):
        """发送IO控制命令 - 格式: '801:1'"""
        msg = String()
        msg.data = '{}:{}'.format(io_address, value)
        self.io_control_pub.publish(msg)
        if not self.simplified_output:
            self.get_logger().info('已发送IO控制命令: {}'.format(msg.data))
        return msg.data
    
    def toggle_io(self, io_address):
        """切换IO状态并发送控制命令"""
        # 切换状态 (0->1, 1->0)
        self.io_status[io_address] = 1 - self.io_status[io_address]
        # 发送控制命令
        command = self.send_io_control(io_address, self.io_status[io_address])
        # 显示状态
        io_names = {
            '800': '启动按钮灯',
            '801': '复位按钮灯', 
            '802': '暂停按钮灯',
            '803': '蜂鸣器',
            '804': '三色红灯',
            '805': '三色黄灯',
            '806': '三色绿灯',
            '810': '顶升气缸下降',
            '811': '齿轮对接气缸伸出',
            '812': '皮带正转启动',
            '813': '皮带反转启动'
        }
        status = "开启" if self.io_status[io_address] else "关闭"
        print("{} {} ({})".format(io_names.get(io_address, 'IO'), status, command))
        # 刷新菜单显示新状态
        if self.current_menu == "manual_mode" or self.current_menu == "auto_mode":
            if self.current_menu == "manual_mode":
                self.print_manual_menu()
            else:
                self.print_auto_menu()
    
    def reset_all_io(self):
        """复位所有IO状态"""
        for io_address in self.io_status:
            if self.io_status[io_address] == 1:  # 只关闭已开启的IO
                self.io_status[io_address] = 0
                self.send_io_control(io_address, 0)
        print("所有IO已复位")
        # 刷新菜单
        if self.current_menu == "manual_mode" or self.current_menu == "auto_mode":
            if self.current_menu == "manual_mode":
                self.print_manual_menu()
            else:
                self.print_auto_menu()
    
    def send_warehouse_start(self):
        """发送入库启动命令"""
        msg = UInt8()
        msg.data = 9
        self.warehouse_start_pub.publish(msg)
        print("启动入库")
    
    def send_warehouse_stop(self):
        """发送入库停止命令"""
        msg = Empty()
        self.warehouse_stop_pub.publish(msg)
        print("停止入库")
    
    def send_outbound_start(self):
        """发送出库启动命令"""
        msg = UInt8()
        msg.data = 9
        self.outbound_start_pub.publish(msg)
        print("启动出库")
    
    def send_outbound_stop(self):
        """发送出库停止命令"""
        msg = Empty()
        self.outbound_stop_pub.publish(msg)
        print("停止出库")
    
    def stop_all_jog(self):
        """停止所有轴的点动 - 简化输出"""
        axes_to_stop = ['axis1', 'axis2', 'axis3', 'axis4', 'axis5', 'axis1_1', 'axis1_2', 'axis2_1', 'axis2_2']
        for axis in axes_to_stop:
            msg = String()
            msg.data = '{}:stop'.format(axis)
            self.jog_pub.publish(msg)
        print("停止所有轴")
    
    def stop_all_motion(self):
        """停止所有运动（包括点动和位移） - 简化输出"""
        self.stop_all_jog()
        self.send_control_command('stop')
        print("停止所有运动")
    
    def enter_manual_mode(self):
        """进入手动模式"""
        self.send_control_command('start_manual')
        time.sleep(0.1)
        self.current_menu = "manual_mode"
        self.manual_mode_active = True
        self.auto_mode_active = False
        self.print_manual_menu()
    
    def enter_auto_mode(self):
        """进入自动模式"""
        self.send_control_command('start_auto')
        time.sleep(0.1)
        self.current_menu = "auto_mode"
        self.auto_mode_active = True
        self.manual_mode_active = False
        self.print_auto_menu()
    
    def return_to_main_menu(self):
        """返回主菜单 - 不发送任何话题"""
        self.current_menu = "main"
        self.manual_mode_active = False
        self.auto_mode_active = False
        self.stop_all_jog()  # 返回主菜单时停止所有轴
        self.print_main_menu()
    
    def handle_manual_mode_command(self, key):
        """处理手动模式下的命令"""
        # 手动模式下的轴控制
        jog_commands = {
            '01': 'axis1_1:forward',
            '02': 'axis1_2:reverse', 
            '03': 'axis2_1:forward',
            '04': 'axis2_2:reverse',
            '05': 'axis3:forward',
            '06': 'axis3:reverse',
            '07': 'axis4:forward',
            '08': 'axis4:reverse',
            '09': 'axis5:forward',
            '10': 'axis5:reverse',
        }
        
        # IO控制命令映射
        io_commands = {
            '11': '800',  # M800启动按钮灯
            '12': '801',  # M801复位按钮灯
            '13': '802',  # M802暂停按钮灯
            '14': '803',  # M803蜂鸣器
            '15': '804',  # M804三色红灯
            '16': '805',  # M805三色黄灯
            '17': '806',  # M806三色绿灯
            '18': '810',  # M810顶升气缸下降
            '19': '811',  # M811齿轮对接气缸伸出
            '20': '812',  # M812皮带正转启动
            '21': '813',  # M813皮带反转启动
        }
        
        if key in jog_commands:
            self.send_jog_command(jog_commands[key])
            # 显示简化的执行信息
            axis_map = {
                'axis1_1:forward': '轴1正转',
                'axis1_2:reverse': '轴1反转',
                'axis2_1:forward': '轴2正转',
                'axis2_2:reverse': '轴2反转',
                'axis3:forward': '轴3正转',
                'axis3:reverse': '轴3反转',
                'axis4:forward': '轴4正转',
                'axis4:reverse': '轴4反转',
                'axis5:forward': '轴5正转',
                'axis5:reverse': '轴5反转',
            }
            print(axis_map.get(jog_commands[key], "执行"))
        elif key in io_commands:
            # 处理IO控制
            self.toggle_io(io_commands[key])
        elif key == 's':
            self.stop_all_jog()
        elif key == 'r':  # 新增：复位所有IO
            self.reset_all_io()
        elif key == 'b':
            self.return_to_main_menu()
        else:
            print("未知命令: {}".format(key))
            self.print_manual_menu()
    
    def handle_auto_mode_command(self, key):
        """处理自动模式下的命令"""
        # 点动控制命令
        jog_commands = {
            '01': 'axis1_1:forward',
            '02': 'axis1_2:reverse',
            '03': 'axis2_1:forward',
            '04': 'axis2_2:reverse',
        }
        
        # IO控制命令映射
        io_commands = {
            '13': '800',  # M800启动按钮灯
            '14': '801',  # M801复位按钮灯
            '15': '802',  # M802暂停按钮灯
            '16': '803',  # M803蜂鸣器
            '17': '804',  # M804三色红灯
            '18': '805',  # M805三色黄灯
            '19': '806',  # M806三色绿灯
            '20': '810',  # M810顶升气缸下降
            '21': '811',  # M811齿轮对接气缸伸出
            '22': '812',  # M812皮带正转启动
            '23': '813',  # M813皮带反转启动
        }
        
        if key in jog_commands:
            self.send_jog_command(jog_commands[key])
            # 显示简化的执行信息
            axis_map = {
                'axis1_1:forward': '轴1_1正转',
                'axis1_2:reverse': '轴1_2反转',
                'axis2_1:forward': '轴2_1正转',
                'axis2_2:reverse': '轴2_2反转',
            }
            print(axis_map.get(jog_commands[key], "执行"))
        elif key in io_commands:
            # 处理IO控制
            self.toggle_io(io_commands[key])
        elif key == 's':
            self.stop_all_motion()
        elif key == 'r':  # 复位所有IO
            self.reset_all_io()
        elif key == 'b':
            self.return_to_main_menu()
        # 轴4位移控制
        elif key == '05':  # 轴4移动到原点
            self.axis4_position = 0.0
            self.send_displacement_command('axis4', 0.0)
            print("轴4移动到原点")
            self.print_auto_menu()
        
        elif key == '06':  # 轴4移动到10mm
            self.axis4_position = 10.0
            self.send_displacement_command('axis4', 10.0)
            print("轴4移动到10mm")
            self.print_auto_menu()
        
        elif key == '07':  # 轴4正向移动10mm
            self.axis4_position += 10.0
            self.send_displacement_command('axis4', self.axis4_position)
            print("轴4正向移动10mm")
            self.print_auto_menu()
        
        elif key == '08':  # 轴4反向移动10mm
            self.axis4_position -= 10.0
            self.send_displacement_command('axis4', self.axis4_position)
            print("轴4反向移动10mm")
            self.print_auto_menu()
        
        # 轴5位移控制
        elif key == '09':  # 轴5移动到原点
            self.axis5_position = 0.0
            self.send_displacement_command('axis5', 0.0)
            print("轴5移动到原点")
            self.print_auto_menu()
        
        elif key == '10':  # 轴5移动到10mm
            self.axis5_position = 10.0
            self.send_displacement_command('axis5', 10.0)
            print("轴5移动到10mm")
            self.print_auto_menu()
        
        elif key == '11':  # 轴5正向移动10mm
            self.axis5_position += 10.0
            self.send_displacement_command('axis5', self.axis5_position)
            print("轴5正向移动10mm")
            self.print_auto_menu()
        
        elif key == '12':  # 轴5反向移动10mm
            self.axis5_position -= 10.0
            self.send_displacement_command('axis5', self.axis5_position)
            print("轴5反向移动10mm")
            self.print_auto_menu()
        
        # 仓库控制命令
        elif key == '24':  # 启动入库
            self.send_warehouse_start()
        
        elif key == '25':  # 停止入库
            self.send_warehouse_stop()
        
        elif key == '26':  # 启动出库
            self.send_outbound_start()
        
        elif key == '27':  # 停止出库
            self.send_outbound_stop()
        
        else:
            print("未知命令: {}".format(key))
            self.print_auto_menu()
    
    def handle_main_menu_command(self, key):
        """处理主菜单命令"""
        main_commands = {
            '01': lambda: self.enter_manual_mode(),
            '02': lambda: self.enter_auto_mode(),
            '03': lambda: (self.send_control_command('stop'), print("停止所有轴")),
            '04': lambda: (self.send_control_command('clear_fault'), print("清除故障")),
            '05': lambda: (self.send_control_command('reset'), print("重置轴")),
            '06': lambda: self.send_trajectory_action(),
        }
        
        if key in main_commands:
            main_commands[key]()
        elif key == 'q':
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            # 退出程序
            print("退出程序")
            sys.exit(0)
        else:
            print("未知命令: {}".format(key))
            self.print_main_menu()
    
    def send_trajectory_action(self):
        """发送轨迹动作"""
        print("轨迹动作需要单独执行action命令")
        print("请手动执行: ros2 action send_goal /arm_2zhou_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory")
    
    def get_key(self):
        """获取按键输入 - 必须输入两位数字或单字母"""
        if select.select([sys.stdin], [], [], 0.1)[0]:
            char = sys.stdin.read(1)
            
            # 如果是字母，直接返回
            if char.isalpha():
                return char
            
            # 如果是数字，必须读取第二位数字
            if char.isdigit():
                # 等待第二位数字 延迟1s内需完成输入
                if select.select([sys.stdin], [], [], 1.0)[0]:
                    char2 = sys.stdin.read(1)
                    if char2.isdigit():
                        return char + char2
                    else:
                        # 如果第二个字符不是数字，忽略输入
                        print("请输入两位数字命令")
                        return None
                else:
                    # 如果超时，忽略输入
                    print("请输入两位数字命令")
                    return None
            
            # 其他字符忽略
            return None
        
        return None
    
    def run(self):
        """主运行循环"""
        try:
            while rclpy.ok():
                key = self.get_key()
                if key:
                    # 根据当前菜单状态处理按键
                    if self.current_menu == "main":
                        self.handle_main_menu_command(key)
                    elif self.current_menu == "manual_mode":
                        self.handle_manual_mode_command(key)
                    elif self.current_menu == "auto_mode":
                        self.handle_auto_mode_command(key)
                
                time.sleep(0.1)
                
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            self.get_logger().info('控制命令节点已关闭')

def main(args=None):
    rclpy.init(args=args)
    
    commander = ControlCommander()
    
    # 在单独的线程中运行ROS2 spinning
    spin_thread = threading.Thread(target=rclpy.spin, args=(commander,), daemon=True)
    spin_thread.start()
    
    try:
        commander.run()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()