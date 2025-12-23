#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import threading
import select
import sys
import tty
import termios
import time

class ControlCommander(Node):
    def __init__(self):
        super().__init__('control_commander')
        
        # 创建发布器
        self.control_pub = self.create_publisher(String, '/control_command', 10)
        self.displacement_pub = self.create_publisher(Float64MultiArray, '/displacement_command', 10)
        self.jog_pub = self.create_publisher(String, '/jog_command', 10)
        
        # 设置非阻塞输入
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        # 菜单状态变量
        self.current_menu = "main"  # 当前菜单状态：main, manual_mode
        self.manual_mode_active = False  # 手动模式是否激活
        
        self.get_logger().info('控制命令节点已启动')
        self.print_main_menu()
    
    def print_main_menu(self):
        """打印主菜单"""
        print("\n" + "="*40)
        print("        ROS2 机械臂控制主菜单")
        print("="*40)
        print("1: 进入手动模式")
        print("2: 启动自动模式") 
        print("3: 停止所有轴")
        print("4: 清除故障")
        print("5: 重置轴")
        print("6: 发送轨迹动作")
        print("q: 退出程序")
        print("="*40)
        print("请输入数字选择命令:")
    
    def print_manual_menu(self):
        """打印手动控制菜单"""
        print("\n" + "="*40)
        print("手动模式 - 轴控制菜单")
        print("="*40)
        print("1: 轴1正转点动")
        print("2: 轴1反转点动") 
        print("3: 轴2正转点动")
        print("4: 轴2反转点动")
        print("5: 轴4正转点动")
        print("6: 轴4反转点动")
        print("7: 轴5正转点动")
        print("8: 轴5反转点动")
        print("0: 停止所有轴点动")
        print("b: 返回主菜单")
        print("="*40)
        print("请输入数字选择命令:")
    
    def send_control_command(self, command_data):
        """发送控制命令"""
        msg = String()
        msg.data = command_data
        self.control_pub.publish(msg)
        self.get_logger().info(f'已发送控制命令: {command_data}')
    
    def send_displacement_command(self, positions):
        """发送位移命令"""
        msg = Float64MultiArray()
        msg.data = positions
        self.displacement_pub.publish(msg)
        self.get_logger().info(f'已发送位移命令: {positions}')
    
    def send_jog_command(self, command):
        """发送点动命令"""
        msg = String()
        msg.data = command
        self.jog_pub.publish(msg)
        self.get_logger().info(f'已发送点动命令: {command}')
    
    def stop_all_jog(self):
        """停止所有轴的点动"""
        axes_to_stop = ['axis1', 'axis2', 'axis4', 'axis5', 'axis1_1', 'axis1_2', 'axis2_1', 'axis2_2']
        for axis in axes_to_stop:
            self.send_jog_command(f'{axis}:stop')
        self.get_logger().info('已停止所有轴点动')
    
    def enter_manual_mode(self):
        """进入手动模式 - 关键修改：发送启动指令"""
        self.send_control_command('start_manual')  # 先发送启动指令
        time.sleep(0.1)  # 短暂延迟确保指令发送
        self.current_menu = "manual_mode"
        self.manual_mode_active = True
        self.get_logger().info('已进入手动模式')
        self.print_manual_menu()
    
    def return_to_main_menu(self):
        """返回主菜单 - 不发送任何话题"""
        self.current_menu = "main"
        self.manual_mode_active = False
        self.stop_all_jog()  # 返回主菜单时停止所有轴
        self.get_logger().info('已返回主菜单')
        self.print_main_menu()
    
    def handle_manual_mode_command(self, key):
        """处理手动模式下的命令"""
        if key == 'b':
            self.return_to_main_menu()
            return
        
        # 手动模式下的轴控制
        jog_commands = {
            '1': 'axis1_1:forward',
            '2': 'axis1_2:reverse', 
            '3': 'axis2_1:forward',
            '4': 'axis2_2:reverse',
            '5': 'axis4:forward',
            '6': 'axis4:reverse',
            '7': 'axis5:forward',
            '8': 'axis5:reverse',
            '0': 'stop_all'
        }
        
        if key in jog_commands:
            if key == '0':
                self.stop_all_jog()
                self.get_logger().info('已发送停止所有轴指令')
            else:
                self.send_jog_command(jog_commands[key])
        else:
            print(f"手动模式下未知命令: {key}")
            self.print_manual_menu()
    
    def handle_main_menu_command(self, key):
        """处理主菜单命令"""
        main_commands = {
            '1': lambda: self.enter_manual_mode(),  # 进入手动模式
            '2': lambda: self.send_control_command('start_auto'),
            '3': lambda: self.send_control_command('stop'),
            '4': lambda: self.send_control_command('clear_fault'),
            '5': lambda: self.send_control_command('reset'),
            '6': lambda: self.send_trajectory_action()
        }
        
        if key in main_commands:
            main_commands[key]()
        else:
            print(f"主菜单未知命令: {key}")
            self.print_main_menu()
    
    def send_trajectory_action(self):
        """发送轨迹动作"""
        self.get_logger().info('轨迹动作需要单独执行action命令')
        print("请手动执行: ros2 action send_goal /arm_2zhou_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"trajectory: {joint_names: [joint2, joint3], points: [{positions: [0.5, 0.0], time_from_start: {sec: 4, nanosec: 0}]}\"")
    
    def get_key(self):
        """获取按键输入"""
        if select.select([sys.stdin], [], [], 0.1)[0]:
            return sys.stdin.read(1)
        return None
    
    def run(self):
        """主运行循环"""
        try:
            while rclpy.ok():
                key = self.get_key()
                if key:
                    if key == 'q':
                        self.get_logger().info('退出程序')
                        break
                    
                    # 根据当前菜单状态处理按键
                    if self.current_menu == "main":
                        self.handle_main_menu_command(key)
                    elif self.current_menu == "manual_mode":
                        self.handle_manual_mode_command(key)
                
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