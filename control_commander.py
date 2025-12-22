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
        
        self.get_logger().info('控制命令节点已启动')
        self.print_menu()
    
    def print_menu(self):
        print("\n=== ROS2 控制命令菜单 ===")
        print("1: 启动手动模式")
        print("2: 启动自动模式") 
        print("3: 停止所有轴")
        print("4: 清除故障")
        print("5: 重置轴")
        print("6: 轴1正转点动")
        print("7: 轴1反转点动")
        print("8: 轴2正转点动")
        print("9: 轴2反转点动")
        print("0: 停止点动")
        print("q: 退出程序")
        print("请输入数字选择命令:")
    
    def send_control_command(self, command_data):
        msg = String()
        msg.data = command_data
        self.control_pub.publish(msg)
        self.get_logger().info(f'已发送控制命令: {command_data}')
    
    def send_displacement_command(self, positions):
        msg = Float64MultiArray()
        msg.data = positions
        self.displacement_pub.publish(msg)
        self.get_logger().info(f'已发送位移命令: {positions}')
    
    def send_jog_command(self, command):
        msg = String()
        msg.data = command
        self.jog_pub.publish(msg)
        self.get_logger().info(f'已发送点动命令: {command}')
    
    def send_trajectory_action(self):
        # 这里需要安装action_msgs包，这里简化处理
        self.get_logger().info('轨迹动作需要单独执行action命令')
        print("请手动执行: ros2 action send_goal /arm_2zhou_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"trajectory: {joint_names: [joint2, joint3], points: [{positions: [0.5, 0.0], time_from_start: {sec: 4, nanosec: 0}]}\"")
    
    def get_key(self):
        if select.select([sys.stdin], [], [], 0.1)[0]:
            return sys.stdin.read(1)
        return None
    
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key:
                    if key == 'q':
                        break
                    elif key == '1':
                        self.send_control_command('start_manual')
                    elif key == '2':
                        self.send_control_command('start_auto')
                    elif key == '3':
                        self.send_control_command('stop')
                    elif key == '4':
                        self.send_control_command('clear_fault')
                    elif key == '5':
                        self.send_control_command('reset')
                    elif key == '6':
                        self.send_jog_command('axis1_2:forward')
                        # self.send_trajectory_action()
                    elif key == '7':
                        self.send_jog_command('axis1_2:forward')
                        # self.send_displacement_command([1.0, 0.0])
                    elif key == '8':
                        self.send_jog_command('axis2_2:forward')
                    elif key == '9':
                        self.send_jog_command('axis2_2:reverse')
                    elif key == '0':
                        self.send_jog_command('axis1_1:stop')
                        self.send_jog_command('axis1_2:stop')
                        self.send_jog_command('axis2_1:stop')
                        self.send_jog_command('axis2_2:stop')
                    else:
                        print(f"未知命令: {key}")
                        self.print_menu()
                
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
        pass
    finally:
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()