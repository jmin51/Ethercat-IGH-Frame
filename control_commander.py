#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, Int8, Empty
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
        self.warehouse_start_pub = self.create_publisher(Int8, '/warehouse_start', 10)
        self.warehouse_stop_pub = self.create_publisher(Empty, '/warehouse_stop', 10)
        # 新增出库控制发布器
        self.outbound_start_pub = self.create_publisher(Int8, '/outbound_start', 10)
        self.outbound_stop_pub = self.create_publisher(Empty, '/outbound_stop', 10)
        # 新增层控制发布器
        self.layer_pub = self.create_publisher(Int8, '/layer_command', 10)
        
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
            """打印主菜单 - 减少换行符使用"""
            menu = (
                "\n" + "=" * 40 + "\n" +
                "ROS2 机械臂控制主菜单\n" +
                "=" * 40 + "\n" +
                "请选择操作模式或功能：\n" +
                "  01: 进入手动模式     02: 进入自动模式\n" +
                "  03: 停止所有轴       04: 清除故障\n" +
                "  05: 重置轴           06: 发送轨迹动作\n" +
                "  q : 退出程序\n" +
                "=" * 40 + "\n" +
                "请输入命令代码 (例如: 01 或 q): "
            )
            print(menu, end='', flush=True)
    
    def print_manual_menu(self):
            """打印手动控制菜单 - 减少换行符使用"""
            # 1. 构建轴控制区块字符串
            axis_ctrl_block = (
                "[轴控制 - 点动]\n" +
                "  01:轴1_1正转点动   02:轴1_1反转点动   03:轴1_2正转点动   04:轴1_2反转点动\n" +
                "  05:轴2_1正转点动   06:轴2_1反转点动   07:轴2_2正转点动   08:轴2_2反转点动\n" +
                "  09:轴3  正转点动   10:轴3  反转点动   11:轴4  正转点动   12:轴4  反转点动\n" +
                "  13:轴5  正转点动   14:轴5  反转点动\n"
            )
            
            # 2. 构建IO控制区块字符串
            io_lines = []
            io_list = [
                ("15:M800启动按钮灯", '800'), ("16:M801复位按钮灯", '801'),
                ("17:M802暂停按钮灯", '802'), ("18:M803蜂鸣器", '803'),
                ("19:M804三色红灯", '804'), ("20:M805三色黄灯", '805'),
                ("21:M806三色绿灯", '806'), ("22:M810顶升气缸下降", '810'),
                ("23:M811齿轮对接气缸伸出", '811'), ("24:M812皮带正转启动", '812'),
                ("25:M813皮带反转启动", '813')
            ]
            for i in range(0, len(io_list), 4):  # 每行显示4个IO项以压缩行数
                line_parts = []
                for j in range(4):
                    idx = i + j
                    if idx < len(io_list):
                        name, addr = io_list[idx]
                        status = "ON" if self.io_status[addr] else "OFF"
                        line_parts.append(f"  {name}[{status}]")
                io_lines.append("".join(line_parts))
            io_ctrl_block = "[IO控制] (当前状态)\n" + "\n".join(io_lines) + "\n"
            
            # 3. 构建其他控制区块字符串
            other_ctrl_block = (
                "\n[其他控制]\n" +
                "  s:停止所有轴点动  r:复位所有IO  b:返回主菜单\n"
            )
            
            # 4. 拼接最终菜单字符串并打印
            menu = (
                "\n" + "=" * 50 + "\n" +
                "手动模式 - 轴控制与IO控制\n" +
                "=" * 50 + "\n" +
                axis_ctrl_block + "\n" +
                io_ctrl_block +
                other_ctrl_block +
                "=" * 50 + "\n" +
                "请输入命令代码: "
            )
            print(menu, end='', flush=True)
    
    def print_auto_menu(self):
            """打印自动控制菜单 - 减少换行符使用"""
            # 1. 点动控制区块
            jog_ctrl_block = (
                "[点动控制]\t" +
                "  01:轴1_1正转点动  02:轴1_1反转点动  03:轴1_2正转点动  04:轴1_2反转点动\t" +
                "  05:轴2_1正转点动  06:轴2_1反转点动  07:轴2_2正转点动  08:轴2_2反转点动\t" +
                "  09:轴3  正转点动  10:轴3  反转点动\t"
            )
            
            # 2. 轴4位移控制区块
            axis4_block = f"[轴4位移控制] (当前位置: {self.axis4_position:.1f}mm)\t" + \
                        "  11:移动到原点(0.0mm)  12:移动到10.0mm位置  13:正向移动10.0mm  14:反向移动10.0mm\t"
            
            # 3. 轴5位移控制区块
            axis5_block = f"[轴5位移控制] (当前位置: {self.axis5_position:.1f}mm)\t" + \
                        "  15:移动到原点(0.0mm)  16:移动到10.0mm位置  17:正向移动10.0mm  18:反向移动10.0mm\n"
            
            # 4. IO控制区块 (构建方式同手动模式)
            io_lines = []
            io_list = [
                ("19:M800启动按钮灯", '800'), ("20:M801复位按钮灯", '801'),
                ("21:M802暂停按钮灯", '802'), ("22:M803蜂鸣器", '803'),
                ("23:M804三色红灯", '804'), ("24:M805三色黄灯", '805'),
                ("25:M806三色绿灯", '806'), ("26:M810顶升气缸下降", '810'),
                ("27:M811齿轮对接气缸伸出", '811'), ("28:M812皮带正转启动", '812'),
                ("29:M813皮带反转启动", '813')
            ]
            for i in range(0, len(io_list), 4):
                line_parts = []
                for j in range(4):
                    idx = i + j
                    if idx < len(io_list):
                        name, addr = io_list[idx]
                        status = "ON" if self.io_status[addr] else "OFF"
                        line_parts.append(f"  {name}[{status}]")
                io_lines.append("".join(line_parts))
            io_ctrl_block = "\n[IO控制] (当前状态)" + "\t".join(io_lines) + "\t"
            
            # 5. 仓库控制区块
            warehouse_block = (
                "\n[仓库控制]" +
                "  30:启动入库  31:停止入库  32:启动出库  33:停止出库\t"
            )
            
            # 6. 层控制区块
            layer_block = (
                "\t[层控制]" +
                "  34:移动到指定层（请输入层号: -20~30）\t"
            )
            
            # 7. 其他控制区块
            other_ctrl_block = (
                "\t[其他控制]" +
                "  s:停止所有轴(含点动和位移)  r:复位所有IO  b:返回主菜单\t"
            )
            
            # 8. 拼接最终菜单
            menu = (
                "\n" + "=" * 50 + "\n" +
                "自动模式 - 位移控制与IO控制\n" +
                "=" * 50 + "\n" +
                jog_ctrl_block + "\n" +
                axis4_block + "\n" +
                axis5_block +
                io_ctrl_block +
                warehouse_block +
                layer_block +
                other_ctrl_block +
                "=" * 50 + "\n" +
                "请输入命令代码: "
            )
            print(menu, end='', flush=True)
    
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
        msg = Int8()
        msg.data = 24
        self.warehouse_start_pub.publish(msg)
        print("启动入库")
    
    def send_warehouse_stop(self):
        """发送入库停止命令"""
        msg = Empty()
        self.warehouse_stop_pub.publish(msg)
        print("停止入库")
    
    def send_outbound_start(self):
        """发送出库启动命令"""
        msg = Int8()
        msg.data = 24
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
        self.send_control_command('stop')  # 发送停止命令以确保所有运动停止
        self.print_main_menu()
    
    def handle_manual_mode_command(self, key):
        """处理手动模式下的命令 - 更新命令映射以支持所有轴的完整正反转"""
        # 手动模式下的轴控制命令映射 (所有轴都有正反转)
        jog_commands = {
            '01': 'axis1_1:forward',   # 轴1_1 正转
            '02': 'axis1_1:reverse',   # 轴1_1 反转
            '03': 'axis1_2:forward',   # 轴1_2 正转
            '04': 'axis1_2:reverse',   # 轴1_2 反转
            '05': 'axis2_1:forward',   # 轴2_1 正转
            '06': 'axis2_1:reverse',   # 轴2_1 反转
            '07': 'axis2_2:forward',   # 轴2_2 正转
            '08': 'axis2_2:reverse',   # 轴2_2 反转
            '09': 'axis3:forward',     # 轴3 正转
            '10': 'axis3:reverse',     # 轴3 反转
            '11': 'axis4:forward',     # 轴4 正转
            '12': 'axis4:reverse',     # 轴4 反转
            '13': 'axis5:forward',     # 轴5 正转
            '14': 'axis5:reverse',     # 轴5 反转
        }
        
        # IO控制命令映射
        io_commands = {
            '15': '800',  # M800启动按钮灯
            '16': '801',  # M801复位按钮灯
            '17': '802',  # M802暂停按钮灯
            '18': '803',  # M803蜂鸣器
            '19': '804',  # M804三色红灯
            '20': '805',  # M805三色黄灯
            '21': '806',  # M806三色绿灯
            '22': '810',  # M810顶升气缸下降
            '23': '811',  # M811齿轮对接气缸伸出
            '24': '812',  # M812皮带正转启动
            '25': '813',  # M813皮带反转启动
        }
        
        if key in jog_commands:
            self.send_jog_command(jog_commands[key])
            # 显示简化的执行信息
            axis_map = {
                'axis1_1:forward': '轴1_1正转执行',
                'axis1_1:reverse': '轴1_1反转执行',
                'axis1_2:forward': '轴1_2正转执行',
                'axis1_2:reverse': '轴1_2反转执行',
                'axis2_1:forward': '轴2_1正转执行',
                'axis2_1:reverse': '轴2_1反转执行',
                'axis2_2:forward': '轴2_2正转执行',
                'axis2_2:reverse': '轴2_2反转执行',
                'axis3:forward': '轴3正转执行',
                'axis3:reverse': '轴3反转执行',
                'axis4:forward': '轴4正转执行',
                'axis4:reverse': '轴4反转执行',
                'axis5:forward': '轴5正转执行',
                'axis5:reverse': '轴5反转执行',
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
        """处理自动模式下的命令 - 更新命令映射以支持所有轴的完整正反转"""
        # 点动控制命令映射 (所有细分轴都有正反转)
        jog_commands = {
            '01': 'axis1_1:forward',   # 轴1_1 正转
            '02': 'axis1_1:reverse',   # 轴1_1 反转
            '03': 'axis1_2:forward',   # 轴1_2 正转
            '04': 'axis1_2:reverse',   # 轴1_2 反转
            '05': 'axis2_1:forward',   # 轴2_1 正转
            '06': 'axis2_1:reverse',   # 轴2_1 反转
            '07': 'axis2_2:forward',   # 轴2_2 正转
            '08': 'axis2_2:reverse',   # 轴2_2 反转
            '09': 'axis3:forward',     # 轴3 正转
            '10': 'axis3:reverse',     # 轴3 反转
        }
        
        # IO控制命令映射
        io_commands = {
            '19': '800',  # M800启动按钮灯
            '20': '801',  # M801复位按钮灯
            '21': '802',  # M802暂停按钮灯
            '22': '803',  # M803蜂鸣器
            '23': '804',  # M804三色红灯
            '24': '805',  # M805三色黄灯
            '25': '806',  # M806三色绿灯
            '26': '810',  # M810顶升气缸下降
            '27': '811',  # M811齿轮对接气缸伸出
            '28': '812',  # M812皮带正转启动
            '29': '813',  # M813皮带反转启动
        }
        
        if key in jog_commands:
            self.send_jog_command(jog_commands[key])
            # 显示简化的执行信息
            axis_map = {
                'axis1_1:forward': '轴1_1正转执行',
                'axis1_1:reverse': '轴1_1反转执行',
                'axis1_2:forward': '轴1_2正转执行',
                'axis1_2:reverse': '轴1_2反转执行',
                'axis2_1:forward': '轴2_1正转执行',
                'axis2_1:reverse': '轴2_1反转执行',
                'axis2_2:forward': '轴2_2正转执行',
                'axis2_2:reverse': '轴2_2反转执行',
                'axis3:forward': '轴3正转执行',
                'axis3:reverse': '轴3反转执行',
            }
            print(axis_map.get(jog_commands[key], "执行"))
        elif key in io_commands:
            # 处理IO控制
            self.toggle_io(io_commands[key])
        elif key == 's':
            self.stop_all_jog()
        elif key == 'r':  # 复位所有IO
            self.reset_all_io()
        elif key == 'b':
            self.return_to_main_menu()
        # 轴4位移控制 (命令代码更新为11-14)
        elif key == '11':  # 轴4移动到原点
            self.axis4_position = 0.0
            self.send_displacement_command('axis4', 0.0)
            print("轴4移动到原点")
            self.print_auto_menu()
        
        elif key == '12':  # 轴4移动到10mm
            self.axis4_position = 10.0
            self.send_displacement_command('axis4', 10.0)
            print("轴4移动到10mm")
            self.print_auto_menu()

        elif key == '13':  # 轴4正向移动10mm
            self.axis4_position += 10.0
            self.send_displacement_command('axis4', self.axis4_position)
            print("轴4正向移动10mm")
            self.print_auto_menu()

        elif key == '14':  # 轴4反向移动10mm
            self.axis4_position -= 10.0
            self.send_displacement_command('axis4', self.axis4_position)
            print("轴4反向移动10mm")
            self.print_auto_menu()
        
        # 轴5位移控制 (命令代码更新为15-18)
        elif key == '15':  # 轴5移动到原点
            self.axis5_position = 0.0
            self.send_displacement_command('axis5', 0.0)
            print("轴5移动到原点")
            self.print_auto_menu()

        elif key == '16':  # 轴5移动到10mm
            self.axis5_position = 10.0
            self.send_displacement_command('axis5', 10.0)
            print("轴5移动到10mm")
            self.print_auto_menu()

        elif key == '17':  # 轴5正向移动10mm
            self.axis5_position += 10.0
            self.send_displacement_command('axis5', self.axis5_position)
            print("轴5正向移动10mm")
            self.print_auto_menu()

        elif key == '18':  # 轴5反向移动10mm
            self.axis5_position -= 10.0
            self.send_displacement_command('axis5', self.axis5_position)
            print("轴5反向移动10mm")
            self.print_auto_menu()
        
        # 仓库控制命令 (命令代码更新为30-33)
        elif key == '30':  # 启动入库
            self.send_warehouse_start()
        
        elif key == '31':  # 停止入库
            self.send_warehouse_stop()
        
        elif key == '32':  # 启动出库
            self.send_outbound_start()
        
        elif key == '33':  # 停止出库
            self.send_outbound_stop()
        # 层控制命令 (新增命令34)
        elif key == '34':  # 移动到指定层
            print("请输入要移动到的层号（-20~30），按回车确认: ", end='', flush=True)
            
            # 临时恢复终端设置以读取整行输入
            import termios
            import tty
            import sys
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            
            try:
                # 读取用户输入的层号
                layer_input = ""
                while True:
                    char = sys.stdin.read(1)
                    if char == '\n' or char == '\r':  # 回车结束输入
                        break
                    elif char == '\x7f' or char == '\b':  # 退格键
                        if layer_input:
                            layer_input = layer_input[:-1]
                            # 回显退格
                            sys.stdout.write('\b \b')
                            sys.stdout.flush()
                    else:
                        layer_input += char
                        sys.stdout.write(char)
                        sys.stdout.flush()
                
                sys.stdout.write('\n')
                sys.stdout.flush()
                
                if layer_input:
                    try:
                        layer_num = int(layer_input)
                        if -20 <= layer_num <= 30:
                            # 发送层控制命令
                            msg = Int8()
                            msg.data = layer_num
                            self.layer_pub.publish(msg)
                            print(f"已发送移动至层 {layer_num} 的命令: ros2 topic pub /layer_command std_msgs/msg/Int8 \"data: {layer_num}\" --once")
                        else:
                            print(f"错误：层号 {layer_num} 超出范围（-20~30）")
                    except ValueError:
                        print(f"错误：'{layer_input}' 不是有效的整数")
            finally:
                # 恢复终端设置
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                tty.setraw(sys.stdin.fileno())
            
            # 重新打印菜单
            self.print_auto_menu()
        
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