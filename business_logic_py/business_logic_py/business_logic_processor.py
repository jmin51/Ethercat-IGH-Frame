#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Empty, Bool, Float64
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

class PassThroughState(Enum):
    """放行流程状态"""
    IDLE = auto()
    RETURNING_TO_LAYER_1 = auto()      # 回到第一层
    WAIT_FOR_PRODUCT_ARRIVAL = auto()  # 等待产品到位
    CONVEYOR_RUNNING = auto()          # 输送带运行中（自动放行）
    WAIT_FOR_CONVEYOR_OUT = auto()
    COMPLETED = auto()

class CommandType(Enum):
    JOG = auto()
    LAYER = auto()
    POSITION = auto()
    STOP = auto()

# 故障码定义（业务逻辑层）- 与C++层保持一致
# C++层定义：CATEGORY_BUSINESS = 0x5000, BUSINESS_SEQUENCE = 0x0200
class FaultCode(Enum):
    """故障码定义 - 遵循C++层 fault_codes.hpp 规范
    业务逻辑错误类别：0x5xxx
    子类别：BUSINESS_SEQUENCE = 0x0200 (业务流程序列)
    """
    NO_FAULT = 0x0000              # 无故障
    WAREHOUSE_TIMEOUT = 0x5201     # 入库流程超时 (0x5000 | 0x0200 | 0x01)
    OUTBOUND_TIMEOUT = 0x5202      # 出库流程超时 (0x5000 | 0x0200 | 0x02)
    GEAR_CYLINDER_TIMEOUT = 0x5203 # 齿轮对接气缸伸出超时 (0x5000 | 0x0200 | 0x03)

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
        
        # ========== 宏开关配置 ==========
        self.ENABLE_SMEMA = True  # True:启用SMEMA协议通讯 False:禁用SMEMA协议
        
        # 初始化状态变量
        self.warehouse_state = WarehouseState.IDLE
        self.outbound_state = OutboundState.IDLE
        self.release_state = PassThroughState.IDLE  # 新增：放行流程状态
        self.current_layer = 1
        self.target_layer = 1
        self.source_layer = 1
        self.outbound_area = 0  # 新增：出库区域（0=正常出库，1=特殊出库到18层）
        
        # 添加上一个状态记录
        self.previous_warehouse_state = WarehouseState.IDLE
        self.previous_outbound_state = OutboundState.IDLE
        self.previous_release_state = PassThroughState.IDLE  # 新增：放行流程上一个状态
        self.previous_io_signals = {
            'buffer_in_position': False,
            'buffer_out_position': False,
            'conveyor_in_position': False,
            'conveyor_out_position': False,
            'buffer_sensor_2': False
        }
        # 添加状态变化计数器（避免频繁打印）
        self.state_change_counter = 0
        
        # POST_LIFT_PROCESSING 延迟计时
        self.post_lift_delay_start = None

        # 控制标志
        self.warehouse_process_requested = False
        self.warehouse_process_stop_requested = False
        self.outbound_process_requested = False
        self.outbound_process_stop_requested = False
        self.release_process_requested = False  # 新增：放行流程启动请求
        self.release_process_stop_requested = False  # 新增：放行流程停止请求
        self.release_request_received = False  # 新增：放行请求信号（收到后启动输送带）
        
        # +++ 新增：暂停状态记录相关 +++
        self.pause_state_reported = False  # 是否已报告暂停状态
        self.resuming_from_pause = False   # 是否正在从暂停恢复
        self.saved_warehouse_state = None  # 保存的入库状态
        self.saved_outbound_state = None   # 保存的出库状态
        self.saved_target_layer = 1        # 保存的目标层
        self.saved_source_layer = 1        # 保存的源层
        
        # +++ 新增：自动模式初始化等待相关 +++
        self.auto_mode_initialized = False  # 自动模式位置初始化是否完成
        self.pending_resume_state = None    # 待执行的恢复状态（等待轴就绪后执行）
        
        # 自动模式状态
        self.auto_mode_enabled = False
        self.enabled = True
        self.initialized = False
        
        # 延迟控制
        self.delay_started = False
        self.delay_counter = 0
        self.delay_condition_triggered = False
        self.conveyor_in_detected = False
        self.buffer_sensor_2_detected = False  # 新增：缓存架对射2检测状态
        
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
        
        # +++ 新增：放行流程状态变量 +++
        self.release_conveyor_in_was_true = False  # conveyor_in曾经为True的标志
        self.release_conveyor_in_completed = False  # conveyor_in检测完成标志（无→有→无）
        self.release_conveyor_out_was_true = False  # conveyor_out曾经为True的标志
        self.release_conveyor_out_delay_started = False  # 放行完成延迟开始标志
        self.release_conveyor_out_delay_counter = 0  # 放行完成延迟计数器
        self.release_completion_published = False  # 放行完成消息发布标志

        # +++ 新增：811齿轮对接气缸检测相关状态 +++
        self.GEAR_CYLINDER_TIMEOUT_SEC = 2.0  # 齿轮对接气缸伸出超时时间2秒
        self.gear_cylinder_811_sent_time = None  # 811发送时间
        self.gear_cylinder_811_checking = False  # 是否正在检测气缸到位
        self.gear_cylinder_811_timeout_reported = False  # 超时已上报标志

        # 新增：产品到位发布状态管理（0x0109）
        self.product_arrival_cycle_active = False  # 是否处于产品到位发布周期中
        self.product_arrival_published_in_cycle = False  # 本轮周期是否已发布过到位
        self.product_arrival_phase = "idle"  # 当前阶段: idle/pre_warehouse/warehouse/post_warehouse
        
        # +++ 新增：产品到位信号跟踪状态（用于判断是否可以要板）+++
        self.feed_detect_was_true = False  # feed_detect曾经为True的标志
        self.buffer_out_was_true_for_arrival = False  # buffer_out曾经为True的标志（产品到位检测）
        self.conveyor_in_was_true_for_arrival = False  # conveyor_in曾经为True的标志（到达接驳台）
        self.conveyor_in_completed_for_arrival = False  # conveyor_in检测完成标志（无→有→无，产品到达接驳台）

        # 常量定义
        self.DELAY_BEFORE_STOP_MS = 600
        self.DELAY_COUNTER_MAX = self.DELAY_BEFORE_STOP_MS // 100
        self.OUTBOUND_DELAY_BEFORE_STOP_MS = 300
        self.OUTBOUND_DELAY_COUNTER_MAX = self.OUTBOUND_DELAY_BEFORE_STOP_MS // 100
        
        # 待处理命令队列
        self.pending_commands = []
        
        # 当前IO信号状态
        self.current_io_signals = {
            'buffer_in_position': False,
            'buffer_out_position': False,
            'conveyor_in_position': False,
            'conveyor_out_position': False,
            'buffer_sensor_2': False,
            'feed_product_detect': False,  # 新增：入料产品检测信号
            # SMEMA信号（C++层维护，Python层只读取）
            'smema_uba': False,   # 上游有板待发
            'smema_dbr': False    # 下游要板信号
        }
        
        # +++ 新增：产品到位检测状态机 +++
        self.product_arrival_state = "IDLE"  # IDLE/WAITING_FEED/CONVEYOR_RUNNING/WAITING_BUFFER_IN/WAITING_BUFFER_OUT/COMPLETED
        self.feed_detected = False           # 是否检测到feed_product_detect
        self.buffer_in_detected = False      # 是否检测到buffer_in
        self.conveyor_started_for_arrival = False  # 输送带是否已启动用于进料
        
        # +++ SMEMA协议状态（从IO状态中读取） +++
        # 不需要额外订阅，直接从current_io_signals中读取smema_uba和smema_dbr信号
        
        # 新增：DO控制状态跟踪，避免重复发送
        self.last_do_commands = {}  # 存储每个DO地址的最后状态
        self.do_command_sent = {}   # 标记DO命令是否已发送
        
        # 创建发布器
        self.jog_pub = self.create_publisher(String, '/jog_command', 10)
        self.do_control_pub = self.create_publisher(String, '/do_control', 10)
        self.layer_pub = self.create_publisher(Int8, '/layer_command', 10)
        
        # 在 __init__ 方法中添加发布器（在现有发布器之后）
        self.warehouse_completed_pub = self.create_publisher(Bool, '/warehouse_completed', 10)
        self.outbound_completed_pub = self.create_publisher(Bool, '/outbound_completed', 10)
        self.product_arrival_pub = self.create_publisher(Bool, '/product_arrival', 10)
        self.release_completed_pub = self.create_publisher(Bool, '/release_completed', 10)  # 新增：放行完成发布器
        
        # +++ SMEMA协议接口（C++层） +++
        if self.ENABLE_SMEMA:
            # 发布产品到位信号（驱动C++层握手）
            self.product_position_pub = self.create_publisher(Bool, '/smema/product_in_position', 10)
        
        # 新增：订阅开始作业信号（用于启动产品到位发布周期）
        self.start_operation_sub = self.create_subscription(
            Bool,
            '/start_operation_signal',
            self.start_operation_signal_callback,
            10
        )
   
        # 创建订阅器 - 使用 /io_status 话题（C++层发布的完整IO状态）
        self.io_status_sub = self.create_subscription(
            String, 
            '/io_status', 
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
            Int8,
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
            Int8,
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
        
        # 新增：订阅出库区域话题
        self.outbound_area_sub = self.create_subscription(
            Int8,
            '/outbound_area',
            self.outbound_area_callback,
            10
        )
        
        # +++ 新增：放行流程订阅器 +++
        self.release_start_sub = self.create_subscription(
            Empty,
            '/release_start',
            self.release_start_callback,
            10
        )
        
        self.release_stop_sub = self.create_subscription(
            Empty,
            '/release_stop',
            self.release_stop_callback,
            10
        )
        
        # 修改：添加层移动状态跟踪
        self.layer_motion_completed = False
        self.previous_layer_completion_state = False  # 添加上一个状态记录
        self.layer_completion_received_time = None    # 添加接收时间记录
        self.layer_completion_timeout = 30.0          # 30秒超时
        
        # 添加状态跟踪变量
        self.warehouse_completion_published = False
        self.outbound_completion_published = False
        self.last_warehouse_state = WarehouseState.IDLE
        self.last_outbound_state = OutboundState.IDLE
        
        # 新增：COMPLETED 状态复位指令发送标志（避免重复日志）
        self.completed_reset_command_sent = False

        # 新增：当前层号（从axis5实际位置计算，支持小数层）
        self.current_layer_float = 1.0  # 浮点层号（如5.5层）
        self.layer_tolerance = 0.1      # 层号容差（如目标5层，实际4.7-5.3都认为到位）

        # +++ 新增：入库/出库流程超时检测 +++
        self.WAREHOUSE_PROCESS_TIMEOUT =60.0  # 入库流程总超时60秒
        self.OUTBOUND_PROCESS_TIMEOUT = 60.0   # 出库流程总超时60秒
        self.warehouse_process_start_time = None  # 入库流程开始时间
        self.outbound_process_start_time = None   # 出库流程开始时间
        self.warehouse_timeout_reported = False   # 入库超时已上报标志
        self.outbound_timeout_reported = False    # 出库超时已上报标志

        # +++ 新增：故障码发布器（发布到独立话题，由C++层统一转发到/fault_code）+++ 
        self.fault_code_pub = self.create_publisher(String, '/business_logic_fault', 10)
        self.last_published_fault_code = FaultCode.NO_FAULT.value  # 上次发布的故障码

        # 创建层移动完成订阅器
        self.layer_completion_sub = self.create_subscription(
            Bool,
            '/layer_motion_completed',
            self.layer_completion_callback,
            10
        )

        # 新增：订阅当前层号（从axis5实际位置计算）
        self.current_layer_sub = self.create_subscription(
            Float64,
            '/axis5_current_layer',
            self.current_layer_callback,
            10
        )
        
        # +++ 新增：创建暂停状态命令订阅器 +++
        self.pause_state_sub = self.create_subscription(
            String,
            '/pause_state_command',
            self.pause_state_command_callback,
            10
        )
        # +++ 新增：创建暂停状态报告发布器 +++
        self.pause_state_report_pub = self.create_publisher(
            String, '/pause_state_report', 10
        )
        

        
        # 定时器 - 处理业务逻辑
        self.timer = self.create_timer(0.1, self.process_logic)  # 100ms周期
        
        self.get_logger().info('Python业务逻辑处理器已启动 - 支持暂停状态记录与恢复')

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
        
        # +++ 新增：检测自动模式初始化完成（恢复时等待轴就绪）+++
        elif '自动模式初始化完成' in status_text:
            if not self.auto_mode_initialized:
                self.auto_mode_initialized = True
                self.get_logger().info('检测到自动模式初始化完成，轴已就绪')
                # 如果有待执行的恢复状态，立即执行
                if self.pending_resume_state is not None:
                    self.execute_pending_resume()
            
            # +++ 关键修复：IO控制模式下，通过自动模式初始化完成信号启用业务逻辑自动模式 +++
            if not self.auto_mode_enabled:
                self.auto_mode_enabled = True
                self.get_logger().info('IO控制模式下，检测到轴自动模式就绪，业务逻辑处理器进入自动模式')

    def io_status_callback(self, msg):
        """处理IO状态更新"""
        try:
            io_data = self.parse_io_status(msg.data)
            self.current_io_signals.update(io_data)
            self.process_io_signals()
        except Exception as e:
            self.get_logger().error(f'IO状态解析错误: {e}')

    def parse_io_status(self, io_data: str) -> Dict[str, bool]:
        """解析IO状态字符串为字典（包含DI和DO信号）"""
        io_signals = {}
        try:
            # 解析DI信号
            di_pattern = r'DI(\d+):(\d)'
            di_matches = re.findall(di_pattern, io_data)
            
            for di_num, value in di_matches:
                di_num_int = int(di_num)
                # 映射到标准信号名称
                signal_name = self.map_signal_name(di_num_int)
                if signal_name:
                    io_signals[signal_name] = (value == '1')
            
            # 解析DO信号（SMEMA需要读取DO状态）
            do_pattern = r'DO(\d+):(\d)'
            do_matches = re.findall(do_pattern, io_data)
            
            for do_num, value in do_matches:
                do_num_int = int(do_num)
                # 映射DO信号
                if do_num_int == 14:
                    io_signals['smema_mr'] = (value == '1')  # M814 SMEMA本机要板
                elif do_num_int == 15:
                    io_signals['smema_ba'] = (value == '1')  # M815 SMEMA本机有板
                    
        except Exception as e:
            self.get_logger().error(f'IO状态解析失败: {e}')
        
        return io_signals

    def map_signal_name(self, di_number: int) -> str:
        """映射DI编号到标准信号名称
        
        DI编号与M寄存器对应关系：
        DI00-DI08: M512-M520 (按钮、急停、气源、安全门、入料检测)
        DI09: M521 buffer_sensor_1 (缓存架对射1)
        DI10: M522 buffer_sensor_2 (缓存架对射2)
        DI11: M523 buffer_in_position (缓存架入料到位)
        DI12: M524 buffer_out_position (缓存架出料到位)
        DI13: M525 conveyor_in_position (接驳台入料到位)
        DI14: M526 conveyor_out_position (接驳台出料到位)
        DI23: M535 smema_uba (SMEMA上游有板)
        DI24: M536 smema_dbr (SMEMA下游要板)
        """
        mapping = {
            8: 'feed_product_detect',   # M520 入料产品检测
            10: 'buffer_sensor_2',      # M522 缓存架对射2
            11: 'buffer_in_position',   # M523 缓存架入料产品到位检测
            12: 'buffer_out_position',  # M524 缓存架出料产品到位检测
            13: 'conveyor_in_position', # M525 接驳台入料产品到位检测
            14: 'conveyor_out_position', # M526 接驳台出料产品到位检测
            21: 'gear_cylinder_2_in_position',  # M533 齿轮对接气缸2伸出到位
            # SMEMA信号
            23: 'smema_uba',            # M535 SMEMA上游有板待发
            24: 'smema_dbr',            # M536 SMEMA下游要板
        }
        return mapping.get(di_number, '')

    def warehouse_start_callback(self, msg):
        """处理入库启动命令"""
        if not self.auto_mode_enabled:
            self.get_logger().warn('自动模式未启用，忽略入库启动命令')
            return
            
        if self.warehouse_state not in [WarehouseState.IDLE, WarehouseState.COMPLETED]:
            self.get_logger().warn('入库流程已在运行中，无法重复启动')
            return
        
        self.target_layer = msg.data
        self.warehouse_process_requested = True
        self.warehouse_process_stop_requested = False
        # 重置完成发布标志，确保下次入库可以正常发布完成消息
        self.warehouse_completion_published = False
        self.get_logger().info(f'收到入库流程启动请求，目标层: {self.target_layer}')

    def warehouse_stop_callback(self, msg):
        """处理入库停止命令"""
        self._reset_key_do_signals()
        self.warehouse_process_stop_requested = True
        self.get_logger().info('收到入库流程停止请求')

    def start_operation_signal_callback(self, msg):
        """处理开始作业信号（0x0105收到后触发）"""
        if msg.data:
            self.get_logger().info('收到开始作业信号(0x0105)，启动产品到位发布周期')
            # 启动产品到位发布周期
            self.product_arrival_cycle_active = True
            self.product_arrival_published_in_cycle = False
            self.product_arrival_phase = "pre_warehouse"
            # 重置产品到位检测状态机
            self._reset_product_arrival_state_machine()

    def process_product_arrival_logic(self):
        """处理产品到位检测状态机（新逻辑）
        
        状态流转：
        IDLE → CONVEYOR_RUNNING: 检测到feed_product_detect上升沿，启动输送带
        CONVEYOR_RUNNING → WAITING_BUFFER_OUT: 检测到buffer_in上升沿
        WAITING_BUFFER_OUT → COMPLETED: 检测到buffer_out有信号，发布0x0109
        COMPLETED → IDLE: 等待下一轮
        """
        # 如果没有处于产品到位发布周期，不处理
        if not self.product_arrival_cycle_active:
            return
        
        # 如果本轮已经发布过到位信息，不重复发布
        if self.product_arrival_published_in_cycle:
            return
        
        di = self.current_io_signals
        feed_detect = di['feed_product_detect']
        buffer_in = di['buffer_in_position']
        buffer_out = di['buffer_out_position']  # 新增：读取buffer_out信号
        
        # 状态机处理
        if self.product_arrival_state == "IDLE":
            # 等待进料检测信号
            if feed_detect and not self.feed_detected:
                # 检测到feed_product_detect上升沿
                self.feed_detected = True
                self.product_arrival_state = "CONVEYOR_RUNNING"
                self.get_logger().info('产品到位检测：检测到feed_product_detect，启动输送带')
                
                # 启动输送带
                self.add_command(ControlAction(
                    CommandType.JOG, "axis1_1", "reverse",
                    description="进料：启动轴1_1反转"
                ))
                self.add_command(ControlAction(
                    CommandType.JOG, "axis1_2", "forward",
                    description="进料：启动轴1_2正转"
                ))
                self.conveyor_started_for_arrival = True
        
        elif self.product_arrival_state == "CONVEYOR_RUNNING":
            # 等待buffer_in信号
            if buffer_in and not self.buffer_in_detected:
                # 检测到buffer_in上升沿
                self.buffer_in_detected = True
                self.product_arrival_state = "WAITING_BUFFER_OUT"
                self.get_logger().info('产品到位检测：检测到buffer_in，等待产品离开')
        
        elif self.product_arrival_state == "WAITING_BUFFER_OUT":
            # 等待buffer_out有信号（产品到达缓存架出料口）
            if buffer_out:
                # 检测到buffer_out有信号，产品到位
                self.get_logger().info('产品到位检测：检测到buffer_out信号，产品已到达缓存架出料口')
                
                # 停止输送带
                if self.conveyor_started_for_arrival:
                    self.add_command(ControlAction(
                        CommandType.JOG, "axis1_1", "stop",
                        description="进料：停止轴1_1"
                    ))
                    self.add_command(ControlAction(
                        CommandType.JOG, "axis1_2", "stop",
                        description="进料：停止轴1_2"
                    ))
                    self.conveyor_started_for_arrival = False
                
                # 发布产品到位消息0x0109
                arrival_msg = Bool()
                arrival_msg.data = True
                self.product_arrival_pub.publish(arrival_msg)
                self.product_arrival_published_in_cycle = True
                self.product_arrival_state = "COMPLETED"
                self.get_logger().info(f'✅ 产品到位检测完成，发布0x0109（阶段: {self.product_arrival_phase}）')
        
        elif self.product_arrival_state == "COMPLETED":
            # 等待feed_detect消失，重置状态机
            if not feed_detect:
                self.product_arrival_state = "IDLE"
                self.feed_detected = False
                self.buffer_in_detected = False
                # +++ 新增：自动清除产品到位发布标志 +++
                # 当feed_detect消失时，表示产品已经完全到位，可以开始下一轮检测
                if self.product_arrival_published_in_cycle:
                    self.product_arrival_published_in_cycle = False
                    self.get_logger().info('产品到位检测：feed_detect消失，自动清除到位标志，准备下一轮检测')
                self.get_logger().info('产品到位检测：重置状态机，等待下一轮')

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
        # 重置完成发布标志，确保下次出库可以正常发布完成消息
        self.outbound_completion_published = False
        self.get_logger().info(f'收到出库流程启动请求，源层: {self.source_layer}')

    def outbound_stop_callback(self, msg):
        """处理出库停止命令"""
        self._reset_key_do_signals()
        self.outbound_process_stop_requested = True
        self.get_logger().info('收到出库流程停止请求')
    
    def outbound_area_callback(self, msg):
        """处理出库区域命令
        
        出库区域：
        0 = 正常出库（到第1层）
        1 = 特殊出库（先到第18层，完成后回到第1层）
        """
        self.outbound_area = msg.data
        self.get_logger().info(f'收到出库区域设置: {self.outbound_area}')
    
    def release_start_callback(self, msg):
        """处理放行启动命令 - 进入等待产品到位状态"""
        if not self.auto_mode_enabled:
            self.get_logger().warn('自动模式未启用，忽略放行启动命令')
            return
            
        if self.release_state not in [PassThroughState.IDLE, PassThroughState.COMPLETED]:
            self.get_logger().warn('放行流程已在运行中，无法重复启动')
            return
        
        self.release_process_requested = True
        self.release_process_stop_requested = False
        # 重置完成发布标志，确保下次放行可以正常发布完成消息
        self.release_completion_published = False
        self.get_logger().info('收到放行流程启动请求，进入等待产品到位状态')

    def _reset_product_arrival_state_machine(self):
        """重置产品到位检测状态机"""
        self.product_arrival_state = "IDLE"
        self.feed_detected = False
        self.buffer_in_detected = False
        # +++ 新增：重置产品到位信号跟踪状态 +++
        self.feed_detect_was_true = False
        self.buffer_out_was_true_for_arrival = False
        self.conveyor_in_was_true_for_arrival = False
        self.conveyor_in_completed_for_arrival = False
        if self.conveyor_started_for_arrival:
            # 如果输送带还在运行，停止它
            self.add_command(ControlAction(
                CommandType.JOG, "axis1_1", "stop",
                description="重置：停止轴1_1"
            ))
            self.add_command(ControlAction(
                CommandType.JOG, "axis1_2", "stop",
                description="重置：停止轴1_2"
            ))
            self.conveyor_started_for_arrival = False
        self.get_logger().debug('产品到位检测状态机已重置')

    def release_stop_callback(self, msg):
        """处理放行停止命令"""
        self._reset_key_do_signals()
        self.release_process_stop_requested = True
        self.get_logger().info('收到放行流程停止请求')

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

    def current_layer_callback(self, msg):
        """当前层号回调（从axis5实际位置计算）"""
        try:
            new_layer = msg.data
            # 只记录显著变化
            if abs(new_layer - self.current_layer_float) > 0.01:
                self.get_logger().debug(f'当前层号更新: {self.current_layer_float:.2f} -> {new_layer:.2f}')
            self.current_layer_float = new_layer
            # 同时更新整数层号（四舍五入）
            self.current_layer = round(new_layer)
        except Exception as e:
            self.get_logger().error(f'当前层号回调处理错误: {e}')

    def is_target_layer_reached(self, target_layer):
        """检查目标层号是否已到达（考虑容差）"""
        diff = abs(self.current_layer_float - target_layer)
        return diff <= self.layer_tolerance

    # +++ 新增：暂停状态命令回调处理 +++
    def pause_state_command_callback(self, msg):
        """处理暂停状态命令"""
        command = msg.data
        self.get_logger().info(f'收到暂停状态命令: {command}')
        
        if command == "RECORD_STATE":
            # 记录当前业务状态并报告
            self.report_current_pause_state()
        elif command.startswith("RESUME:"):
            # 解析恢复命令并执行恢复
            self.handle_resume_command(command)
    
    def report_current_pause_state(self):
        """报告当前业务状态供C++端记录 - 修复：IDLE/COMPLETED状态使用实际层号"""
        # 记录当前状态
        self.saved_warehouse_state = self.warehouse_state
        self.saved_outbound_state = self.outbound_state
        
        # +++ 关键修复：IDLE或COMPLETED状态时，使用当前实际层号（从axis5位置计算）+++
        # 而不是目标层号（可能残留上次的目标层，如-12层）
        if self.warehouse_state in [WarehouseState.IDLE, WarehouseState.COMPLETED]:
            # 使用从axis5位置计算的实际层号（四舍五入为整数，限制在1-28范围）
            actual_layer = max(1, min(28, round(self.current_layer_float)))
            self.saved_target_layer = actual_layer
            self.get_logger().info(f'记录暂停状态：入库{self.warehouse_state.name}，使用实际层号={actual_layer} (浮点层={self.current_layer_float:.2f})')
        else:
            self.saved_target_layer = self.target_layer
            
        if self.outbound_state in [OutboundState.IDLE, OutboundState.COMPLETED]:
            actual_layer = max(1, min(28, round(self.current_layer_float)))
            self.saved_source_layer = actual_layer
            self.get_logger().info(f'记录暂停状态：出库{self.outbound_state.name}，使用实际层号={actual_layer} (浮点层={self.current_layer_float:.2f})')
        else:
            self.saved_source_layer = self.source_layer
        
        # 构建状态报告字符串
        # 格式: warehouse_active=1,warehouse_state=X,warehouse_layer=Y,outbound_active=0,...
        warehouse_active = 1 if self.warehouse_state != WarehouseState.IDLE else 0
        outbound_active = 1 if self.outbound_state != OutboundState.IDLE else 0
        
        report = (f"warehouse_active={warehouse_active},"
                  f"warehouse_state={self.warehouse_state.value},"
                  f"warehouse_layer={self.saved_target_layer},"
                  f"outbound_active={outbound_active},"
                  f"outbound_state={self.outbound_state.value},"
                  f"outbound_layer={self.saved_source_layer}")
        
        # 发布状态报告
        msg = String()
        msg.data = report
        self.pause_state_report_pub.publish(msg)
        
        self.pause_state_reported = True
        self.get_logger().info(f'已报告暂停状态: {report}')
    
    def handle_resume_command(self, command):
        """处理恢复命令，重走之前记录的状态"""
        self.get_logger().info(f'处理恢复命令: {command}')
        
        # 解析恢复命令
        # 格式: RESUME:warehouse_active=1,warehouse_state=X,warehouse_layer=Y,outbound_active=0,...
        try:
            params = {}
            parts = command.replace("RESUME:", "").split(",")
            for part in parts:
                key, value = part.split("=")
                params[key] = int(value)
            
            # 设置恢复标志
            self.resuming_from_pause = True
            
            # 根据记录的状态恢复业务逻辑
            warehouse_active = params.get('warehouse_active', 0)
            outbound_active = params.get('outbound_active', 0)
            
            if warehouse_active:
                # 恢复入库流程
                saved_state = params.get('warehouse_state', 1)
                saved_layer = params.get('warehouse_layer', 1)
                self.restore_warehouse_state(saved_state, saved_layer)
            
            if outbound_active:
                # 恢复出库流程
                saved_state = params.get('outbound_state', 1)
                saved_layer = params.get('outbound_layer', 1)
                self.restore_outbound_state(saved_state, saved_layer)
            
            self.get_logger().info('业务逻辑状态恢复完成，继续执行')
            
        except Exception as e:
            self.get_logger().error(f'处理恢复命令失败: {e}')
    
    def restore_warehouse_state(self, state_value, target_layer):
        """恢复入库流程到指定状态"""
        self.get_logger().info(f'恢复入库流程: 状态={state_value}, 目标层={target_layer}')
        
        # 重置层指令发送状态，确保恢复时可以重新发送层指令
        self.layer_command_sent = False
        self.last_layer_command = None
        
        # 设置目标层
        self.target_layer = target_layer
        
        # 根据状态值恢复
        if state_value == WarehouseState.IDLE.value:
            self.warehouse_state = WarehouseState.IDLE
            # +++ 新增：IDLE状态恢复时，如果不在第1层，发送回到第1层的指令 +++
            if not self.is_target_layer_reached(1):
                if self.auto_mode_initialized:
                    self.send_layer_command(1)
                    self.get_logger().info(f'从IDLE恢复，当前层={self.current_layer_float:.2f}，发送层指令回到第1层')
                else:
                    self.pending_resume_state = {
                        'type': 'warehouse_idle',
                        'layer': 1
                    }
                    self.get_logger().info(f'从IDLE恢复，等待轴自动模式初始化完成后发送层指令回到第1层')
            else:
                self.get_logger().info(f'从IDLE恢复，当前已在第1层({self.current_layer_float:.2f})，无需移动')
        elif state_value == WarehouseState.WAIT_FOR_ENTRY.value:
            self.warehouse_state = WarehouseState.WAIT_FOR_ENTRY
            self.warehouse_process_requested = True
            # 重置完成发布标志，确保后续能正确回包102
            self.warehouse_completion_published = False
            self.get_logger().info('从WAIT_FOR_ENTRY恢复，重置完成发布标志')
        elif state_value == WarehouseState.CONVEYOR_MOVING.value:
            # 从输送带运行状态恢复 - 重新检测条件
            self.warehouse_state = WarehouseState.WAIT_FOR_ENTRY
            self.warehouse_process_requested = True
            # 重置完成发布标志，确保后续能正确回包102
            self.warehouse_completion_published = False
            self.get_logger().info('从CONVEYOR_MOVING恢复，将重新检测入库条件，重置完成发布标志')
        elif state_value == WarehouseState.LIFT_MOVING.value:
            # 从提升机运行状态恢复 - 保持在LIFT_MOVING并重新发送层指令
            # 因为conveyor_in信号可能已不满足，不能直接回退到CONVEYOR_MOVING
            self.warehouse_state = WarehouseState.LIFT_MOVING
            self.warehouse_process_requested = True
            # 重置层移动状态（关键：必须同时重置previous_layer_completion_state）
            self.layer_motion_completed = False
            self.previous_layer_completion_state = False
            self.layer_completion_received_time = None
            # 重置完成发布标志，确保后续能正确回包102
            self.warehouse_completion_published = False
            # +++ 修改：等待自动模式初始化完成后再发送层指令 +++
            if self.auto_mode_initialized:
                # 轴已就绪，立即发送层指令
                self.send_layer_command(self.target_layer)
                self.get_logger().info(f'从LIFT_MOVING恢复，重新发送层指令到目标层 {self.target_layer}')
            else:
                # 轴未就绪，保存状态等待初始化完成
                self.pending_resume_state = {
                    'type': 'warehouse',
                    'state': WarehouseState.LIFT_MOVING,
                    'layer': self.target_layer
                }
                self.get_logger().info(f'从LIFT_MOVING恢复，等待轴自动模式初始化完成后发送层指令到目标层 {self.target_layer}')
        elif state_value == WarehouseState.POST_LIFT_PROCESSING.value:
            # 从层移动后处理恢复
            self.warehouse_state = WarehouseState.LIFT_MOVING
            self.warehouse_process_requested = True
            self.layer_motion_completed = True  # 假设层移动已完成
            self.post_lift_delay_start = None
            # 重置完成发布标志，确保后续能正确回包102
            self.warehouse_completion_published = False
            self.get_logger().info('从POST_LIFT_PROCESSING恢复，将重新执行后续操作，重置完成发布标志')
        elif state_value == WarehouseState.DELAY_PROCESSING.value:
            # 从延迟处理恢复
            self.warehouse_state = WarehouseState.POST_LIFT_PROCESSING
            self.warehouse_process_requested = True
            self.layer_motion_completed = True
            self.post_lift_delay_start = None
            self.delay_started = False
            self.delay_condition_triggered = False
            self.buffer_sensor_2_detected = False  # 重置缓存架对射2检测状态
            # 重置DO发送状态，确保812和811能重新发送
            self.reset_do_command_state("811")
            self.reset_do_command_state("812")
            # 重置完成发布标志，确保后续能正确回包102
            self.warehouse_completion_published = False
            self.get_logger().info('从DELAY_PROCESSING恢复，将重新执行延迟处理，重置完成发布标志')
        elif state_value == WarehouseState.COMPLETED.value:
            self.warehouse_state = WarehouseState.COMPLETED
            self.warehouse_process_requested = True
            # 关键修复：重置完成发布标志，确保恢复后能正确发布完成消息
            self.warehouse_completion_published = False
            # 关键修复：重置层移动状态，确保能正确发送回第1层指令
            self.layer_motion_completed = False
            self.previous_layer_completion_state = False
            self.layer_completion_received_time = None

            # +++ 新增：发送回到第1层的指令 +++
            if self.auto_mode_initialized:
                # 轴已就绪，立即发送回到第1层的指令
                self.send_layer_command(1)
                self.get_logger().info('从COMPLETED恢复，发送层指令回到第1层')
            else:
                # 轴未就绪，保存状态等待初始化完成
                self.pending_resume_state = {
                    'type': 'warehouse_completed',
                    'layer': 1
                }
                self.get_logger().info('从COMPLETED恢复，等待轴自动模式初始化完成后发送层指令回到第1层')
            self.get_logger().info('从COMPLETED恢复，重置完成发布标志和层移动状态，确保能正确回包102并回到第1层')
        
        self.resuming_from_pause = False
    
    def restore_outbound_state(self, state_value, source_layer):
        """恢复出库流程到指定状态"""
        self.get_logger().info(f'恢复出库流程: 状态={state_value}, 源层={source_layer}')
        
        # 重置层指令发送状态，确保恢复时可以重新发送层指令
        self.layer_command_sent = False
        self.last_layer_command = None
        
        # 设置源层
        self.source_layer = source_layer
        
        # 根据状态值恢复
        if state_value == OutboundState.IDLE.value:
            self.outbound_state = OutboundState.IDLE
            # +++ 新增：IDLE状态恢复时，如果不在第1层，发送回到第1层的指令 +++
            if not self.is_target_layer_reached(1):
                if self.auto_mode_initialized:
                    self.send_layer_command(1)
                    self.get_logger().info(f'出库从IDLE恢复，当前层={self.current_layer_float:.2f}，发送层指令回到第1层')
                else:
                    self.pending_resume_state = {
                        'type': 'outbound_idle',
                        'layer': 1
                    }
                    self.get_logger().info(f'出库从IDLE恢复，等待轴自动模式初始化完成后发送层指令回到第1层')
            else:
                self.get_logger().info(f'出库从IDLE恢复，当前已在第1层({self.current_layer_float:.2f})，无需移动')
        elif state_value == OutboundState.WAIT_FOR_EXIT.value:
            self.outbound_state = OutboundState.WAIT_FOR_EXIT
            self.outbound_process_requested = True
            # 重置完成发布标志，确保后续能正确回包104
            self.outbound_completion_published = False
            self.get_logger().info('从WAIT_FOR_EXIT恢复，重置完成发布标志')
        elif state_value == OutboundState.LIFT_MOVING.value:
            # 从提升机运行状态恢复 - 保持在LIFT_MOVING并重新发送层指令
            self.outbound_state = OutboundState.LIFT_MOVING
            self.outbound_process_requested = True
            # 重置层移动状态（关键：必须同时重置previous_layer_completion_state）
            self.layer_motion_completed = False
            self.previous_layer_completion_state = False
            self.layer_completion_received_time = None
            # 重置完成发布标志，确保后续能正确回包104
            self.outbound_completion_published = False
            # +++ 修改：等待自动模式初始化完成后再发送层指令 +++
            if self.auto_mode_initialized:
                # 轴已就绪，立即发送层指令
                self.send_layer_command(self.source_layer)
                self.get_logger().info(f'从LIFT_MOVING恢复，重新发送层指令到源层 {self.source_layer}')
            else:
                # 轴未就绪，保存状态等待初始化完成
                self.pending_resume_state = {
                    'type': 'outbound',
                    'state': OutboundState.LIFT_MOVING,
                    'layer': self.source_layer
                }
                self.get_logger().info(f'从LIFT_MOVING恢复，等待轴自动模式初始化完成后发送层指令到源层 {self.source_layer}')
        elif state_value == OutboundState.POST_LIFT_PROCESSING.value:
            # 从层移动后处理恢复
            self.outbound_process_requested = True
            # 重置DO发送状态，确保811能重新发送
            self.reset_do_command_state("811")
            # 重置完成发布标志，确保后续能正确回包104
            self.outbound_completion_published = False
            # 等待自动模式初始化完成后再进入POST_LIFT_PROCESSING（避免JOG命令在轴STOPPED状态丢失）
            if not self.auto_mode_initialized:
                # 暂时保持LIFT_MOVING，等待轴就绪
                self.outbound_state = OutboundState.LIFT_MOVING
                self.pending_resume_state = {
                    'type': 'outbound_post_lift',
                    'source_layer': source_layer
                }
                self.get_logger().info('从POST_LIFT_PROCESSING恢复，等待轴自动模式初始化完成后执行')
            else:
                self.outbound_state = OutboundState.LIFT_MOVING
                self.layer_motion_completed = True  # 轴已就绪，立即进入POST_LIFT_PROCESSING
                self.get_logger().info('从POST_LIFT_PROCESSING恢复，轴已就绪，将重新执行后续操作')
        elif state_value == OutboundState.CONVEYOR_MOVING.value:
            # 从输送带运行状态恢复 - 需要重新发送层指令到第1层
            self.outbound_state = OutboundState.CONVEYOR_MOVING
            self.outbound_process_requested = True
            # 重置层移动状态（关键：必须同时重置previous_layer_completion_state）
            self.layer_motion_completed = False
            self.previous_layer_completion_state = False
            self.layer_completion_received_time = None
            # 重置完成发布标志，确保后续能正确回包104
            self.outbound_completion_published = False
            # 等待自动模式初始化完成后发送层指令
            if self.auto_mode_initialized:
                self.send_layer_command(1)
                self.get_logger().info('从CONVEYOR_MOVING恢复，立即发送层指令到第1层')
            else:
                self.pending_resume_state = {
                    'type': 'outbound_conveyor',
                    'state': OutboundState.CONVEYOR_MOVING,
                    'layer': 1
                }
                self.get_logger().info('从CONVEYOR_MOVING恢复，等待轴自动模式初始化完成后发送层指令到第1层')
        elif state_value == OutboundState.COMPLETED.value:
            self.outbound_state = OutboundState.COMPLETED
            self.outbound_process_requested = True
            # 关键修复：重置完成发布标志，确保恢复后能正确发布完成消息
            self.outbound_completion_published = False
            # 关键修复：重置层移动状态，确保能正确发送回第1层指令
            self.layer_motion_completed = False
            self.previous_layer_completion_state = False
            self.layer_completion_received_time = None
            self.get_logger().info('从COMPLETED恢复，重置完成发布标志和层移动状态，确保能正确回包104并回到第1层')
        
        self.resuming_from_pause = False

    def process_logic(self):
        """主处理逻辑 - 定时器回调"""
        if not self.auto_mode_enabled or not self.enabled:
            return
        
        # +++ 关键修复：先处理产品到位检测逻辑，再更新产品到位信号 +++
        # 这样可以确保在判断是否可以要板时，product_arrival_published_in_cycle已经被正确设置
        self.process_product_arrival_logic()
        
        # 处理SMEMA协议（仅在启用时）
        if self.ENABLE_SMEMA:
            self.update_product_position()  # 更新产品到位信号
            self.check_smema_handshake()    # 检查握手状态
        
        # 处理入库逻辑
        self.process_warehouse_logic()
        
        # 处理出库逻辑
        self.process_outbound_logic()
        
        # 处理放行逻辑
        self.process_release_logic()
        
        # 处理IO信号变化
        self.process_io_signals()

        # 执行待处理命令
        self.execute_pending_commands()

        # 检查流程超时
        self.check_process_timeout()

        # 检查齿轮对接气缸到位状态
        self.check_gear_cylinder_position()

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
                    f'conveyor_out={self.current_io_signals["conveyor_out_position"]}, '
                    f'buffer_sensor_2={self.current_io_signals["buffer_sensor_2"]}'
                )
        
        # 更新前一个状态
        self.previous_io_signals = self.current_io_signals.copy()

    def check_process_timeout(self):
        """检查入库/出库流程是否超时，超时时发布故障码（不停止流程）"""
        current_time = time.time()

        # 检查入库流程超时
        if (self.warehouse_state != WarehouseState.IDLE and
            self.warehouse_process_start_time is not None and
            not self.warehouse_timeout_reported):

            elapsed = current_time - self.warehouse_process_start_time
            if elapsed > self.WAREHOUSE_PROCESS_TIMEOUT:
                fault_code = FaultCode.WAREHOUSE_TIMEOUT.value
                self.get_logger().error(
                    f'入库流程超时({elapsed:.1f}秒>{self.WAREHOUSE_PROCESS_TIMEOUT}秒)，'
                    f'发布故障码=0x{fault_code:04X}'
                )
                self.publish_fault_code(fault_code)
                self.warehouse_timeout_reported = True

        # 检查出库流程超时
        if (self.outbound_state != OutboundState.IDLE and
            self.outbound_process_start_time is not None and
            not self.outbound_timeout_reported):

            elapsed = current_time - self.outbound_process_start_time
            if elapsed > self.OUTBOUND_PROCESS_TIMEOUT:
                fault_code = FaultCode.OUTBOUND_TIMEOUT.value
                self.get_logger().error(
                    f'出库流程超时({elapsed:.1f}秒>{self.OUTBOUND_PROCESS_TIMEOUT}秒)，'
                    f'发布故障码=0x{fault_code:04X}'
                )
                self.publish_fault_code(fault_code)
                self.outbound_timeout_reported = True

    def publish_fault_code(self, fault_code: int):
        """发布故障码到 /fault_code 话题（相同故障码只发布一次）"""
        if fault_code == self.last_published_fault_code:
            return  # 避免重复发布相同故障码

        self.last_published_fault_code = fault_code

        # 构建故障码字符串（格式与 ethercat_node 一致）
        msg = String()
        if fault_code == 0:
            msg.data = "0"
        else:
            # 格式: "business_logic:0xXXXX"
            msg.data = f'business_logic:0x{fault_code:04X}'

        self.fault_code_pub.publish(msg)
        self.get_logger().warn(f'发布故障码到/fault_code: {msg.data}')

    def check_gear_cylinder_position(self):
        """检查811齿轮对接气缸伸出到位状态（M533）"""
        if not self.gear_cylinder_811_checking:
            return

        di = self.current_io_signals
        gear_cylinder_in_position = di.get('gear_cylinder_2_in_position', False)

        # 检测到位信号
        if gear_cylinder_in_position:
            self.gear_cylinder_811_checking = False
            self.gear_cylinder_811_sent_time = None
            self.get_logger().info('✅ M533齿轮对接气缸2伸出到位检测通过')
            return

        # 检查超时
        if self.gear_cylinder_811_sent_time is not None:
            elapsed = time.time() - self.gear_cylinder_811_sent_time
            if elapsed > self.GEAR_CYLINDER_TIMEOUT_SEC:
                if not self.gear_cylinder_811_timeout_reported:
                    fault_code = FaultCode.GEAR_CYLINDER_TIMEOUT.value
                    self.get_logger().error(
                        f'❌ M533齿轮对接气缸2伸出到位超时({elapsed:.1f}秒>{self.GEAR_CYLINDER_TIMEOUT_SEC}秒)，'
                        f'发布故障码=0x{fault_code:04X}'
                    )
                    self.publish_fault_code(fault_code)
                    self.gear_cylinder_811_timeout_reported = True

    def reset_gear_cylinder_check(self):
        """重置齿轮对接气缸检测状态"""
        self.gear_cylinder_811_checking = False
        self.gear_cylinder_811_sent_time = None
        self.gear_cylinder_811_timeout_reported = False
        self.get_logger().debug('重置齿轮对接气缸检测状态')

    def reset_process_timeout(self, process_type: str):
        """重置流程超时状态（流程开始或结束时调用）
        Args:
            process_type: 'warehouse' 或 'outbound'
        """
        if process_type == 'warehouse':
            self.warehouse_process_start_time = time.time()
            self.warehouse_timeout_reported = False
            self.get_logger().debug('重置入库流程超时计时器')
        elif process_type == 'outbound':
            self.outbound_process_start_time = time.time()
            self.outbound_timeout_reported = False
            self.get_logger().debug('重置出库流程超时计时器')

    def clear_process_timeout(self, process_type: str):
        """清除流程超时状态（流程正常完成时调用）
        Args:
            process_type: 'warehouse' 或 'outbound'
        """
        if process_type == 'warehouse':
            self.warehouse_process_start_time = None
            self.warehouse_timeout_reported = False
            # 如果之前有超时故障，清除故障码
            if self.last_published_fault_code == FaultCode.WAREHOUSE_TIMEOUT.value:
                self.publish_fault_code(0)
                self.last_published_fault_code = 0
        elif process_type == 'outbound':
            self.outbound_process_start_time = None
            self.outbound_timeout_reported = False
            # 如果之前有超时故障，清除故障码
            if self.last_published_fault_code == FaultCode.OUTBOUND_TIMEOUT.value:
                self.publish_fault_code(0)
                self.last_published_fault_code = 0

    def process_warehouse_logic(self):
        """处理入库业务流程"""
        di = self.current_io_signals
        buffer_in = di['buffer_in_position']
        buffer_out = di['buffer_out_position']
        conveyor_in = di['conveyor_in_position']
        conveyor_out = di['conveyor_out_position']
        buffer_sensor_2 = di['buffer_sensor_2']  # 缓存架对射2信号 (M522)

        # 检查状态是否变化
        state_changed = (self.warehouse_state != self.previous_warehouse_state)
        
        # 只在状态变化时打印详细信息
        if state_changed:
            self.get_logger().info(
                f'入库流程状态变化: {self.previous_warehouse_state.name} -> {self.warehouse_state.name}, '
                f'当前层: {self.current_layer_float:.2f}, 目标层: {self.target_layer}'
            )
            self.previous_warehouse_state = self.warehouse_state
        
        # === 产品到位发布逻辑（0x0109）===
        # 已由 process_product_arrival_logic 状态机统一处理
        # 状态机流程：feed_detect → conveyor → buffer_in上升 → buffer_in下降 → 发布0x0109
        # 此处不再重复调用旧方法
    
        # 处理停止请求
        if self.warehouse_process_stop_requested:
            self.warehouse_state = WarehouseState.IDLE
            self.warehouse_process_stop_requested = False
            self.warehouse_process_requested = False
            self.get_logger().info('入库流程已停止')
            return

        if self.warehouse_state == WarehouseState.IDLE:
            # 等待启动信号
            # 关键修复：产品到位检测完成后，产品在缓存架出料口（buffer_out=True），这是正常的
            # 只需要检查产品到位检测完成标志，不需要检查buffer_out
            if (self.warehouse_process_requested and 
                not conveyor_in and not conveyor_out):
                self._reset_key_do_signals()  # 重置关键DO信号，确保安全状态
                self.warehouse_state = WarehouseState.WAIT_FOR_ENTRY
                self.warehouse_process_requested = False
                # 更新产品到位发布阶段为"入库中"（入库前检测到到位仍可发布）
                if self.product_arrival_cycle_active and self.product_arrival_phase == "pre_warehouse":
                    self.product_arrival_phase = "warehouse"
                    self.get_logger().info('入库流程启动，更新产品到位发布阶段为: warehouse')
                # 重置入库流程超时计时器
                self.reset_process_timeout('warehouse')
                self.get_logger().info(f'入库流程启动，进入等待入库状态，目标层: {self.target_layer}')

        elif self.warehouse_state == WarehouseState.WAIT_FOR_ENTRY:
            # 检测入库条件：等待产品到位检测状态机完成（product_arrival_published_in_cycle=True）
            # 且 buffer_in 有产品（产品到位后buffer_in应该为True）
            if self.product_arrival_published_in_cycle :
                # +++ 新增检查：确保当前层是第1层（使用从axis5位置计算的层号）+++
                if not self.is_target_layer_reached(1):
                    self.get_logger().warn(f'提升机不在第1层（当前层={self.current_layer_float:.2f}），等待回到第1层后再开始入库')
                    # 发送回到第1层的指令
                    self.send_layer_command(1)
                    return
                self.warehouse_state = WarehouseState.CONVEYOR_MOVING
                self.get_logger().info(f'检测到入库条件，当前层={self.current_layer_float:.2f}，开始输送')
                
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
            # 条件1：conveyor_out为True（直接检测到出料/接驳台入料位）
            # 条件2：conveyor_in曾经为True后又变为False（检测到货物进入后离开）
            conveyor_out_detected = conveyor_out
            conveyor_in_then_out = (self.conveyor_in_detected and not conveyor_in)
            
            # 新增：条件二触发时的延迟处理
            if conveyor_in_then_out and not self.conveyor_in_then_out_delay_started:
                self.conveyor_in_then_out_delay_started = True
                self.conveyor_in_then_out_delay_counter = 0
                self.get_logger().info('检测到条件二（conveyor_in变化），开始0.2秒延迟')
            
            # 处理条件二的延迟
            if self.conveyor_in_then_out_delay_started:
                self.conveyor_in_then_out_delay_counter += 1
                
                # 0.1秒延迟（1个周期，每周期100ms）
                if self.conveyor_in_then_out_delay_counter >= 1:
                    board_in_position = True
                    self.conveyor_in_then_out_delay_started = False
                    self.get_logger().info('条件二延迟结束，认为板子到位')
                else:
                    board_in_position = False
            else:
                # 条件一(conveyor_out信号)立即触发
                board_in_position = conveyor_out_detected
            
            # 继续输送直到检测到板子到位
            if board_in_position:
                # 记录是哪个条件触发的到位检测
                trigger_condition = []
                if conveyor_out_detected:
                    trigger_condition.append('conveyor_out')
                if conveyor_in_then_out:
                    trigger_condition.append('conveyor_in变化')
                self.get_logger().info(f'检测到板子到位: 触发条件={", ".join(trigger_condition)}')
                
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
                # === 边缘触发：只在进入等待状态时打印一次 ===
                if not getattr(self, '_waiting_layer_motion_printed', False):
                    self.get_logger().info(f'等待层移动完成... 目标层={self.target_layer}, 当前层={self.current_layer_float:.2f}')
                    self._waiting_layer_motion_printed = True
                return  # 继续等待
            
            # 层移动完成后执行后续操作
            self.get_logger().info(f'层移动完成，当前层={self.current_layer_float:.2f}，继续执行入库流程')
            # 重置层移动相关状态
            self.layer_motion_completed = False
            self.previous_layer_completion_state = False
            self.layer_completion_received_time = None
            self._waiting_layer_motion_printed = False  # 重置边缘打印标志
            
            # 重要修改：立即转换到新的状态，避免重新进入等待
            self.post_lift_delay_start = None  # 重置延迟计时器
            self.warehouse_state = WarehouseState.POST_LIFT_PROCESSING
            self.get_logger().info('进入层移动后处理状态')

        elif self.warehouse_state == WarehouseState.POST_LIFT_PROCESSING:
            """新增：层移动后的处理状态，避免状态循环"""
            # === 分步骤执行，轴2_2正转和DO811之间有1秒延迟 ===
            
            # 步骤1：首次进入状态，执行前半部分操作
            if self.post_lift_delay_start is None:
                self.get_logger().info('入库流程：层移动完成，继续执行后续操作')
                
                # 执行前半部分操作
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
                
                # 记录延迟开始时间
                self.post_lift_delay_start = time.time()
                self.get_logger().info('等待1秒后激活DO气缸...')
                return  # 继续等待
            
            # 步骤2：检查延迟是否到达1秒
            elapsed = time.time() - self.post_lift_delay_start
            if elapsed < 0.5:
                return  # 继续等待
            
            # 步骤3：延迟到达，执行后续操作
            self.send_do_control_once("811", True)  # 激活DO气缸伸出信号
            self.get_logger().info(f'DO811气缸伸出已激活，延迟{elapsed:.1f}秒')

            # +++ 新增：启动齿轮对接气缸到位检测 +++
            self.gear_cylinder_811_sent_time = time.time()
            self.gear_cylinder_811_checking = True
            self.gear_cylinder_811_timeout_reported = False
            self.get_logger().info('启动M533齿轮对接气缸2伸出到位检测...')

            # 重置延迟计时器
            self.post_lift_delay_start = None
            
            # 转换到下一个状态
            self.warehouse_state = WarehouseState.DELAY_PROCESSING
            self.get_logger().info('进入延迟处理状态')

        elif self.warehouse_state == WarehouseState.DELAY_PROCESSING:
                # 检测信号变化
                # 修改逻辑：进入检测用buffer_sensor_2=1，离开检测用conveyor_in=0且buffer_sensor_2=0
                if not self.delay_started and not self.delay_condition_triggered:
                    if buffer_sensor_2:
                        # 检测到货物进入（缓存架对射2被触发）
                        if not self.buffer_sensor_2_detected:
                            self.buffer_sensor_2_detected = True
                            self.get_logger().info('检测到货物进入缓存架(buffer_sensor_2=1)')
                    elif self.buffer_sensor_2_detected and not conveyor_in and not buffer_sensor_2:
                        # 货物离开条件：buffer_sensor_2曾经触发过，且当前conveyor_in=0且buffer_sensor_2=0
                        self.delay_condition_triggered = True
                        self.delay_started = True
                        self.delay_counter = 0
                        self.buffer_sensor_2_detected = False
                        self.conveyor_in_detected = False
                        self.get_logger().info(f'检测到货物离开接驳台(conveyor_in=0,buffer_sensor_2=0)，开始{self.DELAY_BEFORE_STOP_MS//1000}秒延迟')
                
                # 处理延迟逻辑
                if self.delay_started:
                    self.delay_counter += 1
                    
                    if self.delay_counter >= self.DELAY_COUNTER_MAX:
                        # 延迟结束
                        self.warehouse_state = WarehouseState.COMPLETED
                        self.delay_started = False
                        self.delay_condition_triggered = False
                        self.buffer_sensor_2_detected = False  # 重置缓存架对射2检测状态
                        self.state_change_counter = 0  # 重置计数器，用于COMPLETED状态等待层移动
                        
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
                        # 延迟中，每3秒记录一次
                        if self.delay_counter % 30 == 0:
                            remaining_seconds = self.DELAY_BEFORE_STOP_MS//1000 - self.delay_counter//10
                            self.get_logger().info(f'延迟剩余时间: {remaining_seconds}秒')

        elif self.warehouse_state == WarehouseState.COMPLETED:
            # 1. 发送升降机复位指令（回到第1层）
            # 注意：send_layer_command 内部会检查重复发送，相同指令不会重复发送
            self.send_layer_command(1)
            
            # 修正日志：明确区分目标层和当前实际层
            # current_layer_float 是异步更新的，可能还未反映最新的层移动
            # 使用标志确保日志只打印一次
            if not self.completed_reset_command_sent:
                self.get_logger().info(f'流程完成，发送升降机复位指令至第1层 (目标层=1, 当前实际层={self.current_layer_float:.2f})')
                self.completed_reset_command_sent = True

            # 2. 发布入库完成消息 (会触发0x102回包)
            if not self.warehouse_completion_published:
                completion_msg = Bool()
                completion_msg.data = True
                self.warehouse_completed_pub.publish(completion_msg)
                self.warehouse_completion_published = True
                self.get_logger().info('入库流程完成，发布完成消息(0x102)')

            # 3. 重置产品到位发布状态，确保能检测下一轮产品
            if self.product_arrival_cycle_active:
                self.product_arrival_published_in_cycle = False
                self.product_arrival_phase = "post_warehouse"
                # 重置产品到位检测状态机，准备下一轮检测
                self._reset_product_arrival_state_machine()

            # +++ 关键修复：等待层移动完成（到达第1层）后再进入IDLE +++
            if not self.is_target_layer_reached(1):
                # 层移动未完成，继续等待
                self.state_change_counter += 1
                if self.state_change_counter >= 50:  # 每5秒打印一次
                    self.state_change_counter = 0
                    self.get_logger().info(f'等待升降机回到第1层... 当前层={self.current_layer_float:.2f}')
                return  # 不进入IDLE，继续等待层移动完成

            # 4. 层移动完成，重置流程状态，回到IDLE
            self.warehouse_process_requested = False
            self.warehouse_completion_published = False
            self.completed_reset_command_sent = False  # 重置标志，为下次流程做准备
            self.warehouse_state = WarehouseState.IDLE
            # 清除入库流程超时状态
            self.clear_process_timeout('warehouse')
            self.get_logger().info(f'升降机已回到第1层(当前层={self.current_layer_float:.2f})，流程状态重置为IDLE')

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
                f'当前层: {self.current_layer_float:.2f}, 源层: {self.source_layer}'
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

            # 等待启动信号 - 出库流程只需要收到请求即可启动，不检查IO条件
            # （货物在缓存架上是正常情况，不需要等待所有IO为False）
            if self.outbound_process_requested:
                self._reset_key_do_signals()
                self.outbound_state = OutboundState.WAIT_FOR_EXIT
                self.outbound_process_requested = False
                # 重置出库流程超时计时器
                self.reset_process_timeout('outbound')
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
                # === 边缘触发：只在进入等待状态时打印一次 ===
                if not getattr(self, '_outbound_waiting_layer_motion_printed', False):
                    self.get_logger().info(f'等待层移动完成... 源层={self.source_layer}, 当前层={self.current_layer_float:.2f}')
                    self._outbound_waiting_layer_motion_printed = True
                return  # 继续等待
            
            # 层移动完成后执行后续操作
            self.get_logger().info(f'层移动完成，当前层={self.current_layer_float:.2f}，继续执行出库流程')
            # 重置层移动相关状态
            self.layer_motion_completed = False
            self.previous_layer_completion_state = False
            self.layer_completion_received_time = None
            self._outbound_waiting_layer_motion_printed = False  # 重置边缘打印标志
            
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
            self.send_do_control_once("811", True)  # 激活DO811齿轮对接气缸伸出
            self.get_logger().info('出库：DO811齿轮对接气缸伸出已激活')

            # +++ 新增：启动齿轮对接气缸到位检测 +++
            self.gear_cylinder_811_sent_time = time.time()
            self.gear_cylinder_811_checking = True
            self.gear_cylinder_811_timeout_reported = False
            self.get_logger().info('启动M533齿轮对接气缸2伸出到位检测...')

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
                if self.outbound_conveyor_in_then_out_delay_counter >= 1:
                    board_in_position = True
                    self.outbound_conveyor_in_then_out_delay_started = False
                    self.get_logger().info('出库条件二延迟结束，认为板子到位')
                else:
                    board_in_position = False
            else:
                # 条件一立即触发
                board_in_position = conveyor_out_detected
            
            if board_in_position: # 检测板子到位条件
                # 记录是哪个条件触发的到位检测
                trigger_condition = []
                if conveyor_out_detected:
                    trigger_condition.append('conveyor_out')
                if conveyor_in_then_out:
                    trigger_condition.append('conveyor_in变化')
                self.get_logger().info(f'出库检测到板子到位: 触发条件={", ".join(trigger_condition)}')
                
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
            """输送带运行状态 - 根据出库区域调整层号
            
            出库区域=0: 正常出库到第1层
            出库区域=1: 特殊出库到第18层，完成后回到第1层
            """
            # 1. 根据出库区域确定目标层
            target_layer = 18 if self.outbound_area == 1 else 1
            
            # 2. 发送层指令，等待提升机到达
            self.send_layer_command(target_layer)
            
            # 3. 等待提升机到达目标层
            if not self.layer_motion_completed:
                # 每5秒打印一次等待状态
                self.state_change_counter += 1
                if self.state_change_counter >= 50:  # 5秒打印一次
                    self.state_change_counter = 0
                    self.get_logger().info(f'等待提升机到达第{target_layer}层...')
                return  # 继续等待，不执行后续逻辑
            
            # 4. 提升机到达目标层后，启动输送带
            if not hasattr(self, 'outbound_conveyor_started') or not self.outbound_conveyor_started:
                self.send_do_control_once("813", True)  # 启动DO14皮带正转
                self.outbound_conveyor_started = True
                self.get_logger().info(f'提升机已到达第{target_layer}层，启动输送带')
                # 重置层移动完成标志，为下一次使用做准备
                self.layer_motion_completed = False
                self.outbound_state = OutboundState.COMPLETED

        elif self.outbound_state == OutboundState.COMPLETED:
            # 4. 检测货物完全送出（conveyor_out从True变为False后再延迟）
            if conveyor_out:
                # 货物还在出料位，记录检测状态
                if not getattr(self, '_outbound_conveyor_out_was_true', False):
                    self._outbound_conveyor_out_was_true = True
                    self.get_logger().info('检测到货物到达出料位，等待货物离开...')
            else:
                # 货物曾经到达过出料位，现在又离开了
                if getattr(self, '_outbound_conveyor_out_was_true', False) and not self.outbound_delay_started:
                    self.outbound_delay_started = True
                    self.outbound_delay_counter = 0
                    self.get_logger().info(f'货物已离开出料位，开始{self.OUTBOUND_DELAY_BEFORE_STOP_MS//1000}秒延迟')
            
            # 5. 延迟处理（货物离开后延迟）
            if self.outbound_delay_started:
                self.outbound_delay_counter += 1
                
                if self.outbound_delay_counter >= self.OUTBOUND_DELAY_COUNTER_MAX:
                    # 停止输送带并完成流程
                    self.send_do_control_once("813", False)
                    self.outbound_delay_started = False
                    self._outbound_conveyor_out_was_true = False  # 重置检测标志
                    self.outbound_state = OutboundState.IDLE
                    self.outbound_process_requested = False  # 关键：重置启动请求标志
                    # 清除出库流程超时状态
                    self.clear_process_timeout('outbound')
                    # 发布出库完成消息（只在状态变化时发布一次）
                    if not self.outbound_completion_published:
                        completion_msg = Bool()
                        completion_msg.data = True
                        self.outbound_completed_pub.publish(completion_msg)
                        self.outbound_completion_published = True
                        self.get_logger().info('出库流程完成，发布完成消息')
                    # 重置所有相关标志
                    if hasattr(self, 'outbound_conveyor_started'):
                        self.outbound_conveyor_started = False
                    self.get_logger().info('出库流程完成')
                    
                    # 关键：如果出库区域为1，出库完成后回到第1层
                    if self.outbound_area == 1:
                        self.send_layer_command(1)
                        self.get_logger().info('出库区域=1，出库完成后发送回到第1层指令')
                        self.outbound_area = 0  # 重置出库区域
                    
                    # +++ 修复：出库流程不应该清除产品到位标志 +++
                    # 产品到位标志应该只由产品到位检测状态机管理
                    # 出库流程完成后，产品可能还在缓存架上，保留标志供后续入库使用
                    # 如果确实需要清除，应该由产品到位检测状态机在检测到产品离开后自动清除
    
    def process_release_logic(self):
        """处理放行业务流程
        
        放行逻辑：
        1. 收到放行启动信号 → 进入等待产品到位状态
        2. 检测 buffer_in=True → 发布产品到位信号
        3. 等待放行请求信号 → 启动输送带（axis1_1反转，axis1_2正转，DO813激活）
        4. 检测停止条件：
           - 收到放行停止信号
           - conveyor_out信号从无到有再到无后，发布放行完成
        """
        di = self.current_io_signals
        buffer_in = di['buffer_in_position']
        conveyor_in = di['conveyor_in_position']
        conveyor_out = di['conveyor_out_position']
        
        # 检查状态是否变化
        state_changed = (self.release_state != self.previous_release_state)
        
        # 只在状态变化时打印
        if state_changed:
            self.get_logger().info(
                f'放行流程状态变化: {self.previous_release_state.name} -> {self.release_state.name}'
            )
            self.previous_release_state = self.release_state
        
        # 处理停止请求
        if self.release_process_stop_requested:
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
            
            self.release_state = PassThroughState.IDLE
            self.release_process_stop_requested = False
            self.release_process_requested = False
            self.get_logger().info('放行流程已停止')
            return
        
        if self.release_state == PassThroughState.IDLE:
            # 等待启动信号
            if self.release_process_requested:
                self._reset_key_do_signals()  # 重置关键DO信号，确保安全状态
                self.release_process_requested = False
                
                # 检查是否在第一层
                if not self.is_target_layer_reached(1):
                    # 不在第一层，发送命令回到第一层
                    if self.auto_mode_initialized:
                        self.send_layer_command(1)
                        self.release_state = PassThroughState.RETURNING_TO_LAYER_1
                        self.get_logger().info(f'放行流程启动，当前层={self.current_layer_float:.2f}，发送层指令回到第1层')
                    else:
                        # 等待轴自动模式初始化
                        self.pending_resume_state = {
                            'type': 'release_return_to_layer_1'
                        }
                        self.release_state = PassThroughState.RETURNING_TO_LAYER_1
                        self.get_logger().info('放行流程启动，等待轴自动模式初始化完成后发送层指令回到第1层')
                else:
                    # 已在第一层，直接进入等待产品到位状态
                    self.release_state = PassThroughState.WAIT_FOR_PRODUCT_ARRIVAL
                    self.get_logger().info(f'放行流程启动，当前已在第1层({self.current_layer_float:.2f})，进入等待产品到位状态')
        
        elif self.release_state == PassThroughState.RETURNING_TO_LAYER_1:
            # 等待接驳台回到第一层
            if self.is_target_layer_reached(1):
                # 已到达第一层，进入等待产品到位状态
                self.release_state = PassThroughState.WAIT_FOR_PRODUCT_ARRIVAL
                self.get_logger().info(f'接驳台已回到第1层({self.current_layer_float:.2f})，进入等待产品到位状态')
        
        elif self.release_state == PassThroughState.WAIT_FOR_PRODUCT_ARRIVAL:
            # 等待产品到位检测状态机完成（由 process_product_arrival_logic 处理）
            # 当 product_arrival_published_in_cycle = True 时表示产品已到位
            if self.product_arrival_published_in_cycle:
                # 产品到位后启动输送带（放行）
                self.add_command(ControlAction(
                    CommandType.JOG, "axis1_1", "reverse",
                    description="启动轴1_1反转（放行）"
                ))
                self.add_command(ControlAction(
                    CommandType.JOG, "axis1_2", "forward",
                    description="启动轴1_2正转（放行）"
                ))
                self.send_do_control_once("813", True)  # 激活DO14皮带反转
                
                self.release_state = PassThroughState.CONVEYOR_RUNNING
                self.get_logger().info('产品到位检测完成，输送带已启动（放行）')
        
        elif self.release_state == PassThroughState.CONVEYOR_RUNNING:
            """输送带运行状态 - 顺序检测conveyor_in和conveyor_out
            
            检测顺序：
            1. conveyor_in: 无 → 有 → 无（货物进入输送带）
            2. conveyor_out: 无 → 有 → 无（货物离开输送带）
            两个都完成后，才进入WAIT_FOR_CONVEYOR_OUT状态
            """
            # 步骤1：检测 conveyor_in 信号变化（无 → 有 → 无）
            if not self.release_conveyor_in_completed:
                if conveyor_in:
                    # conveyor_in 从无到有
                    if not self.release_conveyor_in_was_true:
                        self.release_conveyor_in_was_true = True
                        self.get_logger().info('放行流程：检测到货物进入输送带(conveyor_in=1)')
                else:
                    # conveyor_in 从有到无
                    if self.release_conveyor_in_was_true:
                        self.release_conveyor_in_completed = True
                        self.get_logger().info('放行流程：货物已完全进入输送带(conveyor_in=0)，开始检测conveyor_out')
            
            # 步骤2：conveyor_in完成后，检测 conveyor_out 信号变化（无 → 有 → 无）
            if self.release_conveyor_in_completed:
                if conveyor_out:
                    # conveyor_out 从无到有
                    if not self.release_conveyor_out_was_true:
                        self.release_conveyor_out_was_true = True
                        self.get_logger().info('放行流程：检测到货物到达出料位(conveyor_out=1)')
                else:
                    # conveyor_out 从有到无
                    if self.release_conveyor_out_was_true:
                        self.get_logger().info('放行流程：检测到货物离开出料位(conveyor_out=0)，开始延迟')
                        self.release_conveyor_out_delay_started = True
                        self.release_conveyor_out_delay_counter = 0
                        self.release_state = PassThroughState.WAIT_FOR_CONVEYOR_OUT
        
        elif self.release_state == PassThroughState.WAIT_FOR_CONVEYOR_OUT:
            # 延迟处理（货物离开后延迟0.3秒）
            if self.release_conveyor_out_delay_started:
                self.release_conveyor_out_delay_counter += 1
                
                # 0.3秒延迟（3个周期，每周期100ms）
                if self.release_conveyor_out_delay_counter >= 3:
                    # 停止输送带并完成流程
                    self.add_command(ControlAction(
                        CommandType.JOG, "axis1_1", "stop",
                        description="停止轴1_1"
                    ))
                    self.add_command(ControlAction(
                        CommandType.JOG, "axis1_2", "stop",
                        description="停止轴1_2"
                    ))
                    self.send_do_control_once("813", False)  # 停止DO14皮带反转
                    
                    # 重置所有检测状态
                    self.release_conveyor_out_delay_started = False
                    self.release_conveyor_out_was_true = False
                    self.release_conveyor_in_was_true = False
                    self.release_conveyor_in_completed = False
                    self.release_state = PassThroughState.COMPLETED
                    self.get_logger().info('放行流程：延迟结束，进入完成状态')
        
        elif self.release_state == PassThroughState.COMPLETED:
            # 发布放行完成消息
            if not self.release_completion_published:
                completion_msg = Bool()
                completion_msg.data = True
                self.release_completed_pub.publish(completion_msg)
                self.release_completion_published = True
                self.get_logger().info('✅ 放行流程完成，发布完成消息')
            
            # 重置流程状态，回到IDLE
            self.release_process_requested = False
            self.release_completion_published = False
            self.release_state = PassThroughState.IDLE
            # 重置产品到位检测状态机，准备下一轮检测
            if self.product_arrival_cycle_active:
                self.product_arrival_published_in_cycle = False
                self._reset_product_arrival_state_machine()
                self.get_logger().info('放行流程完成，重置产品到位检测状态机，等待下一轮产品到位')

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
        # self.get_logger().info(f'已发送点动命令: {command}')

    def send_layer_command(self, layer: int):
        """发送层指令 - 优化版本，确保只发送一次"""
        # 检查是否已经发送过相同的层指令
        if self.last_layer_command == layer and self.layer_command_sent:
            self.get_logger().debug(f'层指令已发送过，跳过: 第{layer}层')
            return
        
        # 发送新命令
        msg = Int8()
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

    def _reset_key_do_signals(self):
        """重置关键DO信号（M810-M813）为0"""
        # self.send_do_control_once("810", False)  # 顶升气缸下降不用
        self.send_do_control_once("811", False)  # 齿轮对接气缸伸出
        self.send_do_control_once("812", False)  # 皮带正转启动
        self.send_do_control_once("813", False)  # 皮带反转启动
        self.get_logger().info('已发送关键DO信号复位命令 (M810-M813 -> 0)')

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
        
        # 重置放行流程状态
        self.release_state = PassThroughState.IDLE
        self.release_process_requested = False
        self.release_process_stop_requested = False
        self.release_request_received = False
        self.release_conveyor_in_was_true = False
        self.release_conveyor_in_completed = False
        self.release_conveyor_out_was_true = False
        self.release_conveyor_out_delay_started = False
        self.release_conveyor_out_delay_counter = 0
        self.release_completion_published = False
        
        # 重置延迟计数器
        self.delay_started = False
        self.delay_condition_triggered = False
        self.delay_counter = 0
        
        # 重置信号检测状态
        self.conveyor_in_detected = False
        self.buffer_sensor_2_detected = False  # 重置缓存架对射2检测状态
        self.outbound_conveyor_in_detected = False
        self.outbound_delay_started = False
        self.outbound_delay_condition_triggered = False
        self.outbound_delay_counter = 0
        
        # 新增：重置条件二延迟控制变量
        self.conveyor_in_then_out_delay_started = False
        self.conveyor_in_then_out_delay_counter = 0
        self.outbound_conveyor_in_then_out_delay_started = False
        self.outbound_conveyor_in_then_out_delay_counter = 0
        
        # 重置SMEMA状态（如果启用）
        if self.ENABLE_SMEMA:
            # 清除边沿检测状态
            if hasattr(self, '_last_uba'):
                self._last_uba = False
            if hasattr(self, '_last_dbr'):
                self._last_dbr = False

        # 重置自动模式初始化标志（停止后下次恢复需要重新等待）
        self.auto_mode_initialized = False
        self.pending_resume_state = None

        # 重置层指令发送状态，允许重新发送
        self.layer_command_sent = False
        self.last_layer_command = None

        # 重置层移动完成状态（关键：防止状态残留影响下次流程）
        self.layer_motion_completed = False
        self.previous_layer_completion_state = False
        self.layer_completion_received_time = None

        # 重置DO命令发送状态，允许重新发送
        self.reset_do_command_state()
        
        # 关键修复：重置完成发布标志，确保下次流程能正常发布
        self.warehouse_completion_published = False
        self.outbound_completion_published = False
        
        # 重置 COMPLETED 状态标志
        self.completed_reset_command_sent = False

        # +++ 新增：重置齿轮对接气缸检测状态 +++
        self.reset_gear_cylinder_check()
        
        # 清空待处理命令
        self.pending_commands.clear()
        
        self.get_logger().info('业务逻辑处理器状态已重置')

    # +++ SMEMA协议处理方法 +++
    def get_upstream_handshake_state(self) -> str:
        """根据IO信号判断上游握手状态
        
        状态判断逻辑：
        - UBA=OFF, MR=OFF: UPSTREAM_IDLE（空闲，本机无板）
        - UBA=OFF, MR=ON: UPSTREAM_READY（已发MR，等待上游UBA）
        - UBA=ON, MR=ON: UPSTREAM_RECEIVING（接收中）
        - UBA=OFF（从ON变为OFF）: UPSTREAM_BOARD_ARRIVED（板子到达）
        """
        if not self.ENABLE_SMEMA:
            return "DISABLED"
        
        uba = self.current_io_signals.get('smema_uba', False)
        mr = self.current_io_signals.get('smema_mr', False)
        
        if not mr:
            return "UPSTREAM_IDLE"
        elif not uba and mr:
            # 需要判断是从ON变为OFF，还是一直为OFF
            if hasattr(self, '_last_uba') and self._last_uba and not uba:
                return "UPSTREAM_BOARD_ARRIVED"
            else:
                return "UPSTREAM_READY"
        elif uba and mr:
            return "UPSTREAM_RECEIVING"
        else:
            return "UPSTREAM_IDLE"
    
    def get_downstream_handshake_state(self) -> str:
        """根据IO信号判断下游握手状态
        
        状态判断逻辑：
        - BA=OFF: DOWNSTREAM_IDLE（空闲，本机无板）
        - BA=ON, DBR=OFF: DOWNSTREAM_AVAILABLE（有板待发，等待下游DBR）
        - BA=ON, DBR=ON: DOWNSTREAM_SENDING（发送中）
        - DBR=OFF（从ON变为OFF）: DOWNSTREAM_SENT（板子已发）
        """
        if not self.ENABLE_SMEMA:
            return "DISABLED"
        
        dbr = self.current_io_signals.get('smema_dbr', False)
        ba = self.current_io_signals.get('smema_ba', False)
        
        if not ba:
            return "DOWNSTREAM_IDLE"
        elif ba and not dbr:
            # 需要判断是从ON变为OFF，还是一直为OFF
            if hasattr(self, '_last_dbr') and self._last_dbr and not dbr:
                return "DOWNSTREAM_SENT"
            else:
                return "DOWNSTREAM_AVAILABLE"
        elif ba and dbr:
            return "DOWNSTREAM_SENDING"
        else:
            return "DOWNSTREAM_IDLE"
    
    def update_product_position(self):
        """更新产品到位信号（驱动SMEMA握手）
        
        核心原则：
        - 进料检测信号出现到产品到位期间都不能要板
        - feed_detect、buffer_in、buffer_out三个信号存在期间都不能要板
        - conveyor_in从无到有再无（同时buffer_out没了）才认为到达接驳台，可以要板
        
        场景分析：
        1. 入库流程：
           - feed_product_detect → buffer_in → 产品到位检测完成 → 输送带运行 → conveyor_in
           - 在CONVEYOR_MOVING状态完成前，不能要板
           
        2. 放行流程：
           - buffer_in → 产品到位检测完成 → 输送带运行 → conveyor_in → 下游
           - 在conveyor_in检测完成前，不能要板
        
        判断逻辑：
        - product_in_position = True（不要板）的情况：
          1. feed_detect=True（进料检测信号存在）
          2. buffer_in=True（板子在缓存架入料口）
          3. buffer_out=True（板子在缓存架出料口）
          4. 产品到位检测状态机运行中（从IDLE到COMPLETED）
          5. 产品到位检测完成但未启动流程（板子在缓存架基准层）
          6. 入库流程CONVEYOR_MOVING状态（板子在传输中）
          7. 放行流程CONVEYOR_RUNNING状态且conveyor_in未检测完成
          
        - product_in_position = False（可以要板）的情况：
          1. conveyor_in从无到有再无（同时buffer_out没了）= 产品到达接驳台
          2. 板子已发送到下游
        """
        if not self.ENABLE_SMEMA:
            return
            
        di = self.current_io_signals
        feed_detect = di['feed_product_detect']
        buffer_in = di['buffer_in_position']
        buffer_out = di['buffer_out_position']
        conveyor_in = di['conveyor_in_position']
        
        # === 步骤1：更新信号跟踪状态 ===
        # 跟踪feed_detect是否曾经出现过
        if feed_detect:
            self.feed_detect_was_true = True
        
        # 跟踪buffer_out是否曾经出现过（产品到位检测）
        if buffer_out:
            self.buffer_out_was_true_for_arrival = True
        
        # 跟踪conveyor_in的检测状态（无→有→无）
        if conveyor_in:
            self.conveyor_in_was_true_for_arrival = True
        elif self.conveyor_in_was_true_for_arrival and not conveyor_in:
            # conveyor_in从有到无，且buffer_out也消失了，认为产品到达接驳台
            if not buffer_out and self.buffer_out_was_true_for_arrival:
                self.conveyor_in_completed_for_arrival = True
        
        # === 步骤2：判断是否可以要板 ===
        # 核心逻辑：排除法
        # - 列出所有不要板的情况
        # - 不满足这些情况的，就是可以要板
        
        # 不要板的情况列表
        no_board_reasons = []
        
        # 1. feed_detect=True（进料检测信号存在）
        if feed_detect:
            no_board_reasons.append("进料检测中")
        
        # 2. buffer_in=True（板子在缓存架入料口）
        if buffer_in:
            no_board_reasons.append("缓存架入料口有板")
        
        # 3. buffer_out=True（板子在缓存架出料口）
        if buffer_out:
            no_board_reasons.append("缓存架出料口有板")
        
        # 4. 产品到位检测状态机运行中（从IDLE到COMPLETED）
        if (self.product_arrival_state != "IDLE" and 
            self.product_arrival_state != "COMPLETED"):
            no_board_reasons.append(f"产品到位检测中({self.product_arrival_state})")
        
        # 5. 产品到位检测完成（板子在缓存架基准层）
        # 关键修复：只要产品到位检测完成，就认为产品在缓存架基准层，不要板
        # 只有当入库/放行流程执行到接驳台后，才会清除product_arrival_published_in_cycle标志
        if self.product_arrival_published_in_cycle:
            no_board_reasons.append("产品在缓存架基准层")
        
        # 6. 入库流程CONVEYOR_MOVING状态以及之前到idle（板子在传输中）
        if self.warehouse_state == WarehouseState.CONVEYOR_MOVING:
            no_board_reasons.append("入库传输中")
        
        # 7. 放行流程CONVEYOR_RUNNING状态且release_conveyor_in_completed为false（输送带运行中，产品未到达接驳台）
        if (self.release_state == PassThroughState.CONVEYOR_RUNNING and 
            not self.release_conveyor_in_completed):
            no_board_reasons.append("放行传输中")
        
        # 判断是否不要板（满足任意一个条件）
        product_in_position = len(no_board_reasons) > 0
        
        # 判断是否可以要板（互斥条件）
        can_request_board = not product_in_position
        
        # 发布到C++层
        msg = Bool()
        msg.data = product_in_position
        self.product_position_pub.publish(msg)
        
        # 只在状态变化时打印
        if not hasattr(self, '_last_product_position'):
            self._last_product_position = None
        
        if product_in_position != self._last_product_position:
            # 构建状态信息
            if can_request_board:
                # 可以要板的情况
                state_info = "产品已到达接驳台"
            else:
                # 不要板的情况
                state_info = " | ".join(no_board_reasons) if no_board_reasons else "无板"
            
            self.get_logger().info(
                f'产品到位信号: {product_in_position} - {state_info} - '
                f'{"不要板" if product_in_position else "可以要板"}'
            )
            self._last_product_position = product_in_position
    
    def check_smema_handshake(self):
        """检查SMEMA握手状态（从IO信号中判断）"""
        if not self.ENABLE_SMEMA:
            return
        
        # 获取上游握手状态
        upstream_state = self.get_upstream_handshake_state()
        
        # 检查上游板子是否到达（仅打印日志）
        if upstream_state == "UPSTREAM_BOARD_ARRIVED":
            self.get_logger().info('✅ 上游板子到达')
        
        # 获取下游握手状态
        downstream_state = self.get_downstream_handshake_state()
        
        # 检查下游板子是否发送完成（仅打印日志）
        if downstream_state == "DOWNSTREAM_SENT":
            self.get_logger().info('✅ 下游板子发送完成')
        
        # 更新上一次的UBA/DBR状态（用于边沿检测）
        self._last_uba = self.current_io_signals.get('smema_uba', False)
        self._last_dbr = self.current_io_signals.get('smema_dbr', False)

    def execute_pending_resume(self):
        """执行待处理的恢复状态（轴就绪后调用）"""
        if self.pending_resume_state is None:
            return
        
        state_info = self.pending_resume_state
        self.pending_resume_state = None  # 清除待处理状态
        
        if state_info['type'] == 'warehouse':
            # 执行入库恢复：关键修复！必须同步恢复状态到LIFT_MOVING
            self.warehouse_state = WarehouseState.LIFT_MOVING  # 恢复状态机状态
            self.warehouse_process_requested = True  # 确保流程继续执行
            self.layer_motion_completed = False  # 重置层移动完成标志
            self.previous_layer_completion_state = False  # 重置前一个状态
            self.layer_completion_received_time = None
            self.send_layer_command(state_info['layer'])
            self.get_logger().info(f'轴就绪后执行入库恢复：恢复状态到LIFT_MOVING，发送层指令到目标层 {state_info["layer"]}')
        # +++ 新增：处理入库完成状态的恢复 +++
        elif state_info['type'] == 'warehouse_completed':
            # 执行入库完成恢复：发送回到第1层的指令
            target_layer = state_info.get('layer', 1)  # 默认为第1层
            self.warehouse_state = WarehouseState.IDLE  # 恢复为空闲状态
            self.warehouse_process_requested = False
            self.layer_motion_completed = False
            self.previous_layer_completion_state = False
            self.layer_completion_received_time = None
            self.send_layer_command(target_layer)
            self.get_logger().info(f'轴就绪后执行入库完成恢复：恢复状态到IDLE，发送层指令回到第{target_layer}层')    
        elif state_info['type'] == 'outbound':
            # 执行出库恢复：同步恢复状态到LIFT_MOVING
            self.outbound_state = OutboundState.LIFT_MOVING  # 恢复状态机状态
            self.outbound_process_requested = True  # 确保流程继续执行
            self.layer_motion_completed = False  # 重置层移动完成标志
            self.previous_layer_completion_state = False  # 重置前一个状态
            self.layer_completion_received_time = None
            self.send_layer_command(state_info['layer'])
            self.get_logger().info(f'轴就绪后执行出库恢复：恢复状态到LIFT_MOVING，发送层指令到源层 {state_info["layer"]}')
        elif state_info['type'] == 'outbound_conveyor':
            # 执行出库CONVEYOR_MOVING状态恢复：同步恢复状态到CONVEYOR_MOVING
            self.outbound_state = OutboundState.CONVEYOR_MOVING  # 恢复状态机状态
            self.outbound_process_requested = True  # 确保流程继续执行
            self.layer_motion_completed = False  # 重置层移动完成标志
            self.previous_layer_completion_state = False  # 重置前一个状态
            self.layer_completion_received_time = None
            self.outbound_conveyor_started = False  # 重置输送带启动标志，让流程重新启动输送带
            self.send_layer_command(state_info['layer'])
            self.get_logger().info(f'轴就绪后执行出库恢复：恢复状态到CONVEYOR_MOVING，发送层指令到第{state_info["layer"]}层')
        elif state_info['type'] == 'outbound_post_lift':
            # 执行出库POST_LIFT_PROCESSING恢复
            self.outbound_state = OutboundState.LIFT_MOVING
            self.layer_motion_completed = True  # 标记层移动完成，让流程进入POST_LIFT_PROCESSING
            self.get_logger().info('轴就绪后执行出库POST_LIFT恢复：将进入POST_LIFT_PROCESSING状态执行JOG命令')
        # +++ 新增：处理IDLE状态恢复时的回第1层 +++
        elif state_info['type'] == 'warehouse_idle':
            # 执行入库IDLE恢复：发送回到第1层的指令
            target_layer = state_info.get('layer', 1)
            self.send_layer_command(target_layer)
            self.get_logger().info(f'轴就绪后执行入库IDLE恢复：发送层指令回到第{target_layer}层')
        elif state_info['type'] == 'outbound_idle':
            # 执行出库IDLE恢复：发送回到第1层的指令
            target_layer = state_info.get('layer', 1)
            self.send_layer_command(target_layer)
            self.get_logger().info(f'轴就绪后执行出库IDLE恢复：发送层指令回到第{target_layer}层')
        elif state_info['type'] == 'release_return_to_layer_1':
            # 执行放行流程回到第1层的恢复
            self.send_layer_command(1)
            self.get_logger().info(f'轴就绪后执行放行恢复：发送层指令回到第1层')

def main(args=None):
    rclpy.init(args=args)
    processor = BusinessLogicProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        if processor:
            print('业务逻辑处理器被用户中断')
    finally:
        processor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()