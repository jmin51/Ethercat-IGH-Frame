#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Empty, Bool
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

        # 新增：产品到位发布状态管理（0x0109）
        self.product_arrival_cycle_active = False  # 是否处于产品到位发布周期中
        self.product_arrival_published_in_cycle = False  # 本轮周期是否已发布过到位
        self.product_arrival_phase = "idle"  # 当前阶段: idle/pre_warehouse/warehouse/post_warehouse

        # 常量定义
        self.DELAY_BEFORE_STOP_MS = 800
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
            'buffer_sensor_2': False
        }
        
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
        
        # 新增：订阅开始作业信号（用于启动产品到位发布周期）
        self.start_operation_sub = self.create_subscription(
            Bool,
            '/start_operation_signal',
            self.start_operation_signal_callback,
            10
        )
   
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

        # 创建层移动完成订阅器
        self.layer_completion_sub = self.create_subscription(
            Bool,
            '/layer_motion_completed',
            self.layer_completion_callback,
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
        """映射DI编号到标准信号名称
        
        DI编号与M寄存器对应关系：
        DI00-DI08: M512-M520 (按钮、急停、气源、安全门、入料检测)
        DI09: M521 buffer_sensor_1 (缓存架对射1)
        DI10: M522 buffer_sensor_2 (缓存架对射2)
        DI11: M523 buffer_in_position (缓存架入料到位)
        DI12: M524 buffer_out_position (缓存架出料到位)
        DI13: M525 conveyor_in_position (接驳台入料到位)
        DI14: M526 conveyor_out_position (接驳台出料到位)
        """
        mapping = {
            10: 'buffer_sensor_2',      # M522 缓存架对射2
            11: 'buffer_in_position',   # M523 缓存架入料产品到位检测
            12: 'buffer_out_position',  # M524 缓存架出料产品到位检测
            13: 'conveyor_in_position', # M525 接驳台入料产品到位检测
            14: 'conveyor_out_position' # M526 接驳台出料产品到位检测
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

    def _handle_product_arrival_publication(self, buffer_in: bool):
        """处理产品到位发布逻辑（0x0109）
        
        逻辑：
        1. 开始作业(0x0105)到第一次入库开始前：只发布一次0x0109
        2. 入库完成后到下一次入库开始前：只发布一次0x0109
        """
        # 如果没有处于产品到位发布周期，不处理
        if not self.product_arrival_cycle_active:
            return
        
        # 如果本轮已经发布过到位信息，不重复发布
        if self.product_arrival_published_in_cycle:
            return
        
        # 检查是否有产品到位
        if buffer_in:
            # 发布产品到位消息
            arrival_msg = Bool()
            arrival_msg.data = True
            self.product_arrival_pub.publish(arrival_msg)
            self.product_arrival_published_in_cycle = True
            self.get_logger().info(f'✅ 检测到产品到位，发布0x0109（阶段: {self.product_arrival_phase}）')

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
        """报告当前业务状态供C++端记录"""
        # 记录当前状态
        self.saved_warehouse_state = self.warehouse_state
        self.saved_outbound_state = self.outbound_state
        self.saved_target_layer = self.target_layer
        self.saved_source_layer = self.source_layer
        
        # 构建状态报告字符串
        # 格式: warehouse_active=1,warehouse_state=X,warehouse_layer=Y,outbound_active=0,...
        warehouse_active = 1 if self.warehouse_state != WarehouseState.IDLE else 0
        outbound_active = 1 if self.outbound_state != OutboundState.IDLE else 0
        
        report = (f"warehouse_active={warehouse_active},"
                  f"warehouse_state={self.warehouse_state.value},"
                  f"warehouse_layer={self.target_layer},"
                  f"outbound_active={outbound_active},"
                  f"outbound_state={self.outbound_state.value},"
                  f"outbound_layer={self.source_layer}")
        
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
        elif state_value == WarehouseState.WAIT_FOR_ENTRY.value:
            self.warehouse_state = WarehouseState.WAIT_FOR_ENTRY
            self.warehouse_process_requested = True
        elif state_value == WarehouseState.CONVEYOR_MOVING.value:
            # 从输送带运行状态恢复 - 重新检测条件
            self.warehouse_state = WarehouseState.WAIT_FOR_ENTRY
            self.warehouse_process_requested = True
            self.get_logger().info('从CONVEYOR_MOVING恢复，将重新检测入库条件')
        elif state_value == WarehouseState.LIFT_MOVING.value:
            # 从提升机运行状态恢复 - 保持在LIFT_MOVING并重新发送层指令
            # 因为conveyor_in信号可能已不满足，不能直接回退到CONVEYOR_MOVING
            self.warehouse_state = WarehouseState.LIFT_MOVING
            self.warehouse_process_requested = True
            # 重置层移动状态（关键：必须同时重置previous_layer_completion_state）
            self.layer_motion_completed = False
            self.previous_layer_completion_state = False
            self.layer_completion_received_time = None
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
            self.get_logger().info('从POST_LIFT_PROCESSING恢复，将重新执行后续操作')
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
            self.get_logger().info('从DELAY_PROCESSING恢复，将重新执行延迟处理')
        elif state_value == WarehouseState.COMPLETED.value:
            self.warehouse_state = WarehouseState.COMPLETED
            self.warehouse_process_requested = True
        
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
        elif state_value == OutboundState.WAIT_FOR_EXIT.value:
            self.outbound_state = OutboundState.WAIT_FOR_EXIT
            self.outbound_process_requested = True
        elif state_value == OutboundState.LIFT_MOVING.value:
            # 从提升机运行状态恢复 - 保持在LIFT_MOVING并重新发送层指令
            self.outbound_state = OutboundState.LIFT_MOVING
            self.outbound_process_requested = True
            # 重置层移动状态（关键：必须同时重置previous_layer_completion_state）
            self.layer_motion_completed = False
            self.previous_layer_completion_state = False
            self.layer_completion_received_time = None
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
        
        self.resuming_from_pause = False

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
                    f'conveyor_out={self.current_io_signals["conveyor_out_position"]}, '
                    f'buffer_sensor_2={self.current_io_signals["buffer_sensor_2"]}'
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
        buffer_sensor_2 = di['buffer_sensor_2']  # 缓存架对射2信号 (M522)

        # 检查状态是否变化
        state_changed = (self.warehouse_state != self.previous_warehouse_state)
        
        # 只在状态变化时打印详细信息
        if state_changed:
            self.get_logger().info(
                f'入库流程状态变化: {self.previous_warehouse_state.name} -> {self.warehouse_state.name}, '
                f'当前层: {self.current_layer}, 目标层: {self.target_layer}'
            )
            self.previous_warehouse_state = self.warehouse_state
        
        # === 产品到位发布逻辑（0x0109）===
        # 情况1：入库前/出库前，收到产品到位，发布一次
        # 情况2：入库/出库流程完成后，收到产品到位，再次发布，形成循环
        self._handle_product_arrival_publication(buffer_in)
    
        # 处理停止请求
        if self.warehouse_process_stop_requested:
            self.warehouse_state = WarehouseState.IDLE
            self.warehouse_process_stop_requested = False
            self.warehouse_process_requested = False
            self.get_logger().info('入库流程已停止')
            return

        if self.warehouse_state == WarehouseState.IDLE:
            # 等待启动信号
            if (self.warehouse_process_requested and not buffer_out and 
                not conveyor_in and not conveyor_out):
                self._reset_key_do_signals()  # 重置关键DO信号，确保安全状态
                self.warehouse_state = WarehouseState.WAIT_FOR_ENTRY
                self.warehouse_process_requested = False
                # 更新产品到位发布阶段为"入库中"（入库前检测到到位仍可发布）
                if self.product_arrival_cycle_active and self.product_arrival_phase == "pre_warehouse":
                    self.product_arrival_phase = "warehouse"
                    self.get_logger().info('入库流程启动，更新产品到位发布阶段为: warehouse')
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
                self.get_logger().info('检测到条件二（conveyor_in变化），开始0.2秒延迟')
            
            # 处理条件二的延迟
            if self.conveyor_in_then_out_delay_started:
                self.conveyor_in_then_out_delay_counter += 1
                
                # 0.2秒延迟（2个周期，每周期100ms）
                if self.conveyor_in_then_out_delay_counter >= 2:
                    board_in_position = True
                    self.conveyor_in_then_out_delay_started = False
                    self.get_logger().info('条件二延迟结束，认为板子到位')
                else:
                    board_in_position = False
            else:
                # 条件一立即触发
                board_in_position = conveyor_out_detected
            
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
                # === 边缘触发：只在进入等待状态时打印一次 ===
                if not getattr(self, '_waiting_layer_motion_printed', False):
                    self.get_logger().info('等待层移动完成...')
                    self._waiting_layer_motion_printed = True
                return  # 继续等待
            
            # 层移动完成后执行后续操作
            self.get_logger().info('层移动完成，继续执行入库流程')
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
            if elapsed < 1.0:
                return  # 继续等待
            
            # 步骤3：延迟到达，执行后续操作
            self.send_do_control_once("811", True)  # 激活DO气缸伸出信号
            self.get_logger().info(f'DO气缸已激活，延迟{elapsed:.1f}秒')
            
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
            # 回到第1层
            self.send_layer_command(1)
            
            if (not buffer_in and not buffer_out and 
                not conveyor_in and not conveyor_out):
                self.warehouse_state = WarehouseState.IDLE
                # 发布入库完成消息（只在状态变化时发布一次）
                if not self.warehouse_completion_published:
                    completion_msg = Bool()
                    completion_msg.data = True
                    self.warehouse_completed_pub.publish(completion_msg)
                    self.warehouse_completion_published = True
                    self.get_logger().info('入库流程完成，发布完成消息')
                self.get_logger().info('回到初始状态，等待下一次入库')
                
                # 入库流程完成后，重置产品到位发布状态，允许再次发布
                if self.product_arrival_cycle_active:
                    self.product_arrival_published_in_cycle = False
                    self.product_arrival_phase = "post_warehouse"
                    self.get_logger().info('入库流程完成，重置产品到位发布状态，等待下一轮产品到位')

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

            # 等待启动信号 - 出库流程只需要收到请求即可启动，不检查IO条件
            # （货物在缓存架上是正常情况，不需要等待所有IO为False）
            if self.outbound_process_requested:
                self._reset_key_do_signals()
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
                # === 边缘触发：只在进入等待状态时打印一次 ===
                if not getattr(self, '_outbound_waiting_layer_motion_printed', False):
                    self.get_logger().info('等待层移动完成...')
                    self._outbound_waiting_layer_motion_printed = True
                return  # 继续等待
            
            # 层移动完成后执行后续操作
            self.get_logger().info('层移动完成，继续执行出库流程')
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
                    self.outbound_process_requested = False  # 关键：重置启动请求标志
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
                    
                    # 出库流程完成后，重置产品到位发布状态，允许再次发布
                    if self.product_arrival_cycle_active:
                        self.product_arrival_published_in_cycle = False
                        self.product_arrival_phase = "post_outbound"
                        self.get_logger().info('出库流程完成，重置产品到位发布状态，等待下一轮产品到位')

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
        self.send_do_control_once("810", False)  # 顶升气缸下降
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
        
        # 清空待处理命令
        self.pending_commands.clear()
        
        self.get_logger().info('业务逻辑处理器状态已重置')

    def execute_pending_resume(self):
        """执行待处理的恢复状态（轴就绪后调用）"""
        if self.pending_resume_state is None:
            return
        
        state_info = self.pending_resume_state
        self.pending_resume_state = None  # 清除待处理状态
        
        if state_info['type'] == 'warehouse':
            # 执行入库恢复
            self.send_layer_command(state_info['layer'])
            self.get_logger().info(f'轴就绪后执行入库恢复：发送层指令到目标层 {state_info["layer"]}')
        elif state_info['type'] == 'outbound':
            # 执行出库恢复
            self.send_layer_command(state_info['layer'])
            self.get_logger().info(f'轴就绪后执行出库恢复：发送层指令到源层 {state_info["layer"]}')
        elif state_info['type'] == 'outbound_conveyor':
            # 执行出库CONVEYOR_MOVING状态恢复（发送层指令到第1层）
            self.send_layer_command(state_info['layer'])
            self.get_logger().info(f'轴就绪后执行出库恢复：发送层指令到第{state_info["layer"]}层')
        elif state_info['type'] == 'outbound_post_lift':
            # 执行出库POST_LIFT_PROCESSING恢复
            self.outbound_state = OutboundState.LIFT_MOVING
            self.layer_motion_completed = True  # 标记层移动完成，让流程进入POST_LIFT_PROCESSING
            self.get_logger().info('轴就绪后执行出库POST_LIFT恢复：将进入POST_LIFT_PROCESSING状态执行JOG命令')

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