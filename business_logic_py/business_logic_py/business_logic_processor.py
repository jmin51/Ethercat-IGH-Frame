#!/usr/bin/env python3
# ======================================================================
# 业务逻辑处理器 - ROS2 Node 壳子
# 负责：ROS2生命周期、订阅/发布器、定时器调度
# 业务逻辑委托给: io_handler / pause_resume_mgr / process_handlers
# ======================================================================
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Empty, Bool, Float64
import time

from .models import (
    WarehouseState, OutboundState, PassThroughState,
    CommandType, ControlAction, FaultCode
)
from .io_signal_handler import IoSignalHandler
from .pause_resume_manager import PauseResumeManager
from .process_handlers import ProcessHandlers


class BusinessLogicProcessor(Node):
    def __init__(self):
        super().__init__('business_logic_processor')

        # ========== 宏开关配置 ==========
        self.ENABLE_SMEMA = True

        # ========== 业务流程状态 ==========
        self.warehouse_state = WarehouseState.IDLE
        self.outbound_state = OutboundState.IDLE
        self.release_state = PassThroughState.IDLE
        self.current_layer = 1
        self.target_layer = 1
        self.source_layer = 1
        self.outbound_area = 0

        # 状态变化追踪
        self.previous_warehouse_state = WarehouseState.IDLE
        self.previous_outbound_state = OutboundState.IDLE
        self.previous_release_state = PassThroughState.IDLE
        self.previous_io_signals = {
            'buffer_in_position': False, 'buffer_out_position': False,
            'conveyor_in_position': False, 'conveyor_out_position': False,
            'buffer_sensor_2': False
        }
        self.state_change_counter = 0

        # POST_LIFT_PROCESSING 延迟
        self.post_lift_delay_start = None

        # ========== 控制标志 ==========
        self.warehouse_process_requested = False
        self.warehouse_process_stop_requested = False
        self.outbound_process_requested = False
        self.outbound_process_stop_requested = False
        self.release_process_requested = False
        self.release_process_stop_requested = False
        self.release_request_received = False

        # ========== 暂停/恢复状态 ==========
        self.pause_state_reported = False
        self.operation_ended_during_pause = False
        self.resuming_from_pause = False
        self.saved_warehouse_state = None
        self.saved_outbound_state = None
        self.saved_target_layer = 1
        self.saved_source_layer = 1

        # ========== 自动模式初始化等待 ==========
        self.auto_mode_initialized = False
        self.pending_resume_state = None

        # ========== 运行状态 ==========
        self.auto_mode_enabled = False
        self.enabled = True
        self.initialized = False

        # ========== 延迟控制 - 入库 ==========
        self.delay_started = False
        self.delay_counter = 0
        self.delay_condition_triggered = False
        self.conveyor_in_detected = False
        self.buffer_sensor_2_detected = False

        # ========== 延迟控制 - 出库 ==========
        self.outbound_delay_started = False
        self.outbound_delay_counter = 0
        self.outbound_delay_condition_triggered = False
        self.outbound_conveyor_in_detected = False

        # 出库完成回包延迟
        self.outbound_completion_delay_started = False
        self.outbound_completion_delay_counter = 0
        self.OUTBOUND_COMPLETION_DELAY_SEC = 2.0
        self.OUTBOUND_COMPLETION_DELAY_COUNTER_MAX = int(self.OUTBOUND_COMPLETION_DELAY_SEC * 10)

        # 条件二延迟控制 - 入库
        self.conveyor_in_then_out_delay_started = False
        self.conveyor_in_then_out_delay_counter = 0
        # 条件二延迟控制 - 出库
        self.outbound_conveyor_in_then_out_delay_started = False
        self.outbound_conveyor_in_then_out_delay_counter = 0

        # ========== POST_LIFT_PROCESSING 超时 ==========
        self.POST_LIFT_PROCESS_TIMEOUT = 15.0
        self.post_lift_process_start_time = None
        self.post_lift_timeout_reported = False

        # ========== 层指令状态追踪 ==========
        self.last_layer_command = None
        self.layer_command_sent = False

        # ========== 放行流程状态 ==========
        self.release_conveyor_in_was_true = False
        self.release_conveyor_in_completed = False
        self.release_conveyor_out_was_true = False
        self.release_conveyor_out_delay_started = False
        self.release_conveyor_out_delay_counter = 0
        self.release_completion_published = False

        # ========== 齿轮对接气缸检测 ==========
        self.GEAR_CYLINDER_TIMEOUT_SEC = 2.0
        self.gear_cylinder_811_sent_time = None
        self.gear_cylinder_811_checking = False
        self.gear_cylinder_811_timeout_reported = False

        # ========== 产品到位发布状态 ==========
        self.product_arrival_cycle_active = False
        self.product_arrival_published_in_cycle = False
        self.product_arrival_phase = "idle"

        # 产品到位信号跟踪
        self.feed_detect_was_true = False
        self.buffer_out_was_true_for_arrival = False
        self.conveyor_in_was_true_for_arrival = False
        self.conveyor_in_completed_for_arrival = False

        # ========== 常量 ==========
        self.DELAY_BEFORE_STOP_MS = 400
        self.DELAY_COUNTER_MAX = self.DELAY_BEFORE_STOP_MS // 100
        self.OUTBOUND_DELAY_BEFORE_STOP_MS = 300
        self.OUTBOUND_DELAY_COUNTER_MAX = self.OUTBOUND_DELAY_BEFORE_STOP_MS // 100

        # ========== 数据 ==========
        self.pending_commands = []

        # ========== IO信号 ==========
        self.current_io_signals = {
            'buffer_in_position': False, 'buffer_out_position': False,
            'conveyor_in_position': False, 'conveyor_out_position': False,
            'buffer_sensor_2': False, 'feed_product_detect': False,
            'smema_uba': False, 'smema_dbr': False
        }

        # 产品到位检测状态机
        self.product_arrival_state = "IDLE"
        self.feed_detected = False
        self.buffer_in_detected = False
        self.conveyor_started_for_arrival = False

        # DO控制状态
        self.last_do_commands = {}
        self.do_command_sent = {}

        # ========== 常量 - 接驳台速度控制 ==========
        # 近层(-7~+7)慢速精定位，远层快速节省时间
        self.CONVEYOR_SPEED_NEAR = 150.0    # 近层速度 (mm/s)
        self.CONVEYOR_SPEED_FAR = 300.0     # 远层速度 (mm/s)
        self.CONVEYOR_SPEED_NEUTRAL_RANGE = 7
        self.CONVEYOR_SPEED_AXIS_NAMES = ["axis5"]

        # ========== 超时检测 ==========
        self.WAREHOUSE_PROCESS_TIMEOUT = 60.0
        self.OUTBOUND_PROCESS_TIMEOUT = 40.0
        self.warehouse_process_start_time = None
        self.outbound_process_start_time = None
        self.warehouse_timeout_reported = False
        self.outbound_timeout_reported = False

        # ========== 故障码 ==========
        self.last_published_fault_code = FaultCode.NO_FAULT.value

        # ========== 完成发布状态 ==========
        self.warehouse_completion_published = False
        self.outbound_completion_published = False
        self.completed_reset_command_sent = False

        # ========== 层号追踪 ==========
        self.current_layer_float = 1.0
        self.layer_tolerance = 0.1
        self.layer_motion_completed = False
        self.previous_layer_completion_state = False
        self.layer_completion_received_time = None
        self.layer_completion_timeout = 30.0
        
        # ========== 层移动指令追踪(0x011E) ==========
        self.layer_move_active = False        # 是否有0x011E触发的层移动在执行中
        self.layer_move_target_layer = None   # 0x011E触发的目标层号

        # ================================================================
        # ROS2 发布器
        # ================================================================
        self.jog_pub = self.create_publisher(String, '/jog_command', 10)
        self.do_control_pub = self.create_publisher(String, '/do_control', 10)
        self.layer_pub = self.create_publisher(Int8, '/layer_command', 10)
        self.jog_speed_pub = self.create_publisher(String, '/jog_speed_command', 10)
        self.warehouse_completed_pub = self.create_publisher(Bool, '/warehouse_completed', 10)
        self.outbound_completed_pub = self.create_publisher(Bool, '/outbound_completed', 10)
        self.product_arrival_pub = self.create_publisher(Bool, '/product_arrival', 10)
        self.release_completed_pub = self.create_publisher(Bool, '/release_completed', 10)
        self.fault_code_pub = self.create_publisher(String, '/business_logic_fault', 10)
        self.pause_state_report_pub = self.create_publisher(String, '/pause_state_report', 10)
        self.layer_move_completed_pub = self.create_publisher(Bool, '/layer_move_completed', 10)

        if self.ENABLE_SMEMA:
            self.product_position_pub = self.create_publisher(
                Bool, '/smema/product_in_position', 10)

        # ================================================================
        # ROS2 订阅器
        # ================================================================
        self.create_subscription(Bool, '/start_operation_signal',
                                 self.start_operation_signal_callback, 10)
        self.create_subscription(String, '/io_status',
                                 self.io_status_callback, 10)
        self.create_subscription(String, '/system_status',
                                 self.system_status_callback, 10)
        self.create_subscription(Int8, '/warehouse_start',
                                 self.warehouse_start_callback, 10)
        self.create_subscription(Empty, '/warehouse_stop',
                                 self.warehouse_stop_callback, 10)
        self.create_subscription(Int8, '/outbound_start',
                                 self.outbound_start_callback, 10)
        self.create_subscription(Empty, '/outbound_stop',
                                 self.outbound_stop_callback, 10)
        self.create_subscription(Int8, '/outbound_area',
                                 self.outbound_area_callback, 10)
        self.create_subscription(Empty, '/release_start',
                                 self.release_start_callback, 10)
        self.create_subscription(Empty, '/release_stop',
                                 self.release_stop_callback, 10)
        self.create_subscription(Bool, '/layer_motion_completed',
                                 self.layer_completion_callback, 10)
        self.create_subscription(Float64, '/axis5_current_layer',
                                 self.current_layer_callback, 10)
        self.create_subscription(String, '/pause_state_command',
                                 self.pause_state_command_callback, 10)
        # 新增：层移动指令(0x011E)订阅
        self.create_subscription(Int8, '/layer_move_start',
                                 self.layer_move_start_callback, 10)

        # ================================================================
        # 子模块初始化
        # ================================================================
        self.io_handler = IoSignalHandler(self)
        self.pause_resume_mgr = PauseResumeManager(self)
        self.process_handlers = ProcessHandlers(self)

        # ================================================================
        # 定时器
        # ================================================================
        self.timer = self.create_timer(0.1, self.process_logic)
        self.get_logger().info('Python业务逻辑处理器已启动 - 支持暂停状态记录与恢复')

    # ================================================================
    # 订阅回调
    # ================================================================
    def system_status_callback(self, msg):
        """处理系统状态消息，检测自动模式"""
        status_text = msg.data
        if '执行命令: start_auto' in status_text:
            if not self.auto_mode_enabled:
                self.auto_mode_enabled = True
                self.get_logger().info('检测到自动模式启动命令，业务逻辑处理器进入自动模式')
        elif '执行命令: stop' in status_text:
            if self.auto_mode_enabled:
                self.auto_mode_enabled = False
                self.reset_business_logic()
                self.get_logger().info('检测到停止命令，业务逻辑处理器退出自动模式')
        elif '自动模式初始化完成' in status_text:
            if not self.auto_mode_initialized:
                self.auto_mode_initialized = True
                self.get_logger().info('检测到自动模式初始化完成，轴已就绪')
                if self.pending_resume_state is not None:
                    self.pause_resume_mgr.execute_pending_resume()
            if not self.auto_mode_enabled:
                self.auto_mode_enabled = True
                self.get_logger().info('IO控制模式下，检测到轴自动模式就绪，业务逻辑处理器进入自动模式')

    def io_status_callback(self, msg):
        """处理IO状态更新"""
        try:
            io_data = self.io_handler.parse_io_status(msg.data)
            self.current_io_signals.update(io_data)
            self.process_io_signals()
        except Exception as e:
            self.get_logger().error(f'IO状态解析错误: {e}')

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
        # COMPLETED状态下不重置完成发布标志 — 否则COMPLETED handler会重复发布0x102
        if self.warehouse_state != WarehouseState.COMPLETED:
            self.warehouse_completion_published = False
        self.get_logger().info(f'收到入库流程启动请求，目标层: {self.target_layer}')

    def warehouse_stop_callback(self, msg):
        """处理入库停止命令"""
        if self.pause_state_reported:
            self.operation_ended_during_pause = True
            self.get_logger().info('暂停期间收到入库停止请求，标记结束作业')
        self._reset_key_do_signals()
        self.warehouse_process_stop_requested = True
        self.get_logger().info('收到入库流程停止请求')

    def start_operation_signal_callback(self, msg):
        """处理开始作业信号"""
        if msg.data:
            self.get_logger().info('收到开始作业信号(0x0105)，启动产品到位发布周期')
            self.product_arrival_cycle_active = True
            self.product_arrival_published_in_cycle = False
            self.product_arrival_phase = "pre_warehouse"
            self.io_handler.reset_product_arrival_state_machine()

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
        self.outbound_completion_published = False
        self.get_logger().info(f'收到出库流程启动请求，源层: {self.source_layer}')

    def outbound_stop_callback(self, msg):
        """处理出库停止命令"""
        if self.pause_state_reported:
            self.operation_ended_during_pause = True
            self.get_logger().info('暂停期间收到出库停止请求，标记结束作业')
        self._reset_key_do_signals()
        self.outbound_process_stop_requested = True
        self.get_logger().info('收到出库流程停止请求')

    def outbound_area_callback(self, msg):
        """处理出库区域命令"""
        self.outbound_area = msg.data
        self.get_logger().info(f'收到出库区域设置: {self.outbound_area}')

    def release_start_callback(self, msg):
        """处理放行启动命令"""
        if not self.auto_mode_enabled:
            self.get_logger().warn('自动模式未启用，忽略放行启动命令')
            return
        if self.release_state not in [PassThroughState.IDLE, PassThroughState.COMPLETED]:
            self.get_logger().warn('放行流程已在运行中，无法重复启动')
            return
        self.release_process_requested = True
        self.release_process_stop_requested = False
        self.release_completion_published = False
        self.get_logger().info('收到放行流程启动请求，进入等待产品到位状态')

    def release_stop_callback(self, msg):
        """处理放行停止命令"""
        if self.pause_state_reported:
            self.operation_ended_during_pause = True
            self.get_logger().info('暂停期间收到放行停止请求，标记结束作业')
        self._reset_key_do_signals()
        self.release_process_stop_requested = True
        self.get_logger().info('收到放行流程停止请求')

    def layer_completion_callback(self, msg):
        """层移动完成回调"""
        try:
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.layer_completion_received_time = current_time
            current_state = msg.data
            was_false_now_true = (not self.previous_layer_completion_state and current_state)
            if was_false_now_true:
                self.layer_motion_completed = True
                self.get_logger().info('检测到层移动完成：false -> true')
            elif current_state:
                self.get_logger().debug('重复的层移动完成信号（已经是true状态）')
            else:
                self.get_logger().debug('层移动进行中或刚开始')
            self.previous_layer_completion_state = current_state
        except Exception as e:
            self.get_logger().error(f'层移动完成回调处理错误: {e}')

    def current_layer_callback(self, msg):
        """当前层号回调"""
        try:
            new_layer = msg.data
            if abs(new_layer - self.current_layer_float) > 0.01:
                self.get_logger().debug(
                    f'当前层号更新: {self.current_layer_float:.2f} -> {new_layer:.2f}')
            self.current_layer_float = new_layer
            self.current_layer = round(new_layer)
            # 归位运动中动态调速：接近目标层时降速精定位
            self._check_return_speed_transition()
        except Exception as e:
            self.get_logger().error(f'当前层号回调处理错误: {e}')

    def pause_state_command_callback(self, msg):
        """处理暂停状态命令"""
        command = msg.data
        self.get_logger().info(f'收到暂停状态命令: {command}')
        if command == "RECORD_STATE":
            self.pause_resume_mgr.report_current_pause_state()
        elif command.startswith("RESUME:"):
            self.pause_resume_mgr.handle_resume_command(command)
    
    def layer_move_start_callback(self, msg):
        """处理层移动启动命令 (/layer_move_start，来自0x011E)
        
        纯层移动流程：只管axis5移到位，不涉及入库/出库的IO操作
        """
        target_layer = msg.data
        self.get_logger().info(
            f'收到层移动指令(0x011E)，目标层: {target_layer}, '
            f'自动模式={self.auto_mode_enabled}')
        
        # 设置层移动追踪状态
        self.layer_move_active = True
        self.layer_move_target_layer = target_layer
        self.layer_motion_completed = False
        # 重置边沿检测基准：确保后续的 false→true 边沿能被捕获
        self.previous_layer_completion_state = False
        
        # 重置层指令去重，确保层指令能发出
        self.layer_command_sent = False
        self.last_layer_command = None
        
        # 发送层指令到C++层
        self.send_layer_command(target_layer)
        self.get_logger().info(f'已发送层移动指令: 第{target_layer}层')

    # ================================================================
    # 主调度逻辑
    # ================================================================
    def process_logic(self):
        """主处理逻辑 - 定时器回调"""
        # 层移动(0x011E)是独立流程，手动/自动模式均可运行
        self._check_layer_move_completion()

        if not self.auto_mode_enabled or not self.enabled:
            return

        self.io_handler.process_product_arrival_logic()

        if self.ENABLE_SMEMA:
            self.io_handler.update_product_position()
            self.io_handler.check_smema_handshake()

        self.process_handlers.process_warehouse_logic()
        self.process_handlers.process_outbound_logic()
        self.process_handlers.process_release_logic()
        self.process_io_signals()
        self.execute_pending_commands()
        self.check_process_timeout()
        self.check_gear_cylinder_position()

    # ================================================================
    # IO信号变化检测
    # ================================================================
    def process_io_signals(self):
        """处理IO信号变化"""
        if not self.auto_mode_enabled or not self.enabled:
            return
        io_changed = any(
            self.previous_io_signals.get(k, None) != v
            for k, v in self.current_io_signals.items()
        )
        if io_changed:
            self.get_logger().info(
                f'IO信号状态变化: buffer_in={self.current_io_signals["buffer_in_position"]}, '
                f'buffer_out={self.current_io_signals["buffer_out_position"]}, '
                f'conveyor_in={self.current_io_signals["conveyor_in_position"]}, '
                f'conveyor_out={self.current_io_signals["conveyor_out_position"]}, '
                f'buffer_sensor_2={self.current_io_signals["buffer_sensor_2"]}')
        self.previous_io_signals = self.current_io_signals.copy()

    # ================================================================
    # 超时与故障检测
    # ================================================================
    def check_process_timeout(self):
        """检查入库/出库流程是否超时"""
        current_time = time.time()
        if (self.warehouse_state != WarehouseState.IDLE and
                self.warehouse_process_start_time is not None and
                not self.warehouse_timeout_reported):
            elapsed = current_time - self.warehouse_process_start_time
            if elapsed > self.WAREHOUSE_PROCESS_TIMEOUT:
                fault_code = FaultCode.WAREHOUSE_TIMEOUT.value
                self.get_logger().error(
                    f'入库流程超时({elapsed:.1f}秒>{self.WAREHOUSE_PROCESS_TIMEOUT}秒)，'
                    f'发布故障码=0x{fault_code:04X}')
                self.publish_fault_code(fault_code)
                self.warehouse_timeout_reported = True

        if (self.outbound_state != OutboundState.IDLE and
                self.outbound_process_start_time is not None and
                not self.outbound_timeout_reported):
            elapsed = current_time - self.outbound_process_start_time
            if elapsed > self.OUTBOUND_PROCESS_TIMEOUT:
                fault_code = FaultCode.OUTBOUND_TIMEOUT.value
                self.get_logger().error(
                    f'出库流程超时({elapsed:.1f}秒>{self.OUTBOUND_PROCESS_TIMEOUT}秒)，'
                    f'发布故障码=0x{fault_code:04X}')
                self.publish_fault_code(fault_code)
                self.outbound_timeout_reported = True

    def publish_fault_code(self, fault_code: int):
        """发布故障码"""
        if fault_code == self.last_published_fault_code:
            return
        self.last_published_fault_code = fault_code
        msg = String()
        if fault_code == 0:
            msg.data = "0"
        else:
            msg.data = f'business_logic:0x{fault_code:04X}'
        self.fault_code_pub.publish(msg)
        self.get_logger().warn(f'发布故障码到/fault_code: {msg.data}')

    def check_gear_cylinder_position(self):
        """检查齿轮对接气缸到位状态"""
        if not self.gear_cylinder_811_checking:
            return

        di = self.current_io_signals
        cylinder1 = di.get('gear_cylinder_1_in_position', False)
        cylinder2 = di.get('gear_cylinder_2_in_position', False)

        if cylinder1 and cylinder2:
            self.gear_cylinder_811_checking = False
            self.gear_cylinder_811_sent_time = None
            self.get_logger().info('M531+M533齿轮对接气缸1和2伸出到位检测通过')
            return

        if self._is_gear_cylinder_implicitly_arrived():
            self.gear_cylinder_811_checking = False
            self.gear_cylinder_811_sent_time = None
            self.get_logger().info('齿轮对接气缸隐式到位：状态已推进，皮带运转正常')
            return

        if self.gear_cylinder_811_sent_time is not None:
            elapsed = time.time() - self.gear_cylinder_811_sent_time
            if elapsed > self.GEAR_CYLINDER_TIMEOUT_SEC:
                if not self.gear_cylinder_811_timeout_reported:
                    fault_code = FaultCode.GEAR_CYLINDER_TIMEOUT.value
                    s1 = "到位" if cylinder1 else "未到位"
                    s2 = "到位" if cylinder2 else "未到位"
                    self.get_logger().error(
                        f'齿轮对接气缸伸出超时({elapsed:.1f}秒>{self.GEAR_CYLINDER_TIMEOUT_SEC}秒) '
                        f'M531气缸1:{s1}, M533气缸2:{s2}，发布故障码=0x{fault_code:04X}')
                    self.publish_fault_code(fault_code)
                    self.gear_cylinder_811_timeout_reported = True

    def _is_gear_cylinder_implicitly_arrived(self):
        """判断气缸是否隐式到位"""
        if self.warehouse_state in (WarehouseState.DELAY_PROCESSING, WarehouseState.COMPLETED):
            return True
        if self.outbound_state in (OutboundState.CONVEYOR_MOVING, OutboundState.COMPLETED):
            return True
        return False

    # ================================================================
    # 辅助方法
    # ================================================================
    def is_target_layer_reached(self, target_layer):
        """检查目标层号是否已到达"""
        return abs(self.current_layer_float - target_layer) <= self.layer_tolerance
    
    def _check_layer_move_completion(self):
        """检查0x011E触发的层移动是否完成
        
        双重确认：layer_motion_completed 信号 + current_layer_float 到位校验
        """
        if not self.layer_move_active:
            return
        
        if not self.layer_motion_completed:
            return
        
        # 双重确认：检查当前层号是否接近目标层（容差0.5层）
        if self.layer_move_target_layer is not None:
            layer_diff = abs(self.current_layer_float - self.layer_move_target_layer)
            if layer_diff > 0.5:
                self.get_logger().warn(
                    f'层移动完成信号已收到，但当前层({self.current_layer_float:.2f})'
                    f'与目标层({self.layer_move_target_layer})偏差{layer_diff:.2f}层，仍确认完成')
        
        # 发布完成信号到/layer_move_completed（供byte_multiarray_parser回包0x011F）
        completed_msg = Bool()
        completed_msg.data = True
        self.layer_move_completed_pub.publish(completed_msg)
        
        self.get_logger().info(
            f'层移动(0x011E)完成: 目标层={self.layer_move_target_layer}, '
            f'当前层={self.current_layer_float:.2f}, 自动模式={self.auto_mode_enabled}')
        
        # 重置追踪状态
        self.layer_move_active = False
        self.layer_move_target_layer = None
        self.layer_motion_completed = False

    def reset_process_timeout(self, process_type: str):
        """重置流程超时状态"""
        if process_type == 'warehouse':
            self.warehouse_process_start_time = time.time()
            self.warehouse_timeout_reported = False
        elif process_type == 'outbound':
            self.outbound_process_start_time = time.time()
            self.outbound_timeout_reported = False

    def clear_process_timeout(self, process_type: str):
        """清除流程超时状态"""
        if process_type == 'warehouse':
            self.warehouse_process_start_time = None
            self.warehouse_timeout_reported = False
            if self.last_published_fault_code == FaultCode.WAREHOUSE_TIMEOUT.value:
                self.publish_fault_code(0)
                self.last_published_fault_code = 0
        elif process_type == 'outbound':
            self.outbound_process_start_time = None
            self.outbound_timeout_reported = False
            if self.last_published_fault_code == FaultCode.OUTBOUND_TIMEOUT.value:
                self.publish_fault_code(0)
                self.last_published_fault_code = 0

    def add_command(self, action: ControlAction):
        """添加控制命令到待处理队列"""
        self.pending_commands.append(action)
        self.get_logger().info(
            f'生成控制命令: 类型={action.type.name}, 轴={action.axis_name}, '
            f'值={action.command_value}, 描述={action.description}')

    def execute_pending_commands(self):
        """执行待处理命令"""
        for command in self.pending_commands:
            if command.type == CommandType.JOG:
                self.send_jog_command(f"{command.axis_name}:{command.command_value}")
        self.pending_commands.clear()

    def send_jog_command(self, command: str):
        """发送点动命令"""
        msg = String()
        msg.data = command
        self.jog_pub.publish(msg)

    def calculate_conveyor_speed(self, mapped_layer: int) -> float:
        """根据映射后层号计算接驳台速度：近层慢速，远层快速"""
        # 近层(-7~+7)：慢速精定位
        if -self.CONVEYOR_SPEED_NEUTRAL_RANGE <= mapped_layer <= self.CONVEYOR_SPEED_NEUTRAL_RANGE:
            return self.CONVEYOR_SPEED_NEAR
        # 远层：按距离线性加速
        if mapped_layer < -self.CONVEYOR_SPEED_NEUTRAL_RANGE:
            distance = abs(mapped_layer) - self.CONVEYOR_SPEED_NEUTRAL_RANGE
        else:
            distance = mapped_layer - self.CONVEYOR_SPEED_NEUTRAL_RANGE
        max_distance = max(
            abs(-15 - (-self.CONVEYOR_SPEED_NEUTRAL_RANGE)),
            abs(28 - self.CONVEYOR_SPEED_NEUTRAL_RANGE))
        speed_range = self.CONVEYOR_SPEED_FAR - self.CONVEYOR_SPEED_NEAR
        ratio = min(distance / max_distance, 1.0)
        speed = self.CONVEYOR_SPEED_NEAR + speed_range * ratio
        return round(min(speed, self.CONVEYOR_SPEED_FAR), 1)

    def adjust_conveyor_speed_by_layer(self, mapped_layer: int):
        """根据映射后层号调整接驳台速度"""
        speed = self.calculate_conveyor_speed(mapped_layer)
        for axis_name in self.CONVEYOR_SPEED_AXIS_NAMES:
            msg = String()
            msg.data = f'{axis_name}:{speed}'
            self.jog_speed_pub.publish(msg)
        if speed != self.CONVEYOR_SPEED_NEAR:
            self.get_logger().info(f'接驳台调速: 层号={mapped_layer}, 速度={speed}mm/s')

    def send_layer_command(self, layer: int, fast_return: bool = False):
        """发送层指令
        Args:
            layer: 目标层号（映射后）
            fast_return: True时使用归位调速策略（远层快速，近层降速精定位）
        """
        if self.last_layer_command == layer and self.layer_command_sent:
            self.get_logger().debug(f'层指令已发送过，跳过: 第{layer}层')
            return
        msg = Int8()
        msg.data = layer
        self.layer_pub.publish(msg)
        self.last_layer_command = layer
        self.layer_command_sent = True
        self.get_logger().info(f'已发送层指令: 第{layer}层')
        # 归位模式：根据当前层与目标层的距离动态调速
        if fast_return:
            self._adjust_conveyor_speed_for_return(layer)
        else:
            self.adjust_conveyor_speed_by_layer(layer)

    def _adjust_conveyor_speed_for_return(self, target_layer: int = 1):
        """归位调速：距目标层>7层用快速(300)，≤7层用慢速(150)精定位"""
        distance = abs(self.current_layer_float - target_layer)
        if distance <= self.CONVEYOR_SPEED_NEUTRAL_RANGE:
            speed = self.CONVEYOR_SPEED_NEAR
        else:
            speed = self.CONVEYOR_SPEED_FAR
        for axis_name in self.CONVEYOR_SPEED_AXIS_NAMES:
            msg = String()
            msg.data = f'{axis_name}:{speed}'
            self.jog_speed_pub.publish(msg)
        self.get_logger().info(
            f'接驳台归位调速: 目标层={target_layer}, 当前层={self.current_layer_float:.2f}, '
            f'距离={distance:.1f}层, 速度={speed}mm/s')

    def _check_return_speed_transition(self):
        """归位运动中动态调速：接近目标层7层时从300降速到150"""
        # 仅在归位运动且层指令已发出时生效
        if not self.layer_command_sent or self.last_layer_command is None:
            return
        # 非归位模式不干预（作业模式由adjust_conveyor_speed_by_layer管理）
        # 判断依据：入库COMPLETED/出库COMPLETED/放行COMPLETED都会send_layer_command(1, fast_return=True)
        target = self.last_layer_command
        distance = abs(self.current_layer_float - target)
        # 计算当前应有的速度
        if distance <= self.CONVEYOR_SPEED_NEUTRAL_RANGE:
            expected_speed = self.CONVEYOR_SPEED_NEAR
        else:
            expected_speed = self.CONVEYOR_SPEED_FAR
        # 避免重复下发相同速度
        if not hasattr(self, '_last_return_speed'):
            self._last_return_speed = None
        if expected_speed != self._last_return_speed:
            self._last_return_speed = expected_speed
            for axis_name in self.CONVEYOR_SPEED_AXIS_NAMES:
                msg = String()
                msg.data = f'{axis_name}:{expected_speed}'
                self.jog_speed_pub.publish(msg)
            self.get_logger().info(
                f'归位动态调速: 距目标层{target}={distance:.1f}层, '
                f'切换速度={expected_speed}mm/s')

    def send_do_control_once(self, do_address: str, state: bool):
        """发送DO控制命令"""
        command_str = f'{do_address}:{1 if state else 0}'
        if (do_address in self.last_do_commands and
                self.last_do_commands[do_address] == state):
            if do_address not in self.do_command_sent or not self.do_command_sent[do_address]:
                self.do_command_sent[do_address] = True
            else:
                self.get_logger().debug(f'DO控制命令已发送过，跳过: {command_str}')
                return
        msg = String()
        msg.data = command_str
        self.do_control_pub.publish(msg)
        self.last_do_commands[do_address] = state
        self.do_command_sent[do_address] = True
        self.get_logger().info(f'已发送DO控制: {msg.data}')

    def reset_do_command_state(self, do_address: str = None):
        """重置DO命令发送状态"""
        if do_address is None:
            for addr in self.do_command_sent:
                self.do_command_sent[addr] = False
            self.get_logger().info('重置所有DO命令发送状态')
        elif do_address in self.do_command_sent:
            self.do_command_sent[do_address] = False
            self.get_logger().info(f'重置DO命令发送状态: {do_address}')

    def _reset_key_do_signals(self):
        """重置关键DO信号"""
        self.send_do_control_once("811", False)
        self.send_do_control_once("812", False)
        self.send_do_control_once("813", False)
        self.get_logger().info('已发送关键DO信号复位命令 (M810-M813 -> 0)')

    def reset_business_logic(self):
        """重置业务逻辑状态"""
        self.warehouse_state = WarehouseState.IDLE
        self.warehouse_process_requested = False
        self.warehouse_process_stop_requested = False
        self.outbound_state = OutboundState.IDLE
        self.outbound_process_requested = False
        self.outbound_process_stop_requested = False
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
        self.delay_started = False
        self.delay_condition_triggered = False
        self.delay_counter = 0
        self.conveyor_in_detected = False
        self.buffer_sensor_2_detected = False
        self.outbound_conveyor_in_detected = False
        self.outbound_delay_started = False
        self.outbound_delay_condition_triggered = False
        self.outbound_delay_counter = 0
        self.conveyor_in_then_out_delay_started = False
        self.conveyor_in_then_out_delay_counter = 0
        self.outbound_conveyor_in_then_out_delay_started = False
        self.outbound_conveyor_in_then_out_delay_counter = 0
        self.post_lift_process_start_time = None
        self.post_lift_timeout_reported = False
        if self.ENABLE_SMEMA:
            if hasattr(self, '_last_uba'):
                self._last_uba = False
            if hasattr(self, '_last_dbr'):
                self._last_dbr = False
        self.auto_mode_initialized = False
        self.pending_resume_state = None
        self.layer_command_sent = False
        self.last_layer_command = None
        self._last_return_speed = None
        self.layer_motion_completed = False
        self.previous_layer_completion_state = False
        self.layer_completion_received_time = None
        # 重置层移动(0x011E)追踪状态
        self.layer_move_active = False
        self.layer_move_target_layer = None
        self.reset_do_command_state()
        self.warehouse_completion_published = False
        self.outbound_completion_published = False
        self.completed_reset_command_sent = False
        self.gear_cylinder_811_checking = False
        self.gear_cylinder_811_sent_time = None
        self.gear_cylinder_811_timeout_reported = False
        self.product_arrival_cycle_active = False
        self.product_arrival_published_in_cycle = False
        self.product_arrival_phase = "idle"
        self.product_arrival_state = "IDLE"
        self.feed_detected = False
        self.buffer_in_detected = False
        self.conveyor_started_for_arrival = False
        self.feed_detect_was_true = False
        self.buffer_out_was_true_for_arrival = False
        self.conveyor_in_was_true_for_arrival = False
        self.conveyor_in_completed_for_arrival = False
        self.pending_commands.clear()
        self.get_logger().info('业务逻辑处理器状态已重置')


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
