#!/usr/bin/env python3
# ======================================================================
# 业务流程状态机 - 入库/出库/放行
# 负责：三大核心业务流程的状态驱动逻辑
# ======================================================================
import time

from .models import (
    CommandType, ControlAction,
    WarehouseState, OutboundState, PassThroughState, FaultCode
)


class ProcessHandlers:
    """业务流程处理器 - 入库/出库/放行三大状态机"""

    def __init__(self, processor):
        self.proc = processor

    # ================================================================
    # 入库流程
    # ================================================================
    def process_warehouse_logic(self):
        """处理入库业务流程"""
        di = self.proc.current_io_signals
        buffer_in = di['buffer_in_position']
        buffer_out = di['buffer_out_position']
        conveyor_in_gap = di['conveyor_entry_gap_detect']   # M541 替代 conveyor_in
        conveyor_out = di['conveyor_out_position']           # 保留原信号
        buffer_sensor_2 = di['buffer_sensor_2']
        feed_detect = di['feed_product_detect']

        state_changed = (self.proc.warehouse_state != self.proc.previous_warehouse_state)
        if state_changed:
            self.proc.get_logger().info(
                f'入库流程状态变化: {self.proc.previous_warehouse_state.name} -> '
                f'{self.proc.warehouse_state.name}, '
                f'当前层: {self.proc.current_layer_float:.2f}, 目标层: {self.proc.target_layer}')
            self.proc.previous_warehouse_state = self.proc.warehouse_state

        # 停止请求
        if self.proc.warehouse_process_stop_requested:
            self.proc.warehouse_state = WarehouseState.IDLE
            self.proc.warehouse_process_stop_requested = False
            self.proc.warehouse_process_requested = False
            self.proc.get_logger().info('入库流程已停止')
            return

        # === IDLE ===
        if self.proc.warehouse_state == WarehouseState.IDLE:
            if (self.proc.warehouse_process_requested and
                    not conveyor_in_gap and not conveyor_out):
                self.proc._reset_key_do_signals()
                self.proc.warehouse_state = WarehouseState.WAIT_FOR_ENTRY
                self.proc.warehouse_process_requested = False
                if (self.proc.product_arrival_cycle_active and
                        self.proc.product_arrival_phase == "pre_warehouse"):
                    self.proc.product_arrival_phase = "warehouse"
                    self.proc.get_logger().info('入库流程启动，更新产品到位发布阶段为: warehouse')
                self.proc.reset_process_timeout('warehouse')
                self.proc.get_logger().info(
                    f'入库流程启动，进入等待入库状态，目标层: {self.proc.target_layer}')

        # === WAIT_FOR_ENTRY ===
        elif self.proc.warehouse_state == WarehouseState.WAIT_FOR_ENTRY:
            # 双重判定：事件标志(正常路径) + 物理信号(竞态回退路径)
            product_ready = (self.proc.product_arrival_published_in_cycle or
                             (buffer_out and not feed_detect))
            if product_ready:
                if buffer_out and not self.proc.product_arrival_published_in_cycle:
                    self.proc.get_logger().info(
                        '入库流程：事件标志已丢失，通过buffer_out物理信号回退启动')
                if not self.proc.is_target_layer_reached(1):
                    self.proc.get_logger().warn(
                        f'提升机不在第1层（当前层={self.proc.current_layer_float:.2f}），'
                        f'等待回到第1层后再开始入库')
                    self.proc.send_layer_command(1)
                    return
                self.proc.warehouse_state = WarehouseState.CONVEYOR_MOVING
                self.proc.product_arrival_published_in_cycle = False
                self.proc.get_logger().info(
                    f'检测到入库条件，当前层={self.proc.current_layer_float:.2f}，开始输送'
                    f'（消费产品到位标志）')
                # 恢复输送带默认速度（外部命令可能改过速度）
                self.proc.send_axis_speed("axis1_1", self.proc.DEFAULT_JOG_SPEEDS["axis1_1"])
                self.proc.send_axis_speed("axis1_2", self.proc.DEFAULT_JOG_SPEEDS["axis1_2"])
                self.proc.add_command(ControlAction(
                    CommandType.JOG, "axis1_1", "reverse",
                    description="启动轴1_1正转"))
                self.proc.add_command(ControlAction(
                    CommandType.JOG, "axis1_2", "forward",
                    description="启动轴1_2正转"))

        # === CONVEYOR_MOVING ===
        elif self.proc.warehouse_state == WarehouseState.CONVEYOR_MOVING:
            if buffer_out:
                self.proc.send_do_control_once("813", True)

            if conveyor_in_gap and not self.proc.conveyor_in_detected:
                self.proc.conveyor_in_detected = True

            conveyor_out_detected = conveyor_out
            conveyor_in_then_out = (self.proc.conveyor_in_detected and not conveyor_in_gap)

            # C++层M541防抖(300ms)已确保信号稳定消失, 无需业务层重复延迟
            board_in_position = conveyor_in_then_out or conveyor_out_detected

            if board_in_position:
                trigger_condition = []
                if conveyor_out_detected:
                    trigger_condition.append('conveyor_out')
                if conveyor_in_then_out:
                    trigger_condition.append('conveyor_in变化')
                self.proc.get_logger().info(
                    f'检测到板子到位: 触发条件={", ".join(trigger_condition)}')

                self.proc.add_command(ControlAction(
                    CommandType.JOG, "axis1_1", "stop", description="停止轴1_1"))
                self.proc.add_command(ControlAction(
                    CommandType.JOG, "axis1_2", "stop", description="停止轴1_2"))
                self.proc.send_do_control_once("813", False)
                self.proc.conveyor_in_detected = False
                self.proc.conveyor_in_then_out_delay_started = False

                self.proc.send_layer_command(self.proc.target_layer)
                self.proc.warehouse_state = WarehouseState.LIFT_MOVING
                self._reset_layer_motion_state()

        # === LIFT_MOVING ===
        elif self.proc.warehouse_state == WarehouseState.LIFT_MOVING:
            if not self.proc.layer_motion_completed:
                if not getattr(self.proc, '_waiting_layer_motion_printed', False):
                    self.proc.get_logger().info(
                        f'等待层移动完成... 目标层={self.proc.target_layer}, '
                        f'当前层={self.proc.current_layer_float:.2f}')
                    self.proc._waiting_layer_motion_printed = True
                return

            # 双重确认：信号到达 + 实际层号接近目标，防止暂停恢复时误触发
            if not self.proc.is_target_layer_reached(self.proc.target_layer):
                self.proc.get_logger().warn(
                    f'收到层移动完成信号但当前层({self.proc.current_layer_float:.2f})'
                    f'未到达目标层({self.proc.target_layer})'
                    f'(偏差{abs(self.proc.current_layer_float - self.proc.target_layer):.2f}层)，'
                    f'忽略误信号，继续等待轴到达')
                self.proc.layer_motion_completed = False
                return

            self.proc.get_logger().info(
                f'层移动完成，当前层={self.proc.current_layer_float:.2f}，继续执行入库流程')
            self._reset_layer_motion_state()
            self.proc._waiting_layer_motion_printed = False
            self.proc.post_lift_delay_start = None
            self.proc.warehouse_state = WarehouseState.POST_LIFT_PROCESSING
            self.proc.get_logger().info('进入层移动后处理状态')

        # === POST_LIFT_PROCESSING ===
        elif self.proc.warehouse_state == WarehouseState.POST_LIFT_PROCESSING:
            if self.proc.post_lift_delay_start is None:
                self.proc.get_logger().info('入库流程：层移动完成，继续执行后续操作')
                self.proc.send_do_control_once("812", True)
                # 恢复内部输送轴默认速度
                self.proc.send_axis_speed("axis2_1", self.proc.DEFAULT_JOG_SPEEDS["axis2_1"])
                self.proc.send_axis_speed("axis2_2", self.proc.DEFAULT_JOG_SPEEDS["axis2_2"])
                self.proc.add_command(ControlAction(
                    CommandType.JOG, "axis2_1", "forward", description="启动轴2_1正转"))
                self.proc.add_command(ControlAction(
                    CommandType.JOG, "axis2_2", "reverse", description="启动轴2_2反转"))
                self.proc.post_lift_delay_start = time.time()
                self.proc.get_logger().info('等待1秒后激活DO气缸...')
                return

            elapsed = time.time() - self.proc.post_lift_delay_start
            if elapsed < 0.3:
                return

            self.proc.send_do_control_once("811", True)
            self.proc.get_logger().info(f'DO811气缸伸出已激活，延迟{elapsed:.1f}秒')

            # 启动齿轮对接气缸到位检测
            self.proc.gear_cylinder_811_sent_time = time.time()
            self.proc.gear_cylinder_811_checking = True
            self.proc.gear_cylinder_811_timeout_reported = False
            self.proc.get_logger().info('启动M531+M533齿轮对接气缸1和2伸出到位检测...')

            self.proc.post_lift_delay_start = None
            self.proc.warehouse_state = WarehouseState.DELAY_PROCESSING
            self.proc.get_logger().info('进入延迟处理状态')

        # === DELAY_PROCESSING ===
        elif self.proc.warehouse_state == WarehouseState.DELAY_PROCESSING:
            if not self.proc.delay_started and not self.proc.delay_condition_triggered:
                if buffer_sensor_2:
                    if not self.proc.buffer_sensor_2_detected:
                        self.proc.buffer_sensor_2_detected = True
                        self.proc.get_logger().info('检测到货物进入缓存架(buffer_sensor_2=1)')
                elif (self.proc.buffer_sensor_2_detected and
                      not conveyor_in_gap and not buffer_sensor_2):
                    self.proc.delay_condition_triggered = True
                    self.proc.delay_started = True
                    self.proc.delay_counter = 0
                    self.proc.buffer_sensor_2_detected = False
                    self.proc.conveyor_in_detected = False
                    self.proc.get_logger().info(
                        f'检测到货物离开接驳台(conveyor_in=0,buffer_sensor_2=0)，'
                        f'开始{self.proc.DELAY_BEFORE_STOP_MS // 1000}秒延迟')

            if self.proc.delay_started:
                self.proc.delay_counter += 1
                if self.proc.delay_counter >= self.proc.DELAY_COUNTER_MAX:
                    self.proc.warehouse_state = WarehouseState.COMPLETED
                    self.proc.delay_started = False
                    self.proc.delay_condition_triggered = False
                    self.proc.buffer_sensor_2_detected = False
                    self.proc.state_change_counter = 0
                    self.proc.add_command(ControlAction(
                        CommandType.JOG, "axis2_1", "stop", description="轴2_1停止"))
                    self.proc.add_command(ControlAction(
                        CommandType.JOG, "axis2_2", "stop", description="轴2_2停止"))
                    self.proc.send_do_control_once("811", False)
                    self.proc.send_do_control_once("812", False)
                    self.proc.get_logger().info('延迟结束，停止轴2并进入完成状态')
                else:
                    if self.proc.delay_counter % 30 == 0:
                        remaining = self.proc.DELAY_BEFORE_STOP_MS // 1000 - self.proc.delay_counter // 10
                        self.proc.get_logger().info(f'延迟剩余时间: {remaining}秒')

        # === COMPLETED ===
        elif self.proc.warehouse_state == WarehouseState.COMPLETED:
            self.proc.send_layer_command(1, fast_return=True)
            if not self.proc.completed_reset_command_sent:
                self.proc.get_logger().info(
                    f'流程完成，发送升降机复位指令至第1层 '
                    f'(目标层=1, 当前实际层={self.proc.current_layer_float:.2f})')
                self.proc.completed_reset_command_sent = True

            if not self.proc.warehouse_completion_published:
                from std_msgs.msg import Bool
                completion_msg = Bool()
                completion_msg.data = True
                self.proc.warehouse_completed_pub.publish(completion_msg)
                self.proc.warehouse_completion_published = True
                self.proc.get_logger().info('入库流程完成，发布完成消息(0x102)')

                # 到位标志清理 — 仅在首次发布0x102时执行一次
                # 每周期清除会导致第二轮0x0109标志被覆盖
                if self.proc.product_arrival_cycle_active:
                    self.proc.product_arrival_published_in_cycle = False
                    self.proc.product_arrival_phase = "post_warehouse"
                    # 不在此处重置状态机: 入库刚发完0x0109, 板子大概率仍在buffer_out,
                    # reset回IDLE会被IDLE的buffer_out恢复逻辑误判为"作业重启需补发",
                    # 产生幽灵0x109(入库完成->出库启动的2.6秒误发即此路径)。
                    # 状态机保持COMPLETED, 由COMPLETED自然流转
                    # (not feed_detect and not buffer_out) 回IDLE。

            if not self.proc.is_target_layer_reached(1):
                self.proc.state_change_counter += 1
                if self.proc.state_change_counter >= 50:
                    self.proc.state_change_counter = 0
                    self.proc.get_logger().info(
                        f'等待升降机回到第1层... 当前层={self.proc.current_layer_float:.2f}')
                return

            # COMPLETED→IDLE: 保留warehouse_process_requested
            # 新入库命令可能在COMPLETED期间到达，不应吞掉
            self.proc.warehouse_completion_published = False
            self.proc.completed_reset_command_sent = False
            self.proc.warehouse_state = WarehouseState.IDLE
            self.proc.clear_process_timeout('warehouse')
            # 升降机已归位第1层, 下一块板可入库,
            # 回传事件闸门: 解锁下一轮0x0109发布.
            self.proc.board_dispatched = True
            self.proc.get_logger().info(
                f'升降机已回到第1层(当前层={self.proc.current_layer_float:.2f})，流程状态重置为IDLE')

    # ================================================================
    # 出库流程
    # ================================================================
    def process_outbound_logic(self):
        """处理出库业务流程"""
        di = self.proc.current_io_signals
        conveyor_in_gap = di['conveyor_entry_gap_detect']   # M541 替代 conveyor_in
        conveyor_out = di['conveyor_out_position']           # 保留原信号(POST_LIFT使用)
        conveyor_out_gap = di['conveyor_exit_gap_detect']    # M540 替代 conveyor_out(COMPLETED使用)

        state_changed = (self.proc.outbound_state != self.proc.previous_outbound_state)
        if state_changed:
            self.proc.get_logger().info(
                f'出库流程状态变化: {self.proc.previous_outbound_state.name} -> '
                f'{self.proc.outbound_state.name}, '
                f'当前层: {self.proc.current_layer_float:.2f}, 源层: {self.proc.source_layer}')
            self.proc.previous_outbound_state = self.proc.outbound_state

        # 停止请求
        if self.proc.outbound_process_stop_requested:
            self.proc.outbound_state = OutboundState.IDLE
            self.proc.outbound_process_stop_requested = False
            self.proc.outbound_process_requested = False
            self.proc.get_logger().info('出库流程已停止')
            return

        # === IDLE ===
        if self.proc.outbound_state == OutboundState.IDLE:
            if self.proc.outbound_process_requested:
                self.proc._reset_key_do_signals()
                self.proc.outbound_state = OutboundState.WAIT_FOR_EXIT
                self.proc.outbound_process_requested = False
                self.proc.reset_process_timeout('outbound')
                self.proc.get_logger().info(
                    f'出库流程启动，进入等待出库状态，源层: {self.proc.source_layer}')

        # === WAIT_FOR_EXIT ===
        elif self.proc.outbound_state == OutboundState.WAIT_FOR_EXIT:
            # 轴资源互斥: 等待入库归位完成，不抢占正在运动中的提升机
            if self.proc.warehouse_state != WarehouseState.IDLE:
                self.proc.get_logger().info(
                    f'出库等待入库流程完成'
                    f'(当前入库状态={self.proc.warehouse_state.name})，暂不抢占轴资源')
                return
            self.proc.get_logger().info('检测到出库条件，开始提升机运行')
            self.proc.send_layer_command(self.proc.source_layer)
            self.proc.outbound_state = OutboundState.LIFT_MOVING
            self._reset_layer_motion_state()

        # === LIFT_MOVING ===
        elif self.proc.outbound_state == OutboundState.LIFT_MOVING:
            if not self.proc.layer_motion_completed:
                if not getattr(self.proc, '_outbound_waiting_layer_motion_printed', False):
                    self.proc.get_logger().info(
                        f'等待层移动完成... 源层={self.proc.source_layer}, '
                        f'当前层={self.proc.current_layer_float:.2f}')
                    self.proc._outbound_waiting_layer_motion_printed = True
                return

            # 双重确认：信号到达 + 实际层号接近源层，防止暂停恢复时误触发
            if not self.proc.is_target_layer_reached(self.proc.source_layer):
                self.proc.get_logger().warn(
                    f'收到层移动完成信号但当前层({self.proc.current_layer_float:.2f})'
                    f'未到达源层({self.proc.source_layer})'
                    f'(偏差{abs(self.proc.current_layer_float - self.proc.source_layer):.2f}层)，'
                    f'忽略误信号，继续等待轴到达')
                self.proc.layer_motion_completed = False
                return

            self.proc.get_logger().info(
                f'层移动完成，当前层={self.proc.current_layer_float:.2f}，继续执行出库流程')
            self._reset_layer_motion_state()
            self.proc._outbound_waiting_layer_motion_printed = False

            self.proc.send_do_control_once("813", True)
            # 恢复内部输送轴默认速度
            self.proc.send_axis_speed("axis2_1", self.proc.DEFAULT_JOG_SPEEDS["axis2_1"])
            self.proc.send_axis_speed("axis2_2", self.proc.DEFAULT_JOG_SPEEDS["axis2_2"])
            self.proc.add_command(ControlAction(
                CommandType.JOG, "axis2_1", "reverse",
                description="启动轴2_1反转（出库）"))
            self.proc.add_command(ControlAction(
                CommandType.JOG, "axis2_2", "forward",
                description="启动轴2_2正转（出库）"))
            self.proc.send_do_control_once("811", True)
            self.proc.get_logger().info('出库：DO811齿轮对接气缸伸出已激活')

            # 启动齿轮对接气缸到位检测
            self.proc.gear_cylinder_811_sent_time = time.time()
            self.proc.gear_cylinder_811_checking = True
            self.proc.gear_cylinder_811_timeout_reported = False
            self.proc.get_logger().info('启动M531+M533齿轮对接气缸1和2伸出到位检测...')

            self.proc.outbound_state = OutboundState.POST_LIFT_PROCESSING
            self.proc.post_lift_process_start_time = time.time()
            self.proc.post_lift_timeout_reported = False
            self.proc.get_logger().info('出库流程：层移动完成，继续执行后续操作')

        # === POST_LIFT_PROCESSING ===
        elif self.proc.outbound_state == OutboundState.POST_LIFT_PROCESSING:
            # 接驳台入料超时检测
            if (self.proc.post_lift_process_start_time is not None and
                    not self.proc.post_lift_timeout_reported):
                elapsed = time.time() - self.proc.post_lift_process_start_time
                if elapsed > self.proc.POST_LIFT_PROCESS_TIMEOUT:
                    fault_code = FaultCode.CONVEYOR_IN_TIMEOUT.value
                    self.proc.get_logger().error(
                        f'接驳台入料超时({elapsed:.1f}秒>{self.proc.POST_LIFT_PROCESS_TIMEOUT}秒)，'
                        f'发布故障码=0x{fault_code:04X}')
                    self.proc.publish_fault_code(fault_code)
                    return

            if conveyor_in_gap and not self.proc.outbound_conveyor_in_detected:
                self.proc.outbound_conveyor_in_detected = True
                self.proc.get_logger().info('检测到货物进入接驳台(conveyor_in_gap=1)')

            conveyor_out_detected = conveyor_out
            conveyor_in_then_out = (self.proc.outbound_conveyor_in_detected and not conveyor_in_gap)

            # C++层M541防抖(300ms)已确保信号稳定消失, 无需业务层重复延迟
            board_in_position = conveyor_in_then_out or conveyor_out_detected

            if board_in_position:
                trigger_condition = []
                if conveyor_out_detected:
                    trigger_condition.append('conveyor_out')
                if conveyor_in_then_out:
                    trigger_condition.append('conveyor_in变化')
                self.proc.get_logger().info(
                    f'出库检测到板子到位: 触发条件={", ".join(trigger_condition)}')

                self.proc.add_command(ControlAction(
                    CommandType.JOG, "axis2_1", "stop", description="停止轴2_1"))
                self.proc.add_command(ControlAction(
                    CommandType.JOG, "axis2_2", "stop", description="停止轴2_2"))
                self.proc.send_do_control_once("811", False)
                self.proc.send_do_control_once("813", False)
                self.proc.outbound_conveyor_in_detected = False
                self.proc.outbound_conveyor_in_then_out_delay_started = False
                self.proc.post_lift_process_start_time = None
                self.proc.post_lift_timeout_reported = False
                self.proc.outbound_state = OutboundState.CONVEYOR_MOVING
                self.proc.get_logger().info('进入输送带运行状态')

        # === CONVEYOR_MOVING ===
        elif self.proc.outbound_state == OutboundState.CONVEYOR_MOVING:
            if state_changed:
                self._reset_layer_motion_state()
                self.proc.outbound_conveyor_started = False
                self.proc._outbound_conveyor_out_was_true = False
                self.proc.outbound_delay_started = False
                self.proc.state_change_counter = 0

            target_layer = 18 if self.proc.outbound_area == 1 else 1
            self.proc.send_layer_command(target_layer)

            if not self.proc.layer_motion_completed:
                self.proc.state_change_counter += 1
                if self.proc.state_change_counter >= 50:
                    self.proc.state_change_counter = 0
                    self.proc.get_logger().info(f'等待提升机到达第{target_layer}层...')
                return

            if not hasattr(self.proc, 'outbound_conveyor_started') or not self.proc.outbound_conveyor_started:
                self.proc.send_do_control_once("813", True)
                self.proc.outbound_conveyor_started = True
                self.proc.get_logger().info(f'提升机已到达第{target_layer}层，启动输送带')
                self.proc.layer_motion_completed = False
                self.proc.outbound_state = OutboundState.COMPLETED

        # === COMPLETED ===
        elif self.proc.outbound_state == OutboundState.COMPLETED:
            if conveyor_out_gap:
                if not getattr(self.proc, '_outbound_conveyor_out_was_true', False):
                    self.proc._outbound_conveyor_out_was_true = True
                    self.proc.get_logger().info('检测到货物到达出料位(M540)，等待货物离开...')
            else:
                if (getattr(self.proc, '_outbound_conveyor_out_was_true', False) and
                        not self.proc.outbound_delay_started):
                    self.proc.outbound_delay_started = True
                    self.proc.outbound_delay_counter = 0
                    self.proc.get_logger().info(
                        f'货物已离开出料位，开始{self.proc.OUTBOUND_DELAY_BEFORE_STOP_MS // 1000}秒延迟')

            if self.proc.outbound_delay_started:
                self.proc.outbound_delay_counter += 1
                if self.proc.outbound_delay_counter >= self.proc.OUTBOUND_DELAY_COUNTER_MAX:
                    self.proc.send_do_control_once("813", False)
                    self.proc.outbound_delay_started = False
                    self.proc._outbound_conveyor_out_was_true = False
                    self.proc.outbound_completion_delay_started = True
                    self.proc.outbound_completion_delay_counter = 0
                    self.proc.get_logger().info(
                        f'出库延迟结束，启动回包延迟{self.proc.OUTBOUND_COMPLETION_DELAY_SEC}秒')

            if self.proc.outbound_completion_delay_started:
                self.proc.outbound_completion_delay_counter += 1
                if (self.proc.outbound_completion_delay_counter >=
                        self.proc.OUTBOUND_COMPLETION_DELAY_COUNTER_MAX):
                    self.proc.outbound_completion_delay_started = False
                    self.proc.outbound_state = OutboundState.IDLE
                    self.proc.outbound_process_requested = False
                    self.proc.clear_process_timeout('outbound')
                    if not self.proc.outbound_completion_published:
                        from std_msgs.msg import Bool
                        completion_msg = Bool()
                        completion_msg.data = True
                        self.proc.outbound_completed_pub.publish(completion_msg)
                        self.proc.outbound_completion_published = True
                        self.proc.get_logger().info(
                            f'出库流程完成（延迟{self.proc.OUTBOUND_COMPLETION_DELAY_SEC}秒后发布完成消息）')
                    if hasattr(self.proc, 'outbound_conveyor_started'):
                        self.proc.outbound_conveyor_started = False
                    if self.proc.outbound_area == 1:
                        self.proc.send_layer_command(1, fast_return=True)
                        self.proc.get_logger().info('出库区域=1，出库完成后发送回到第1层指令')
                        self.proc.outbound_area = 0

    # ================================================================
    # 放行流程
    # ================================================================
    def process_release_logic(self):
        """处理放行业务流程"""
        di = self.proc.current_io_signals
        conveyor_in_gap = di['conveyor_entry_gap_detect']   # M541 替代 conveyor_in
        conveyor_out_gap = di['conveyor_exit_gap_detect']    # M540 替代 conveyor_out
        buffer_out = di['buffer_out_position']

        state_changed = (self.proc.release_state != self.proc.previous_release_state)
        if state_changed:
            self.proc.get_logger().info(
                f'放行流程状态变化: {self.proc.previous_release_state.name} -> '
                f'{self.proc.release_state.name}')
            self.proc.previous_release_state = self.proc.release_state

        # 停止请求
        if self.proc.release_process_stop_requested:
            self.proc.add_command(ControlAction(
                CommandType.JOG, "axis1_1", "stop", description="停止轴1_1"))
            self.proc.add_command(ControlAction(
                CommandType.JOG, "axis1_2", "stop", description="停止轴1_2"))
            self.proc.send_do_control_once("813", False)
            self.proc.release_state = PassThroughState.IDLE
            self.proc.release_process_stop_requested = False
            self.proc.release_process_requested = False
            self.proc.get_logger().info('放行流程已停止')
            return

        # === IDLE ===
        if self.proc.release_state == PassThroughState.IDLE:
            if self.proc.release_process_requested:
                self.proc._reset_key_do_signals()
                self.proc.release_process_requested = False
                if not self.proc.is_target_layer_reached(1):
                    if self.proc.auto_mode_initialized:
                        self.proc.send_layer_command(1, fast_return=True)
                        self.proc.release_state = PassThroughState.RETURNING_TO_LAYER_1
                        self.proc.get_logger().info(
                            f'放行流程启动，当前层={self.proc.current_layer_float:.2f}，发送层指令回到第1层')
                    else:
                        self.proc.pending_resume_state = {'type': 'release_return_to_layer_1'}
                        self.proc.release_state = PassThroughState.RETURNING_TO_LAYER_1
                        self.proc.get_logger().info(
                            '放行流程启动，等待轴自动模式初始化完成后发送层指令回到第1层')
                else:
                    self.proc.release_state = PassThroughState.WAIT_FOR_PRODUCT_ARRIVAL
                    self.proc.get_logger().info(
                        f'放行流程启动，当前已在第1层({self.proc.current_layer_float:.2f})，'
                        f'进入等待产品到位状态')

        # === RETURNING_TO_LAYER_1 ===
        elif self.proc.release_state == PassThroughState.RETURNING_TO_LAYER_1:
            # 主动消费 pending_resume_state，不依赖外部事件触发
            if self.proc.auto_mode_initialized and self.proc.pending_resume_state is not None:
                self.proc.pause_resume_mgr.execute_pending_resume()
            if self.proc.is_target_layer_reached(1):
                self.proc.release_state = PassThroughState.WAIT_FOR_PRODUCT_ARRIVAL
                self.proc.get_logger().info(
                    f'接驳台已回到第1层({self.proc.current_layer_float:.2f})，进入等待产品到位状态')

        # === WAIT_FOR_PRODUCT_ARRIVAL ===
        elif self.proc.release_state == PassThroughState.WAIT_FOR_PRODUCT_ARRIVAL:
            # 双重判定：事件标志(正常路径) + 物理信号(竞态回退路径)
            # 回退原因：产品到位状态机可能在放行流程启动前就清除了事件标志
            if self.proc.product_arrival_published_in_cycle or buffer_out:
                if buffer_out and not self.proc.product_arrival_published_in_cycle:
                    self.proc.get_logger().info(
                        '放行流程：事件标志已丢失，通过buffer_out物理信号回退启动')
                # 恢复输送带默认速度
                self.proc.send_axis_speed("axis1_1", self.proc.DEFAULT_JOG_SPEEDS["axis1_1"])
                self.proc.send_axis_speed("axis1_2", self.proc.DEFAULT_JOG_SPEEDS["axis1_2"])
                self.proc.add_command(ControlAction(
                    CommandType.JOG, "axis1_1", "reverse",
                    description="启动轴1_1反转（放行）"))
                self.proc.add_command(ControlAction(
                    CommandType.JOG, "axis1_2", "forward",
                    description="启动轴1_2正转（放行）"))
                self.proc.send_do_control_once("813", True)
                self.proc.product_arrival_published_in_cycle = False
                self.proc.release_state = PassThroughState.CONVEYOR_RUNNING
                self.proc.get_logger().info('产品到位检测完成，输送带已启动（放行，消费产品到位标志）')

        # === CONVEYOR_RUNNING ===
        elif self.proc.release_state == PassThroughState.CONVEYOR_RUNNING:
            if not self.proc.release_conveyor_in_completed:
                if conveyor_in_gap:
                    if not self.proc.release_conveyor_in_was_true:
                        self.proc.release_conveyor_in_was_true = True
                        self.proc.get_logger().info('放行流程：检测到货物进入输送带(M541=1)')
                else:
                    if self.proc.release_conveyor_in_was_true:
                        self.proc.release_conveyor_in_completed = True
                        # 板子已确认完全进入输送带(M541 无->有->无 完成),
                        # 回传事件闸门: 接驳台已空, 可接收下一块板的0x0109发布.
                        self.proc.board_dispatched = True
                        self.proc.get_logger().info(
                            '放行流程：货物已完全进入输送带(M541=0)，开始检测M540')

            if self.proc.release_conveyor_in_completed:
                if conveyor_out_gap:
                    if not self.proc.release_conveyor_out_was_true:
                        self.proc.release_conveyor_out_was_true = True
                        self.proc.get_logger().info('放行流程：检测到货物到达出料位(M540=1)')
                else:
                    if self.proc.release_conveyor_out_was_true:
                        self.proc.get_logger().info('放行流程：检测到货物离开出料位(M540=0)，开始延迟')
                        self.proc.release_conveyor_out_delay_started = True
                        self.proc.release_conveyor_out_delay_counter = 0
                        self.proc.release_state = PassThroughState.WAIT_FOR_CONVEYOR_OUT

        # === WAIT_FOR_CONVEYOR_OUT ===
        elif self.proc.release_state == PassThroughState.WAIT_FOR_CONVEYOR_OUT:
            if self.proc.release_conveyor_out_delay_started:
                self.proc.release_conveyor_out_delay_counter += 1
                # M540防抖(300ms)已确认板子完全离开, 1周期(100ms)余量足够
                if self.proc.release_conveyor_out_delay_counter >= 1:
                    self.proc.add_command(ControlAction(
                        CommandType.JOG, "axis1_1", "stop", description="停止轴1_1"))
                    self.proc.add_command(ControlAction(
                        CommandType.JOG, "axis1_2", "stop", description="停止轴1_2"))
                    self.proc.send_do_control_once("813", False)
                    self.proc.release_conveyor_out_delay_started = False
                    self.proc.release_conveyor_out_was_true = False
                    self.proc.release_conveyor_in_was_true = False
                    self.proc.release_conveyor_in_completed = False
                    self.proc.release_state = PassThroughState.COMPLETED
                    self.proc.get_logger().info('放行流程：延迟结束，进入完成状态')

        # === COMPLETED ===
        elif self.proc.release_state == PassThroughState.COMPLETED:
            if not self.proc.release_completion_published:
                from std_msgs.msg import Bool
                completion_msg = Bool()
                completion_msg.data = True
                self.proc.release_completed_pub.publish(completion_msg)
                self.proc.release_completion_published = True
                self.proc.get_logger().info('放行流程完成，发布完成消息')
            self.proc.release_process_requested = False
            self.proc.release_completion_published = False
            self.proc.release_state = PassThroughState.IDLE
            if self.proc.product_arrival_cycle_active:
                self.proc.product_arrival_published_in_cycle = False
                # 不在此处重置状态机: 放行刚送走板子, 但buffer_out可能残留/抖动,
                # reset回IDLE会被IDLE的buffer_out恢复逻辑误判为"需补发",
                # 或在PENDING_PUBLISH期间板子已离开时产生幽灵0x109。
                # 状态机保持COMPLETED, 由COMPLETED自然流转
                # (not feed_detect and not buffer_out) 回IDLE。

    # ================================================================
    # 内部辅助
    # ================================================================
    def _reset_layer_motion_state(self):
        self.proc.layer_motion_completed = False
        self.proc.previous_layer_completion_state = False
        self.proc.layer_completion_received_time = None
