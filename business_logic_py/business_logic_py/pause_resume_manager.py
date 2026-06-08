#!/usr/bin/env python3
# ======================================================================
# 暂停/恢复状态管理
# 负责：暂停状态记录、恢复命令解析、状态还原、待执行恢复调度
# ======================================================================
from .models import WarehouseState, OutboundState


class PauseResumeManager:
    """暂停恢复管理器 - 解耦暂停记录/恢复逻辑与业务状态机"""

    def __init__(self, processor):
        self.proc = processor

    def report_current_pause_state(self):
        """报告当前业务状态供C++端记录"""
        self.proc.saved_warehouse_state = self.proc.warehouse_state
        self.proc.saved_outbound_state = self.proc.outbound_state

        # IDLE/COMPLETED使用实际层号（避免残留目标层干扰）
        if self.proc.warehouse_state in [WarehouseState.IDLE, WarehouseState.COMPLETED]:
            actual_layer = max(1, min(28, round(self.proc.current_layer_float)))
            self.proc.saved_target_layer = actual_layer
            self.proc.get_logger().info(
                f'记录暂停状态：入库{self.proc.warehouse_state.name}，'
                f'使用实际层号={actual_layer} (浮点层={self.proc.current_layer_float:.2f})')
        else:
            self.proc.saved_target_layer = self.proc.target_layer

        if self.proc.outbound_state in [OutboundState.IDLE, OutboundState.COMPLETED]:
            actual_layer = max(1, min(28, round(self.proc.current_layer_float)))
            self.proc.saved_source_layer = actual_layer
            self.proc.get_logger().info(
                f'记录暂停状态：出库{self.proc.outbound_state.name}，'
                f'使用实际层号={actual_layer} (浮点层={self.proc.current_layer_float:.2f})')
        else:
            self.proc.saved_source_layer = self.proc.source_layer

        warehouse_active = 1 if self.proc.warehouse_state != WarehouseState.IDLE else 0
        outbound_active = 1 if self.proc.outbound_state != OutboundState.IDLE else 0

        report = (f"warehouse_active={warehouse_active},"
                  f"warehouse_state={self.proc.warehouse_state.value},"
                  f"warehouse_layer={self.proc.saved_target_layer},"
                  f"outbound_active={outbound_active},"
                  f"outbound_state={self.proc.outbound_state.value},"
                  f"outbound_layer={self.proc.saved_source_layer}")

        from std_msgs.msg import String
        msg = String()
        msg.data = report
        self.proc.pause_state_report_pub.publish(msg)
        self.proc.pause_state_reported = True
        self.proc.get_logger().info(f'已报告暂停状态: {report}')

    def handle_resume_command(self, command: str):
        """处理恢复命令，重走之前记录的状态"""
        # 暂停期间收到结束作业 → 放弃恢复，三个流程保持IDLE
        if self.proc.operation_ended_during_pause:
            self.proc.operation_ended_during_pause = False
            self.proc.pause_state_reported = False
            self.proc.get_logger().info('暂停期间收到结束作业(0x0107)，放弃恢复之前流程状态')
            return

        self.proc.get_logger().info(f'处理恢复命令: {command}')
        try:
            params = {}
            parts = command.replace("RESUME:", "").split(",")
            for part in parts:
                key, value = part.split("=")
                params[key] = int(value)

            self.proc.resuming_from_pause = True

            if params.get('warehouse_active', 0):
                self.restore_warehouse_state(
                    params.get('warehouse_state', 1),
                    params.get('warehouse_layer', 1))

            if params.get('outbound_active', 0):
                self.restore_outbound_state(
                    params.get('outbound_state', 1),
                    params.get('outbound_layer', 1))

            self.proc.pause_state_reported = False
            self.proc.get_logger().info('业务逻辑状态恢复完成，继续执行')
        except Exception as e:
            self.proc.get_logger().error(f'处理恢复命令失败: {e}')

    def restore_warehouse_state(self, state_value, target_layer):
        """恢复入库流程到指定状态"""
        self.proc.get_logger().info(f'恢复入库流程: 状态={state_value}, 目标层={target_layer}')
        self.proc.layer_command_sent = False
        self.proc.last_layer_command = None
        self.proc.target_layer = target_layer

        if state_value == WarehouseState.IDLE.value:
            self.proc.warehouse_state = WarehouseState.IDLE
            if not self.proc.is_target_layer_reached(1):
                if self.proc.auto_mode_initialized:
                    self.proc.send_layer_command(1)
                    self.proc.get_logger().info(
                        f'从IDLE恢复，当前层={self.proc.current_layer_float:.2f}，发送层指令回到第1层')
                else:
                    self.proc.pending_resume_state = {'type': 'warehouse_idle', 'layer': 1}
                    self.proc.get_logger().info('从IDLE恢复，等待轴自动模式初始化完成后发送层指令回到第1层')
            else:
                self.proc.get_logger().info(
                    f'从IDLE恢复，当前已在第1层({self.proc.current_layer_float:.2f})，无需移动')

        elif state_value == WarehouseState.WAIT_FOR_ENTRY.value:
            self.proc.warehouse_state = WarehouseState.WAIT_FOR_ENTRY
            self.proc.warehouse_process_requested = True
            self.proc.warehouse_completion_published = False
            self.proc.get_logger().info('从WAIT_FOR_ENTRY恢复，重置完成发布标志')

        elif state_value == WarehouseState.CONVEYOR_MOVING.value:
            self.proc.warehouse_state = WarehouseState.WAIT_FOR_ENTRY
            self.proc.warehouse_process_requested = True
            self.proc.warehouse_completion_published = False
            self.proc.get_logger().info('从CONVEYOR_MOVING恢复，将重新检测入库条件，重置完成发布标志')

        elif state_value == WarehouseState.LIFT_MOVING.value:
            self.proc.warehouse_state = WarehouseState.LIFT_MOVING
            self.proc.warehouse_process_requested = True
            self._reset_layer_motion_state()
            self.proc.warehouse_completion_published = False
            if self.proc.auto_mode_initialized:
                self.proc.send_layer_command(self.proc.target_layer)
                self.proc.get_logger().info(
                    f'从LIFT_MOVING恢复，重新发送层指令到目标层 {self.proc.target_layer}')
            else:
                self.proc.pending_resume_state = {
                    'type': 'warehouse',
                    'state': WarehouseState.LIFT_MOVING,
                    'layer': self.proc.target_layer}
                self.proc.get_logger().info(
                    f'从LIFT_MOVING恢复，等待轴自动模式初始化完成后发送层指令到目标层 {self.proc.target_layer}')

        elif state_value == WarehouseState.POST_LIFT_PROCESSING.value:
            self.proc.warehouse_state = WarehouseState.LIFT_MOVING
            self.proc.warehouse_process_requested = True
            self.proc.layer_motion_completed = True
            self.proc.post_lift_delay_start = None
            self.proc.warehouse_completion_published = False
            self.proc.get_logger().info('从POST_LIFT_PROCESSING恢复，将重新执行后续操作，重置完成发布标志')

        elif state_value == WarehouseState.DELAY_PROCESSING.value:
            self.proc.warehouse_state = WarehouseState.POST_LIFT_PROCESSING
            self.proc.warehouse_process_requested = True
            self.proc.layer_motion_completed = True
            self.proc.post_lift_delay_start = None
            self.proc.delay_started = False
            self.proc.delay_condition_triggered = False
            self.proc.buffer_sensor_2_detected = False
            self.proc.reset_do_command_state("811")
            self.proc.reset_do_command_state("812")
            self.proc.warehouse_completion_published = False
            self.proc.get_logger().info('从DELAY_PROCESSING恢复，将重新执行延迟处理，重置完成发布标志')

        elif state_value == WarehouseState.COMPLETED.value:
            self.proc.warehouse_state = WarehouseState.COMPLETED
            self.proc.warehouse_process_requested = True
            self.proc.warehouse_completion_published = False
            self._reset_layer_motion_state()
            if self.proc.auto_mode_initialized:
                self.proc.send_layer_command(1)
                self.proc.get_logger().info('从COMPLETED恢复，发送层指令回到第1层')
            else:
                self.proc.pending_resume_state = {'type': 'warehouse_completed', 'layer': 1}
                self.proc.get_logger().info('从COMPLETED恢复，等待轴自动模式初始化完成后发送层指令回到第1层')
            self.proc.get_logger().info(
                '从COMPLETED恢复，重置完成发布标志和层移动状态，确保能正确回包102并回到第1层')

        self.proc.resuming_from_pause = False

    def restore_outbound_state(self, state_value, source_layer):
        """恢复出库流程到指定状态"""
        self.proc.get_logger().info(f'恢复出库流程: 状态={state_value}, 源层={source_layer}')
        self.proc.layer_command_sent = False
        self.proc.last_layer_command = None
        self.proc.source_layer = source_layer

        if state_value == OutboundState.IDLE.value:
            self.proc.outbound_state = OutboundState.IDLE
            if not self.proc.is_target_layer_reached(1):
                if self.proc.auto_mode_initialized:
                    self.proc.send_layer_command(1)
                    self.proc.get_logger().info(
                        f'出库从IDLE恢复，当前层={self.proc.current_layer_float:.2f}，发送层指令回到第1层')
                else:
                    self.proc.pending_resume_state = {'type': 'outbound_idle', 'layer': 1}
                    self.proc.get_logger().info('出库从IDLE恢复，等待轴自动模式初始化完成后发送层指令回到第1层')
            else:
                self.proc.get_logger().info(
                    f'出库从IDLE恢复，当前已在第1层({self.proc.current_layer_float:.2f})，无需移动')

        elif state_value == OutboundState.WAIT_FOR_EXIT.value:
            self.proc.outbound_state = OutboundState.WAIT_FOR_EXIT
            self.proc.outbound_process_requested = True
            self.proc.outbound_completion_published = False
            self.proc.get_logger().info('从WAIT_FOR_EXIT恢复，重置完成发布标志')

        elif state_value == OutboundState.LIFT_MOVING.value:
            self.proc.outbound_state = OutboundState.LIFT_MOVING
            self.proc.outbound_process_requested = True
            self._reset_layer_motion_state()
            self.proc.outbound_completion_published = False
            if self.proc.auto_mode_initialized:
                self.proc.send_layer_command(self.proc.source_layer)
                self.proc.get_logger().info(
                    f'从LIFT_MOVING恢复，重新发送层指令到源层 {self.proc.source_layer}')
            else:
                self.proc.pending_resume_state = {
                    'type': 'outbound',
                    'state': OutboundState.LIFT_MOVING,
                    'layer': self.proc.source_layer}
                self.proc.get_logger().info(
                    f'从LIFT_MOVING恢复，等待轴自动模式初始化完成后发送层指令到源层 {self.proc.source_layer}')

        elif state_value == OutboundState.POST_LIFT_PROCESSING.value:
            self.proc.outbound_process_requested = True
            self.proc.reset_do_command_state("811")
            self.proc.outbound_completion_published = False
            if not self.proc.auto_mode_initialized:
                self.proc.outbound_state = OutboundState.LIFT_MOVING
                self.proc.pending_resume_state = {
                    'type': 'outbound_post_lift',
                    'source_layer': source_layer}
                self.proc.get_logger().info('从POST_LIFT_PROCESSING恢复，等待轴自动模式初始化完成后执行')
            else:
                self.proc.outbound_state = OutboundState.LIFT_MOVING
                self.proc.layer_motion_completed = True
                self.proc.get_logger().info('从POST_LIFT_PROCESSING恢复，轴已就绪，将重新执行后续操作')

        elif state_value == OutboundState.CONVEYOR_MOVING.value:
            self.proc.outbound_state = OutboundState.CONVEYOR_MOVING
            self.proc.outbound_process_requested = True
            self._reset_layer_motion_state()
            self.proc.outbound_completion_published = False
            if self.proc.auto_mode_initialized:
                self.proc.send_layer_command(1, fast_return=True)
                self.proc.get_logger().info('从CONVEYOR_MOVING恢复，立即发送层指令到第1层')
            else:
                self.proc.pending_resume_state = {
                    'type': 'outbound_conveyor',
                    'state': OutboundState.CONVEYOR_MOVING,
                    'layer': 1}
                self.proc.get_logger().info('从CONVEYOR_MOVING恢复，等待轴自动模式初始化完成后发送层指令到第1层')

        elif state_value == OutboundState.COMPLETED.value:
            self.proc.outbound_state = OutboundState.COMPLETED
            self.proc.outbound_process_requested = True
            self.proc.outbound_completion_published = False
            self._reset_layer_motion_state()
            self.proc.get_logger().info('从COMPLETED恢复，重置完成发布标志和层移动状态，确保能正确回包104并回到第1层')

        self.proc.resuming_from_pause = False

    def execute_pending_resume(self):
        """执行待处理的恢复状态（轴就绪后调用）"""
        if self.proc.pending_resume_state is None:
            return

        state_info = self.proc.pending_resume_state
        self.proc.pending_resume_state = None

        resume_type = state_info['type']

        if resume_type == 'warehouse':
            self.proc.warehouse_state = WarehouseState.LIFT_MOVING
            self.proc.warehouse_process_requested = True
            self._reset_layer_motion_state()
            self.proc.send_layer_command(state_info['layer'])
            self.proc.get_logger().info(
                f'轴就绪后执行入库恢复：恢复状态到LIFT_MOVING，发送层指令到目标层 {state_info["layer"]}')

        elif resume_type == 'warehouse_completed':
            target_layer = state_info.get('layer', 1)
            self.proc.warehouse_state = WarehouseState.IDLE
            self.proc.warehouse_process_requested = False
            self._reset_layer_motion_state()
            self.proc.send_layer_command(target_layer, fast_return=(target_layer == 1))
            self.proc.get_logger().info(
                f'轴就绪后执行入库完成恢复：恢复状态到IDLE，发送层指令回到第{target_layer}层')

        elif resume_type == 'warehouse_idle':
            target_layer = state_info.get('layer', 1)
            self.proc.send_layer_command(target_layer, fast_return=(target_layer == 1))
            self.proc.get_logger().info(f'轴就绪后执行入库IDLE恢复：发送层指令回到第{target_layer}层')

        elif resume_type == 'outbound':
            self.proc.outbound_state = OutboundState.LIFT_MOVING
            self.proc.outbound_process_requested = True
            self._reset_layer_motion_state()
            self.proc.send_layer_command(state_info['layer'])
            self.proc.get_logger().info(
                f'轴就绪后执行出库恢复：恢复状态到LIFT_MOVING，发送层指令到源层 {state_info["layer"]}')

        elif resume_type == 'outbound_idle':
            target_layer = state_info.get('layer', 1)
            self.proc.send_layer_command(target_layer, fast_return=(target_layer == 1))
            self.proc.get_logger().info(f'轴就绪后执行出库IDLE恢复：发送层指令回到第{target_layer}层')

        elif resume_type == 'outbound_conveyor':
            self.proc.outbound_state = OutboundState.CONVEYOR_MOVING
            self.proc.outbound_process_requested = True
            self._reset_layer_motion_state()
            self.proc.outbound_conveyor_started = False
            target_layer = state_info['layer']
            self.proc.send_layer_command(target_layer, fast_return=(target_layer == 1))
            self.proc.get_logger().info(
                f'轴就绪后执行出库恢复：恢复状态到CONVEYOR_MOVING，发送层指令到第{target_layer}层')

        elif resume_type == 'outbound_post_lift':
            self.proc.outbound_state = OutboundState.LIFT_MOVING
            self.proc.layer_motion_completed = True
            self.proc.get_logger().info('轴就绪后执行出库POST_LIFT恢复：将进入POST_LIFT_PROCESSING状态执行JOG命令')

        elif resume_type == 'release_return_to_layer_1':
            self.proc.send_layer_command(1)
            self.proc.get_logger().info('轴就绪后执行放行恢复：发送层指令回到第1层')

    # ================================================================
    # 内部辅助
    # ================================================================
    def _reset_layer_motion_state(self):
        """重置层移动完成状态"""
        self.proc.layer_motion_completed = False
        self.proc.previous_layer_completion_state = False
        self.proc.layer_completion_received_time = None
