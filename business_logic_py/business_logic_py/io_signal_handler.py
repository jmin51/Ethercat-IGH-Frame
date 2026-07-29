#!/usr/bin/env python3
# ======================================================================
# IO信号处理 & SMEMA协议 & 产品到位检测
# 负责：IO解析、信号映射、SMEMA握手、产品到位状态机
# ======================================================================
import re
import time
from typing import Dict

from .models import (
    CommandType, ControlAction,
    WarehouseState, OutboundState, PassThroughState, FaultCode
)


class IoSignalHandler:
    """IO信号解析与SMEMA协议处理 - 无ROS依赖的纯逻辑单元"""

    def __init__(self, processor):
        """绑定处理器实例（反向引用，避免重复传递状态）"""
        self.proc = processor

    # ================================================================
    # IO信号解析
    # ================================================================
    def parse_io_status(self, io_data: str) -> Dict[str, bool]:
        """解析IO状态字符串为字典（包含DI和DO信号）"""
        io_signals = {}
        try:
            di_matches = re.findall(r'DI(\d+):(\d)', io_data)
            for di_num, value in di_matches:
                signal_name = self.map_signal_name(int(di_num))
                if signal_name:
                    io_signals[signal_name] = (value == '1')

            do_matches = re.findall(r'DO(\d+):(\d)', io_data)
            for do_num, value in do_matches:
                do_num_int = int(do_num)
                if do_num_int == 14:
                    io_signals['smema_mr'] = (value == '1')
                elif do_num_int == 15:
                    io_signals['smema_ba'] = (value == '1')
        except Exception as e:
            self.proc.get_logger().error(f'IO状态解析失败: {e}')
        return io_signals

    @staticmethod
    def map_signal_name(di_number: int) -> str:
        """映射DI编号到标准信号名称"""
        mapping = {
            8: 'feed_product_detect',
            10: 'buffer_sensor_2',
            11: 'buffer_in_position',
            12: 'buffer_out_position',
            13: 'conveyor_in_position',
            14: 'conveyor_out_position',
            28: 'conveyor_exit_gap_detect',   # M540 接驳台出料检测(缝隙)
            29: 'conveyor_entry_gap_detect',  # M541 接驳台入料检测(缝隙)
            19: 'gear_cylinder_1_in_position',
            21: 'gear_cylinder_2_in_position',
            23: 'smema_uba',
            24: 'smema_dbr',
        }
        return mapping.get(di_number, '')

    # ================================================================
    # 产品到位检测状态机
    # ================================================================
    def process_product_arrival_logic(self):
        """处理产品到位检测状态机

        状态流转：
        IDLE → CONVEYOR_RUNNING: feed_product_detect上升沿
        CONVEYOR_RUNNING → WAITING_BUFFER_OUT: buffer_in上升沿
        WAITING_BUFFER_OUT → COMPLETED: buffer_out有信号且输送带空闲，发布0x0109
        WAITING_BUFFER_OUT → PENDING_PUBLISH: buffer_out有信号但输送带被占用，延迟发布
        PENDING_PUBLISH → COMPLETED: 输送带释放后，发布0x0109
        COMPLETED → IDLE: feed_detect消失，准备下一轮
        """
        if not self.proc.product_arrival_cycle_active:
            return

        # ========== 输送带占用状态 ==========
        # 不冻结整个状态机，仅用于决定：是否启动输送带、是否立即发布0x0109
        # COMPLETED但升降机未归位第1层时仍占用——防止新板推入未归位接驳台
        warehouse_completed_but_not_home = (
            self.proc.warehouse_state == WarehouseState.COMPLETED and
            not self.proc.is_target_layer_reached(1))
        conveyor_occupied_by_warehouse = (
            self.proc.warehouse_state == WarehouseState.CONVEYOR_MOVING or
            warehouse_completed_but_not_home)
        conveyor_occupied_by_release = (
            self.proc.release_state == PassThroughState.CONVEYOR_RUNNING)
        conveyor_occupied = conveyor_occupied_by_warehouse or conveyor_occupied_by_release

        di = self.proc.current_io_signals
        feed_detect = di['feed_product_detect']
        buffer_in = di['buffer_in_position']
        buffer_out = di['buffer_out_position']

        # === IDLE ===
        if self.proc.product_arrival_state == "IDLE":
            # 报错/结束作业恢复：buffer_out已有板 → 直接跳PENDING_PUBLISH补发0x0109
            if buffer_out:
                self.proc.product_arrival_state = "PENDING_PUBLISH"
                self.proc.get_logger().info(
                    '产品到位检测：IDLE检测到buffer_out已有板，跳转PENDING_PUBLISH')
            elif feed_detect and not self.proc.feed_detected:
                self.proc.feed_detected = True
                self.proc.product_arrival_state = "CONVEYOR_RUNNING"
                if not conveyor_occupied and not buffer_out:
                    # 输送带空闲，启动输送带
                    self.proc.send_axis_speed("axis1_1", self.proc.DEFAULT_JOG_SPEEDS["axis1_1"])
                    self.proc.send_axis_speed("axis1_2", self.proc.DEFAULT_JOG_SPEEDS["axis1_2"])
                    self.proc.add_command(ControlAction(
                        CommandType.JOG, "axis1_1", "reverse",
                        description="进料：启动轴1_1反转"
                    ))
                    self.proc.add_command(ControlAction(
                        CommandType.JOG, "axis1_2", "forward",
                        description="进料：启动轴1_2正转"
                    ))
                    self.proc.conveyor_started_for_arrival = True
                    self.proc.get_logger().info(
                        '产品到位检测：检测到feed_product_detect，启动输送带')
                else:
                    # 输送带被入库/放行占用，PCB随现有输送带运动
                    self.proc.conveyor_started_for_arrival = False
                    self.proc.get_logger().info(
                        '产品到位检测：检测到feed_product_detect，'
                        '输送带被占用，PCB随现有输送带运动')

        # === CONVEYOR_RUNNING ===
        elif self.proc.product_arrival_state == "CONVEYOR_RUNNING":
            # 补启输送带：仅处理"从未启动"的场景
            # buffer_in是电平信号，作为重启条件会导致100ms风暴
            need_restart = (
                not conveyor_occupied and not buffer_out and
                not self.proc.conveyor_started_for_arrival)
            if need_restart:
                self.proc.send_axis_speed("axis1_1", self.proc.DEFAULT_JOG_SPEEDS["axis1_1"])
                self.proc.send_axis_speed("axis1_2", self.proc.DEFAULT_JOG_SPEEDS["axis1_2"])
                self.proc.add_command(ControlAction(
                    CommandType.JOG, "axis1_1", "reverse",
                    description="进料：补启轴1_1反转"
                ))
                self.proc.add_command(ControlAction(
                    CommandType.JOG, "axis1_2", "forward",
                    description="进料：补启轴1_2正转"
                ))
                self.proc.conveyor_started_for_arrival = True
                self.proc.get_logger().info(
                    '产品到位检测：输送带已释放，补启输送带')

            if buffer_in and not self.proc.buffer_in_detected:
                self.proc.buffer_in_detected = True
                self.proc.product_arrival_state = "WAITING_BUFFER_OUT"
                self.proc.get_logger().info(
                    '产品到位检测：检测到buffer_in，等待产品到达buffer_out')

        # === WAITING_BUFFER_OUT ===
        elif self.proc.product_arrival_state == "WAITING_BUFFER_OUT":
            # 补启输送带：仅处理"从未启动"的场景
            # buffer_in是电平信号，作为重启条件会导致100ms风暴
            need_restart = (
                not conveyor_occupied and not buffer_out and
                not self.proc.conveyor_started_for_arrival)
            if need_restart:
                self.proc.send_axis_speed("axis1_1", self.proc.DEFAULT_JOG_SPEEDS["axis1_1"])
                self.proc.send_axis_speed("axis1_2", self.proc.DEFAULT_JOG_SPEEDS["axis1_2"])
                self.proc.add_command(ControlAction(
                    CommandType.JOG, "axis1_1", "reverse",
                    description="进料：补启轴1_1反转(等待buffer_out)"
                ))
                self.proc.add_command(ControlAction(
                    CommandType.JOG, "axis1_2", "forward",
                    description="进料：补启轴1_2正转(等待buffer_out)"
                ))
                self.proc.conveyor_started_for_arrival = True
                self.proc.get_logger().info(
                    '产品到位检测：输送带已释放，补启输送带(等待buffer_out)')

            if buffer_out:
                # 产品到达buffer_out → 立即停轴，无论输送带是否占用
                # 停轴是物理安全需求，发布0x0109才是流程协调需求
                self._stop_arrival_conveyor()
                if not conveyor_occupied:
                    # 输送带空闲：立即发布0x0109
                    self._publish_arrival()
                else:
                    # 输送带被占用：延迟发布
                    self.proc.product_arrival_state = "PENDING_PUBLISH"
                    self.proc.get_logger().info(
                        '产品到位检测：PCB已到达buffer_out，'
                        '但输送带被占用，延迟发布0x0109')

        # === PENDING_PUBLISH ===
        elif self.proc.product_arrival_state == "PENDING_PUBLISH":
            if not conveyor_occupied:
                if buffer_out:
                    # 输送带已释放且板子仍在buffer_out: 发布0x0109
                    self._publish_arrival()
                    self.proc.get_logger().info(
                        '产品到位检测：输送带已释放，发布延迟的0x0109')
                else:
                    # 板子已离开buffer_out — 取消发布, 回IDLE等下一块板
                    self.proc.product_arrival_state = "IDLE"
                    self.proc.feed_detected = False
                    self.proc.buffer_in_detected = False
                    self.proc.get_logger().info(
                        '产品到位检测：PENDING_PUBLISH期间板子已离开buffer_out, 取消发布')

        # === COMPLETED ===
        elif self.proc.product_arrival_state == "COMPLETED":
            # 三重前置门: 信号归零 + 板已被物理确认送走(事件闸门)
            # 仅 not feed_detect and not buffer_out 会被毛刺击穿,
            # board_dispatched 是流程完成时回传的单调事件, 不可伪造, 杜绝幽灵补发.
            if not feed_detect and not buffer_out and self.proc.board_dispatched:
                self.proc.product_arrival_state = "IDLE"
                self.proc.feed_detected = False
                self.proc.buffer_in_detected = False
                # 原则：发布者不清除事件标志，消费者负责清除
                # product_arrival_published_in_cycle 由入库/放行流程消费后清除
                # 不再在此处判断"是否被使用"并清除，避免竞态窗口
                self.proc.board_dispatched = False   # 消费闸门, 准备下一轮
                self.proc.get_logger().info('产品到位检测：重置状态机，等待下一轮')

    def _stop_arrival_conveyor(self):
        """停止由产品到位检测启动的输送带（立即发送，绕过命令队列）"""
        if self.proc.conveyor_started_for_arrival:
            self.proc.send_jog_command("axis1_1:stop")
            self.proc.send_jog_command("axis1_2:stop")
            self.proc.get_logger().info('进料：立即停止轴1_1和轴1_2（绕过队列）')
            self.proc.conveyor_started_for_arrival = False

    def _publish_arrival(self):
        """发布产品到位信号0x0109"""
        # 幂等兜底: 本轮已发布则拒绝重复发布.
        # 第一道防线是 board_dispatched 闸门锁死 COMPLETED->IDLE,
        # 此处为纵深防御, 防止任何路径绕过闸门二次触发.
        if self.proc.product_arrival_published_in_cycle:
            self.proc.get_logger().warn(
                '产品到位检测：本轮0x0109已发布，拒绝重复发布')
            return
        from std_msgs.msg import Bool
        arrival_msg = Bool()
        arrival_msg.data = True
        self.proc.product_arrival_pub.publish(arrival_msg)
        self.proc.product_arrival_published_in_cycle = True
        self.proc.product_arrival_state = "COMPLETED"
        self.proc.get_logger().info(
            f'产品到位检测完成，发布0x0109（阶段: {self.proc.product_arrival_phase}）')

    def reset_product_arrival_state_machine(self):
        """重置产品到位检测状态机"""
        self.proc.product_arrival_state = "IDLE"
        self.proc.feed_detected = False
        self.proc.buffer_in_detected = False
        self.proc.feed_detect_was_true = False
        self.proc.buffer_out_was_true_for_arrival = False
        self.proc.conveyor_in_was_true_for_arrival = False
        self.proc.conveyor_in_completed_for_arrival = False
        if self.proc.conveyor_started_for_arrival:
            self.proc.add_command(ControlAction(
                CommandType.JOG, "axis1_1", "stop",
                description="重置：停止轴1_1"
            ))
            self.proc.add_command(ControlAction(
                CommandType.JOG, "axis1_2", "stop",
                description="重置：停止轴1_2"
            ))
            self.proc.conveyor_started_for_arrival = False
        self.proc.get_logger().debug('产品到位检测状态机已重置')

    # ================================================================
    # SMEMA协议处理
    # ================================================================
    def get_upstream_handshake_state(self) -> str:
        """根据IO信号判断上游握手状态"""
        if not self.proc.ENABLE_SMEMA:
            return "DISABLED"
        uba = self.proc.current_io_signals.get('smema_uba', False)
        mr = self.proc.current_io_signals.get('smema_mr', False)
        if not mr:
            return "UPSTREAM_IDLE"
        elif not uba and mr:
            if hasattr(self.proc, '_last_uba') and self.proc._last_uba and not uba:
                return "UPSTREAM_BOARD_ARRIVED"
            else:
                return "UPSTREAM_READY"
        elif uba and mr:
            return "UPSTREAM_RECEIVING"
        else:
            return "UPSTREAM_IDLE"

    def get_downstream_handshake_state(self) -> str:
        """根据IO信号判断下游握手状态"""
        if not self.proc.ENABLE_SMEMA:
            return "DISABLED"
        dbr = self.proc.current_io_signals.get('smema_dbr', False)
        ba = self.proc.current_io_signals.get('smema_ba', False)
        if not ba:
            return "DOWNSTREAM_IDLE"
        elif ba and not dbr:
            if hasattr(self.proc, '_last_dbr') and self.proc._last_dbr and not dbr:
                return "DOWNSTREAM_SENT"
            else:
                return "DOWNSTREAM_AVAILABLE"
        elif ba and dbr:
            return "DOWNSTREAM_SENDING"
        else:
            return "DOWNSTREAM_IDLE"

    def update_product_position(self):
        """更新产品到位信号（驱动SMEMA握手）"""
        if not self.proc.ENABLE_SMEMA:
            return

        # 未收到0x0105开始作业 / 报错 / 结束作业 → 禁止要板
        # product_in_position=True → C++ smema_set_product_in_position(True)
        # → SMEMA状态机: product_in_position=true → MR=OFF → 不要板
        if not self.proc.product_arrival_cycle_active:
            from std_msgs.msg import Bool
            msg = Bool()
            msg.data = True
            self.proc.product_position_pub.publish(msg)
            return

        di = self.proc.current_io_signals
        feed_detect = di['feed_product_detect']
        buffer_in = di['buffer_in_position']
        buffer_out = di['buffer_out_position']
        conveyor_in = di['conveyor_in_position']

        # 信号跟踪
        if feed_detect:
            self.proc.feed_detect_was_true = True
        if buffer_out:
            self.proc.buffer_out_was_true_for_arrival = True
        if conveyor_in:
            self.proc.conveyor_in_was_true_for_arrival = True
        elif self.proc.conveyor_in_was_true_for_arrival and not conveyor_in:
            if not buffer_out and self.proc.buffer_out_was_true_for_arrival:
                self.proc.conveyor_in_completed_for_arrival = True

        # ================================================================
        # 要板信号判定 — 配合 0x0109 周期, 以 IDLE 为唯一要板门控
        #
        # 设计原则:
        #   MR=ON 的唯一条件: product_arrival_state == "IDLE"
        #   COMPLETED/PENDING_PUBLISH 表示板子在基准层, 禁止要板
        #   COMPLETED->IDLE 的 board_dispatched 门控已保证接驳台空闲
        #
        # 出库期间允许要板: 出库板路径(conveyor_in->conveyor_out)不经过
        # 基准层(buffer_out), 新板(feed_detect->buffer_out)与出库板物理路径
        # 不重叠, 可并行作业
        # ================================================================
        no_board_reasons = []
        if feed_detect:
            no_board_reasons.append("进料检测中")
        if buffer_in:
            no_board_reasons.append("缓存架入料口有板")
        if buffer_out:
            no_board_reasons.append("缓存架出料口有板")
        if self.proc.product_arrival_state != "IDLE":
            no_board_reasons.append(
                f"产品到位状态非IDLE({self.proc.product_arrival_state})")
        if self.proc.product_arrival_published_in_cycle:
            no_board_reasons.append("产品在缓存架基准层")
        if self.proc.warehouse_state == WarehouseState.CONVEYOR_MOVING:
            no_board_reasons.append("入库传输中")
        if self.proc.release_state in (PassThroughState.CONVEYOR_RUNNING,
                                        PassThroughState.WAIT_FOR_CONVEYOR_OUT):
            no_board_reasons.append("放行传输中")

        product_in_position = len(no_board_reasons) > 0
        can_request_board = not product_in_position

        from std_msgs.msg import Bool
        msg = Bool()
        msg.data = product_in_position
        self.proc.product_position_pub.publish(msg)

        if not hasattr(self.proc, '_last_product_position'):
            self.proc._last_product_position = None
        if product_in_position != self.proc._last_product_position:
            state_info = ("产品已到达接驳台" if can_request_board
                          else " | ".join(no_board_reasons) if no_board_reasons else "无板")
            self.proc.get_logger().info(
                f'产品到位信号: {product_in_position} - {state_info} - '
                f'{"不要板" if product_in_position else "可以要板"}')
            self.proc._last_product_position = product_in_position

    def check_smema_handshake(self):
        """检查SMEMA握手状态"""
        if not self.proc.ENABLE_SMEMA:
            return
        upstream_state = self.get_upstream_handshake_state()
        if upstream_state == "UPSTREAM_BOARD_ARRIVED":
            self.proc.get_logger().info('上游板子到达')
        downstream_state = self.get_downstream_handshake_state()
        if downstream_state == "DOWNSTREAM_SENT":
            self.proc.get_logger().info('下游板子发送完成')
        self.proc._last_uba = self.proc.current_io_signals.get('smema_uba', False)
        self.proc._last_dbr = self.proc.current_io_signals.get('smema_dbr', False)
