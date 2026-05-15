#!/usr/bin/env python3
# ======================================================================
# 数据模型定义 - 枚举、数据类、常量
# 业务逻辑层的共享类型，无状态，无副作用
# ======================================================================
from enum import Enum, auto

class WarehouseState(Enum):
    IDLE = auto()
    WAIT_FOR_ENTRY = auto()
    CONVEYOR_MOVING = auto()
    LIFT_MOVING = auto()
    POST_LIFT_PROCESSING = auto()
    DELAY_PROCESSING = auto()
    COMPLETED = auto()

class OutboundState(Enum):
    IDLE = auto()
    WAIT_FOR_EXIT = auto()
    LIFT_MOVING = auto()
    POST_LIFT_PROCESSING = auto()
    CONVEYOR_MOVING = auto()
    COMPLETED = auto()

class PassThroughState(Enum):
    """放行流程状态"""
    IDLE = auto()
    RETURNING_TO_LAYER_1 = auto()
    WAIT_FOR_PRODUCT_ARRIVAL = auto()
    CONVEYOR_RUNNING = auto()
    WAIT_FOR_CONVEYOR_OUT = auto()
    COMPLETED = auto()

class CommandType(Enum):
    JOG = auto()
    LAYER = auto()
    POSITION = auto()
    STOP = auto()

class FaultCode(Enum):
    """故障码定义 - 遵循C++层 fault_codes.hpp 规范
    业务逻辑错误类别：0x5xxx
    子类别：BUSINESS_SEQUENCE = 0x0200 (业务流程序列)
    """
    NO_FAULT = 0x0000
    WAREHOUSE_TIMEOUT = 0x5201
    OUTBOUND_TIMEOUT = 0x5202
    GEAR_CYLINDER_TIMEOUT = 0x5203
    CONVEYOR_IN_TIMEOUT = 0x5204

class ControlAction:
    def __init__(self, cmd_type: CommandType, axis_name: str, command_value: str,
                 target_position: float = 0.0, description: str = ""):
        self.type = cmd_type
        self.axis_name = axis_name
        self.command_value = command_value
        self.target_position = target_position
        self.description = description
