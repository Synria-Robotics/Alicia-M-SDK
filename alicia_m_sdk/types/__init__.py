"""类型定义层：全局数据结构

集中定义所有数据类、枚举和异常，被 SDK 所有层共享。
类型定义层自身无业务依赖，仅包含纯数据结构。
"""

from .state import JointState, MitParams, RobotStatus, VersionInfo
from .config import RobotConfig
from .enums import ControlAim, ControlMode, GripperType, ErrorCode
from .exceptions import (
    AliciaSDKError,
    ConnectionError,
    TimeoutError,
    ProtocolError,
    ValidationError,
    RobotStateError,
    HardwareFaultError,
    MotionError,
)

__all__ = [
    # 状态类型
    'JointState',
    'MitParams',
    'RobotStatus',
    'VersionInfo',
    # 配置类型
    'RobotConfig',
    # 枚举
    'ControlAim',
    'ControlMode',
    'GripperType',
    'ErrorCode',
    # 异常
    'AliciaSDKError',
    'ConnectionError',
    'TimeoutError',
    'ProtocolError',
    'ValidationError',
    'RobotStateError',
    'HardwareFaultError',
    'MotionError',
]
