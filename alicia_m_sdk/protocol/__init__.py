"""协议层：通信协议的编解码

提供帧结构定义、消息数据类、编解码器和协议常量。
这是 SDK 协议栈的核心层，将原始字节流与结构化消息对象之间进行双向转换。
"""

from .constants import (
    # 帧结构
    FRAME_HEADER, FRAME_FOOTER, PLACEHOLDER,
    # 指令 ID
    CMD_VERSION, CMD_ZERO_RESET, CMD_TORQUE,
    CMD_JOINT_STATE, CMD_ENABLE, CMD_MOTOR_PARAM, CMD_ERROR,
    ZERO_RESET_WEAK, ZERO_RESET_STRONG,
    # 功能码
    FUNC_READ_BIT, FUNC_WRITE_BIT,
    FUNC_VERSION_REQ, FUNC_VERSION_RESP,
    # 部位标识
    AIM_LEADER, AIM_FOLLOWER,
    # 0x06 数据地址
    ADDR_POSITION, ADDR_VELOCITY, ADDR_TORQUE,
    ADDR_KP, ADDR_KD, ADDR_LINEAR_VEL,
    # 0x11 电机参数地址
    MOTOR_PARAM_CTRL_MODE, CTRL_MODE_MIT, CTRL_MODE_PV,
    # 数据映射
    NUM_JOINTS, NUM_MOTORS,
    DEFAULT_KP_LARGE, DEFAULT_KD_LARGE,
    DEFAULT_KP_SMALL, DEFAULT_KD_SMALL,
)
from .frame import Frame, crc32_check
from .messages import (
    VersionRequest, VersionResponse,
    JointStateRequest, JointStateResponse,
    JointControlRequest, JointControlResponse,
    TorqueRequest, ZeroResetRequest,
    EnableRequest, MotorParamRequest,
    ErrorResponse,
)
from .codec import MessageCodec

__all__ = [
    # 帧
    'Frame', 'crc32_check',
    # 消息
    'VersionRequest', 'VersionResponse',
    'JointStateRequest', 'JointStateResponse',
    'JointControlRequest', 'JointControlResponse',
    'TorqueRequest', 'ZeroResetRequest',
    'EnableRequest', 'MotorParamRequest',
    'ErrorResponse',
    # 编解码器
    'MessageCodec',
    # 常量
    'FRAME_HEADER', 'FRAME_FOOTER', 'PLACEHOLDER',
    'CMD_VERSION', 'CMD_ZERO_RESET', 'CMD_TORQUE',
    'CMD_JOINT_STATE', 'CMD_ENABLE', 'CMD_MOTOR_PARAM', 'CMD_ERROR',
    'ZERO_RESET_WEAK', 'ZERO_RESET_STRONG',
    'FUNC_READ_BIT', 'FUNC_WRITE_BIT',
    'FUNC_VERSION_REQ', 'FUNC_VERSION_RESP',
    'AIM_LEADER', 'AIM_FOLLOWER',
    'ADDR_POSITION', 'ADDR_VELOCITY', 'ADDR_TORQUE',
    'ADDR_KP', 'ADDR_KD', 'ADDR_LINEAR_VEL',
    'MOTOR_PARAM_CTRL_MODE', 'CTRL_MODE_MIT', 'CTRL_MODE_PV',
    'NUM_JOINTS', 'NUM_MOTORS',
    'DEFAULT_KP_LARGE', 'DEFAULT_KD_LARGE',
    'DEFAULT_KP_SMALL', 'DEFAULT_KD_SMALL',
]
