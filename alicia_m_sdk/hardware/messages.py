"""消息定义：各指令 ID 对应的请求/响应消息数据类

为每个指令定义结构化的消息类，使协议语义清晰可读。
上层代码操作消息对象而非裸字节，由 codec.py 完成双向转换。
"""

from dataclasses import dataclass, field
from typing import List, Optional


# ============================================================
# 0x01 — 版本与设备信息
# ============================================================

@dataclass
class VersionRequest:
    """查询版本信息的请求

    无额外参数，功能码固定为 0x7E，数据为占位符 0xFE。
    """
    pass


@dataclass
class VersionResponse:
    """版本信息响应

    Attributes:
        serial_number: 16 字节 ASCII 唯一序列号
        hardware_version: 硬件版本字符串（如 "1.0.0"）
        firmware_version: 固件版本字符串（如 "1.1.0"）
    """
    serial_number: str
    hardware_version: str
    firmware_version: str


# ============================================================
# 0x06 — 关节与夹具状态控制（核心指令）
# ============================================================

@dataclass
class JointStateRequest:
    """查询关节状态的请求

    Attributes:
        aim: 目标部位（AIM_LEADER=0x01 / AIM_FOLLOWER=0x02）
        start_addr: 起始数据地址（通常为 ADDR_POSITION=0x00）
        addr_count: 偏移数量，决定每个电机返回几个地址的数据
            - 1: 仅位置
            - 2: 位置 + 速度 (PV)
            - 3: 位置 + 速度 + 力矩
            - 5: 全部（pos + vel + torque + kp + kd，MIT）
    """
    aim: int
    start_addr: int
    addr_count: int


@dataclass
class JointStateResponse:
    """关节状态响应

    Attributes:
        start_addr: 起始数据地址
        addr_count: 偏移数量
        motor_data: 每个电机的原始数据列表，外层为电机索引，内层为各地址的原始值
            形状: [motor_count][addr_count]，每个值为 16bit 无符号整数
        run_status: 运行状态字节（1 字节，各 bit 含义取决于设备类型）
    """
    start_addr: int
    addr_count: int
    motor_data: List[List[int]]
    run_status: int


@dataclass
class JointControlRequest:
    """关节控制请求（写入目标状态）

    Attributes:
        aim: 目标部位（AIM_LEADER / AIM_FOLLOWER）
        start_addr: 起始数据地址
        addr_count: 偏移数量
            - 2: PV 模式（pos + vel）
            - 5: MIT 模式（pos + vel + torque + kp + kd）
        motor_data: 每个电机的目标数据列表
            形状: [motor_count][addr_count]，每个值为 16bit 无符号整数
    """
    aim: int
    start_addr: int
    addr_count: int
    motor_data: List[List[int]]


@dataclass
class JointControlResponse:
    """关节控制响应（写入结果反馈）

    Attributes:
        start_addr: 起始数据地址（bit7=1 表示反馈帧）
        addr_count: 偏移数量
        result: 执行结果，0x01=成功，0x00=失败
    """
    start_addr: int
    addr_count: int
    result: int


# ============================================================
# 0x05 — 肢体力矩控制
# ============================================================

@dataclass
class TorqueRequest:
    """力矩开关请求（仅 MIT 模式可用）

    通过 0x05 指令将选定关节的 kp=kd 置零实现卸力。

    Attributes:
        aim: 目标部位
        start_joint: 起始关节 ID（从 0 开始）
        joint_count: 关节数量（正方向偏移）
    """
    aim: int
    start_joint: int
    joint_count: int


# ============================================================
# 0x03 — 位姿重置
# ============================================================

@dataclass
class ZeroResetRequest:
    """设置当前位姿为零位

    Attributes:
        aim: 目标部位
        start_joint: 起始关节 ID
        joint_count: 关节数量
        reset_mode: 调零方式，0x00=弱调零，0x01=强调零
    """
    aim: int
    start_joint: int
    joint_count: int
    reset_mode: Optional[int] = None


# ============================================================
# 0x09 — 部位失能/使能
# ============================================================

@dataclass
class EnableRequest:
    """部位使能/失能请求（任何模式可用）

    硬件级使能/失能，与控制模式无关。
    使能后必须立即以当前位置发送首帧，防止关节突跳。

    Attributes:
        aim: 目标部位
        enable: True=使能，False=失能
    """
    aim: int
    enable: bool


# ============================================================
# 0x11 — 电机驱动参数设置
# ============================================================

@dataclass
class MotorParamRequest:
    """电机驱动参数写入请求

    用于修改电机底层参数，最常见用途是控制模式切换。

    Attributes:
        aim: 目标部位
        start_motor: 起始电机编号（1-based）
        motor_count: 电机数量
        param_addr: 参数地址（如 MOTOR_PARAM_CTRL_MODE=0x0B）
        param_value: 参数值（4 字节，小端序写入）
        save_to_flash: 是否在参数值后追加掉电保存标志
    """
    aim: int
    start_motor: int
    motor_count: int
    param_addr: int
    param_value: int
    save_to_flash: bool = False


@dataclass
class MotorParamReadRequest:
    """电机驱动参数读取请求

    Attributes:
        aim: 目标部位
        start_motor: 起始电机编号（1-based）
        motor_count: 电机数量
        param_addr: 参数地址（如 MOTOR_PARAM_CTRL_MODE=0x0B）
    """
    aim: int
    start_motor: int
    motor_count: int
    param_addr: int


# ============================================================
# 0xEE — 错误反馈（仅设备端发送）
# ============================================================

@dataclass
class ErrorResponse:
    """错误反馈响应

    设备端在检测到帧错误时主动发送。

    Attributes:
        error_code: 错误类型（功能码字段，对应 ERR_* 常量）
        error_data: 错误携带数据（含义取决于错误类型）
    """
    error_code: int
    error_data: bytes
