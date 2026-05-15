"""枚举定义：控制目标、控制模式、夹爪类型、协议错误码"""

from __future__ import annotations

from enum import Enum, IntEnum

__all__ = [
    'ControlAim',
    'ControlMode',
    'GripperType',
    'ErrorCode',
]


class ControlAim(IntEnum):
    """控制目标部位

    对应协议功能码低 7 位中的部位标识。

    Attributes:
        LEADER: 示教臂 (bit0=1)
        FOLLOWER: 操作臂 (bit1=1)
    """
    LEADER = 0x01      # 示教臂
    FOLLOWER = 0x02    # 操作臂


class ControlMode(Enum):
    """控制模式

    PV 与 MIT 是两种本质不同的控制范式：
    - PV: 发送目标位置+速度，固件内部做加减速插值（发后不管）
    - MIT: 每帧发送完整阻抗参数 (pos/vel/torque/kp/kd)，SDK 持有控制权（持续控制）

    Attributes:
        PV: 位置-速度模式
        MIT: MIT 阻抗控制模式
    """
    PV = "pv"          # 位置-速度模式（固件控制加减速）
    MIT = "mit"        # MIT 阻抗控制模式（每帧发送 pos/vel/torque/kp/kd）


class GripperType(Enum):
    """夹爪类型

    Attributes:
        MM_50: 50mm 行程夹爪
        MM_100: 100mm 行程夹爪
    """
    MM_50 = "50mm"
    MM_100 = "100mm"

    @property
    def firmware_value(self) -> int:
        """Firmware user-setting value used by command 0x02."""
        return 2 if self is GripperType.MM_100 else 0

    @property
    def option_value(self) -> int:
        """Demo/UI option value used by examples."""
        return 40 if self is GripperType.MM_100 else 10

    @property
    def label(self) -> str:
        """Chinese display label for CLI examples."""
        return "大夹爪" if self is GripperType.MM_100 else "小夹爪"

    @classmethod
    def from_firmware_value(cls, value: int) -> "GripperType":
        """Convert a firmware setting value to a gripper type."""
        return cls.MM_100 if int(value) & 0x02 else cls.MM_50

    @classmethod
    def from_option_value(cls, value: int) -> "GripperType":
        """Convert a demo/UI option value to a gripper type."""
        option = int(value)
        if option == cls.MM_50.option_value:
            return cls.MM_50
        if option == cls.MM_100.option_value:
            return cls.MM_100
        raise ValueError("gripper option must be 10 or 40")

    @classmethod
    def parse(cls, value) -> "GripperType":
        """Parse public gripper inputs without changing existing enum values."""
        if isinstance(value, cls):
            return value
        if isinstance(value, str):
            text = value.strip().lower()
            aliases = {
                "small": cls.MM_50,
                "mini": cls.MM_50,
                "50": cls.MM_50,
                "50mm": cls.MM_50,
                "large": cls.MM_100,
                "big": cls.MM_100,
                "100": cls.MM_100,
                "100mm": cls.MM_100,
            }
            if text in aliases:
                return aliases[text]
            value = int(text, 0)
        else:
            value = int(value)

        if value in {cls.MM_50.option_value, cls.MM_100.option_value}:
            return cls.from_option_value(value)
        if value in {cls.MM_50.firmware_value, cls.MM_100.firmware_value}:
            return cls.from_firmware_value(value)
        raise ValueError("gripper type must be 0/2, option 10/40, or small/large/50mm/100mm")


class ErrorCode(IntEnum):
    """协议错误码

    对应 0xEE 错误反馈指令中的错误类型字段。

    Attributes:
        FRAME_HEADER: 帧头/帧尾校验错误
        LENGTH_MISMATCH: 数据长度校验错误
        CRC_FAILED: CRC32 校验不通过
        SYSTEM_MODE: 系统模式错误（机械臂类型检测失败）
        ANGLE_LIMIT: 电机角度限位中
        MOTOR_OFFSET: motorData 偏移与部位数量不符
        ADDR_OVERFLOW: insID 偏移超过最大地址
    """
    FRAME_HEADER = 0x00       # 帧头/帧尾校验错误
    LENGTH_MISMATCH = 0x01    # 数据长度校验错误
    CRC_FAILED = 0x02         # CRC32 校验不通过
    SYSTEM_MODE = 0x03        # 系统模式错误（机械臂类型检测失败）
    ANGLE_LIMIT = 0x04        # 电机角度限位中
    MOTOR_OFFSET = 0x05       # motorData 偏移与部位数量不符
    ADDR_OVERFLOW = 0x06      # insID 偏移超过最大地址
