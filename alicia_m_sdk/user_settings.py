"""User settings command helpers for Alicia-M."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import List, Optional, Union

from .hardware.constants import (
    CMD_ERROR,
    CMD_USER_SETTINGS,
    FUNC_READ_ALL_SETTINGS,
    FUNC_WRITE_GRIPPER_TYPE,
    USER_SETTING_WRITE_ACCEPT,
)
from .hardware.frame import Frame
from .types.enums import GripperType

GRIPPER_TYPE_VALUES = {
    GripperType.MM_50.firmware_value: "默认小夹爪",
    GripperType.MM_100.firmware_value: GripperType.MM_100.label,
}

GRIPPER_TYPE_OPTIONS = {
    GripperType.MM_50.option_value: GripperType.MM_50.firmware_value,
    GripperType.MM_100.option_value: GripperType.MM_100.firmware_value,
}

GRIPPER_TYPE_OPTION_LABELS = {
    GripperType.MM_50.option_value: GripperType.MM_50.label,
    GripperType.MM_100.option_value: GripperType.MM_100.label,
}

SETTING_NAMES = [
    "开机动作配置",
    "夹爪类型配置",
    "定时上传开关",
]


@dataclass(frozen=True)
class UserSettings:
    """Structured user settings returned by command 0x02."""

    values: List[int]
    frame: Frame

    @property
    def startup_action(self) -> Optional[int]:
        return self.values[0] if len(self.values) > 0 else None

    @property
    def gripper_type(self) -> Optional[int]:
        return gripper_type_config_value(self.values[1]) if len(self.values) > 1 else None

    @property
    def timed_upload_enabled(self) -> Optional[int]:
        return self.values[2] if len(self.values) > 2 else None


def make_read_settings_frame() -> Frame:
    """Build a read-all user settings frame."""
    return Frame(cmd_id=CMD_USER_SETTINGS, func_code=FUNC_READ_ALL_SETTINGS)


def make_write_gripper_type_frame(gripper_type: Union[int, str, GripperType]) -> Frame:
    """Build a gripper type write frame."""
    return Frame(
        cmd_id=CMD_USER_SETTINGS,
        func_code=FUNC_WRITE_GRIPPER_TYPE,
        data=struct.pack("<I", normalize_gripper_type(gripper_type)),
    )


def send_user_settings_frame(device, frame: Frame, timeout: float = 1.0) -> Optional[Frame]:
    """Send a user settings frame and wait for a normal or error response."""
    response = device.send_and_wait_first(
        frame,
        [CMD_USER_SETTINGS, CMD_ERROR],
        timeout=timeout,
    )
    if response is None or response.cmd_id == CMD_ERROR:
        return response
    return response


def get_user_settings(device, timeout: float = 1.0) -> Optional[UserSettings]:
    """Read all user settings."""
    frame = send_user_settings_frame(device, make_read_settings_frame(), timeout=timeout)
    if frame is None or frame.cmd_id == CMD_ERROR:
        return None
    return parse_settings_response(frame)


def set_gripper_type(
    device,
    gripper_type: Union[int, str, GripperType],
    timeout: float = 3.0,
    readback: bool = True,
) -> bool:
    """Write gripper type and optionally confirm by reading settings back."""
    value = normalize_gripper_type(gripper_type)
    response = send_user_settings_frame(
        device,
        make_write_gripper_type_frame(value),
        timeout=timeout,
    )
    accepted = response is not None and response.cmd_id != CMD_ERROR and is_write_accepted(response)
    if not readback:
        return accepted

    settings = get_user_settings(device, timeout=timeout)
    if settings is None or settings.gripper_type is None:
        return accepted
    return settings.gripper_type == gripper_type_config_value(value)


def parse_settings_response(frame: Frame) -> UserSettings:
    """Parse a read-all user settings response."""
    if frame.cmd_id != CMD_USER_SETTINGS:
        raise ValueError(f"非个性化设置响应指令 ID: 0x{frame.cmd_id:02X}")
    if frame.func_code != FUNC_READ_ALL_SETTINGS:
        raise ValueError(f"非读取响应功能码: 0x{frame.func_code:02X}")
    if len(frame.data) % 4 != 0:
        raise ValueError(f"响应数据长度异常: {len(frame.data)} 字节")
    return UserSettings(
        values=list(struct.unpack(f"<{len(frame.data) // 4}I", frame.data)),
        frame=frame,
    )


def is_write_accepted(frame: Frame) -> bool:
    """Return whether a write response was accepted by firmware."""
    if frame.cmd_id != CMD_USER_SETTINGS:
        raise ValueError(f"非个性化设置响应指令 ID: 0x{frame.cmd_id:02X}")
    if frame.func_code != FUNC_WRITE_GRIPPER_TYPE:
        raise ValueError(f"非写入响应功能码: 0x{frame.func_code:02X}")
    if len(frame.data) != 1:
        raise ValueError(f"写入响应数据长度异常: {len(frame.data)} 字节")
    return frame.data[0] == USER_SETTING_WRITE_ACCEPT


def normalize_gripper_type(gripper_type: Union[int, str, GripperType]) -> int:
    """Normalize public gripper type inputs to firmware config values 0 or 2."""
    return GripperType.parse(gripper_type).firmware_value


def gripper_type_config_value(value: int) -> int:
    """Parse gripper type by protocol bit1."""
    return GripperType.from_firmware_value(value).firmware_value


def gripper_type_label(value: int) -> str:
    """Return a display label for a gripper type config value."""
    config_value = gripper_type_config_value(value)
    label = GRIPPER_TYPE_VALUES[config_value]
    if value in GRIPPER_TYPE_VALUES:
        return f"{value} ({label})"
    return f"{value} (按 bit1 解析为{label}，建议重新写入规范配置值 {config_value})"


def gripper_type_option_label(gripper_type: Union[int, str]) -> str:
    """Return a display label for CLI options 10/40."""
    option = int(str(gripper_type), 0)
    value = GRIPPER_TYPE_OPTIONS[option]
    return f"{option} ({GRIPPER_TYPE_OPTION_LABELS[option]} -> 配置值 {value})"
