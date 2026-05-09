"""Diagnostic command helpers for Alicia-M."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

from .hardware.constants import AIM_FOLLOWER, AIM_LEADER, CMD_DIAGNOSTIC, CMD_ERROR
from .hardware.frame import Frame
from .utils.version import supports_min_version

DIAGNOSTIC_SUPPORTED_VERSION = (1, 0, 6)
DIAGNOSTIC_BLOCK_LEN = 15
DIAGNOSTIC_JOINT_COUNT = 7
DIAGNOSTIC_REQUEST = Frame(
    cmd_id=CMD_DIAGNOSTIC,
    func_code=AIM_FOLLOWER,
    data=bytes([0xFE]),
)

MODE_NAMES = {
    0x0: "普通状态",
    0x1: "控制协议状态",
    0x2: "重力补偿状态",
    0x3: "双臂同步状态",
    0x4: "固件升级状态",
    0x5: "控制锁定状态",
}

MOTOR_STATE_NAMES = {
    0x0: "失能",
    0x1: "使能",
    0x4: "电机轴异常",
    0x8: "超压",
    0x9: "欠压",
    0xA: "过电流",
    0xB: "MOS 过温",
    0xC: "电机线圈过温",
    0xD: "通讯丢失",
    0xE: "过载",
}

CONTROL_MODE_NAMES = {
    0x00: "未读取或未初始化",
    0x01: "mit模式",
    0x02: "pv模式",
    0x03: "速度模式",
    0x04: "位置、速度、电流混合模式",
}


@dataclass(frozen=True)
class DiagnosticArmSnapshot:
    """Parsed diagnostic snapshot for one arm."""

    arm_name: str
    comm_bitmap: int
    motor_states: List[int]
    control_modes: List[int]


@dataclass(frozen=True)
class DiagnosticResult:
    """Structured diagnostic response."""

    frame: Optional[Frame]
    snapshots: List[DiagnosticArmSnapshot]
    error_code: Optional[int] = None
    error_data: bytes = b""

    @property
    def ok(self) -> bool:
        return self.frame is not None and self.error_code is None and bool(self.snapshots)


def supports_diagnostic(version: object) -> bool:
    """Return whether firmware supports diagnostics."""
    return supports_min_version(version, DIAGNOSTIC_SUPPORTED_VERSION)


def run_diagnostic(device, timeout: float = 3.0) -> DiagnosticResult:
    """Send the diagnostic request and parse the first normal or error response."""
    response = device.send_and_wait_first(
        DIAGNOSTIC_REQUEST,
        [CMD_DIAGNOSTIC, CMD_ERROR],
        timeout=timeout,
    )
    if response is None:
        return DiagnosticResult(frame=None, snapshots=[])
    return parse_diagnostic_response(response)


def parse_diagnostic_response(frame: Frame) -> DiagnosticResult:
    """Parse a diagnostic response frame into a structured result."""
    if frame.cmd_id == CMD_ERROR:
        return DiagnosticResult(
            frame=frame,
            snapshots=[],
            error_code=frame.func_code,
            error_data=frame.data,
        )

    if frame.cmd_id != CMD_DIAGNOSTIC:
        raise ValueError(f"非自检响应指令 ID: 0x{frame.cmd_id:02X}")

    data = frame.data
    if len(data) not in (DIAGNOSTIC_BLOCK_LEN, DIAGNOSTIC_BLOCK_LEN * 2):
        raise ValueError(f"自检响应数据长度异常: {len(data)} 字节")

    block_count = len(data) // DIAGNOSTIC_BLOCK_LEN
    arm_names = diagnostic_arm_names(frame.func_code, block_count)
    snapshots = []
    for block_index in range(block_count):
        offset = block_index * DIAGNOSTIC_BLOCK_LEN
        block = data[offset:offset + DIAGNOSTIC_BLOCK_LEN]
        snapshots.append(DiagnosticArmSnapshot(
            arm_name=arm_names[block_index],
            comm_bitmap=block[0],
            motor_states=list(block[1:8]),
            control_modes=list(block[8:15]),
        ))
    return DiagnosticResult(frame=frame, snapshots=snapshots)


def diagnostic_arm_names(func_code: int, block_count: int) -> List[str]:
    """Return arm names from the response arm mask and block count."""
    arm_mask = func_code & 0x7F
    names = []
    if arm_mask & AIM_LEADER:
        names.append("示教臂")
    if arm_mask & AIM_FOLLOWER:
        names.append("操作臂")
    if len(names) == block_count:
        return names
    return ["操作臂" if block_count == 1 else f"机械臂{index + 1}" for index in range(block_count)]


def code_name(value: int, names: Dict[int, str]) -> str:
    """Return a readable code label with its raw value."""
    return f"{names.get(value, '未知')} (0x{value:02X})"


def mode_name(mode: int) -> str:
    """Return a readable mode name."""
    return MODE_NAMES.get(mode, "未知状态")
