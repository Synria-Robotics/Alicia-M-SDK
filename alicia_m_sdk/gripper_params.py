"""夹爪夹持参数命令 0x17 辅助工具。"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Dict, Mapping, Optional, Union

from .hardware.constants import (
    AIM_FOLLOWER,
    AIM_LEADER,
    CMD_GRIPPER_PARAM,
    FUNC_WRITE_BIT,
)
from .hardware.frame import Frame


@dataclass(frozen=True)
class GripperParamSpec:
    """@brief 单个夹爪夹持参数的元数据。"""

    name: str
    mask: int
    label: str
    unit: str = ""


GRIPPER_PARAM_SPECS = (
    GripperParamSpec("target_force", 0x01, "目标夹持力", "N"),
    GripperParamSpec("open_feedforward", 0x02, "张开前馈力矩", "N*m"),
    GripperParamSpec("close_feedforward", 0x04, "闭合前馈力矩", "N*m"),
    GripperParamSpec("hold_torque", 0x08, "最大保持力矩", "N*m"),
    GripperParamSpec("force_kp", 0x10, "力控比例"),
    GripperParamSpec("force_ki", 0x20, "力控积分"),
    GripperParamSpec("integral_limit", 0x40, "积分限幅"),
    GripperParamSpec("close_torque_scale", 0x80, "接近闭合时的力矩缩放"),
)

GRIPPER_PARAM_BY_NAME = {spec.name: spec for spec in GRIPPER_PARAM_SPECS}
GRIPPER_PARAM_BY_MASK = {spec.mask: spec for spec in GRIPPER_PARAM_SPECS}


@dataclass(frozen=True)
class GripperParamResult:
    """@brief 夹爪参数响应的结构化结果。"""

    frame: Frame
    target_byte: Optional[int]
    mask: int
    values: Dict[str, float]
    write_ok: Optional[bool] = None


def normalize_gripper_param_values(
    values: Mapping[Union[str, int], float],
) -> Dict[int, float]:
    """@brief 将参数名或掩码位归一化为按掩码索引的字典。

    @param values 参数值，键可以是公开名称或协议掩码位。
    @return 按协议掩码位索引的参数值。
    """
    normalized: Dict[int, float] = {}
    for key, value in values.items():
        if isinstance(key, str):
            spec = GRIPPER_PARAM_BY_NAME.get(key)
            if spec is None:
                raise ValueError(f"Unknown gripper parameter name: {key}")
            mask = spec.mask
        else:
            mask = int(key)
            if mask not in GRIPPER_PARAM_BY_MASK:
                raise ValueError(f"Unknown gripper parameter mask: 0x{mask:02X}")
        normalized[mask] = float(value)
    return normalized


def gripper_param_mask(values: Mapping[Union[str, int], float]) -> int:
    """@brief 计算一组夹爪参数对应的协议掩码。

    @param values 参数值，键可以是公开名称或协议掩码位。
    @return 掩码按位或结果。
    """
    mask = 0
    for bit in normalize_gripper_param_values(values):
        mask |= bit
    return mask


def aim_code(aim: Union[str, int]) -> int:
    """@brief 将部位目标归一化为协议 aim 编码。

    @param aim follower/leader 字符串或 AIM_* 常量。
    @return 协议 aim 编码。
    """
    if isinstance(aim, str):
        key = aim.lower()
        if key == "follower":
            return AIM_FOLLOWER
        if key == "leader":
            return AIM_LEADER
        raise ValueError("aim must be 'follower', 'leader', AIM_FOLLOWER, or AIM_LEADER")
    if aim in (AIM_FOLLOWER, AIM_LEADER):
        return int(aim)
    raise ValueError("aim must be 'follower', 'leader', AIM_FOLLOWER, or AIM_LEADER")


def make_read_gripper_params_frame(aim: Union[str, int] = "follower", mask: int = 0) -> Frame:
    """@brief 构造 0x17 读取帧。

    @param aim 目标部位。
    @param mask 读取掩码，0 表示读取全部参数。
    @return 协议帧。
    """
    data = b"" if mask == 0 else bytes([mask & 0xFF])
    return Frame(cmd_id=CMD_GRIPPER_PARAM, func_code=aim_code(aim), data=data)


def make_write_gripper_params_frame(
    values: Mapping[Union[str, int], float],
    aim: Union[str, int] = "follower",
) -> Frame:
    """@brief 构造 0x17 写入帧。

    @param values 待写入参数值。
    @param aim 目标部位。
    @return 协议帧。
    """
    normalized = normalize_gripper_param_values(values)
    mask = 0
    for bit in normalized:
        mask |= bit

    data = bytearray([mask])
    for spec in GRIPPER_PARAM_SPECS:
        if spec.mask & mask:
            data.extend(struct.pack("<f", normalized[spec.mask]))
    return Frame(
        cmd_id=CMD_GRIPPER_PARAM,
        func_code=FUNC_WRITE_BIT | aim_code(aim),
        data=bytes(data),
    )


def parse_gripper_params_response(frame: Frame) -> GripperParamResult:
    """@brief 解析 0x17 读写响应。

    @param frame 响应帧。
    @return 结构化响应结果。
    """
    if frame.cmd_id != CMD_GRIPPER_PARAM:
        raise ValueError(f"Not a gripper parameter response: 0x{frame.cmd_id:02X}")
    if len(frame.data) < 2:
        raise ValueError(f"Gripper parameter response is too short: {len(frame.data)} bytes")

    target_byte = frame.data[0]
    mask = frame.data[1]
    payload = frame.data[2:]

    if len(payload) == 1:
        return GripperParamResult(
            frame=frame,
            target_byte=target_byte,
            mask=mask,
            values={},
            write_ok=payload[0] == 0x01,
        )

    values: Dict[str, float] = {}
    offset = 0
    for spec in GRIPPER_PARAM_SPECS:
        if not (mask & spec.mask):
            continue
        if offset + 4 > len(payload):
            raise ValueError(f"Missing payload bytes for gripper parameter {spec.name}")
        values[spec.name] = struct.unpack_from("<f", payload, offset)[0]
        offset += 4

    return GripperParamResult(
        frame=frame,
        target_byte=target_byte,
        mask=mask,
        values=values,
    )
