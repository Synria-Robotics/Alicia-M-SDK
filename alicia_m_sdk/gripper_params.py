"""@file gripper_params.py
@brief 夹爪夹持参数命令 0x17 辅助工具。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Dict, Mapping, Optional, Tuple, Union

from .hardware.constants import (
    AIM_FOLLOWER,
    AIM_LEADER,
    CMD_GRIPPER_PARAM,
    FUNC_WRITE_BIT,
)
from .hardware.frame import Frame
from .types.enums import GripperType


@dataclass(frozen=True)
class GripperParamSpec:
    """@brief 单个夹爪夹持参数的元数据。"""

    name: str
    mask: int
    label: str
    unit: str = ""
    range_text: str = ""
    small_range: Optional[Tuple[float, float]] = None
    large_range: Optional[Tuple[float, float]] = None
    aliases: Tuple[str, ...] = ()

    def allowed_range(self, gripper_type: Optional[Union[str, int, GripperType]] = None) -> Optional[Tuple[float, float]]:
        """Return the allowed value range for a gripper type.

        When gripper_type is None, return the union range accepted by the
        public protocol so low-level helpers still reject clearly invalid
        values without requiring hardware readback.
        """
        if self.small_range is None or self.large_range is None:
            return None
        if gripper_type is None:
            return (
                min(self.small_range[0], self.large_range[0]),
                max(self.small_range[1], self.large_range[1]),
            )
        parsed = GripperType.parse(gripper_type)
        return self.large_range if parsed is GripperType.MM_100 else self.small_range


GRIPPER_PARAM_SPECS = (
    GripperParamSpec(
        "target_force",
        0x01,
        "目标夹持力",
        "N",
        "小夹爪[1,80]；大夹爪[1,120]",
        small_range=(1.0, 80.0),
        large_range=(1.0, 120.0),
    ),
    GripperParamSpec(
        "open_feedforward",
        0x02,
        "张开前馈力矩",
        "N*m",
        "小夹爪[0.2,3.0]；大夹爪[0.2,5.0]",
        small_range=(0.2, 3.0),
        large_range=(0.2, 5.0),
        aliases=("open_feedforward_torque",),
    ),
    GripperParamSpec(
        "close_feedforward",
        0x04,
        "闭合前馈力矩",
        "N*m",
        "小夹爪[-5.0,-0.2]；大夹爪[-8.0,-0.2]",
        small_range=(-5.0, -0.2),
        large_range=(-8.0, -0.2),
        aliases=("close_feedforward_torque",),
    ),
    GripperParamSpec(
        "hold_torque",
        0x08,
        "最大保持力矩",
        "N*m",
        "小夹爪[0.5,5.0]；大夹爪[0.5,10.0]",
        small_range=(0.5, 5.0),
        large_range=(0.5, 10.0),
        aliases=("max_hold_torque",),
    ),
    GripperParamSpec("force_kp", 0x10, "力控比例", "", "[0,2.0]", small_range=(0.0, 2.0), large_range=(0.0, 2.0)),
    GripperParamSpec("force_ki", 0x20, "力控积分", "1/s", "[0,2.0]", small_range=(0.0, 2.0), large_range=(0.0, 2.0)),
    GripperParamSpec("integral_limit", 0x40, "积分限幅", "N*s", "[0,100]", small_range=(0.0, 100.0), large_range=(0.0, 100.0)),
    GripperParamSpec(
        "close_torque_scale",
        0x80,
        "接近闭合时的力矩缩放",
        "比例",
        "[0,1]",
        small_range=(0.0, 1.0),
        large_range=(0.0, 1.0),
    ),
)

GRIPPER_PARAM_BY_NAME = {spec.name: spec for spec in GRIPPER_PARAM_SPECS}
for _spec in GRIPPER_PARAM_SPECS:
    for _alias in _spec.aliases:
        GRIPPER_PARAM_BY_NAME[_alias] = _spec
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


def validate_gripper_param_values(
    values: Mapping[Union[str, int], float],
    gripper_type: Optional[Union[str, int, GripperType]] = None,
) -> Dict[int, float]:
    """Normalize and validate 0x17 gripper parameter values.

    @param values 参数值，键可以是公开名称、别名或协议掩码位。
    @param gripper_type 夹爪类型。None 表示按大小夹爪合并后的协议范围校验。
    @return 按协议掩码位索引的参数值。
    """
    normalized = normalize_gripper_param_values(values)
    parsed_type = None if gripper_type is None else GripperType.parse(gripper_type)
    for mask, value in normalized.items():
        spec = GRIPPER_PARAM_BY_MASK[mask]
        allowed = spec.allowed_range(parsed_type)
        if allowed is None:
            continue
        lower, upper = allowed
        if value < lower or value > upper:
            type_text = "协议允许范围" if parsed_type is None else parsed_type.label
            unit = f" {spec.unit}" if spec.unit else ""
            raise ValueError(
                f"{spec.label}={value:g}{unit}，{type_text}为 "
                f"[{lower:g}, {upper:g}]{unit}"
            )
    return normalized


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
    gripper_type: Optional[Union[str, int, GripperType]] = None,
    save: bool = False,
) -> Frame:
    """@brief 构造 0x17 写入帧。

    @param values 待写入参数值。
    @param aim 目标部位。
    @param gripper_type 夹爪类型；None 表示按大小夹爪合并后的协议范围校验。
    @param save True 时在数据区末尾追加保存标志，请求设备掉电保存当前完整夹爪参数配置。
    @return 协议帧。
    @note save 默认为 False，保持旧协议写入后立即生效但不主动掉电保存的行为。
    """
    normalized = validate_gripper_param_values(values, gripper_type=gripper_type)
    mask = 0
    for bit in normalized:
        mask |= bit

    data = bytearray([mask])
    for spec in GRIPPER_PARAM_SPECS:
        if spec.mask & mask:
            data.extend(struct.pack("<f", normalized[spec.mask]))
    if save:
        data.append(0x01)
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
