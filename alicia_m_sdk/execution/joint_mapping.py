"""Joint mapping helpers for Alicia-D leader to Alicia-M follower teleoperation."""

from __future__ import annotations

import math
from typing import List, Sequence

URDF_LIMIT = {
    "ALICIA_D": [
        {"jointName": "Joint1", "lower": -157.5, "upper": 157.5},
        {"jointName": "Joint2", "lower": -100.2, "upper": 100.2},
        {"jointName": "Joint3", "lower": -34.3, "upper": 126.0},
        {"jointName": "Joint4", "lower": -159.8, "upper": 159.8},
        {"jointName": "Joint5", "lower": -89.9, "upper": 89.9},
        {"jointName": "Joint6", "lower": -179.9, "upper": 179.9},
    ],
    "ALICIA_M": [
        {"jointName": "Joint1", "lower": -157.5, "upper": 157.5},
        {"jointName": "Joint2", "lower": -179.9, "upper": 0},
        {"jointName": "Joint3", "lower": -179.9, "upper": 0},
        {"jointName": "Joint4", "lower": -89.9, "upper": 89.9},
        {"jointName": "Joint5", "lower": -89.9, "upper": 89.9},
        {"jointName": "Joint6", "lower": -157.5, "upper": 157.5},
    ],
}

REVERSED_JOINT_INDEXES = {3, 5}
PROPORTIONAL_JOINT_INDEXES = {2}
NEGATED_INPUT_JOINT_INDEXES = {2}


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp value into [lower, upper]."""
    return max(lower, min(value, upper))


def map_joint_value(value: float, src_min: float, src_max: float, dst_min: float, dst_max: float) -> float:
    """Map a joint value linearly from one interval to another."""
    return ((value - src_min) / (src_max - src_min)) * (dst_max - dst_min) + dst_min


def map_joint_value_with_m_limit(
    value: float,
    dst_min: float,
    dst_max: float,
    reverse: bool = False,
    align_to_center: bool = True,
) -> float:
    """Map 1:1 into Alicia-M limits, optionally reversing direction."""
    dst_origin = (dst_min + dst_max) / 2 if align_to_center else 0.0
    direction = -1.0 if reverse else 1.0
    mapped = direction * value + dst_origin
    return clamp(mapped, min(dst_min, dst_max), max(dst_min, dst_max))


def convert_joints_deg_from_alicia_d_to_alicia_m(joints_deg: Sequence[float]) -> List[float]:
    """Convert Alicia-D joint angles in degrees to Alicia-M joint angles in degrees."""
    if len(joints_deg) != 6:
        raise ValueError(f"joints_deg 长度错误: 期望 6，实际 {len(joints_deg)}")

    result = []
    for i, joint in enumerate(joints_deg):
        if i in PROPORTIONAL_JOINT_INDEXES:
            if i in NEGATED_INPUT_JOINT_INDEXES:
                joint = -joint
                src_min = -URDF_LIMIT["ALICIA_D"][i]["upper"]
                src_max = -URDF_LIMIT["ALICIA_D"][i]["lower"]
            else:
                src_min = URDF_LIMIT["ALICIA_D"][i]["lower"]
                src_max = URDF_LIMIT["ALICIA_D"][i]["upper"]
            dst_min = URDF_LIMIT["ALICIA_M"][i]["lower"]
            dst_max = URDF_LIMIT["ALICIA_M"][i]["upper"]
            mapped = map_joint_value(
                clamp(joint, src_min, src_max),
                src_min,
                src_max,
                dst_min,
                dst_max,
            )
            mapped = clamp(mapped, min(dst_min, dst_max), max(dst_min, dst_max))
        else:
            mapped = map_joint_value_with_m_limit(
                joint,
                URDF_LIMIT["ALICIA_M"][i]["lower"],
                URDF_LIMIT["ALICIA_M"][i]["upper"],
                reverse=i in REVERSED_JOINT_INDEXES,
            )
        result.append(mapped)
    return result


def convert_joints_rad_from_alicia_d_to_alicia_m(joints_rad: Sequence[float]) -> List[float]:
    """Convert Alicia-D joint angles in radians to Alicia-M joint angles in radians."""
    joints_deg = [math.degrees(value) for value in joints_rad]
    mapped_deg = convert_joints_deg_from_alicia_d_to_alicia_m(joints_deg)
    return [math.radians(value) for value in mapped_deg]
