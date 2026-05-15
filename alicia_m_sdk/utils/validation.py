"""参数校验工具

提供关节角度、速度、夹爪值等输入参数的合法性校验。
校验策略：超限时优先**裁剪 + 警告**（而非直接拒绝），降低用户使用门槛。
"""

from typing import List, Union, Optional

from .beauty_logger import logger

# 夹爪值范围
_GRIPPER_MIN = 0.0
_GRIPPER_MAX = 1000.0

# 用户速度范围
_SPEED_MIN = 0.0
_SPEED_MAX = 400.0


def validate_joint_angles(
    angles: List[float],
    joint_limits_lower: List[float],
    joint_limits_upper: List[float],
) -> List[float]:
    """校验并裁剪关节角度到安全范围

    逐关节检查角度是否在限位范围内，超限时裁剪到边界并发出警告。

    :param angles, 目标关节角度列表 (rad)
    :param joint_limits_lower, 各关节角度下限 (rad)
    :param joint_limits_upper, 各关节角度上限 (rad)
    :return, 裁剪后的关节角度列表 (rad)
    """
    clipped = []
    for i, angle in enumerate(angles):
        lower = joint_limits_lower[i]
        upper = joint_limits_upper[i]

        if angle < lower:
            logger.warning(f"关节{i}超限: 目标={angle:.4f} rad, 下限={lower:.4f} rad, 已裁剪")
            clipped.append(lower)
        elif angle > upper:
            logger.warning(f"关节{i}超限: 目标={angle:.4f} rad, 上限={upper:.4f} rad, 已裁剪")
            clipped.append(upper)
        else:
            clipped.append(angle)

    return clipped


def validate_speed(
    speed: Union[float, int, List[float]],
    num_joints: int,
) -> List[float]:
    """校验速度参数，返回每关节速度列表

    支持标量（所有关节使用相同速度）和列表（逐关节独立速度）两种输入形式。
    超限时裁剪到 [0, 400] 并发出警告。

    :param speed, 速度值，标量或列表，范围 [0, 400]
    :param num_joints, 关节数量
    :return, 长度为 num_joints 的速度列表
    """
    # 标量 → 列表
    if isinstance(speed, (int, float)):
        speeds = [float(speed)] * num_joints
    else:
        speeds = [float(s) for s in speed]

    # 长度校验
    if len(speeds) != num_joints:
        logger.warning(
            f"速度列表长度 ({len(speeds)}) 与关节数 ({num_joints}) 不匹配，将截断或补齐"
        )
        if len(speeds) > num_joints:
            speeds = speeds[:num_joints]
        else:
            # 不足时使用最后一个值填充
            last = speeds[-1] if speeds else 0.0
            speeds.extend([last] * (num_joints - len(speeds)))

    # 范围裁剪
    for i in range(len(speeds)):
        if speeds[i] < _SPEED_MIN:
            logger.warning(f"关节{i}速度超限: {speeds[i]:.2f} < {_SPEED_MIN:.2f}, 已裁剪")
            speeds[i] = _SPEED_MIN
        elif speeds[i] > _SPEED_MAX:
            logger.warning(f"关节{i}速度超限: {speeds[i]:.2f} > {_SPEED_MAX:.2f}, 已裁剪")
            speeds[i] = _SPEED_MAX

    return speeds


def validate_gripper_value(value: float) -> float:
    """校验夹爪值范围 [0, 1000]

    超限时裁剪到边界并发出警告。

    :param value, 夹爪目标值
    :return, 裁剪后的夹爪值 [0, 1000]
    """
    if value < _GRIPPER_MIN:
        logger.warning(f"夹爪值超限: {value:.2f} < {_GRIPPER_MIN:.2f}, 已裁剪")
        return _GRIPPER_MIN
    elif value > _GRIPPER_MAX:
        logger.warning(f"夹爪值超限: {value:.2f} > {_GRIPPER_MAX:.2f}, 已裁剪")
        return _GRIPPER_MAX
    return float(value)
