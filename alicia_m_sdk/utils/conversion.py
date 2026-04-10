"""单位转换工具

提供物理量与协议值之间的双向转换函数。
映射关系遵循云擎通讯协议 v1.0.0 定义的编码规则。

映射参数汇总:
    - 位置: [-12.5, +12.5] rad  ↔ 16 bit [0, 65535]
    - 速度: [-10.0, +10.0] rad/s ↔ 12 bit [0, 4095]
    - 力矩(大关节 M0~M2): [-28.0, +28.0] N·m ↔ 12 bit [0, 4095]
    - 力矩(小关节 M3~M6): [-10.0, +10.0] N·m ↔ 12 bit [0, 4095]
    - Kp: [0, 500] ↔ 16 bit [0, 65535]
    - Kd: [0, 5]   ↔ 16 bit [0, 65535]
"""

import math
from typing import Union


# ============================================================
# 通用映射函数
# ============================================================

def float_to_uint(value: float, min_val: float, max_val: float, bits: int) -> int:
    """物理浮点值 → 无符号协议整数（通用映射）

    将 [min_val, max_val] 范围内的浮点值线性映射到 [0, 2^bits - 1]。
    超出范围的值会被裁剪到边界。

    Args:
        value: 物理浮点值
        min_val: 映射范围下界
        max_val: 映射范围上界
        bits: 目标无符号整数的位宽

    Returns:
        映射后的无符号整数
    """
    span = max_val - min_val
    max_uint = (1 << bits) - 1
    result = int((value - min_val) / span * max_uint)
    return max(0, min(max_uint, result))


def uint_to_float(raw: int, min_val: float, max_val: float, bits: int) -> float:
    """无符号协议整数 → 物理浮点值

    将 [0, 2^bits - 1] 范围内的无符号整数线性映射回 [min_val, max_val]。

    Args:
        raw: 协议中的无符号整数值
        min_val: 映射范围下界
        max_val: 映射范围上界
        bits: 无符号整数的位宽

    Returns:
        映射后的物理浮点值
    """
    max_uint = (1 << bits) - 1
    return raw / max_uint * (max_val - min_val) + min_val


# ============================================================
# 位置编解码: [-12.5, +12.5] rad ↔ 16 bit [0, 65535]
# ============================================================

def encode_position(rad: float) -> int:
    """位置弧度值 → 16 bit 协议值

    Args:
        rad: 关节位置 (rad)，有效范围 [-12.5, +12.5]

    Returns:
        16 bit 无符号协议整数 [0, 65535]
    """
    return float_to_uint(rad, -12.5, 12.5, 16)


def decode_position(raw: int) -> float:
    """16 bit 协议值 → 位置弧度值

    Args:
        raw: 16 bit 无符号协议整数

    Returns:
        关节位置 (rad)
    """
    return uint_to_float(raw, -12.5, 12.5, 16)


# ============================================================
# 速度编解码: [-10.0, +10.0] rad/s ↔ 12 bit [0, 4095]
# ============================================================

def encode_velocity(rad_s: float) -> int:
    """速度值 → 12 bit 协议值

    Args:
        rad_s: 关节速度 (rad/s)，有效范围 [-10.0, +10.0]

    Returns:
        12 bit 无符号协议整数 [0, 4095]
    """
    return float_to_uint(rad_s, -10.0, 10.0, 12)


def decode_velocity(raw: int) -> float:
    """12 bit 协议值 → 速度值

    Args:
        raw: 12 bit 无符号协议整数

    Returns:
        关节速度 (rad/s)
    """
    return uint_to_float(raw, -10.0, 10.0, 12)


# ============================================================
# 力矩编解码: 大关节 ±28.0, 小关节 ±10.0 N·m ↔ 12 bit [0, 4095]
# ============================================================

# 大关节（M0~M2）力矩映射范围
_TORQUE_RANGE_LARGE = 28.0
# 小关节（M3~M6，含夹爪）力矩映射范围
_TORQUE_RANGE_SMALL = 10.0
# 大关节与小关节的分界索引（<= 此值为大关节）
_LARGE_MOTOR_MAX_INDEX = 2


def _torque_range(motor_index: int) -> float:
    """根据电机索引获取力矩映射范围

    Args:
        motor_index: 电机索引 (0~6)

    Returns:
        力矩映射范围 (N·m)
    """
    return _TORQUE_RANGE_LARGE if motor_index <= _LARGE_MOTOR_MAX_INDEX else _TORQUE_RANGE_SMALL


def encode_torque(nm: float, motor_index: int) -> int:
    """力矩值 → 12 bit 协议值

    大关节（M0~M2）映射范围 [-28.0, +28.0] N·m，
    小关节（M3~M6）映射范围 [-10.0, +10.0] N·m。

    Args:
        nm: 力矩值 (N·m)
        motor_index: 电机索引 (0~6)

    Returns:
        12 bit 无符号协议整数 [0, 4095]
    """
    tor_range = _torque_range(motor_index)
    return float_to_uint(nm, -tor_range, tor_range, 12)


def decode_torque(raw: int, motor_index: int) -> float:
    """12 bit 协议值 → 力矩值

    Args:
        raw: 12 bit 无符号协议整数
        motor_index: 电机索引 (0~6)

    Returns:
        力矩值 (N·m)
    """
    tor_range = _torque_range(motor_index)
    return uint_to_float(raw, -tor_range, tor_range, 12)


# ============================================================
# Kp/Kd 编解码
# ============================================================

def encode_kp(kp: float) -> int:
    """Kp 增益值 → 16 bit 协议值

    Args:
        kp: Kp 增益值，有效范围 [0, 500]

    Returns:
        16 bit 无符号协议整数 [0, 65535]
    """
    return float_to_uint(kp, 0.0, 500.0, 16)


def decode_kp(raw: int) -> float:
    """16 bit 协议值 → Kp 增益值

    Args:
        raw: 16 bit 无符号协议整数

    Returns:
        Kp 增益值
    """
    return uint_to_float(raw, 0.0, 500.0, 16)


def encode_kd(kd: float) -> int:
    """Kd 增益值 → 16 bit 协议值

    Args:
        kd: Kd 增益值，有效范围 [0, 5]

    Returns:
        16 bit 无符号协议整数 [0, 65535]
    """
    return float_to_uint(kd, 0.0, 5.0, 16)


def decode_kd(raw: int) -> float:
    """16 bit 协议值 → Kd 增益值

    Args:
        raw: 16 bit 无符号协议整数

    Returns:
        Kd 增益值
    """
    return uint_to_float(raw, 0.0, 5.0, 16)


# ============================================================
# 角度单位互转
# ============================================================

def deg_to_rad(deg: float) -> float:
    """角度 → 弧度

    Args:
        deg: 角度值 (度)

    Returns:
        弧度值 (rad)
    """
    return deg * math.pi / 180.0


def rad_to_deg(rad: float) -> float:
    """弧度 → 角度

    Args:
        rad: 弧度值 (rad)

    Returns:
        角度值 (度)
    """
    return rad * 180.0 / math.pi


# ============================================================
# 用户速度 ↔ 固件速度
# ============================================================

# 用户速度范围上限（无量纲幅值）
_USER_SPEED_MAX = 400.0
# 固件速度范围上限 (rad/s)
_FIRMWARE_SPEED_MAX = 10.0


def speed_user_to_firmware(speed: float) -> float:
    """用户速度 → 固件速度 (rad/s)

    用户层速度为无量纲幅值 [0, 400]，映射到固件速度 [0, 10] rad/s。
    映射公式: firmware_speed = user_speed * 10.0 / 400.0

    Args:
        speed: 用户层速度值 [0, 400]

    Returns:
        固件速度幅值 (rad/s) [0, 10]
    """
    return speed * _FIRMWARE_SPEED_MAX / _USER_SPEED_MAX


def speed_firmware_to_user(speed_rad: float) -> float:
    """固件速度 (rad/s) → 用户速度

    固件速度 [0, 10] rad/s 映射到用户层无量纲幅值 [0, 400]。
    映射公式: user_speed = firmware_speed * 400.0 / 10.0

    Args:
        speed_rad: 固件速度幅值 (rad/s) [0, 10]

    Returns:
        用户层速度值 [0, 400]
    """
    return speed_rad * _USER_SPEED_MAX / _FIRMWARE_SPEED_MAX


# ============================================================
# 夹爪量程转换
# ============================================================

def encode_gripper(value: float) -> int:
    """夹爪值 [0, 1000] → 16bit 协议值

    编码路径: [0, 1000] → 线性映射到 [32768, 39688]（对应固件 0~2.64 rad）。
    固件将夹爪视为普通电机，命令帧中的位置字段按弧度编码。

    Args:
        value: 夹爪值 [0=闭合, 1000=完全打开]

    Returns:
        16bit 协议原始值
    """
    clamped = max(0.0, min(1000.0, value))
    # 0 → 32768 (0 rad), 1000 → 39688 (2.64 rad)
    return int(32768 + clamped / 1000.0 * (39688 - 32768))


def decode_gripper(raw: int) -> float:
    """16bit 协议值 → 夹爪值 [0, 1000]

    解码路径: [32768, 39688] → [0, 1000]，是 encode_gripper 的逆映射。
    固件将夹爪视为普通电机，返回值使用与关节相同的 16bit 位置编码。

    Args:
        raw: 16bit 协议原始值

    Returns:
        夹爪值 [0=闭合, 1000=完全打开]
    """
    result = (raw - 32768) / (39688 - 32768) * 1000.0
    return max(0.0, min(1000.0, result))


# ============================================================
# 线圈温度解码
# ============================================================

def decode_temperature(raw: int) -> float:
    """16bit 协议值 → 线圈温度 (°C)

    映射: [0, 65535] → [0, 120] °C

    Args:
        raw: 16bit 协议原始值

    Returns:
        线圈温度 (°C)
    """
    return max(0, min(65535, raw)) / 65535.0 * 120.0


def gripper_normalize(raw: int, gripper_range: int) -> float:
    """夹爪原始值 → 归一化值

    将夹爪的原始协议值归一化到 [0, 1000] 范围。

    Args:
        raw: 夹爪原始协议值
        gripper_range: 夹爪量程（与硬件型号相关）

    Returns:
        归一化后的夹爪值 [0, 1000]
    """
    if gripper_range == 0:
        return 0.0
    return max(0.0, min(1000.0, raw / gripper_range * 1000.0))


def gripper_denormalize(value: float, gripper_range: int) -> int:
    """归一化值 → 夹爪原始值

    将 [0, 1000] 范围的夹爪值转换回原始协议值。

    Args:
        value: 归一化夹爪值 [0, 1000]
        gripper_range: 夹爪量程（与硬件型号相关）

    Returns:
        夹爪原始协议值
    """
    return int(max(0.0, min(1000.0, value)) / 1000.0 * gripper_range)
