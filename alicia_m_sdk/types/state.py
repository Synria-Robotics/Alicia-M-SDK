"""状态类型：JointState、MitParams、RobotStatus、VersionInfo

集中定义机器人状态相关的数据结构，被所有层共享。
数据类仅承载数据，不包含业务逻辑。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

__all__ = [
    'JointState',
    'MitParams',
    'RobotStatus',
    'VersionInfo',
]


@dataclass
class JointState:
    """关节状态

    由硬件层（StateCache）在读线程中构造，通过原子引用替换更新。
    调用方应视为只读快照，不应修改其内容。

    Attributes:
        angles: 6 个关节角度 (rad)
        gripper: 夹爪值 (0~1000)
        timestamp: 状态采样时间戳（time.perf_counter）
        run_status: 原始运行状态字节（由 RobotStatus 进一步解析）
        velocities: 7 个电机速度 (rad/s)，含夹爪
        torques: 7 个电机力矩 (N*m)，含夹爪
        kps: 7 个电机位置环 Kp [0, 500]，含夹爪
        kds: 7 个电机速度环 Kd [0, 5]，含夹爪
        linear_vels: 7 个电机插补速度 (rad/s)，含夹爪
        temperatures: 7 个电机线圈温度 (°C)，含夹爪
    """
    angles: List[float]                         # 6 个关节角度 (rad)
    gripper: float                              # 夹爪值 (0~1000)
    timestamp: float                            # 时间戳
    run_status: int                             # 原始运行状态字节
    velocities: Optional[List[float]] = None    # 7 个电机速度 (rad/s)，含夹爪
    torques: Optional[List[float]] = None       # 7 个电机力矩 (N*m)，含夹爪
    kps: Optional[List[float]] = None           # 7 个电机位置环 Kp [0, 500]，含夹爪
    kds: Optional[List[float]] = None           # 7 个电机速度环 Kd [0, 5]，含夹爪
    linear_vels: Optional[List[float]] = None   # 7 个电机插补速度 (rad/s)，含夹爪
    temperatures: Optional[List[float]] = None  # 7 个电机线圈温度 (°C)，含夹爪


@dataclass
class MitParams:
    """单个电机的 MIT 阻抗控制参数

    MIT 控制律:
        tau = kp * (pos_ref - pos_cur) + kd * (vel_ref - vel_cur) + t_ref

    关于 kp/kd 的默认值策略:
    - kp/kd 默认为 None，由控制层根据电机编号自动填充安全默认值：
      - 大关节 (M0~M2): kp=150, kd=2
      - 小关节 (M3~M6，含夹爪): kp=20, kd=1
    - 显式传入 0 表示零力矩（卸力），不会被自动填充覆盖
    - 显式传入具体数值则直接使用，不做自动填充

    Attributes:
        pos_ref: 目标位置 (rad)
        vel_ref: 目标速度 (rad/s)
        t_ref: 前馈力矩 (N*m)
        kp: 位置增益 [0, 500]，None 表示使用默认值
        kd: 速度增益 [0, 5]，None 表示使用默认值
    """
    pos_ref: float = 0.0                # 目标位置 (rad)
    vel_ref: float = 0.0                # 目标速度 (rad/s)
    t_ref: float = 0.0                  # 前馈力矩 (N*m)
    kp: Optional[float] = None          # 位置增益 [0, 500]，None=使用默认值
    kd: Optional[float] = None          # 速度增益 [0, 5]，None=使用默认值


@dataclass
class RobotStatus:
    """机器人综合状态（从运行状态字节解析）

    由 JointState.run_status 字节解析得出，
    提供可读的布尔状态标志位。

    Attributes:
        is_locked: 关节锁定状态
        is_synced: 示教臂与操作臂同步状态
        has_motor_error: 电机错误标志
        gripper_torque_locked: 夹爪力矩锁定状态
        single_click: 示教臂按钮单击（仅示教臂有效）
        double_click: 示教臂按钮双击（仅示教臂有效）
        long_press: 示教臂按钮长按（仅示教臂有效）
    """
    is_locked: bool = False
    is_synced: bool = False
    has_motor_error: bool = False
    gripper_torque_locked: bool = False
    # 示教臂特有状态
    single_click: bool = False
    double_click: bool = False
    long_press: bool = False


@dataclass
class VersionInfo:
    """版本信息

    通过 0x01 指令查询获得的设备版本信息。

    Attributes:
        serial_number: 16 字节 ASCII 唯一序列号
        hardware_version: 硬件版本（如 "v1.0.0"）
        firmware_version: 固件版本（如 "v1.1.0"）
        product_type: 产品类别（AM=云擎, BM=云弈 ...）
        device_type: 设备类型（L=示教臂, F=操作臂 ...）
    """
    serial_number: str                  # 16 字节唯一序列号
    hardware_version: str               # 硬件版本
    firmware_version: str               # 固件版本
    product_type: str                   # 产品类别
    device_type: str                    # 设备类型
