"""配置类型：RobotConfig

集中定义机器人连接、控制、运动学等全局配置参数。
配置实例在 create_robot() 工厂函数中构造，传递给各子模块。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

__all__ = [
    'RobotConfig',
]

# === 默认关节限位 (rad) ===
# 云擎 M 系列 6 关节默认限位范围
_DEFAULT_JOINT_LIMITS_LOWER: List[float] = [
    -math.pi,      # J0
    -math.pi,      # J1
    -math.pi,      # J2
    -math.pi,      # J3
    -math.pi,      # J4
    -math.pi,      # J5
]

_DEFAULT_JOINT_LIMITS_UPPER: List[float] = [
    math.pi,       # J0
    math.pi,       # J1
    math.pi,       # J2
    math.pi,       # J3
    math.pi,       # J4
    math.pi,       # J5
]

@dataclass
class RobotConfig:
    """机器人配置

    包含连接参数、机器人参数、控制参数、运动学参数和关节限位等。
    配置实例在 create_robot() 工厂函数中构造，贯穿 SDK 各层。

    Attributes:
        port: 串口端口路径，空字符串表示自动发现
        baudrate: 波特率
        auto_connect: 是否在创建实例后自动连接
        version: 硬件版本标识。``"auto"`` 表示连接后从固件自动检测；
            也可显式指定 ``"v1_0"``、``"v1_1"``、``"v1_2"`` 等跳过自动检测。
        variant: 变体标识，None 表示自动检测
        control_aim: 控制目标 ("leader"/"follower"/None=自动检测)
        control_mode: 控制模式 ("pv"/"mit")
        num_joints: 关节数量（不含夹爪）
        num_motors: 电机总数（含夹爪）
        model_format: 运动学模型格式
        base_link: 运动学基座链接名
        end_link: 运动学末端链接名
        backend: RoboCore 计算后端 ("numpy"/"torch")
        joint_limits_lower: 关节下限位 (rad)，长度等于 num_joints
        joint_limits_upper: 关节上限位 (rad)，长度等于 num_joints
        debug_mode: 调试模式开关
    """

    # --- 连接参数 ---
    port: str = ""                              # 串口端口（空=自动发现）
    baudrate: int = 1_000_000                   # 波特率
    auto_connect: bool = True                   # 自动连接

    # --- 机器人参数 ---
    version: str = "auto"                       # 硬件版本 ("auto" | "v1_0" | "v1_1" | "v1_2")
    variant: Optional[str] = None               # 变体（自动检测）
    control_aim: Optional[str] = None           # "leader" / "follower"（自动检测）

    # --- 控制参数 ---
    control_mode: Optional[str] = None          # "pv" / "mit" / None=检测固件当前模式
    sync_control_mode: bool = True              # 连接时是否检测/同步固件控制模式
    num_joints: int = 6                         # 关节数量（不含夹爪）
    num_motors: int = 7                         # 电机数量（含夹爪）

    # --- 运动学参数（RoboCore）---
    model_format: str = "urdf"                  # 模型格式
    base_link: str = "base_link"                # 基座链接名
    end_link: str = "tool0"                     # 末端链接名
    backend: str = "cpp"                       # 计算后端 ("numpy" / "torch" / "cpp")

    # --- 关节限位 (rad) ---
    joint_limits_lower: List[float] = field(
        default_factory=lambda: list(_DEFAULT_JOINT_LIMITS_LOWER)
    )
    joint_limits_upper: List[float] = field(
        default_factory=lambda: list(_DEFAULT_JOINT_LIMITS_UPPER)
    )

    # --- 调试 ---
    debug_mode: bool = False                    # 调试模式
