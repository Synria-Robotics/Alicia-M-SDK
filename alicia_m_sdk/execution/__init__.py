"""执行层：运动控制与轨迹执行

从 API 层分离出的运动控制逻辑，负责:
- JointController: 关节级运动控制（PV/MIT 双模式）
- TrajectoryExecutor: 轨迹回放执行器
- Teleoperation: 遥操作控制器
"""

from .joint_control import JointController
from .joint_mapping import (
    convert_joints_deg_from_alicia_d_to_alicia_m,
    convert_joints_rad_from_alicia_d_to_alicia_m,
)
from .trajectory_executor import TrajectoryExecutor
from .teleoperation import Teleoperation

__all__ = [
    'JointController',
    'TrajectoryExecutor',
    'Teleoperation',
    'convert_joints_deg_from_alicia_d_to_alicia_m',
    'convert_joints_rad_from_alicia_d_to_alicia_m',
]
