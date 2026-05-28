"""@file _demo_helpers.py
@brief Alicia-M 示例脚本专用辅助入口。

@details
本模块用于把示例脚本中的命令行参数、打印、绘图和展示元数据声明到一处，
让普通 demo 文件保持清晰的公开 API 调用风格：
``import alicia_m_sdk`` + ``robot.xxx(...)``。

@note
本文件位于 `examples/`，只做 demo 专用声明和 re-export，不放具体 demo 的流程逻辑。
"""

from alicia_m_sdk.diagnostics import supports_diagnostic
from alicia_m_sdk.gripper_params import GRIPPER_PARAM_SPECS, gripper_param_mask
from alicia_m_sdk.user_settings import (
    SETTING_NAMES,
    gripper_type_label,
    gripper_type_option_label,
    normalize_gripper_type,
)
from alicia_m_sdk.utils.beauty_logger import beauty_print, beauty_print_array
from alicia_m_sdk.utils.cli import (
    NonBlockingKeyReader,
    add_port_argument,
    select_waypoint_mode,
)
from alicia_m_sdk.utils.protocol import format_bytes
from alicia_m_sdk.utils.trajectory_plot import (
    plot_joint_tracking,
    plot_joint_velocity_tracking,
    plot_trajectory,
)
from alicia_m_sdk.utils.version import supports_min_version

__all__ = [
    "GRIPPER_PARAM_SPECS",
    "NonBlockingKeyReader",
    "SETTING_NAMES",
    "add_port_argument",
    "beauty_print",
    "beauty_print_array",
    "format_bytes",
    "gripper_param_mask",
    "gripper_type_label",
    "gripper_type_option_label",
    "normalize_gripper_type",
    "plot_joint_tracking",
    "plot_joint_velocity_tracking",
    "plot_trajectory",
    "select_waypoint_mode",
    "supports_diagnostic",
    "supports_min_version",
]
