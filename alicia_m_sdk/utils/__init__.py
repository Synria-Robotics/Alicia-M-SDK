"""工具层：通用工具函数

提供单位转换、参数校验、高精度定时、日志系统等通用能力。
"""

from .conversion import (
    # 通用映射
    float_to_uint,
    uint_to_float,
    # 位置
    encode_position,
    decode_position,
    # 速度
    encode_velocity,
    decode_velocity,
    # 线性轨迹速度
    encode_linear_velocity,
    decode_linear_velocity,
    # 力矩
    encode_torque,
    decode_torque,
    # Kp / Kd
    encode_kp,
    decode_kp,
    encode_kd,
    decode_kd,
    # 角度单位
    deg_to_rad,
    rad_to_deg,
    # 用户/固件速度映射
    speed_user_to_firmware,
    speed_firmware_to_user,
    # 夹爪量程
    gripper_normalize,
    gripper_denormalize,
)

from .validation import (
    validate_joint_angles,
    validate_speed,
    validate_gripper_value,
)

from .timing import (
    precise_sleep,
    FPSCounter,
)

from .beauty_logger import (
    get_logger,
    get_sdk_log_file_path,
    print_info,
    print_success,
    print_warning,
    print_error,
    format_array,
    logger,
)
from .protocol import format_bytes
from .version import parse_firmware_version, supports_min_version
from .model_resolver import HW_VERSION_MAP, resolve_model_version, load_robot_model
from .cli import NonBlockingKeyReader, add_port_argument, select_waypoint_mode
from .trajectory_plot import (
    plot_trajectory,
    plot_joint_tracking,
    plot_joint_velocity_tracking,
    waypoint_times_for_plot,
)

__all__ = [
    # conversion
    "float_to_uint",
    "uint_to_float",
    "encode_position",
    "decode_position",
    "encode_velocity",
    "decode_velocity",
    "encode_linear_velocity",
    "decode_linear_velocity",
    "encode_torque",
    "decode_torque",
    "encode_kp",
    "decode_kp",
    "encode_kd",
    "decode_kd",
    "deg_to_rad",
    "rad_to_deg",
    "speed_user_to_firmware",
    "speed_firmware_to_user",
    "gripper_normalize",
    "gripper_denormalize",
    # validation
    "validate_joint_angles",
    "validate_speed",
    "validate_gripper_value",
    # timing
    "precise_sleep",
    "FPSCounter",
    # logger
    "get_logger",
    "print_info",
    "print_success",
    "print_warning",
    "print_error",
    "format_array",
    "get_sdk_log_file_path",
    "logger",
    "format_bytes",
    "parse_firmware_version",
    "supports_min_version",
    # model_resolver
    "HW_VERSION_MAP",
    "resolve_model_version",
    "load_robot_model",
    "add_port_argument",
    "select_waypoint_mode",
    "NonBlockingKeyReader",
    # trajectory_plot
    "plot_trajectory",
    "plot_joint_tracking",
    "plot_joint_velocity_tracking",
    "waypoint_times_for_plot",
]
