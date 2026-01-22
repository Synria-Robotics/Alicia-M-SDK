# Copyright (c) 2025 Synria Robotics Co., Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
Alicia-M SDK v1.0.0 - Bridged with RoboCore

Architecture Layers:
- User Layer: SynriaRobotAPI (unified user interface)
- Execution Layer: TrajectoryExecutor, DragTeaching (trajectory execution)
- Hardware Layer: ServoDriver (low-level hardware drivers)
- Kinematics Layer: RoboCore kinematics functions (FK/IK/Jacobian)
- Planning Layer: RoboCore trajectory planning functions

RoboCore Integration:
- robocore.kinematics: Provides FK/IK/Jacobian calculations
- robocore.planning: Provides trajectory planning functionality
- robocore.modeling: Provides RobotModel for robot representation
"""

from alicia_m_sdk.api import SynriaRobotAPI
from alicia_m_sdk.hardware import ServoDriver

# Import from RoboCore for kinematics and modeling
from robocore.modeling import RobotModel
from robocore.kinematics import forward_kinematics, inverse_kinematics, jacobian
from synriard import get_model_path
import json
from pathlib import Path
from typing import Optional


__version__ = "1.0.0"
__author__ = "Synria Robotics"
__description__ = "Alicia-M Robot Arm SDK v1.0.0 - Bridged with RoboCore"

# Re-export RoboCore components for convenience
__all__ = [
    # Core API
    "SynriaRobotAPI",
    "create_robot",
    
    # Hardware Layer
    "ServoDriver",
    
    # RoboCore - Modeling
    "RobotModel",
    
    # RoboCore - Kinematics
    "forward_kinematics",
    "inverse_kinematics",
    "jacobian",
]


def _get_gripper_type_from_json() -> str:
    """Read gripper type from JSON file, return default if not found."""
    json_path = Path(__file__).parent / "api" / "gripper_type.json"
    if json_path.exists():
        try:
            with open(json_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            cached_type = data.get("type_name")
            if isinstance(cached_type, str) and cached_type:
                return cached_type
        except Exception:
            pass
    return "100mm"


def create_robot(
    port: str = "",
    version: str = "v1_0",
    variant: str = "follower",
    model_format: str = "urdf",
    debug_mode: bool = False,
    auto_connect: bool = True,
    base_link: str = "base_link",
    end_link: str = "tool0",
    backend: Optional[str] = None,
    device: str = "cpu",
    model_path: str = None,
    gripper_type: str = None,
    # M-SDK specific parameters
    baudrate: int = 1000000,
    control_aim: str = None,
    control_mode: str = None,
) -> SynriaRobotAPI:
    """
    Create robot instance.

    :param port: Serial port
    :param version: Version name, e.g., "v1_0", "v1_1", etc.
    :param variant: Variant name, e.g., "gripper_50mm", "gripper_100mm", etc.
    :param model_format: Model format, 'urdf' or 'mjcf', default is 'urdf'
    :param debug_mode: Debug mode
    :param auto_connect: Automatically connect on initialization
    :param base_link: Base link name in the robot model (default 'base_link')
    :param end_link: End link name in the robot model (default 'tool0')
    :param backend: Computation backend, 'numpy' or 'torch' (default: None, uses 'numpy')
    :param device: Device for torch backend, 'cpu' or 'cuda' (default: 'cpu')
    :param model_path: Model path, if None, use default model path
    :param gripper_type: Gripper type: deprecated, use variant instead please
        - explicit value such as "50mm" / "100mm" for user-defined configuration
        - None to auto-select from saved JSON (if available) or default to "100mm"
    :param baudrate: Serial port baudrate (default: 1000000)
    :param control_mode: Control mode string - "pv", "pvt", "v", "mit", "mit_position", "mit_speed", "mit_torque"
    :return: SynriaRobotAPI instance
    """
    effective_gripper_type = gripper_type if gripper_type is not None else _get_gripper_type_from_json()
    variant = variant if variant is not None else f"gripper_{effective_gripper_type}"

    # Auto-infer from variant: if variant contains "leader", it's a teach arm
    if variant is not None and "leader" in variant.lower():
        control_aim_const = ServoDriver.AIM_TEACH
    else:
        control_aim_const = ServoDriver.AIM_OPERATION

    servo_driver = ServoDriver(
        port=port,
        baudrate=baudrate,
        debug_mode=debug_mode,
        control_aim=control_aim_const
    )

    # Convert control_mode string to constant
    control_mode_const = None
    if control_mode is not None:
        if isinstance(control_mode, str):
            control_mode_lower = control_mode.lower()
            mode_map = {
                'pv': ServoDriver.PATTERN_PV,
                'pvt': ServoDriver.PATTERN_PVT,
                'v': ServoDriver.PATTERN_V,
                'mit': ServoDriver.PATTERN_MIT,
                'mit_position': ServoDriver.PATTERN_MIT_POSITION,
                'mit_speed': ServoDriver.PATTERN_MIT_SPEED,
                'mit_torque': ServoDriver.PATTERN_MIT_TORQUE,
            }
            control_mode_const = mode_map.get(control_mode_lower)
            if control_mode_const is None:
                raise ValueError(f"Unknown control_mode: {control_mode}. Valid values: {list(mode_map.keys())}")
        else:
            # Backward compatibility: accept tuple directly
            control_mode_const = control_mode

    if model_path is None:
        model_path = get_model_path(
            "Alicia_M",
            version=version,
            variant=variant,
            model_format=model_format
        )
    robot_model = RobotModel(str(model_path), base_link=base_link, end_link=end_link)

    robot = SynriaRobotAPI(
        servo_driver=servo_driver,
        robot_model=robot_model,
        auto_connect=auto_connect,
        backend=backend,
        device=device
    )
    
    # Set control_mode if provided
    if control_mode_const is not None:
        robot.control_mode = control_mode_const

    return robot
