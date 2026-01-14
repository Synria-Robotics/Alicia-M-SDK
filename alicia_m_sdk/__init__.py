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
    gripper_type: str = None,
    robot_version: str = "v1_0",
    debug_mode: bool = False,
    auto_connect: bool = True,
    base_link: str = "base_link",
    end_link: str = "tool0",
    control_aim: int = None,
    control_mode: tuple = None,
) -> SynriaRobotAPI:
    """
    Create robot instance.

    :param port: Serial port
    :param gripper_type: Gripper type:
        - explicit value such as "50mm" / "100mm" for user-defined configuration
        - None to auto-select from saved JSON (if available) or default to "100mm"
    :param robot_version: Robot version (e.g., "v1_0", "v1_1")
    :param debug_mode: Debug mode
    :param auto_connect: Auto connect to robot
    :param base_link: Base link name in the robot model (default 'base_link')
    :param end_link: End link name in the robot model (default 'tool0')
    :param control_aim: Control target (ServoDriver.AIM_TEACH, AIM_OPERATION, etc.) - Motor-specific
    :param control_mode: Control mode (ServoDriver.PATTERN_PV, PATTERN_PVT, etc.) - Motor-specific
    :return: SynriaRobotAPI instance
    """
    effective_gripper_type = gripper_type if gripper_type is not None else _get_gripper_type_from_json()

    # Create hardware layer
    servo_driver = ServoDriver(
        port=port,
        baudrate=1000000,  # Default baudrate for M-SDK
        debug_mode=debug_mode,
        control_aim=control_aim
    )
    
    # Create kinematics layer using RoboCore and synriard
    urdf_path = get_model_path(
        "Alicia_M",
        version=robot_version,
        variant=f"gripper_{effective_gripper_type}"
    )
    robot_model = RobotModel(str(urdf_path), base_link=base_link, end_link=end_link)
    
    # Create user layer API
    robot = SynriaRobotAPI(
        servo_driver=servo_driver,
        robot_model=robot_model,
        auto_connect=auto_connect
    )
    
    # Set control_mode if provided
    if control_mode is not None:
        robot.control_mode = control_mode

    return robot
