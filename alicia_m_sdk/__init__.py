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


def _get_variant_for_version(version: str) -> str:
    """Get the default variant name for a given version.
    
    :param version: Version string (e.g., 'v1_0', 'v1_1')
    :return: Default variant name for the version
    """
    # Version to variant mapping
    version_variant_map = {
        'v1_0': 'follower',  # v1_0 uses 'follower' variant
        'v1_1': 'follower',  # v1_1 uses 'follower' variant
    }
    return version_variant_map.get(version, 'follower')


def create_robot(
    port: str = "",
    version: str = "v1_1",
    variant: str = None,
    model_format: str = "urdf",
    debug_mode: bool = False,
    auto_connect: bool = True,
    base_link: str = "base_link",
    end_link: str = "tool0",
    backend: Optional[str] = None,
    device: str = "cpu",
    model_path: str = None,
    # M-SDK specific parameters
    baudrate: int = 1000000,
    control_aim: str = None,
    control_mode: str = None,
    skip_mit_init: bool = False,
) -> SynriaRobotAPI:
    """
    Create robot instance.

    :param port: Serial port
    :param version: Version name, e.g., "v1_0", "v1_1", etc. Default is "v1_1".
        The SDK will automatically select the appropriate URDF model from synriard package.
    :param variant: Variant name, e.g., "follower", "leader", etc.
        If not specified, defaults to "follower" for most versions.
    :param model_format: Model format, 'urdf' or 'mjcf', default is 'urdf'
    :param debug_mode: Debug mode
    :param auto_connect: Automatically connect on initialization
    :param base_link: Base link name in the robot model (default 'base_link')
    :param end_link: End link name in the robot model (default 'tool0')
    :param backend: Computation backend, 'numpy' or 'torch' (default: None, uses 'numpy')
    :param device: Device for torch backend, 'cpu' or 'cuda' (default: 'cpu')
    :param model_path: Model path, if None, use default model path from synriard
    :param baudrate: Serial port baudrate (default: 1000000)
    :param control_aim: Control aim string - "teach", "operation"
    :param control_mode: Control mode string - "pv" or "mit"
    :param skip_mit_init: Skip MIT Kp/Kd initialization, keep arm in current state (for read-only use)
    :return: SynriaRobotAPI instance
    """
    # Get default variant for the specified version
    if variant is None:
        variant = _get_variant_for_version(version)

    # Determine control_aim: prioritize user's input, then infer from variant
    if control_aim is not None:
        # User explicitly specified control_aim
        if isinstance(control_aim, str):
            control_aim_lower = control_aim.lower()
            aim_map = {
                'teach': ServoDriver.AIM_TEACH,
                'operation': ServoDriver.AIM_OPERATION,
            }
            control_aim_const = aim_map.get(control_aim_lower)
            if control_aim_const is None:
                raise ValueError(f"Unknown control_aim: {control_aim}. Valid values: {list(aim_map.keys())}")
        else:
            # Backward compatibility: accept constant directly
            control_aim_const = control_aim
    else:
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
                'mit': ServoDriver.PATTERN_MIT,
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
        device=device,
        control_mode=control_mode_const,
        skip_mit_init=skip_mit_init
    )

    return robot
