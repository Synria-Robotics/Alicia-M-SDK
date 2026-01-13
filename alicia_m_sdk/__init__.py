"""
Alicia-M SDK v1.0.0 - Bridged with RoboCore

Architecture Layers:
- User Layer: SynriaRobotAPI (unified user interface)
- Execution Layer: HardwareExecutor (hardware execution)
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


def create_robot(
        port: str = "", 
        baudrate: int = 1000000, 
        robot_version: str = "v1_1",
        robot_type: str = "follower",
        gripper_type: str = "100mm",
        firmware_version: None = None,
        debug_mode: bool = False,
        speed_rad_s: float = 0.2,
        control_aim: int = None,
        control_mode: tuple = None,
        auto_connect: bool = True,
        base_link: str = "base_link",
        end_link: str = "link6"
    ) -> SynriaRobotAPI:
    """
    Create robot instance.
    
    :param port: Serial port
    :param baudrate: Baud rate
    :param robot_version: Robot version (e.g., "v1_0", "v1_1")
    :param robot_type: Robot type (e.g., "follower")
    :param gripper_type: Gripper type (e.g., "100mm")
    :param firmware_version: Firmware version
    :param debug_mode: Debug mode
    :param speed_rad_s: Default speed in radians per second
    :param control_aim: Control target (ServoDriver.AIM_TEACH, AIM_OPERATION, etc.)
    :param control_mode: Control mode (ServoDriver.PATTERN_PV, PATTERN_PVT, etc.)
    :param auto_connect: Auto connect to robot
    :param base_link: Base link name in the robot model (default 'base_link')
    :param end_link: End link name in the robot model (default 'link6')
    :return: SynriaRobotAPI instance
    """
    # Create hardware layer
    servo_driver = ServoDriver(
        port=port, 
        baudrate=baudrate, 
        debug_mode=debug_mode, 
        firmware_version=firmware_version, 
        robot_type=robot_type,
        control_aim=control_aim
    )
    
    # Create kinematics layer using RoboCore and synriard
    urdf_path = get_model_path(
        "Alicia_M",
        version=robot_version,
        variant=f"gripper_{gripper_type}"
    )
    robot_model = RobotModel(str(urdf_path), base_link=base_link, end_link=end_link)
    
    # Create user layer API
    robot = SynriaRobotAPI(
        servo_driver=servo_driver,
        robot_model=robot_model,
        auto_connect=auto_connect
    )
    
    return robot
