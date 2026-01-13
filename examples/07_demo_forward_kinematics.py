"""
Demo: Forward kinematics

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Read current joint angles
- Calculate end-effector pose
- Display position, rotation matrix, Euler angles, and quaternion
"""

import numpy as np
import alicia_m_sdk
from alicia_m_sdk.hardware import ServoDriver
from robocore.utils.beauty_logger import beauty_print_array, beauty_print


def main(args):
    """Demonstrate forward kinematics.
    
    :param args: Command line arguments containing port, version, and gripper_type
    """
    # Initialize robot instance
    robot = alicia_m_sdk.create_robot(
        port=args.port,
        robot_version=args.robot_version,
        gripper_type=args.gripper_type,
        control_aim=ServoDriver.AIM_TEACH,
        control_mode=ServoDriver.PATTERN_MIT
    )
    
    robot_model = robot.robot_model
    if not robot.connect():
        return
    
    # Display robot model information
    robot_model.summary(show_chain=True)
    robot_model.print_tree(show_fixed=True)
    
    # Get detailed pose information using API
    pose_info = robot.get_pose()
    if pose_info is None:
        beauty_print("Failed to get pose")
        return
    
    # Extract pose components
    position_fk = pose_info['position']
    rotation_fk = pose_info['rotation']
    euler_fk = pose_info['euler_xyz']
    quat_fk = pose_info['quaternion_xyzw']
    T_fk = pose_info['transform']
    
    # Display results
    beauty_print("End-Effector Position (m):")
    print(f"  p = {beauty_print_array(position_fk)}")
    beauty_print("End-Effector Orientation (Euler XYZ, radians):")
    print(f"  rpy = {beauty_print_array(euler_fk)}")
    beauty_print("End-Effector Orientation (Euler XYZ, degrees):")
    print(f"  rpy = {beauty_print_array(np.rad2deg(euler_fk))}")
    beauty_print("End-Effector Orientation (Quaternion xyzw):")
    print(f"  quat = {beauty_print_array(quat_fk, precision=6)}")
    # Add note about quaternion sign ambiguity
    quat_neg = -quat_fk
    print("  Note: q and -q represent the same rotation")
    print(f"  -quat = {beauty_print_array(quat_neg, precision=6)} (equivalent)")
    beauty_print("Rotation Matrix:")
    print(beauty_print_array(rotation_fk, precision=6))
    beauty_print("Homogeneous Transformation Matrix:")
    print(beauty_print_array(T_fk, precision=6))
    
    robot.disconnect()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Forward Kinematics Demo")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="/dev/ttyUSB0",   help="Serial port (e.g., /dev/ttyUSB0 or COM3)")
    parser.add_argument('--robot_version', type=str, default="v1_0",  help="Robot version (default: v1_0)")
    parser.add_argument('--gripper_type', type=str, default="100mm", help="Gripper type (default: 100mm)")
    
    args = parser.parse_args()
    

    main(args)
