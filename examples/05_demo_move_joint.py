"""
Demo: Control robot to move to target joint positions

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL 

Features:
- Joint space motion control (MIT position mode)
- Support degree and radian input
- Automatic joint angle interpolation
- Adjustable motion speed

MIT Mode Setup:
- MIT mode requires configuration first: python examples/01_demo_config_mit_params.py
- Configuration makes motors return to zero position
- No need to repeat configuration unless STM32 power cycles
- For details, see: examples/README_MIT_CONFIG.md
"""

import alicia_m_sdk
from alicia_m_sdk.hardware import ServoDriver
import time

def main(args):
    """Control robot joint movements.
    
    :param args: Command line arguments
    """
    # Initialize robot instance
    robot = alicia_m_sdk.create_robot(
        port=args.port,
        baudrate=args.baudrate,
        robot_version=args.robot_version,
        gripper_type=args.gripper_type,
        control_aim=ServoDriver.AIM_TEACH,
        control_mode=ServoDriver.PATTERN_PV
    )

    try:
        # Connect to robot
        if not robot.connect():
            print("✗ Connection failed, please check serial port settings")
            return
        
        robot.set_home()
        
        # Set target joint positions in degrees
        target_joints_deg = [90, -90.0, -90.0, 90.0, 0.0, 0.0, 20.0]
        
        print(f"Target joint angles (deg): {target_joints_deg}")
        
        # Send target position to robot
        robot.set_joint_target(
            target_joints=target_joints_deg,
            joint_format='deg',
            speeds=args.interplotation_speed,
            wait_for_completion=True
        )
        
        print("\n" + "="*50)
        print("Note: MIT mode has no real-time position feedback")
        print("For position verification, use non-MIT mode (e.g., PATTERN_PV)")
        print("="*50)

        time.sleep(0.3)

    except KeyboardInterrupt:
        print("\n✗ Processing interrupted")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Joint Motion Control Demo")
    
    parser.add_argument('--port', type=str, default="/dev/ttyUSB0", help="Serial port (e.g., /dev/ttyUSB0 or COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="Baudrate (default: 1000000)")
    parser.add_argument('--robot_version', type=str, default="v1_0",  help="Robot version (default: v1_0)")
    parser.add_argument('--gripper_type', type=str, default="100mm",  help="Gripper type (default: 100mm)")
    parser.add_argument('--interplotation_speed', type=float, default=0.2,  help="Motion speed factor (0.0-1.0, default: 1.0)")
    
    args = parser.parse_args()
    main(args)
