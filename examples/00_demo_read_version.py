"""
Demo: Read robot firmware version

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Connect to robot and read firmware version
- Auto-search available serial ports
- Display version information
"""

import alicia_m_sdk
from alicia_m_sdk.hardware import ServoDriver
from alicia_m_sdk.utils.logger import logger

def main(args):
    """Read and print robot firmware version.

    :param args: Command line arguments containing port, baudrate, version, and gripper_type
    """
    # Initialize robot instance
    robot = alicia_m_sdk.create_robot(
        port=args.port,
        robot_version=args.robot_version,
        gripper_type=args.gripper_type,
        control_aim=ServoDriver.AIM_TEACH,
        control_mode=ServoDriver.PATTERN_MIT
    )

    try:
        # Connect to robot
        if not robot.connect():
            logger.error("Connection failed, please check serial port settings")
            return
        firmware_version = robot.get_firmware_version()

    except KeyboardInterrupt:
        logger.info("\nOperation interrupted by user")

    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}")

    finally:
        robot.disconnect()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Read robot firmware version")

    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="Serial port (e.g., /dev/ttyUSB0 or COM3)")
    parser.add_argument('--robot_version', type=str, default="v1_0",  help="Robot version (default: v1_0)")
    parser.add_argument('--gripper_type', type=str, default="100mm",  help="Gripper type (default: 100mm)")
    args = parser.parse_args()

    main(args)