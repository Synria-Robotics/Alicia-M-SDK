#!/usr/bin/env python3
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
Demo: Read robot firmware version
"""

import alicia_m_sdk
from alicia_m_sdk.hardware import ServoDriver
from alicia_m_sdk.utils.logger import logger

def main(args):
    """Read and print robot firmware version.

    :param args: Command line arguments containing port
    """
    # Initialize robot instance
    robot = alicia_m_sdk.create_robot(
        port=args.port,
        gripper_type=args.gripper_type,
        robot_version=args.robot_version,
        base_link=args.base_link,
        end_link=args.end_link,
        control_aim=args.control_aim,
        control_mode=args.control_mode
    )

    try:
        robot_version = robot.get_robot_state("version")
        if robot_version:
            logger.info(
                "Version info: "
                f"Unique ID = {robot_version.get('serial_number')}, "
                f"Hardware Version = {robot_version.get('hardware_version')}, "
                f"Firmware Version = {robot_version.get('firmware_version')}"
            )
        
        gripper_type = robot.get_robot_state("gripper_type")
        if gripper_type:
            logger.info(f"Gripper type: {gripper_type}")

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
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--gripper_type', type=str, default="100mm", help="夹爪类型")
    parser.add_argument('--robot_version', type=str, default="v1_0", help="机械臂版本")
    parser.add_argument('--base_link', type=str, default="base_link", help="基座链路名称")
    parser.add_argument('--end_link', type=str, default="Link6", help="末端执行器链路名称")
    parser.add_argument('--control-aim', type=str, default='teach', choices=['teach', 'operation'],
                        help='Control aim: teach or operation (motor-specific, auto-detected if not specified)')
    parser.add_argument('--control-mode', type=str, default='pv', choices=['pv', 'pvt', 'v', 'mit', 'mit_position', 'mit_speed', 'mit_torque'],
                        help='Control mode: pv, pvt, v, mit, mit_position, mit_speed, mit_torque')
    args = parser.parse_args()

    main(args)