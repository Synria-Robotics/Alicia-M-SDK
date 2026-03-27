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
Demo: Gripper control

Features:
- Open/close gripper
- Control gripper to specific angle
- Wait for gripper motion completion
"""

import alicia_m_sdk
import time
from alicia_m_sdk.utils.logger import logger

def main(args):
    """Demonstrate gripper control.

    :param args: Command line arguments
    """
    # Initialize robot instance
    robot = alicia_m_sdk.create_robot(
        port=args.port,
        version=args.version,
        control_aim=args.control_aim,
        control_mode=args.control_mode
    )
    
    try:
        # Get gripper value (0-1000)
        gripper_value = robot.get_robot_state("gripper")
        if gripper_value is not None:
            logger.info(f"Gripper value: {gripper_value:.1f}/1000")
        else:
            logger.warning("Failed to read gripper value")

        # Test 1: Open gripper
        robot.set_robot_state(gripper_value=1000, wait_for_completion=True, gripper_speed=100)
        time.sleep(1)
        # Test 2: Partially open
        robot.set_robot_state(gripper_value=500, wait_for_completion=True, gripper_speed=100)
        time.sleep(1)
        # Test 3: Open gripper again
        robot.set_robot_state(gripper_value=1000, wait_for_completion=True, gripper_speed=100)
        time.sleep(1)
        # Test 4: Close gripper
        robot.set_robot_state(gripper_value=0, wait_for_completion=True, gripper_speed=100)
        time.sleep(1)

        
    except KeyboardInterrupt:
        print("\n✗ Processing interrupted")
    
    except Exception as e:
        import traceback
        traceback.print_exc()
    
    finally:
        robot.disconnect()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Gripper Control Demo")

    # Serial port settings
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--version', type=str, default="v1_1", help="机械臂版本 (可选: v1_0, v1_1，默认: v1_1)")
    parser.add_argument('--control-aim', type=str, default='operation', choices=['teach', 'operation'],
                        help='Control aim: teach or operation (默认: operation)')
    parser.add_argument('--control-mode', type=str, default='pv',
                        choices=['pv', 'mit'],
                        help='Control mode (默认: pv)')
    args = parser.parse_args()

    main(args)
