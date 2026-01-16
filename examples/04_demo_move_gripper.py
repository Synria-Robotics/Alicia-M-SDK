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
- Open/close gripper (0-100)
- Control gripper to specific values
- Optimized wait times for fast operation
"""

import alicia_m_sdk
from alicia_m_sdk.hardware import ServoDriver
import time
from alicia_m_sdk.utils.logger import logger

def main(args):
    """Demonstrate gripper control.
    
    :param args: Command line arguments
    """
    # Initialize robot instance
    robot = alicia_m_sdk.create_robot(
        port=args.port,
        gripper_type=args.gripper_type,
        robot_version=args.robot_version,
        control_aim=args.control_aim,
        control_mode="pv"
    )
    
    try:
        # Get current gripper value (0-100)
        gripper_value = robot.get_robot_state("gripper")
        if gripper_value is not None:
            logger.info(f"Gripper value: {gripper_value:.1f} (0-100, 0=closed, 100=open)")
        else:
            logger.warning("Failed to read gripper value")
        
        # Test 1: Open gripper fully
        robot.set_robot_state(gripper_value=100, wait_for_completion=True)
        time.sleep(0.5)
        
        # Test 2: Close gripper
        robot.set_robot_state(gripper_value=0, wait_for_completion=True)
        time.sleep(1.5)
        
        # Test 3: Partially open
        robot.set_robot_state(gripper_value=50, wait_for_completion=True)
        time.sleep(1.5)

        
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
    parser.add_argument('--gripper_type', type=str, default="100mm", help="夹爪类型")
    parser.add_argument('--robot_version', type=str, default="v1_1", help="机械臂版本")
    parser.add_argument('--base_link', type=str, default="base_link", help="基座链路名称")
    parser.add_argument('--end_link', type=str, default="Link9", help="末端执行器链路名称")
    parser.add_argument('--control-aim', type=str, default='teach', choices=['teach', 'operation'],
                        help='Control aim: teach or operation (motor-specific, auto-detected if not specified)')
    
    args = parser.parse_args()
    
    main(args)
