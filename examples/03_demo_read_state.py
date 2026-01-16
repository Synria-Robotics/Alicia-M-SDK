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
Demo: Read and print robot state

Features:
- Read joint angles (radians or degrees)
- Read end-effector pose
- Read gripper state
- Support single or continuous printing
- Configurable reading frequency (FPS) for continuous mode
"""

import alicia_m_sdk
from alicia_m_sdk.hardware import ServoDriver
from alicia_m_sdk.utils.logger import logger


def main(args):
    """Read and print robot state.
    
    :param args: Command line arguments
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
        # Print robot state once
        if args.single:
            robot.print_state(continuous=False, output_format=args.format, robot_type=args.robot_type)
        else:
            # Print robot state continuously with specified FPS
            # Note: M-SDK's print_state doesn't support fps parameter, 
            # so we implement custom continuous printing with fps control
            if args.fps > 0:
                import time
                interval = 1.0 / args.fps
                logger.info(f"开始连续状态打印，按 Ctrl+C 停止 (目标FPS: {args.fps})")
                try:
                    while True:
                        start_time = time.perf_counter()
                        robot.print_state(continuous=False, output_format=args.format, robot_type=args.robot_type)
                        dt_time = time.perf_counter() - start_time
                        sleep_time = max(0, interval - dt_time)
                        if sleep_time > 0:
                            time.sleep(sleep_time)
                except KeyboardInterrupt:
                    logger.info("停止连续状态打印")
            else:
                # Use built-in continuous mode (fixed interval ~16.7 Hz)
                robot.print_state(continuous=True, output_format=args.format, robot_type=args.robot_type)
        
    except KeyboardInterrupt:
        logger.info("\n✗ Reading interrupted")
    
    except Exception as e:
        logger.error(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        robot.disconnect()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Read robot state")
    
    # Serial port settings
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--gripper_type', type=str, default="100mm", help="夹爪型号 (默认: 100mm)")
    parser.add_argument('--robot_version', type=str, default="v1_1", help="机械臂版本")
    parser.add_argument('--base_link', type=str, default="base_link", help="基座链路名称")
    parser.add_argument('--end_link', type=str, default="link6", help="末端执行器链路名称 (默认: tool0)")
    parser.add_argument('--control-aim', type=str, default='teach', choices=['teach', 'operation'],
                        help='Control aim: teach or operation (motor-specific, auto-detected if not specified)')
    parser.add_argument('--control-mode', type=str, default='pv', choices=['pv', 'pvt', 'v', 'mit', 'mit_position', 'mit_speed', 'mit_torque'],
                        help='Control mode: pv, pvt, v, mit, mit_position, mit_speed, mit_torque')
    
    # Display settings
    parser.add_argument('--format', type=str, default='deg', choices=['rad', 'deg'], 
                        help="Angle display format: rad(radians) or deg(degrees)")
    parser.add_argument('--single', action='store_true', 
                        help="Print state once (default: continuous print)")
    parser.add_argument('--fps', type=float, default=0.0, 
                        help="Target frames per second for continuous mode (default: 0, uses built-in interval ~16.7 Hz)")
    parser.add_argument('--robot-type', type=str, default='follower', choices=['follower', 'leader'],
                        help="Robot type: follower or leader (default: follower)")
    
    args = parser.parse_args()
    main(args)
