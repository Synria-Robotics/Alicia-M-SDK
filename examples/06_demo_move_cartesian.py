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
Demo: Multi-point Cartesian trajectory planning

Features:
- Multi-point trajectory planning in Cartesian space
- Manual drag teaching to record waypoints
- Automatic trajectory interpolation and execution
- Support trajectory visualization
"""

import argparse
import alicia_m_sdk
from alicia_m_sdk.hardware import ServoDriver
from alicia_m_sdk.utils.logger import logger
from alicia_m_sdk.execution import CartesianWaypointPlanner


def main(cmd_args):
    """Demonstrate multi-point Cartesian trajectory planning.
    
    :param cmd_args: Command line arguments
    """
    logger.info("=== Multi-point Cartesian trajectory planning demo ===")
    
    # Initialize and connect to the robot
    robot = alicia_m_sdk.create_robot(
        port=cmd_args.port,
        gripper_type=cmd_args.gripper_type,
        robot_version=cmd_args.robot_version,
        base_link=cmd_args.base_link,
        end_link=cmd_args.end_link,
        control_aim=cmd_args.control_aim,
        control_mode="pv"
    )

    try:
        # Create Cartesian waypoint controller
        planner = CartesianWaypointPlanner(robot)
        
        # Move to initial position
        logger.info("\n1. Moving to initial position...")
        robot.set_home(speed_deg_s=10)
        
        # Record waypoints (manual drag mode)
        waypoints = planner.record_teaching_waypoints()
        
        if not waypoints:
            logger.error("No waypoints recorded, exiting demo")
            return
        
        # Select execution mode
        step_by_step = input("\nSelect execution mode:\n"
                            "1. Continuous execution (recommended)\n"
                            "2. Step-by-step execution\n"
                            "Enter choice (1/2): ").strip() == "2"
        
        # Execute trajectory (optimized step_delay for faster operation)
        planner.execute_trajectory(
            waypoints=waypoints,
            move_duration=cmd_args.move_duration,
            num_points=cmd_args.num_points,
            ik_method=cmd_args.ik_method,
            visualize=cmd_args.visualize,
            step_by_step=step_by_step,
            step_delay=0.2 if step_by_step else 0.05
        )
        
        logger.info("\n✓ Demo completed!")
        
    except KeyboardInterrupt:
        logger.info("\nUser interrupted")
    except Exception as e:
        logger.error(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        robot.disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Multi-Point Cartesian Trajectory Demo")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--gripper_type', type=str, default="100mm", help="夹爪类型")
    parser.add_argument('--robot_version', type=str, default="v1_1", help="机械臂版本")
    parser.add_argument('--base_link', type=str, default="base_link", help="基座链路名称")
    parser.add_argument('--end_link', type=str, default="tool0", help="末端执行器链路名称")
    parser.add_argument('--control-aim', type=str, default='teach', choices=['teach', 'operation'],
                        help='Control aim: teach or operation (motor-specific)')
    
    # Trajectory planning settings
    parser.add_argument('--move_duration', type=float, default=3.0, help="Move time per waypoint (seconds, default: 3.0)")
    parser.add_argument('--num_points', type=int, default=200, help="Trajectory interpolation points (default: 200)")
    parser.add_argument('--ik_method', type=str, default='dls',
                       choices=['dls', 'pinv', 'lm'], help="Inverse kinematics method (default: dls)")
    parser.add_argument('--visualize', action='store_true', help="Enable trajectory visualization")
    
    args = parser.parse_args()
    
    main(args)

