"""
Demo: Multi-point Cartesian trajectory planning

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

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
        robot_version=cmd_args.robot_version,
        gripper_type=cmd_args.gripper_type,
        control_aim=ServoDriver.AIM_TEACH,
        control_mode=ServoDriver.PATTERN_PV
    )
    
    if not robot.connect():
        logger.error("Unable to connect to the robot")
        return
    
    try:
        # Create Cartesian waypoint controller
        planner = CartesianWaypointPlanner(robot)
        
        # Move to initial position
        logger.info("\n1. Moving to initial position...")
        robot.set_home()
        
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
    parser.add_argument('--port', type=str, default="", help="Serial port (e.g., /dev/ttyCH343USB0 or COM3)")
    parser.add_argument('--robot_version', type=str, default="v1_0",  help="Robot version (default: v1_0)")
    parser.add_argument('--gripper_type', type=str, default="100mm",  help="Gripper type (default: 100mm)")
    
    
    # Trajectory planning settings
    parser.add_argument('--move_duration', type=float, default=3.0, help="Move time per waypoint (seconds, default: 3.0)")
    parser.add_argument('--num_points', type=int, default=200, help="Trajectory interpolation points (default: 200)")
    parser.add_argument('--ik_method', type=str, default='dls',
                       choices=['dls', 'pinv', 'lm'], help="Inverse kinematics method (default: dls)")
    parser.add_argument('--visualize', action='store_true', help="Enable trajectory visualization")
    
    args = parser.parse_args()
    
    main(args)

