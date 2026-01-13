"""
Demo: Gripper control

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Open/close gripper (0-100%)
- Control gripper to specific angles
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
        control_aim=ServoDriver.AIM_TEACH,
        control_mode=ServoDriver.PATTERN_PV
    )
    
    try:
        # Connect to robot
        if not robot.connect():
            return
        
        # Get current gripper value (0-100)
        gripper_value = robot.get_gripper()
        if gripper_value is not None:
            logger.info(f"Gripper value: {gripper_value:.1f}")
        else:
            logger.warning("Failed to read gripper value")
        
        # Test 1: Open gripper fully
        robot.set_gripper_target(value=100.0)
        time.sleep(0.3)
        
        # Test 2: Close gripper
        robot.set_gripper_target(value=0.0)
        time.sleep(0.3)
        
        # Test 3: Partially open
        robot.set_gripper_target(value=50.0)
        time.sleep(0.3)

        
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
    parser.add_argument('--port', type=str, default="", help="Serial port (e.g., /dev/ttyUSB0 or COM3)")
    parser.add_argument('--gripper_type', type=str, default="100mm",  help="Gripper type (default: 100mm)")
    
    args = parser.parse_args()
    
    
    main(args)
