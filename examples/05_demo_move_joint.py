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
Demo: Control robot to move to target joint positions

Features:
- Joint space motion control
- Support degree and radian input
- Adjustable motion speed
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
        gripper_type=args.gripper_type,
        robot_version=args.robot_version,
        base_link=args.base_link,
        end_link=args.end_link,
        control_aim=args.control_aim,
        control_mode=args.control_mode,
        debug_mode=True  # 启用调试模式以打印发送和接收的数据包
    )

    try:
        # 检查后台线程状态
        thread_status = robot.servo_driver.get_update_thread_status()
        print(f"\n后台线程状态: {thread_status}")
        print(f"暂停标志状态: {robot.servo_driver._pause_update.is_set()}\n")
        
        # Set target joint positions in degrees
        target_joints_deg = [90, -90.0, -90.0, 90.0, 0.0, 0.0]
        robot.set_home(speed_deg_s=args.speed_deg_s)
        time.sleep(1)
        # Use unified joint and gripper target interface
        robot.set_robot_state(
            target_joints=target_joints_deg,
            joint_format='deg',
            speed_deg_s=args.speed_deg_s,
            wait_for_completion=True
        )
        time.sleep(1)
        robot.set_home(speed_deg_s=args.speed_deg_s)

    except KeyboardInterrupt:
        print("\n✗ Processing interrupted")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="机械臂运动控制示例")
    
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--gripper_type', type=str, default="100mm", help="夹爪类型")
    parser.add_argument('--robot_version', type=str, default="v1_1", help="机械臂版本")
    parser.add_argument('--base_link', type=str, default="base_link", help="基座链路名称")
    parser.add_argument('--end_link', type=str, default="link6", help="末端执行器链路名称")
    parser.add_argument('--control-aim', type=str, default='teach', choices=['teach', 'operation'],
                        help='Control aim: teach or operation (motor-specific, auto-detected if not specified)')
    parser.add_argument('--control-mode', type=str, default='pv', choices=['pv', 'pvt', 'v', 'mit', 'mit_position', 'mit_speed', 'mit_torque'],
                        help='Control mode: pv, pvt, v, mit, mit_position, mit_speed, mit_torque')
    parser.add_argument('--speed_deg_s', type=int, default=20, help="关节运动速度 (单位: 度/秒，默认: 20，范围: 10-400度/秒)")
    
    args = parser.parse_args()
    main(args)
