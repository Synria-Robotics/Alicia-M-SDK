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
        version=args.version,
        base_link=args.base_link,
        end_link=args.end_link,
        control_aim=args.control_aim,
        control_mode=args.control_mode,
    )

    try:
        # Step 1: Go home
        print("\n===== Step 1: 回到Home位置 =====")
        robot.go_home(speed=args.speed)
        time.sleep(2)

        # Step 2: Move joints AND close gripper simultaneously
        print("\n===== Step 2: 关节运动 + 夹爪张开（同时进行）=====")
        target_joints_deg = [90, -90.0, -90.0, 90.0, 0.0, 0.0]
        robot.set_robot_state(
            target_joints=target_joints_deg,
            gripper_value=1000,
            joint_format='deg',
            speed=args.speed,
            wait_for_completion=True,
            timeout=100
        )
        time.sleep(2)

        # Step 3: Return home AND open gripper simultaneously
        print("\n===== Step 3: 回Home + 夹爪闭合（同时进行）=====")
        robot.set_robot_state(
            target_joints=[0, 0, 0, 0, 0, 0],
            gripper_value=0,
            joint_format='deg',
            speed=args.speed,
            wait_for_completion=True,
            timeout=100
        )

    except KeyboardInterrupt:
        print("\n✗ Processing interrupted")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="机械臂运动控制示例")
    
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--version', type=str, default="v1_1", help="机械臂版本 (可选: v1_0, v1_1，默认: v1_1)")
    parser.add_argument('--base_link', type=str, default="base_link", help="基座链路名称")
    parser.add_argument('--end_link', type=str, default="link6", help="末端执行器链路名称")
    parser.add_argument('--control-aim', type=str, default='operation', choices=['teach', 'operation'],
                        help='Control aim: teach (0x01示教臂) or operation (0x02操作臂) (默认: operation)')
    parser.add_argument('--control-mode', type=str, default='pv', choices=['pv', 'mit'],
                        help='Control mode: pv or mit (默认: pv)')
    parser.add_argument('--speed', type=int, default=15, help="关节运动速度 (默认: 20，范围: 0-400)")
    
    args = parser.parse_args()
    main(args)
