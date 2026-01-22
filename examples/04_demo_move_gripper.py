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
        # 等待后台查询获取数据（简洁输出）
        print("等待后台查询获取数据...")

        # 检查通信管理器状态
        if hasattr(robot.servo_driver, 'comm_manager') and robot.servo_driver.comm_manager:
            cm = robot.servo_driver.comm_manager
            stats = cm.get_stats()
            print(f"通信管理器: running={cm.is_running()}, auto_query={stats.get('auto_query_enabled', False)}, frames={stats.get('frames_received', 0)}")

        # 等待几次查询周期，让后台获取到真实数据（仅简短输出结果）
        found_valid_joints = False
        waited_ms = None
        for i in range(10):  # 等待最多1秒 (10次 * 100ms)
            time.sleep(0.1)
            joints = robot.get_robot_state("joint")
            if joints is not None and all(abs(j) < 6.28 for j in joints):
                found_valid_joints = True
                waited_ms = (i + 1) * 100
                break
        if found_valid_joints:
            print(f"获取到有效关节数据 (等待了 {waited_ms} ms)")
        else:
            print("未能在短时间内获取到有效关节数据")

        # 再次检查通信统计
        if hasattr(robot.servo_driver, 'comm_manager') and robot.servo_driver.comm_manager:
            stats = robot.servo_driver.comm_manager.get_stats()
            print(f"通信统计: commands_sent={stats.get('commands_sent', 0)}, frames_received={stats.get('frames_received', 0)}")

        # 读取初始状态
        # 初始状态（精简）
        initial_joints = robot.get_robot_state("joint")
        if initial_joints is not None:
            degs = [f"{j*57.2958:.2f}°" for j in initial_joints]
            print(f"初始关节（度）: {degs}")
        else:
            print("初始关节：无法读取")

        gripper_value = robot.get_robot_state("gripper")
        if gripper_value is not None:
            # 使用 logger 输出更少但可追踪的信息
            logger.info(f"初始夹爪值: {gripper_value:.1f}")

        # 三次夹爪动作（简洁输出）
        robot.set_robot_state(gripper_value=100, wait_for_completion=True)
        time.sleep(0.5)
        g1 = robot.get_robot_state("gripper")
        print(f"测试1 完成，夹爪={g1:.1f}" if g1 is not None else "测试1 完成，无法读取夹爪值")

        robot.set_robot_state(gripper_value=0, wait_for_completion=True)
        time.sleep(1.5)
        g2 = robot.get_robot_state("gripper")
        print(f"测试2 完成，夹爪={g2:.1f}" if g2 is not None else "测试2 完成，无法读取夹爪值")

        robot.set_robot_state(gripper_value=50, wait_for_completion=True)
        time.sleep(1.5)
        g3 = robot.get_robot_state("gripper")
        print(f"测试3 完成，夹爪={g3:.1f}" if g3 is not None else "测试3 完成，无法读取夹爪值")

        # 简短总结
        final_gripper = robot.get_robot_state("gripper")
        if final_gripper is not None:
            print(f"最终夹爪值: {final_gripper:.1f}")
        else:
            print("最终夹爪值：无法读取")

        
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
    parser.add_argument('--control-aim', type=str, default='operation', choices=['teach', 'operation'],
                        help='Control aim: teach or operation (motor-specific, auto-detected if not specified)')
    
    args = parser.parse_args()
    
    main(args)
