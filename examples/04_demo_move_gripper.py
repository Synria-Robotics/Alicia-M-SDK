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
        # 等待后台查询获取数据
        print("\n" + "="*60)
        print("【等待后台查询获取数据...】")
        print("="*60)
        
        # 检查通信管理器状态
        if hasattr(robot.servo_driver, 'comm_manager') and robot.servo_driver.comm_manager:
            cm = robot.servo_driver.comm_manager
            stats = cm.get_stats()
            print(f"通信管理器状态:")
            print(f"  - 运行中: {cm.is_running()}")
            print(f"  - 自动查询启用: {stats.get('auto_query_enabled', False)}")
            print(f"  - 已接收帧数: {stats.get('frames_received', 0)}")
        
        # 等待几次查询周期，让后台获取到真实数据
        print("\n等待后台查询...")
        for i in range(10):  # 等待最多1秒 (10次 * 100ms)
            time.sleep(0.1)
            joints = robot.get_robot_state("joint")
            if joints is not None:
                # 检查是否是有效数据（不是 -12.5 rad）
                if all(abs(j) < 6.28 for j in joints):  # 有效范围约 ±360°
                    print(f"✓ 获取到有效关节数据 (等待了 {(i+1)*100}ms)")
                    break
                else:
                    print(f"  [{i+1}] 数据无效: {[f'{j:.2f}' for j in joints]}")
            else:
                print(f"  [{i+1}] 无法读取关节数据")
        
        # 再次检查通信统计
        if hasattr(robot.servo_driver, 'comm_manager') and robot.servo_driver.comm_manager:
            stats = robot.servo_driver.comm_manager.get_stats()
            print(f"\n通信统计:")
            print(f"  - 已发送命令: {stats.get('commands_sent', 0)}")
            print(f"  - 已接收帧数: {stats.get('frames_received', 0)}")
        
        # 读取初始状态
        print("\n" + "="*60)
        print("【初始状态】")
        print("="*60)
        
        initial_joints = robot.get_robot_state("joint")
        if initial_joints is not None:
            print(f"初始关节角度（弧度）: {[f'{j:.4f}' for j in initial_joints]}")
            print(f"初始关节角度（度数）: {[f'{j*57.2958:.2f}°' for j in initial_joints]}")
        else:
            print("⚠ 无法读取初始关节角度")
        
        # Get current gripper value (0-100)
        gripper_value = robot.get_robot_state("gripper")
        if gripper_value is not None:
            logger.info(f"初始夹爪值: {gripper_value:.1f} (0-100, 0=closed, 100=open)")
        else:
            logger.warning("Failed to read gripper value")
        
        # Test 1: Open gripper fully
        print("\n" + "="*60)
        print("【测试1】完全打开夹爪（gripper_value=100）")
        print("="*60)
        robot.set_robot_state(gripper_value=100, wait_for_completion=True)
        time.sleep(0.5)
        
        # 检查关节是否移动
        joints_after_test1 = robot.get_robot_state("joint")
        if joints_after_test1 is not None:
            print(f"测试1后关节角度（度数）: {[f'{j*57.2958:.2f}°' for j in joints_after_test1]}")
            if initial_joints is not None:
                joint_diff = [abs(j1 - j0) for j1, j0 in zip(joints_after_test1, initial_joints)]
                print(f"关节角度变化（度数）: {[f'{d*57.2958:.2f}°' for d in joint_diff]}")
                if max(joint_diff) > 0.01:  # 0.01 rad ≈ 0.57°
                    print("⚠️ 警告：关节位置发生了变化！")
                else:
                    print("✓ 关节位置未变化")
        
        gripper_after_test1 = robot.get_robot_state("gripper")
        print(f"测试1后夹爪值: {gripper_after_test1:.1f}")
        
        # Test 2: Close gripper
        print("\n" + "="*60)
        print("【测试2】完全关闭夹爪（gripper_value=0）")
        print("="*60)
        robot.set_robot_state(gripper_value=0, wait_for_completion=True)
        time.sleep(1.5)
        
        # 检查关节是否移动
        joints_after_test2 = robot.get_robot_state("joint")
        if joints_after_test2 is not None:
            print(f"测试2后关节角度（度数）: {[f'{j*57.2958:.2f}°' for j in joints_after_test2]}")
            if initial_joints is not None:
                joint_diff = [abs(j1 - j0) for j1, j0 in zip(joints_after_test2, initial_joints)]
                print(f"关节角度变化（度数）: {[f'{d*57.2958:.2f}°' for d in joint_diff]}")
                if max(joint_diff) > 0.01:
                    print("⚠️ 警告：关节位置发生了变化！")
                else:
                    print("✓ 关节位置未变化")
        
        gripper_after_test2 = robot.get_robot_state("gripper")
        print(f"测试2后夹爪值: {gripper_after_test2:.1f}")
        
        # Test 3: Partially open
        print("\n" + "="*60)
        print("【测试3】半开夹爪（gripper_value=50）")
        print("="*60)
        robot.set_robot_state(gripper_value=50, wait_for_completion=True)
        time.sleep(1.5)
        
        # 检查关节是否移动
        joints_after_test3 = robot.get_robot_state("joint")
        if joints_after_test3 is not None:
            print(f"测试3后关节角度（度数）: {[f'{j*57.2958:.2f}°' for j in joints_after_test3]}")
            if initial_joints is not None:
                joint_diff = [abs(j1 - j0) for j1, j0 in zip(joints_after_test3, initial_joints)]
                print(f"关节角度变化（度数）: {[f'{d*57.2958:.2f}°' for d in joint_diff]}")
                if max(joint_diff) > 0.01:
                    print("⚠️ 警告：关节位置发生了变化！")
                else:
                    print("✓ 关节位置未变化")
        
        gripper_after_test3 = robot.get_robot_state("gripper")
        print(f"测试3后夹爪值: {gripper_after_test3:.1f}")
        
        print("\n" + "="*60)
        print("【总结】")
        print("="*60)
        if initial_joints is not None and joints_after_test3 is not None:
            total_diff = [abs(j1 - j0) for j1, j0 in zip(joints_after_test3, initial_joints)]
            max_diff_deg = max(total_diff) * 57.2958
            print(f"整个测试过程中最大关节变化: {max_diff_deg:.2f}°")
            if max_diff_deg > 1.0:
                print("⚠️ 关节位置发生了明显变化！")
                print("   这可能表明 set_robot_state(gripper_value=X) 也在发送关节目标。")
            else:
                print("✓ 关节位置基本未变化（仅控制了夹爪）")
        print("="*60 + "\n")

        
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
