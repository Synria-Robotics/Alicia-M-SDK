#!/usr/bin/env python3
"""18_demo_mit_torque_switch.py — MIT 力矩开关测试

测试流程：
1. 连接机器人并切到 MIT 模式
2. 直接执行 torque off（内部使用 MIT 零阻抗实现）
3. 手动拖动机械臂
4. 执行 torque on（内部恢复默认阻抗）
"""

import argparse
import math
import time

import alicia_m_sdk
from demo_common import add_port_argument
from alicia_m_sdk.utils.beauty_logger import beauty_print, beauty_print_array


def _print_joints(robot, label):
    """Print current joint angles in deg/rad.

    :param robot: Connected robot instance
    :param label: Prefix label for log message
    :return: None
    """
    joints = robot.get_robot_state("joint")
    if joints is None:
        beauty_print(f"{label}: 无法读取关节角度", type="warning")
        return
    joints_deg = [a * 180.0 / math.pi for a in joints]
    beauty_print(f"{label} (deg): {beauty_print_array(joints_deg, precision=2)}", type="info")
    beauty_print(f"{label} (rad): {beauty_print_array(joints, precision=4)}", type="info")


def main(args):
    beauty_print("Demo: MIT 力矩开关测试", type="module")
    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print(f"机器人连接成功（当前 {robot.control_mode.value.upper()} 模式）", type="success")

    try:
        if robot.control_mode.value != "mit":
            beauty_print("本示例需要 MIT 模式，切换过程中机械臂将短暂失能", type="warning")
            input("按 Enter 切换到 MIT 模式...")
            robot.switch_mode("mit")
            beauty_print("已切换到 MIT 模式", type="success")

        _print_joints(robot, "当前关节")
        beauty_print("即将 torque off（使用新实现），机械臂将可自由拖动。", type="warning")
        input("确认安全后按 Enter 执行 torque off...")
        if robot.torque_control("off"):
            beauty_print("torque off 成功，请手动拖动机械臂到新位置。", type="success")
        else:
            beauty_print("torque off 失败。", type="error")
            return

        input("\n拖动完成后按 Enter 读取当前位置...")
        _print_joints(robot, "拖动后关节")

        beauty_print("即将 torque on，机械臂将恢复力矩锁定。", type="warning")
        input("确认安全后按 Enter 执行 torque on...")
        if robot.torque_control("on"):
            beauty_print("torque on 成功。", type="success")
        else:
            beauty_print("torque on 失败。", type="error")
            return

        time.sleep(0.2)
        _print_joints(robot, "恢复后关节")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test MIT torque off/on on Alicia-M")
    add_port_argument(parser)
    args = parser.parse_args()
    main(args)
