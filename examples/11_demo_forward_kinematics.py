"""11_demo_forward_kinematics.py — 正运动学

演示正运动学（FK）计算：
  1. 读取当前关节角度 -> 计算 FK -> 打印末端位姿
  2. 给定预设关节角度 -> 计算 FK -> 打印末端位姿
"""

import argparse
import math
import numpy as np
import alicia_m_sdk
from alicia_m_sdk import forward_kinematics, RobotModel
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from demo_utils.demo_common import add_port_argument
from alicia_m_sdk.utils.beauty_logger import beauty_print, beauty_print_array
from robocore.transform import matrix_to_euler, matrix_to_quaternion


# 预设关节角度 (度)
PRESET_JOINTS_DEG = [0, 0, 0, 0, 0, 90]


def _print_pose(label: str, T: np.ndarray):
    """打印末端位姿的详细信息"""
    position = T[:3, 3]
    rotation = T[:3, :3]
    euler_xyz = matrix_to_euler(rotation, seq='xyz')
    quat_xyzw = matrix_to_quaternion(rotation)

    beauty_print(f"{label}", type="module")
    beauty_print(f"  位置 (m):          {beauty_print_array(position, precision=5)}", type="info")
    beauty_print(f"  欧拉角 XYZ (deg):  {beauty_print_array(np.rad2deg(euler_xyz), precision=2)}", type="info")
    beauty_print(f"  欧拉角 XYZ (rad):  {beauty_print_array(euler_xyz, precision=5)}", type="info")
    beauty_print(f"  四元数 (xyzw):     {beauty_print_array(quat_xyzw, precision=6)}", type="info")
    beauty_print("  旋转矩阵:", type="info")
    beauty_print(f"    {beauty_print_array(rotation, precision=6)}", type="info")


def main():
    beauty_print("Demo: 正运动学 (FK)", type="module")

    parser = argparse.ArgumentParser(description="Run Alicia-M forward kinematics demo.")
    add_port_argument(parser)
    args = parser.parse_args()

    # 创建并连接机器人
    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print("机器人连接成功", type="success")

    try:
        robot_model = robot.robot_model

        # --- 1. 当前关节角度的 FK ---
        beauty_print("1. 当前关节角度的正运动学", type="module")
        joints_rad = robot.get_robot_state("joint")
        if joints_rad is not None:
            joints_deg = [a * 180.0 / math.pi for a in joints_rad]
            beauty_print(f"  当前关节角度 (deg): {beauty_print_array(joints_deg, precision=2)}", type="info")
            beauty_print(f"  当前关节角度 (rad): {beauty_print_array(joints_rad, precision=4)}", type="info")

            # 计算 FK
            T_fk = forward_kinematics(robot_model, joints_rad, return_end=True)
            _print_pose("  当前末端位姿", T_fk)
        else:
            beauty_print("  无法获取当前关节角度", type="warning")

        # --- 2. 预设关节角度的 FK ---
        beauty_print("2. 预设关节角度的正运动学", type="module")
        preset_rad = [d * math.pi / 180.0 for d in PRESET_JOINTS_DEG]
        beauty_print(f"  预设关节角度 (deg): {PRESET_JOINTS_DEG}", type="info")
        beauty_print(f"  预设关节角度 (rad): {beauty_print_array(preset_rad, precision=4)}", type="info")

        T_preset = forward_kinematics(robot_model, preset_rad, return_end=True)
        _print_pose("  预设角度末端位姿", T_preset)

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
