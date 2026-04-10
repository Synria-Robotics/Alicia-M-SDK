"""10_demo_move_full_arm_mit.py — 关节+夹爪协同控制 (MIT)

演示 MIT 模式下关节与夹爪的独立控制及协同控制：
  回零+夹爪打开 -> 仅关节运动 -> 仅夹爪运动 -> 同时控制 -> 回零。
"""

import time
import alicia_m_sdk
from robocore.utils.beauty_logger import beauty_print, beauty_print_array


# 预设安全关节位置 (度)
POSE_A = [90, -90.0, -90.0, 90.0, 0.0, 0.0]
POSE_B = [0, -30.0, -60.0, 45.0, 0.0, 45.0]


def main():
    beauty_print("Demo: 关节+夹爪协同控制 (MIT)", type="module")

    # 创建并连接机器人（MIT 模式）
    robot = alicia_m_sdk.create_robot(control_mode="mit")
    beauty_print("机器人连接成功（MIT 模式）", type="success")

    try:
        # --- 初始化：回零位 + 夹爪打开 ---
        beauty_print("初始化: 回零位，夹爪打开...", type="info")
        robot.set_robot_state(
            target_joints=[0, 0, 0, 0, 0, 0],
            gripper_value=1000,
            joint_format="deg",
            speed=30,
            gripper_speed=100,
            wait_for_completion=True,
        )
        beauty_print("初始化完成", type="success")
        time.sleep(1.0)

        # --- 演示 1：仅控制关节（夹爪保持不变）---
        beauty_print("演示 1: 仅控制关节（夹爪保持不变）", type="module")
        beauty_print(f"  目标关节: {POSE_A} (deg)", type="info")
        robot.set_robot_state(
            target_joints=POSE_A,
            gripper_value=None,
            joint_format="deg",
            speed=30,
            wait_for_completion=True,
        )
        beauty_print("关节运动完成", type="success")
        time.sleep(1.0)

        # --- 演示 2：仅控制夹爪（关节保持不变）---
        beauty_print("演示 2: 仅控制夹爪（关节保持不变）", type="module")
        beauty_print("  关闭夹爪 (0)...", type="info")
        robot.set_robot_state(
            target_joints=None,
            gripper_value=0,
            gripper_speed=100,
            wait_for_completion=True,
        )
        beauty_print("夹爪关闭完成", type="success")
        time.sleep(1.0)

        # --- 演示 3：同时控制关节和夹爪 ---
        beauty_print("演示 3: 同时控制关节和夹爪", type="module")
        beauty_print(f"  目标关节: {POSE_B} (deg), 夹爪打开 (1000)", type="info")
        robot.set_robot_state(
            target_joints=POSE_B,
            gripper_value=1000,
            joint_format="deg",
            speed=30,
            gripper_speed=100,
            wait_for_completion=True,
        )
        beauty_print("关节+夹爪协同运动完成", type="success")
        time.sleep(1.0)

        # --- 回零位 ---
        beauty_print("回零位，夹爪闭合...", type="info")
        robot.set_robot_state(
            target_joints=[0, 0, 0, 0, 0, 0],
            gripper_value=0,
            joint_format="deg",
            speed=30,
            gripper_speed=100,
            wait_for_completion=True,
        )
        beauty_print("回零完成", type="success")
        time.sleep(1.0)

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
