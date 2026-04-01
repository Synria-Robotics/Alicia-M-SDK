"""08_demo_move_full_arm.py — 关节+夹爪协同控制

演示关节与夹爪的独立控制及协同控制：
  回零+夹爪打开 -> 仅关节运动 -> 仅夹爪运动 -> 同时控制 -> 回零。
PV 和 MIT 模式下均可运行。
"""

import time
import alicia_m_sdk
from robocore.utils.beauty_logger import beauty_print, beauty_print_array


# 预设安全关节位置 (度)
POSE_A = [0, 30, 0, 0, -30, 0]
POSE_B = [0, -30, 0, 0, 30, 0]


def main():
    beauty_print("Demo: 关节+夹爪协同控制", type="module")

    # 创建并连接机器人
    robot = alicia_m_sdk.create_robot(control_mode="pv")
    beauty_print("机器人连接成功", type="success")

    try:
        # --- 初始化：回零位 + 夹爪打开 ---
        beauty_print("初始化: 回零位，夹爪打开...", type="info")
        robot.set_robot_state(
            target_joints=[0, 0, 0, 0, 0, 0],
            gripper_value=1000,
            joint_format="deg",
            speed=15,
            wait_for_completion=True,
        )
        beauty_print("初始化完成", type="success")
        time.sleep(1.0)

        # --- 演示 1：仅控制关节（夹爪保持不变）---
        beauty_print("演示 1: 仅控制关节（夹爪保持不变）", type="module")
        beauty_print(f"  目标关节: {POSE_A} (deg)", type="info")
        robot.set_robot_state(
            target_joints=POSE_A,
            gripper_value=None,  # 夹爪保持当前状态
            joint_format="deg",
            speed=15,
            wait_for_completion=True,
        )
        beauty_print("关节运动完成", type="success")
        time.sleep(1.0)

        # --- 演示 2：仅控制夹爪（关节保持不变）---
        beauty_print("演示 2: 仅控制夹爪（关节保持不变）", type="module")
        beauty_print("  关闭夹爪 (0)...", type="info")
        robot.set_robot_state(
            target_joints=None,  # 关节保持当前位置
            gripper_value=0,
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
            speed=15,
            wait_for_completion=True,
        )
        beauty_print("关节+夹爪协同运动完成", type="success")
        time.sleep(1.0)

        # --- 回零位 ---
        beauty_print("回零位...", type="info")
        robot.go_home(speed=15)
        beauty_print("已到达零位", type="success")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
