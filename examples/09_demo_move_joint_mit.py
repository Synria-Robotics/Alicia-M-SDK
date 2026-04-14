"""09_demo_move_joint_mit.py — 关节控制 (MIT)

演示 MIT 模式关节空间运动控制：回零 -> 目标位置 -> 回零。
"""

import argparse
import time
import alicia_m_sdk
from robocore.utils.beauty_logger import beauty_print, beauty_print_array


# 预设安全关节位置 (度)
POSITION = [90, -90.0, -90.0, 90.0, 0.0, 0.0]
# POSITION = [0, 0.0, -30.0, 0.0, 0.0, 0.0]
# POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def main():
    beauty_print("Demo: 关节控制 (MIT)", type="module")

    parser = argparse.ArgumentParser(description="关节控制示例 (MIT)")
    parser.add_argument(
        "--speed", type=float, default=30,
        help="运动速度 (默认: 30, 范围: 0-400)"
    )
    args = parser.parse_args()

    # 创建并连接机器人（MIT 模式）
    robot = alicia_m_sdk.create_robot(control_mode="mit")
    beauty_print("机器人连接成功（MIT 模式）", type="success")

    try:
        # --- 回零位 ---
        beauty_print("回零位...", type="info")
        robot.go_home(speed=args.speed)
        beauty_print("已到达零位", type="success")
        time.sleep(1.0)

        # --- 移动到目标位置 ---
        beauty_print(f"移动到目标位置: {POSITION} (deg)...", type="info")
        robot.set_robot_state(
            target_joints=POSITION,
            joint_format="deg",
            speed=args.speed,
            wait_for_completion=True,
        )
        beauty_print("已到达目标位置", type="success")
        time.sleep(1.0)

        # --- 回零位 ---
        beauty_print("回零位...", type="info")
        robot.go_home(speed=args.speed)
        beauty_print("已到达零位", type="success")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
