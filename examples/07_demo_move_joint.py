"""07_demo_move_joint.py — 关节控制 (PV)

演示 PV 模式关节空间运动控制：回零 -> 目标位置 -> 回零。
"""

import argparse
import time
import alicia_m_sdk
from alicia_m_sdk.demo_utils.demo_common import add_port_argument
from alicia_m_sdk.utils.beauty_logger import beauty_print, beauty_print_array


# 预设安全关节位置 (度)
POSITION = [0.0, -150.0, -150.0, 0.0, 0.0, 0.0]
# POSITION = [0, 0, 0, 0, 0, 20]

def main():
    beauty_print("Demo: 关节控制 (PV)", type="module")

    parser = argparse.ArgumentParser(description="关节控制示例 (PV)")
    parser.add_argument(
        "--speed", type=float, default=15,
        help="运动速度 (默认: 15, 范围: 0-400)"
    )
    add_port_argument(parser)
    args = parser.parse_args()

    # 创建并连接机器人（不指定模式，避免自动切换）
    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print(f"机器人连接成功（当前 {robot.control_mode.value.upper()} 模式）", type="success")

    # 需要 PV 模式，若当前不是则提示用户确认后切换
    if robot.control_mode.value != "pv":
        beauty_print("本示例需要 PV 模式，切换过程中机械臂将短暂失能", type="warning")
        input("按 Enter 切换到 PV 模式...")
        robot.switch_mode("pv")
        beauty_print("已切换到 PV 模式", type="success")

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
