"""07_demo_move_joint.py — 关节控制 (PV)

演示 PV 模式关节空间运动控制：回零 -> 目标位置 -> 回零。
"""

import argparse
import math
import time
import alicia_m_sdk
from _demo_helpers import add_port_argument, beauty_print, beauty_print_array


# 预设安全关节位置 (度)
# POSITION = [0.0, -150.0, -150.0, 0.0, 0.0, 0.0]
# POSITION = [0, 0, 0, 0, 0, 20]
POSITION = [21, -67, -19, 8, 56, -7]
# POSITION = [21, -67, -150, 8, 56, -7]
HOME_POSITION = [0.0] * 6


def build_joint_error_report(target_deg, feedback_rad):
    """Build per-joint error values in radians and degrees."""
    if len(target_deg) != 6 or len(feedback_rad) < 6:
        raise ValueError("expected 6 target joints and at least 6 feedback joints")

    target_rad = [math.radians(float(value)) for value in target_deg[:6]]
    error_rad = [
        float(actual) - expected
        for actual, expected in zip(feedback_rad[:6], target_rad)
    ]
    return {
        "error_rad": [round(value, 4) for value in error_rad],
        "error_deg": [round(math.degrees(value), 4) for value in error_rad],
    }


def print_joint_error(robot, target_deg, label):
    """Read current joint state and print target-vs-feedback error."""
    feedback_rad = robot.get_robot_state("joint")
    if feedback_rad is None:
        beauty_print(f"{label}：无法读取关节反馈，跳过误差打印", type="warning")
        return

    report = build_joint_error_report(target_deg, feedback_rad)
    beauty_print(f"{label} 角度误差 (rad, 反馈-目标): {beauty_print_array(report['error_rad'], precision=4)}", type="info")
    beauty_print(f"{label} 角度误差 (deg, 反馈-目标): {beauty_print_array(report['error_deg'], precision=4)}", type="info")


def main():
    beauty_print("Demo: 关节控制 (PV)", type="module")

    parser = argparse.ArgumentParser(description="Move one Alicia-M joint in PV mode.")
    parser.add_argument(
        "--speed", type=float, default=15,
        help="Motion speed; default 15, range 0-400."
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
        print_joint_error(robot, HOME_POSITION, "回零位")
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
        print_joint_error(robot, POSITION, "目标位置")
        time.sleep(1.0)

        # --- 回零位 ---
        beauty_print("回零位...", type="info")
        robot.go_home(speed=args.speed)
        beauty_print("已到达零位", type="success")
        print_joint_error(robot, HOME_POSITION, "回零位")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
