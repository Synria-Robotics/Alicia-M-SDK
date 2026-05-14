"""09_demo_move_joint_mit.py — 关节控制 (MIT)

演示 MIT 模式关节空间运动控制：回零 -> 目标位置 -> 回零。
使用逐电机 MIT 阻抗参数，展示 kp/kd/torque/vel_ref 的逐关节设置方式。

MIT 控制律: tau = kp * (pos_ref - pos_cur) + kd * (vel_ref - vel_cur) + t_ref
"""

import argparse
import time
import alicia_m_sdk
from _common import add_port_argument
from alicia_m_sdk.utils.beauty_logger import beauty_print, beauty_print_array


# 预设安全关节位置 (度)
POSITION = [0.0, -90.0, -90.0, 90.0, 0.0, 0.0]

# MIT 默认阻抗参数（逐电机: M0~M5 关节, M6 夹爪）
MIT_KP = [500.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0]
MIT_KD = [5.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
MIT_TORQUE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
MIT_VEL_REF = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def main():
    beauty_print("Demo: 关节控制 (MIT)", type="module")

    parser = argparse.ArgumentParser(description="关节控制示例 (MIT)")
    parser.add_argument(
        "--speed", type=float, default=30,
        help="运动速度 (默认: 30, 范围: 0-400)"
    )
    add_port_argument(parser)
    args = parser.parse_args()

    # 创建并连接机器人（不指定模式，避免自动切换）
    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print(f"机器人连接成功（当前 {robot.control_mode.value.upper()} 模式）", type="success")

    # 需要 MIT 模式，若当前不是则提示用户确认后切换
    if robot.control_mode.value != "mit":
        beauty_print("本示例需要 MIT 模式，切换过程中机械臂将短暂失能", type="warning")
        input("按 Enter 切换到 MIT 模式...")
        robot.switch_mode("mit")
        beauty_print("已切换到 MIT 模式", type="success")

    try:
        beauty_print("初始化 MIT 阻抗增益（读取当前 Kp/Kd 并线性过渡）...", type="info")
        robot.initialize_mit_gains(
            kp=MIT_KP,
            kd=MIT_KD,
            torque=MIT_TORQUE,
            vel_ref=MIT_VEL_REF,
        )

        # --- 回零位 ---
        beauty_print("回零位...", type="info")
        robot.set_robot_state(
            target_joints=[0.0] * 6,
            joint_format="rad",
            speed=args.speed,
            wait_for_completion=True,
            kp=MIT_KP,
            kd=MIT_KD,
            torque=MIT_TORQUE,
            vel_ref=MIT_VEL_REF,
        )
        beauty_print("已到达零位", type="success")
        time.sleep(1.0)

        # --- 移动到目标位置（逐电机 MIT 参数） ---
        beauty_print(f"移动到目标位置: {POSITION} (deg)...", type="info")
        robot.set_robot_state(
            target_joints=POSITION,
            joint_format="deg",
            speed=args.speed,
            wait_for_completion=True,
            kp=MIT_KP,
            kd=MIT_KD,
            torque=MIT_TORQUE,
            vel_ref=MIT_VEL_REF,
        )
        beauty_print("已到达目标位置", type="success")
        time.sleep(1.0)

        # --- 回零位 ---
        beauty_print("回零位...", type="info")
        robot.set_robot_state(
            target_joints=[0.0] * 6,
            joint_format="rad",
            speed=args.speed,
            wait_for_completion=True,
            kp=MIT_KP,
            kd=MIT_KD,
            torque=MIT_TORQUE,
            vel_ref=MIT_VEL_REF,
        )
        beauty_print("已到达零位", type="success")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
