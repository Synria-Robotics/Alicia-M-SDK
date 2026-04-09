"""16_demo_teleop.py — 遥操作: Alicia-D (示教臂) → Alicia-M (操作臂)

使用 Alicia-D 伺服示教臂实时控制 Alicia-M 电机操作臂（PV 模式）。
两臂需通过 USB 串口连接至同一台电脑。

用法:
    # 自动检测串口
    python 16_demo_teleop.py

    # 指定串口
    python 16_demo_teleop.py --leader-port /dev/ttyACM0 --follower-port /dev/ttyACM1

    # 调整频率和速度
    python 16_demo_teleop.py --frequency 100 --speed 300

    # 跳过回零
    python 16_demo_teleop.py --skip-home
"""

import argparse
import numpy as np

import alicia_d_sdk
import alicia_m_sdk
from alicia_m_sdk.control.teleoperation import Teleoperation
from robocore.utils.beauty_logger import beauty_print


def main(args):
    beauty_print("遥操作: Alicia-D → Alicia-M (PV 模式)", type="module")

    # --- 连接 leader (Alicia-D 示教臂) ---
    beauty_print("连接 leader (Alicia-D)...", type="info")
    leader = alicia_d_sdk.create_robot(port=args.leader_port)
    if not leader.is_connected():
        beauty_print("leader 连接失败", type="error")
        return

    # --- 连接 follower (Alicia-M 操作臂, PV 模式) ---
    beauty_print("连接 follower (Alicia-M)...", type="info")
    follower = alicia_m_sdk.create_robot(
        port=args.follower_port,
        version=args.follower_version,
        control_mode="pv",
    )
    if not follower.is_connected():
        beauty_print("follower 连接失败", type="error")
        leader.disconnect()
        return

    beauty_print("两臂均连接成功", type="success")

    # --- 打印初始状态 ---
    leader_joints = leader.get_robot_state("joint")
    follower_joints = follower.get_robot_state("joint")
    if leader_joints is not None:
        beauty_print(f"Leader  关节 (deg): {np.round(np.degrees(leader_joints), 1).tolist()}")
    if follower_joints is not None:
        beauty_print(f"Follower 关节 (deg): {np.round(np.degrees(follower_joints), 1).tolist()}")

    # --- 可选: follower 先回零 ---
    if not args.skip_home:
        beauty_print("Follower 回零位...", type="info")
        follower.go_home(speed=20)
        beauty_print("Follower 已归零", type="success")

    # --- 创建遥操作控制器 ---
    teleop = Teleoperation(
        leader=leader,
        follower=follower,
        frequency_hz=args.frequency,
        follower_speed=args.speed,
        joint_signs=[1.0, 1.0, -1.0, -1.0, 1.0, -1.0],
    )

    if args.verbose:
        def print_state(joints, gripper, count):
            if count % int(args.frequency) == 0:  # 每秒打印一次
                deg = np.round(np.degrees(joints), 1).tolist()
                print(f"  [{count:6d}] joints={deg}  gripper={gripper:.0f}")
        teleop.set_state_callback(print_state)

    beauty_print(
        f"启动遥操作: {args.frequency} Hz, PV 模式 (speed={args.speed})",
        type="module",
    )
    beauty_print("拖动 leader (Alicia-D) 控制 follower (Alicia-M)")
    beauty_print("按 Enter 或 Ctrl+C 停止\n")

    try:
        teleop.run_interactive()
    finally:
        beauty_print("断开连接...", type="info")
        leader.disconnect()
        follower.disconnect()
        beauty_print("完成", type="success")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="遥操作: Alicia-D (示教臂) → Alicia-M (操作臂)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('--leader-port', type=str, default="/dev/ttyACM0",
                        help="Leader 串口 (Alicia-D)")
    parser.add_argument('--follower-port', type=str, default="/dev/ttyACM1",
                        help="Follower 串口 (Alicia-M)")
    parser.add_argument('--follower-version', type=str, default="v1_1",
                        help="Alicia-M 硬件版本，可选 v1_0/v1_1 (默认: v1_1)")
    parser.add_argument('--frequency', type=float, default=100.0,
                        help="控制循环频率 [10-200] Hz (默认: 100)")
    parser.add_argument('--speed', type=float, default=400.0,
                        help="Follower 运动速度 [0-400]，映射到 [0-10] rad/s (默认: 400)")
    parser.add_argument('--skip-home', action='store_true',
                        help="跳过 follower 回零")
    parser.add_argument('--verbose', '-v', action='store_true',
                        help="打印遥操作过程中的关节状态")

    main(parser.parse_args())
