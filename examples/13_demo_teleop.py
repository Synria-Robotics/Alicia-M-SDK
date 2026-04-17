"""13_demo_teleop.py — 遥操作: Alicia-D (示教臂) → Alicia-M (操作臂)

使用 Alicia-D 伺服示教臂实时控制 Alicia-M 电机操作臂。
支持 PV 模式和 MIT 模式（默认 MIT），MIT 模式支持逐电机设置阻抗参数。

MIT 控制律: tau = kp * (pos_ref - pos_cur) + kd * (vel_ref - vel_cur) + t_ref

用法:
    # MIT 模式遥操作（默认，不使用插值）
    python 13_demo_teleop.py

    # MIT 模式 + 线性轨迹插值
    python 13_demo_teleop.py --interpolation --speed 200

    # PV 模式遥操作
    python 13_demo_teleop.py --mode pv

    # 指定串口
    python 13_demo_teleop.py --leader-port /dev/ttyACM0 --follower-port /dev/ttyACM1

    # 调整频率和速度
    python 13_demo_teleop.py --frequency 100 --speed 300

    # 跳过回零
    python 13_demo_teleop.py --skip-home
"""

import argparse
import numpy as np

import alicia_d_sdk
import alicia_m_sdk
from alicia_m_sdk import ControlMode
from alicia_m_sdk.control.teleoperation import Teleoperation
from robocore.utils.beauty_logger import beauty_print


# MIT 默认阻抗参数（逐电机: M0~M5 关节, M6 夹爪）
MIT_KP = [150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0]
MIT_KD = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
MIT_TORQUE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
MIT_VEL_REF = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def main(args):
    mode = args.mode.lower()
    target_mode = ControlMode(mode)

    beauty_print(f"遥操作: Alicia-D → Alicia-M ({mode.upper()} 模式)", type="module")

    # --- 连接 leader (Alicia-D 示教臂) ---
    beauty_print("连接 leader (Alicia-D)...", type="info")
    leader = alicia_d_sdk.create_robot(port=args.leader_port)
    if not leader.is_connected():
        beauty_print("leader 连接失败", type="error")
        return

    # --- 连接 follower (Alicia-M 操作臂, 自动检测模式) ---
    # 显式指定 control_aim="follower"，防止端口接反时误操作示教臂
    beauty_print("连接 follower (Alicia-M)...", type="info")
    follower = alicia_m_sdk.create_robot(
        port=args.follower_port,
        version=args.follower_version,
        control_aim="follower",
    )
    if not follower.is_connected():
        beauty_print("follower 连接失败", type="error")
        leader.disconnect()
        return

    beauty_print("两臂均连接成功", type="success")
    beauty_print(f"Follower 当前模式: {follower.control_mode.value.upper()}", type="info")

    try:
        # --- 可选: follower 先回零（在 PV 模式下执行以确保等待到达） ---
        if args.home:
            if follower.control_mode != ControlMode.PV:
                beauty_print("需要临时切换到 PV 模式以执行回零", type="warning")
                input("按 Enter 切换到 PV 模式...")
                follower.switch_mode("pv")
            beauty_print("Follower 回零位...", type="info")
            follower.go_home(speed=20)
            beauty_print("Follower 已归零", type="success")

        # --- 检测并切换到目标控制模式 ---
        if follower.control_mode != target_mode:
            beauty_print(f"需要切换到 {mode.upper()} 模式，切换过程中机械臂将短暂失能", type="warning")
            input(f"按 Enter 切换到 {mode.upper()} 模式...")
            follower.switch_mode(mode)
            beauty_print(f"已切换到 {mode.upper()} 模式", type="success")

        # --- 打印初始状态 ---
        leader_joints = leader.get_robot_state("joint")
        follower_joints = follower.get_robot_state("joint")
        if leader_joints is not None:
            beauty_print(f"Leader  关节 (deg): {np.round(np.degrees(leader_joints), 1).tolist()}")
        if follower_joints is not None:
            beauty_print(f"Follower 关节 (deg): {np.round(np.degrees(follower_joints), 1).tolist()}")

        # --- 创建遥操作控制器（逐电机 MIT 参数） ---
        teleop = Teleoperation(
            leader=leader,
            follower=follower,
            frequency_hz=args.frequency,
            follower_speed=args.speed,
            joint_signs=[1.0, 1.0, -1.0, -1.0, 1.0, -1.0],
            use_interpolation=args.interpolation,
            kp=MIT_KP,
            kd=MIT_KD,
            torque=MIT_TORQUE,
            vel_ref=MIT_VEL_REF,
        )

        if args.verbose:
            def print_state(joints, gripper, count):
                if count % int(args.frequency) == 0:  # 每秒打印一次
                    deg = np.round(np.degrees(joints), 1).tolist()
                    print(f"  [{count:6d}] joints={deg}  gripper={gripper:.0f}")
            teleop.set_state_callback(print_state)

        # --- 等待用户确认后启动 ---
        interp_str = "插值" if args.interpolation else "无插值"
        beauty_print(
            f"遥操作配置: {args.frequency} Hz, {mode.upper()} 模式 ({interp_str}), speed={args.speed}",
            type="module",
        )
        beauty_print("拖动 leader (Alicia-D) 控制 follower (Alicia-M)")
        input("\n按 Enter 开始遥操作...")

        teleop.run_interactive()

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
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

    parser.add_argument('--mode', type=str, default="mit",
                        choices=["pv", "mit"],
                        help="控制模式: pv / mit (默认: mit)")
    parser.add_argument('--leader-port', type=str, default="/dev/ttyACM0",
                        help="Leader 串口 (Alicia-D)")
    parser.add_argument('--port', '--follower-port', dest='follower_port',
                        type=str, default="",
                        help="Follower 串口 (Alicia-M)，不指定则自动发现")
    parser.add_argument('--follower-version', type=str, default="v1_1",
                        help="Alicia-M 硬件版本，可选 v1_0/v1_1 (默认: v1_1)")
    parser.add_argument('--frequency', type=float, default=100.0,
                        help="控制循环频率 [10-200] Hz (默认: 100)")
    parser.add_argument('--speed', type=float, default=400.0,
                        help="Follower 运动速度 [0-400]，映射到 [0-10] rad/s (默认: 400, PV/MIT 均生效)")
    parser.add_argument('--interpolation', action='store_true',
                        help="MIT 模式启用线性轨迹插值（运动更平滑，建议配合 --speed 200 使用）")
    parser.add_argument('--home', action='store_true',
                        help="启动前 follower 先回零（默认跳过）")
    parser.add_argument('--verbose', '-v', action='store_true',
                        help="打印遥操作过程中的关节状态")

    main(parser.parse_args())
