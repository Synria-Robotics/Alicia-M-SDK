"""14_demo_teleop_mapped.py — 遥操作 (带 URDF 限位映射): Alicia-D → Alicia-M

使用 Alicia-D 伺服示教臂实时控制 Alicia-M 电机操作臂，并使用 SDK 内置的
URDF 限位映射代替简单的符号翻转，具备:
- D 零位 → M 区间中点对齐
- 关节 3 按比例缩放（D/M 行程不同）
- 所有关节输出裁剪到 M 的 URDF 限位

MIT 模式支持逐电机设置阻抗参数（kp/kd/torque/vel_ref），修改文件顶部常量即可。

MIT 控制律: tau = kp * (pos_ref - pos_cur) + kd * (vel_ref - vel_cur) + t_ref

用法:
    # MIT 模式（默认）
    python 14_demo_teleop_mapped.py

    # MIT 模式 + 线性轨迹插值
    python 14_demo_teleop_mapped.py --interpolation --speed 200

    # PV 模式
    python 14_demo_teleop_mapped.py --mode pv

    # 指定串口
    python 14_demo_teleop_mapped.py --leader-port /dev/ttyACM0 --follower-port /dev/ttyACM1

    # 跳过回零
    python 14_demo_teleop_mapped.py --skip-home
"""

import argparse
import numpy as np

import alicia_d_sdk
import alicia_m_sdk
from alicia_m_sdk import ControlMode
from alicia_m_sdk.utils.beauty_logger import beauty_print


# MIT 默认阻抗参数（逐电机: M0~M5 关节, M6 夹爪）
MIT_KP = [500.0, 500.0, 500.0, 20.0, 60.0, 20.0, 150.0]
MIT_KD = [5.0, 5.0, 5.0, 1.0, 2.0, 1.0, 2.0]
MIT_TORQUE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
MIT_VEL_REF = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def main(args):
    mode = args.mode.lower()
    target_mode = ControlMode(mode)

    beauty_print(f"遥操作 (URDF映射): Alicia-D → Alicia-M ({mode.upper()} 模式)", type="module")

    # --- 连接 leader (Alicia-D 示教臂) ---
    beauty_print("连接 leader (Alicia-D)...", type="info")
    leader = alicia_d_sdk.create_robot(port=args.leader_port)
    if not leader.is_connected():
        beauty_print("leader 连接失败", type="error")
        return

    # --- 连接 follower (Alicia-M 操作臂) ---
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
        mit_gains_initialized = False

        # --- 可选: follower 先回零 ---
        if args.home:
            if target_mode == ControlMode.MIT:
                if follower.control_mode != ControlMode.MIT:
                    beauty_print("需要切换到 MIT 模式以使用指定阻抗参数回零", type="warning")
                    input("按 Enter 切换到 MIT 模式...")
                    follower.switch_mode("mit")
                beauty_print("初始化 Follower MIT 阻抗增益（读取当前 Kp/Kd 并线性过渡）...", type="info")
                follower.initialize_mit_gains(
                    kp=MIT_KP,
                    kd=MIT_KD,
                    torque=MIT_TORQUE,
                    vel_ref=MIT_VEL_REF,
                )
                mit_gains_initialized = True
            elif follower.control_mode != ControlMode.PV:
                beauty_print("需要临时切换到 PV 模式以执行回零", type="warning")
                input("按 Enter 切换到 PV 模式...")
                follower.switch_mode("pv")
            beauty_print("Follower 回零位...", type="info")
            if target_mode == ControlMode.MIT:
                follower.set_robot_state(
                    target_joints=[0.0] * 6,
                    joint_format="rad",
                    speed=20,
                    wait_for_completion=True,
                    kp=MIT_KP,
                    kd=MIT_KD,
                    torque=MIT_TORQUE,
                    vel_ref=MIT_VEL_REF,
                )
            else:
                follower.go_home(speed=20)
            beauty_print("Follower 已归零", type="success")

        # --- 检测并切换到目标控制模式 ---
        if follower.control_mode != target_mode:
            beauty_print(f"需要切换到 {mode.upper()} 模式，切换过程中机械臂将短暂失能", type="warning")
            input(f"按 Enter 切换到 {mode.upper()} 模式...")
            follower.switch_mode(mode)
            beauty_print(f"已切换到 {mode.upper()} 模式", type="success")

        if target_mode == ControlMode.MIT and not mit_gains_initialized:
            beauty_print("初始化 Follower MIT 阻抗增益（读取当前 Kp/Kd 并线性过渡）...", type="info")
            follower.initialize_mit_gains(
                kp=MIT_KP,
                kd=MIT_KD,
                torque=MIT_TORQUE,
                vel_ref=MIT_VEL_REF,
            )

        # --- 打印初始状态 ---
        leader_joints = leader.get_robot_state("joint")
        follower_joints = follower.get_robot_state("joint")
        if leader_joints is not None:
            beauty_print(f"Leader  关节 (deg): {np.round(np.degrees(leader_joints), 1).tolist()}")
        if follower_joints is not None:
            beauty_print(f"Follower 关节 (deg): {np.round(np.degrees(follower_joints), 1).tolist()}")

        # --- 创建遥操作控制器（使用 URDF 限位映射 + 逐电机 MIT 参数） ---
        teleop = follower.create_mapped_teleoperation(
            leader=leader,
            frequency_hz=args.frequency,
            follower_speed=args.speed,
            use_interpolation=args.interpolation,
            kp=MIT_KP,
            kd=MIT_KD,
            torque=MIT_TORQUE,
            vel_ref=MIT_VEL_REF,
        )

        if args.verbose:
            teleop.set_state_callback(follower.make_mapped_teleop_state_printer(args.frequency))

        # --- 等待用户确认后启动 ---
        interp_str = "插值" if args.interpolation else "无插值"
        beauty_print(
            f"遥操作配置: {args.frequency} Hz, {mode.upper()} 模式 ({interp_str}), speed={args.speed}",
            type="module",
        )
        beauty_print("使用 URDF 限位映射 (零点对齐 + 比例缩放 + 限位保护)", type="info")
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
        description="Teleoperate Alicia-M from Alicia-D with URDF-limit joint mapping.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('--mode', type=str, default="mit",
                        choices=["pv", "mit"],
                        help="Control mode: pv or mit; default mit.")
    parser.add_argument('--leader-port', type=str, default="COM63",
                        help="Leader serial port for Alicia-D.")
    parser.add_argument('--port', '--follower-port', default="COM51", dest='follower_port',
                        type=str,
                        help="Follower serial port for Alicia-M; omit to auto-detect.")
    parser.add_argument('--follower-version', type=str, default="v1_1",
                        help="Alicia-M hardware version: v1_0 or v1_1; default v1_1.")
    parser.add_argument('--frequency', type=float, default=100.0,
                        help="Control loop frequency in Hz, recommended 10-200; default 100.")
    parser.add_argument('--speed', type=float, default=200.0,
                        help="Follower motion speed, range 0-400; maps to 0-10 rad/s.")
    parser.add_argument('--interpolation', action='store_true',
                        help="Enable linear trajectory interpolation in MIT mode.")
    parser.add_argument('--home', action='store_true',
                        help="Move the follower to zero before teleoperation.")
    parser.add_argument('--verbose', '-v', action='store_true',
                        help="Print joint states during teleoperation, including mapped values.")
    args = parser.parse_args()

    main(args)
