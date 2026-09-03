#!/usr/bin/env python3
"""Alicia-M 6+1 关节滑块姿态记录与精确回放 Demo。

记录模式提供固定 720x480 窗口，通过 6 个弧度关节滑块和 1 个夹爪滑块
控制机械臂。关节目标也可直接输入；失焦或按 Enter 后经限位校验再发送。
点击记录时保存的是机械臂实际反馈，而不是滑块目标。

回放模式会先完整校验 TXT，再逐行运动。终端原地显示目标、最大关节误差、
夹爪误差和连续到位计数，默认关节到位阈值为 0.008 rad。程序还会监测
固件真实 PV/MIT 模式：MIT 下停止发送并跟随手动姿态，返回 PV 后同步当前
位置且清零线性插补速度，避免突然返回旧目标。
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import alicia_m_sdk
from _demo_helpers import add_port_argument, beauty_print
from alicia_m_sdk.utils.pose_slider import (
    ConsolePlaybackProgress,
    DEFAULT_JOINT_LIMITS_RAD,
    PoseFileError,
    PosePlaybackError,
    PoseSliderWindow,
    count_pose_lines,
    execute_pose_sequence,
    load_poses,
    parse_urdf_revolute_limits,
    read_actual_control_mode,
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="用 6+1 滑块记录 Alicia-M 实际姿态，或从 TXT 逐行回放。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    add_port_argument(parser)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--record", metavar="TXT", help="打开 720x480 滑块窗口并记录到 TXT")
    action.add_argument("--execute", metavar="TXT", help="校验并逐行执行 TXT 中的姿态")
    parser.add_argument("--urdf", help="可选 URDF 路径；默认使用硬件自动匹配的 follower URDF")
    parser.add_argument("--speed", type=float, default=15.0, help="关节速度参数 (1~100)")
    parser.add_argument("--gripper-speed", type=float, default=100.0, help="夹爪速度参数 (1~400)")
    parser.add_argument("--send-interval-ms", type=int, default=120, help="GUI 拖动目标最短发送间隔 (50~500 ms)")
    parser.add_argument("--tolerance", type=float, default=0.008, help="回放关节到位误差 (rad)")
    parser.add_argument("--gripper-tolerance", type=float, default=10.0, help="回放夹爪到位误差")
    parser.add_argument("--timeout", type=float, default=10.0, help="每个姿态的到位超时 (s)")
    parser.add_argument("--dwell", type=float, default=0.3, help="每个姿态到位后的停留时间 (s)")
    parser.add_argument("--yes", action="store_true", help="跳过回放/切换 PV 前的键盘确认")
    args = parser.parse_args()

    if not 1 <= args.speed <= 100:
        parser.error("--speed 必须在 1~100 之间")
    if not 1 <= args.gripper_speed <= 400:
        parser.error("--gripper-speed 必须在 1~400 之间")
    if not 50 <= args.send_interval_ms <= 500:
        parser.error("--send-interval-ms 必须在 50~500 之间")
    if args.tolerance <= 0 or args.gripper_tolerance < 0 or args.timeout <= 0 or args.dwell < 0:
        parser.error("到位阈值、超时和停留时间参数无效")

    beauty_print("Demo: Alicia-M 6+1 滑块姿态记录/回放", type="module")
    robot = None
    try:
        robot = alicia_m_sdk.create_robot(port=args.port)
        beauty_print(
            f"已连接 {robot.connected_port}，当前模式 {robot.control_mode.value.upper()}",
            type="success",
        )

        joint_limits = DEFAULT_JOINT_LIMITS_RAD
        urdf_path = Path(args.urdf).expanduser() if args.urdf else None
        if urdf_path is None and robot.resolved_model_version:
            try:
                from synriard import get_model_path

                urdf_path = Path(
                    get_model_path(
                        "Alicia_M",
                        version=robot.resolved_model_version,
                        variant="follower",
                        model_format="urdf",
                    )
                )
            except (ImportError, ValueError, AttributeError, OSError) as exc:
                beauty_print(f"未能定位自动匹配 URDF，将使用内置安全限位：{exc}", type="warning")
        if urdf_path is not None:
            joint_limits = parse_urdf_revolute_limits(urdf_path)
            beauty_print(f"关节限位来源: {urdf_path}", type="info")

        pose_path = Path(args.record or args.execute).expanduser()
        poses = None
        if args.execute:
            poses = load_poses(pose_path, joint_limits)
            beauty_print(f"TXT 校验通过，共 {len(poses)} 个姿态: {pose_path}", type="success")
        elif pose_path.exists() and count_pose_lines(pose_path):
            existing = load_poses(pose_path, joint_limits)
            beauty_print(f"将继续追加到现有文件（{len(existing)} 条）: {pose_path}", type="info")

        if robot.control_mode.value != "pv":
            beauty_print("本 Demo 使用纯 PV；切换时机械臂会短暂失能，请先扶稳并清空周围空间。", type="warning")
            if not args.yes:
                input("扶稳机械臂后按 Enter 切换到 PV；Ctrl+C 取消...")
            if not robot.switch_mode("pv"):
                raise RuntimeError("切换 PV 模式失败")
            beauty_print("已切换到 PV 模式", type="success")

        actual_mode = read_actual_control_mode(robot)
        if actual_mode != "pv":
            raise RuntimeError(f"固件实际模式不是 PV（检测结果：{actual_mode}），未发送运动命令")
        if robot.set_linear_interpolation_velocity(0.0) is False:
            raise RuntimeError("线性插补速度清零失败")
        beauty_print("已将 7 个电机的线性插补速度清零（协议精确零 0xFFFF）", type="info")

        if args.execute:
            beauty_print(
                f"即将按文件顺序运动；速度={args.speed:g}，到位阈值={args.tolerance:g} rad。",
                type="warning",
            )
            if not args.yes:
                input("确认机械臂、桌面、线缆和人员均安全后按 Enter 执行；Ctrl+C 取消...")
            execute_pose_sequence(
                robot,
                poses,
                speed=args.speed,
                gripper_speed=args.gripper_speed,
                tolerance_rad=args.tolerance,
                gripper_tolerance=args.gripper_tolerance,
                timeout=args.timeout,
                dwell=args.dwell,
                progress=ConsolePlaybackProgress(),
            )
            beauty_print(f"回放完成，共执行 {len(poses)} 个姿态", type="success")
        else:
            state = None
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                state = robot.get_robot_state("joint_gripper")
                if state and len(state.get("angles", ())) == 6 and state.get("gripper") is not None:
                    break
                time.sleep(0.05)
            if not state or len(state.get("angles", ())) != 6 or state.get("gripper") is None:
                raise RuntimeError("3 秒内未收到完整关节/夹爪反馈，未打开控制窗口")

            import tkinter as tk

            beauty_print("窗口已打开；滑块目标与实际反馈分栏显示，终端不会持续刷状态。", type="info")
            root = tk.Tk()
            PoseSliderWindow(
                root,
                robot,
                pose_path,
                state["angles"],
                state["gripper"],
                joint_limits,
                speed=args.speed,
                gripper_speed=args.gripper_speed,
                send_interval_ms=args.send_interval_ms,
            )
            root.mainloop()
    except KeyboardInterrupt:
        beauty_print("用户取消，正在安全断开", type="warning")
    except (PoseFileError, PosePlaybackError, RuntimeError, OSError) as exc:
        beauty_print(f"操作终止: {exc}", type="error")
        raise SystemExit(1) from exc
    finally:
        if robot is not None and robot.is_connected():
            robot.disconnect()
            beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
