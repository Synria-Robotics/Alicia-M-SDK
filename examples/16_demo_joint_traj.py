#!/usr/bin/env python3
"""16_demo_joint_traj.py — 关节空间轨迹规划与执行

演示内容：
1. 手动录点（通过 Enter 采样当前关节角）
2. 自动生成关节路点
3. 从文件加载关节路点
4. 关节轨迹规划（B-Spline / Multi-Segment）
5. 可选执行轨迹到机器人（PV 回放）
6. 规划成功后保存轨迹为 traj_YYYYMMDD_HHMMSS.csv
7. ``--plot``：三子图（位置/速度/加速度），每子图六关节同轴；位置子图叠加输入路点（子采样）
8. ``--track-joints``：在每帧 PV 下发后从设备状态缓存读取 6 轴关节反馈，时间戳与规划的
   ``timestamps`` 一致；``--track-hz`` 仅用于相对轨迹采样率的下采样以降低点数（非额外线程）
   结束前会提示按 Enter，避免进程退出瞬间窗口被关掉。
"""

import argparse
import time

import numpy as np
import alicia_m_sdk
from _demo_helpers import (
    add_port_argument,
    beauty_print,
    beauty_print_array,
    plot_joint_tracking,
    plot_joint_velocity_tracking,
    plot_trajectory,
    select_waypoint_mode,
)

# Console limits when loading dense CSV (e.g. saved trajectory as waypoints)
_MAX_WAYPOINT_PRINT = 24


def main(args):
    beauty_print("Demo: 关节空间轨迹规划与执行", type="module")
    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print(f"机器人连接成功（当前 {robot.control_mode.value.upper()} 模式）", type="success")

    if robot.control_mode.value != "pv":
        beauty_print("本示例需要 PV 模式，切换过程中机械臂将短暂失能", type="warning")
        input("按 Enter 切换到 PV 模式...")
        robot.switch_mode("pv")
        beauty_print("已切换到 PV 模式", type="success")

    try:
        mode = select_waypoint_mode()
        wp_meta = {}
        if mode == "1":
            waypoints = robot.manual_record_waypoints_with_torque_off()
        elif mode == "2":
            waypoints = robot.auto_generate_waypoints(
                num_waypoints=args.num_waypoints,
                joint_scale=args.joint_scale,
                use_current_joints=args.use_current_joints,
                seed=args.seed,
            )
        else:
            loaded = robot.load_waypoints(args.waypoint_file)
            if loaded is None:
                return
            waypoints, wp_meta = loaded

        if waypoints is None:
            return

        beauty_print("关节路点（deg）:", type="module")
        n_wp = len(waypoints)
        if n_wp <= _MAX_WAYPOINT_PRINT:
            indices = range(n_wp)
        else:
            head = _MAX_WAYPOINT_PRINT // 2
            tail = _MAX_WAYPOINT_PRINT - head
            indices = list(range(head)) + list(range(n_wp - tail, n_wp))
            beauty_print(
                f"  （共 {n_wp} 个路点，仅打印前 {head} 与后 {tail} 个）",
                type="info",
            )
        for i in indices:
            q = waypoints[i]
            print(f"  [{i:04d}] {beauty_print_array(np.rad2deg(q), precision=2)}")

        beauty_print("开始轨迹规划...", type="module")
        planner_kwargs = {"duration": args.duration, "frequency": args.frequency}
        if args.planner == "b_spline":
            planner_kwargs["order"] = args.order
        else:
            planner_kwargs["segment_type"] = args.segment_type

        traj = robot.plan_joint_trajectory(
            waypoints=waypoints.tolist(),
            planner_type=args.planner,
            **planner_kwargs,
        )
        if not traj.get("success", False):
            beauty_print("轨迹规划失败。", type="error")
            return

        beauty_print("轨迹规划成功", type="success")
        beauty_print(f"  点数: {traj['num_points']}  时长: {traj['duration']:.3f}s", type="info")
        beauty_print(f"  起点(deg): {beauty_print_array(np.rad2deg(traj['positions'][0]), precision=2)}", type="info")
        beauty_print(f"  终点(deg): {beauty_print_array(np.rad2deg(traj['positions'][-1]), precision=2)}", type="info")

        save_dir = args.traj_save_dir if args.traj_save_dir else robot.default_trajectory_save_dir(__file__)
        traj_csv = robot.save_joint_trajectory_csv(traj, output_dir=save_dir)
        beauty_print(f"轨迹已保存: {traj_csv}", type="success")

        plot_block = not args.plot_noblock
        if args.plot:
            beauty_print("绘制轨迹可视化...", type="module")
            plot_trajectory(
                traj,
                waypoints,
                waypoints_time_s=wp_meta.get("waypoints_time_s"),
                block=plot_block,
            )

        if not args.execute:
            input("\n按 Enter 执行轨迹，Ctrl+C 取消...")

        track_hz = args.track_hz if args.track_joints else None
        ok, tracking = robot.execute_planned_joint_trajectory(traj, args.speed, track_hz=track_hz)
        beauty_print("轨迹执行完成" if ok else "轨迹执行中断", type="success" if ok else "warning")
        if tracking is not None:
            beauty_print("绘制目标 vs 实测关节角（时间轴与 PV 回放一致）...", type="module")
            plot_joint_tracking(traj, tracking, block=plot_block)
            beauty_print("绘制目标速度 vs 实测速度（同时间轴）...", type="module")
            plot_joint_velocity_tracking(traj, tracking, block=plot_block)
        if args.plot_noblock and (args.plot or tracking is not None):
            beauty_print(
                "非阻塞绘图：请在终端按 Enter 后再断开连接（否则窗口会随进程结束立刻关闭）。",
                type="warning",
            )
            input()
        time.sleep(0.2)

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plan and execute Alicia-M joint-space trajectories.")
    add_port_argument(parser)
    parser.add_argument("--planner", type=str, default="b_spline", choices=["b_spline", "multi_segment"])
    parser.add_argument("--duration", type=float, default=10.0, help="Total trajectory duration in seconds.")
    parser.add_argument("--frequency", type=float, default=500.0, help="Trajectory sampling frequency in Hz.")
    parser.add_argument("--order", type=int, default=5, help="B-Spline order.")
    parser.add_argument("--segment-type", type=str, default="quintic", choices=["cubic", "quintic"])
    parser.add_argument("--num-waypoints", type=int, default=5, help="Number of automatically generated waypoints.")
    parser.add_argument("--joint-scale", type=float, default=0.6, help="Scale factor for random_q.")
    parser.add_argument("--use-current-joints", action="store_true", help="Use current joint angles as the first waypoint in automatic mode.")
    parser.add_argument("--seed", type=int, default=666, help="Random seed.")
    parser.add_argument("--waypoint-file", type=str, default="", help="Waypoint file path for file mode: csv, txt, or npy.")
    parser.add_argument("--execute", action="store_true", help="Execute immediately after planning.")
    parser.add_argument("--speed", type=float, default=100.0, help="Speed parameter shown and used during execution.")
    parser.add_argument("--plot", action="store_true", help="Show planned trajectory plot.")
    parser.add_argument(
        "--plot-noblock",
        action="store_true",
        help="Show plots without blocking; press Enter before exit to keep windows open.",
    )
    parser.add_argument(
        "--track-joints",
        action="store_true",
        help="Record joint feedback during execution and plot target-vs-measured angle and velocity.",
    )
    parser.add_argument(
        "--track-hz",
        type=float,
        default=200.0,
        help="Display frequency for tracking curves in Hz; high values approach full-frame feedback.",
    )
    parser.add_argument(
        "--traj-save-dir",
        type=str,
        default="",
        help="Directory for saved trajectory CSV files; default is Alicia-M-SDK/logs.",
    )
    args = parser.parse_args()
    
    main(args)
