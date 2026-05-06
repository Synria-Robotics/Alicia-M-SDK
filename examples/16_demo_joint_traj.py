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
from demo_common import add_port_argument
from alicia_m_sdk.execution.trajectory_executor import (
    execute_joint_trajectory,
    load_waypoints_from_file,
    resolve_default_traj_save_dir,
    save_trajectory_csv,
)
from alicia_m_sdk.utils.trajectory_plot import (
    plot_joint_tracking,
    plot_joint_velocity_tracking,
    plot_trajectory,
)
from alicia_m_sdk.utils.beauty_logger import beauty_print, beauty_print_array

# Console limits when loading dense CSV (e.g. saved trajectory as waypoints)
_MAX_WAYPOINT_PRINT = 24


def select_mode():
    """Select waypoint input mode.

    :return: Selected mode id ('1'/'2'/'3')
    """
    beauty_print("请选择路点来源：", type="module")
    print("  1) 手动录点")
    print("  2) 自动生成")
    print("  3) 从文件加载")
    while True:
        mode = input("输入模式编号 [1/2/3]: ").strip()
        if mode in {"1", "2", "3"}:
            return mode
        beauty_print("输入无效，请重新输入。", type="warning")


def manual_record_waypoints(robot):
    """Record joint waypoints from current robot state.

    :param robot: Connected robot instance
    :return: Waypoints array [N, 6], or None
    """
    beauty_print("手动录点模式：移动机械臂后按 Enter 采样，输入 q 结束。", type="module")
    waypoints = []
    idx = 1
    while True:
        cmd = input(f"[路点 {idx}] Enter=采样, q=结束: ").strip().lower()
        if cmd == "q":
            break
        joints = robot.get_robot_state("joint")
        if joints is None:
            beauty_print("读取关节状态失败，跳过本次采样。", type="warning")
            continue
        waypoints.append(np.asarray(joints, dtype=np.float64))
        beauty_print(f"  已记录路点 {idx}: {beauty_print_array(np.rad2deg(joints), precision=2)} deg")
        idx += 1

    if len(waypoints) < 2:
        beauty_print("路点不足（至少 2 个），取消。", type="warning")
        return None
    return np.asarray(waypoints, dtype=np.float64)


def manual_record_waypoints_with_torque_off(robot):
    """Record waypoints with torque disabled for drag-teach.

    :param robot: Connected robot instance
    :return: Waypoints array [N, 6], or None
    """
    switched_to_mit = False
    torque_disabled = False
    try:
        if robot.control_mode.value != "mit":
            beauty_print("手动录点将临时切到 MIT 并卸力，结束后恢复 PV。", type="warning")
            input("按 Enter 开始切换...")
            robot.switch_mode("mit")
            switched_to_mit = True
            beauty_print("已切换到 MIT 模式。", type="success")

        if not robot.torque_control("off"):
            beauty_print("卸力失败，无法进入拖动示教。", type="error")
            return None
        torque_disabled = True
        beauty_print("已卸力（新 torque off 实现），可手动拖动采点。", type="success")

        return manual_record_waypoints(robot)
    finally:
        if torque_disabled:
            if robot.torque_control("on"):
                beauty_print("已恢复力矩。", type="info")
            else:
                beauty_print("恢复力矩失败，请手动检查。", type="warning")
        if switched_to_mit:
            robot.switch_mode("pv")
            beauty_print("已切回 PV 模式。", type="info")


def auto_generate_waypoints(robot, robot_model, args):
    """Generate random joint waypoints.

    :param robot: Connected robot instance
    :param robot_model: Robot model instance
    :param args: CLI arguments
    :return: Waypoints array [N, 6]
    """
    beauty_print("自动生成模式", type="module")
    num_waypoints = max(2, args.num_waypoints)
    beauty_print(f"生成路点数: {num_waypoints}", type="info")

    waypoints = []
    if args.use_current_joints:
        current = robot.get_robot_state("joint")
        if current is not None:
            waypoints.append(np.asarray(current, dtype=np.float64))
            beauty_print("首个路点使用当前关节角。", type="info")

    seed = args.seed
    while len(waypoints) < num_waypoints:
        if hasattr(robot_model, "random_q"):
            q = robot_model.random_q(seed=seed, scale=args.joint_scale)
            q = np.asarray(q, dtype=np.float64)
        else:
            rng = np.random.default_rng(seed)
            q = rng.uniform(
                low=np.deg2rad([-120, -120, -120, -170, -120, -170]),
                high=np.deg2rad([120, 120, 120, 170, 120, 170]),
                size=(6,),
            ).astype(np.float64)
        waypoints.append(q)
        if seed is not None:
            seed += 1

    return np.asarray(waypoints, dtype=np.float64)


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
        mode = select_mode()
        wp_meta = {}
        if mode == "1":
            waypoints = manual_record_waypoints_with_torque_off(robot)
        elif mode == "2":
            waypoints = auto_generate_waypoints(robot, robot.robot_model, args)
        else:
            file_path = args.waypoint_file or input("请输入路点文件路径: ").strip()
            loaded = load_waypoints_from_file(file_path)
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

        save_dir = args.traj_save_dir if args.traj_save_dir else resolve_default_traj_save_dir(__file__)
        traj_csv = save_trajectory_csv(traj, output_dir=save_dir)
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
        ok, tracking = execute_joint_trajectory(robot, traj, args.speed, track_hz=track_hz)
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
    parser = argparse.ArgumentParser(description="Alicia-M 关节空间轨迹规划与执行")
    add_port_argument(parser)
    parser.add_argument("--planner", type=str, default="b_spline", choices=["b_spline", "multi_segment"])
    parser.add_argument("--duration", type=float, default=10.0, help="轨迹总时长（秒）")
    parser.add_argument("--frequency", type=float, default=500.0, help="采样频率（Hz）")
    parser.add_argument("--order", type=int, default=5, help="B-Spline 阶数")
    parser.add_argument("--segment-type", type=str, default="quintic", choices=["cubic", "quintic"])
    parser.add_argument("--num-waypoints", type=int, default=5, help="自动生成路点数")
    parser.add_argument("--joint-scale", type=float, default=0.6, help="random_q 缩放系数")
    parser.add_argument("--use-current-joints", action="store_true", help="自动模式首个路点使用当前关节角")
    parser.add_argument("--seed", type=int, default=666, help="随机种子")
    parser.add_argument("--waypoint-file", type=str, default="", help="文件模式路点文件路径（csv/txt/npy）")
    parser.add_argument("--execute", action="store_true", help="规划后直接执行")
    parser.add_argument("--speed", type=float, default=100.0, help="执行信息显示速度参数")
    parser.add_argument("--plot", action="store_true", help="显示规划轨迹可视化")
    parser.add_argument(
        "--plot-noblock",
        action="store_true",
        help="图表非阻塞显示（脚本结束前须按 Enter，便于在无 GUI 阻塞环境下查看）",
    )
    parser.add_argument(
        "--track-joints",
        action="store_true",
        help="执行时记录关节反馈，结束后绘制与目标轨迹同一时间轴的角度/速度对比图",
    )
    parser.add_argument(
        "--track-hz",
        type=float,
        default=200.0,
        help="跟踪曲线目标显示频率 Hz（对每帧 PV 后的反馈下采样；设大如 5000 可接近全帧）",
    )
    parser.add_argument(
        "--traj-save-dir",
        type=str,
        default="",
        help="轨迹 CSV 保存目录（默认自动定位 Alicia-M-SDK/logs）",
    )
    args = parser.parse_args()
    
    main(args)
