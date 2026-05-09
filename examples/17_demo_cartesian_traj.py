#!/usr/bin/env python3
"""17_demo_cartesian_traj.py — 笛卡尔轨迹规划 + 批量 IK + 执行

演示内容：
1. 手动录制笛卡尔路点（读取当前末端位姿）
2. 自动生成笛卡尔路点
3. 从文件加载笛卡尔路点
4. 笛卡尔样条轨迹规划
5. 批量 IK 求解并可选执行（PV 回放）
"""

import argparse
from pathlib import Path
from contextlib import contextmanager

import numpy as np
import alicia_m_sdk
from _common import add_port_argument
from robocore.transform import matrix_to_quaternion  # stable in robocore 2.x
from alicia_m_sdk.utils.beauty_logger import beauty_print, beauty_print_array


@contextmanager
def temporary_ik_backend(backend):
    """Temporarily switch RoboCore backend for IK stage.

    :param backend: Target backend name
    :return: Context manager
    """
    if backend is None:
        yield
        return
    try:
        import robocore as rc
        prev = rc.get_backend()
        rc.set_backend(backend)
        beauty_print(f"IK backend switched: {prev} -> {backend}", type="info")
        try:
            yield
        finally:
            rc.set_backend(prev)
            beauty_print(f"IK backend restored: {prev}", type="info")
    except Exception as e:
        beauty_print(f"切换 IK backend 失败，继续使用当前后端: {e}", type="warning")
        yield


def select_mode():
    """Select waypoint input mode.

    :return: Selected mode id ('1'/'2'/'3')
    """
    beauty_print("请选择路点来源：", type="module")
    print("  1) 手动录点（当前末端位姿）")
    print("  2) 自动生成")
    print("  3) 从文件加载")
    while True:
        mode = input("输入模式编号 [1/2/3]: ").strip()
        if mode in {"1", "2", "3"}:
            return mode
        beauty_print("输入无效，请重新输入。", type="warning")


def manual_record_cartesian_waypoints(robot):
    """Record Cartesian waypoints from current end pose.

    :param robot: Connected robot instance
    :return: Waypoints array [N, 7], or None
    """
    beauty_print("手动录点模式：移动机械臂后按 Enter 采样，输入 q 结束。", type="module")
    waypoints = []
    idx = 1
    while True:
        cmd = input(f"[路点 {idx}] Enter=采样, q=结束: ").strip().lower()
        if cmd == "q":
            break
        pose_info = robot.get_pose()
        if pose_info is None:
            beauty_print("读取末端位姿失败，跳过本次采样。", type="warning")
            continue
        pos = np.asarray(pose_info["position"], dtype=np.float64)
        quat = np.asarray(pose_info["quaternion_xyzw"], dtype=np.float64)
        wp = np.concatenate([pos, quat], axis=0)
        waypoints.append(wp)
        beauty_print(
            f"  已记录路点 {idx}: pos={beauty_print_array(pos, precision=4)}, quat={beauty_print_array(quat, precision=4)}"
        )
        idx += 1

    if len(waypoints) < 2:
        beauty_print("路点不足（至少 2 个），取消。", type="warning")
        return None
    return np.asarray(waypoints, dtype=np.float64)


def manual_record_cartesian_waypoints_with_torque_off(robot):
    """Record Cartesian waypoints with torque disabled for drag-teach.

    :param robot: Connected robot instance
    :return: Waypoints array [N, 7], or None
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

        return manual_record_cartesian_waypoints(robot)
    finally:
        if torque_disabled:
            if robot.torque_control("on"):
                beauty_print("已恢复力矩。", type="info")
            else:
                beauty_print("恢复力矩失败，请手动检查。", type="warning")
        if switched_to_mit:
            robot.switch_mode("pv")
            beauty_print("已切回 PV 模式。", type="info")


def _random_joint_waypoint(robot_model, seed, scale):
    """Generate one random joint sample.

    :param robot_model: Robot model instance
    :param seed: Random seed
    :param scale: Random scale
    :return: Joint array [6]
    """
    if hasattr(robot_model, "random_q"):
        q = robot_model.random_q(seed=seed, scale=scale)
        return np.asarray(q, dtype=np.float64)
    rng = np.random.default_rng(seed)
    return rng.uniform(
        low=np.deg2rad([-120, -120, -120, -170, -120, -170]),
        high=np.deg2rad([120, 120, 120, 170, 120, 170]),
        size=(6,),
    ).astype(np.float64)


def auto_generate_cartesian_waypoints(robot, robot_model, args):
    """Generate random Cartesian waypoints from random joints + FK.

    :param robot: Connected robot instance
    :param robot_model: Robot model instance
    :param args: CLI arguments
    :return: Waypoints array [N, 7]
    """
    beauty_print("自动生成模式", type="module")
    num_waypoints = max(2, args.num_waypoints)
    beauty_print(f"生成路点数: {num_waypoints}", type="info")

    waypoints = []
    seed = args.seed

    if args.use_current_pose:
        pose = robot.get_pose()
        if pose is not None:
            waypoints.append(
                np.concatenate(
                    [
                        np.asarray(pose["position"], dtype=np.float64),
                        np.asarray(pose["quaternion_xyzw"], dtype=np.float64),
                    ]
                )
            )
            beauty_print("首个路点使用当前末端位姿。", type="info")

    while len(waypoints) < num_waypoints:
        q = _random_joint_waypoint(robot_model, seed, args.workspace_scale)
        fk = alicia_m_sdk.forward_kinematics(robot_model, q, return_end=True)
        pos = np.asarray(fk[:3, 3], dtype=np.float64)
        rot = np.asarray(fk[:3, :3], dtype=np.float64)
        quat = np.asarray(matrix_to_quaternion(rot), dtype=np.float64)
        waypoints.append(np.concatenate([pos, quat], axis=0))
        if seed is not None:
            seed += 1

    return np.asarray(waypoints, dtype=np.float64)


def load_cartesian_waypoints_from_file(path):
    """Load Cartesian waypoints from text/npy file.

    :param path: Waypoint file path
    :return: Waypoints array [N, 7] or [N, 6], or None
    """
    file_path = Path(path).expanduser()
    if not file_path.exists():
        beauty_print(f"文件不存在: {file_path}", type="error")
        return None

    if file_path.suffix.lower() == ".npy":
        data = np.load(file_path)
    else:
        data = np.loadtxt(file_path, delimiter=",", ndmin=2)

    data = np.asarray(data, dtype=np.float64)
    if data.ndim != 2 or data.shape[0] < 2 or data.shape[1] not in (6, 7):
        beauty_print("文件格式错误：期望 [N,6] 或 [N,7] 且 N>=2。", type="error")
        return None
    return data


def solve_ik_for_poses(robot, poses, timestamps, args):
    """Solve IK one-by-one with previous solution warm start.

    :param robot: Connected robot instance
    :param poses: Pose sequence [N, 7]
    :param timestamps: Timestamp sequence [N]
    :param args: CLI arguments
    :return: Solved timestamps [M], joint trajectory [M, 6], success ratio
    """
    with temporary_ik_backend(args.ik_backend):
        batch = robot.solve_ik_for_trajectory(
            target_poses=poses.tolist(),
            q_init=robot.get_robot_state("joint") if args.init_strategy == "current" else None,
            method=args.method,
            max_iterations=args.max_iters,
            tolerance=args.tol,
            num_initial_guesses=args.num_inits,
            initial_guess_strategy=args.init_strategy if args.init_strategy != "current" else "random",
        )
    results = batch.get("results", [])
    if len(results) == 0:
        return None, None, 0.0

    solved_ts = []
    solved_joints = []
    for i, result in enumerate(results):
        if not result.get("success", False):
            beauty_print(f"  IK 失败: index={i}", type="warning")
            continue
        solved_ts.append(float(timestamps[i]))
        solved_joints.append(np.asarray(result["q"], dtype=np.float64))

    if len(solved_joints) == 0:
        return None, None, 0.0

    ratio = len(solved_joints) / len(poses)
    return np.asarray(solved_ts, dtype=np.float64), np.asarray(solved_joints, dtype=np.float64), ratio


def execute_joint_sequence(robot, timestamps, joints):
    """Execute joint sequence with PV trajectory executor.

    :param robot: Connected robot instance
    :param timestamps: Timestamps [N]
    :param joints: Joint trajectory [N, 6]
    :return: True if execution finished
    """
    state = robot.get_robot_state("joint_gripper")
    gripper = float(state["gripper"]) if state and state.get("gripper") is not None else 0.0
    gripper_col = np.full((joints.shape[0], 1), gripper, dtype=np.float64)
    pos_full = np.hstack([joints, gripper_col])

    velocities = np.zeros_like(pos_full)
    if len(timestamps) > 1:
        dt = np.diff(timestamps)
        dt = np.where(dt <= 1e-6, 1e-6, dt)
        velocities[1:, :] = np.diff(pos_full, axis=0) / dt[:, None]

    return robot._traj_executor.execute_pv(
        np.asarray(timestamps, dtype=np.float64),
        pos_full,
        velocities,
    )


def plot_trajectory(traj, waypoints, solved_ts=None, joints=None):
    """Plot Cartesian and IK trajectory for quick inspection.

    :param traj: Cartesian trajectory dictionary
    :param waypoints: Cartesian waypoint array
    :param solved_ts: Optional solved IK timestamps [M]
    :param joints: Optional solved joint trajectory [M, 6]
    :return: None
    """
    try:
        import matplotlib.pyplot as plt

        ts = np.asarray(traj.get("timestamps", []), dtype=np.float64)
        positions = np.asarray(traj.get("positions", []), dtype=np.float64)
        if positions.ndim != 2 or positions.shape[1] < 3 or len(ts) == 0:
            beauty_print("轨迹数据不足，跳过可视化。", type="warning")
            return

        fig = plt.figure("Cartesian Trajectory", figsize=(10, 8))
        ax_xyz = fig.add_subplot(2, 1, 1)
        ax_xyz.plot(ts, positions[:, 0], label="x")
        ax_xyz.plot(ts, positions[:, 1], label="y")
        ax_xyz.plot(ts, positions[:, 2], label="z")
        ax_xyz.set_title("Cartesian Position")
        ax_xyz.set_xlabel("time (s)")
        ax_xyz.set_ylabel("position (m)")
        ax_xyz.grid(True, alpha=0.3)
        ax_xyz.legend()

        wp = np.asarray(waypoints, dtype=np.float64)
        if wp.ndim == 2 and wp.shape[1] >= 3 and len(wp) >= 2:
            wp_t = np.linspace(ts[0], ts[-1], len(wp))
            ax_xyz.scatter(wp_t, wp[:, 0], marker="o", s=20, label="x-waypoints")
            ax_xyz.scatter(wp_t, wp[:, 1], marker="o", s=20, label="y-waypoints")
            ax_xyz.scatter(wp_t, wp[:, 2], marker="o", s=20, label="z-waypoints")

        ax_path = fig.add_subplot(2, 1, 2)
        ax_path.plot(positions[:, 0], positions[:, 1], label="trajectory xy")
        if wp.ndim == 2 and wp.shape[1] >= 3:
            ax_path.scatter(wp[:, 0], wp[:, 1], c="r", marker="x", s=30, label="waypoints xy")
        ax_path.set_title("Top View (X-Y)")
        ax_path.set_xlabel("x (m)")
        ax_path.set_ylabel("y (m)")
        ax_path.grid(True, alpha=0.3)
        ax_path.legend()
        fig.tight_layout()

        if solved_ts is not None and joints is not None:
            ts_ik = np.asarray(solved_ts, dtype=np.float64)
            q_ik = np.asarray(joints, dtype=np.float64)
            if q_ik.ndim == 2 and q_ik.shape[1] >= 6 and len(ts_ik) == len(q_ik):
                fig_q = plt.figure("IK Joint Trajectory", figsize=(10, 6))
                ax_q = fig_q.add_subplot(1, 1, 1)
                for i in range(6):
                    ax_q.plot(ts_ik, np.rad2deg(q_ik[:, i]), label=f"J{i+1}")
                ax_q.set_title("IK Joint Angles")
                ax_q.set_xlabel("time (s)")
                ax_q.set_ylabel("angle (deg)")
                ax_q.grid(True, alpha=0.3)
                ax_q.legend(ncol=3)
                fig_q.tight_layout()

        plt.show(block=False)
        plt.pause(0.1)
    except ImportError:
        beauty_print("matplotlib not installed, skipping visualization.", type="warning")


def main():
    parser = argparse.ArgumentParser(description="Alicia-M 笛卡尔轨迹规划 + 批量 IK + 执行")
    add_port_argument(parser)
    parser.add_argument("--duration", type=float, default=6.0, help="轨迹总时长（秒）")
    parser.add_argument("--frequency", type=float, default=500.0, help="采样频率（Hz）")
    parser.add_argument("--num-waypoints", type=int, default=4, help="自动生成路点数")
    parser.add_argument("--workspace-scale", type=float, default=0.6, help="随机工作空间缩放")
    parser.add_argument("--use-current-pose", action="store_true", help="自动模式首个路点使用当前位姿")
    parser.add_argument("--seed", type=int, default=666, help="随机种子")
    parser.add_argument("--waypoint-file", type=str, default="", help="文件模式路点文件路径（csv/txt/npy）")
    parser.add_argument("--method", type=str, default="dls", help="IK 方法")
    parser.add_argument("--ik-backend", type=str, default="cpp", choices=["cpp", "numpy", "torch"], help="IK 计算后端")
    parser.add_argument("--max-iters", type=int, default=200, help="IK 最大迭代次数")
    parser.add_argument("--tol", type=float, default=1e-3, help="IK 收敛阈值")
    parser.add_argument("--num-inits", type=int, default=4, help="IK 多起点数")
    parser.add_argument(
        "--init-strategy",
        type=str,
        default="current",
        choices=["current", "zero", "random", "sobol", "latin", "center", "uniform"],
        help="IK 初值策略",
    )
    parser.add_argument("--execute", action="store_true", help="规划后直接执行")
    parser.add_argument("--min-success-rate", type=float, default=0.8, help="执行前最小 IK 成功率")
    parser.add_argument("--plot", action="store_true", help="显示规划轨迹可视化")
    args = parser.parse_args()

    beauty_print("Demo: 笛卡尔轨迹规划 + 批量 IK + 执行", type="module")
    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print(f"机器人连接成功（当前 {robot.control_mode.value.upper()} 模式）", type="success")

    if robot.control_mode.value != "pv":
        beauty_print("本示例需要 PV 模式，切换过程中机械臂将短暂失能", type="warning")
        input("按 Enter 切换到 PV 模式...")
        robot.switch_mode("pv")
        beauty_print("已切换到 PV 模式", type="success")

    try:
        mode = select_mode()
        if mode == "1":
            waypoints = manual_record_cartesian_waypoints_with_torque_off(robot)
        elif mode == "2":
            waypoints = auto_generate_cartesian_waypoints(robot, robot.robot_model, args)
        else:
            file_path = args.waypoint_file or input("请输入路点文件路径: ").strip()
            waypoints = load_cartesian_waypoints_from_file(file_path)

        if waypoints is None:
            return

        beauty_print("笛卡尔路点:", type="module")
        for i, pose in enumerate(waypoints):
            if pose.shape[0] == 7:
                msg = (
                    f"  [{i:02d}] pos={beauty_print_array(pose[:3], precision=4)} "
                    f"quat={beauty_print_array(pose[3:], precision=4)}"
                )
            else:
                msg = (
                    f"  [{i:02d}] pos={beauty_print_array(pose[:3], precision=4)} "
                    f"euler={beauty_print_array(np.rad2deg(pose[3:]), precision=2)} deg"
                )
            print(msg)

        beauty_print("开始笛卡尔轨迹规划...", type="module")
        traj = robot.plan_cartesian_trajectory(
            waypoints=waypoints.tolist(),
            duration=args.duration,
            frequency=args.frequency,
        )
        if not traj.get("success", False):
            beauty_print("笛卡尔轨迹规划失败。", type="error")
            return

        beauty_print("轨迹规划成功", type="success")
        beauty_print(f"  点数: {traj['num_points']}  时长: {traj['duration']:.3f}s", type="info")

        poses = np.asarray(traj.get("poses", []), dtype=np.float64)
        if poses.ndim != 2 or poses.shape[1] < 7:
            beauty_print("规划结果缺少 poses（[N,7]），无法继续 IK。", type="error")
            return

        beauty_print("开始批量 IK 求解...", type="module")
        solved_ts, joints, success_rate = solve_ik_for_poses(
            robot,
            poses[:, :7],
            np.asarray(traj["timestamps"], dtype=np.float64),
            args,
        )
        if joints is None or solved_ts is None:
            beauty_print("IK 全部失败。", type="error")
            return
        beauty_print(f"IK 成功率: {success_rate * 100:.1f}%", type="info")
        if success_rate < args.min_success_rate:
            beauty_print("IK 成功率低于阈值，取消执行。", type="warning")
            return

        if args.plot:
            beauty_print("绘制轨迹可视化...", type="module")
            plot_trajectory(traj, waypoints, solved_ts=solved_ts, joints=joints)

        if not args.execute:
            input("\n按 Enter 执行轨迹，Ctrl+C 取消...")

        ok = execute_joint_sequence(robot, solved_ts, joints)
        beauty_print("轨迹执行完成" if ok else "轨迹执行中断", type="success" if ok else "warning")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
