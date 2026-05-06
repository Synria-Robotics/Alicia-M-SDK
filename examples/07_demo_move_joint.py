"""07_demo_move_joint.py — 关节控制 (PV)

演示 PV 模式关节空间运动控制：回零 -> 目标位置 -> 回零。
"""

import argparse
import time
import threading
from typing import List, Optional

import numpy as np
import alicia_m_sdk
from demo_common import add_port_argument
from alicia_m_sdk.utils.beauty_logger import beauty_print


# 预设安全关节位置 (度) 
POSITION = [0, -130.0, -90.0, 80.0, 0.0, 0.0]
# POSITION = [0, 0, 0, 0, 0, 20]


def _matplotlib_show(block):
    """Flush GUI event loop for matplotlib.

    :param block,bool: If True, wait until user closes all figures
    """
    import matplotlib.pyplot as plt

    plt.tight_layout()
    plt.show(block=block)
    if not block:
        plt.pause(0.25)


def _collect_joint_feedback(robot, stop_event, out_samples, base_t):
    """Collect real-time joint position and velocity from state cache.

    :param robot: Connected robot instance
    :param stop_event: Thread stop event
    :param out_samples: Append target list for (t, q, qd)
    :param base_t: Absolute start time from perf counter
    :return: None
    """
    while not stop_event.is_set():
        joints = robot.get_robot_state("joint")
        vels = robot.get_robot_state("velocity")
        if joints is not None:
            t_rel = time.perf_counter() - base_t
            q_raw = np.asarray(joints, dtype=np.float64).reshape(-1)
            if q_raw.size < 6:
                time.sleep(0.01)
                continue
            q = q_raw[:6].copy()

            if vels is None:
                qd = np.full((6,), np.nan, dtype=np.float64)
            else:
                qd_raw = np.asarray(vels, dtype=np.float64).reshape(-1)
                if qd_raw.size < 6:
                    qd = np.full((6,), np.nan, dtype=np.float64)
                else:
                    qd = qd_raw[:6].copy()
            out_samples.append((t_rel, q, qd))
        time.sleep(0.01)


def _move_and_track(robot, target_joints, speed, label, samples, base_t):
    """Execute one PV point motion and track states until completion.

    :param robot: Connected robot instance
    :param target_joints: Joint target in deg
    :param speed: Motion speed argument
    :param label: Segment label for logs
    :param samples: Shared samples list
    :param base_t: Absolute start time from perf counter
    :return: bool
    """
    beauty_print(f"{label}...", type="info")
    stop_event = threading.Event()
    worker = threading.Thread(
        target=_collect_joint_feedback,
        args=(robot, stop_event, samples, base_t),
        daemon=True,
    )
    worker.start()
    ok = False
    try:
        ok = robot.set_robot_state(
            target_joints=target_joints,
            joint_format="deg",
            speed=speed,
            wait_for_completion=True,
        )
    finally:
        stop_event.set()
        worker.join(timeout=0.5)
    return ok


def _plot_joint_feedback(samples, *, block=True):
    """Plot real-time joint position and velocity feedback.

    :param samples: List[(t, q, qd)] captured during motion
    :param block: Wait until figure window is closed when True
    :return: None
    """
    if len(samples) < 2:
        beauty_print("采样点不足，跳过实时曲线绘制。", type="warning")
        return

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        beauty_print("matplotlib 不可用，跳过实时曲线绘制。", type="warning")
        return

    t = np.asarray([row[0] for row in samples], dtype=np.float64)
    q = np.stack([row[1] for row in samples], axis=0)
    qd = np.stack([row[2] for row in samples], axis=0)

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9.0, 7.0))
    for j in range(6):
        axes[0].plot(t, q[:, j], lw=1.0, label=f"J{j}")
    axes[0].set_ylabel("q (rad)")
    axes[0].set_title("Real-time joint feedback: position / velocity")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right", ncol=3, fontsize=8)

    valid_vel = np.all(np.isfinite(qd), axis=1)
    if np.any(valid_vel):
        for j in range(6):
            axes[1].plot(t[valid_vel], qd[valid_vel, j], lw=1.0, label=f"J{j}")
        axes[1].legend(loc="upper right", ncol=3, fontsize=8)
    else:
        axes[1].text(
            0.5,
            0.5,
            "(no valid velocity samples)",
            ha="center",
            va="center",
            transform=axes[1].transAxes,
            fontsize=10,
            alpha=0.55,
        )
    axes[1].set_ylabel("qd (rad/s)")
    axes[1].set_xlabel("t (s)")
    axes[1].grid(True, alpha=0.3)

    _matplotlib_show(block)


def main(args):
    beauty_print("Demo: 关节控制 (PV)", type="module")

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
        samples = []
        t0 = time.perf_counter()

        # --- 回零位 ---
        if not _move_and_track(
            robot,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            args.speed,
            "回零位",
            samples,
            t0,
        ):
            beauty_print("回零位失败，提前结束。", type="error")
            return
        beauty_print("已到达零位", type="success")
        time.sleep(1.0)

        # --- 移动到目标位置 ---
        if not _move_and_track(robot, POSITION, args.speed, f"移动到目标位置: {POSITION} (deg)", samples, t0):
            beauty_print("移动到目标位置失败，提前结束。", type="error")
            return
        beauty_print("已到达目标位置", type="success")
        time.sleep(1.0)

        # --- 回零位 ---
        if not _move_and_track(
            robot,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            args.speed,
            "回零位",
            samples,
            t0,
        ):
            beauty_print("回零位失败，提前结束。", type="error")
            return
        beauty_print("已到达零位", type="success")

        if args.plot:
            beauty_print("绘制实时位置/速度曲线...", type="module")
            _plot_joint_feedback(samples, block=not args.plot_noblock)
            if args.plot_noblock:
                beauty_print(
                    "非阻塞绘图：请在终端按 Enter 后再断开连接（否则窗口会随进程结束立刻关闭）。",
                    type="warning",
                )
                input()

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="关节控制示例 (PV)")
    parser.add_argument(
        "--speed", type=float, default=15,
        help="运动速度 (默认: 15, 范围: 0-400)"
    )
    parser.add_argument("--plot", action="store_true", help="显示实时关节位置/速度曲线")
    parser.add_argument(
        "--plot-noblock",
        action="store_true",
        help="图表非阻塞显示（脚本结束前须按 Enter，便于在无 GUI 阻塞环境下查看）",
    )
    add_port_argument(parser)
    args = parser.parse_args()
    
    main(args)
