"""Trajectory plotting helpers based on matplotlib."""

import numpy as np

from ..protocol.constants import NUM_JOINTS
from .beauty_logger import beauty_print

_MAX_WAYPOINT_SCATTER = 400


def _matplotlib_show(block):
    """Flush GUI event loop for matplotlib.

    :param block: If True, wait until user closes all figures
    :return: None
    """
    import matplotlib.pyplot as plt

    plt.tight_layout()
    plt.show(block=block)
    if not block:
        plt.pause(0.25)


def waypoint_times_for_plot(traj_timestamps, n_waypoints, waypoints_time_s=None):
    """Map waypoint samples to the replanned trajectory time axis.

    :param traj_timestamps: Monotonic time array of the planned trajectory
    :param n_waypoints: Number of waypoint rows
    :param waypoints_time_s: Optional original sample times
    :return: Time array for each waypoint row
    """
    t = np.asarray(traj_timestamps, dtype=np.float64)
    if n_waypoints < 2:
        return np.array([t[0]], dtype=np.float64)
    if waypoints_time_s is not None and len(waypoints_time_s) == n_waypoints:
        tw = np.asarray(waypoints_time_s, dtype=np.float64)
        t0, t1 = float(tw[0]), float(tw[-1])
        if t1 > t0 + 1e-12:
            return (tw - t0) / (t1 - t0) * (t[-1] - t[0]) + t[0]
    return np.linspace(t[0], t[-1], n_waypoints, dtype=np.float64)


def plot_trajectory(traj, waypoints, waypoints_time_s=None, *, block=True):
    """Plot replanned joint trajectory with q, qd and qdd.

    :param traj: Planned trajectory from plan_joint_trajectory
    :param waypoints: Planner input joints [N, num_joints]
    :param waypoints_time_s: Optional per-row times
    :param block: Wait until figure window is closed when True
    :return: None
    """
    try:
        import matplotlib.pyplot as plt

        t = np.asarray(traj["timestamps"], dtype=np.float64)
        q = np.asarray(traj["positions"], dtype=np.float64)
        wp = np.asarray(waypoints, dtype=np.float64)
        if q.ndim != 2 or wp.ndim != 2 or q.shape[1] != wp.shape[1]:
            beauty_print("绘图跳过：路点与轨迹关节维数不一致。", type="warning")
            return

        nj = q.shape[1]
        qd = np.asarray(traj.get("velocities", []), dtype=np.float64)
        qdd = np.asarray(traj.get("accelerations", []), dtype=np.float64)

        fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9.0, 8.0))

        for j in range(nj):
            axes[0].plot(t, q[:, j], lw=1.0, label=f"J{j}")
        axes[0].set_ylabel("q (rad)")
        axes[0].set_title("Joint trajectory (replanned): q / qd / qdd — dots: input waypoints")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc="upper right", ncol=3, fontsize=7)

        tw_plot = waypoint_times_for_plot(t, len(wp), waypoints_time_s)
        n_wp = len(wp)
        if n_wp > _MAX_WAYPOINT_SCATTER:
            idx = np.unique(np.linspace(0, n_wp - 1, num=_MAX_WAYPOINT_SCATTER, dtype=int))
        else:
            idx = np.arange(n_wp, dtype=int)
        for j in range(nj):
            axes[0].scatter(
                tw_plot[idx],
                wp[idx, j],
                color=f"C{j % 10}",
                s=8,
                alpha=0.45,
                zorder=3,
                marker="o",
                edgecolors="none",
            )

        if qd.shape == q.shape:
            for j in range(nj):
                axes[1].plot(t, qd[:, j], lw=1.0, label=f"J{j}")
            axes[1].legend(loc="upper right", ncol=3, fontsize=7)
        else:
            axes[1].text(0.5, 0.5, "(no qd data)", ha="center", va="center", transform=axes[1].transAxes, fontsize=10, alpha=0.55)
        axes[1].set_ylabel("qd (rad/s)")
        axes[1].grid(True, alpha=0.3)

        if qdd.size > 0 and qdd.shape == q.shape:
            for j in range(nj):
                axes[2].plot(t, qdd[:, j], lw=1.0, label=f"J{j}")
            axes[2].legend(loc="upper right", ncol=3, fontsize=7)
        else:
            axes[2].text(0.5, 0.5, "(no qdd data)", ha="center", va="center", transform=axes[2].transAxes, fontsize=10, alpha=0.55)
        axes[2].set_ylabel(r"qdd (rad/s$^2$)")
        axes[2].set_xlabel("t (s)")
        axes[2].grid(True, alpha=0.3)

        _matplotlib_show(block)
    except ImportError:
        beauty_print("matplotlib 不可用，跳过可视化。", type="warning")


def plot_joint_tracking(traj, tracking, *, block=True):
    """Plot planned joint positions vs measured joint positions.

    :param traj: Planned trajectory with timestamps and positions
    :param tracking: Dict with sampled timestamps and positions
    :param block: Wait until figure window is closed when True
    :return: None
    """
    try:
        import matplotlib.pyplot as plt

        t_tgt = np.asarray(traj["timestamps"], dtype=np.float64)
        q_tgt = np.asarray(traj["positions"], dtype=np.float64)
        if q_tgt.shape[1] < NUM_JOINTS:
            beauty_print("跟踪图跳过：轨迹关节列不足。", type="warning")
            return

        q_tgt = q_tgt[:, :NUM_JOINTS]
        t_m = np.asarray(tracking["timestamps"], dtype=np.float64)
        q_m = np.asarray(tracking["positions"], dtype=np.float64)
        if t_m.size == 0 or q_m.shape[0] == 0:
            beauty_print("跟踪图跳过：无采样数据。", type="warning")
            return
        if q_m.shape[1] != NUM_JOINTS:
            beauty_print("跟踪图跳过：采样关节维数不是 6。", type="warning")
            return

        fig, axes = plt.subplots(NUM_JOINTS, 1, sharex=True, figsize=(9.0, 10.0))
        if NUM_JOINTS == 1:
            axes = [axes]

        t_end = float(t_tgt[-1])
        for j in range(NUM_JOINTS):
            ax = axes[j]
            ax.plot(t_tgt, q_tgt[:, j], lw=1.25, color="C0", label="target")
            ax.plot(t_m, q_m[:, j], lw=1.0, color="C1", alpha=0.85, label="actual")
            ax.set_ylabel(f"J{j}\nq (rad)")
            ax.grid(True, alpha=0.3)
            ax.set_xlim(left=max(0.0, t_tgt[0]), right=max(t_end, float(np.max(t_m)) if t_m.size else t_end))

        axes[0].set_title("Joint tracking: target (planned PV) vs actual (polled joint state)")
        axes[-1].set_xlabel("t (s, same as planned trajectory timestamps)")
        h0, l0 = axes[0].get_legend_handles_labels()
        axes[0].legend(h0, l0, loc="upper right", fontsize=8)

        _matplotlib_show(block)
    except ImportError:
        beauty_print("matplotlib 不可用，跳过跟踪对比图。", type="warning")


def plot_joint_velocity_tracking(traj, tracking, *, block=True):
    """Plot planned joint velocity vs measured joint velocity.

    :param traj: Planned trajectory with timestamps and velocities
    :param tracking: Dict with sampled timestamps and velocities
    :param block: Wait until figure window is closed when True
    :return: None
    """
    try:
        import matplotlib.pyplot as plt

        t_tgt = np.asarray(traj["timestamps"], dtype=np.float64)
        qd_tgt = np.asarray(traj["velocities"], dtype=np.float64)
        if qd_tgt.shape[1] < NUM_JOINTS:
            beauty_print("速度跟踪图跳过：轨迹速度列不足。", type="warning")
            return
        qd_tgt = qd_tgt[:, :NUM_JOINTS]

        t_m = np.asarray(tracking["timestamps"], dtype=np.float64)
        qd_m = np.asarray(tracking.get("velocities", []), dtype=np.float64)
        if t_m.size == 0 or qd_m.shape[0] == 0:
            beauty_print("速度跟踪图跳过：无速度采样数据。", type="warning")
            return
        if qd_m.shape[1] != NUM_JOINTS:
            beauty_print("速度跟踪图跳过：采样关节维数不是 6。", type="warning")
            return

        valid_rows = np.all(np.isfinite(qd_m), axis=1)
        if not np.any(valid_rows):
            beauty_print("速度跟踪图跳过：设备未返回有效速度字段。", type="warning")
            return
        t_v = t_m[valid_rows]
        qd_valid = qd_m[valid_rows]

        fig, axes = plt.subplots(NUM_JOINTS, 1, sharex=True, figsize=(9.0, 10.0))
        if NUM_JOINTS == 1:
            axes = [axes]

        t_end = float(t_tgt[-1])
        for j in range(NUM_JOINTS):
            ax = axes[j]
            ax.plot(t_tgt, qd_tgt[:, j], lw=1.25, color="C0", label="target qd")
            ax.plot(t_v, qd_valid[:, j], lw=1.0, color="C3", alpha=0.85, label="actual qd")
            ax.set_ylabel(f"J{j}\nqd (rad/s)")
            ax.grid(True, alpha=0.3)
            right = max(t_end, float(np.max(t_v)) if t_v.size else t_end)
            ax.set_xlim(left=max(0.0, t_tgt[0]), right=right)

        axes[0].set_title("Joint velocity tracking: target qd vs measured qd")
        axes[-1].set_xlabel("t (s, same as planned trajectory timestamps)")
        h0, l0 = axes[0].get_legend_handles_labels()
        axes[0].legend(h0, l0, loc="upper right", fontsize=8)

        _matplotlib_show(block)
    except ImportError:
        beauty_print("matplotlib 不可用，跳过速度跟踪图。", type="warning")
