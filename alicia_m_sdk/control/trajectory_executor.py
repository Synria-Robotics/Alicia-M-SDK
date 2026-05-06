"""PV trajectory playback and common trajectory helpers."""

import time
import threading
import csv
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from ..hardware.device import Device
from ..protocol.constants import NUM_JOINTS, NUM_MOTORS
from ..types.exceptions import ValidationError, MotionError
from ..utils.beauty_logger import beauty_print, logger
from ..utils.timing import precise_sleep

class TrajectoryExecutor:
    """Playback executor that emits one PV frame per planned timestep.

    :param device, Device used to encode and send PV frames
    """

    def __init__(self, device: Device):
        self._device = device
        self._running = False
        self._stop_event = threading.Event()

    @property
    def is_running(self) -> bool:
        """Trajectory playback flag exposed for callers.

        :return, True while execute_pv is active until its finally block
        """
        return self._running

    def execute_pv(
        self,
        timestamps: np.ndarray,
        positions: np.ndarray,
        velocities: np.ndarray,
        *,
        perf_counter_origin_out: Optional[List[float]] = None,
        frame_joint_feedback_out: Optional[
            List[Tuple[float, np.ndarray, Optional[np.ndarray]]]
        ] = None,
    ) -> bool:
        """Stream planned PV samples on the trajectory clock.

        :param timestamps, seconds from trajectory start, shape [N], monotonic
        :param positions, rad, shape [N, NUM_MOTORS], six joints plus gripper
        :param velocities, signed rad/s, shape [N, NUM_MOTORS]
        :param perf_counter_origin_out, if given, append playback start time.perf_counter() once (same origin as timestamps)
        :param frame_joint_feedback_out, if given, append
            ``(timestamp_s, q_rad[6], qd_rad_s[6] or None)`` after each
            ``send_pv`` from device joint_state cache
        :return, True when all frames sent; False if stop interrupted the loop
        """
        # Input checks
        n_frames = len(timestamps)
        if n_frames == 0:
            raise ValidationError("empty trajectory: timestamps length is 0")

        if positions.shape != (n_frames, NUM_MOTORS):
            raise ValidationError(
                f"positions shape mismatch: expected ({n_frames}, {NUM_MOTORS}), "
                f"got {positions.shape}"
            )
        if velocities.shape != (n_frames, NUM_MOTORS):
            raise ValidationError(
                f"velocities shape mismatch: expected ({n_frames}, {NUM_MOTORS}), "
                f"got {velocities.shape}"
            )

        # Arm playback
        self._stop_event.clear()
        self._running = True
        logger.info(f"PV trajectory start: {n_frames} frames, span {timestamps[-1] - timestamps[0]:.2f}s")

        try:
            t_start = time.perf_counter()
            if perf_counter_origin_out is not None:
                perf_counter_origin_out.append(t_start)

            for frame_idx in range(n_frames):
                if self._stop_event.is_set():
                    logger.info(f"PV trajectory interrupted at frame {frame_idx}/{n_frames}")
                    return False

                target_time = t_start + timestamps[frame_idx]
                now = time.perf_counter()
                if target_time > now:
                    precise_sleep(target_time - now)

                pos_frame = [float(positions[frame_idx, m]) for m in range(NUM_MOTORS)]
                pos_frame[-1] = 0
                # vel_frame = [float(velocities[frame_idx, m]) for m in range(NUM_MOTORS)]
                vel_frame = [float(10) for m in range(NUM_MOTORS)]
                vel_frame[-1] = 0

                self._device.send_pv(self._device.aim, pos_frame, vel_frame)

                if frame_joint_feedback_out is not None:
                    js = self._device.joint_state
                    if js is not None and len(js.angles) >= NUM_JOINTS:
                        q_fb = np.array(js.angles[:NUM_JOINTS], dtype=np.float64, copy=True)
                        qd_fb = None
                        if js.velocities is not None and len(js.velocities) >= NUM_JOINTS:
                            qd_fb = np.array(
                                js.velocities[:NUM_JOINTS],
                                dtype=np.float64,
                                copy=True,
                            )
                        frame_joint_feedback_out.append(
                            (float(timestamps[frame_idx]), q_fb, qd_fb)
                        )

            elapsed = time.perf_counter() - t_start
            logger.info(f"PV trajectory done: elapsed {elapsed:.3f}s")
            return True

        except Exception as e:
            logger.error(f"PV trajectory failed: {e}")
            raise MotionError(f"trajectory playback failed: {e}") from e

        finally:
            self._running = False

    def stop(self) -> None:
        """Signal execute_pv to exit and send a zero-velocity hold at current pose."""
        self._stop_event.set()

        # Hold current angle with zero commanded velocity
        state = self._device.joint_state
        if state is not None:
            positions = list(state.angles[:NUM_JOINTS]) + [state.gripper]
            velocities = [0.0] * NUM_MOTORS
            self._device.send_pv(self._device.aim, positions, velocities)
            logger.info("trajectory stop: sent zero-velocity hold frame")
        else:
            logger.warning("trajectory stop: no joint state; stop flag only")


def load_waypoints_from_file(path):
    """Load joint waypoints from text/npy file.

    :param path: Waypoint file path
    :return: (waypoints [N, NUM_JOINTS], meta dict) or None
    """
    file_path = Path(path).expanduser()
    if not file_path.exists():
        beauty_print(f"文件不存在: {file_path}", type="error")
        return None

    meta = {}
    if file_path.suffix.lower() == ".npy":
        data = np.load(file_path)
    else:
        skiprows = 0
        with file_path.open(encoding="utf-8") as f:
            first = f.readline()
        if first.strip():
            cell0 = first.split(",", 1)[0].strip()
            try:
                float(cell0)
            except ValueError:
                skiprows = 1
        data = np.loadtxt(file_path, delimiter=",", ndmin=2, skiprows=skiprows)

    data = np.asarray(data, dtype=np.float64)
    if data.ndim != 2 or data.shape[0] < 2:
        beauty_print("文件格式错误：至少需要 2 行有效数据。", type="error")
        return None

    ncol = data.shape[1]
    traj_no_acc = 1 + 2 * NUM_JOINTS
    traj_with_acc = 1 + 3 * NUM_JOINTS
    if ncol == NUM_JOINTS:
        return data, meta

    if ncol not in (traj_no_acc, traj_with_acc):
        beauty_print(
            f"文件格式错误：期望 [{NUM_JOINTS}] 列路点，或轨迹导出 "
            f"({traj_no_acc}/{traj_with_acc} 列)，当前 {ncol} 列。",
            type="error",
        )
        return None

    beauty_print(
        "检测到轨迹 CSV：使用关节位置列为密集路点并重新规划（非 CSV 逐点回放）。",
        type="info",
    )
    meta["waypoints_time_s"] = np.asarray(data[:, 0], dtype=np.float64).copy()
    return data[:, 1 : 1 + NUM_JOINTS], meta


def save_trajectory_csv(traj, output_dir=None):
    """Save planned trajectory columns to a timestamped CSV file.

    :param traj: Plan result with timestamps, positions, velocities and optional accelerations
    :param output_dir: Directory for the CSV; cwd if None
    :return: Path to the written file
    """
    root = Path(output_dir or ".").expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    name = f"traj_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    path = root / name

    t = np.asarray(traj["timestamps"], dtype=np.float64)
    q = np.asarray(traj["positions"], dtype=np.float64)
    qd = np.asarray(traj["velocities"], dtype=np.float64)
    qdd = np.asarray(traj.get("accelerations", []), dtype=np.float64)

    n, nj = q.shape
    if len(t) != n:
        raise ValueError("timestamps length must match positions rows")
    if qd.shape != (n, nj):
        raise ValueError("velocities shape must match positions")

    headers = ["timestamp_s"]
    headers += [f"q{i}_rad" for i in range(nj)]
    headers += [f"qd{i}_rad_s" for i in range(nj)]
    include_qdd = qdd.size > 0 and qdd.shape == (n, nj)
    if include_qdd:
        headers += [f"qdd{i}_rad_s2" for i in range(nj)]

    rows = []
    for i in range(n):
        row = [float(t[i])]
        row.extend(float(x) for x in q[i])
        row.extend(float(x) for x in qd[i])
        if include_qdd:
            row.extend(float(x) for x in qdd[i])
        rows.append(row)

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        writer.writerows(rows)
    return path


def resolve_default_traj_save_dir(anchor_file):
    """Resolve default trajectory save directory under Alicia-M-SDK/logs.

    :param anchor_file: Current script file path
    :return: Default output directory path
    """
    script_dir = Path(anchor_file).resolve().parent
    for parent in [script_dir, *script_dir.parents]:
        if parent.name == "Alicia-M-SDK":
            return parent / "logs"
    return script_dir.parent / "logs"


def execute_joint_trajectory(robot, traj, speed, track_hz=None):
    """Execute planned joint trajectory in PV mode.

    :param robot: Connected robot instance
    :param traj: Trajectory dictionary from planner
    :param speed: Execution speed hint for logs
    :param track_hz: Target sample rate for tracking
    :return: (success flag, tracking dict or None)
    """
    timestamps = np.asarray(traj["timestamps"], dtype=np.float64)
    positions = np.asarray(traj["positions"], dtype=np.float64)
    velocities = np.asarray(traj["velocities"], dtype=np.float64)

    if positions.shape[1] != NUM_JOINTS:
        beauty_print("轨迹维度不是 6 轴，无法执行。", type="error")
        return False, None

    state = robot.get_robot_state("joint_gripper")
    gripper = float(state["gripper"]) if state and state.get("gripper") is not None else 0.0
    gripper_col = np.full((positions.shape[0], 1), gripper, dtype=np.float64)
    pos_full = np.hstack([positions, gripper_col])
    vel_full = np.hstack([velocities, np.zeros_like(gripper_col)])

    use_track = track_hz is not None and float(track_hz) > 0.0
    if not use_track:
        beauty_print(f"开始执行轨迹（speed={speed}，PV 回放）", type="info")
        ok = robot._traj_executor.execute_pv(timestamps, pos_full, vel_full)
        return ok, None

    hz = float(track_hz)
    beauty_print(
        f"开始执行轨迹（speed={speed}，PV 回放，每帧读关节反馈，目标显示约 {hz:.1f} Hz）",
        type="info",
    )
    feedback = []
    ok = robot._traj_executor.execute_pv(
        timestamps,
        pos_full,
        vel_full,
        frame_joint_feedback_out=feedback,
    )

    if feedback:
        dts = np.diff(timestamps)
        med_dt = float(np.median(dts)) if timestamps.size >= 2 else 0.0
        stride = max(1, int(round((1.0 / med_dt) / hz))) if med_dt > 1e-9 else 1
        if stride > 1:
            feedback = feedback[::stride]

    if feedback:
        t_arr = np.asarray([p[0] for p in feedback], dtype=np.float64)
        q_arr = np.stack([p[1] for p in feedback], axis=0)
        qd_rows = []
        for _, _, qd in feedback:
            qd_rows.append([np.nan] * NUM_JOINTS if qd is None else qd.tolist())
        qd_arr = np.asarray(qd_rows, dtype=np.float64)
    else:
        t_arr = np.zeros(0, dtype=np.float64)
        q_arr = np.zeros((0, NUM_JOINTS), dtype=np.float64)
        qd_arr = np.zeros((0, NUM_JOINTS), dtype=np.float64)

    tracking: Dict[str, np.ndarray] = {
        "timestamps": t_arr,
        "positions": q_arr,
        "velocities": qd_arr,
    }
    beauty_print(
        f"关节反馈 {len(t_arr)} 点（时间轴=规划 timestamps；PV 高频时勿依赖单独轮询线程读缓存）",
        type="info",
    )
    return ok, tracking
