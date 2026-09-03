"""Pose recording, playback validation, and the slider recorder window."""

from __future__ import annotations

import math
import os
import queue
import sys
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from functools import partial
from pathlib import Path
from typing import Any, Callable, Iterable, Sequence


DEFAULT_JOINT_LIMITS_RAD: tuple[tuple[float, float], ...] = (
    (-2.7475, 2.7475),
    (-3.14, 0.0),
    (-3.14, 0.0),
    (-1.57, 1.57),
    (-1.57, 1.57),
    (-2.791, 2.791),
)
GRIPPER_LIMIT = (0.0, 1000.0)


class PoseFileError(ValueError):
    """A pose file is malformed or contains an unsafe value."""


class PosePlaybackError(RuntimeError):
    """A pose could not be sent or did not arrive before its timeout."""


@dataclass(frozen=True)
class PoseRecord:
    """Six joint positions in radians plus one gripper position."""

    joints_rad: tuple[float, float, float, float, float, float]
    gripper: float

    def __post_init__(self) -> None:
        if len(self.joints_rad) != 6:
            raise ValueError(f"姿态必须包含 6 个关节，实际为 {len(self.joints_rad)}")


def _validate_limits(joint_limits_rad: Sequence[tuple[float, float]]) -> None:
    if len(joint_limits_rad) != 6:
        raise ValueError(f"关节限位必须包含 6 组，实际为 {len(joint_limits_rad)}")


def validate_pose(
    pose: PoseRecord,
    joint_limits_rad: Sequence[tuple[float, float]],
    *,
    context: str = "姿态",
) -> None:
    """Reject non-finite or out-of-range pose values."""
    _validate_limits(joint_limits_rad)
    for index, (value, (lower, upper)) in enumerate(zip(pose.joints_rad, joint_limits_rad), start=1):
        if not math.isfinite(value):
            raise PoseFileError(f"{context}: J{index} 必须是有限数字")
        if value < lower or value > upper:
            raise PoseFileError(
                f"{context}: J{index}={value:.6f} rad 超出限位 [{lower:.6f}, {upper:.6f}]"
            )
    if not math.isfinite(pose.gripper):
        raise PoseFileError(f"{context}: 夹爪必须是有限数字")
    if not GRIPPER_LIMIT[0] <= pose.gripper <= GRIPPER_LIMIT[1]:
        raise PoseFileError(f"{context}: 夹爪={pose.gripper:g} 超出范围 [0, 1000]")


def parse_pose_lines(
    lines: Iterable[str],
    joint_limits_rad: Sequence[tuple[float, float]] = DEFAULT_JOINT_LIMITS_RAD,
) -> list[PoseRecord]:
    """Parse and validate a complete pose file before any movement occurs."""
    poses: list[PoseRecord] = []
    for line_number, raw_line in enumerate(lines, start=1):
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = stripped.split()
        if len(fields) != 7:
            raise PoseFileError(f"第 {line_number} 行: 应有 7 个空白分隔字段，实际为 {len(fields)}")
        try:
            values = [float(field) for field in fields]
        except ValueError as exc:
            raise PoseFileError(f"第 {line_number} 行: 所有字段都必须是数字") from exc
        pose = PoseRecord(tuple(values[:6]), values[6])
        validate_pose(pose, joint_limits_rad, context=f"第 {line_number} 行")
        poses.append(pose)
    if not poses:
        raise PoseFileError("姿态文件中没有有效姿态")
    return poses


def load_poses(
    path: Path | str,
    joint_limits_rad: Sequence[tuple[float, float]] = DEFAULT_JOINT_LIMITS_RAD,
) -> list[PoseRecord]:
    pose_path = Path(path).expanduser()
    if not pose_path.is_file():
        raise PoseFileError(f"姿态文件不存在: {pose_path}")
    try:
        return parse_pose_lines(pose_path.read_text(encoding="utf-8").splitlines(), joint_limits_rad)
    except OSError as exc:
        raise PoseFileError(f"无法读取姿态文件: {pose_path}: {exc}") from exc


def format_pose(pose: PoseRecord) -> str:
    values = [*(f"{value:.8f}" for value in pose.joints_rad), f"{pose.gripper:.8f}"]
    return " ".join(values)


def append_pose(
    path: Path | str,
    pose: PoseRecord,
    joint_limits_rad: Sequence[tuple[float, float]] = DEFAULT_JOINT_LIMITS_RAD,
) -> None:
    validate_pose(pose, joint_limits_rad)
    pose_path = Path(path).expanduser()
    pose_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with pose_path.open("a", encoding="utf-8", newline="\n") as stream:
            stream.write(format_pose(pose) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
    except OSError as exc:
        raise PoseFileError(f"无法写入姿态文件: {pose_path}: {exc}") from exc


def undo_last_pose(
    path: Path | str,
    joint_limits_rad: Sequence[tuple[float, float]] = DEFAULT_JOINT_LIMITS_RAD,
) -> PoseRecord:
    pose_path = Path(path).expanduser()
    if not pose_path.is_file():
        raise PoseFileError(f"姿态文件不存在: {pose_path}")
    try:
        lines = pose_path.read_text(encoding="utf-8").splitlines(keepends=True)
    except OSError as exc:
        raise PoseFileError(f"无法读取姿态文件: {pose_path}: {exc}") from exc

    last_index = None
    removed = None
    for index in range(len(lines) - 1, -1, -1):
        stripped = lines[index].strip()
        if stripped and not stripped.startswith("#"):
            removed = parse_pose_lines([stripped], joint_limits_rad)[0]
            last_index = index
            break
    if last_index is None or removed is None:
        raise PoseFileError("姿态文件中没有可撤销的姿态")

    del lines[last_index]
    temporary = pose_path.with_name(pose_path.name + ".tmp")
    try:
        temporary.write_text("".join(lines), encoding="utf-8", newline="")
        os.replace(temporary, pose_path)
    except OSError as exc:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        raise PoseFileError(f"无法更新姿态文件: {pose_path}: {exc}") from exc
    return removed


def count_pose_lines(path: Path | str) -> int:
    pose_path = Path(path).expanduser()
    if not pose_path.exists():
        return 0
    try:
        return sum(
            1
            for line in pose_path.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        )
    except OSError as exc:
        raise PoseFileError(f"无法读取姿态文件: {pose_path}: {exc}") from exc


def parse_urdf_revolute_limits(path: Path | str) -> tuple[tuple[float, float], ...]:
    urdf_path = Path(path).expanduser()
    if not urdf_path.is_file():
        raise PoseFileError(f"URDF 文件不存在: {urdf_path}")
    try:
        root = ET.parse(urdf_path).getroot()
    except (ET.ParseError, OSError) as exc:
        raise PoseFileError(f"无法解析 URDF: {urdf_path}: {exc}") from exc
    limits: list[tuple[float, float]] = []
    for joint in root.findall("joint"):
        if joint.attrib.get("type") != "revolute":
            continue
        limit = joint.find("limit")
        name = joint.attrib.get("name", "<unnamed>")
        if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
            raise PoseFileError(f"URDF 旋转关节 {name} 缺少 lower/upper 限位")
        try:
            limits.append((float(limit.attrib["lower"]), float(limit.attrib["upper"])))
        except ValueError as exc:
            raise PoseFileError(f"URDF 旋转关节 {name} 的限位不是数字") from exc
        if len(limits) == 6:
            return tuple(limits)
    raise PoseFileError(f"URDF 中仅找到 {len(limits)} 个旋转关节，需要至少 6 个")


def parse_target_value(
    text: str,
    index: int,
    joint_limits_rad: Sequence[tuple[float, float]] = DEFAULT_JOINT_LIMITS_RAD,
) -> float:
    """Parse an editable target using the same precision shown in the GUI."""
    _validate_limits(joint_limits_rad)
    if not 0 <= index <= 6:
        raise ValueError(f"目标索引无效: {index}")
    try:
        value = float(text.strip())
    except (AttributeError, ValueError) as exc:
        raise ValueError("请输入有效数字") from exc
    if not math.isfinite(value):
        raise ValueError("请输入有限数字")
    if index < 6:
        value = round(value, 3)
        lower, upper = joint_limits_rad[index]
        if not lower <= value <= upper:
            raise ValueError(f"J{index + 1}={value:.3f} rad 超出限位 [{lower:.3f}, {upper:.3f}]")
    else:
        value = round(value, 1)
        if not GRIPPER_LIMIT[0] <= value <= GRIPPER_LIMIT[1]:
            raise ValueError(f"夹爪={value:.1f} 超出范围 [0, 1000]")
    return value


def read_actual_control_mode(robot: Any) -> str:
    """Read the six arm motor modes from firmware, not the SDK mode cache."""
    try:
        modes = robot.get_robot_state("control_mode")
    except Exception:
        return "unknown"
    if not modes or len(modes) < 6:
        return "unknown"
    values: list[int] = []
    try:
        for item in modes[:6]:
            values.append(int(item.get("value")) if isinstance(item, dict) else int(item))
    except (TypeError, ValueError):
        return "unknown"
    if all(value == 2 for value in values):
        return "pv"
    if all(value == 1 for value in values):
        return "mit"
    return "mixed"


class ArrivalDetector:
    """Require several consecutive in-tolerance samples to reject jitter."""

    def __init__(self, tolerance_rad: float, gripper_tolerance: float, consecutive_samples: int = 3):
        if tolerance_rad <= 0 or gripper_tolerance < 0 or consecutive_samples < 1:
            raise ValueError("到位阈值必须为正数，连续样本数至少为 1")
        self.tolerance_rad = float(tolerance_rad)
        self.gripper_tolerance = float(gripper_tolerance)
        self.consecutive_samples = int(consecutive_samples)
        self._count = 0

    @property
    def stable_count(self) -> int:
        return self._count

    def update(self, joints_rad: Sequence[float], gripper: float, target: PoseRecord) -> bool:
        if len(joints_rad) != 6:
            raise ValueError(f"反馈必须包含 6 个关节，实际为 {len(joints_rad)}")
        arrived = (
            all(abs(float(actual) - expected) <= self.tolerance_rad for actual, expected in zip(joints_rad, target.joints_rad))
            and abs(float(gripper) - target.gripper) <= self.gripper_tolerance
        )
        self._count = self._count + 1 if arrived else 0
        return self._count >= self.consecutive_samples


@dataclass(frozen=True)
class PlaybackProgress:
    """One playback feedback sample for terminal or custom displays."""

    index: int
    total: int
    target: PoseRecord
    joint_errors_rad: tuple[float, float, float, float, float, float]
    gripper_error: float
    stable_count: int
    required_stable_count: int
    elapsed: float
    status: str


class ConsolePlaybackProgress:
    """Render moving feedback on one terminal line and keep final results."""

    def __init__(
        self,
        stream: Any = None,
        refresh_interval: float = 0.2,
        clock: Callable[[], float] = time.monotonic,
        interactive: bool | None = None,
    ) -> None:
        self.stream = stream if stream is not None else sys.stdout
        self.refresh_interval = max(0.0, float(refresh_interval))
        self.clock = clock
        self.interactive = bool(self.stream.isatty()) if interactive is None else bool(interactive)
        self._last_refresh = -math.inf
        self._last_width = 0

    def __call__(self, progress: PlaybackProgress) -> None:
        final = progress.status in {"arrived", "timeout"}
        now = self.clock()
        if not final and (not self.interactive or now - self._last_refresh < self.refresh_interval):
            return
        self._last_refresh = now
        line = self._format(progress)
        if self.interactive:
            padded = line.ljust(self._last_width)
            self.stream.write("\r" + padded + ("\n" if final else ""))
            self._last_width = 0 if final else len(line)
        elif final:
            self.stream.write(line + "\n")
        self.stream.flush()

    def _format(self, progress: PlaybackProgress) -> str:
        errors = progress.joint_errors_rad
        max_index = max(range(6), key=lambda index: errors[index])
        max_error = errors[max_index]
        targets = ",".join(f"{value:+.3f}" for value in progress.target.joints_rad)
        labels = {"moving": "运动中", "arrived": "已到位", "timeout": "超时"}
        line = (
            f"[{progress.index}/{progress.total}] {labels.get(progress.status, progress.status)} "
            f"目标(rad)=[{targets}] 夹爪={progress.target.gripper:.1f} | "
            f"最大误差 J{max_index + 1}={max_error:.3f}rad "
            f"夹爪误差={progress.gripper_error:.1f} "
            f"稳定={progress.stable_count}/{progress.required_stable_count} "
            f"耗时={progress.elapsed:.2f}s"
        )
        if progress.status == "timeout":
            axes = ",".join(f"{error:.3f}" for error in errors)
            line += f" 各轴误差=[{axes}]"
        return line


def execute_pose_sequence(
    robot: Any,
    poses: Sequence[PoseRecord],
    *,
    speed: float = 15.0,
    gripper_speed: float = 100.0,
    tolerance_rad: float = 0.008,
    gripper_tolerance: float = 10.0,
    timeout: float = 10.0,
    dwell: float = 0.3,
    poll_interval: float = 0.05,
    progress: Callable[[PlaybackProgress], None] | None = None,
) -> None:
    """Execute already validated poses and verify actual feedback at each line."""
    if not poses:
        raise PosePlaybackError("没有可执行的姿态")
    for index, pose in enumerate(poses, start=1):
        actual_mode = read_actual_control_mode(robot)
        if actual_mode != "pv":
            label = actual_mode.upper() if actual_mode != "unknown" else "未知"
            raise PosePlaybackError(f"第 {index} 个姿态发送前检测到实际模式为 {label}，已停止回放")
        sent = robot.set_robot_state(
            target_joints=list(pose.joints_rad),
            gripper_value=pose.gripper,
            joint_format="rad",
            speed=speed,
            gripper_speed=gripper_speed,
            wait_for_completion=False,
        )
        if sent is False:
            raise PosePlaybackError(f"第 {index} 个姿态下发失败")
        detector = ArrivalDetector(tolerance_rad, gripper_tolerance, consecutive_samples=3)
        started = time.monotonic()
        deadline = started + timeout
        max_error = math.inf
        joint_errors = (math.inf,) * 6
        gripper_error = math.inf
        if progress:
            progress(PlaybackProgress(
                index=index,
                total=len(poses),
                target=pose,
                joint_errors_rad=joint_errors,
                gripper_error=gripper_error,
                stable_count=0,
                required_stable_count=detector.consecutive_samples,
                elapsed=0.0,
                status="moving",
            ))
        while time.monotonic() < deadline:
            state = robot.get_robot_state("joint_gripper")
            if state and state.get("angles") is not None and state.get("gripper") is not None:
                joint_errors = tuple(
                    abs(float(actual) - expected)
                    for actual, expected in zip(state["angles"], pose.joints_rad)
                )
                max_error = max(joint_errors)
                gripper_error = abs(float(state["gripper"]) - pose.gripper)
                arrived = detector.update(state["angles"], state["gripper"], pose)
                if progress:
                    progress(PlaybackProgress(
                        index=index,
                        total=len(poses),
                        target=pose,
                        joint_errors_rad=joint_errors,
                        gripper_error=gripper_error,
                        stable_count=detector.stable_count,
                        required_stable_count=detector.consecutive_samples,
                        elapsed=time.monotonic() - started,
                        status="arrived" if arrived else "moving",
                    ))
                if arrived:
                    break
            time.sleep(poll_interval)
        else:
            if progress:
                progress(PlaybackProgress(
                    index=index,
                    total=len(poses),
                    target=pose,
                    joint_errors_rad=joint_errors,
                    gripper_error=gripper_error,
                    stable_count=detector.stable_count,
                    required_stable_count=detector.consecutive_samples,
                    elapsed=time.monotonic() - started,
                    status="timeout",
                ))
            raise PosePlaybackError(
                f"第 {index}/{len(poses)} 个姿态在 {timeout:g}s 内未到位，关节最大误差 {max_error:.6f} rad"
            )
        if dwell > 0:
            time.sleep(dwell)


class PoseSliderWindow:
    """Fixed-size, throttled Tk recorder for six joints and one gripper."""

    def __init__(
        self,
        root: Any,
        robot: Any,
        record_path: Path | str,
        initial_joints_rad: Sequence[float],
        initial_gripper: float,
        joint_limits_rad: Sequence[tuple[float, float]] = DEFAULT_JOINT_LIMITS_RAD,
        *,
        speed: float = 15.0,
        gripper_speed: float = 100.0,
        send_interval_ms: int = 120,
    ) -> None:
        if len(initial_joints_rad) != 6:
            raise ValueError("初始反馈必须包含 6 个关节")
        _validate_limits(joint_limits_rad)
        import tkinter as tk
        from tkinter import ttk

        self.root = root
        self.robot = robot
        self.record_path = Path(record_path).expanduser()
        self.joint_limits_rad = tuple(joint_limits_rad)
        self._tk = tk
        self._ttk = ttk
        self._closed = False
        self._initializing = True
        self._dirty = False
        self._syncing_target = False
        self._send_after: str | None = None
        self._feedback_after: str | None = None
        self._mode_after: str | None = None
        self._forced_mode_after: str | None = None
        self._mode_query_running = False
        self._mode_results: queue.SimpleQueue[tuple[str, float]] = queue.SimpleQueue()
        self._io_lock = threading.Lock()
        self._live_mode = "pv"
        self._worker_mode = "pv"
        self._ignore_mode_results_until = 0.0
        self._last_single_click = False
        self._actual_joints = tuple(float(value) for value in initial_joints_rad)
        self._actual_gripper = float(initial_gripper)
        self._pose_count = count_pose_lines(self.record_path)

        root.title("Alicia-M 姿态记录器")
        root.geometry("720x480")
        root.resizable(False, False)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.configure(background="#f3f5f7")

        style = ttk.Style(root)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 13, "bold"), background="#f3f5f7")
        style.configure("Hint.TLabel", foreground="#5c6670", background="#f3f5f7")
        style.configure("Value.TLabel", font=("Consolas", 9))
        style.configure("Accent.TButton", font=("Microsoft YaHei UI", 10, "bold"))

        self.status_var = tk.StringVar(value="就绪：拖动滑块控制，记录按钮保存实际反馈")
        self.mode_var = tk.StringVar(value="实际模式：PV")
        self.path_var = tk.StringVar(value=f"文件：{self.record_path}    已记录：{self._pose_count}")
        self.speed_var = tk.DoubleVar(value=float(speed))
        self.gripper_speed_var = tk.DoubleVar(value=float(gripper_speed))
        self.interval_var = tk.DoubleVar(value=float(send_interval_ms))
        self.parameter_text_vars: list[Any] = []
        self.target_vars: list[Any] = []
        self.target_text_vars: list[Any] = []
        self.actual_text_vars: list[Any] = []
        self.sliders: list[Any] = []
        self.target_entries: list[Any] = []

        self._build(initial_joints_rad, initial_gripper)
        self._initializing = False
        self._schedule_feedback()
        self._schedule_mode_check(250)

    def _build(self, initial_joints_rad: Sequence[float], initial_gripper: float) -> None:
        ttk = self._ttk
        header = ttk.Frame(self.root, padding=(16, 10, 16, 4))
        header.pack(fill="x")
        ttk.Label(header, text="Alicia-M 姿态记录器", style="Title.TLabel").pack(side="left")
        ttk.Label(header, textvariable=self.mode_var, style="Hint.TLabel").pack(side="right")

        table = ttk.LabelFrame(self.root, text="关节与夹爪", padding=(10, 5))
        table.pack(fill="x", padx=14)
        ttk.Label(table, text="轴", width=5).grid(row=0, column=0)
        ttk.Label(table, text="目标(rad/值)", width=13).grid(row=0, column=2)
        ttk.Label(table, text="实际(rad/值)", width=13).grid(row=0, column=3)
        table.columnconfigure(1, weight=1)

        initial_values = [*(float(v) for v in initial_joints_rad), float(initial_gripper)]
        ranges = [*self.joint_limits_rad, GRIPPER_LIMIT]
        for index, (value, limits) in enumerate(zip(initial_values, ranges)):
            name = f"J{index + 1}" if index < 6 else "夹爪"
            variable = self._tk.DoubleVar(value=min(max(value, limits[0]), limits[1]))
            target_text = self._tk.StringVar(value=self._format_target(index, variable.get()))
            actual_text = self._tk.StringVar(value=self._format_target(index, value))
            self.target_vars.append(variable)
            self.target_text_vars.append(target_text)
            self.actual_text_vars.append(actual_text)
            ttk.Label(table, text=name, width=5).grid(row=index + 1, column=0, sticky="w", pady=1)
            slider = ttk.Scale(
                table,
                from_=limits[0],
                to=limits[1],
                variable=variable,
                command=partial(self._slider_changed, index),
                length=410,
            )
            slider.grid(row=index + 1, column=1, sticky="ew", padx=(2, 10), pady=1)
            entry = ttk.Entry(table, textvariable=target_text, width=13, justify="right", font=("Consolas", 9))
            entry.grid(row=index + 1, column=2, padx=(0, 5))
            entry.bind("<FocusOut>", partial(self._target_entry_committed, index))
            entry.bind("<Return>", partial(self._target_entry_committed, index))
            ttk.Label(table, textvariable=actual_text, style="Value.TLabel", width=13).grid(row=index + 1, column=3)
            self.sliders.append(slider)
            self.target_entries.append(entry)

        controls = ttk.LabelFrame(self.root, text="运动参数", padding=(10, 4))
        controls.pack(fill="x", padx=14, pady=(7, 0))
        settings = (
            ("关节速度", self.speed_var, 1, 100, ""),
            ("夹爪速度", self.gripper_speed_var, 1, 400, ""),
            ("发送间隔", self.interval_var, 50, 500, " ms"),
        )
        for column, (label, variable, lower, upper, unit) in enumerate(settings):
            block = ttk.Frame(controls)
            block.grid(row=0, column=column, sticky="ew", padx=(0, 12 if column < 2 else 0))
            controls.columnconfigure(column, weight=1)
            ttk.Label(block, text=label).pack(anchor="w")
            parameter_text = self._tk.StringVar(value=f"{variable.get():.0f}{unit}")
            self.parameter_text_vars.append(parameter_text)
            ttk.Scale(
                block,
                from_=lower,
                to=upper,
                variable=variable,
                command=partial(self._parameter_changed, column, unit),
                length=150,
            ).pack(side="left")
            value_label = ttk.Label(block, textvariable=parameter_text, width=7)
            value_label.pack(side="left", padx=(4, 0))

        actions = ttk.Frame(self.root, padding=(14, 8, 14, 2))
        actions.pack(fill="x")
        ttk.Button(actions, text="记录实际姿态", style="Accent.TButton", command=self.record_actual).pack(side="left")
        ttk.Button(actions, text="撤销上一条", command=self.undo).pack(side="left", padx=8)
        ttk.Label(actions, textvariable=self.path_var, style="Hint.TLabel").pack(side="left", padx=8)

        status = ttk.Label(self.root, textvariable=self.status_var, anchor="w", relief="sunken", padding=(10, 5))
        status.pack(side="bottom", fill="x")

    def _slider_changed(self, index: int, raw_value: str) -> None:
        if self._closed or self._initializing or self._syncing_target or self._live_mode != "pv":
            return
        value = round(float(raw_value), 3 if index < 6 else 1)
        self._syncing_target = True
        try:
            self.target_vars[index].set(value)
            self.target_text_vars[index].set(self._format_target(index, value))
        finally:
            self._syncing_target = False
        self._queue_target_send()

    def _target_entry_committed(self, index: int, _event: Any = None) -> str | None:
        if self._closed:
            return None
        if self._live_mode != "pv":
            self._sync_one_target_text(index)
            self.status_var.set("当前不是 PV，目标输入未发送")
            return "break" if getattr(_event, "keysym", "") == "Return" else None
        previous = float(self.target_vars[index].get())
        try:
            value = parse_target_value(self.target_text_vars[index].get(), index, self.joint_limits_rad)
        except ValueError as exc:
            self._sync_one_target_text(index)
            self.status_var.set(f"目标输入无效：{exc}")
            try:
                self.target_entries[index].selection_range(0, "end")
            except Exception:
                pass
            return "break" if getattr(_event, "keysym", "") == "Return" else None

        self._syncing_target = True
        try:
            self.target_vars[index].set(value)
            self.target_text_vars[index].set(self._format_target(index, value))
        finally:
            self._syncing_target = False
        if value != previous:
            self._queue_target_send()
        else:
            self.status_var.set("目标值未变化")
        return "break" if getattr(_event, "keysym", "") == "Return" else None

    def _format_target(self, index: int, value: float) -> str:
        return f"{float(value):+.3f}" if index < 6 else f"{float(value):.1f}"

    def _sync_one_target_text(self, index: int) -> None:
        self.target_text_vars[index].set(self._format_target(index, self.target_vars[index].get()))

    def _queue_target_send(self) -> None:
        self._dirty = True
        if self._send_after is None:
            self._send_after = self.root.after(max(50, int(self.interval_var.get())), self._flush_target)

    def _parameter_changed(self, index: int, unit: str, raw_value: str) -> None:
        self.parameter_text_vars[index].set(f"{float(raw_value):.0f}{unit}")

    def _flush_target(self) -> None:
        self._send_after = None
        if self._closed or not self._dirty:
            return
        if self._live_mode != "pv":
            self._dirty = False
            self.status_var.set("实际模式不是 PV，目标未发送")
            return
        if not self._io_lock.acquire(blocking=False):
            self._send_after = self.root.after(50, self._flush_target)
            return
        self._dirty = False
        joints_rad = [variable.get() for variable in self.target_vars[:6]]
        gripper = self.target_vars[6].get()
        try:
            ok = self.robot.set_robot_state(
                target_joints=joints_rad,
                gripper_value=gripper,
                joint_format="rad",
                speed=max(1.0, min(100.0, self.speed_var.get())),
                gripper_speed=max(1.0, min(400.0, self.gripper_speed_var.get())),
                wait_for_completion=False,
            )
            self.status_var.set("目标已发送" if ok is not False else "发送失败，请检查机械臂状态")
        except Exception as exc:
            self.status_var.set(f"发送失败：{exc}")
        finally:
            self._io_lock.release()
        if self._dirty and not self._closed:
            self._send_after = self.root.after(max(50, int(self.interval_var.get())), self._flush_target)

    def _schedule_feedback(self) -> None:
        if not self._closed:
            self._feedback_after = self.root.after(160, self._refresh_feedback)

    def _refresh_feedback(self) -> None:
        self._feedback_after = None
        if self._closed:
            return
        try:
            state = self.robot.get_robot_state("joint_gripper")
            if state and len(state.get("angles", ())) == 6 and state.get("gripper") is not None:
                self._actual_joints = tuple(float(value) for value in state["angles"])
                self._actual_gripper = float(state["gripper"])
                values = [*self._actual_joints, self._actual_gripper]
                for index, value in enumerate(values):
                    self.actual_text_vars[index].set(self._format_target(index, value))
                if self._live_mode == "mit":
                    self._sync_targets_to_actual()
            status = self.robot.get_robot_state("status")
            single_click = bool(getattr(status, "single_click", False)) if status is not None else False
            if single_click and not self._last_single_click:
                self._block_for_button_transition()
            self._last_single_click = single_click
            self._consume_mode_results()
        except Exception as exc:
            self.status_var.set(f"反馈读取失败：{exc}")
        self._schedule_feedback()

    def _sync_targets_to_actual(self) -> None:
        values = [*self._actual_joints, self._actual_gripper]
        self._syncing_target = True
        try:
            for index, value in enumerate(values):
                normalized = round(value, 3 if index < 6 else 1)
                self.target_vars[index].set(normalized)
                self.target_text_vars[index].set(self._format_target(index, normalized))
        finally:
            self._syncing_target = False

    def _set_target_controls_enabled(self, enabled: bool) -> None:
        slider_state = ["!disabled"] if enabled else ["disabled"]
        entry_state = ["!readonly", "!disabled"] if enabled else ["readonly"]
        for slider in self.sliders:
            slider.state(slider_state)
        for entry in self.target_entries:
            entry.state(entry_state)

    def _cancel_pending_target(self) -> None:
        self._dirty = False
        if self._send_after is not None:
            try:
                self.root.after_cancel(self._send_after)
            except Exception:
                pass
            self._send_after = None

    def _block_for_button_transition(self) -> None:
        self._live_mode = "checking"
        self._ignore_mode_results_until = time.monotonic() + 0.30
        self._cancel_pending_target()
        self._set_target_controls_enabled(False)
        self.mode_var.set("实际模式：检测中")
        self.status_var.set("检测到模式按键，已暂停发送并等待固件切换")
        if self._forced_mode_after is not None:
            try:
                self.root.after_cancel(self._forced_mode_after)
            except Exception:
                pass
        self._forced_mode_after = self.root.after(350, self._start_mode_query)

    def _schedule_mode_check(self, delay_ms: int = 900) -> None:
        if not self._closed:
            self._mode_after = self.root.after(delay_ms, self._mode_check_tick)

    def _mode_check_tick(self) -> None:
        self._mode_after = None
        self._start_mode_query()
        self._schedule_mode_check()

    def _start_mode_query(self) -> None:
        self._forced_mode_after = None
        if self._closed or self._mode_query_running:
            return
        self._mode_query_running = True
        threading.Thread(target=self._mode_query_worker, daemon=True).start()

    def _mode_query_worker(self) -> None:
        result = "unknown"
        try:
            with self._io_lock:
                result = read_actual_control_mode(self.robot)
                if result == "pv" and self._worker_mode != "pv":
                    if self.robot.set_linear_interpolation_velocity(0.0) is False:
                        result = "unknown"
                self._worker_mode = result
        except Exception:
            result = "unknown"
        finally:
            self._mode_results.put((result, time.monotonic()))
            self._mode_query_running = False

    def _consume_mode_results(self) -> None:
        latest: tuple[str, float] | None = None
        while True:
            try:
                latest = self._mode_results.get_nowait()
            except queue.Empty:
                break
        if latest is None or latest[1] < self._ignore_mode_results_until:
            return
        mode = latest[0]
        previous = self._live_mode
        self._live_mode = mode
        if mode == "pv":
            if previous != "pv":
                self._sync_targets_to_actual()
            self._set_target_controls_enabled(True)
            self.mode_var.set("实际模式：PV")
            self.status_var.set("PV 已就绪；目标已同步当前姿态，等待用户操作")
        else:
            self._cancel_pending_target()
            self._set_target_controls_enabled(False)
            labels = {"mit": "MIT 手动跟随", "mixed": "混合模式", "unknown": "未知/查询失败"}
            self.mode_var.set(f"实际模式：{labels.get(mode, mode)}")
            if mode == "mit":
                self._sync_targets_to_actual()
                self.status_var.set("MIT 手动模式：滑块跟随实际姿态，禁止目标发送")
            else:
                self.status_var.set("模式不确定：已禁止目标发送")

    def record_actual(self) -> None:
        try:
            state = self.robot.get_robot_state("joint_gripper")
            if not state or len(state.get("angles", ())) != 6 or state.get("gripper") is None:
                raise PoseFileError("点击记录时没有收到完整实际反馈")
            self._actual_joints = tuple(float(value) for value in state["angles"])
            self._actual_gripper = float(state["gripper"])
            pose = PoseRecord(self._actual_joints, self._actual_gripper)
            append_pose(self.record_path, pose, self.joint_limits_rad)
            self._pose_count += 1
            self.path_var.set(f"文件：{self.record_path}    已记录：{self._pose_count}")
            self.status_var.set(f"已记录第 {self._pose_count} 条（实际反馈，不是滑块目标）")
        except PoseFileError as exc:
            self.status_var.set(f"记录失败：{exc}")

    def undo(self) -> None:
        try:
            undo_last_pose(self.record_path, self.joint_limits_rad)
            self._pose_count = max(0, self._pose_count - 1)
            self.path_var.set(f"文件：{self.record_path}    已记录：{self._pose_count}")
            self.status_var.set("已撤销上一条姿态")
        except PoseFileError as exc:
            self.status_var.set(f"无法撤销：{exc}")

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        for after_id in (
            self._send_after,
            self._feedback_after,
            self._mode_after,
            self._forced_mode_after,
        ):
            if after_id is not None:
                try:
                    self.root.after_cancel(after_id)
                except Exception:
                    pass
        try:
            with self._io_lock:
                self.robot.disconnect()
        finally:
            self.root.destroy()
