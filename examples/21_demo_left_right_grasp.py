"""21_demo_left_right_grasp.py - left/right grasp action sequence.

Run a fixed joint-space grasp sequence using radians, with configurable
fast/slow/free speeds, gripper open/close values, and left/right cycles.
"""

from __future__ import annotations

import argparse
import csv
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from math import ceil
from pathlib import Path
from typing import Any, Optional, Union

import alicia_m_sdk
from _demo_helpers import add_port_argument, beauty_print, beauty_print_array


HOME_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
DEFAULT_GRIPPER_SPEED = 100.0
DEFAULT_GRIPPER_CLOSE_SPEED = 40.0
DEFAULT_GRIPPER_OPEN_SPEED = 100.0
DEFAULT_GRIPPER_RAMP_FREQUENCY = 25.0
DEFAULT_GRIPPER_MAX_RAMP_DURATION = 6.0
GRIPPER_FULL_RANGE_RAD = 2.64
OSCILLATION_SAMPLE_INTERVAL_S = 0.02
GRIPPER_ABORT_MASK = 1 << 6
PV_ARM_ARRIVAL_TOLERANCE_RAD = 0.05
MIT_ARM_ARRIVAL_TOLERANCE_RAD = 0.15
ARM_ARRIVAL_TIMEOUT_S = 15.0
ARM_ARRIVAL_POLL_INTERVAL_S = 0.01
PV_STATIONARY_TARGET_EPSILON_RAD = 1e-6


@dataclass(frozen=True)
class TrajectoryProfile:
    name: str
    mid_pose: list[float]
    left_transfer_pose: list[float]
    right_transfer_pose: list[float]
    grasp_depth_pose: list[float]


@dataclass(frozen=True)
class ArmAction:
    label: str
    joints: list[float]
    speed: float


@dataclass(frozen=True)
class GripperAction:
    label: str
    value: float


Action = Union[ArmAction, GripperAction]


@dataclass
class GripperCommandState:
    """Last commanded J7 target carried across subsequent arm motions."""

    target: Optional[float] = None
    speed: Optional[float] = None


@dataclass
class ArmCommandState:
    """Last commanded J1-J6 target used by PV diagnostic isolation."""

    target: Optional[list[float]] = None


def _indexed_fields(prefix: str) -> list[str]:
    return [f"{prefix}_j{index}" for index in range(1, 8)]


OSCILLATION_CSV_FIELDS = [
    "host_time_s",
    "elapsed_s",
    "state_timestamp_s",
    "trajectory",
    "control_mode",
    "event",
    "action_index",
    "action_count",
    "action_type",
    "action_label",
    "action_phase",
    "action_speed",
    "target_gripper",
    *[f"target_j{index}_rad" for index in range(1, 7)],
    "run_status",
    "gripper_abort",
    *[f"position_j{index}_rad" for index in range(1, 7)],
    "position_j7_gripper",
    *_indexed_fields("velocity_rad_s"),
    *_indexed_fields("torque_nm"),
    *_indexed_fields("kp"),
    *_indexed_fields("kd"),
    *_indexed_fields("linear_velocity_rad_s"),
    *_indexed_fields("temperature_c"),
]


class OscillationRecorder:
    """Record cached joint state without issuing additional device requests."""

    def __init__(
        self,
        robot: Any,
        trajectory: str,
        output_path: Path,
        sample_interval_s: float = OSCILLATION_SAMPLE_INTERVAL_S,
        control_mode: str = "pv",
    ) -> None:
        self.robot = robot
        self.trajectory = str(trajectory)
        self.control_mode = str(control_mode).lower()
        self.output_path = Path(output_path)
        self.sample_interval_s = max(float(sample_interval_s), 0.001)
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._rows: list[dict[str, Any]] = []
        self._started_at = 0.0
        self._last_state_timestamp: Optional[float] = None
        self._action: Optional[Action] = None
        self._action_index = 0
        self._action_count = 0
        self._action_phase = "idle"

    @property
    def row_count(self) -> int:
        with self._lock:
            return len(self._rows)

    def start(self) -> None:
        if self._thread is not None:
            return
        self._started_at = time.time()
        self._stop_event.clear()
        self._capture("session_start", deduplicate=False)
        self._thread = threading.Thread(
            target=self._sample_loop,
            daemon=True,
            name="demo21-oscillation-recorder",
        )
        self._thread.start()

    def stop(self) -> None:
        if self._thread is None:
            return
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        self._thread = None
        self._capture("session_stop", deduplicate=False)

    def mark_action(
        self,
        action_index: int,
        action_count: int,
        action: Action,
        phase: str,
    ) -> None:
        with self._lock:
            self._action_index = int(action_index)
            self._action_count = int(action_count)
            self._action = action
            self._action_phase = str(phase)
            self._capture(f"action_{phase}", deduplicate=False)

    def write_csv(self) -> Path:
        with self._lock:
            rows = list(self._rows)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=OSCILLATION_CSV_FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        return self.output_path

    def capture_once(self) -> bool:
        """Capture one cache snapshot; exposed for deterministic tests."""
        return self._capture("sample", deduplicate=True)

    def _sample_loop(self) -> None:
        while not self._stop_event.wait(self.sample_interval_s):
            self.capture_once()

    @staticmethod
    def _sequence_value(values: Any, index: int) -> Any:
        if values is None or len(values) <= index:
            return ""
        return float(values[index])

    def _capture(self, event: str, deduplicate: bool) -> bool:
        try:
            state = self.robot.get_robot_state("all")
        except Exception:
            return False
        if state is None:
            return False

        state_timestamp = float(getattr(state, "timestamp", 0.0) or 0.0)
        with self._lock:
            if deduplicate and self._last_state_timestamp == state_timestamp:
                return False
            if deduplicate:
                self._last_state_timestamp = state_timestamp
            action = self._action
            action_index = self._action_index
            action_count = self._action_count
            action_phase = self._action_phase

        now = time.time()
        run_status = int(getattr(state, "run_status", 0) or 0)
        angles = getattr(state, "angles", None)
        row = {field: "" for field in OSCILLATION_CSV_FIELDS}
        row.update(
            {
                "host_time_s": f"{now:.6f}",
                "elapsed_s": f"{max(now - self._started_at, 0.0):.6f}",
                "state_timestamp_s": f"{state_timestamp:.6f}",
                "trajectory": self.trajectory,
                "control_mode": self.control_mode,
                "event": event,
                "action_index": action_index or "",
                "action_count": action_count or "",
                "action_type": "arm" if isinstance(action, ArmAction) else "gripper" if action is not None else "",
                "action_label": action.label if action is not None else "",
                "action_phase": action_phase,
                "action_speed": action.speed if isinstance(action, ArmAction) else "",
                "target_gripper": action.value if isinstance(action, GripperAction) else "",
                "run_status": run_status,
                "gripper_abort": 1 if run_status & GRIPPER_ABORT_MASK else 0,
                "position_j7_gripper": float(getattr(state, "gripper", 0.0)),
            }
        )

        for index in range(6):
            row[f"position_j{index + 1}_rad"] = self._sequence_value(angles, index)
            if isinstance(action, ArmAction):
                row[f"target_j{index + 1}_rad"] = self._sequence_value(action.joints, index)

        series = (
            ("velocity_rad_s", getattr(state, "velocities", None)),
            ("torque_nm", getattr(state, "torques", None)),
            ("kp", getattr(state, "kps", None)),
            ("kd", getattr(state, "kds", None)),
            ("linear_velocity_rad_s", getattr(state, "linear_vels", None)),
            ("temperature_c", getattr(state, "temperatures", None)),
        )
        for prefix, values in series:
            for index in range(7):
                row[f"{prefix}_j{index + 1}"] = self._sequence_value(values, index)

        with self._lock:
            self._rows.append(row)
        return True


def default_oscillation_output_path(trajectory: str, control_mode: str = "pv") -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("logs") / f"trajectory{trajectory}_{str(control_mode).lower()}_oscillation_{timestamp}.csv"


TRAJECTORY_PROFILES = {
    "1": TrajectoryProfile(
        name="trajectory1",
        mid_pose=[0.0, -0.8, -1.1, 0.0, -0.7, 0.0],
        left_transfer_pose=[-1.57, -0.8, -1.1, 0.0, -0.7, 0.0],
        right_transfer_pose=[1.57, -0.8, -1.1, 0.0, -0.7, 0.0],
        grasp_depth_pose=[0.0, -1.33, -1.2, 0.0, -0.93, 0.0],
    ),
    "2": TrajectoryProfile(
        name="trajectory2",
        mid_pose=[0.0, -1.06, -1.46, 0.0, -1.05, 0.0],
        left_transfer_pose=[-1.57, -1.06, -1.46, 0.0, -1.05, 0.0],
        right_transfer_pose=[1.57, -1.06, -1.46, 0.0, -1.05, 0.0],
        grasp_depth_pose=[0.0, -1.28, -1.46, 0.0, -1.09, 0.0],
    ),
}

MID_POSE = TRAJECTORY_PROFILES["1"].mid_pose
LEFT_TRANSFER_POSE = TRAJECTORY_PROFILES["1"].left_transfer_pose
RIGHT_TRANSFER_POSE = TRAJECTORY_PROFILES["1"].right_transfer_pose
GRASP_DEPTH_POSE = TRAJECTORY_PROFILES["1"].grasp_depth_pose
LEFT_GRASP_POSE = [-1.57] + GRASP_DEPTH_POSE[1:]
RIGHT_GRASP_POSE = [1.57] + GRASP_DEPTH_POSE[1:]
GRASP_POSE = LEFT_GRASP_POSE


def adapt_grasp_pose_for_transfer(profile: TrajectoryProfile, transfer_pose: list[float]) -> list[float]:
    grasp_pose = list(profile.grasp_depth_pose)
    grasp_pose[0] = transfer_pose[0]
    return grasp_pose


def format_feedback_values(values, precision: int) -> str:
    if values is None:
        return "N/A"
    return "[" + ", ".join(f"{float(value):.{precision}f}" for value in values) + "]"


def format_feedback_snapshot(state) -> str:
    torques = getattr(state, "torques", None) if state is not None else None
    temperatures = getattr(state, "temperatures", None) if state is not None else None
    return "\n".join(
        [
            f"torques(N*m): {format_feedback_values(torques, 3)}",
            f"temperatures(C): {format_feedback_values(temperatures, 1)}",
        ]
    )


def print_feedback_snapshot(robot) -> None:
    try:
        state = robot.get_robot_state("all")
    except Exception:
        state = None
    print(format_feedback_snapshot(state), flush=True)


def feedback_loop(robot, stop_event: threading.Event, interval_s: float) -> None:
    interval_s = max(float(interval_s), 0.1)
    while not stop_event.is_set():
        print_feedback_snapshot(robot)
        stop_event.wait(interval_s)


def select_gripper_speed(label: str, args) -> float:
    fallback_speed = getattr(args, "gripper_speed", DEFAULT_GRIPPER_SPEED)
    if label == "夹住":
        return getattr(args, "gripper_close_speed", fallback_speed)
    if label == "张开":
        return getattr(args, "gripper_open_speed", fallback_speed)
    return fallback_speed


def read_current_gripper_value(robot, fallback: float) -> float:
    try:
        state = robot.get_robot_state("joint_gripper")
    except Exception:
        return fallback
    if isinstance(state, dict) and state.get("gripper") is not None:
        return float(state["gripper"])
    gripper = getattr(state, "gripper", None)
    return fallback if gripper is None else float(gripper)


def build_gripper_ramp_values(
    *,
    start: float,
    target: float,
    speed: float,
    frequency_hz: float,
    max_duration_s: float,
) -> list[float]:
    start = float(start)
    target = float(target)
    distance = abs(target - start)
    if distance < 1e-6:
        return [target]

    frequency_hz = max(float(frequency_hz), 1.0)
    max_duration_s = max(float(max_duration_s), 0.1)
    firmware_speed = max(float(speed), 0.1) * 10.0 / 400.0
    units_per_second = firmware_speed * 1000.0 / GRIPPER_FULL_RANGE_RAD
    duration_s = min(max_duration_s, max(0.2, distance / units_per_second))
    steps = max(1, int(ceil(duration_s * frequency_hz)))

    return [
        start + (target - start) * step / steps
        for step in range(1, steps + 1)
    ]


def build_action_plan(
    *,
    profile: TrajectoryProfile,
    fast_speed: float,
    slow_speed: float,
    free_speed: float,
    cycles: int,
    gripper_open: float,
    gripper_close: float,
) -> list[Action]:
    if cycles < 0:
        raise ValueError("cycles must be greater than or equal to 0")

    actions: list[Action] = [
        ArmAction("快速 1/3: 回到零位", HOME_POSE, fast_speed),
        ArmAction("快速 2/3: 到中间位", profile.mid_pose, fast_speed),
        ArmAction("快速 3/3: 到左侧", profile.left_transfer_pose, fast_speed),
        GripperAction("张开", gripper_open),
        ArmAction("慢速 1/2: 下探抓取位", adapt_grasp_pose_for_transfer(profile, profile.left_transfer_pose), slow_speed),
        GripperAction("夹住", gripper_close),
        ArmAction("慢速 2/2: 抬起到左侧", profile.left_transfer_pose, slow_speed),
        ArmAction("自由速度: 到右侧", profile.right_transfer_pose, free_speed),
    ]
    final_transfer_pose = profile.right_transfer_pose

    for index in range(cycles):
        cycle = index + 1
        actions.extend(
            [
                ArmAction(f"自由速度循环 {cycle}/{cycles}: 左侧", profile.left_transfer_pose, free_speed),
                ArmAction(f"自由速度循环 {cycle}/{cycles}: 右侧", profile.right_transfer_pose, free_speed),
            ]
        )
        final_transfer_pose = profile.right_transfer_pose

    actions.extend(
        [
            ArmAction("慢速: 下探放置位", adapt_grasp_pose_for_transfer(profile, final_transfer_pose), slow_speed),
            GripperAction("张开", gripper_open),
            ArmAction("快速 1/3: 回到右侧", profile.right_transfer_pose, fast_speed),
            ArmAction("快速 2/3: 回到中间位", profile.mid_pose, fast_speed),
            ArmAction("快速 3/3: 回到零位", HOME_POSE, fast_speed),
        ]
    )
    return actions


def arm_arrival_tolerance(robot) -> float:
    """Match the existing SDK arrival tolerance for the active arm mode."""
    control_mode = getattr(getattr(robot, "control_mode", None), "value", "pv")
    return (
        MIT_ARM_ARRIVAL_TOLERANCE_RAD
        if str(control_mode).lower() == "mit"
        else PV_ARM_ARRIVAL_TOLERANCE_RAD
    )


def wait_for_arm_target(
    robot,
    joints: list[float],
    timeout: float = ARM_ARRIVAL_TIMEOUT_S,
    tolerance: Optional[float] = None,
) -> bool:
    """Wait for J1-J6 without considering the gripper position."""
    effective_tolerance = (
        arm_arrival_tolerance(robot)
        if tolerance is None
        else float(tolerance)
    )
    deadline = time.perf_counter() + float(timeout)
    while time.perf_counter() < deadline:
        feedback = robot.get_robot_state("joint")
        if feedback is not None and len(feedback) >= 6:
            if all(
                abs(float(feedback[index]) - float(joints[index])) < effective_tolerance
                for index in range(6)
            ):
                return True
        time.sleep(ARM_ARRIVAL_POLL_INTERVAL_S)
    return False


def send_arm_target(
    robot,
    joints: list[float],
    speed: float,
    gripper_state: Optional[GripperCommandState] = None,
    arm_state: Optional[ArmCommandState] = None,
    isolate_pv_stationary_joints: bool = False,
) -> bool:
    """Send one arm target while preserving the last explicit J7 command."""
    command_speed: Union[float, list[float]] = float(speed)
    control_mode = getattr(getattr(robot, "control_mode", None), "value", "pv")
    if (
        isolate_pv_stationary_joints
        and str(control_mode).lower() == "pv"
        and arm_state is not None
        and arm_state.target is not None
    ):
        command_speed = [
            0.0
            if abs(float(target) - float(previous)) <= PV_STATIONARY_TARGET_EPSILON_RAD
            else float(speed)
            for target, previous in zip(joints, arm_state.target)
        ]

    if gripper_state is None or gripper_state.target is None:
        command_sent = robot.set_robot_state(
            target_joints=joints,
            joint_format="rad",
            speed=command_speed,
            wait_for_completion=True,
        )
        result = bool(command_sent)
    else:
        gripper_speed = (
            float(gripper_state.speed)
            if gripper_state.speed is not None
            else DEFAULT_GRIPPER_SPEED
        )
        command_sent = robot.set_robot_state(
            target_joints=joints,
            gripper_value=float(gripper_state.target),
            joint_format="rad",
            speed=command_speed,
            gripper_speed=gripper_speed,
            wait_for_completion=False,
            use_interpolation=True,
        )
        result = bool(command_sent) and wait_for_arm_target(robot, joints)

    if arm_state is not None:
        arm_state.target = [float(joint) for joint in joints]
    return result


def move_arm(
    robot,
    joints: list[float],
    speed: float,
    label: str,
    args,
    gripper_state: Optional[GripperCommandState] = None,
    arm_state: Optional[ArmCommandState] = None,
) -> bool:
    beauty_print(
        f"{label}: joints(rad)={beauty_print_array(joints, precision=2)}, speed={speed}",
        type="info",
    )
    stop_feedback = threading.Event()
    feedback_thread = None
    if not getattr(args, "no_feedback", False):
        feedback_thread = threading.Thread(
            target=feedback_loop,
            args=(robot, stop_feedback, getattr(args, "feedback_interval", 1.0)),
            daemon=True,
        )
        feedback_thread.start()
    try:
        ok = send_arm_target(
            robot,
            joints,
            speed,
            gripper_state=gripper_state,
            arm_state=arm_state,
            isolate_pv_stationary_joints=getattr(
                args,
                "pv_stationary_speed_isolation",
                False,
            ),
        )
    finally:
        stop_feedback.set()
        if feedback_thread is not None:
            feedback_thread.join(timeout=1.0)
    if not ok:
        beauty_print(f"{label} 执行失败", type="warning")
        return False
    time.sleep(args.pause)
    return True


def move_gripper(
    robot,
    value: float,
    label: str,
    args,
    gripper_state: Optional[GripperCommandState] = None,
    arm_state: Optional[ArmCommandState] = None,
) -> bool:
    arm_target = (
        list(arm_state.target)
        if arm_state is not None and arm_state.target is not None
        else None
    )
    arm_target_kwargs = (
        {
            "target_joints": arm_target,
            "joint_format": "rad",
            "use_interpolation": True,
        }
        if arm_target is not None
        else {"target_joints": None}
    )
    gripper_speed = select_gripper_speed(label, args)
    if not getattr(args, "no_gripper_ramp", True):
        fallback = (
            getattr(args, "gripper_open", 1000.0)
            if value < 500.0
            else getattr(args, "gripper_close", 0.0)
        )
        start = read_current_gripper_value(robot, fallback=fallback)
        frequency_hz = getattr(args, "gripper_ramp_frequency", DEFAULT_GRIPPER_RAMP_FREQUENCY)
        values = build_gripper_ramp_values(
            start=start,
            target=value,
            speed=gripper_speed,
            frequency_hz=frequency_hz,
            max_duration_s=getattr(args, "gripper_max_ramp_duration", DEFAULT_GRIPPER_MAX_RAMP_DURATION),
        )
        beauty_print(
            f"{label}: gripper {start:.1f}->{value}, gripper_speed={gripper_speed}, ramp_frames={len(values)}",
            type="info",
        )
        ok = True
        interval_s = 1.0 / max(float(frequency_hz), 1.0)
        for index, gripper_value in enumerate(values):
            if gripper_state is not None:
                gripper_state.target = float(gripper_value)
                gripper_state.speed = float(gripper_speed)
            frame_ok = robot.set_robot_state(
                gripper_value=gripper_value,
                gripper_speed=gripper_speed,
                wait_for_completion=False,
                **arm_target_kwargs,
            )
            if not frame_ok:
                ok = False
            if index < len(values) - 1:
                time.sleep(interval_s)
        if not ok:
            beauty_print(f"{label} 部分 ramp 命令返回失败，继续后续动作", type="warning")
        time.sleep(args.pause)
        return True

    beauty_print(
        f"{label}: gripper={value}, gripper_speed={gripper_speed}",
        type="info",
    )
    if gripper_state is not None:
        gripper_state.target = float(value)
        gripper_state.speed = float(gripper_speed)
    ok = robot.set_robot_state(
        gripper_value=value,
        gripper_speed=gripper_speed,
        wait_for_completion=False,
        **arm_target_kwargs,
    )
    if not ok:
        beauty_print(f"{label} 命令已发送但 SDK 返回失败，继续后续动作", type="warning")
    time.sleep(args.pause)
    return True


def return_home_after_interrupt(
    robot,
    args,
    gripper_state: Optional[GripperCommandState] = None,
    arm_state: Optional[ArmCommandState] = None,
) -> bool:
    beauty_print("收到 Ctrl+C，尝试回到 6 关节全 0 位置...", type="warning")
    try:
        ok = send_arm_target(
            robot,
            HOME_POSE,
            speed=getattr(args, "fast_speed", 40.0),
            gripper_state=gripper_state,
            arm_state=arm_state,
            isolate_pv_stationary_joints=getattr(
                args,
                "pv_stationary_speed_isolation",
                False,
            ),
        )
    except Exception as exc:
        beauty_print(f"中断回零失败: {exc}", type="warning")
        return False

    beauty_print("已回到全 0 位置" if ok else "中断回零未确认到位", type="success" if ok else "warning")
    return bool(ok)


def execute_actions(
    robot,
    actions: list[Action],
    args,
    recorder: Optional[OscillationRecorder] = None,
    gripper_state: Optional[GripperCommandState] = None,
    arm_state: Optional[ArmCommandState] = None,
) -> bool:
    if gripper_state is None:
        gripper_state = GripperCommandState()
    if arm_state is None:
        arm_state = ArmCommandState()
    action_count = len(actions)
    for action_index, action in enumerate(actions, start=1):
        if recorder is not None:
            recorder.mark_action(action_index, action_count, action, "started")
        try:
            if isinstance(action, ArmAction):
                ok = move_arm(
                    robot,
                    action.joints,
                    action.speed,
                    action.label,
                    args,
                    gripper_state=gripper_state,
                    arm_state=arm_state,
                )
            else:
                if action.label == "张开":
                    time.sleep(args.gripper_settle_delay)
                ok = move_gripper(
                    robot,
                    action.value,
                    action.label,
                    args,
                    gripper_state=gripper_state,
                    arm_state=arm_state,
                )
                if ok and action.label == "夹住":
                    time.sleep(args.gripper_settle_delay)
        except KeyboardInterrupt:
            if recorder is not None:
                recorder.mark_action(action_index, action_count, action, "interrupted")
            raise
        except Exception:
            if recorder is not None:
                recorder.mark_action(action_index, action_count, action, "failed")
            raise

        if recorder is not None:
            recorder.mark_action(action_index, action_count, action, "completed" if ok else "failed")
        if not ok:
            return False
    return True


def ensure_control_mode(robot, requested_mode: str, prompt=input) -> bool:
    """Ensure J1-J6 use the requested mode; J7 remains firmware-managed MIT."""
    requested_mode = str(requested_mode).lower()
    if robot.control_mode.value.lower() == requested_mode:
        return True

    prompt(
        f"Press Enter to switch J1-J6 to {requested_mode.upper()} mode; "
        "the arm will be disabled briefly..."
    )
    if not robot.switch_mode(requested_mode):
        return False
    return robot.control_mode.value.lower() == requested_mode


def handle_action_interrupt(
    robot,
    args,
    gripper_state: Optional[GripperCommandState] = None,
    arm_state: Optional[ArmCommandState] = None,
) -> bool:
    """Skip recovery motion during diagnostics; retain legacy home otherwise."""
    if getattr(args, "oscillation_diagnostics", False):
        return False
    if arm_state is None:
        return return_home_after_interrupt(
            robot,
            args,
            gripper_state=gripper_state,
        )
    return return_home_after_interrupt(
        robot,
        args,
        gripper_state=gripper_state,
        arm_state=arm_state,
    )


def cleanup_session(robot, recorder: Optional[OscillationRecorder] = None) -> Optional[Path]:
    """Stop sampling, disconnect immediately, then perform deferred CSV I/O."""
    if recorder is not None:
        recorder.stop()
    robot.disconnect()
    if recorder is not None:
        return recorder.write_csv()
    return None


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Alicia-M left/right grasp demo.")
    add_port_argument(parser)
    parser.add_argument(
        "--trajectory",
        choices=sorted(TRAJECTORY_PROFILES.keys()),
        required=True,
        help="Trajectory profile to run: 1 or 2.",
    )
    parser.add_argument(
        "--control-mode",
        choices=("pv", "mit"),
        default="pv",
        help="J1-J6 control mode; default pv. J7 remains ForceGrasp MIT.",
    )
    parser.add_argument(
        "--pv-stationary-speed-isolation",
        action="store_true",
        help=(
            "Diagnostic: in PV mode, command zero speed for J1-J6 targets "
            "unchanged from the preceding arm command."
        ),
    )
    parser.add_argument("--fast-speed", type=float, default=40.0, help="Fast motion speed; default 40.")
    parser.add_argument("--slow-speed", type=float, default=15.0, help="Slow motion speed; default 15.")
    parser.add_argument("--free-speed", type=float, default=30.0, help="Free motion speed; default 30.")
    parser.add_argument("--cycles", type=int, default=3, help="Left/right free-speed cycles; default 3.")
    parser.add_argument("--gripper-open", type=float, default=1000.0, help="Open gripper value; default 1000.")
    parser.add_argument("--gripper-close", type=float, default=0.0, help="Closed gripper value; default 0.")
    parser.add_argument(
        "--gripper-speed",
        type=float,
        default=None,
        help="Legacy gripper speed override for both open and close; default 100.",
    )
    parser.add_argument(
        "--gripper-close-speed",
        type=float,
        default=None,
        help="Close gripper speed; default 40.",
    )
    parser.add_argument(
        "--gripper-open-speed",
        type=float,
        default=None,
        help="Open gripper speed; default 100.",
    )
    parser.add_argument(
        "--no-gripper-ramp",
        action="store_true",
        help="Disable demo-side gripper ramp and send a single gripper target.",
    )
    parser.add_argument(
        "--gripper-ramp-frequency",
        type=float,
        default=DEFAULT_GRIPPER_RAMP_FREQUENCY,
        help="Gripper ramp command frequency; default 25Hz.",
    )
    parser.add_argument(
        "--gripper-max-ramp-duration",
        type=float,
        default=DEFAULT_GRIPPER_MAX_RAMP_DURATION,
        help="Maximum gripper ramp duration; default 6.0s.",
    )
    parser.add_argument("--pause", type=float, default=0.5, help="Pause after each completed action; default 0.5s.")
    parser.add_argument(
        "--gripper-settle-delay",
        type=float,
        default=1.0,
        help="Extra delay before opening and after closing the gripper; default 1.0s.",
    )
    parser.add_argument(
        "--feedback-interval",
        type=float,
        default=1.0,
        help="Torque/temperature feedback print interval during arm motion; default 1.0s.",
    )
    parser.add_argument("--no-feedback", action="store_true", help="Disable torque/temperature feedback printing.")
    parser.add_argument(
        "--oscillation-diagnostics",
        action="store_true",
        help="Record cached joint state at 50Hz for oscillation diagnosis.",
    )
    parser.add_argument(
        "--oscillation-output",
        type=str,
        default="",
        help="Oscillation CSV path; default includes trajectory and control mode.",
    )
    args = parser.parse_args(argv)

    legacy_gripper_speed = args.gripper_speed
    args.gripper_speed = (
        DEFAULT_GRIPPER_SPEED
        if legacy_gripper_speed is None
        else legacy_gripper_speed
    )
    args.gripper_close_speed = (
        args.gripper_close_speed
        if args.gripper_close_speed is not None
        else (
            legacy_gripper_speed
            if legacy_gripper_speed is not None
            else DEFAULT_GRIPPER_CLOSE_SPEED
        )
    )
    args.gripper_open_speed = (
        args.gripper_open_speed
        if args.gripper_open_speed is not None
        else (
            legacy_gripper_speed
            if legacy_gripper_speed is not None
            else DEFAULT_GRIPPER_OPEN_SPEED
        )
    )
    return args


def main() -> None:
    beauty_print("Demo: 左右抓取", type="module")
    args = parse_args()
    profile = TRAJECTORY_PROFILES[args.trajectory]
    beauty_print(f"使用轨迹: {args.trajectory}", type="info")
    beauty_print(f"J1-J6 控制模式: {args.control_mode.upper()}", type="info")
    actions = build_action_plan(
        profile=profile,
        fast_speed=args.fast_speed,
        slow_speed=args.slow_speed,
        free_speed=args.free_speed,
        cycles=args.cycles,
        gripper_open=args.gripper_open,
        gripper_close=args.gripper_close,
    )

    robot = alicia_m_sdk.create_robot(port=args.port)
    recorder: Optional[OscillationRecorder] = None
    gripper_state = GripperCommandState()
    arm_state = ArmCommandState()

    try:
        beauty_print(f"机器人连接成功（当前 {robot.control_mode.value.upper()} 模式）", type="success")
        if not ensure_control_mode(robot, args.control_mode):
            raise RuntimeError(f"J1-J6 未能切换到 {args.control_mode.upper()} 模式")
        beauty_print(f"J1-J6 已处于 {args.control_mode.upper()} 模式", type="success")

        if not args.no_feedback or args.oscillation_diagnostics:
            try:
                robot.set_extended_polling(True)
                beauty_print("已启用扩展状态轮询（速度、力矩、增益、插补速度和温度）", type="info")
            except Exception as exc:
                beauty_print(f"启用扩展状态轮询失败，将继续运行: {exc}", type="warning")

        if args.oscillation_diagnostics:
            output_path = (
                Path(args.oscillation_output).expanduser()
                if args.oscillation_output
                else default_oscillation_output_path(args.trajectory, args.control_mode)
            )
            recorder = OscillationRecorder(
                robot,
                args.trajectory,
                output_path,
                control_mode=args.control_mode,
            )
            recorder.start()
            beauty_print(f"震荡诊断采集已启动: {output_path}", type="info")

        ok = execute_actions(
            robot,
            actions,
            args,
            recorder=recorder,
            gripper_state=gripper_state,
            arm_state=arm_state,
        )
        beauty_print("左右抓取动作执行完成" if ok else "左右抓取动作中断", type="success" if ok else "warning")
    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
        if args.oscillation_diagnostics:
            beauty_print("诊断模式将立即断开，不执行回零或开爪", type="warning")
        handle_action_interrupt(
            robot,
            args,
            gripper_state=gripper_state,
            arm_state=arm_state,
        )
    finally:
        output_path = cleanup_session(robot, recorder)
        beauty_print("已断开连接", type="info")
        if output_path is not None and recorder is not None:
            beauty_print(f"震荡诊断 CSV 已保存: {output_path} ({recorder.row_count} rows)", type="info")


if __name__ == "__main__":
    main()
