"""@file sop.py
@brief Alicia-M 出货 SOP 测试流程与报告生成模块。

@details
本模块用于产线出货准入测试，核心目标是用保守、可追溯、可自动判定的
流程确认机械臂是否满足交付标准。测试流程会先执行配置安全检查、设备识别、
零点准确性检查和固件自检，再进入任何会产生运动的步骤。

设计原则：
- SOP 程序只负责判定是否合格，不自动写入零点或修改永久配置。
- 默认运动范围和速度保持保守，防止产线误操作造成风险。
- 所有失败都写入结构化报告，便于追溯设备、操作员和失败原因。
- 老化测试作为可选 profile，与默认出货 quick 流程分离。
"""

from __future__ import annotations

import csv
import json
import math
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Iterable, Optional

from . import create_robot
from .diagnostics import supports_diagnostic


PASS = "PASS"
FAIL = "FAIL"


@dataclass
class SopConfig:
    """@brief SOP 测试配置。

    @details
    该配置集中描述产线运行 SOP 所需的全部参数，包括串口、profile、
    零点标准值、容差、安全上限、老化循环参数和报告输出目录。
    所有会影响安全的参数都会在 `_validate_safety_config()` 中先做硬校验；
    校验失败时不会连接或驱动机械臂运动。
    """

    port: str = ""
    profile: str = "quick"
    serial: str = ""
    operator: str = ""
    output_dir: str = "logs/sop"
    speed: float = 15.0
    max_safe_speed: float = 40.0
    zero_joints_deg: list[float] = field(default_factory=lambda: [0.0] * 6)
    zero_tolerances_deg: list[float] = field(default_factory=lambda: [1.0] * 6)
    zero_gripper: Optional[float] = 0.0
    zero_gripper_tolerance: float = 80.0
    motion_delta_deg: list[float] = field(
        default_factory=lambda: [3.0, -3.0, 3.0, 0.0, 3.0, 0.0]
    )
    max_motion_delta_deg: float = 10.0
    motion_tolerance_deg: float = 2.0
    gripper_targets: list[float] = field(default_factory=lambda: [1000.0, 0.0, 500.0])
    gripper_tolerance: float = 120.0
    diagnostic_timeout: float = 3.0
    state_timeout: float = 3.0
    aging_cycles: Optional[int] = None
    aging_duration_min: Optional[float] = None
    aging_interval_s: float = 0.5
    max_temperature_c: Optional[float] = 75.0
    max_abs_torque: Optional[float] = None
    require_confirmation: bool = False
    disable_on_exit: bool = True


@dataclass
class SopStepResult:
    """@brief 单个 SOP 步骤的执行结果。

    @details
    每个步骤都独立记录 PASS/FAIL、耗时、错误信息和结构化指标。
    即使步骤失败，也尽量保留可诊断数据，例如零点误差、故障状态、
    温度/力矩峰值等，方便产线复核。
    """

    name: str
    status: str
    duration_s: float
    error: str = ""
    metrics: dict[str, Any] = field(default_factory=dict)


@dataclass
class SopReport:
    """@brief 一次完整 SOP 测试的汇总报告。

    @details
    报告会被写出为 JSON、CSV 和 Markdown 三种格式：
    - JSON 用于程序化归档和后续数据分析。
    - CSV 用于产线统计，每台设备一行摘要。
    - Markdown 用于人工查看 PASS/FAIL 和失败原因。
    """

    status: str
    profile: str
    started_at: str
    finished_at: str
    duration_s: float
    serial: str = ""
    operator: str = ""
    port: str = ""
    device: dict[str, Any] = field(default_factory=dict)
    steps: list[SopStepResult] = field(default_factory=list)
    report_paths: dict[str, str] = field(default_factory=dict)


class SopStepError(AssertionError):
    """@brief 携带结构化指标的步骤失败异常。

    @param message 面向操作员或日志的失败说明。
    @param metrics 该步骤失败时需要写入报告的结构化诊断数据。
    """

    def __init__(self, message: str, metrics: dict[str, Any]):
        super().__init__(message)
        self.metrics = metrics


def _now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _safe_value(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, (list, tuple)):
        return [_safe_value(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _safe_value(val) for key, val in value.items()}
    if hasattr(value, "__dict__"):
        return _safe_value(vars(value))
    return str(value)


def _angles_to_deg(values: Iterable[float]) -> list[float]:
    return [math.degrees(float(value)) for value in values]


def _max_abs(values: Iterable[float]) -> float:
    items = [abs(float(value)) for value in values]
    return max(items) if items else 0.0


class SopRunner:
    """@brief Alicia-M 出货 SOP 编排器。

    @details
    `SopRunner` 负责按照 profile 编排全部测试步骤，并保证失败后短路：
    一旦零点、自检、安全门禁或运动步骤失败，就不再继续后续危险动作。
    退出阶段会默认调用 `disable_robot()`，尽量让设备回到安全状态。

    @param config SOP 测试配置。
    @param robot_factory 机械臂对象工厂，默认使用 `alicia_m_sdk.create_robot`。
                         单元测试可传入 mock factory，避免依赖真机。
    """

    def __init__(
        self,
        config: SopConfig,
        robot_factory: Callable[..., Any] = create_robot,
    ):
        self.config = config
        self.robot_factory = robot_factory
        self.robot = None
        self.steps: list[SopStepResult] = []
        self.device: dict[str, Any] = {}
        self._failed = False

    def run(self) -> SopReport:
        """@brief 执行 SOP 测试并生成报告。

        @return SopReport 完整测试报告，包含最终 PASS/FAIL、设备信息、
                步骤结果和报告文件路径。
        """
        started_at = _now_iso()
        start = time.perf_counter()
        try:
            self._validate_safety_config()
            if not self._failed:
                self._connect()
            if not self._failed:
                if self.config.profile == "aging":
                    self._run_aging()
                else:
                    self._run_quick_checks()
                    if self.config.profile == "full" and not self._failed:
                        self._run_full_extension()
        except KeyboardInterrupt as exc:
            self._failed = True
            self._add_step("interrupted", FAIL, 0.0, str(exc) or "Interrupted by user")
        except Exception as exc:
            self._failed = True
            self._add_step("fatal_error", FAIL, 0.0, str(exc))
        finally:
            self._safe_shutdown()
            self._disconnect()

        finished_at = _now_iso()
        report = SopReport(
            status=FAIL if self._failed or any(step.status == FAIL for step in self.steps) else PASS,
            profile=self.config.profile,
            started_at=started_at,
            finished_at=finished_at,
            duration_s=round(time.perf_counter() - start, 3),
            serial=self.config.serial or str(self.device.get("serial_number", "")),
            operator=self.config.operator,
            port=str(self.device.get("connected_port", self.config.port)),
            device=self.device,
            steps=self.steps,
        )
        report.report_paths = write_report(report, self.config.output_dir)
        return report

    def _run_quick_checks(self) -> None:
        """@brief 执行默认出货 quick 流程。

        @details
        quick 流程代表产线默认准入测试：零点、自检、安全门禁、
        使能失能、小行程运动和夹爪动作。任何 fatal 步骤失败都会短路。
        """
        for step in (
            self._check_zero,
            self._read_basic_state,
            self._run_diagnostic,
            self._pre_motion_safety_gate,
            self._check_enable_disable,
            self._check_motion,
            self._check_gripper,
        ):
            if self._failed:
                break
            step()

    def _run_full_extension(self) -> None:
        """@brief 执行 full profile 的附加检查。

        @details
        当前 full profile 在 quick 通过后额外执行一次安全回零。
        后续如果需要加入更完整的运动覆盖，应继续保持保守速度和小范围原则。
        """
        self._run_step("full_return_home", lambda: self.robot.go_home(speed=self.config.speed))

    def _validate_safety_config(self) -> None:
        """@brief 校验所有影响安全的 SOP 配置。

        @details
        该步骤是整个 SOP 的第一道门。它会检查速度上限、小行程幅度、
        夹爪目标范围、老化参数和列表长度。配置不安全时不会连接机械臂，
        防止错误参数被带入真实硬件。
        """
        def action() -> dict[str, Any]:
            failures = []
            if self.config.profile not in {"quick", "full", "aging"}:
                failures.append(f"unsupported profile: {self.config.profile}")
            if self.config.speed <= 0 or self.config.speed > self.config.max_safe_speed:
                failures.append(
                    f"speed {self.config.speed} exceeds safe range (0, {self.config.max_safe_speed}]"
                )
            if len(self.config.zero_joints_deg) != 6:
                failures.append("zero_joints_deg must contain 6 values")
            if len(self.config.zero_tolerances_deg) != 6:
                failures.append("zero_tolerances_deg must contain 6 values")
            if len(self.config.motion_delta_deg) != 6:
                failures.append("motion_delta_deg must contain 6 values")
            if any(abs(delta) > self.config.max_motion_delta_deg for delta in self.config.motion_delta_deg):
                failures.append(
                    f"motion delta exceeds {self.config.max_motion_delta_deg} deg limit"
                )
            bad_gripper_targets = [
                target for target in self.config.gripper_targets if target < 0 or target > 1000
            ]
            if bad_gripper_targets:
                failures.append(f"gripper targets out of [0, 1000]: {bad_gripper_targets}")
            if self.config.aging_cycles is not None and self.config.aging_cycles <= 0:
                failures.append("aging_cycles must be positive")
            if self.config.aging_duration_min is not None and self.config.aging_duration_min <= 0:
                failures.append("aging_duration_min must be positive")

            metrics = {
                "speed": self.config.speed,
                "max_safe_speed": self.config.max_safe_speed,
                "motion_delta_deg": self.config.motion_delta_deg,
                "max_motion_delta_deg": self.config.max_motion_delta_deg,
                "gripper_targets": self.config.gripper_targets,
                "require_confirmation": self.config.require_confirmation,
                "disable_on_exit": self.config.disable_on_exit,
                "failures": failures,
            }
            if failures:
                raise SopStepError("Unsafe SOP configuration", metrics)
            return metrics

        self._run_step("safety_config_check", action, fatal=True)

    def _connect(self) -> None:
        """@brief 连接机械臂并记录设备识别信息。

        @details
        连接后读取固件版本、硬件版本、序列号和实际串口。报告中的
        serial 优先使用 CLI 传入的生产序列号；如果未传入，则使用固件 SN。
        """
        def action() -> dict[str, Any]:
            self.robot = self.robot_factory(
                port=self.config.port,
                sync_control_mode=False,
                auto_connect=True,
            )
            version = self.robot.get_robot_state("version")
            self.device = _safe_value(version) if version is not None else {}
            self.device["connected_port"] = getattr(self.robot, "connected_port", self.config.port)
            self.device["sdk_profile"] = self.config.profile
            return self.device

        self._run_step("connect_and_identify", action, fatal=True)

    def _disconnect(self) -> None:
        """@brief 断开机械臂连接。

        @details
        断开只做资源释放，安全失能由 `_safe_shutdown()` 负责。
        这里吞掉断开异常，避免掩盖前面真正的 SOP 失败原因。
        """
        if self.robot is None:
            return
        try:
            self.robot.disconnect()
        except Exception:
            pass

    def _check_zero(self) -> None:
        """@brief 检查当前零点姿态是否符合固定工装标准。

        @details
        该步骤读取当前 6 轴角度和夹爪值，与配置中的标准零点和容差比较。
        任一关节或夹爪超差都会直接 FAIL，并阻止后续运动测试。
        SOP 不会自动调用 `set_zero_position()`，避免在错误装夹或错误姿态下
        写入错误零点。
        """
        def action() -> dict[str, Any]:
            state = self._require_state()
            actual_deg = _angles_to_deg(state.angles)
            expected = self.config.zero_joints_deg
            tolerances = self.config.zero_tolerances_deg
            errors = [actual - target for actual, target in zip(actual_deg, expected)]
            failures = [
                {
                    "joint": index,
                    "actual_deg": round(actual_deg[index], 4),
                    "expected_deg": round(expected[index], 4),
                    "error_deg": round(errors[index], 4),
                    "tolerance_deg": tolerances[index],
                }
                for index in range(6)
                if abs(errors[index]) > tolerances[index]
            ]
            gripper_error = None
            if self.config.zero_gripper is not None:
                gripper_error = float(state.gripper) - float(self.config.zero_gripper)
                if abs(gripper_error) > self.config.zero_gripper_tolerance:
                    failures.append(
                        {
                            "axis": "gripper",
                            "actual": state.gripper,
                            "expected": self.config.zero_gripper,
                            "error": round(gripper_error, 4),
                            "tolerance": self.config.zero_gripper_tolerance,
                        }
                    )
            metrics = {
                "actual_joints_deg": [round(value, 4) for value in actual_deg],
                "expected_joints_deg": expected,
                "joint_errors_deg": [round(value, 4) for value in errors],
                "zero_tolerances_deg": tolerances,
                "actual_gripper": state.gripper,
                "expected_gripper": self.config.zero_gripper,
                "gripper_error": round(gripper_error, 4) if gripper_error is not None else None,
                "failures": failures,
            }
            if failures:
                raise SopStepError("Zero position check failed", metrics)
            return metrics

        self._run_step("zero_position_check", action, fatal=True)

    def _read_basic_state(self) -> None:
        """@brief 读取并记录基础状态快照。

        @details
        记录关节角、夹爪、速度、力矩、温度、运行状态和控制模式。
        该步骤主要用于出货报告留档，也为后续异常分析提供初始快照。
        """
        def action() -> dict[str, Any]:
            state = self._require_state()
            return {
                "joints_deg": [round(value, 4) for value in _angles_to_deg(state.angles)],
                "gripper": state.gripper,
                "velocities": _safe_value(state.velocities),
                "torques": _safe_value(state.torques),
                "temperatures": _safe_value(state.temperatures),
                "run_status": state.run_status,
                "status": _safe_value(self.robot.get_robot_state("status")),
                "control_mode": _safe_value(self.robot.get_robot_state("control_mode")),
            }

        self._run_step("basic_state_read", action, fatal=True)

    def _run_diagnostic(self) -> None:
        """@brief 执行固件自检并判定结果。

        @details
        固件版本必须支持自检；否则按不可执行处理并 FAIL。
        自检要求通信 bitmap 完整、电机状态处于可接受状态、控制模式可读。
        """
        def action() -> dict[str, Any]:
            version = self.device.get("firmware_version")
            if not supports_diagnostic(version):
                raise AssertionError(f"Firmware {version or 'unknown'} does not support diagnostic")
            result = self.robot.run_diagnostic(timeout=self.config.diagnostic_timeout)
            metrics = _safe_value(result)
            failures = []
            if not result.ok:
                failures.append("diagnostic_response_not_ok")
            for snapshot in result.snapshots:
                if snapshot.comm_bitmap != 0x7F:
                    failures.append(f"{snapshot.arm_name}: comm_bitmap=0x{snapshot.comm_bitmap:02X}")
                bad_states = [
                    {"motor": index, "state": state}
                    for index, state in enumerate(snapshot.motor_states)
                    if state not in {0x0, 0x1}
                ]
                if bad_states:
                    failures.append(f"{snapshot.arm_name}: bad_motor_states={bad_states}")
                bad_modes = [
                    {"motor": index, "mode": mode}
                    for index, mode in enumerate(snapshot.control_modes)
                    if mode == 0x00
                ]
                if bad_modes:
                    failures.append(f"{snapshot.arm_name}: unread_control_modes={bad_modes}")
            metrics["failures"] = failures
            if failures:
                raise SopStepError("; ".join(failures), metrics)
            return metrics

        self._run_step("firmware_diagnostic", action, fatal=True)

    def _pre_motion_safety_gate(self) -> None:
        """@brief 运动前安全门禁。

        @details
        这是所有运动步骤前的最后一道保护。它会检查 RobotStatus 电机错误位、
        JointState 原始运行状态错误位、温度上限和可选力矩上限。
        如果门禁失败，程序不会执行使能和运动。
        """
        def action() -> dict[str, Any]:
            state = self._require_state()
            status = self.robot.get_robot_state("status")
            failures = []
            if getattr(status, "has_motor_error", False):
                failures.append("robot_status.has_motor_error is set")
            if state.run_status & 0x80:
                failures.append(f"joint_state.run_status motor error bit set: 0x{state.run_status:02X}")
            max_temp = None
            if state.temperatures:
                max_temp = max(float(value) for value in state.temperatures)
                if self.config.max_temperature_c is not None and max_temp > self.config.max_temperature_c:
                    failures.append(
                        f"temperature {max_temp:.2f}C exceeds {self.config.max_temperature_c:.2f}C"
                    )
            max_torque = None
            if state.torques:
                max_torque = _max_abs(state.torques)
                if self.config.max_abs_torque is not None and max_torque > self.config.max_abs_torque:
                    failures.append(
                        f"torque {max_torque:.3f} exceeds {self.config.max_abs_torque:.3f}"
                    )
            metrics = {
                "run_status": state.run_status,
                "robot_status": _safe_value(status),
                "max_temperature_c": max_temp,
                "max_abs_torque": max_torque,
                "failures": failures,
            }
            if failures:
                raise SopStepError("Pre-motion safety gate failed", metrics)
            return metrics

        self._run_step("pre_motion_safety_gate", action, fatal=True)

    def _check_enable_disable(self) -> None:
        """@brief 验证使能/失能控制链路。

        @details
        先失能再使能，确认控制器能够接受基础系统控制命令。
        该步骤通过后才进入真实运动测试。
        """
        def action() -> dict[str, Any]:
            disabled = bool(self.robot.disable_robot())
            enabled = bool(self.robot.enable_robot())
            if not disabled or not enabled:
                raise AssertionError(f"disable={disabled}, enable={enabled}")
            return {"disable_ok": disabled, "enable_ok": enabled}

        self._run_step("enable_disable_check", action, fatal=True)

    def _check_motion(self) -> None:
        """@brief 执行零点附近小行程关节运动测试。

        @details
        如果当前不是 PV 模式，会先切换到 PV。目标点由当前角度加上
        `motion_delta_deg` 得到，因此默认不会离开零点附近的小范围安全区。
        运动完成后读取反馈并按 `motion_tolerance_deg` 判定误差。
        """
        def action() -> dict[str, Any]:
            self._confirm_if_required("small-range joint motion")
            if getattr(self.robot.control_mode, "value", self.robot.control_mode) != "pv":
                if not self.robot.switch_mode("pv"):
                    raise AssertionError("Failed to switch to PV mode")
            state = self._require_state()
            start_deg = _angles_to_deg(state.angles)
            target_deg = [
                start + delta for start, delta in zip(start_deg, self.config.motion_delta_deg)
            ]
            if not self.robot.set_robot_state(
                target_joints=target_deg,
                joint_format="deg",
                speed=self.config.speed,
                wait_for_completion=True,
            ):
                raise AssertionError("Motion command returned False")
            final_state = self._require_state()
            actual_deg = _angles_to_deg(final_state.angles)
            errors = [actual - target for actual, target in zip(actual_deg, target_deg)]
            max_error = _max_abs(errors)
            metrics = {
                "start_joints_deg": [round(value, 4) for value in start_deg],
                "target_joints_deg": [round(value, 4) for value in target_deg],
                "actual_joints_deg": [round(value, 4) for value in actual_deg],
                "errors_deg": [round(value, 4) for value in errors],
                "max_error_deg": round(max_error, 4),
                "tolerance_deg": self.config.motion_tolerance_deg,
            }
            if max_error > self.config.motion_tolerance_deg:
                raise SopStepError("Motion target error exceeded tolerance", metrics)
            return metrics

        self._run_step("small_range_motion_check", action, fatal=True)

    def _check_gripper(self) -> None:
        """@brief 执行夹爪开、关、中间位测试。

        @details
        依次下发 `gripper_targets` 中的目标值，并读取反馈确认误差。
        目标值必须在配置安全检查阶段满足 [0, 1000]。
        """
        def action() -> dict[str, Any]:
            self._confirm_if_required("gripper motion")
            results = []
            for target in self.config.gripper_targets:
                if not self.robot.set_gripper_target(
                    value=target,
                    wait_for_completion=True,
                ):
                    raise AssertionError(f"Gripper command failed for target {target}")
                state = self._require_state()
                error = float(state.gripper) - float(target)
                results.append(
                    {
                        "target": target,
                        "actual": state.gripper,
                        "error": round(error, 4),
                    }
                )
                if abs(error) > self.config.gripper_tolerance:
                    raise AssertionError(f"Gripper target {target} exceeded tolerance")
            return {"targets": results, "tolerance": self.config.gripper_tolerance}

        self._run_step("gripper_check", action, fatal=True)

    def _run_aging(self) -> None:
        """@brief 执行可选老化测试 profile。

        @details
        aging 会先完整执行 quick 流程，确认设备处于合格基线后，
        再按小行程轨迹循环运行。循环过程中记录完成轮次、温度峰值、
        力矩峰值和失败信息。Ctrl+C 中断时仍会生成报告并尝试失能。
        """
        self._run_quick_checks()
        if self._failed:
            return

        def action() -> dict[str, Any]:
            deadline = None
            if self.config.aging_duration_min is not None:
                deadline = time.perf_counter() + self.config.aging_duration_min * 60.0
            cycles = self.config.aging_cycles
            if cycles is None and deadline is None:
                cycles = 100

            completed = 0
            failures = []
            max_temp = None
            max_torque = None
            while cycles is None or completed < cycles:
                if deadline is not None and time.perf_counter() >= deadline:
                    break
                try:
                    self._aging_cycle()
                    state = self._require_state()
                    if state.temperatures:
                        max_temp = max(max_temp or 0.0, max(float(v) for v in state.temperatures))
                    if state.torques:
                        max_torque = max(max_torque or 0.0, _max_abs(state.torques))
                    if self.config.max_temperature_c is not None and max_temp is not None:
                        if max_temp > self.config.max_temperature_c:
                            raise AssertionError(f"Temperature {max_temp:.2f}C exceeded limit")
                    if self.config.max_abs_torque is not None and max_torque is not None:
                        if max_torque > self.config.max_abs_torque:
                            raise AssertionError(f"Torque {max_torque:.3f} exceeded limit")
                    completed += 1
                    if self.config.aging_interval_s > 0:
                        time.sleep(self.config.aging_interval_s)
                except Exception as exc:
                    failures.append({"cycle": completed + 1, "error": str(exc)})
                    raise
            return {
                "completed_cycles": completed,
                "requested_cycles": cycles,
                "duration_min": self.config.aging_duration_min,
                "max_temperature_c": max_temp,
                "max_abs_torque": max_torque,
                "failures": failures,
            }

        self._run_step("aging_cycle_check", action, fatal=True)

    def _aging_cycle(self) -> None:
        """@brief 执行单轮老化运动。

        @details
        单轮老化包含“小幅偏移目标”和“返回起点”两个运动段。
        每轮都基于当前反馈位置生成目标，避免累计漂移。
        """
        self._confirm_if_required("aging cycle")
        state = self._require_state()
        start_deg = _angles_to_deg(state.angles)
        target_a = [start + delta for start, delta in zip(start_deg, self.config.motion_delta_deg)]
        target_b = start_deg
        for target in (target_a, target_b):
            ok = self.robot.set_robot_state(
                target_joints=target,
                joint_format="deg",
                speed=self.config.speed,
                wait_for_completion=True,
            )
            if not ok:
                raise AssertionError("Aging motion command returned False")

    def _require_state(self) -> Any:
        """@brief 在限定时间内获取有效关节状态。

        @return 最新 JointState 快照。
        @throws AssertionError 超过 `state_timeout` 仍未读到状态时抛出。
        """
        deadline = time.perf_counter() + self.config.state_timeout
        state = None
        while time.perf_counter() <= deadline:
            state = self.robot.get_robot_state("all")
            if state is not None:
                return state
            time.sleep(0.05)
        raise AssertionError("Timed out waiting for robot state")

    def _confirm_if_required(self, action_name: str) -> None:
        """@brief 可选人工安全确认。

        @param action_name 即将执行的动作名称，用于提示操作员。
        @throws AssertionError 当开启确认但操作员未输入 YES 时抛出。
        """
        if not self.config.require_confirmation:
            return
        answer = input(f"Confirm safe area before {action_name}. Type YES to continue: ").strip()
        if answer != "YES":
            raise AssertionError(f"Operator did not confirm safe area for {action_name}")

    def _safe_shutdown(self) -> None:
        """@brief 退出时尝试让机械臂失能。

        @details
        默认 `disable_on_exit=True`，无论 SOP PASS/FAIL 都会在断开前调用
        `disable_robot()`。如果产线调试需要保持使能，必须显式传入
        `--keep-enabled-on-exit`。
        """
        if self.robot is None or not self.config.disable_on_exit:
            return

        def action() -> dict[str, Any]:
            disabled = bool(self.robot.disable_robot())
            if not disabled:
                raise AssertionError("disable_robot returned False")
            return {"disable_ok": disabled}

        self._run_step("safe_shutdown_disable", action, fatal=False)

    def _run_step(
        self,
        name: str,
        action: Callable[[], Any],
        fatal: bool = False,
    ) -> SopStepResult:
        start = time.perf_counter()
        try:
            metrics = action()
            result = self._add_step(
                name,
                PASS,
                time.perf_counter() - start,
                metrics=_safe_value(metrics),
            )
            return result
        except SopStepError as exc:
            result = self._add_step(
                name,
                FAIL,
                time.perf_counter() - start,
                error=str(exc),
                metrics=_safe_value(exc.metrics),
            )
            if fatal:
                self._failed = True
            return result
        except Exception as exc:
            result = self._add_step(
                name,
                FAIL,
                time.perf_counter() - start,
                error=str(exc),
            )
            if fatal:
                self._failed = True
            return result

    def _add_step(
        self,
        name: str,
        status: str,
        duration_s: float,
        error: str = "",
        metrics: Optional[dict[str, Any]] = None,
    ) -> SopStepResult:
        step = SopStepResult(
            name=name,
            status=status,
            duration_s=round(duration_s, 3),
            error=error,
            metrics=metrics or {},
        )
        self.steps.append(step)
        return step


def write_report(report: SopReport, output_dir: str) -> dict[str, str]:
    """@brief 写出 SOP 报告文件。

    @param report 完整 SOP 报告对象。
    @param output_dir 报告输出目录。
    @return 包含 json、csv、markdown 三种报告路径的字典。
    """

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    serial = report.serial or "unknown"
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = out_dir / f"sop_{serial}_{stamp}"
    json_path = base.with_suffix(".json")
    csv_path = base.with_suffix(".csv")
    md_path = base.with_suffix(".md")

    report_dict = _safe_value(asdict(report))
    report_dict["report_paths"] = {}
    json_path.write_text(json.dumps(report_dict, indent=2, ensure_ascii=False), encoding="utf-8")

    with csv_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "status",
                "profile",
                "serial",
                "operator",
                "port",
                "started_at",
                "finished_at",
                "duration_s",
                "failed_steps",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "status": report.status,
                "profile": report.profile,
                "serial": report.serial,
                "operator": report.operator,
                "port": report.port,
                "started_at": report.started_at,
                "finished_at": report.finished_at,
                "duration_s": report.duration_s,
                "failed_steps": ";".join(step.name for step in report.steps if step.status == FAIL),
            }
        )

    failed = [step for step in report.steps if step.status == FAIL]
    lines = [
        f"# Alicia-M SOP Report: {report.status}",
        "",
        f"- Profile: {report.profile}",
        f"- Serial: {report.serial}",
        f"- Operator: {report.operator}",
        f"- Port: {report.port}",
        f"- Started: {report.started_at}",
        f"- Finished: {report.finished_at}",
        f"- Duration: {report.duration_s}s",
        "",
        "## Steps",
        "",
    ]
    for step in report.steps:
        line = f"- {step.status} {step.name} ({step.duration_s}s)"
        if step.error:
            line += f": {step.error}"
        lines.append(line)
    if failed:
        lines.extend(["", "## Failed Steps", ""])
        for step in failed:
            lines.append(f"- {step.name}: {step.error}")
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    return {
        "json": str(json_path),
        "csv": str(csv_path),
        "markdown": str(md_path),
    }


def run_sop(config: SopConfig) -> SopReport:
    """@brief 运行一次 SOP 测试。

    @param config SOP 测试配置。
    @return SopReport 完整测试报告。
    """

    return SopRunner(config).run()
