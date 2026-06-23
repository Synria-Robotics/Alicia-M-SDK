#!/usr/bin/env python3
"""@file 19_demo_slider_control.py
@brief Alicia-M 滑块窗口控制示例。

@details
本示例连接真实 Alicia-M 机械臂，并打开一个 tkinter 窗口。窗口中包含
7 个滑块：6 个关节角度滑块和 1 个夹爪滑块。关节滑块以角度显示，
实际发送给 SDK 时使用 ``joint_format="deg"``；夹爪滑块范围固定为
``0~1000``。拖动滑块时，程序会合并最新目标并按固定周期节流发送，
避免滑块连续回调导致串口命令过密。

@note
本示例会向真实机械臂发送运动命令。运行前应清空工作空间并确认安全。
"""

from __future__ import annotations

import argparse
import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Optional, Sequence

import alicia_m_sdk
from _demo_helpers import add_port_argument, beauty_print


# @brief 未显式传入 URDF 时使用的 Alicia-M v1.1 follower 关节限位，单位 rad。
DEFAULT_JOINT_LIMITS_RAD: tuple[tuple[float, float], ...] = (
    (-2.7475, 2.7475),
    (-3.14, 0.0),
    (-3.14, 0.0),
    (-1.57, 1.57),
    (-1.57, 1.57),
    (-2.791, 2.791),
)
# @brief 默认限位来源描述，用于启动日志。
DEFAULT_LIMIT_SOURCE = "Alicia-M v1.1 follower defaults"
# @brief 夹爪滑块和发送前裁剪范围。
GRIPPER_LIMIT = (0.0, 1000.0)


@dataclass(frozen=True)
class JointLimitSource:
    """@brief 关节限位及来源描述。

    @param limits_rad 6 个旋转关节的 ``(lower, upper)`` 限位，单位 rad。
    @param description 面向用户打印的限位来源说明。
    """

    # @brief 6 个关节的弧度限位。
    limits_rad: tuple[tuple[float, float], ...]
    # @brief 限位数据来源描述。
    description: str


def clamp(value: float, lower: float, upper: float) -> float:
    """@brief 将数值裁剪到闭区间内。

    @param value 原始数值。
    @param lower 下限。
    @param upper 上限。
    @return 裁剪后的数值。
    """
    return max(lower, min(upper, value))


def clamp_joints_deg(
    values: Sequence[float],
    limits_deg: Sequence[tuple[float, float]],
) -> list[float]:
    """@brief 按给定角度限位裁剪 6 个关节角。

    @param values 6 个关节角，单位 deg。
    @param limits_deg 6 个关节的 ``(lower, upper)`` 限位，单位 deg。
    @return 裁剪后的 6 个关节角，单位 deg。
    @throws ValueError 当关节值或限位数量不是 6 时抛出。
    """
    if len(values) != 6:
        raise ValueError(f"expected 6 joint values, got {len(values)}")
    if len(limits_deg) != 6:
        raise ValueError(f"expected 6 joint limits, got {len(limits_deg)}")
    return [
        clamp(float(value), float(lower), float(upper))
        for value, (lower, upper) in zip(values, limits_deg)
    ]


def rad_limits_to_deg(
    limits_rad: Sequence[tuple[float, float]],
) -> list[tuple[float, float]]:
    """@brief 将 6 个关节限位从弧度转换为角度。

    @param limits_rad 6 个关节的 ``(lower, upper)`` 限位，单位 rad。
    @return 6 个关节的 ``(lower, upper)`` 限位，单位 deg。
    @throws ValueError 当限位数量不是 6 时抛出。
    """
    if len(limits_rad) != 6:
        raise ValueError(f"expected 6 joint limits, got {len(limits_rad)}")
    return [
        (math.degrees(float(lower)), math.degrees(float(upper)))
        for lower, upper in limits_rad
    ]


def parse_urdf_revolute_limits(path: Path) -> tuple[tuple[float, float], ...]:
    """@brief 从 URDF 解析前 6 个旋转关节限位。

    @details
    本函数只读取 ``<joint type="revolute">`` 节点中的 ``<limit lower upper>``，
    不解析碰撞几何、动力学参数或其他关节类型。解析结果用于滑块范围和发送前裁剪。

    @param path URDF 文件路径。
    @return 前 6 个旋转关节的 ``(lower, upper)`` 限位，单位 rad。
    @throws FileNotFoundError 当 URDF 文件不存在时抛出。
    @throws ValueError 当 URDF 无法解析、旋转关节不足 6 个或缺少 lower/upper 时抛出。
    """
    if not path.exists():
        raise FileNotFoundError(f"URDF file not found: {path}")
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        raise ValueError(f"failed to parse URDF: {path}: {exc}") from exc

    limits: list[tuple[float, float]] = []
    for joint in root.findall("joint"):
        if joint.attrib.get("type") != "revolute":
            continue
        limit = joint.find("limit")
        joint_name = joint.attrib.get("name", "<unnamed>")
        if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
            raise ValueError(f"revolute joint missing lower/upper limit: {joint_name}")
        try:
            lower = float(limit.attrib["lower"])
            upper = float(limit.attrib["upper"])
        except ValueError as exc:
            raise ValueError(f"invalid limit value on revolute joint: {joint_name}") from exc
        limits.append((lower, upper))
        if len(limits) == 6:
            return tuple(limits)

    raise ValueError(f"expected at least 6 revolute joints in URDF, got {len(limits)}")


def load_joint_limits(urdf_path: str) -> JointLimitSource:
    """@brief 加载滑块使用的关节限位。

    @param urdf_path 可选 URDF 路径；为空时使用脚本内置默认限位。
    @return 关节限位及来源描述。
    @throws FileNotFoundError 当显式传入的 URDF 不存在时抛出。
    @throws ValueError 当显式传入的 URDF 限位解析失败时抛出。
    """
    if not urdf_path:
        return JointLimitSource(DEFAULT_JOINT_LIMITS_RAD, DEFAULT_LIMIT_SOURCE)
    path = Path(urdf_path).expanduser()
    return JointLimitSource(
        parse_urdf_revolute_limits(path),
        f"URDF: {path}",
    )


def _as_control_mode_value(control_mode: Any) -> str:
    """@brief 兼容枚举或字符串形式的控制模式。

    @param control_mode SDK 返回的控制模式对象或字符串。
    @return 小写控制模式名称。
    """
    return str(getattr(control_mode, "value", control_mode)).lower()


class SliderControlWindow:
    """@brief tkinter 滑块窗口与节流发送器。

    @details
    窗口负责显示 6 个关节滑块和 1 个夹爪滑块。每次滑块变化只记录最新目标，
    实际发送由 ``tkinter.after`` 延迟触发，确保连续拖动时不会每个回调都写串口。
    """

    def __init__(
        self,
        root: Any,
        robot: Any,
        initial_joints_deg: Sequence[float],
        initial_gripper: float,
        joint_limits_deg: Sequence[tuple[float, float]],
        speed: float,
        gripper_speed: float,
        send_interval_ms: int,
    ) -> None:
        """@brief 初始化滑块窗口。

        @param root tkinter 根窗口。
        @param robot 已连接的 ``SynriaRobotAPI`` 实例。
        @param initial_joints_deg 当前 6 个关节角，单位 deg。
        @param initial_gripper 当前夹爪值，范围 0~1000。
        @param joint_limits_deg 6 个关节滑块限位，单位 deg。
        @param speed 关节运动速度参数。
        @param gripper_speed 夹爪运动速度参数。
        @param send_interval_ms 滑块目标最小发送间隔，单位 ms。
        """
        self.root = root
        self.robot = robot
        # @brief 当前窗口使用的 6 轴角度限位，单位 deg。
        self.joint_limits_deg = list(joint_limits_deg)
        # @brief 关节目标下发速度。
        self.speed = speed
        # @brief 夹爪目标下发速度。
        self.gripper_speed = gripper_speed
        # @brief tkinter.after 节流周期。
        self.send_interval_ms = send_interval_ms
        self._tk = __import__("tkinter")
        # @brief 当前已注册但尚未触发的 after 回调 id。
        self._pending_after_id: Optional[str] = None
        # @brief 待发送的最新 6 轴角度目标，单位 deg。
        self._pending_joint_target: Optional[list[float]] = None
        # @brief 待发送的最新夹爪目标。
        self._pending_gripper_target: Optional[float] = None
        # @brief 窗口是否已经关闭，用于阻止后续发送。
        self._closed = False
        # @brief 初始化滑块变量期间禁止触发发送。
        self._initializing = True

        self.root.title("Alicia-M Slider Control")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.status_var = self._tk.StringVar(value="Connected")
        self.joint_vars: list[Any] = []
        self.joint_value_vars: list[Any] = []
        self.gripper_var = self._tk.DoubleVar(value=clamp(initial_gripper, *GRIPPER_LIMIT))
        self.gripper_value_var = self._tk.StringVar(value=f"{self.gripper_var.get():.0f}")

        self._build_widgets(initial_joints_deg)
        self._initializing = False

    def _build_widgets(self, initial_joints_deg: Sequence[float]) -> None:
        """@brief 构建窗口布局和 7 个滑块。

        @param initial_joints_deg 当前 6 个关节角，单位 deg。
        """
        frame = self._tk.Frame(self.root, padx=12, pady=12)
        frame.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

        clipped_joints = clamp_joints_deg(initial_joints_deg, self.joint_limits_deg)
        for index, (value, (lower, upper)) in enumerate(zip(clipped_joints, self.joint_limits_deg)):
            self._add_joint_slider(frame, index, value, lower, upper)

        self._add_gripper_slider(frame, row=6)
        status = self._tk.Label(frame, textvariable=self.status_var, anchor="w")
        status.grid(row=7, column=0, columnspan=3, sticky="ew", pady=(12, 0))

    def _add_joint_slider(
        self,
        parent: Any,
        index: int,
        value: float,
        lower: float,
        upper: float,
    ) -> None:
        """@brief 添加单个关节滑块行。

        @param parent 父容器。
        @param index 关节索引，范围 0~5。
        @param value 初始关节角，单位 deg。
        @param lower 滑块下限，单位 deg。
        @param upper 滑块上限，单位 deg。
        """
        label = self._tk.Label(parent, text=f"J{index + 1} (deg)", width=12, anchor="w")
        label.grid(row=index, column=0, sticky="w", pady=3)

        var = self._tk.DoubleVar(value=value)
        value_var = self._tk.StringVar(value=f"{value:.1f}")
        slider = self._tk.Scale(
            parent,
            from_=lower,
            to=upper,
            orient=self._tk.HORIZONTAL,
            resolution=0.1,
            variable=var,
            showvalue=False,
            command=lambda _value, joint_index=index: self._on_joint_change(joint_index),
        )
        slider.grid(row=index, column=1, sticky="ew", padx=8, pady=3)
        value_label = self._tk.Label(parent, textvariable=value_var, width=8, anchor="e")
        value_label.grid(row=index, column=2, sticky="e", pady=3)

        self.joint_vars.append(var)
        self.joint_value_vars.append(value_var)

    def _add_gripper_slider(self, parent: Any, row: int) -> None:
        """@brief 添加夹爪滑块行。

        @param parent 父容器。
        @param row tkinter grid 行号。
        """
        label = self._tk.Label(parent, text="Gripper", width=12, anchor="w")
        label.grid(row=row, column=0, sticky="w", pady=3)
        slider = self._tk.Scale(
            parent,
            from_=GRIPPER_LIMIT[0],
            to=GRIPPER_LIMIT[1],
            orient=self._tk.HORIZONTAL,
            resolution=1,
            variable=self.gripper_var,
            showvalue=False,
            command=lambda _value: self._on_gripper_change(),
        )
        slider.grid(row=row, column=1, sticky="ew", padx=8, pady=3)
        value_label = self._tk.Label(parent, textvariable=self.gripper_value_var, width=8, anchor="e")
        value_label.grid(row=row, column=2, sticky="e", pady=3)

    def _on_joint_change(self, joint_index: int) -> None:
        """@brief 响应关节滑块变化。

        @param joint_index 发生变化的关节索引，范围 0~5。
        """
        value = float(self.joint_vars[joint_index].get())
        self.joint_value_vars[joint_index].set(f"{value:.1f}")
        if self._initializing:
            return
        self._pending_joint_target = self._current_joint_target()
        self._schedule_send()

    def _on_gripper_change(self) -> None:
        """@brief 响应夹爪滑块变化。"""
        value = clamp(float(self.gripper_var.get()), *GRIPPER_LIMIT)
        self.gripper_value_var.set(f"{value:.0f}")
        if self._initializing:
            return
        self._pending_gripper_target = value
        self._schedule_send()

    def _current_joint_target(self) -> list[float]:
        """@brief 读取并裁剪当前 6 个关节滑块值。

        @return 当前 6 个关节目标角，单位 deg。
        """
        return clamp_joints_deg(
            [float(var.get()) for var in self.joint_vars],
            self.joint_limits_deg,
        )

    def _schedule_send(self) -> None:
        """@brief 安排一次节流发送。

        @details
        若已有待触发的 after 回调，则只更新待发送目标，不重复注册回调。
        """
        if self._closed or self._pending_after_id is not None:
            return
        delay = max(1, int(self.send_interval_ms))
        self._pending_after_id = self.root.after(delay, self._flush_pending_targets)

    def _flush_pending_targets(self) -> None:
        """@brief 发送当前累计的最新关节/夹爪目标。

        @details
        该函数由 ``tkinter.after`` 调用。发送前会取出并清空 pending 目标，
        因此连续拖动期间只会发送每个节流周期内的最新值。
        """
        self._pending_after_id = None
        if self._closed:
            return

        joint_target = self._pending_joint_target
        gripper_target = self._pending_gripper_target
        self._pending_joint_target = None
        self._pending_gripper_target = None

        try:
            # 关键步骤 1：若有关节目标，合并 6 轴当前滑块值并非阻塞下发。
            if joint_target is not None:
                ok = self.robot.set_robot_state(
                    target_joints=joint_target,
                    joint_format="deg",
                    speed=self.speed,
                    wait_for_completion=False,
                )
                self._set_status(ok, "joint", joint_target)
            # 关键步骤 2：若有夹爪目标，单独下发夹爪位置，关节保持不变。
            if gripper_target is not None:
                ok = self.robot.set_robot_state(
                    target_joints=None,
                    gripper_value=gripper_target,
                    gripper_speed=self.gripper_speed,
                    wait_for_completion=False,
                )
                self._set_status(ok, "gripper", gripper_target)
        except Exception as exc:  # Keep the UI alive so the user can close safely.
            self.status_var.set(f"Send failed: {exc}")

    def _set_status(self, ok: bool, target_name: str, target: Any) -> None:
        """@brief 更新窗口底部状态文本。

        @param ok SDK 下发结果。
        @param target_name 目标类型名称。
        @param target 已发送的目标值。
        """
        timestamp = time.strftime("%H:%M:%S")
        if ok:
            self.status_var.set(f"{timestamp} sent {target_name}: {target}")
        else:
            self.status_var.set(f"{timestamp} send failed: {target_name}")

    def close(self) -> None:
        """@brief 关闭窗口并断开机械臂连接。"""
        if self._closed:
            return
        self._closed = True
        if self._pending_after_id is not None:
            try:
                self.root.after_cancel(self._pending_after_id)
            except Exception:
                pass
            self._pending_after_id = None
        try:
            self.robot.disconnect()
            beauty_print("Robot disconnected", type="info")
        finally:
            self.status_var.set("Disconnected")
            self.root.destroy()


def _state_to_initial_values(state: Any) -> tuple[list[float], float]:
    """@brief 从 SDK 状态对象中提取滑块初始值。

    @param state ``robot.get_robot_state("all")`` 返回的状态对象或兼容字典。
    @return ``(joints_deg, gripper)``，其中 joints_deg 为 6 个关节角，单位 deg。
    @throws RuntimeError 当状态中无法读取 6 个关节角时抛出。
    """
    angles = getattr(state, "angles", None)
    gripper = getattr(state, "gripper", None)
    if angles is None and isinstance(state, dict):
        angles = state.get("angles") or state.get("joint") or state.get("joints")
        gripper = state.get("gripper", gripper)
    if angles is None or len(angles) < 6:
        raise RuntimeError("failed to read current 6 joint angles from robot state")
    if gripper is None:
        gripper = 0.0
    joints_deg = [math.degrees(float(value)) for value in angles[:6]]
    return joints_deg, float(gripper)


def _confirm_safe_to_run(skip_confirmation: bool) -> None:
    """@brief 在启动窗口前进行人工安全确认。

    @param skip_confirmation True 表示跳过确认，通常来自 ``--yes``。
    """
    if skip_confirmation:
        return
    beauty_print(
        "This demo sends live motion commands while sliders move. Clear the workspace first.",
        type="warning",
    )
    input("Press Enter to start slider control...")


def _prepare_pv_mode(robot: Any, prompt: Callable[[str], str] = input) -> None:
    """@brief 确保机械臂处于 PV 控制模式。

    @param robot 已连接的 ``SynriaRobotAPI`` 实例。
    @param prompt 交互输入函数，测试时可注入替身。
    """
    current = _as_control_mode_value(robot.control_mode)
    if current == "pv":
        return
    beauty_print(
        "This demo requires PV mode. The arm may briefly disable while switching.",
        type="warning",
    )
    prompt("Press Enter to switch to PV mode...")
    robot.switch_mode("pv")
    beauty_print("Switched to PV mode", type="success")


def run(args: argparse.Namespace) -> None:
    """@brief 执行滑块控制 demo 主流程。

    @param args 命令行参数。
    """
    # 关键步骤 1：加载并打印关节限位，滑块范围和发送前裁剪均使用同一份限位。
    limits = load_joint_limits(args.urdf_path)
    limits_deg = rad_limits_to_deg(limits.limits_rad)
    beauty_print(f"Joint limits source: {limits.description}", type="info")
    for index, (lower, upper) in enumerate(limits_deg, start=1):
        beauty_print(f"  J{index}: {lower:.1f} deg to {upper:.1f} deg", type="info")

    # 关键步骤 2：真实运动前进行安全确认。
    _confirm_safe_to_run(args.yes)

    # 关键步骤 3：连接设备并确保 PV 模式。
    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print(
        f"Robot connected, current mode: {_as_control_mode_value(robot.control_mode).upper()}",
        type="success",
    )

    try:
        _prepare_pv_mode(robot)
        # 关键步骤 4：读取当前状态作为滑块初值，避免窗口打开后突然跳到默认姿态。
        state = robot.get_robot_state("all")
        initial_joints_deg, initial_gripper = _state_to_initial_values(state)

        # 关键步骤 5：创建 tkinter 窗口并进入事件循环。
        import tkinter as tk

        root = tk.Tk()
        SliderControlWindow(
            root=root,
            robot=robot,
            initial_joints_deg=initial_joints_deg,
            initial_gripper=initial_gripper,
            joint_limits_deg=limits_deg,
            speed=args.speed,
            gripper_speed=args.gripper_speed,
            send_interval_ms=args.send_interval_ms,
        )
        root.mainloop()
    except KeyboardInterrupt:
        beauty_print("Interrupted by user", type="warning")
        robot.disconnect()
    except Exception:
        robot.disconnect()
        raise


def build_parser() -> argparse.ArgumentParser:
    """@brief 构建命令行参数解析器。

    @return 配置完成的 ``argparse.ArgumentParser``。
    """
    parser = argparse.ArgumentParser(description="Control Alicia-M joints and gripper with sliders.")
    add_port_argument(parser)
    parser.add_argument("--speed", type=float, default=15.0, help="Joint motion speed; default 15.")
    parser.add_argument("--gripper-speed", type=float, default=100.0, help="Gripper motion speed; default 100.")
    parser.add_argument(
        "--send-interval-ms",
        type=int,
        default=120,
        help="Minimum interval between slider command flushes; default 120 ms.",
    )
    parser.add_argument("--urdf-path", type=str, default="", help="Optional URDF path for joint limits.")
    parser.add_argument("--yes", action="store_true", help="Skip the startup safety confirmation.")
    return parser


def main() -> None:
    """@brief 命令行入口。"""
    run(build_parser().parse_args())


if __name__ == "__main__":
    main()
