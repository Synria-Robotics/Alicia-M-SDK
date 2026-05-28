"""示例脚本运行时辅助工具。"""

from __future__ import annotations

import sys

import numpy as np

from ..diagnostics import (
    CONTROL_MODE_NAMES,
    DIAGNOSTIC_JOINT_COUNT,
    MOTOR_STATE_NAMES,
    DiagnosticResult,
    code_name,
    mode_name,
)
from ..execution.trajectory_executor import load_waypoints_from_file
from ..execution.joint_mapping import convert_joints_rad_from_alicia_d_to_alicia_m
from ..hardware.constants import ERROR_DESCRIPTIONS
from .beauty_logger import beauty_print, beauty_print_array
from .protocol import format_bytes


ERROR_EXTRA_LABELS = {
    0x00: "收到的帧长度",
    0x01: "收到的帧长度",
    0x02: "下位机计算出的校验字节",
    0x03: "预留信息",
    0x04: "越界关节编号",
    0x05: "当前数据长度或数量",
    0x06: "非法地址或越界值",
    0x07: "请求功能码",
    0x08: "预留信息",
}


def print_diagnostic_response(result: DiagnosticResult) -> bool:
    """@brief 打印自检响应快照。

    @param result 自检结构化结果。
    @return True 表示响应正常，False 表示超时或错误响应。
    """
    if result.frame is None:
        return False
    if result.error_code is not None:
        print_error_response(result)
        return False

    for snapshot in result.snapshots:
        beauty_print(f"{snapshot.arm_name}自检快照:", type="info")
        beauty_print(f"  通信状态位图: 0x{snapshot.comm_bitmap:02X}", type="info")
        beauty_print(f"  通信状态: {format_comm_bitmap(snapshot.comm_bitmap)}", type="info")
        beauty_print(f"  电机状态: {format_bytes(bytes(snapshot.motor_states))}", type="info")
        beauty_print(f"  状态含义: {format_motor_states(snapshot.motor_states)}", type="info")
        beauty_print(f"  电机模式: {format_bytes(bytes(snapshot.control_modes))}", type="info")
        beauty_print(f"  模式含义: {format_control_modes(snapshot.control_modes)}", type="info")
    return True


def print_error_response(result: DiagnosticResult):
    """@brief 打印 0xEE 错误响应。"""
    error_type = result.error_code
    extra = result.error_data[0] if result.error_data else None
    error_name = ERROR_DESCRIPTIONS.get(error_type, "未知错误")
    beauty_print(f"自检返回错误: {error_name} (0x{error_type:02X})", type="warning")

    if extra is None:
        beauty_print("  附加信息: 缺失", type="warning")
        return

    if error_type == 0xEE:
        current_mode = (extra & 0xF0) >> 4
        target_mode = extra & 0x0F
        beauty_print(f"  当前模式: 0x{current_mode:X} {mode_name(current_mode)}", type="info")
        beauty_print(f"  目标模式: 0x{target_mode:X} {mode_name(target_mode)}", type="info")
        return

    extra_label = ERROR_EXTRA_LABELS.get(error_type, "附加信息")
    beauty_print(f"  {extra_label}: 0x{extra:02X}", type="info")


def format_comm_bitmap(comm_bitmap: int) -> str:
    """@brief 格式化 7 个关节的通信快照位。"""
    states = []
    for joint_index in range(DIAGNOSTIC_JOINT_COUNT):
        state = "收到" if comm_bitmap & (1 << joint_index) else "未收到"
        states.append(f"J{joint_index + 1}={state}")
    return ", ".join(states)


def format_motor_states(states: list[int]) -> str:
    """@brief 格式化 J1~J7 电机状态码。"""
    return ", ".join(
        f"J{index + 1}={code_name(value, MOTOR_STATE_NAMES)}"
        for index, value in enumerate(states)
    )


def format_control_modes(modes: list[int]) -> str:
    """@brief 格式化 J1~J7 控制模式。"""
    return ", ".join(
        f"J{index + 1}={code_name(value, CONTROL_MODE_NAMES)}"
        for index, value in enumerate(modes)
    )


try:
    import msvcrt
except ImportError:
    msvcrt = None

if msvcrt is None:
    import select
    import termios
    import tty
else:
    select = None
    termios = None
    tty = None


class NonBlockingKeyReader:
    """@brief 跨平台非阻塞单键读取器。"""

    def __init__(self):
        self._original_termios = None

    def __enter__(self):
        if msvcrt is None and sys.stdin.isatty():
            self._original_termios = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc, traceback):
        if msvcrt is None and self._original_termios is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._original_termios)
        return False

    def read_key(self):
        """@brief 非阻塞读取一个按键；没有按键时返回 None。"""
        if msvcrt is not None:
            if not msvcrt.kbhit():
                return None
            key = msvcrt.getwch()
            if key in ("\x00", "\xe0"):
                if msvcrt.kbhit():
                    msvcrt.getwch()
                return None
            return key.lower()

        if not sys.stdin.isatty():
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if not readable:
            return None
        return sys.stdin.read(1).lower()


def format_values(values, precision=4):
    """@brief 格式化状态数组，便于连续打印。"""
    if values is None:
        return "N/A"
    return "[" + ", ".join(f"{value:.{precision}f}" for value in values) + "]"


def print_robot_state(robot):
    """@brief 打印当前关节位置、速度和力矩。"""
    state = robot.get_robot_state("all")
    if state is None:
        print("pos=N/A vel=N/A tor=N/A", flush=True)
        return

    pos = list(state.angles) + [state.gripper]
    print(
        f"pos={format_values(pos)} "
        f"vel={format_values(state.velocities)} "
        f"tor={format_values(state.torques)}",
        flush=True,
    )


def select_mode():
    """@brief 选择路点输入模式。"""
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
    """@brief 从当前机器人状态手动记录关节路点。"""
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
    """@brief 卸力后拖动示教并记录关节路点。"""
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
        beauty_print("已卸力，可手动拖动采点。", type="success")
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


def auto_generate_waypoints(
    robot,
    robot_model,
    num_waypoints: int = 5,
    joint_scale: float = 0.6,
    use_current_joints: bool = False,
    seed=666,
):
    """@brief 自动生成随机关节路点。

    @param robot 机器人实例（用于读取当前关节角）。
    @param robot_model RoboCore RobotModel（用于生成随机关节角）。
    @param num_waypoints 路点数量，至少为 2。
    @param joint_scale random_q 缩放系数。
    @param use_current_joints 是否将当前关节角作为首个路点。
    @param seed 随机种子。
    @return 路点数组 [N, 6]。
    """
    beauty_print("自动生成模式", type="module")
    num_waypoints = max(2, num_waypoints)
    beauty_print(f"生成路点数: {num_waypoints}", type="info")

    waypoints = []
    if use_current_joints:
        current = robot.get_robot_state("joint")
        if current is not None:
            waypoints.append(np.asarray(current, dtype=np.float64))
            beauty_print("首个路点使用当前关节角。", type="info")

    _seed = seed
    while len(waypoints) < num_waypoints:
        if hasattr(robot_model, "random_q"):
            q = robot_model.random_q(seed=_seed, scale=joint_scale)
            q = np.asarray(q, dtype=np.float64)
        else:
            rng = np.random.default_rng(_seed)
            q = rng.uniform(
                low=np.deg2rad([-120, -120, -120, -170, -120, -170]),
                high=np.deg2rad([120, 120, 120, 170, 120, 170]),
                size=(6,),
            ).astype(np.float64)
        waypoints.append(q)
        if _seed is not None:
            _seed += 1

    return np.asarray(waypoints, dtype=np.float64)


def load_waypoints_interactive(path: str):
    """@brief 从显式路径或用户输入路径加载路点。"""
    file_path = path or input("请输入路点文件路径: ").strip()
    return load_waypoints_from_file(file_path)


def make_mapped_teleop_state_printer(frequency_hz: float):
    """@brief 创建遥操作映射状态打印回调。

    @param frequency_hz 遥操作循环频率。
    @return 可传入 Teleoperation.set_state_callback 的回调。
    """
    interval = max(1, int(frequency_hz))

    def _print_state(joints, gripper, count):
        if count % interval == 0:
            leader_deg = np.round(np.degrees(joints), 1).tolist()
            follower_deg = np.round(
                np.degrees(convert_joints_rad_from_alicia_d_to_alicia_m(joints)),
                1,
            ).tolist()
            print(f"  [{count:6d}] 示教臂={leader_deg}  操作臂(映射)={follower_deg}  夹爪={gripper:.0f}")

    return _print_state


def print_joint_state(device, continuous: bool = False, output_format: str = "deg") -> None:
    """@brief 打印关节角度、速度和力矩。

    @param device 硬件 Device 实例。
    @param continuous 为 True 时持续打印，直到 Ctrl-C 中断。
    @param output_format ``"deg"`` 或 ``"rad"``。
    """
    import math as _math
    import time as _time

    try:
        from robocore.utils.beauty_logger import beauty_print as _rc_beauty_print
        from robocore.utils.beauty_logger import beauty_print_array as _bpa

        def _bp(content, type=None):
            if type is None:
                _rc_beauty_print(content)
            else:
                _rc_beauty_print(content, type=type)

    except ImportError:
        def _bp(content, type=None):
            print(content)

        def _bpa(arr, **kw):
            return str(arr)

    def _print_once():
        state = device.joint_state
        if state is None:
            _bp("未获取到状态数据", type="warning")
            return
        angles = list(state.angles)
        if output_format == "deg":
            angles_display = [_math.degrees(a) for a in angles]
            unit = "deg"
        else:
            angles_display = list(angles)
            unit = "rad"
        _bp(f"关节角度 ({unit}):")
        print(f"  {_bpa(angles_display, precision=2)}")
        _bp(f"夹爪: {state.gripper:.0f}")
        if state.velocities:
            _bp("速度 (rad/s):")
            print(f"  {_bpa(state.velocities, precision=3)}")
        if state.torques:
            _bp("力矩 (N·m):")
            print(f"  {_bpa(state.torques, precision=3)}")

    if continuous:
        try:
            while True:
                _print_once()
                print("---")
                _time.sleep(0.1)
        except KeyboardInterrupt:
            pass
    else:
        _print_once()
