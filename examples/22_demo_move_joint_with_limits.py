"""22_demo_move_joint_with_limits.py — 带底层限位裁剪的关节控制 (PV).

演示 PV 模式关节空间运动控制，按所选构型硬限位额外扣除 1% 安全余量，
发送前自动裁剪目标。
"""

import argparse
import math
import time

import alicia_m_sdk
from _demo_helpers import add_port_argument, beauty_print, beauty_print_array


LIMIT_MARGIN_RATIO = 0.01
ARRIVAL_TOLERANCE_RAD = 0.008
ARRIVAL_TIMEOUT_SECONDS = 15.0
ARRIVAL_POLL_INTERVAL_SECONDS = 0.01
DEFAULT_CONTROL_MODE = "pv"
GRIPPER_CLOSE_TARGET = 0.0
DEFAULT_GRIPPER_CLOSE_SPEED = 40.0

# STM32 standard / ARX 构型 J1-J6 机械硬限位，单位 rad。
LIMIT_PROFILES = {
    "standard": (
        (-2.7475, 2.7475),
        (-3.14, 0.0),
        (-3.14, 0.0),
        (-1.57, 1.57),
        (-1.57, 1.57),
        (-2.6, 2.6),
    ),
    "arx": (
        (-2.74889, 2.74889),
        (-3.14159, 0.0),
        (-3.14159, 0.0),
        (-1.57080, 0.78540),
        (-1.57080, 1.57080),
        (-2.79253, 2.79253),
    ),
}

# 测试流程目标点，单位 deg；J2/J3 的 0 deg 会在发送前裁剪到安全上限。
TEST_FLOW_DEG = [
    [0, 0, 0, 0, 0, 0],
    [30, 0, 0, 0, 0, 0],
    [0, -30, 0, 0, 0, 0],
    [0, 0, -30, 0, 0, 0],
    [0, 0, 0, -30, 0, 0],
    [0, 0, 0, 0, 30, 0],
    [0, 0, 0, 0, 0, -30],
    [0, 0, 0, 0, 0, 0],
]


def build_safe_joint_limits(hard_limits_rad, margin_ratio=LIMIT_MARGIN_RATIO):
    """Build firmware-equivalent safe joint limits from hard limits."""
    if len(hard_limits_rad) != 6:
        raise ValueError("expected 6 joint limits")

    safe_limits = []
    for lower, upper in hard_limits_rad:
        lower = float(lower)
        upper = float(upper)
        margin = (upper - lower) * float(margin_ratio)
        safe_limits.append((round(lower + margin, 5), round(upper - margin, 5)))
    return tuple(safe_limits)


DEFAULT_LIMIT_PROFILE = "standard"
SAFE_JOINT_LIMITS_RAD = build_safe_joint_limits(LIMIT_PROFILES[DEFAULT_LIMIT_PROFILE])


def deg_to_rad_list(values_deg):
    """Convert six joint values from degrees to radians."""
    if len(values_deg) != 6:
        raise ValueError("expected 6 joint values")
    return [math.radians(float(value)) for value in values_deg[:6]]


def clamp_joint_targets_rad(target_rad, limits_rad=SAFE_JOINT_LIMITS_RAD):
    """Clamp joint targets into safe firmware limits."""
    if len(target_rad) != 6:
        raise ValueError("expected 6 joint targets")
    if len(limits_rad) != 6:
        raise ValueError("expected 6 joint limits")

    clipped = []
    clipped_indexes = []
    for index, (value, (lower, upper)) in enumerate(zip(target_rad, limits_rad), start=1):
        value = float(value)
        limited = max(float(lower), min(float(upper), value))
        if abs(limited - value) > 1e-9:
            clipped_indexes.append(index)
        limited = round(limited, 4)
        clipped.append(limited)
    return clipped, clipped_indexes


def build_safe_test_flow_rad(flow_deg, limits_rad=SAFE_JOINT_LIMITS_RAD):
    """Convert a degree waypoint flow into clipped radian targets."""
    return [
        clamp_joint_targets_rad(deg_to_rad_list(waypoint_deg), limits_rad)[0]
        for waypoint_deg in flow_deg
    ]


def build_joint_error_report(target_rad, feedback_rad):
    """Build per-joint error values in radians and degrees."""
    if len(target_rad) != 6 or len(feedback_rad) < 6:
        raise ValueError("expected 6 target joints and at least 6 feedback joints")

    error_rad = joint_errors_rad(target_rad, feedback_rad)
    return {
        "error_rad": [round(value, 4) for value in error_rad],
        "error_deg": [round(math.degrees(value), 4) for value in error_rad],
    }


def joint_errors_rad(target_rad, feedback_rad):
    """Return unrounded feedback-minus-target joint errors in radians."""
    if len(target_rad) != 6 or len(feedback_rad) < 6:
        raise ValueError("expected 6 target joints and at least 6 feedback joints")
    return [
        float(actual) - float(expected)
        for actual, expected in zip(feedback_rad[:6], target_rad[:6])
    ]


def is_target_reached(error_rad, tolerance=ARRIVAL_TOLERANCE_RAD):
    """Return True when every joint error is strictly inside tolerance."""
    if len(error_rad) != 6:
        raise ValueError("expected 6 joint errors")
    return all(abs(float(value)) < float(tolerance) for value in error_rad[:6])


def is_feedback_target_reached(target_rad, feedback_rad, tolerance=ARRIVAL_TOLERANCE_RAD):
    """Return True when raw feedback-minus-target errors are inside tolerance."""
    return is_target_reached(joint_errors_rad(target_rad, feedback_rad), tolerance=tolerance)


def print_joint_error(robot, target_rad, label):
    """Read current joint feedback and print only rad/deg angle errors."""
    feedback_rad = robot.get_robot_state("joint")
    if feedback_rad is None:
        beauty_print(f"{label}：无法读取关节反馈，跳过误差打印", type="warning")
        return False

    report = build_joint_error_report(target_rad, feedback_rad)
    beauty_print(f"{label} 角度误差 (rad, 反馈-目标): {beauty_print_array(report['error_rad'], precision=4)}", type="info")
    beauty_print(f"{label} 角度误差 (deg, 反馈-目标): {beauty_print_array(report['error_deg'], precision=4)}", type="info")
    return is_feedback_target_reached(target_rad, feedback_rad)


def describe_clipping(label, clipped_indexes, target_rad):
    """Print a concise warning when a target was clipped."""
    if not clipped_indexes:
        return
    joints = ", ".join(f"J{index}" for index in clipped_indexes)
    target_deg = [round(math.degrees(value), 2) for value in target_rad]
    beauty_print(
        f"{label}：{joints} 已按底层安全限位裁剪，实际发送目标 (deg): {target_deg}",
        type="warning",
    )


def create_limited_robot(port, safe_limits_rad, control_mode=DEFAULT_CONTROL_MODE):
    """Create a robot whose SDK-side limits match this demo's safe firmware limits."""
    lower_limits = [lower for lower, _upper in safe_limits_rad]
    upper_limits = [upper for _lower, upper in safe_limits_rad]
    config = alicia_m_sdk.RobotConfig(
        port=port,
        control_mode=control_mode,
        joint_limits_lower=lower_limits,
        joint_limits_upper=upper_limits,
    )
    robot = alicia_m_sdk.SynriaRobotAPI(config)
    robot.connect()
    return robot


def wait_for_joint_target(
    robot,
    target_rad,
    timeout=ARRIVAL_TIMEOUT_SECONDS,
    tolerance=ARRIVAL_TOLERANCE_RAD,
):
    """Wait for J1-J6 only, so a grasped object never blocks arrival."""
    deadline = time.perf_counter() + float(timeout)
    while time.perf_counter() < deadline:
        feedback_rad = robot.get_robot_state("joint")
        if feedback_rad is not None and is_feedback_target_reached(
            target_rad,
            feedback_rad,
            tolerance=tolerance,
        ):
            return True
        time.sleep(ARRIVAL_POLL_INTERVAL_SECONDS)
    return False


def prepare_grasp_test(robot, close_speed=DEFAULT_GRIPPER_CLOSE_SPEED, prompt=input):
    """Start ForceGrasp, then wait for the operator to confirm the load is secure."""
    close_started = robot.set_robot_state(
        target_joints=None,
        gripper_value=GRIPPER_CLOSE_TARGET,
        gripper_speed=close_speed,
        wait_for_completion=False,
    )
    if not close_started:
        raise RuntimeError("Failed to send the gripper close command")
    prompt("Wait for a secure grasp, then press Enter to start the fixed path...")


def move_and_report(
    robot,
    target_rad,
    label,
    speed,
    safe_limits_rad,
    grasp_test=False,
    gripper_close_speed=DEFAULT_GRIPPER_CLOSE_SPEED,
):
    """Send a safe target, then print the explicit 0.008 rad error judgement."""
    clipped_target, clipped_indexes = clamp_joint_targets_rad(target_rad, safe_limits_rad)
    describe_clipping(label, clipped_indexes, clipped_target)

    beauty_print(f"{label}: {beauty_print_array(clipped_target, precision=4)} (rad)", type="info")
    if grasp_test:
        command_sent = robot.set_robot_state(
            target_joints=clipped_target,
            gripper_value=GRIPPER_CLOSE_TARGET,
            joint_format="rad",
            speed=speed,
            gripper_speed=gripper_close_speed,
            wait_for_completion=False,
            use_interpolation=True,
        )
        sdk_reached = command_sent and wait_for_joint_target(robot, clipped_target)
    else:
        sdk_reached = robot.set_robot_state(
            target_joints=clipped_target,
            joint_format="rad",
            speed=speed,
            wait_for_completion=True,
            use_interpolation=True,
        )
    if not sdk_reached:
        beauty_print(f"{label}：SDK 等待到位返回 False，将继续读取反馈误差", type="warning")

    reached = print_joint_error(robot, clipped_target, label)
    if reached:
        beauty_print(f"{label}：0.008 rad 误差判定通过", type="success")
    else:
        beauty_print(f"{label}：0.008 rad 误差判定未通过", type="warning")
    return reached


def build_arg_parser():
    """Build the CLI parser for demo 22."""
    parser = argparse.ArgumentParser(description="Move Alicia-M joints in PV or MIT mode with firmware safe-limit clipping.")
    parser.add_argument(
        "--speed", type=float, default=15,
        help="Motion speed; default 15, range 0-400.",
    )
    parser.add_argument(
        "--limit-profile",
        choices=sorted(LIMIT_PROFILES.keys()),
        default=DEFAULT_LIMIT_PROFILE,
        help="Joint hard-limit profile; default standard.",
    )
    parser.add_argument(
        "--control-mode",
        choices=("pv", "mit"),
        default=DEFAULT_CONTROL_MODE,
        help="J1-J6 control mode; default pv.",
    )
    parser.add_argument(
        "--grasp-test",
        action="store_true",
        help="Close J7 before the fixed path and preserve its close target during motion.",
    )
    parser.add_argument(
        "--gripper-close-speed",
        type=float,
        default=DEFAULT_GRIPPER_CLOSE_SPEED,
        help="Gripper close speed for --grasp-test; default 40, range 0-400.",
    )
    add_port_argument(parser)
    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    safe_limits_rad = build_safe_joint_limits(LIMIT_PROFILES[args.limit_profile])

    requested_mode = args.control_mode.lower()
    mode_label = requested_mode.upper()

    beauty_print(f"Demo 22: 带底层限位裁剪的关节控制 ({mode_label})", type="module")
    beauty_print(f"限位配置: {args.limit_profile}", type="info")
    beauty_print(f"连接时将检查并在必要时把 M0-M5 统一为 {mode_label} 模式", type="warning")
    beauty_print("模式同步会短暂失能，机械臂可能因重力下坠", type="warning")
    input(f"确认机械臂已固定，按 Enter 连接并同步 {mode_label} 模式...")
    robot = create_limited_robot(args.port, safe_limits_rad, control_mode=requested_mode)
    beauty_print(f"机器人连接成功（当前 {robot.control_mode.value.upper()} 模式）", type="success")

    if robot.control_mode.value != requested_mode:
        robot.disconnect()
        raise RuntimeError(
            f"22demo 需要 {mode_label} 模式，模式同步未成功，已停止测试流程"
        )

    try:
        if args.grasp_test:
            beauty_print("夹持测试已启用：J7 将保持闭合目标，结束时不会自动开爪", type="warning")
            prepare_grasp_test(robot, close_speed=args.gripper_close_speed)

        for index, waypoint_deg in enumerate(TEST_FLOW_DEG, start=1):
            target_rad = deg_to_rad_list(waypoint_deg)
            label = f"测试点 {index}/{len(TEST_FLOW_DEG)}"
            beauty_print(f"{label} 原始目标 (deg): {waypoint_deg}", type="info")
            move_and_report(
                robot,
                target_rad,
                label,
                args.speed,
                safe_limits_rad,
                grasp_test=args.grasp_test,
                gripper_close_speed=args.gripper_close_speed,
            )
            if index < len(TEST_FLOW_DEG):
                time.sleep(1.0)

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
