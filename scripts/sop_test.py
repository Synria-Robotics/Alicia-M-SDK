#!/usr/bin/env python3
"""@file sop_test.py
@brief Alicia-M 出货 SOP 测试命令行入口。

@details
该脚本面向产线和硬件在环测试人员。它只负责解析命令行参数、
构造 `SopConfig`，然后调用 `run_sop()` 执行实际流程。
所有安全判定、运动控制和报告生成逻辑都在 `alicia_m_sdk.sop` 模块中，
避免 CLI 与核心业务逻辑耦合。
"""

from __future__ import annotations

import argparse
import sys

from alicia_m_sdk.sop import SopConfig, run_sop


def _float_list(text: str, expected_len: int, name: str) -> list[float]:
    """@brief 解析固定长度的逗号分隔浮点数列表。

    @param text 命令行传入的逗号分隔字符串，例如 ``"0,0,0,0,0,0"``。
    @param expected_len 期望元素数量。
    @param name 参数名称，用于构造错误提示。
    @return 解析后的浮点数列表。
    @throws argparse.ArgumentTypeError 当元素数量不符合要求时抛出。
    """
    values = [float(item.strip()) for item in text.split(",") if item.strip()]
    if len(values) != expected_len:
        raise argparse.ArgumentTypeError(f"{name} must contain {expected_len} comma-separated numbers")
    return values


def build_parser() -> argparse.ArgumentParser:
    """@brief 构造 SOP 命令行参数解析器。

    @return argparse.ArgumentParser 已配置好的参数解析器。
    """
    parser = argparse.ArgumentParser(description="Run Alicia-M production SOP tests.")
    parser.add_argument("--port", type=str, default="", help="Serial port, for example COM37.")
    parser.add_argument(
        "--profile",
        type=str,
        choices=["quick", "full", "aging"],
        default="quick",
        help="SOP profile to run.",
    )
    parser.add_argument("--serial", type=str, default="", help="Production serial number.")
    parser.add_argument("--operator", type=str, default="", help="Operator name or station ID.")
    parser.add_argument("--output-dir", type=str, default="logs/sop", help="Report output directory.")
    parser.add_argument("--speed", type=float, default=15.0, help="Conservative motion speed.")
    parser.add_argument("--max-safe-speed", type=float, default=40.0, help="Hard speed ceiling for SOP.")
    parser.add_argument(
        "--zero-joints-deg",
        type=lambda text: _float_list(text, 6, "zero-joints-deg"),
        default=[0.0] * 6,
        help="Expected fixture zero joints in degrees, comma-separated.",
    )
    parser.add_argument(
        "--zero-tolerances-deg",
        type=lambda text: _float_list(text, 6, "zero-tolerances-deg"),
        default=[1.0] * 6,
        help="Per-joint zero tolerances in degrees, comma-separated.",
    )
    parser.add_argument(
        "--zero-gripper",
        type=float,
        default=0.0,
        help="Expected fixture gripper value. Use --skip-zero-gripper to omit this check.",
    )
    parser.add_argument(
        "--skip-zero-gripper",
        action="store_true",
        help="Do not compare gripper value during zero-position check.",
    )
    parser.add_argument("--zero-gripper-tolerance", type=float, default=80.0)
    parser.add_argument(
        "--motion-delta-deg",
        type=lambda text: _float_list(text, 6, "motion-delta-deg"),
        default=[3.0, -3.0, 3.0, 0.0, 3.0, 0.0],
        help="Small-range test motion delta in degrees, comma-separated.",
    )
    parser.add_argument("--motion-tolerance-deg", type=float, default=2.0)
    parser.add_argument(
        "--max-motion-delta-deg",
        type=float,
        default=10.0,
        help="Hard absolute limit for each small-range motion delta.",
    )
    parser.add_argument(
        "--gripper-targets",
        type=str,
        default="1000,0,500",
        help="Comma-separated gripper targets for quick/full checks.",
    )
    parser.add_argument("--gripper-tolerance", type=float, default=120.0)
    parser.add_argument("--diagnostic-timeout", type=float, default=3.0)
    parser.add_argument("--state-timeout", type=float, default=3.0)
    parser.add_argument("--aging-cycles", type=int, default=None)
    parser.add_argument("--duration-min", type=float, default=None, help="Aging duration in minutes.")
    parser.add_argument("--aging-interval-s", type=float, default=0.5)
    parser.add_argument("--max-temperature-c", type=float, default=75.0)
    parser.add_argument("--max-abs-torque", type=float, default=None)
    parser.add_argument(
        "--require-confirmation",
        action="store_true",
        help="Ask the operator to type YES before each motion step.",
    )
    parser.add_argument(
        "--keep-enabled-on-exit",
        action="store_true",
        help="Do not automatically disable the robot before disconnecting.",
    )
    return parser


def config_from_args(args: argparse.Namespace) -> SopConfig:
    """@brief 将命令行参数转换为 SOP 配置。

    @param args argparse 解析后的命名空间。
    @return SopConfig 可直接传给 `run_sop()` 的配置对象。
    """
    gripper_targets = [float(item.strip()) for item in args.gripper_targets.split(",") if item.strip()]
    return SopConfig(
        port=args.port,
        profile=args.profile,
        serial=args.serial,
        operator=args.operator,
        output_dir=args.output_dir,
        speed=args.speed,
        max_safe_speed=args.max_safe_speed,
        zero_joints_deg=args.zero_joints_deg,
        zero_tolerances_deg=args.zero_tolerances_deg,
        zero_gripper=None if args.skip_zero_gripper else args.zero_gripper,
        zero_gripper_tolerance=args.zero_gripper_tolerance,
        motion_delta_deg=args.motion_delta_deg,
        max_motion_delta_deg=args.max_motion_delta_deg,
        motion_tolerance_deg=args.motion_tolerance_deg,
        gripper_targets=gripper_targets,
        gripper_tolerance=args.gripper_tolerance,
        diagnostic_timeout=args.diagnostic_timeout,
        state_timeout=args.state_timeout,
        aging_cycles=args.aging_cycles,
        aging_duration_min=args.duration_min,
        aging_interval_s=args.aging_interval_s,
        max_temperature_c=args.max_temperature_c,
        max_abs_torque=args.max_abs_torque,
        require_confirmation=args.require_confirmation,
        disable_on_exit=not args.keep_enabled_on_exit,
    )


def main(argv: list[str] | None = None) -> int:
    """@brief CLI 主函数。

    @param argv 可选参数列表；为 None 时由调用方传入 `sys.argv[1:]`。
    @return 进程退出码，PASS 返回 0，FAIL 返回 1。
    """
    parser = build_parser()
    args = parser.parse_args(argv)
    report = run_sop(config_from_args(args))

    print(f"SOP result: {report.status}")
    print(f"Profile: {report.profile}")
    print(f"Serial: {report.serial}")
    for kind, path in report.report_paths.items():
        print(f"{kind}: {path}")
    failed = [step for step in report.steps if step.status == "FAIL"]
    if failed:
        print("Failed steps:")
        for step in failed:
            print(f"  - {step.name}: {step.error}")
    return 0 if report.status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
