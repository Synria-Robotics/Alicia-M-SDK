"""04_demo_switch_mode.py — 切换控制模式

演示 PV / MIT 模式之间的切换（仅关节 M0-M5，夹爪 M6 固件锁定 MIT）。
连接后自动检测当前模式，按 Enter 切换到另一种模式，再按 Enter 切回。

默认不发送 SDK 全局失能/使能，也不保存 Flash。
切换到 MIT 后关节会失去 PV 位置保持，机械臂仍可能下坠。
"""

import argparse

import alicia_m_sdk
from alicia_m_sdk import ControlMode
from _demo_helpers import add_port_argument, beauty_print


def build_arg_parser():
    """Build the command-line parser without connecting to hardware."""
    parser = argparse.ArgumentParser(description="Switch Alicia-M control mode.")
    add_port_argument(parser)
    parser.add_argument(
        "--target-mode",
        choices=("pv", "mit"),
        help="Recover mixed joint modes by explicitly normalizing M0-M5.",
    )
    parser.add_argument(
        "--disable-and-save",
        action="store_true",
        help="Disable motors and save --target-mode to ESC Flash.",
    )
    return parser


def parse_args(argv=None):
    """Parse CLI arguments and reject meaningless repeated Flash writes."""
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if args.disable_and_save and not args.target_mode:
        parser.error("--disable-and-save requires --target-mode pv|mit")
    return args


def mode_switch_options(args):
    """Map the demo's single persistence flag to SDK switch options."""
    persistent = bool(args.disable_and_save)
    return {
        "disable_before_switch": persistent,
        "save_to_flash": persistent,
    }


def joint_modes_match(modes, target_mode):
    """Return whether M0-M5 all match the requested mode; ignore gripper M6."""
    expected = 0x02 if target_mode == "pv" else 0x01
    return modes is not None and len(modes) >= 6 and all(
        mode["value"] == expected for mode in modes[:6]
    )


def _print_mode_switch_warning(target_mode, disable_and_save=False):
    if target_mode == "mit":
        beauty_print("切换到 MIT 后关节失去 PV 位置保持，机械臂可能下坠", type="warning")
    if disable_and_save:
        beauty_print("将先失能机械臂，并把目标模式保存到 ESC Flash", type="warning")
        beauty_print("切换过程中机械臂可能因重力下坠", type="warning")
    else:
        beauty_print("本次不发送 SDK 全局失能/使能，也不保存 Flash", type="info")


def _recover_mixed_modes(args):
    target = args.target_mode
    options = mode_switch_options(args)
    _print_mode_switch_warning(target, args.disable_and_save)
    input(f"\n确认机械臂已固定，按 Enter 将 M0-M5 统一为 {target.upper()} 模式...")

    robot = alicia_m_sdk.create_robot(port=args.port, sync_control_mode=False)
    try:
        if not robot.switch_mode(target, **options):
            beauty_print(f"模式恢复失败：无法写入 {target.upper()} 模式", type="warning")
            return False
        modes = robot.get_robot_state("control_mode")
        if not joint_modes_match(modes, target):
            beauty_print(f"模式恢复失败：M0-M5 未全部进入 {target.upper()} 模式", type="warning")
            return False
        beauty_print(f"模式恢复成功：M0-M5 均为 {target.upper()}，夹爪保持 MIT", type="success")
        return True
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


def main():
    beauty_print("Demo: 切换控制模式 (PV <-> MIT)", type="module")
    args = parse_args()
    switch_options = mode_switch_options(args)

    if args.target_mode:
        _recover_mixed_modes(args)
        return

    # 自动检测固件当前模式（不强制指定）
    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print("机器人连接成功", type="success")

    try:
        # --- 显示当前模式 ---
        current = robot.control_mode
        other = ControlMode.MIT if current == ControlMode.PV else ControlMode.PV
        beauty_print(f"当前控制模式: {current.value.upper()}", type="info")

        # --- 切换到另一种模式 ---
        beauty_print("", type="info")
        _print_mode_switch_warning(other.value)
        input(f"\n确认安全后按 Enter 切换到 {other.value.upper()} 模式...")

        beauty_print(f"正在切换到 {other.value.upper()} 模式...", type="info")
        result = robot.switch_mode(other.value, **switch_options)
        if result:
            beauty_print(f"已切换到 {other.value.upper()} 模式", type="success")
        else:
            beauty_print(f"切换到 {other.value.upper()} 模式失败", type="warning")

        # --- 切换回原模式 ---
        beauty_print("", type="info")
        _print_mode_switch_warning(current.value)
        input(f"\n确认安全后按 Enter 切换回 {current.value.upper()} 模式...")

        beauty_print(f"正在切换回 {current.value.upper()} 模式...", type="info")
        result = robot.switch_mode(current.value, **switch_options)
        if result:
            beauty_print(f"已切换回 {current.value.upper()} 模式", type="success")
        else:
            beauty_print(f"切换回 {current.value.upper()} 模式失败", type="warning")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
