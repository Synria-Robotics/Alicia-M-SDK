"""18_demo_user_settings.py — 个性化设置。

默认读取全部个性化设置；可通过 10 或 40 写入夹爪类型。
"""

import argparse

import alicia_m_sdk
from alicia_m_sdk.user_settings import (
    GRIPPER_TYPE_OPTION_LABELS,
    GRIPPER_TYPE_OPTIONS,
    SETTING_NAMES,
    UserSettings,
    gripper_type_config_value,
    gripper_type_label,
    normalize_gripper_type,
)
from alicia_m_sdk.utils.beauty_logger import beauty_print
from _common import add_port_argument


def print_settings_response(settings: UserSettings):
    """打印读取个性化设置的响应。"""
    for index, value in enumerate(settings.values):
        name = SETTING_NAMES[index] if index < len(SETTING_NAMES) else f"配置项{index}"
        extra = gripper_type_label(value) if index == 1 else str(value)
        beauty_print(f"  {name}: {extra}", type="info")


def confirm_gripper_type(settings: UserSettings, expected_value: int) -> bool:
    """通过读回配置确认夹爪类型是否已写入。"""
    if len(settings.values) < 2:
        beauty_print("无法通过读回确认夹爪类型配置", type="warning")
        return False

    actual_value = settings.values[1]
    ok = gripper_type_config_value(actual_value) == gripper_type_config_value(expected_value)
    msg_type = "success" if ok else "warning"
    beauty_print(
        f"夹爪类型写入{'已确认' if ok else '未确认'}: {gripper_type_label(actual_value)}",
        type=msg_type,
    )
    return ok


def parse_gripper_type(text: str) -> int:
    """解析并限制夹爪类型配置值。"""
    try:
        return normalize_gripper_type(text)
    except ValueError as exc:
        choices = ", ".join(str(item) for item in sorted(GRIPPER_TYPE_OPTIONS))
        raise ValueError(f"夹爪类型只支持: {choices}") from exc


def gripper_type_option_label(text: str) -> str:
    """返回用户输入选项对应的显示文本。"""
    option = int(text, 0)
    value = GRIPPER_TYPE_OPTIONS[option]
    return f"{option} ({GRIPPER_TYPE_OPTION_LABELS[option]} -> 配置值 {value})"


def main():
    beauty_print("Demo: 0x02 个性化设置", type="module")

    parser = argparse.ArgumentParser(description="Read/write Alicia-M user settings with command 0x02.")
    add_port_argument(parser)
    parser.add_argument(
        "--gripper-type",
        choices=["10", "40"],
        metavar="{10,40}",
        default=None,
        help="写入夹爪类型配置：10=小夹爪，40=大夹爪。",
    )
    parser.add_argument("--timeout", type=float, default=1.0,
                        help="等待 0x02 响应超时，单位秒。")
    parser.add_argument("--write-timeout", type=float, default=3.0,
                        help="等待写入响应超时，单位秒。")
    parser.add_argument("--yes", "-y", action="store_true",
                        help="写入前不等待二次确认。")
    parser.add_argument("--skip-readback", action="store_true",
                        help="写入后不自动读回全部个性化设置。")
    args = parser.parse_args()

    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print("机器人连接成功", type="success")

    try:
        if args.gripper_type is not None:
            gripper_type = parse_gripper_type(args.gripper_type)
            beauty_print(f"即将写入夹爪类型: {gripper_type_option_label(args.gripper_type)}", type="warning")
            beauty_print("该配置会掉电保存，并影响后续夹爪动作。", type="warning")
            if not args.yes:
                input("确认机械臂安全后按 Enter 发送写入帧，Ctrl+C 取消...")

            ok = robot.set_gripper_type(
                gripper_type,
                timeout=args.write_timeout,
                readback=not args.skip_readback,
            )
            beauty_print(f"夹爪类型写入{'已确认' if ok else '未确认'}", type="success" if ok else "warning")

            if not args.skip_readback:
                beauty_print("写入后读回全部个性化设置...", type="module")
                settings = robot.get_user_settings(timeout=args.timeout)
                if settings is not None:
                    print_settings_response(settings)
                    confirm_gripper_type(settings, gripper_type)
                else:
                    beauty_print("未收到 0x02 个性化设置响应", type="warning")
        else:
            beauty_print("读取全部个性化设置...", type="module")
            settings = robot.get_user_settings(timeout=args.timeout)
            if settings is not None:
                print_settings_response(settings)
            else:
                beauty_print("未收到 0x02 个性化设置响应", type="warning")

    except KeyboardInterrupt:
        beauty_print("\n用户取消", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
