"""18_demo_user_settings.py - 个性化设置读写示例。"""

import argparse

import alicia_m_sdk
from _demo_helpers import (
    SETTING_NAMES,
    add_port_argument,
    beauty_print,
    gripper_type_label,
    gripper_type_option_label,
    normalize_gripper_type,
)


def main():
    beauty_print("Demo: 0x02 个性化设置", type="module")

    parser = argparse.ArgumentParser(description="Read/write Alicia-M user settings with command 0x02.")
    add_port_argument(parser)
    parser.add_argument(
        "--gripper-type",
        choices=["10", "40"],
        metavar="{10,40}",
        default=None,
        help="Write gripper type: 10 for small gripper, 40 for large gripper.",
    )
    parser.add_argument("--timeout", type=float, default=1.0,
                        help="Timeout for command 0x02 response, in seconds.")
    parser.add_argument("--write-timeout", type=float, default=3.0,
                        help="Timeout for write response, in seconds.")
    parser.add_argument("--yes", "-y", action="store_true",
                        help="Skip interactive confirmation before writing.")
    parser.add_argument("--skip-readback", action="store_true",
                        help="Do not read all user settings after writing.")
    args = parser.parse_args()

    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print("机器人连接成功", type="success")

    try:
        if args.gripper_type is not None:
            gripper_type = normalize_gripper_type(args.gripper_type)
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
                    for index, value in enumerate(settings.values):
                        name = SETTING_NAMES[index] if index < len(SETTING_NAMES) else f"配置项 {index}"
                        extra = gripper_type_label(value) if index == 1 else str(value)
                        beauty_print(f"  {name}: {extra}", type="info")
                    confirmed = robot.confirm_gripper_type(settings, gripper_type)
                    beauty_print(
                        f"夹爪类型读回{'已确认' if confirmed else '未确认'}: {gripper_type_label(settings.values[1])}",
                        type="success" if confirmed else "warning",
                    )
                else:
                    beauty_print("未收到 0x02 个性化设置响应", type="warning")
        else:
            beauty_print("读取全部个性化设置...", type="module")
            settings = robot.get_user_settings(timeout=args.timeout)
            if settings is not None:
                for index, value in enumerate(settings.values):
                    name = SETTING_NAMES[index] if index < len(SETTING_NAMES) else f"配置项 {index}"
                    extra = gripper_type_label(value) if index == 1 else str(value)
                    beauty_print(f"  {name}: {extra}", type="info")
            else:
                beauty_print("未收到 0x02 个性化设置响应", type="warning")

    except KeyboardInterrupt:
        beauty_print("\n用户取消", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
