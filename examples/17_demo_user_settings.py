"""17_demo_user_settings.py — 个性化设置。

默认读取全部个性化设置；可通过 10 或 40 写入夹爪类型。
"""

import argparse
import time

import alicia_m_sdk
from alicia_m_sdk.demo_utils.demo_common import add_port_argument
from alicia_m_sdk.demo_utils.user_settings_support import (
    confirm_gripper_type,
    gripper_type_option_label,
    gripper_type_label,
    make_read_settings_frame,
    make_write_gripper_type_frame,
    parse_gripper_type,
    print_settings_response,
    print_write_response,
    send_user_settings_frame,
)
from robocore.utils.beauty_logger import beauty_print


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

            response = send_user_settings_frame(
                robot,
                make_write_gripper_type_frame(gripper_type),
                args.write_timeout,
                warn_on_timeout=args.skip_readback,
            )
            if response is not None:
                print_write_response(response)
            elif not args.skip_readback:
                beauty_print("未收到写入确认，继续通过读回确认配置。", type="info")

            if not args.skip_readback:
                time.sleep(0.2)
                beauty_print("写入后读回全部个性化设置...", type="module")
                response = send_user_settings_frame(robot, make_read_settings_frame(), args.timeout)
                if response is not None:
                    print_settings_response(response)
                    confirm_gripper_type(response, gripper_type)
        else:
            beauty_print("读取全部个性化设置...", type="module")
            response = send_user_settings_frame(robot, make_read_settings_frame(), args.timeout)
            if response is not None:
                print_settings_response(response)

    except KeyboardInterrupt:
        beauty_print("\n用户取消", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
