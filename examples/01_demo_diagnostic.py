"""01_demo_diagnostic.py — 自检功能

演示 Alicia-M 固件 1.0.6 的自检功能调用方式。
"""

import argparse

import alicia_m_sdk
from alicia_m_sdk.demo_utils.demo_common import add_port_argument
from alicia_m_sdk.demo_utils.diagnostic_support import (
    print_diagnostic_response,
    send_diagnostic,
    supports_diagnostic,
)
from robocore.utils.beauty_logger import beauty_print


def main():
    beauty_print("Demo: 自检功能（固件 1.0.6）", type="module")

    parser = argparse.ArgumentParser(description="Run Alicia-M diagnostic demo.")
    add_port_argument(parser)
    parser.add_argument(
        "--timeout",
        type=float,
        default=1.0,
        help="等待自检响应的超时时间（秒）。",
    )
    args = parser.parse_args()

    # 创建并连接机器人
    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print("机器人连接成功", type="success")

    try:
        firmware_version = robot.get_firmware_version(timeout=args.timeout)
        if not supports_diagnostic(firmware_version):
            beauty_print(
                f"当前固件版本为 {firmware_version or '未知'}，自检功能仅支持 1.0.6",
                type="warning",
            )
            return

        beauty_print("正在执行自检...", type="info")
        response = send_diagnostic(robot, args.timeout)
        if response is not None and print_diagnostic_response(response):
            beauty_print("自检完成", type="success")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
