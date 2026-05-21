"""01_demo_diagnostic.py - 自检命令示例。"""

import argparse

import alicia_m_sdk
from _demo_helpers import add_port_argument, beauty_print, format_bytes, supports_diagnostic


def main():
    beauty_print("Demo: 自检功能", type="module")

    parser = argparse.ArgumentParser(description="Run Alicia-M diagnostic demo.")
    add_port_argument(parser)
    parser.add_argument(
        "--timeout",
        type=float,
        default=3.0,
        help="Maximum time to wait for a diagnostic response, in seconds.",
    )
    args = parser.parse_args()

    robot = alicia_m_sdk.create_robot(port=args.port, sync_control_mode=False)
    beauty_print("机器人连接成功", type="success")

    try:
        version = robot.get_firmware_version(timeout=args.timeout)
        beauty_print(f"固件版本: {version}", type="info")

        if not supports_diagnostic(version):
            beauty_print(
                f"固件版本 {version} 不支持自检功能，需 v1.0.6 或以上",
                type="warning",
            )
            return

        beauty_print("正在发送自检帧...", type="info")
        result = robot.run_diagnostic(timeout=args.timeout)
        if result.frame is None:
            beauty_print("自检超时，未收到响应", type="warning")
            return
        beauty_print(f"RX: {format_bytes(result.frame.encode())}", type="info")

        ok = robot.print_diagnostic_response(result)
        if ok:
            beauty_print("自检完成", type="success")
        else:
            beauty_print("自检返回异常响应", type="warning")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
