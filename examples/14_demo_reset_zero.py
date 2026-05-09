"""14_demo_reset_zero.py - 零位标定示例。

运行期间会持续打印 pos / vel / tor。
P 键强调零；固件版本 1.0.6 及以上时开放 Q 键弱调零。
"""

import argparse
import time

import alicia_m_sdk
from alicia_m_sdk.demo_utils.demo_common import NonBlockingKeyReader, add_port_argument
from alicia_m_sdk.demo_utils.reset_zero_support import (
    PRINT_INTERVAL,
    print_robot_state,
    send_strong_zero_position,
    send_weak_zero_position,
    supports_weak_zero,
)
from alicia_m_sdk.utils.beauty_logger import beauty_print


def main():
    beauty_print("Demo: 零位标定", type="module")

    parser = argparse.ArgumentParser(description="Alicia-M 零位标定示例")
    add_port_argument(parser)
    args = parser.parse_args()

    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print("机器人连接成功", type="success")

    firmware_version = robot.get_firmware_version(timeout=1.0)
    weak_zero_enabled = supports_weak_zero(firmware_version)

    try:
        beauty_print("=" * 50)
        beauty_print("零位标定操作：", type="module")
        beauty_print("  1. 手动将机械臂移动到期望零点。", type="info")
        beauty_print("  2. 如果夹爪需要参与零位，请先闭合夹爪。", type="info")
        beauty_print("  3. 确认机械臂安全稳定后，再发送调零指令。", type="info")
        beauty_print("=" * 50)
        beauty_print("程序会持续打印 pos / vel / tor。", type="info")
        beauty_print(f"当前固件版本: {firmware_version or '未知'}", type="info")
        if weak_zero_enabled:
            beauty_print("按 Q 弱调零，按 P 强调零，按 Ctrl+C 退出。", type="info")
        else:
            beauty_print("按 P 强调零，按 Ctrl+C 退出。", type="info")
            beauty_print("弱调零暂不开放；需要固件版本 1.0.6 及以上。", type="warning")

        last_print = 0.0
        with NonBlockingKeyReader() as key_reader:
            while True:
                key = key_reader.read_key()
                if key == "q":
                    if weak_zero_enabled:
                        beauty_print("正在发送弱调零指令...", type="info")
                        send_weak_zero_position(robot)
                        beauty_print("弱调零完成", type="success")
                    else:
                        beauty_print("弱调零暂不开放；需要固件版本 1.0.6 及以上。", type="warning")
                elif key == "p":
                    beauty_print("正在发送强调零指令...", type="info")
                    send_strong_zero_position(robot)
                    beauty_print("强调零完成", type="success")

                now = time.perf_counter()
                if now - last_print >= PRINT_INTERVAL:
                    print_robot_state(robot)
                    last_print = now

                time.sleep(0.02)

    except KeyboardInterrupt:
        beauty_print("\n用户取消", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
