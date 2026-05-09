"""13_demo_reset_zero.py - 零位标定示例。

运行期间会持续打印 pos / vel / tor。
P 键强调零；固件版本 1.0.6 及以上时开放 Q 键弱调零。
"""

import argparse
import sys
import time

import alicia_m_sdk
from alicia_m_sdk.utils.beauty_logger import beauty_print
from alicia_m_sdk.utils.version import supports_min_version
from _common import add_port_argument


PRINT_INTERVAL = 0.2
MIN_WEAK_ZERO_VERSION = (1, 0, 6)

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
    """跨平台非阻塞单键读取器。"""

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
        """非阻塞读取一个按键；没有按键时返回 None。"""
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


def print_robot_state(robot):
    """打印当前关节位置、速度和力矩。"""
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


def format_values(values, precision=4):
    """格式化状态数组，便于连续打印。"""
    if values is None:
        return "N/A"
    return "[" + ", ".join(f"{value:.{precision}f}" for value in values) + "]"


def main():
    beauty_print("Demo: 零位标定", type="module")

    parser = argparse.ArgumentParser(description="Alicia-M 零位标定示例")
    add_port_argument(parser)
    args = parser.parse_args()

    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print("机器人连接成功", type="success")

    firmware_version = robot.get_firmware_version(timeout=1.0)
    weak_zero_enabled = supports_min_version(firmware_version, MIN_WEAK_ZERO_VERSION)

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
                        robot.set_zero_position(mode="weak")
                        beauty_print("弱调零完成", type="success")
                    else:
                        beauty_print("弱调零暂不开放；需要固件版本 1.0.6 及以上。", type="warning")
                elif key == "p":
                    beauty_print("正在发送强调零指令...", type="info")
                    robot.set_zero_position(mode="strong")
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
