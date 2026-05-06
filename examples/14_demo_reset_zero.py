"""14_demo_reset_zero.py - 零位标定示例。

运行期间会持续打印 pos / vel / tor。
当前只开放 P 键强调零。

注意：
弱调零需要固件版本 1.0.6 及以上，当前 demo 暂不开放弱调零入口。
"""

import argparse
import time

import alicia_m_sdk
from demo_common import add_port_argument
from alicia_m_sdk.utils.beauty_logger import beauty_print


try:
    import msvcrt
except ImportError:
    msvcrt = None


PRINT_INTERVAL = 0.2


def _read_key():
    """非阻塞读取键盘输入。"""
    if msvcrt is None or not msvcrt.kbhit():
        return None

    key = msvcrt.getwch()
    if key in ("\x00", "\xe0"):
        if msvcrt.kbhit():
            msvcrt.getwch()
        return None
    return key.lower()


def _format_values(values, precision=4):
    """格式化状态数组，便于连续打印。"""
    if values is None:
        return "N/A"
    return "[" + ", ".join(f"{value:.{precision}f}" for value in values) + "]"


def _print_state(robot):
    """打印当前关节位置、速度和力矩。"""
    state = robot.get_robot_state("all")
    if state is None:
        print("pos=N/A vel=N/A tor=N/A", flush=True)
        return

    pos = list(state.angles) + [state.gripper]
    print(
        f"pos={_format_values(pos)} "
        f"vel={_format_values(state.velocities)} "
        f"tor={_format_values(state.torques)}",
        flush=True,
    )


def _send_strong_zero_position(robot):
    """发送强调零指令，并兼容未升级的本地 SDK。"""
    robot.set_zero_position()


def main(args):
    beauty_print("Demo: 零位标定", type="module")

    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print("机器人连接成功", type="success")

    try:
        beauty_print("=" * 50)
        beauty_print("零位标定操作：", type="module")
        beauty_print("  1. 手动将机械臂移动到期望零点。", type="info")
        beauty_print("  2. 如果夹爪需要参与零位，请先闭合夹爪。", type="info")
        beauty_print("  3. 确认机械臂安全稳定后，按 P 发送强调零。", type="info")
        beauty_print("=" * 50)
        beauty_print("程序会持续打印 pos / vel / tor。", type="info")
        beauty_print("按 P 强调零，按 Ctrl+C 退出。", type="info")
        beauty_print("弱调零暂不开放；需要固件版本 1.0.6 及以上。", type="warning")

        last_print = 0.0
        while True:
            key = _read_key()
            if key == "q":
                beauty_print("弱调零暂不开放；需要固件版本 1.0.6 及以上。", type="warning")
            elif key == "p":
                beauty_print("正在发送强调零指令...", type="info")
                _send_strong_zero_position(robot)
                beauty_print("强调零完成", type="success")

            now = time.perf_counter()
            if now - last_print >= PRINT_INTERVAL:
                _print_state(robot)
                last_print = now

            time.sleep(0.02)

    except KeyboardInterrupt:
        beauty_print("\n用户取消", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Alicia-M 零位标定示例")
    add_port_argument(parser)
    args = parser.parse_args()

    main(args)
