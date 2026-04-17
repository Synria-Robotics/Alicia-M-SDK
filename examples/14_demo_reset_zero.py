"""14_demo_reset_zero.py — 零位标定

流程:
1. 连接机械臂，运行本脚本
2. 提示用户通过机械臂按键切换到 MIT 模式（非重力补偿）
3. 用户手动将机械臂移动到期望零点位置，夹爪闭合
4. 用户通过机械臂按键切换回 PV 模式
5. 用户按 Enter 确认
6. SDK 发送零位标定指令（0x03）
"""

import argparse
import time

import alicia_m_sdk
from demo_common import add_port_argument
from robocore.utils.beauty_logger import beauty_print


try:
    import msvcrt
except ImportError:
    msvcrt = None


PRINT_INTERVAL = 0.2


def _read_key():
    if msvcrt is None or not msvcrt.kbhit():
        return None

    key = msvcrt.getwch()
    if key in ("\x00", "\xe0"):
        if msvcrt.kbhit():
            msvcrt.getwch()
        return None
    return key.lower()


def _format_values(values, precision=4):
    if values is None:
        return "N/A"
    return "[" + ", ".join(f"{value:.{precision}f}" for value in values) + "]"


def _print_state(robot):
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


def main():
    beauty_print("Demo: 零位标定", type="module")

    parser = argparse.ArgumentParser(description="Reset Alicia-M zero position.")
    add_port_argument(parser)
    args = parser.parse_args()

    # 创建并连接机器人
    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print("机器人连接成功", type="success")

    try:
        # --- 引导用户手动调零 ---
        beauty_print("=" * 50)
        beauty_print("零位标定操作步骤:", type="module")
        beauty_print("  1. 通过机械臂按键切换到 MIT 模式（非重力补偿）", type="info")
        beauty_print("  2. 手动将机械臂各关节移动到期望的零点位置", type="info")
        beauty_print("  3. 确保夹爪完全闭合", type="info")
        beauty_print("  4. 通过机械臂按键切换回 PV 模式", type="info")
        beauty_print("=" * 50)

        beauty_print("运行中会持续打印 pos / vel / tor", type="info")
        beauty_print("按 Q 发送弱调零，按 P 发送强调零，按 Ctrl+C 退出", type="info")

        last_print = 0.0
        while True:
            key = _read_key()
            # 功能实现中
            # if key == "q":
            #     beauty_print("正在发送弱调零指令...", type="info")
            #     robot.set_zero_position(mode="weak")
            #     beauty_print("弱调零完成", type="success")
            # elif 
            if key == "p":
                beauty_print("正在发送强调零指令...", type="info")
                robot.set_zero_position(mode="strong")
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
    main()
