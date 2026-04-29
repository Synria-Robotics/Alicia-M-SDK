"""04_demo_switch_mode.py — 切换控制模式

演示 PV / MIT 模式之间的切换（仅关节 M0-M5，夹爪 M6 固件锁定 MIT）。
连接后自动检测当前模式，按 Enter 切换到另一种模式，再按 Enter 切回。

注意: 模式切换瞬间固件会短暂失能再使能，机械臂会因重力瞬间下坠。
      切换到 MIT 后关节可自由活动（无力矩锁定），夹爪始终为 MIT 模式。
"""

import argparse

import alicia_m_sdk
from alicia_m_sdk import ControlMode
from alicia_m_sdk.demo_utils.demo_common import add_port_argument
from robocore.utils.beauty_logger import beauty_print


def main():
    beauty_print("Demo: 切换控制模式 (PV <-> MIT)", type="module")

    parser = argparse.ArgumentParser(description="Switch Alicia-M control mode.")
    add_port_argument(parser)
    args = parser.parse_args()

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
        if other == ControlMode.MIT:
            beauty_print("切换到 MIT 模式时机械臂会短暂卸力下坠!", type="warning")
            beauty_print("  切换后关节可自由活动（无力矩锁定）", type="warning")
        else:
            beauty_print("切回 PV 模式时机械臂会短暂卸力后重新锁定!", type="warning")
        beauty_print("  切换过程中机械臂将短暂失能，可能因重力下坠", type="warning")
        input(f"\n确认安全后按 Enter 切换到 {other.value.upper()} 模式...")

        beauty_print(f"正在切换到 {other.value.upper()} 模式...", type="info")
        result = robot.switch_mode(other.value)
        if result:
            beauty_print(f"已切换到 {other.value.upper()} 模式", type="success")
        else:
            beauty_print(f"切换到 {other.value.upper()} 模式失败", type="warning")

        # --- 切换回原模式 ---
        beauty_print("", type="info")
        if current == ControlMode.MIT:
            beauty_print("切换到 MIT 模式时机械臂会短暂卸力下坠!", type="warning")
        else:
            beauty_print("切回 PV 模式时机械臂会短暂卸力后重新锁定!", type="warning")
        input(f"\n确认安全后按 Enter 切换回 {current.value.upper()} 模式...")

        beauty_print(f"正在切换回 {current.value.upper()} 模式...", type="info")
        result = robot.switch_mode(current.value)
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
