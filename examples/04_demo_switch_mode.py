"""04_demo_switch_mode.py — 切换控制模式

演示 PV / MIT 模式之间的切换。
连接时默认 PV 模式，按 Enter 切换为 MIT，再按 Enter 切回 PV。

注意: 模式切换瞬间固件会短暂失能再使能，机械臂会因重力瞬间下坠。
      切换到 MIT 后关节可自由活动（无力矩锁定）。
"""

import alicia_m_sdk
from robocore.utils.beauty_logger import beauty_print


def main():
    beauty_print("Demo: 切换控制模式 (PV <-> MIT)", type="module")

    robot = alicia_m_sdk.create_robot(control_mode="pv")
    beauty_print("机器人连接成功（PV 模式）", type="success")

    try:
        beauty_print("当前控制模式: PV（位置-速度模式）", type="info")

        # --- 切换到 MIT ---
        beauty_print("", type="info")
        beauty_print("⚠ 警告: 切换到 MIT 模式时机械臂会短暂卸力下坠!", type="warning")
        beauty_print("  切换后关节可自由活动（无力矩锁定）", type="warning")
        beauty_print("  请确保机械臂周围安全，并用手扶住机械臂", type="warning")
        input("\n确认安全后按 Enter 切换到 MIT 模式...")

        beauty_print("正在切换到 MIT 模式...", type="info")
        result = robot.switch_mode("mit")
        if result:
            beauty_print("已切换到 MIT 模式（关节可自由活动）", type="success")
        else:
            beauty_print("切换到 MIT 模式失败", type="warning")

        # --- 切换回 PV ---
        beauty_print("", type="info")
        beauty_print("⚠ 警告: 切回 PV 模式时机械臂会短暂卸力后重新锁定!", type="warning")
        input("\n确认安全后按 Enter 切换回 PV 模式...")

        beauty_print("正在切换回 PV 模式...", type="info")
        result = robot.switch_mode("pv")
        if result:
            beauty_print("已切换回 PV 模式（位置-速度模式）", type="success")
        else:
            beauty_print("切换回 PV 模式失败", type="warning")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
