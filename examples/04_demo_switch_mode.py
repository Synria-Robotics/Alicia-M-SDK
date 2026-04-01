"""04_demo_switch_mode.py — 切换控制模式

演示 PV / MIT 模式之间的切换。
连接时默认 PV 模式，按 Enter 切换为 MIT，再按 Enter 切回 PV。
"""

import alicia_m_sdk
from robocore.utils.beauty_logger import beauty_print, beauty_print_array


def main():
    beauty_print("Demo: 切换控制模式 (PV <-> MIT)", type="module")

    # 以 PV 模式连接
    robot = alicia_m_sdk.create_robot(control_mode="pv")
    beauty_print("机器人连接成功（PV 模式）", type="success")

    try:
        # --- 打印当前模式 ---
        beauty_print("当前控制模式: PV（位置-速度模式）", type="info")

        # --- 切换到 MIT 模式 ---
        input("\n按 Enter 切换到 MIT 模式...")
        beauty_print("正在切换到 MIT 模式...", type="info")
        result = robot.switch_mode("mit")
        if result:
            beauty_print("已切换到 MIT 模式（阻抗控制模式）", type="success")
        else:
            beauty_print("切换到 MIT 模式失败", type="error")

        # --- 切换回 PV 模式 ---
        input("\n按 Enter 切换回 PV 模式...")
        beauty_print("正在切换回 PV 模式...", type="info")
        result = robot.switch_mode("pv")
        if result:
            beauty_print("已切换回 PV 模式（位置-速度模式）", type="success")
        else:
            beauty_print("切换回 PV 模式失败", type="error")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
