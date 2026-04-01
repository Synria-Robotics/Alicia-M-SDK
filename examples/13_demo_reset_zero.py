"""13_demo_reset_zero.py — 零位标定

演示零位标定流程（MIT 模式）：
  连接 MIT -> 卸载力矩 -> 用户手动摆到零位 -> 执行标定 -> 恢复力矩。

警告: 此操作将永久更改零位，无法恢复出厂零位！
      如需恢复，需要购买校准工具。
"""

import alicia_m_sdk
from robocore.utils.beauty_logger import beauty_print, beauty_print_array


def main():
    beauty_print("Demo: 零位标定 (MIT 模式)", type="module")

    # 以 MIT 模式连接（零位标定需要力矩控制能力）
    robot = alicia_m_sdk.create_robot(control_mode="mit")
    beauty_print("机器人连接成功（MIT 模式）", type="success")

    try:
        # --- 安全警告 ---
        beauty_print("=" * 60, type="warning")
        beauty_print("  警告: 此操作将永久更改零位!", type="warning")
        beauty_print("  一旦设置新零位，无法恢复至出厂零位!", type="warning")
        beauty_print("  如需恢复，需要购买校准工具!", type="warning")
        beauty_print("=" * 60, type="warning")
        beauty_print("请用手扶住机械臂，防止卸力后坠落", type="warning")

        # --- 卸载力矩 ---
        input("\n按 Enter 卸载力矩（Kp=0, Kd=0）...")
        beauty_print("正在卸载力矩...", type="info")
        robot.torque_control('off')
        beauty_print("力矩已卸载，电机可自由运动", type="success")

        # --- 用户摆到零位 ---
        beauty_print("请手动将机械臂摆到目标零位位置", type="info")
        beauty_print("确认所有关节都在正确的零位后，按 Enter 继续", type="info")
        input("\n按 Enter 执行零位标定...")

        # --- 执行零位标定 ---
        beauty_print("正在执行零位标定...", type="info")
        result = robot.set_zero_position()
        if result:
            beauty_print("零位标定成功!", type="success")
        else:
            beauty_print("零位标定失败!", type="error")

        # --- 恢复力矩 ---
        beauty_print("正在恢复力矩...", type="info")
        robot.torque_control('on')
        beauty_print("力矩已恢复", type="success")

    except KeyboardInterrupt:
        beauty_print("\n用户中断，操作已取消", type="warning")
        # 安全恢复: 尝试恢复力矩
        try:
            robot.torque_control('on')
            beauty_print("已安全恢复力矩", type="info")
        except Exception:
            beauty_print("恢复力矩失败，请手动检查机械臂状态", type="error")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
