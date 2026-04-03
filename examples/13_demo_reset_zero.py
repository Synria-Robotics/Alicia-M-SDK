"""13_demo_reset_zero.py — 零位标定

流程:
1. 连接机械臂，运行本脚本
2. 提示用户通过机械臂按键切换到 MIT 模式（非重力补偿）
3. 用户手动将机械臂移动到期望零点位置，夹爪闭合
4. 用户通过机械臂按键切换回 PV 模式
5. 用户按 Enter 确认
6. SDK 发送零位标定指令（0x03）
"""

import alicia_m_sdk
from robocore.utils.beauty_logger import beauty_print


def main():
    beauty_print("Demo: 零位标定", type="module")

    # 创建并连接机器人
    robot = alicia_m_sdk.create_robot()
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

        input("\n完成以上步骤后，按 Enter 发送零位标定指令...")

        # --- 发送零位标定指令 ---
        beauty_print("正在发送零位标定指令...", type="info")
        robot.set_zero_position()
        beauty_print("零位标定完成！当前位置已设为新零点", type="success")

    except KeyboardInterrupt:
        beauty_print("\n用户取消", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
