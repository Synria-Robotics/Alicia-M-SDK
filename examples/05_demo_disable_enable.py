"""05_demo_disable_enable.py — 失能/使能交互

演示失能（卸载力矩，电机自由）和重新使能的流程。
使能时会在当前位置恢复控制，确保不突跳到旧位置。
"""

import argparse
import alicia_m_sdk
from alicia_m_sdk.utils.cli import add_port_argument
from alicia_m_sdk.utils.beauty_logger import beauty_print

def main():
    beauty_print("Demo: 失能/使能交互", type="module")

    parser = argparse.ArgumentParser(description="Disable and enable Alicia-M.")
    add_port_argument(parser)
    args = parser.parse_args()

    # 创建并连接机器人
    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print("机器人连接成功", type="success")

    try:
        # --- 打印当前关节角度 ---
        beauty_print("当前关节角度", type="module")
        robot.print_state(output_format="deg")

        # --- 失能：卸载力矩 ---
        beauty_print("失能后机械臂将失去力矩，可能因重力下坠", type="warning")
        input("\n按 Enter 失能（电机将失去力矩，可自由拖动）...")
        result = robot.disable_robot()
        if result:
            beauty_print("已失能，电机自由", type="success")
            beauty_print("请手动移动机械臂到新位置", type="info")
        else:
            beauty_print("失能操作失败", type="warning")

        # --- 重新使能 ---
        input("\n按 Enter 在当前位置重新使能...")
        result = robot.enable_robot()
        if result:
            beauty_print("已在当前位置使能", type="success")
        else:
            beauty_print("使能操作失败", type="warning")

        # --- 打印新的关节角度 ---
        beauty_print("新的关节角度", type="module")
        robot.print_state(output_format="deg")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
