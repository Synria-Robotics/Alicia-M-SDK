"""06_demo_move_gripper.py — 夹爪控制

演示夹爪的打开、关闭、半开操作。
夹爪电机 M6 固件锁定 MIT 模式，无须切换关节控制模式，PV 和 MIT 下均可运行。

MIT 阻抗参数仅影响夹爪电机（列表第 7 个元素），关节部分保持当前位置不动。
"""

import argparse
import time
import alicia_m_sdk
from _demo_helpers import add_port_argument, beauty_print, beauty_print_array

# MIT 默认阻抗参数（逐电机: M0~M5 关节, M6 夹爪）
MIT_KP = [150.0, 150.0, 150.0, 150.0, 150.0, 150.0, 150.0]
MIT_KD = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
MIT_TORQUE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
MIT_VEL_REF = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def main():
    beauty_print("Demo: 夹爪控制", type="module")

    parser = argparse.ArgumentParser(description="Move Alicia-M gripper.")
    add_port_argument(parser)
    args = parser.parse_args()

    # 创建并连接机器人（无须指定模式，夹爪电机固件锁定 MIT）
    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print(f"机器人连接成功（当前 {robot.control_mode.value.upper()} 模式）", type="success")

    try:
        beauty_print("初始化 MIT 阻抗增益（读取当前 Kp/Kd 并线性过渡）...", type="info")
        robot.initialize_mit_gains(
            kp=MIT_KP,
            kd=MIT_KD,
            torque=MIT_TORQUE,
            vel_ref=MIT_VEL_REF,
        )

        # --- 半开夹爪 (500) ---
        beauty_print("移动夹爪到半开 (500)...", type="info")
        robot.set_robot_state(
            gripper_value=500, gripper_speed=100,
            wait_for_completion=True,
            kp=MIT_KP, kd=MIT_KD, torque=MIT_TORQUE, vel_ref=MIT_VEL_REF,
        )
        beauty_print("夹爪已半开", type="success")
        time.sleep(1.0)

        # --- 打开夹爪 (1000 = 全开) ---
        beauty_print("打开夹爪 (1000)...", type="info")
        robot.set_robot_state(
            gripper_value=1000, gripper_speed=100,
            wait_for_completion=True,
            kp=MIT_KP, kd=MIT_KD, torque=MIT_TORQUE, vel_ref=MIT_VEL_REF,
        )
        beauty_print("夹爪已打开", type="success")
        time.sleep(1.0)

        # --- 半开夹爪 (500) ---
        beauty_print("移动夹爪到半开 (500)...", type="info")
        robot.set_robot_state(
            gripper_value=500, gripper_speed=100,
            wait_for_completion=True,
            kp=MIT_KP, kd=MIT_KD, torque=MIT_TORQUE, vel_ref=MIT_VEL_REF,
        )
        beauty_print("夹爪已半开", type="success")
        time.sleep(1.0)

        # --- 关闭夹爪 (0 = 全闭) ---
        beauty_print("关闭夹爪 (0)...", type="info")
        robot.set_robot_state(
            gripper_value=0, gripper_speed=100,
            wait_for_completion=True,
            kp=MIT_KP, kd=MIT_KD, torque=MIT_TORQUE, vel_ref=MIT_VEL_REF,
        )
        beauty_print("夹爪已关闭", type="success")
        time.sleep(2.0)

        # --- 读取当前夹爪值 ---
        state = robot.get_robot_state("joint_gripper")
        if state is not None:
            beauty_print(f"当前夹爪值: {state['gripper']:.0f} / 1000", type="info")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
