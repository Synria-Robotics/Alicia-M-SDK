"""03_demo_read_states.py — 读取关节状态

演示循环读取并打印关节角度、夹爪、速度、力矩。
使用 --extend 参数可启用扩展查询（插补速度、线圈温度，需新固件支持）。
按 Ctrl+C 退出。
"""

import argparse
import math
import time
import alicia_m_sdk
from robocore.utils.beauty_logger import beauty_print, beauty_print_array


def main():
    beauty_print("Demo: 读取关节状态（循环打印）", type="module")

    parser = argparse.ArgumentParser(description="读取关节状态示例")
    parser.add_argument(
        "--extend", action="store_true",
        help="启用扩展查询（插补速度、线圈温度，需新固件支持）"
    )
    args = parser.parse_args()

    # 创建并连接机器人
    robot = alicia_m_sdk.create_robot()
    beauty_print(f"机器人连接成功（{robot.control_mode.value.upper()} 模式）", type="success")

    # 按需启用扩展查询
    if args.extend:
        robot.set_extended_polling(True)
        beauty_print("已启用扩展状态查询（插补速度 + 线圈温度）", type="info")

    beauty_print("按 Ctrl+C 退出循环", type="info")

    try:
        while True:
            # --- 读取关节+夹爪状态 ---
            state = robot.get_robot_state("all")
            if state is None:
                beauty_print("等待状态数据...", type="warning")
                time.sleep(0.5)
                continue

            # 关节角度 (rad)
            angles_rad = state.angles
            angles_deg = [a * 180.0 / math.pi for a in angles_rad]

            beauty_print("--- 关节状态 ---", type="module")
            beauty_print(f"  关节角度 (deg): {beauty_print_array(angles_deg, precision=2)}", type="info")
            beauty_print(f"  关节角度 (rad): {beauty_print_array(angles_rad, precision=4)}", type="info")

            # 夹爪开合度 (0~1000)
            beauty_print(f"  夹爪开合度:     {state.gripper:.0f} / 1000", type="info")

            # 关节速度
            if state.velocities is not None:
                beauty_print(f"  关节速度 (rad/s): {beauty_print_array(state.velocities, precision=3)}", type="info")

            # 关节力矩
            if state.torques is not None:
                beauty_print(f"  关节力矩 (N*m):  {beauty_print_array(state.torques, precision=3)}", type="info")

            # 扩展字段（仅 --extend 时有数据）
            if state.kps is not None:
                beauty_print(f"  位置环 Kp:        {beauty_print_array(state.kps, precision=1)}", type="info")

            if state.kds is not None:
                beauty_print(f"  速度环 Kd:        {beauty_print_array(state.kds, precision=2)}", type="info")

            if state.linear_vels is not None:
                beauty_print(f"  插补速度 (rad/s): {beauty_print_array(state.linear_vels, precision=3)}", type="info")

            if state.temperatures is not None:
                beauty_print(f"  线圈温度 (°C):    {beauty_print_array(state.temperatures, precision=1)}", type="info")

            # 控制打印频率，约 5Hz
            time.sleep(0.2)

    except KeyboardInterrupt:
        beauty_print("\n用户中断，退出循环", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
