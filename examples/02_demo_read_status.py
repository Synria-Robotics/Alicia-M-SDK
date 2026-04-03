"""02_demo_read_status.py — 读取机械臂模式与使能状态

演示查询各电机控制模式（通过 0x11 读取）和运行状态。
"""

import alicia_m_sdk
from robocore.utils.beauty_logger import beauty_print, beauty_print_array


def main():
    beauty_print("Demo: 读取机械臂模式与使能状态", type="module")

    # 创建并连接机器人
    robot = alicia_m_sdk.create_robot()
    beauty_print(f"机器人连接成功（{robot.control_mode.value.upper()} 模式）", type="success")

    try:
        # --- 查询各电机控制模式 (0x11, addr=0x0B) ---
        beauty_print("查询各电机控制模式...", type="info")
        modes = robot.get_robot_state("control_mode")
        if modes is not None:
            for i, m in enumerate(modes):
                label = f"M{i}" if i < 6 else "夹爪"
                beauty_print(f"  {label}: {m['name']} (0x{m['value']:02X})", type="info")
        else:
            beauty_print("  控制模式查询超时", type="warning")

        # --- 查询运行状态 ---
        beauty_print("查询运行状态...", type="info")
        status = robot.get_robot_state("status")
        if status is not None:
            beauty_print(f"  关节锁定: {status.is_locked}", type="info")
            beauty_print(f"  双臂同步: {status.is_synced}", type="info")
            beauty_print(f"  电机错误: {status.has_motor_error}", type="info")
            beauty_print(f"  夹爪力矩锁定: {status.gripper_torque_locked}", type="info")
        else:
            beauty_print("  运行状态暂不可用", type="warning")

        beauty_print("查询完成", type="success")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
