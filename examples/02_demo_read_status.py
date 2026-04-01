"""02_demo_read_status.py — 读取机械臂模式与使能状态

演示查询当前控制模式（PV/MIT）和使能状态。
注意: 底层读取模式信息功能尚未更新，当前以占位形式实现。
"""

import alicia_m_sdk
from robocore.utils.beauty_logger import beauty_print, beauty_print_array


def main():
    beauty_print("Demo: 读取机械臂模式与使能状态（底层待更新）", type="module")

    # 创建并连接机器人
    robot = alicia_m_sdk.create_robot(control_mode="pv")
    beauty_print("机器人连接成功", type="success")

    try:
        # --- 查询当前控制模式 ---
        beauty_print("查询当前控制模式...", type="info")
        control_mode = robot.get_robot_state("control_mode")
        if control_mode is not None:
            beauty_print(f"  当前模式: {control_mode}", type="info")
        else:
            beauty_print("  控制模式查询暂未实现（底层待更新）", type="warning")
            beauty_print("  预留接口: robot.get_robot_state('control_mode')", type="info")

        # --- 查询使能状态 ---
        beauty_print("查询使能状态...", type="info")
        # 通过关节状态中的 run_status 推断使能信息
        state = robot.get_robot_state("joint_gripper")
        if state is not None:
            beauty_print(f"  关节状态可用: 是", type="info")
            # run_status 字节包含锁定/同步等状态位
            if hasattr(state, 'run_status'):
                beauty_print(f"  运行状态字节: 0x{state.run_status:02X}", type="info")
            else:
                beauty_print(f"  运行状态: {state}", type="info")
        else:
            beauty_print("  使能状态查询暂未实现（底层待更新）", type="warning")

        beauty_print("查询完成", type="success")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
