"""14_demo_drag_teaching.py — 拖动示教与回放

流程:
1. PV 模式连接 -> Enter 切换 MIT（关节自由活动）
2. 拖动机械臂和夹爪，后台录制路点
3. q+Enter 停止录制 -> 切换回 PV
4. y+Enter 回放轨迹（PV 逐点控制）
"""

import math
import time
import threading
import alicia_m_sdk
from robocore.utils.beauty_logger import beauty_print, beauty_print_array


def main():
    beauty_print("Demo: 拖动示教与回放", type="module")

    # PV 模式连接（确保从 PV 开始，若固件当前为 MIT 则自动切换）
    robot = alicia_m_sdk.create_robot(control_mode="pv")
    beauty_print(f"机器人连接成功（{robot.control_mode.value.upper()} 模式）", type="success")

    # 录制的路点数据: [(timestamp, [6 angles], gripper), ...]
    recorded_waypoints = []
    recording = False
    stop_event = threading.Event()

    def _record_loop():
        """后台录制线程：持续采样关节角度"""
        start_time = time.time()
        while not stop_event.is_set():
            state = robot.get_robot_state("all")
            if state is not None:
                t = time.time() - start_time
                recorded_waypoints.append((t, list(state.angles), state.gripper))
            time.sleep(0.01)  # 约 100Hz 采样率

    try:
        # --- 切换到 MIT，进入自由拖动 ---
        beauty_print("切换到 MIT 模式后关节可自由活动", type="warning")
        beauty_print("请用手扶住机械臂，避免突然掉落", type="warning")
        input("\n按 Enter 切换到 MIT 模式并开始录制...")

        beauty_print("正在切换到 MIT 模式...", type="info")
        robot.switch_mode("mit")
        beauty_print("已切换到 MIT 模式，可以自由拖动机械臂和夹爪", type="success")

        # --- 开始录制 ---
        beauty_print("开始录制路点... 按 q + Enter 停止录制", type="info")
        recording = True
        stop_event.clear()
        record_thread = threading.Thread(target=_record_loop, daemon=True)
        record_thread.start()

        # 等待用户按 q 停止
        while True:
            try:
                user_input = input()
                if user_input.strip().lower() == 'q':
                    break
            except EOFError:
                break

        # 停止录制
        stop_event.set()
        record_thread.join(timeout=2.0)
        recording = False

        beauty_print(f"录制完成! 共 {len(recorded_waypoints)} 个路点", type="success")

        # --- 切换回 PV ---
        beauty_print("正在切换回 PV 模式...", type="info")
        robot.switch_mode("pv")
        beauty_print("已切换回 PV 模式", type="success")

        if len(recorded_waypoints) < 2:
            beauty_print("路点数据不足，无法回放", type="warning")
            return

        # --- 展示轨迹统计 ---
        beauty_print("轨迹信息:", type="module")
        total_time = recorded_waypoints[-1][0] - recorded_waypoints[0][0]
        beauty_print(f"  录制时长: {total_time:.2f} 秒", type="info")
        beauty_print(f"  路点数量: {len(recorded_waypoints)}", type="info")
        beauty_print(f"  采样频率: {len(recorded_waypoints) / max(total_time, 0.001):.1f} Hz", type="info")

        start_angles_deg = [a * 180.0 / math.pi for a in recorded_waypoints[0][1]]
        end_angles_deg = [a * 180.0 / math.pi for a in recorded_waypoints[-1][1]]
        beauty_print(f"  起始角度 (deg): {beauty_print_array(start_angles_deg, precision=1)}", type="info")
        beauty_print(f"  结束角度 (deg): {beauty_print_array(end_angles_deg, precision=1)}", type="info")

        # --- 询问是否回放 ---
        beauty_print("输入 y + Enter 开始回放，其他键跳过", type="info")
        try:
            user_input = input("  > ")
        except EOFError:
            user_input = ""
        if user_input.strip().lower() != 'y':
            beauty_print("跳过回放", type="info")
            return

        # --- 移动到起始位置 ---
        start_angles = recorded_waypoints[0][1]
        start_gripper = recorded_waypoints[0][2]
        beauty_print("移动到轨迹起始位置...", type="info")
        robot.set_robot_state(
            target_joints=start_angles,
            gripper_value=start_gripper,
            joint_format="rad",
            speed=10,
            wait_for_completion=True,
        )
        beauty_print("已到达起始位置", type="success")
        time.sleep(0.5)

        # --- PV 回放 ---
        beauty_print("开始 PV 回放轨迹...", type="info")
        playback_start = time.time()

        for i, (t, angles, gripper) in enumerate(recorded_waypoints):
            # 等待到对应时间点
            target_time = playback_start + t
            now = time.time()
            if target_time > now:
                time.sleep(target_time - now)

            # PV 模式逐点发送（不等待，持续发送）
            robot.set_robot_state(
                target_joints=angles,
                gripper_value=gripper,
                joint_format="rad",
                speed=50,
                wait_for_completion=False,
            )

            # 每 50 个路点打印进度
            if i % 50 == 0:
                progress = (i + 1) / len(recorded_waypoints) * 100
                beauty_print(f"  回放进度: {progress:.0f}% ({i + 1}/{len(recorded_waypoints)})", type="info")

        beauty_print("轨迹回放完成!", type="success")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
        if recording:
            stop_event.set()
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
