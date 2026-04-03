"""07_demo_move_joint.py — 关节控制

演示关节空间运动控制：回零 -> 位置A -> 位置B -> 回零。
通过 --control-mode 参数选择 PV 或 MIT 模式。
"""

import argparse
import time
import alicia_m_sdk
from robocore.utils.beauty_logger import beauty_print, beauty_print_array


# 预设安全关节位置 (度)
POSITION_A = [90, -90.0, -90.0, 90.0, 0.0, 0.0]
# POSITION_B = [0, -30, 0, 0, 30, 0]


def main():
    beauty_print("Demo: 关节控制 (PV / MIT)", type="module")

    # 解析命令行参数
    parser = argparse.ArgumentParser(description="关节控制示例")
    parser.add_argument(
        "--control-mode", type=str, default="pv",
        choices=["pv", "mit"],
        help="控制模式: pv 或 mit (默认: pv)"
    )
    parser.add_argument(
        "--speed", type=float, default=15,
        help="运动速度 (默认: 15, 范围: 0-400)"
    )
    args = parser.parse_args()

    # 创建并连接机器人
    robot = alicia_m_sdk.create_robot(control_mode=args.control_mode)
    beauty_print(f"机器人连接成功（{args.control_mode.upper()} 模式）", type="success")

    """ >>> 调试: 拦截第一帧 PV/MIT 写入指令，打印原始十六进制"""
    _orig_send = robot._device.send_frame
    _debug_fired = [False]

    def _debug_send(frame):
        if not _debug_fired[0]:
            raw = frame.encode()
            beauty_print(
                f"[DEBUG] 原始帧 ({len(raw)}B): {raw.hex(' ')}", type="info"
            )
            beauty_print(
                f"[DEBUG] cmd=0x{frame.cmd_id:02X} func=0x{frame.func_code:02X} "
                f"data({len(frame.data)}B)={frame.data.hex(' ')}", type="info"
            )
            _debug_fired[0] = True
        _orig_send(frame)

    robot._device.send_frame = _debug_send
    """ <<< 调试结束 """

    try:
        # --- 回零位 ---
        beauty_print("回零位...", type="info")
        robot.go_home(speed=args.speed)
        beauty_print("已到达零位", type="success")
        time.sleep(1.0)

        # --- 移动到位置 A ---
        beauty_print(f"移动到位置 A: {POSITION_A} (deg)...", type="info")
        _debug_fired[0] = False  # 重置，捕获位置A的帧
        robot.set_robot_state(
            target_joints=POSITION_A,
            joint_format="deg",
            speed=args.speed,
            wait_for_completion=True,
        )
        beauty_print("已到达位置 A", type="success")
        time.sleep(1.0)

        # --- 移动到位置 B ---
        # beauty_print(f"移动到位置 B: {POSITION_B} (deg)...", type="info")
        # robot.set_robot_state(
        #     target_joints=POSITION_B,
        #     joint_format="deg",
        #     speed=args.speed,
        #     wait_for_completion=True,
        # )
        # beauty_print("已到达位置 B", type="success")
        # time.sleep(1.0)

        # --- 回零位 ---
        beauty_print("回零位...", type="info")
        robot.go_home(speed=args.speed)
        beauty_print("已到达零位", type="success")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
