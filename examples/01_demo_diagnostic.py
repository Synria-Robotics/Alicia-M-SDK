"""01_demo_diagnostic.py — 自检功能

演示自检功能的调用方式。
需要固件版本 1.0.6 或以上。
"""

import argparse

import alicia_m_sdk
from alicia_m_sdk.diagnostics import (
    CONTROL_MODE_NAMES,
    DIAGNOSTIC_JOINT_COUNT,
    MOTOR_STATE_NAMES,
    DiagnosticResult,
    code_name,
    mode_name,
    supports_diagnostic,
)
from alicia_m_sdk.hardware.constants import ERROR_DESCRIPTIONS
from alicia_m_sdk.utils.beauty_logger import beauty_print
from alicia_m_sdk.utils.protocol import format_bytes
from _common import add_port_argument


ERROR_EXTRA_LABELS = {
    0x00: "收到的帧长度",
    0x01: "收到的帧长度",
    0x02: "下位机计算出的校验字节",
    0x03: "预留信息",
    0x04: "越界关节编号",
    0x05: "当前数据长度或数量",
    0x06: "非法地址或越界值",
    0x07: "请求功能码",
    0x08: "预留信息",
}


def print_diagnostic_response(result: DiagnosticResult) -> bool:
    """打印自检响应快照。"""
    if result.frame is None:
        return False
    if result.error_code is not None:
        print_error_response(result)
        return False

    for snapshot in result.snapshots:
        beauty_print(f"{snapshot.arm_name}自检快照:", type="info")
        beauty_print(f"  通信状态位图: 0x{snapshot.comm_bitmap:02X}", type="info")
        beauty_print(f"  通信状态: {format_comm_bitmap(snapshot.comm_bitmap)}", type="info")
        beauty_print(f"  电机状态: {format_bytes(bytes(snapshot.motor_states))}", type="info")
        beauty_print(f"  状态含义: {format_motor_states(snapshot.motor_states)}", type="info")
        beauty_print(f"  电机模式: {format_bytes(bytes(snapshot.control_modes))}", type="info")
        beauty_print(f"  模式含义: {format_control_modes(snapshot.control_modes)}", type="info")
    return True


def print_error_response(result: DiagnosticResult):
    """按 0xEE 错误反馈格式打印错误帧。"""
    error_type = result.error_code
    extra = result.error_data[0] if result.error_data else None
    error_name = ERROR_DESCRIPTIONS.get(error_type, "未知错误")
    beauty_print(f"自检返回错误: {error_name} (0x{error_type:02X})", type="warning")

    if extra is None:
        beauty_print("  附加信息: 缺失", type="warning")
        return

    if error_type == 0xEE:
        current_mode = (extra & 0xF0) >> 4
        target_mode = extra & 0x0F
        beauty_print(f"  当前模式: 0x{current_mode:X} {mode_name(current_mode)}", type="info")
        beauty_print(f"  目标模式: 0x{target_mode:X} {mode_name(target_mode)}", type="info")
        return

    extra_label = ERROR_EXTRA_LABELS.get(error_type, "附加信息")
    beauty_print(f"  {extra_label}: 0x{extra:02X}", type="info")


def format_comm_bitmap(comm_bitmap: int) -> str:
    """格式化 7 个关节的通信快照位。"""
    states = []
    for joint_index in range(DIAGNOSTIC_JOINT_COUNT):
        state = "收到" if comm_bitmap & (1 << joint_index) else "未收到"
        states.append(f"J{joint_index + 1}={state}")
    return ", ".join(states)


def format_motor_states(states: list[int]) -> str:
    """格式化 J1~J7 电机状态码。"""
    return ", ".join(
        f"J{index + 1}={code_name(value, MOTOR_STATE_NAMES)}"
        for index, value in enumerate(states)
    )


def format_control_modes(modes: list[int]) -> str:
    """格式化 J1~J7 控制模式。"""
    return ", ".join(
        f"J{index + 1}={code_name(value, CONTROL_MODE_NAMES)}"
        for index, value in enumerate(modes)
    )


def main():
    beauty_print("Demo: 自检功能", type="module")

    parser = argparse.ArgumentParser(description="Run Alicia-M diagnostic demo.")
    add_port_argument(parser)
    parser.add_argument(
        "--timeout",
        type=float,
        default=3.0,
        help="等待自检响应的最长时间（秒）",
    )
    args = parser.parse_args()

    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print("机器人连接成功", type="success")

    try:
        version = robot.get_firmware_version(timeout=args.timeout)
        beauty_print(f"固件版本: {version}", type="info")

        if not supports_diagnostic(version):
            beauty_print(
                f"固件版本 {version} 不支持自检功能，需 v1.0.6 或以上",
                type="warning",
            )
            return

        beauty_print("正在发送自检帧...", type="info")
        result = robot.run_diagnostic(timeout=args.timeout)
        if result.frame is None:
            beauty_print("自检超时，未收到响应", type="warning")
            return
        beauty_print(f"RX: {format_bytes(result.frame.encode())}", type="info")

        ok = print_diagnostic_response(result)
        if ok:
            beauty_print("自检完成", type="success")
        else:
            beauty_print("自检返回异常响应", type="warning")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
