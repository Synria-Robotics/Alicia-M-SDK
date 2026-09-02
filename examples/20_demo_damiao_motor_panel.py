#!/usr/bin/env python3
"""@file 20_demo_damiao_motor_panel.py
@brief 达妙电机 USB-CAN 调试窗口 Demo。
@details
本示例不使用 Alicia-M SDK 公共协议层，而是直接打开达妙 USB-CAN/调试器串口。
启动后扫描 7 台电机，读取关键寄存器，必要时将前 6 台临时切换到 POS_VEL、
第 7 台临时切换到 MIT，随后打开固定 1080x720 的 tkinter/ttk 调试窗口。

@note 本示例会直接向真机电机发送 CAN 控制帧。运行前请确认机械臂处于安全空间。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import serial

from _demo_helpers import add_port_argument, beauty_print


# @brief 20_demo_src 目录以数字开头，不能作为普通包导入，因此将目录加入 sys.path。
REPO_ROOT = Path(__file__).resolve().parents[1]
DEMO20_SRC = REPO_ROOT / "20_demo_src"
if str(DEMO20_SRC) not in sys.path:
    sys.path.insert(0, str(DEMO20_SRC))

from damiao_protocol import (  # noqa: E402
    CTRL_MODE_MIT,
    CTRL_MODE_POS_VEL,
    REGISTER_SPECS,
    REGISTER_CAN_ID,
    ScanError,
    make_refresh_status_data,
    make_register_read_data,
    parse_scan_ids,
    scan_motor_ids,
)
from damiao_usbcan import DamiaoUsbCan, encode_send_frame  # noqa: E402
from demo20_gui import DamiaoMotorSession, Demo20Window  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    """@brief 构建命令行参数解析器。
    @return argparse.ArgumentParser 实例。
    """
    parser = argparse.ArgumentParser(
        description="Demo 20: Damiao USB-CAN motor debug panel.",
    )
    add_port_argument(parser)
    parser.add_argument("--baudrate", type=int, default=921_600,
                        help="Serial baudrate for Damiao USB-CAN adapter.")
    parser.add_argument("--scan-ids", default="1-7",
                        help="CAN IDs to scan, e.g. '1-7' or '1,2,3'.")
    parser.add_argument("--expected-motors", type=int, default=7,
                        help="Expected number of motors before opening the GUI.")
    parser.add_argument("--control-period-ms", type=int, default=40,
                        help="Minimum period for slider target sends.")
    parser.add_argument("--inter-frame-us", type=int, default=250,
                        help="Delay inserted after every two CAN frames.")
    parser.add_argument("--read-timeout-ms", type=int, default=20,
                        help="Serial read timeout in milliseconds.")
    parser.add_argument("--scan-attempts", type=int, default=10,
                        help="How many scan rounds to run before failing.")
    parser.add_argument("--scan-interval-ms", type=int, default=150,
                        help="Delay after each scan round, in milliseconds.")
    parser.add_argument("--debug-raw", action="store_true",
                        help="Print raw u2can TX/RX bytes when scan fails.")
    parser.add_argument("--yes", "-y", action="store_true",
                        help="Skip startup safety confirmation.")
    return parser


def format_hex(data: bytes) -> str:
    """@brief 将字节序列格式化为十六进制文本。
    @param data 原始字节。
    @return 空格分隔的大写十六进制文本。
    """
    return " ".join(f"{byte:02X}" for byte in data)


def raw_u2can_probe(port: str, baudrate: int, motor_ids: list[int]) -> None:
    """@brief 直接按达妙 u2can 串口壳打印只读探测的 TX/RX 原始字节。
    @param port 串口名。
    @param baudrate 串口波特率。
    @param motor_ids 待探测电机 ID。
    """
    if not port:
        beauty_print("--debug-raw 需要显式传入 --port，自动端口不做原始探测", type="warning")
        return
    beauty_print(f"RAW probe: {port} @ {baudrate}", type="module")
    try:
        ser = serial.Serial(port, baudrate, timeout=0.5)
    except Exception as exc:
        beauty_print(f"RAW probe 打开串口失败: {exc}", type="error")
        return

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        for motor_id in motor_ids:
            frames = [
                ("refresh", encode_send_frame(REGISTER_CAN_ID, make_refresh_status_data(motor_id))),
                ("read_cmode", encode_send_frame(REGISTER_CAN_ID, make_register_read_data(motor_id, 10))),
            ]
            for label, frame in frames:
                beauty_print(f"TX {label} M{motor_id}: {format_hex(frame)}", type="debug")
                ser.write(frame)
                time.sleep(0.12)
                rx = ser.read_all()
                beauty_print(f"RX {label} M{motor_id}: {len(rx)} bytes {format_hex(rx)}", type="debug")
    finally:
        ser.close()


def wait_for_runtime_registers(session: DamiaoMotorSession, timeout_s: float = 1.0) -> None:
    """@brief 等待关键运行寄存器读回。
    @param session 电机会话对象。
    @param timeout_s 最长等待时间。
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        _feedback, registers, _runtime = session.snapshot()
        ready = all(
            (motor_id, rid) in registers
            for motor_id in session.motor_ids
            for rid in (10, 21, 22, 23)
        )
        if ready:
            return
        time.sleep(0.02)


def wait_for_initial_feedback(session: DamiaoMotorSession, timeout_s: float = 0.5) -> None:
    """@brief 请求并等待窗口启动前的首轮电机反馈。
    @param session 电机会话对象。
    @param timeout_s 最长等待时间。
    """
    for motor_id in session.motor_ids:
        session.refresh_motor_status(motor_id)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        feedback, _registers, _runtime = session.snapshot()
        if all(motor_id in feedback for motor_id in session.motor_ids):
            return
        time.sleep(0.02)


def ensure_mixed_control_modes(session: DamiaoMotorSession, yes: bool) -> None:
    """@brief 检查并临时切换 Demo 20 所需混合控制模式。
    @param session 电机会话对象。
    @param yes true 表示跳过人工确认。
    """
    _feedback, _registers, runtime = session.snapshot()
    desired_modes = {
        motor_id: (CTRL_MODE_MIT if index == 6 else CTRL_MODE_POS_VEL)
        for index, motor_id in enumerate(session.motor_ids)
    }
    mismatched = [
        (motor_id, desired_mode)
        for motor_id, desired_mode in desired_modes.items()
        if runtime.get(motor_id) is not None
        and runtime[motor_id].control_mode != desired_mode
    ]
    if not mismatched:
        beauty_print("控制模式已满足要求：M1~M6 POS_VEL，M7 MIT", type="success")
        return

    pretty = [f"M{motor_id}->CMODE {desired_mode}" for motor_id, desired_mode in mismatched]
    beauty_print(f"以下电机将临时切换控制模式: {pretty}", type="warning")
    beauty_print("该写入默认不保存 Flash；只用于本次调试窗口的位置/速度控制。", type="warning")
    if not yes:
        input("确认机械臂安全后按 Enter 继续，Ctrl+C 取消...")

    for motor_id, desired_mode in mismatched:
        session.write_register(motor_id, 10, desired_mode, save_to_flash=False)
    time.sleep(0.2)
    session.request_runtime_registers()
    wait_for_runtime_registers(session, timeout_s=0.8)


def print_register_summary(session: DamiaoMotorSession) -> None:
    """@brief 打印启动时读取到的关键寄存器。
    @param session 电机会话对象。
    """
    _feedback, registers, _runtime = session.snapshot()
    for motor_id in session.motor_ids:
        parts = []
        for rid in (10, 21, 22, 23):
            spec = REGISTER_SPECS[rid]
            value = registers.get((motor_id, rid))
            parts.append(f"{spec.name}={value.value if value else '--'}")
        beauty_print(f"M{motor_id}: " + ", ".join(parts), type="info")


def run(args: argparse.Namespace) -> None:
    """@brief Demo 20 主流程。
    @param args 命令行参数。
    @throws RuntimeError 当扫描结果不满足期望数量时抛出。
    """
    scan_ids = parse_scan_ids(args.scan_ids)
    if args.expected_motors <= 0:
        raise ValueError("--expected-motors must be positive")

    beauty_print("Demo 20: 达妙 USB-CAN 电机调试窗口", type="module")
    beauty_print(f"扫描 CAN ID: {scan_ids}", type="info")
    beauty_print(f"串口: {args.port or 'auto'} @ {args.baudrate}", type="info")

    if not args.yes:
        input("该 demo 会直接控制真机电机。确认供电、急停和工作空间安全后按 Enter 继续...")

    transport = DamiaoUsbCan(
        port=args.port,
        baudrate=args.baudrate,
        read_timeout_ms=args.read_timeout_ms,
        inter_frame_us=args.inter_frame_us,
    )
    session: DamiaoMotorSession | None = None

    try:
        transport.connect()
        beauty_print(f"USB-CAN 串口已连接: {transport.port}", type="success")

        session = DamiaoMotorSession(
            transport=transport,
            motor_ids=scan_ids,
            control_period_ms=args.control_period_ms,
        )

        found_ids = scan_motor_ids(
            scan_ids,
            send_read=session.read_register,
            collect_found=lambda: list(session.snapshot()[0].keys())
            + [key[0] for key in session.snapshot()[1].keys()],
            expected_count=args.expected_motors,
            attempts=args.scan_attempts,
            interval_s=max(args.scan_interval_ms, 1) / 1000.0,
            send_refresh=session.refresh_motor_status,
        )
        found_ids = found_ids[:args.expected_motors]
        session.motor_ids = found_ids
        beauty_print(f"扫描成功，发现 {len(found_ids)} 台电机: {found_ids}", type="success")

        session.request_runtime_registers()
        wait_for_runtime_registers(session, timeout_s=1.2)
        print_register_summary(session)
        ensure_mixed_control_modes(session, args.yes)
        wait_for_initial_feedback(session, timeout_s=0.5)

        # @brief tkinter 只在扫描成功后导入并创建窗口，避免无设备时弹空窗口。
        import tkinter as tk

        def cleanup() -> None:
            """@brief 窗口关闭时失能电机并断开串口。"""
            if session is not None:
                beauty_print("正在失能电机并关闭串口...", type="warning")
                session.disable_all()
            transport.close()
            beauty_print("USB-CAN 串口已断开", type="info")

        root = tk.Tk()
        Demo20Window(root, session, on_close=cleanup)
        root.mainloop()

    except ScanError as exc:
        transport.close()
        beauty_print(str(exc), type="error")
        beauty_print(
            "官方 u2can 协议已用于扫描；若仍为 0 台，通常是端口不是 u2can、CAN 线/供电/终端电阻异常、"
            "电机 CAN ID 不在扫描范围，或设备并非串口版 u2can。",
            type="warning",
        )
        if args.debug_raw:
            raw_u2can_probe(args.port, args.baudrate, scan_ids[:args.expected_motors])
        raise SystemExit(1) from exc
    except KeyboardInterrupt:
        beauty_print("\n用户取消", type="warning")
        if session is not None:
            session.disable_all()
        transport.close()
    except Exception:
        if session is not None:
            session.disable_all()
        transport.close()
        raise


def main() -> None:
    """@brief 命令行入口。"""
    run(build_parser().parse_args())


if __name__ == "__main__":
    main()
