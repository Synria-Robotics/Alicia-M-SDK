"""16_demo_gripper_params.py — 0x17 夹爪夹持参数读写

默认读取操作臂全部夹爪夹持参数。传入任意参数选项时，按 0x17 协议写入
对应掩码位，并可在写入后自动读回全部参数。

用法:
    # 读取操作臂全部夹爪参数
    python 16_demo_gripper_params.py

    # 设置目标夹持力为 80N，最大保持力矩为 70N·m
    python 16_demo_gripper_params.py --target-force 80 --hold-torque 70

    # 读取指定掩码，例如 0x09 = 目标夹持力 + 最大保持力矩
    python 16_demo_gripper_params.py --mask 0x09
"""

import argparse
import math
import struct
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import alicia_m_sdk
from alicia_m_sdk.demo_utils.demo_common import add_port_argument
from alicia_m_sdk.protocol.constants import (
    AIM_FOLLOWER,
    AIM_LEADER,
    CMD_GRIPPER_PARAM,
    FUNC_WRITE_BIT,
)
from alicia_m_sdk.protocol.frame import Frame
from robocore.utils.beauty_logger import beauty_print


@dataclass(frozen=True)
class GripperParamSpec:
    cli_name: str
    mask: int
    label: str
    unit: str


PARAMS: List[GripperParamSpec] = [
    GripperParamSpec("target_force", 0x01, "目标夹持力", "N"),
    GripperParamSpec("open_feedforward", 0x02, "张开前馈力矩", "N·m"),
    GripperParamSpec("close_feedforward", 0x04, "闭合前馈力矩", "N·m"),
    GripperParamSpec("hold_torque", 0x08, "最大保持力矩", "N·m"),
    GripperParamSpec("force_kp", 0x10, "力控比例", ""),
    GripperParamSpec("force_ki", 0x20, "力控积分", ""),
    GripperParamSpec("integral_limit", 0x40, "积分限幅", ""),
    GripperParamSpec("close_torque_scale", 0x80, "接近闭合时的力矩缩放", ""),
]


def format_bytes(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def mask_from_values(values: Dict[int, float]) -> int:
    mask = 0
    for bit in values:
        mask |= bit
    return mask


def make_read_frame(aim: int, mask: int = 0) -> Frame:
    data = b"" if mask == 0 else bytes([mask & 0xFF])
    return Frame(cmd_id=CMD_GRIPPER_PARAM, func_code=aim, data=data)


def make_write_frame(aim: int, values: Dict[int, float]) -> Frame:
    mask = mask_from_values(values)
    data = bytearray([mask])
    for spec in PARAMS:
        if spec.mask & mask:
            data.extend(struct.pack("<f", values[spec.mask]))
    return Frame(
        cmd_id=CMD_GRIPPER_PARAM,
        func_code=FUNC_WRITE_BIT | aim,
        data=bytes(data),
    )


def send_frame(robot, frame: Frame, timeout: float) -> Optional[Frame]:
    beauty_print(f"TX: {format_bytes(frame.encode())}", type="info")
    response = robot._device.send_and_wait(frame, CMD_GRIPPER_PARAM, timeout=timeout)
    if response is None:
        beauty_print("未收到 0x17 响应", type="warning")
        return None
    beauty_print(f"RX: {format_bytes(response.encode())}", type="info")
    return response


def parse_response(frame: Frame) -> None:
    if len(frame.data) < 2:
        beauty_print(f"响应数据过短: {format_bytes(frame.data)}", type="warning")
        return

    target_byte = frame.data[0]
    mask = frame.data[1]
    payload = frame.data[2:]
    beauty_print(f"响应部位字节: 0x{target_byte:02X}, 掩码: 0x{mask:02X}", type="info")

    if len(payload) == 1:
        ok = payload[0] == 0x01
        msg_type = "success" if ok else "warning"
        beauty_print(f"写入结果: {'成功' if ok else '失败'}", type=msg_type)
        return

    offset = 0
    for spec in PARAMS:
        if not (mask & spec.mask):
            continue
        if offset + 4 > len(payload):
            beauty_print(f"{spec.label}: 响应数据不足", type="warning")
            break
        value = struct.unpack_from("<f", payload, offset)[0]
        unit = f" {spec.unit}" if spec.unit else ""
        beauty_print(f"{spec.label}: {value:.4g}{unit}", type="info")
        offset += 4


def collect_write_values(args) -> Dict[int, float]:
    values: Dict[int, float] = {}
    for spec in PARAMS:
        value = getattr(args, spec.cli_name)
        if value is not None:
            values[spec.mask] = value
    return values


def add_write_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--target-force", type=finite_float, default=None,
                        help="目标夹持力 (N)，掩码 0x01")
    parser.add_argument("--open-feedforward", type=finite_float, default=None,
                        help="张开前馈力矩 (N·m)，掩码 0x02")
    parser.add_argument("--close-feedforward", type=finite_float, default=None,
                        help="闭合前馈力矩 (N·m)，掩码 0x04")
    parser.add_argument("--hold-torque", type=finite_float, default=None,
                        help="最大保持力矩 (N·m)，掩码 0x08")
    parser.add_argument("--force-kp", type=finite_float, default=None,
                        help="力控比例，掩码 0x10")
    parser.add_argument("--force-ki", type=finite_float, default=None,
                        help="力控积分，掩码 0x20")
    parser.add_argument("--integral-limit", type=finite_float, default=None,
                        help="积分限幅，掩码 0x40")
    parser.add_argument("--close-torque-scale", type=finite_float, default=None,
                        help="接近闭合时的力矩缩放，掩码 0x80")


def finite_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError("参数值必须是有限数字，不能是 NaN 或 inf")
    return value


def parse_mask(text: str) -> int:
    value = int(text, 0)
    if value < 0 or value > 0xFF:
        raise argparse.ArgumentTypeError("mask 必须在 0x00~0xFF 范围内")
    return value


def main():
    beauty_print("Demo: 0x17 夹爪夹持参数读写", type="module")

    parser = argparse.ArgumentParser(description="Read/write Alicia-M gripper parameters with command 0x17.")
    add_port_argument(parser)
    parser.add_argument("--aim", choices=["follower", "leader"], default="follower",
                        help="目标机械臂，默认 follower/操作臂")
    parser.add_argument("--mask", type=parse_mask, default=0,
                        help="读取掩码；默认 0 表示读取全部参数")
    parser.add_argument("--timeout", type=float, default=1.0,
                        help="等待 0x17 响应超时，单位秒")
    parser.add_argument("--yes", "-y", action="store_true",
                        help="写入前不等待二次确认")
    parser.add_argument("--skip-readback", action="store_true",
                        help="写入后不自动读回全部参数")
    add_write_arguments(parser)
    args = parser.parse_args()

    write_values = collect_write_values(args)
    if write_values and args.mask != 0:
        parser.error("--mask 仅用于读取；写入时请使用具体参数选项，掩码会自动生成")

    aim = AIM_FOLLOWER if args.aim == "follower" else AIM_LEADER

    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print("机器人连接成功", type="success")

    try:
        if write_values:
            mask = mask_from_values(write_values)
            beauty_print(f"即将写入 0x17 参数，掩码 0x{mask:02X}", type="warning")
            beauty_print("写入不会主动闭合夹爪，但会影响后续夹爪动作。", type="warning")
            for spec in PARAMS:
                if spec.mask in write_values:
                    unit = f" {spec.unit}" if spec.unit else ""
                    beauty_print(f"  {spec.label}: {write_values[spec.mask]:.4g}{unit}", type="info")
            if not args.yes:
                input("确认机械臂安全后按 Enter 发送写入帧，Ctrl+C 取消...")

            response = send_frame(robot, make_write_frame(aim, write_values), args.timeout)
            if response is not None:
                parse_response(response)

            if not args.skip_readback:
                time.sleep(0.2)
                beauty_print("写入后读回全部夹爪参数...", type="module")
                response = send_frame(robot, make_read_frame(aim), args.timeout)
                if response is not None:
                    parse_response(response)
        else:
            read_text = "全部参数" if args.mask == 0 else f"掩码 0x{args.mask:02X}"
            beauty_print(f"读取 0x17 夹爪参数: {read_text}", type="module")
            response = send_frame(robot, make_read_frame(aim, args.mask), args.timeout)
            if response is not None:
                parse_response(response)

    except KeyboardInterrupt:
        beauty_print("\n用户取消", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
