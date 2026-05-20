"""15_demo_gripper_params.py - 夹爪夹持参数读写示例（0x17）。"""

import argparse
import math

import alicia_m_sdk
from _demo_helpers import (
    GRIPPER_PARAM_SPECS,
    add_port_argument,
    beauty_print,
    format_bytes,
    gripper_param_mask,
)


def main():
    beauty_print("Demo: 0x17 夹爪夹持参数读写", type="module")

    parser = argparse.ArgumentParser(description="Read/write Alicia-M gripper parameters with command 0x17.")
    add_port_argument(parser)
    parser.add_argument("--aim", choices=["follower", "leader"], default="follower",
                        help="Target arm: follower or leader; default follower.")
    parser.add_argument("--mask", type=lambda text: int(text, 0), default=0,
                        help="Read mask; 0 reads all parameters.")
    parser.add_argument("--timeout", type=float, default=1.0,
                        help="Timeout for command 0x17 response, in seconds.")
    parser.add_argument("--yes", "-y", action="store_true",
                        help="Skip interactive confirmation before writing.")
    parser.add_argument("--skip-readback", action="store_true",
                        help="Do not read all parameters after writing.")
    parser.add_argument("--gripper-type", choices=["auto", "small", "large", "50mm", "100mm"],
                        default="auto",
                        help="Validate write values with a gripper-specific range; default auto reads user settings.")
    for spec in GRIPPER_PARAM_SPECS:
        flags = ["--" + spec.name.replace("_", "-")]
        flags.extend("--" + alias.replace("_", "-") for alias in spec.aliases)
        help_parts = [f"{spec.label}"]
        if spec.unit:
            help_parts.append(f"单位 {spec.unit}")
        if spec.range_text:
            help_parts.append(f"建议范围 {spec.range_text}")
        help_parts.append(f"mask 0x{spec.mask:02X}")
        parser.add_argument(
            *flags,
            dest=spec.name,
            type=float,
            default=None,
            help="；".join(help_parts),
        )
    args = parser.parse_args()

    if not math.isfinite(args.timeout):
        parser.error("--timeout 必须是有限数字")
    if args.mask < 0 or args.mask > 0xFF:
        parser.error("--mask 必须在 0x00~0xFF 范围内")

    write_values = {
        spec.name: getattr(args, spec.name)
        for spec in GRIPPER_PARAM_SPECS
        if getattr(args, spec.name) is not None
    }
    if any(not math.isfinite(value) for value in write_values.values()):
        parser.error("写入参数必须是有限数字，不能是 NaN 或 inf")
    if write_values and args.mask != 0:
        parser.error("--mask 仅用于读取；写入时请使用具体参数选项，掩码会自动生成")

    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print("机器人连接成功", type="success")

    try:
        if write_values:
            try:
                mask = gripper_param_mask(write_values)
                beauty_print(f"即将写入 0x17 参数，掩码 0x{mask:02X}", type="warning")
                beauty_print("写入不会主动闭合夹爪，但会影响后续夹爪动作。SDK 会按夹爪类型校验参数范围。", type="warning")
                beauty_print(f"夹爪类型校验: {args.gripper_type}", type="info")
                for spec in GRIPPER_PARAM_SPECS:
                    if spec.name in write_values:
                        unit = f" {spec.unit}" if spec.unit else ""
                        beauty_print(f"  {spec.label}: {write_values[spec.name]:.4g}{unit}", type="info")
                        if spec.range_text:
                            beauty_print(f"    建议范围: {spec.range_text}", type="info")
                if not args.yes:
                    input("确认机械臂安全后按 Enter 发送写入帧，Ctrl+C 取消...")

                result = robot.set_gripper_params(
                    write_values,
                    aim=args.aim,
                    timeout=args.timeout,
                    readback=not args.skip_readback,
                    gripper_type=args.gripper_type,
                )
            except ValueError as exc:
                beauty_print(f"设置失败，值不属于指定范围：{exc}", type="warning")
                return
            if result is None:
                beauty_print("未收到 0x17 响应", type="warning")
                return

            write_result = result["write"] if isinstance(result, dict) else result
            beauty_print(f"RX: {format_bytes(write_result.frame.encode())}", type="info")
            ok = bool(write_result.write_ok)
            beauty_print(f"写入结果: {'成功' if ok else '失败'}", type="success" if ok else "warning")

            if isinstance(result, dict) and "readback" in result:
                beauty_print("写入后读回全部夹爪参数:", type="module")
                readback = result["readback"]
                beauty_print(f"RX: {format_bytes(readback.frame.encode())}", type="info")
                for spec in GRIPPER_PARAM_SPECS:
                    if spec.name in readback.values:
                        unit = f" {spec.unit}" if spec.unit else ""
                        beauty_print(f"{spec.label}: {readback.values[spec.name]:.4g}{unit}", type="info")
        else:
            read_text = "全部参数" if args.mask == 0 else f"掩码 0x{args.mask:02X}"
            beauty_print(f"读取 0x17 夹爪参数: {read_text}", type="module")
            result = robot.get_gripper_params(mask=args.mask, aim=args.aim, timeout=args.timeout)
            if result is None:
                beauty_print("未收到 0x17 响应", type="warning")
                return
            beauty_print(f"RX: {format_bytes(result.frame.encode())}", type="info")
            beauty_print(f"响应部位字节: 0x{result.target_byte:02X}, 掩码: 0x{result.mask:02X}", type="info")
            for spec in GRIPPER_PARAM_SPECS:
                if spec.name in result.values:
                    unit = f" {spec.unit}" if spec.unit else ""
                    beauty_print(f"{spec.label}: {result.values[spec.name]:.4g}{unit}", type="info")

    except KeyboardInterrupt:
        beauty_print("\n用户取消", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
