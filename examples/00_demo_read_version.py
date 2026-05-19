"""00_demo_read_version.py — 读取固件版本号

演示如何连接机器人并读取版本信息（序列号、硬件版本、固件版本）。
"""

import argparse

import alicia_m_sdk
from alicia_m_sdk.utils.cli import add_port_argument
from alicia_m_sdk.utils.beauty_logger import beauty_print, beauty_print_array


def main():
    beauty_print("Demo: 读取固件版本号", type="module")

    parser = argparse.ArgumentParser(description="Read Alicia-M firmware version.")
    add_port_argument(parser)
    args = parser.parse_args()

    # 创建并连接机器人（自动检测控制模式，仅查询信息无需指定模式）
    robot = alicia_m_sdk.create_robot(port=args.port, sync_control_mode=False)
    beauty_print("机器人连接成功", type="success")

    try:
        # --- 查询版本信息 ---
        version = robot.get_robot_state("version")
        if version is not None:
            beauty_print("版本信息:", type="info")
            beauty_print(f"  序列号:     {version.serial_number}", type="info")
            beauty_print(f"  硬件版本:   {version.hardware_version}", type="info")
            beauty_print(f"  固件版本:   {version.firmware_version}", type="info")
            beauty_print(f"  产品类别:   {version.product_type}", type="info")
            beauty_print(f"  设备类型:   {version.device_type}", type="info")
        else:
            beauty_print("未能获取版本信息（固件可能未响应）", type="warning")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
