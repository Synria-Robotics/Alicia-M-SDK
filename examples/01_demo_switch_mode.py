#!/usr/bin/env python3
# Copyright (c) 2025 Synria Robotics Co., Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
Demo: Switch control mode from MIT to PV

Features:
- Send mode switch command continuously
- Wait for confirmation response
- Automatic mode switching
"""

import alicia_m_sdk
import time
from alicia_m_sdk.utils.logger import logger
from PyCRC.CRC32 import CRC32


def calculate_checksum(data):
    """Calculate CRC32 checksum (last 8 bits)
    
    :param data: Data bytes to calculate checksum for
    :return: Checksum value (0-255)
    """
    crc_calculator = CRC32()
    crc = crc_calculator.calculate(bytes(data))
    return crc & 0xFF


def build_mode_switch_command(target_mode='pv'):
    """Build mode switch command
    
    PV mode byte: 0x02
    MIT mode byte: 0x01
    
    :param target_mode: 'pv' or 'mit'
    :return: Complete command frame as list of integers
    """
    mode_byte = 0x02 if target_mode == 'pv' else 0x01
    # Build frame without checksum and footer
    frame = [0xAA, 0x11, 0x82, 0x07, 0x01, 0x07, 0x0B, mode_byte, 0x00, 0x00, 0x00]
    
    # Calculate checksum for bytes from index 1 to end (excluding header and footer)
    payload = frame[1:]
    checksum = calculate_checksum(payload)
    
    # Add checksum and footer
    frame.append(checksum)
    frame.append(0xFF)
    
    return frame


def is_mode_switch_confirmation(frame):
    """Check if received frame is mode switch confirmation
    
    Expected response:
    [0xAA] [0x11] [0x82] [0x04] [0x01] [0x07] [0x8B] [0x01] [CRC] [0xFF]
    
    :param frame: Received frame
    :return: True if it's the confirmation frame
    """
    if frame is None or len(frame) < 10:
        return False
    
    # Check key bytes (ignore checksum for now)
    expected_pattern = [0xAA, 0x11, 0x82, 0x04, 0x01, 0x07, 0x8B, 0x01]
    
    # Compare first 8 bytes
    if frame[:8] != expected_pattern:
        return False
    
    # Verify frame structure (should end with 0xFF)
    if frame[-1] != 0xFF:
        return False
    
    # Verify checksum
    payload = frame[1:-2]
    calculated_checksum = calculate_checksum(payload)
    received_checksum = frame[-2]
    
    return calculated_checksum == received_checksum


def switch_control_mode(robot, target_mode='pv', max_attempts=100, send_interval=0.1):
    """Switch control mode (MIT <-> PV)
    
    :param robot: Robot instance
    :param target_mode: 'pv' or 'mit'
    :param max_attempts: Maximum number of send attempts
    :param send_interval: Time interval between sends (seconds)
    :return: True if successful, False if timeout
    """
    mode_name = "PV" if target_mode == 'pv' else "MIT"
    logger.info(f"开始切换控制模式 -> {mode_name}")
    
    # Build mode switch command
    switch_command = build_mode_switch_command(target_mode)
    logger.info(f"模式切换指令: {' '.join(f'{b:02X}' for b in switch_command)}")
    
    # Get serial communication interface
    serial_comm = robot.servo_driver.serial_comm
    
    # Pause background update thread to avoid interference
    was_thread_running = robot.servo_driver.is_update_thread_running()
    if was_thread_running:
        robot.servo_driver._pause_update.set()
        time.sleep(0.2)  # Give thread time to pause
    
    try:
        attempt = 0
        while attempt < max_attempts:
            attempt += 1
            
            # Send mode switch command
            success = serial_comm.send_data(switch_command)
            if not success:
                logger.warning(f"发送指令失败 (尝试 {attempt}/{max_attempts})")
                time.sleep(send_interval)
                continue
            
            # Check for response
            start_check = time.time()
            check_timeout = send_interval * 0.8  # Check for 80% of interval
            
            while time.time() - start_check < check_timeout:
                frame = serial_comm.read_frame()
                if frame and is_mode_switch_confirmation(frame):
                    logger.info(f"✓ {mode_name} 模式切换成功！")
                    return True
                time.sleep(0.01)
            
            # Wait before next send
            time.sleep(max(0, send_interval - (time.time() - start_check)))
        
        logger.error(f"✗ {mode_name} 模式切换超时")
        return False
    
    finally:
        # Resume background thread if it was running
        if was_thread_running:
            robot.servo_driver._pause_update.clear()
            time.sleep(0.1)


def main(args):
    """Main function for interactive mode switching.
    
    :param args: Command line arguments
    """
    # Initialize serial connection
    logger.info("正在连接机器人...")
    robot = alicia_m_sdk.create_robot(
        port=args.port,
        gripper_type=args.gripper_type,
        version=args.version,
        control_aim=args.control_aim,
        control_mode='pv'
    )
    
    try:
        if not robot.connect():
            logger.error("✗ 无法连接到机器人，请检查端口设置")
            return False
            
        logger.info("✓ 机器人已连接")
        print("\n" + "=" * 50)
        print("模式切换交互终端")
        print("  - 输入 'P' : 切换到 PV 模式")
        print("  - 输入 'M' : 切换到 MIT 模式")
        print("  - 输入 'Q' : 退出程序")
        print("=" * 50)
        
        while True:
            choice = input("\n请输入指令 [P/M/Q]: ").strip().upper()
            
            if choice == 'P':
                switch_control_mode(robot, target_mode='pv')
            elif choice == 'M':
                switch_control_mode(robot, target_mode='mit')
            elif choice == 'Q':
                logger.info("正在退出...")
                break
            else:
                print("无效指令,请输入 P, M 或 Q")
        
        return True
        
    except KeyboardInterrupt:
        logger.warning("\n✗ 用户中断操作")
        return False
    finally:
        robot.disconnect()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Control Mode Switch Demo (MIT -> PV)")
    
    # Serial port settings
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--gripper_type', type=str, default="100mm", help="夹爪类型")
    parser.add_argument('--version', type=str, default="v1_1", help="机械臂版本")
    parser.add_argument('--control-aim', type=str, default='operation', choices=['teach', 'operation'],
                        help='Control aim: teach or operation (motor-specific, auto-detected if not specified)')
    
    # Mode switch parameters
    parser.add_argument('--max-attempts', type=int, default=100, 
                        help="最大发送次数 (默认: 100)")
    parser.add_argument('--send-interval', type=float, default=0.1, 
                        help="发送间隔时间/秒 (默认: 0.1)")
    
    args = parser.parse_args()
    
    main(args)
