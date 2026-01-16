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


def build_mode_switch_command():
    """Build mode switch command from MIT to PV mode
    
    Command structure:
    [0xAA] [0x11] [0x82] [0x07] [0x01] [0x07] [0x0B] [0x02] [0x00] [0x00] [0x00] [CRC] [0xFF]
    
    :return: Complete command frame as list of integers
    """
    # Build frame without checksum and footer
    frame = [0xAA, 0x11, 0x82, 0x07, 0x01, 0x07, 0x0B, 0x02, 0x00, 0x00, 0x00]
    
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


def switch_mode_mit_to_pv(robot, max_attempts=100, send_interval=0.1):
    """Switch control mode from MIT to PV
    
    :param robot: Robot instance
    :param max_attempts: Maximum number of send attempts
    :param send_interval: Time interval between sends (seconds)
    :return: True if successful, False if timeout
    """
    logger.info("开始切换控制模式：MIT -> PV")
    
    # Build mode switch command
    switch_command = build_mode_switch_command()
    logger.info(f"模式切换指令: {' '.join(f'{b:02X}' for b in switch_command)}")
    
    # Get serial communication interface
    serial_comm = robot.servo_driver.serial_comm
    data_parser = robot.servo_driver.data_parser
    
    # Pause background update thread to avoid interference
    logger.info("暂停后台更新线程...")
    was_thread_running = robot.servo_driver.is_update_thread_running()
    if was_thread_running:
        robot.servo_driver._pause_update.set()
        time.sleep(0.2)  # Give thread time to pause
        logger.info(f"后台线程已暂停 (状态: {robot.servo_driver._pause_update.is_set()})")
    
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
            
            if attempt % 10 == 1:  # Print every 10 attempts
                logger.info(f"正在发送模式切换指令... (尝试 {attempt}/{max_attempts})")
            
            # Check for response
            start_check = time.time()
            check_timeout = send_interval * 0.8  # Check for 80% of interval
            
            while time.time() - start_check < check_timeout:
                frame = serial_comm.read_frame()
                
                if frame:
                    # Debug: print received frame
                    logger.info(f"[调试] 接收到数据包: {' '.join(f'{b:02X}' for b in frame)}")
                    
                    # Check if it's the confirmation frame
                    if is_mode_switch_confirmation(frame):
                        logger.info("✓ 模式切换成功！")
                        logger.info(f"确认数据包: {' '.join(f'{b:02X}' for b in frame)}")
                        return True
                    else:
                        # Print why it didn't match
                        if len(frame) >= 8:
                            expected = [0xAA, 0x11, 0x82, 0x04, 0x01, 0x07, 0x8B, 0x01]
                            logger.info(f"[调试] 期望前8字节: {' '.join(f'{b:02X}' for b in expected)}")
                            logger.info(f"[调试] 实际前8字节: {' '.join(f'{b:02X}' for b in frame[:8])}")
                
                time.sleep(0.01)  # Small delay between checks
            
            # Wait before next send
            time.sleep(send_interval - (time.time() - start_check))
        
        logger.error(f"✗ 模式切换超时 (已尝试 {max_attempts} 次)")
        return False
    
    finally:
        # Resume background thread if it was running
        if was_thread_running:
            logger.info("恢复后台更新线程...")
            robot.servo_driver._pause_update.clear()
            time.sleep(0.1)  # Give thread time to resume
            logger.info(f"后台线程已恢复 (暂停标志: {robot.servo_driver._pause_update.is_set()})")
            logger.info(f"后台线程运行状态: {robot.servo_driver.is_update_thread_running()}")


def main(args):
    """Main function for mode switching demo.
    
    :param args: Command line arguments
    """
    # Initialize serial connection (no specific mode needed)
    logger.info("初始化串口连接...")
    robot = alicia_m_sdk.create_robot(
        port=args.port,
        gripper_type=args.gripper_type,
        robot_version=args.robot_version,
        control_aim=args.control_aim,
        control_mode='pv'  # Use any mode just to establish serial connection
    )
    
    try:
        logger.info("开始模式切换流程：MIT -> PV")
        logger.info("=" * 50)
        
        print()
        
        # Perform mode switch by sending command to STM32
        success = switch_mode_mit_to_pv(
            robot,
            max_attempts=args.max_attempts,
            send_interval=args.send_interval
        )
        
        if success:
            logger.info("=" * 50)
            logger.info("模式切换完成！机械臂现在处于 PV 控制模式")
            logger.info("=" * 50)
        else:
            logger.error("模式切换失败！")
        
        return success
        
    except KeyboardInterrupt:
        logger.warning("\n✗ 用户中断操作")
        return False
    
    except Exception as e:
        logger.error(f"发生异常: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        robot.disconnect()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Control Mode Switch Demo (MIT -> PV)")
    
    # Serial port settings
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--gripper_type', type=str, default="100mm", help="夹爪类型")
    parser.add_argument('--robot_version', type=str, default="v1_1", help="机械臂版本")
    parser.add_argument('--control-aim', type=str, default='operation', choices=['teach', 'operation'],
                        help='Control aim: teach or operation (motor-specific)')
    
    # Mode switch parameters
    parser.add_argument('--max-attempts', type=int, default=100, 
                        help="最大发送次数 (默认: 100)")
    parser.add_argument('--send-interval', type=float, default=0.1, 
                        help="发送间隔时间/秒 (默认: 0.1)")
    
    args = parser.parse_args()
    
    main(args)
