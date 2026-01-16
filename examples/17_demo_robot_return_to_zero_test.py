



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
Demo: Robot Go Home (Zero Position)

Features:
- Send go home command continuously
- Wait for confirmation response
- Automatic homing process
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


def build_go_home_command():
    """Build go home (zero position) command
    
    Command structure:
    [0xAA] [0x03] [0x02] [0x02] [0x01] [0x07] [CRC] [0xFF]
    
    :return: Complete command frame as list of integers
    """
    # Build frame without checksum and footer
    frame = [0xAA, 0x03, 0x02, 0x02, 0x01, 0x07]
    
    # Calculate checksum for bytes from index 1 to end (excluding header and footer)
    payload = frame[1:]
    checksum = calculate_checksum(payload)
    
    # Add checksum and footer
    frame.append(checksum)
    frame.append(0xFF)
    
    return frame


def is_go_home_confirmation(frame):
    """Check if received frame is go home confirmation
    
    Expected response:
    [0xAA] [0x03] [0x83] [0x01] [0x01] [CRC] [0xFF]
    
    :param frame: Received frame
    :return: True if it's the confirmation frame
    """
    if frame is None or len(frame) < 7:
        return False
    
    # Check key bytes (ignore checksum for now)
    expected_pattern = [0xAA, 0x03, 0x82, 0x01, 0x01]
    
    # Compare first 5 bytes
    if frame[:5] != expected_pattern:
        return False
    
    # Verify frame structure (should end with 0xFF)
    if frame[-1] != 0xFF:
        return False
    
    # Verify checksum
    payload = frame[1:-2]
    calculated_checksum = calculate_checksum(payload)
    received_checksum = frame[-2]
    
    return calculated_checksum == received_checksum

def robot_go_home(robot, max_attempts=100, send_interval=0.1):
    """Send robot to home (zero) position
    
    :param robot: Robot instance
    :param max_attempts: Maximum number of send attempts
    :param send_interval: Time interval between sends (seconds)
    :return: True if successful, False if timeout
    """
    logger.info("开始执行机械臂归零操作")
    
    # Build go home command
    home_command = build_go_home_command()
    logger.info(f"归零指令: {' '.join(f'{b:02X}' for b in home_command)}")
    
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
            
            # Send go home command
            success = serial_comm.send_data(home_command)
            if not success:
                logger.warning(f"发送指令失败 (尝试 {attempt}/{max_attempts})")
                time.sleep(send_interval)
                continue
            
            if attempt % 10 == 1:  # Print every 10 attempts
                logger.info(f"正在发送归零指令... (尝试 {attempt}/{max_attempts})")
            
            # Check for response
            start_check = time.time()
            check_timeout = send_interval * 0.8  # Check for 80% of interval
            
            while time.time() - start_check < check_timeout:
                frame = serial_comm.read_frame()
                
                if frame:
                    # Debug: print received frame
                    logger.info(f"[调试] 接收到数据包: {' '.join(f'{b:02X}' for b in frame)}")
                    
                    # Check if it's the confirmation frame
                    if is_go_home_confirmation(frame):
                        logger.info("✓ 机械臂归零成功！")
                        logger.info(f"确认数据包: {' '.join(f'{b:02X}' for b in frame)}")
                        return True
                    else:
                        # Print why it didn't match
                        if len(frame) >= 5:
                            expected = [0xAA, 0x03, 0x83, 0x01, 0x01]
                            logger.info(f"[调试] 期望前5字节: {' '.join(f'{b:02X}' for b in expected)}")
                            logger.info(f"[调试] 实际前5字节: {' '.join(f'{b:02X}' for b in frame[:5])}")
                
                time.sleep(0.01)  # Small delay between checks
            
            # Wait before next send
            time.sleep(send_interval - (time.time() - start_check))
        
        logger.error(f"✗ 归零操作超时 (已尝试 {max_attempts} 次)")
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
    """Main function for robot homing demo.
    
    :param args: Command line arguments
    """
    # Initialize serial connection
    logger.info("初始化串口连接...")
    robot = alicia_m_sdk.create_robot(
        port=args.port,
        gripper_type=args.gripper_type,
        robot_version=args.robot_version,
        control_aim=args.control_aim,
        control_mode='pv'  # Use any mode just to establish serial connection
    )
    
    try:
        logger.info("开始机械臂归零流程")
        logger.info("=" * 50)
        
        print()
        
        # Perform go home by sending command to STM32
        success = robot_go_home(
            robot,
            max_attempts=args.max_attempts,
            send_interval=args.send_interval
        )
        
        if success:
            logger.info("=" * 50)
            logger.info("机械臂归零完成！所有关节已回到零位")
            logger.info("=" * 50)
        else:
            logger.error("机械臂归零失败！")
        
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
    parser = argparse.ArgumentParser(description="Robot Go Home (Zero Position) Demo")
    
    # Serial port settings
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--gripper_type', type=str, default="100mm", help="夹爪类型")
    parser.add_argument('--robot_version', type=str, default="v1_1", help="机械臂版本")
    parser.add_argument('--control-aim', type=str, default='operation', choices=['teach', 'operation'],
                        help='Control aim: teach or operation (motor-specific)')
    
    # Go home parameters
    parser.add_argument('--max-attempts', type=int, default=100, 
                        help="最大发送次数 (默认: 100)")
    parser.add_argument('--send-interval', type=float, default=0.1, 
                        help="发送间隔时间/秒 (默认: 0.1)")
    
    args = parser.parse_args()
    
    main(args)
