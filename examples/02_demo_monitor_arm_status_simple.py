#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
实时监控机械臂状态位 - 简化版

直接使用 ServoDriver 底层接口，不依赖 SynriaRobotAPI
"""

import sys
import os
import time
import threading

# 直接导入底层模块，避免通过 __init__.py
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# 使用完整路径导入，避免触发 __init__.py
import importlib.util

def load_module_from_path(module_name, file_path):
    """从路径加载模块"""
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

# 获取基础路径
base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

# 加载依赖模块
logger_module = load_module_from_path(
    "beauty_logger",
    os.path.join(base_path, "alicia_m_sdk/utils/logger/beauty_logger.py")
)

serial_comm_module = load_module_from_path(
    "serial_comm",
    os.path.join(base_path, "alicia_m_sdk/hardware/serial_comm.py")
)

data_parser_module = load_module_from_path(
    "data_parser",
    os.path.join(base_path, "alicia_m_sdk/hardware/data_parser.py")
)

servo_driver_module = load_module_from_path(
    "servo_driver",
    os.path.join(base_path, "alicia_m_sdk/hardware/servo_driver.py")
)

# 获取类
ServoDriver = servo_driver_module.ServoDriver


def print_separator():
    """打印分隔线"""
    print("=" * 80)


def print_status_header():
    """打印状态表头"""
    print_separator()
    print("机械臂状态位实时监控")
    print_separator()


def print_basic_info(servo_driver, debug_mode=False):
    """打印基本信息"""
    device_type = servo_driver.data_parser.get_device_type()
    device_code = servo_driver.data_parser.get_device_code()
    
    print(f"\n【设备信息】")
    print(f"  设备类型: {device_type if device_type else '未知'}")
    if device_code is not None:
        print(f"  设备代码: 0x{device_code:02X}")
    else:
        print(f"  设备代码: 未知")


def print_raw_status(servo_driver, debug_mode=False):
    """打印原始状态信息（仅在debug模式）"""
    if not debug_mode:
        return
    
    status_byte = servo_driver.data_parser.get_run_status_byte()
    status_text = servo_driver.data_parser.get_run_status_text()
    
    print(f"\n【原始状态 - DEBUG】")
    if status_byte is not None:
        print(f"  状态字节: 0x{status_byte:02X} ({status_byte:08b})")
        print(f"  状态文本: {status_text if status_text else 'idle'}")
    else:
        print(f"  状态字节: 未接收")


def print_teach_status(status_dict):
    """打印示教臂状态位"""
    print(f"\n【示教臂状态位】")
    print(f"  ├─ 锁定状态 (locked):             {'✓ 是' if status_dict.get('locked') else '✗ 否'}")
    print(f"  ├─ 同步状态 (sync):                {'✓ 是' if status_dict.get('sync') else '✗ 否'}")
    print(f"  ├─ 待定1 (reserved1):              {'✓ 是' if status_dict.get('reserved1') else '✗ 否'}")
    print(f"  ├─ 待定2 (reserved2):              {'✓ 是' if status_dict.get('reserved2') else '✗ 否'}")
    print(f"  ├─ 待定3 (reserved3):              {'✓ 是' if status_dict.get('reserved3') else '✗ 否'}")
    print(f"  ├─ 待定4 (reserved4):              {'✓ 是' if status_dict.get('reserved4') else '✗ 否'}")
    print(f"  ├─ 夹具力矩锁定 (gripper_torque):  {'✓ 是' if status_dict.get('gripper_torque_lock') else '✗ 否'}")
    print(f"  └─ 电机错误 (motor_err):           {'✓ 是' if status_dict.get('motor_err') else '✗ 否'}")


def print_oper_status(status_dict):
    """打印操作臂状态位"""
    print(f"\n【操作臂状态位】")
    print(f"  ├─ 单击 (single_click):            {'✓ 是' if status_dict.get('single_click') else '✗ 否'}")
    print(f"  ├─ 双击 (double_click):            {'✓ 是' if status_dict.get('double_click') else '✗ 否'}")
    print(f"  ├─ 长按 (long_press):              {'✓ 是' if status_dict.get('long_press') else '✗ 否'}")
    print(f"  ├─ 重复长按 (repeat_long_press):   {'✓ 是' if status_dict.get('repeat_long_press') else '✗ 否'}")
    print(f"  ├─ 待定1 (reserved1):              {'✓ 是' if status_dict.get('reserved1') else '✗ 否'}")
    print(f"  ├─ 待定2 (reserved2):              {'✓ 是' if status_dict.get('reserved2') else '✗ 否'}")
    print(f"  ├─ 夹具力矩锁定 (gripper_torque):  {'✓ 是' if status_dict.get('gripper_torque_lock') else '✗ 否'}")
    print(f"  └─ 电机错误 (motor_err):           {'✓ 是' if status_dict.get('motor_err') else '✗ 否'}")


def print_current_status(servo_driver):
    """根据设备类型打印当前状态"""
    device_code = servo_driver.data_parser.get_device_code()
    
    if device_code == 0x01:  # 示教臂
        status = servo_driver.data_parser.get_teach_status()
        print_teach_status(status)
    elif device_code == 0x02:  # 操作臂
        status = servo_driver.data_parser.get_oper_status()
        print_oper_status(status)
    else:
        print(f"\n【状态位】")
        print(f"  设备类型未知或未连接")


def print_joint_state(servo_driver, debug_mode=False):
    """打印关节状态（仅在debug模式）"""
    if not debug_mode:
        return
    
    joint_state = servo_driver.data_parser.get_joint_state()
    
    print(f"\n【关节状态 - DEBUG】")
    if joint_state:
        print(f"  关节角度 (弧度): {[f'{a:.3f}' for a in joint_state.angles]}")
        print(f"  夹爪开合: {joint_state.gripper:.2f}%")
        print(f"  时间戳: {joint_state.timestamp:.3f}")
    else:
        print(f"  关节数据: 未接收")


def print_all_status(servo_driver, debug_mode=False):
    """打印所有状态信息"""
    print("\033[2J\033[H")  # 清屏并移动光标到左上角
    
    print_status_header()
    print_basic_info(servo_driver, debug_mode)
    print_raw_status(servo_driver, debug_mode)
    print_current_status(servo_driver)
    print_joint_state(servo_driver, debug_mode)
    print_separator()
    print("提示: 按 Ctrl+C 退出监控")
    print_separator()


def monitor_status(servo_driver, update_interval=0.005, debug_mode=False):
    """
    持续监控机械臂状态 (200Hz)
    
    Args:
        servo_driver: ServoDriver 实例
        update_interval: 更新间隔（秒，默认0.005s = 200Hz）
        debug_mode: 是否显示调试信息
    """
    print("开始监控机械臂状态位...")
    print(f"调试模式: {'开启' if debug_mode else '关闭'}")
    print(f"更新间隔: {update_interval}秒 ({1.0/update_interval:.0f} Hz)")
    
    try:
        while True:
            # 请求关节状态（这会触发状态位更新）
            servo_driver.acquire_info("joint", wait=True, timeout=1.0)
            
            # 打印所有状态
            print_all_status(servo_driver, debug_mode)
            
            # 等待下次更新
            time.sleep(update_interval)
            
    except KeyboardInterrupt:
        print("\n\n监控已停止")
    except Exception as e:
        print(f"\n\n错误: {str(e)}")
        import traceback
        traceback.print_exc()


def print_status_once(servo_driver, debug_mode=False):
    """
    打印一次当前状态（用于快速查看）
    
    Args:
        servo_driver: ServoDriver 实例
        debug_mode: 是否显示调试信息
    """
    # 请求关节状态
    servo_driver.acquire_info("joint", wait=True, timeout=2.0)
    
    # 打印所有状态
    print_all_status(servo_driver, debug_mode)
    
    # 在debug模式下显示完整信息
    if debug_mode:
        print("\n【完整状态信息 - DEBUG】")
        all_info = servo_driver.data_parser.get_all_status_info()
        import json
        print(json.dumps(all_info, indent=2, ensure_ascii=False))


def main():
    """主函数"""
    print("=" * 80)
    print("机械臂状态位监控工具")
    print("=" * 80)
    
    # 调试模式设置（默认关闭）
    DEBUG_MODE = False  # 设置为 True 可以看到原始数据、关节状态等详细信息
    
    print(f"\n调试模式: {'开启' if DEBUG_MODE else '关闭'}")
    print("(可以修改代码中的 DEBUG_MODE 来开启/关闭详细信息)")
    
    print("\n正在初始化...")
    
    # 创建 ServoDriver
    try:
        servo_driver = ServoDriver(
            port="",  # 自动搜索串口
            baudrate=1000000,
            debug_mode=False  # ServoDriver 的通信日志，保持关闭
        )
    except Exception as e:
        print(f"创建 ServoDriver 失败: {e}")
        return
    
    # 连接机械臂
    print("正在连接机械臂...")
    if not servo_driver.connect():
        print("连接失败！")
        print("请检查:")
        print("  1. 机械臂是否已连接到电脑")
        print("  2. 串口权限是否正确")
        print("  3. 是否有其他程序正在使用串口")
        return
    
    print("连接成功！")
    
    # 启动后台更新线程
    servo_driver.start_update_thread()
    time.sleep(0.2)  # 等待首次更新
    
    # 选择模式
    print("\n请选择运行模式:")
    print("1. 实时监控模式 (持续更新显示，推荐用于验证状态位变化)")
    print("2. 单次查看模式 (查看一次后退出)")
    
    try:
        choice = input("请输入选择 (1 或 2，默认 1): ").strip() or "1"
    except KeyboardInterrupt:
        print("\n\n已取消")
        servo_driver.disconnect()
        return
    
    if choice == "2":
        # 单次查看模式
        print_status_once(servo_driver, debug_mode=DEBUG_MODE)
    else:
        # 实时监控模式
        print("\n提示: 现在可以改变机械臂状态位，程序会实时显示变化")
        print("      例如: 按下示教臂按钮、锁定/解锁、触发错误等")
        time.sleep(0.5)
        monitor_status(servo_driver, update_interval=0.005, debug_mode=DEBUG_MODE)
    
    # 断开连接
    servo_driver.disconnect()
    print("已断开连接")


if __name__ == "__main__":
    main()
