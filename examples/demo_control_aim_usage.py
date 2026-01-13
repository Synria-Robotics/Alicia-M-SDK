#!/usr/bin/env python3
"""
示例：使用不同的控制目标（示教臂 vs 操作臂）

本示例演示如何：
1. 创建指定控制目标的机械臂实例
2. 在运行时切换控制目标
3. 验证不同控制目标的功能码
"""

import sys
import os
import time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from alicia_m_sdk.hardware.servo_driver import ServoDriver
from alicia_m_sdk.utils.logger import logger

def demo_operation_arm():
    """演示操作臂控制（默认）"""
    print("\n" + "=" * 80)
    print("演示1: 操作臂控制（默认）")
    print("=" * 80)
    
    # 创建操作臂实例（默认）
    driver = ServoDriver(port="", baudrate=1000000, debug_mode=True)
    print(f"默认控制目标: 0x{driver.default_control_aim:02X}")
    
    # 如果连接成功，请求关节信息
    if driver.connect():
        print("\n请求操作臂关节信息...")
        # 使用默认控制目标（操作臂）
        success = driver.acquire_info("joint", wait=False, timeout=1.0)
        print(f"请求结果: {'成功' if success else '失败'}")
        
        # 获取状态
        time.sleep(0.5)
        state = driver.data_parser.get_joint_state()
        if state:
            print(f"关节角度: {[f'{a:.3f}' for a in state.angles]}")
            print(f"夹爪: {state.gripper:.3f}")
            print(f"状态: {state.run_status_text}")
            
            # 获取操作臂特定状态
            oper_status = driver.data_parser.get_oper_status()
            print(f"操作臂状态: {oper_status}")
        
        driver.disconnect()
    else:
        print("无法连接到设备（这是正常的，仅演示）")
    
    print("=" * 80)

def demo_teaching_arm():
    """演示示教臂控制"""
    print("\n" + "=" * 80)
    print("演示2: 示教臂控制")
    print("=" * 80)
    
    # 创建示教臂实例
    driver = ServoDriver(
        port="", 
        baudrate=1000000, 
        control_aim=ServoDriver.AIM_TEACH,  # 指定为示教臂
        debug_mode=True
    )
    print(f"默认控制目标: 0x{driver.default_control_aim:02X}")
    
    # 如果连接成功，请求关节信息
    if driver.connect():
        print("\n请求示教臂关节信息...")
        # 使用默认控制目标（示教臂）
        success = driver.acquire_info("joint", wait=False, timeout=1.0)
        print(f"请求结果: {'成功' if success else '失败'}")
        
        # 获取状态
        time.sleep(0.5)
        state = driver.data_parser.get_joint_state()
        if state:
            print(f"关节角度: {[f'{a:.3f}' for a in state.angles]}")
            print(f"夹爪: {state.gripper:.3f}")
            print(f"状态: {state.run_status_text}")
            
            # 获取示教臂特定状态
            teach_status = driver.data_parser.get_teach_status()
            print(f"示教臂状态: {teach_status}")
        
        driver.disconnect()
    else:
        print("无法连接到设备（这是正常的，仅演示）")
    
    print("=" * 80)

def demo_runtime_switch():
    """演示运行时切换控制目标"""
    print("\n" + "=" * 80)
    print("演示3: 运行时切换控制目标")
    print("=" * 80)
    
    # 创建操作臂实例
    driver = ServoDriver(port="", baudrate=1000000, debug_mode=True)
    print(f"初始默认目标: 0x{driver.default_control_aim:02X} (操作臂)")
    
    if driver.connect():
        # 使用操作臂目标
        print("\n1. 使用操作臂目标请求...")
        driver.acquire_info("joint", control_aim=ServoDriver.AIM_OPERATION)
        
        # 临时切换到示教臂目标
        print("\n2. 临时切换到示教臂目标...")
        driver.acquire_info("joint", control_aim=ServoDriver.AIM_TEACH)
        
        # 恢复使用默认目标（操作臂）
        print("\n3. 恢复使用默认目标...")
        driver.acquire_info("joint")  # 不指定，使用默认
        
        driver.disconnect()
    else:
        print("无法连接到设备（这是正常的，仅演示）")
    
    print("=" * 80)

def demo_control_frame_difference():
    """演示控制帧的差异"""
    print("\n" + "=" * 80)
    print("演示4: 控制帧功能码差异")
    print("=" * 80)
    
    driver = ServoDriver(port="", baudrate=1000000, debug_mode=False)
    
    # 构建操作臂控制帧
    print("\n1. 操作臂控制帧:")
    frame_op = driver._build_send_joint_frame(
        joint_angles=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        gripper_value=50.0,
        speed_rad_s=1.0,
        control_aim=ServoDriver.AIM_OPERATION
    )
    print(f"   功能码: 0x{frame_op[2]:02X} (应为 0x82)")
    print(f"   完整帧: {' '.join(f'{b:02X}' for b in frame_op[:10])}...")
    
    # 构建示教臂控制帧
    print("\n2. 示教臂控制帧:")
    frame_teach = driver._build_send_joint_frame(
        joint_angles=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        gripper_value=50.0,
        speed_rad_s=1.0,
        control_aim=ServoDriver.AIM_TEACH
    )
    print(f"   功能码: 0x{frame_teach[2]:02X} (应为 0x81)")
    print(f"   完整帧: {' '.join(f'{b:02X}' for b in frame_teach[:10])}...")
    
    # 验证差异
    print("\n3. 差异分析:")
    print(f"   功能码差异: 0x{frame_op[2]:02X} vs 0x{frame_teach[2]:02X}")
    print(f"   差异位置: frame[2]")
    print(f"   其他字节: {'相同' if frame_op[3:] == frame_teach[3:] else '不同'}")
    
    print("=" * 80)

if __name__ == "__main__":
    print("\n" + "=" * 80)
    print("控制目标 (control_aim) 使用示例")
    print("=" * 80)
    print("\n本示例演示如何使用不同的控制目标（示教臂 vs 操作臂）")
    print("注意：如果没有连接真实设备，部分功能会显示失败，这是正常的。")
    
    demo_operation_arm()
    demo_teaching_arm()
    demo_runtime_switch()
    demo_control_frame_difference()
    
    print("\n" + "=" * 80)
    print("✅ 演示完成！")
    print("=" * 80)
    print("\n关键要点:")
    print("  1. 默认控制目标是 AIM_OPERATION (0x02, 操作臂)")
    print("  2. 可在初始化时指定 control_aim 参数")
    print("  3. 可在每次调用时覆盖 control_aim 参数")
    print("  4. 请求帧功能码: 0x01=示教臂, 0x02=操作臂")
    print("  5. 控制帧功能码: 0x81=示教臂, 0x82=操作臂")
    print("  6. DataParser 自动识别并解析不同设备的状态")
    print("=" * 80 + "\n")
