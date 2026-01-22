#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Demo: SparkVis UI bidirectional synchronization and data logging (Real Robot)

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

Features:
- Start WebSocket server for UI ↔ Robot bidirectional sync
- UI → Robot: Receive joint_update and send directly to real robot
- Robot → UI: Periodically broadcast current robot state to UI (toggleable)
- Data logging: Record joint data from UI commands to CSV (optional)

Usage:
1. Start SparkVis backend server:
   cd SparkVis
   python backend_server.py

2. Start SparkVis web server:
   cd SparkVis
   python -m http.server 8080

3. Run this demo (robot bridge):
   cd Alicia-D-SDK/examples
   python 10_demo_sparkvis.py --port /dev/ttyUSB0

4. Open browser and visit:
   http://localhost:8080

Note: All three components must be running simultaneously for full functionality.
"""

import alicia_m_sdk
from alicia_m_sdk.hardware import ServoDriver
from alicia_m_sdk.execution.sparkvis import SparkVisBridge


def main(args):
    """SparkVis WebSocket bridge demonstration.
    
    :param args: Command line arguments   
    """
    # Initialize robot instance with control_aim and control_mode
    robot = alicia_m_sdk.create_robot(
        port=args.port,
        baudrate=args.baudrate,
        robot_version=args.robot_version,
        gripper_type=args.gripper_type,
        control_aim="teach",
        control_mode="pv"
    )

    try:
        # Connect to robot
        if not robot.connect():
            print("✗ 连接失败，请检查串口设置")
            return

        # Optional: Move to home position for safety
        try:
            robot.go_home()
        except Exception as e:
            print(f"✗ 移动到home位置失败: {e}")

        # Create and start SparkVis bridge
        bridge = SparkVisBridge(
            robot=robot,
            host=args.host,
            port=args.websocket_port,
            output_file=args.output_file or None,
            enable_robot_sync=args.enable_robot_sync,
            robot_sync_rate_hz=args.robot_sync_rate,
            log_source=args.log_source,
            # speed_rad_s=[0.0436, 0.0436, 0.0314, 0.0314, 0.0262, 0.0262, 0.0262],  # 关节速度设置（弧度/秒）
        )
        bridge.start_server()

    except KeyboardInterrupt:
        print("\n✗ Processing interrupted")
    finally:
        try:
            robot.disconnect()
        except Exception as e:
            print(f"✗ 断开连接失败: {e}")


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='SparkVis ↔ 真实机器人 同步与数据记录 Demo')
    
    # Robot connection settings
    parser.add_argument('--port', type=str, default='/dev/ttyCH343USB0', help='串口设备，如 /dev/ttyUSB0 或 COM3')
    parser.add_argument('--baudrate', type=int, default=1000000, help='串口波特率，默认1000000；若读取超时请尝试 921600')
    parser.add_argument('--robot_version', type=str, default='v1_1', help='机器人版本 (默认: v1_1)')
    parser.add_argument('--gripper_type', type=str, default='100mm', help='夹爪型号 (默认: 100mm)')
    
    # WebSocket settings
    parser.add_argument('--host', type=str, default='localhost', help='WebSocket主机')
    parser.add_argument('--websocket-port', type=int, default=8765, help='WebSocket端口')
    
    # Data logging settings
    parser.add_argument('--output-file', type=str, default='', help='CSV输出路径（留空不记录）')
    parser.add_argument('--log-source', type=str, default='ui', choices=['ui','robot','both'], help='记录UI指令/机器人状态/二者')
    # Robot sync settings
    parser.add_argument('--enable-robot-sync', action='store_true', help='启用 机器人→UI 状态同步')
    parser.add_argument('--robot-sync-rate', type=float, default=500.0, help='机器人状态广播频率 Hz (默认500Hz，建议范围50-500Hz)')
    
    args = parser.parse_args()
    main(args)

