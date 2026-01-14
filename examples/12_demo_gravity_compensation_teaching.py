#!/usr/bin/env python3
"""
重力补偿拖动示教示例

功能:
1. 使用MIT扭矩模式进行重力补偿
2. 200Hz高频采样录制轨迹
3. 使用MIT位置模式高频回放

使用:
    python 09_demo_gravity_compensation_teaching.py --port /dev/ttyUSB0 --mode record --motion pick_place
    python 09_demo_gravity_compensation_teaching.py --port /dev/ttyUSB0 --mode replay --motion pick_place

作者: AI Assistant
日期: 2026-01-04
"""

import argparse
import sys
import os

# 添加SDK路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import alicia_m_sdk
from alicia_m_sdk.hardware import ServoDriver
from alicia_m_sdk.execution import GravityCompensationTeaching, HardwareExecutor
from alicia_m_sdk.execution.drag_teaching import list_available_motions
import json


def parse_args():
    parser = argparse.ArgumentParser(description='重力补偿拖动示教')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0',
                       help='串口路径')
    parser.add_argument('--mode', type=str, choices=['record', 'replay', 'list'], 
                       default='record',
                       help='工作模式: record=录制, replay=回放, list=列出动作')
    parser.add_argument('--motion', type=str, default='test_motion',
                       help='动作名称')
    parser.add_argument('--urdf', type=str, 
                       default='Alicia_M_v1_1_gripper_100mm.urdf',
                       help='URDF文件路径')
    parser.add_argument('--sample-hz', type=float, default=200.0,
                       help='采样频率 (Hz), 默认200')
    parser.add_argument('--playback-hz', type=float, default=200.0,
                       help='回放频率 (Hz), 默认200')
    parser.add_argument('--torque-scale', type=float, default=1.0,
                       help='扭矩缩放系数, 默认1.0')
    parser.add_argument('--debug', action='store_true',
                       help='启用调试模式')
    
    return parser.parse_args()


def main():
    args = parse_args()
    
    # 模式：列出动作
    if args.mode == 'list':
        motions = list_available_motions()
        print("\n=== 可用的动作列表 ===")
        if not motions:
            print("未找到任何已录制的动作")
            return
        
        for i, motion in enumerate(motions, 1):
            motion_dir = os.path.join("example_motions", motion)
            meta_path = os.path.join(motion_dir, "meta.json")
            
            info = f"{i:2d}. {motion}"
            if os.path.exists(meta_path):
                try:
                    with open(meta_path, 'r') as f:
                        meta = json.load(f)
                    mode = meta.get('mode', 'unknown')
                    count = meta.get('count', 0)
                    duration = meta.get('duration', 0)
                    info += f" (模式: {mode}, 点数: {count}, 时长: {duration:.2f}s)"
                except:
                    pass
            print(info)
        return
    
    # 检查URDF文件
    if not os.path.exists(args.urdf):
        print(f"[错误] 未找到URDF文件: {args.urdf}")
        print("[提示] 请确保URDF文件在当前目录或指定正确路径")
        return
    
    # 连接机器人
    print(f"\n[连接] 串口: {args.port}")
    robot = alicia_m_sdk.create_robot(
        port=args.port,
        debug_mode=args.debug,
        control_aim="teach",
        control_mode="mit_torque"
    )
    
    if not robot.connect():
        print("[错误] 无法连接到机器人")
        return
    
    try:
        # 模式：录制
        if args.mode == 'record':
            print("\n" + "="*60)
            print("  重力补偿拖动示教 - 录制模式")
            print("="*60)
            print(f"动作名称: {args.motion}")
            print(f"URDF文件: {args.urdf}")
            print(f"采样频率: {args.sample_hz} Hz")
            print(f"扭矩缩放: {args.torque_scale}")
            print()
            
            # 创建示教对象
            teaching = GravityCompensationTeaching(
                controller=robot,
                urdf_path=args.urdf,
                sample_hz=args.sample_hz,
                torque_scale=args.torque_scale,
                debug=args.debug
            )
            
            # 交互式录制
            teaching.run_interactive(args.motion)
        
        # 模式：回放
        elif args.mode == 'replay':
            print("\n" + "="*60)
            print("  重力补偿拖动示教 - 回放模式")
            print("="*60)
            print(f"动作名称: {args.motion}")
            print(f"回放频率: {args.playback_hz} Hz")
            print()
            
            # 加载轨迹
            motion_dir = os.path.join("example_motions", args.motion)
            traj_path = os.path.join(motion_dir, "joint_traj.json")
            meta_path = os.path.join(motion_dir, "meta.json")
            
            if not os.path.exists(traj_path):
                print(f"[错误] 未找到轨迹文件: {traj_path}")
                print("\n可用的动作:")
                motions = list_available_motions()
                for motion in motions[:5]:
                    print(f"  - {motion}")
                return
            
            # 读取轨迹数据
            with open(traj_path, 'r') as f:
                trajectory = json.load(f)
            
            # 读取元信息
            meta = {}
            if os.path.exists(meta_path):
                with open(meta_path, 'r') as f:
                    meta = json.load(f)
            
            print(f"[加载] 轨迹点数: {len(trajectory)}")
            print(f"[加载] 原始频率: {meta.get('sample_hz', 'Unknown')} Hz")
            print(f"[加载] 时长: {meta.get('duration', 0):.2f}秒")
            print()
            
            # 提取关节角度和夹爪
            joint_traj = [point['q'] for point in trajectory]
            gripper_traj = [point.get('grip', 0.0) for point in trajectory]
            
            # 确认回放
            confirm = input("是否回放轨迹? (y/n): ").strip().lower()
            if confirm != 'y':
                print("[取消] 用户取消回放")
                return
            
            # 使用HardwareExecutor回放
            executor = HardwareExecutor(robot.servo_driver)
            
            print(f"\n[回放] 使用MIT位置模式回放...")
            print(f"[回放] 频率: {args.playback_hz} Hz")
            
            # 使用MIT位置模式高频回放
            executor.execute(
                joint_traj=joint_traj,
                gripper_traj=gripper_traj,
                visualize=False,
                interaction=False,
                use_mit_mode=True,
                playback_hz=args.playback_hz
            )
            
            print("\n[完成] 轨迹回放完成！")
    
    except KeyboardInterrupt:
        print("\n[中断] 用户中断")
    except Exception as e:
        print(f"[错误] 运行失败: {e}")
        if args.debug:
            import traceback
            traceback.print_exc()
    finally:
        robot.disconnect()
        print("[断开] 已断开连接")


if __name__ == "__main__":
    main()
