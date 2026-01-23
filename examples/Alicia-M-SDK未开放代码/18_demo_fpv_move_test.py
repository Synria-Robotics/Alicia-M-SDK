#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) 2025 Synria Robotics Co., Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

"""
18_demo_fpv_move_test.py - 高频控制通信频率测试

功能描述:
    1. 使用SDK底层直接控制机械臂，绕过高层API开销
    2. 控制机械臂从点A→B→C→A循环运动
    3. 实时绘制控制频率曲线（学术风格）
    4. 目标：验证能否达到500Hz控制频率

运动轨迹:
    点A: [0, -15, -60, 0, 30, 0] 度
    点B: [45, -45, -45, 30, 0, 20] 度  
    点C: [-45, -90, -75, -30, -60, 40] 度
    
使用方法:
    python 18_demo_fpv_move_test.py --port /dev/ttyACM1 --duration 60
    python 18_demo_fpv_move_test.py --port /dev/ttyACM1 --no-plot  # 不显示图形
"""

import os
import sys
import time
import math
import argparse
import threading
import multiprocessing as mp
from collections import deque
from typing import List, Optional, Tuple

import numpy as np

# 直接导入底层模块,避免通过__init__.py的导入问题
import sys
import os
sdk_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if sdk_path not in sys.path:
    sys.path.insert(0, sdk_path)

# 直接从hardware模块导入,绕过api层
import importlib.util

def import_module_directly(module_name, file_path):
    """直接从文件导入模块"""
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module

# 导入logger (被servo_driver依赖)
logger_init_path = os.path.join(sdk_path, "alicia_m_sdk", "utils", "logger", "__init__.py")
if os.path.exists(logger_init_path):
    logger_module = import_module_directly("alicia_m_sdk.utils.logger", logger_init_path)

# 导入serial_comm
serial_comm_path = os.path.join(sdk_path, "alicia_m_sdk", "hardware", "serial_comm.py")
serial_comm_module = import_module_directly("alicia_m_sdk.hardware.serial_comm", serial_comm_path)
SerialComm = serial_comm_module.SerialComm

# 导入data_parser
data_parser_path = os.path.join(sdk_path, "alicia_m_sdk", "hardware", "data_parser.py")
data_parser_module = import_module_directly("alicia_m_sdk.hardware.data_parser", data_parser_path)
DataParser = data_parser_module.DataParser

# 导入comm_manager
comm_manager_path = os.path.join(sdk_path, "alicia_m_sdk", "hardware", "comm_manager.py")
comm_manager_module = import_module_directly("alicia_m_sdk.hardware.comm_manager", comm_manager_path)

# 导入servo_driver
servo_driver_path = os.path.join(sdk_path, "alicia_m_sdk", "hardware", "servo_driver.py")
servo_driver_module = import_module_directly("alicia_m_sdk.hardware.servo_driver", servo_driver_path)
ServoDriver = servo_driver_module.ServoDriver


# ==================== 配置参数 ====================
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi

# 目标点定义 (单位: 度)
POINT_A = [0.0, -45.0, -30.0, 0.0, 30.0, 0.0]
POINT_B = [45.0, -45.0, -85.0, 30.0, 0.0, 20.0]
POINT_C = [-45.0, -125.0, -55.0, -30.0, -60.0, 40.0]

# 运动参数
TARGET_SPEED_DEG_S = 50.0   # 目标速度 (度/秒)
INTERPOLATION_FREQ = 500    # 插值频率 (Hz) - 目标500Hz


class PerformanceStats:
    """性能统计数据 - 支持跨进程共享"""
    def __init__(self, maxlen: int = 600, use_mp: bool = False):
        """
        Args:
            maxlen: 历史记录最大长度
            use_mp: 是否使用多进程共享内存
        """
        self.use_mp = use_mp
        self.maxlen = maxlen
        
        if use_mp:
            # 多进程模式: 使用共享内存
            self._send_count = mp.Value('i', 0)
            self._recv_count = mp.Value('i', 0)
            self._error_count = mp.Value('i', 0)
            self._timeout_count = mp.Value('i', 0)
            self._start_time = mp.Value('d', 0.0)
            self._running = mp.Value('i', 1)
            self._cycle_count = mp.Value('i', 0)
            self._avg_latency = mp.Value('d', 0.0)
            
            # 用于绘图的共享数据队列
            self._plot_queue = mp.Queue(maxsize=100)
        else:
            # 单进程模式: 使用普通变量
            self.send_count: int = 0
            self.recv_count: int = 0
            self.error_count: int = 0
            self.start_time: float = 0.0
            
            # 历史记录用于绘图
            self.time_history = deque(maxlen=maxlen)
            self.send_fps_history = deque(maxlen=maxlen)
            self.recv_fps_history = deque(maxlen=maxlen)
            self.sync_rate_history = deque(maxlen=maxlen)
            self.position_history = deque(maxlen=maxlen)
            
            self.lock = threading.Lock()
            self.running = True
    
    # ========== 多进程模式的属性访问 ==========
    @property
    def send_count(self):
        return self._send_count.value if self.use_mp else self._send_count_val
    
    @send_count.setter
    def send_count(self, val):
        if self.use_mp:
            self._send_count.value = val
        else:
            self._send_count_val = val
    
    @property
    def recv_count(self):
        return self._recv_count.value if self.use_mp else self._recv_count_val
    
    @recv_count.setter  
    def recv_count(self, val):
        if self.use_mp:
            self._recv_count.value = val
        else:
            self._recv_count_val = val
    
    @property
    def running(self):
        return bool(self._running.value) if self.use_mp else self._running_val
    
    @running.setter
    def running(self, val):
        if self.use_mp:
            self._running.value = 1 if val else 0
        else:
            self._running_val = val
    
    def put_plot_data(self, data: dict):
        """向绘图进程发送数据"""
        if self.use_mp:
            try:
                self._plot_queue.put_nowait(data)
            except:
                pass  # 队列满则丢弃
    
    def get_plot_data(self, timeout: float = 0.1):
        """从队列获取绘图数据"""
        if self.use_mp:
            try:
                return self._plot_queue.get(timeout=timeout)
            except:
                return None
        return None


class TrajectoryInterpolator:
    """轨迹插值器 - 生成平滑轨迹"""
    
    def __init__(self, frequency: float = 500.0, speed_deg_s: float = 50.0):
        """
        Args:
            frequency: 插值频率 (Hz)
            speed_deg_s: 运动速度 (度/秒)
        """
        self.frequency = frequency
        self.dt = 1.0 / frequency
        self.speed_deg_s = speed_deg_s
        
    def interpolate_segment(self, start: List[float], end: List[float]) -> List[List[float]]:
        """
        线性插值生成从start到end的轨迹点
        
        Args:
            start: 起点关节角度 (度)
            end: 终点关节角度 (度)
            
        Returns:
            插值后的轨迹点列表
        """
        start = np.array(start)
        end = np.array(end)
        
        # 计算最大角度变化
        diff = end - start
        max_diff = np.max(np.abs(diff))
        
        # 计算运动时间
        if max_diff < 0.001:
            return [list(end)]
        
        duration = max_diff / self.speed_deg_s
        
        # 计算步数
        num_steps = max(int(duration * self.frequency), 1)
        
        # 生成插值点 (使用S曲线平滑)
        trajectory = []
        for i in range(num_steps + 1):
            # S曲线参数 (0->1)
            t = i / num_steps
            # 使用三次多项式平滑: 3t^2 - 2t^3
            s = 3 * t * t - 2 * t * t * t
            
            point = start + s * diff
            trajectory.append(list(point))
        
        return trajectory
    
    def generate_abc_trajectory(self) -> List[List[float]]:
        """生成 A→B→C→A 完整轨迹"""
        trajectory = []
        
        # A → B
        seg1 = self.interpolate_segment(POINT_A, POINT_B)
        trajectory.extend(seg1)
        
        # B → C
        seg2 = self.interpolate_segment(POINT_B, POINT_C)
        trajectory.extend(seg2[1:])  # 跳过第一个点(和上一段重复)
        
        # C → A
        seg3 = self.interpolate_segment(POINT_C, POINT_A)
        trajectory.extend(seg3[1:])  # 跳过第一个点
        
        return trajectory


class HighFrequencyController:
    """高频控制器 - 使用SDK底层同步接口实现高同步率"""
    
    def __init__(self, port: str, baudrate: int = 1000000, control_aim: int = 0x01,
                 sync_mode: bool = True, response_timeout: float = 0.002,
                 use_multiprocess: bool = False):
        """
        Args:
            port: 串口路径
            baudrate: 波特率
            control_aim: 控制目标 (0x01=示教, 0x02=操作)
            sync_mode: 是否使用同步模式 (发送后等待响应)
            response_timeout: 响应超时时间 (秒), 默认2ms
            use_multiprocess: 是否使用多进程模式 (绘图时推荐开启)
        """
        self.port = port
        self.baudrate = baudrate
        self.control_aim = control_aim
        self.sync_mode = sync_mode
        self.response_timeout = response_timeout
        self.use_multiprocess = use_multiprocess
        
        # 创建底层驱动 (不使用comm_manager,减少开销)
        self.driver = ServoDriver(
            port=port, 
            baudrate=baudrate,
            debug_mode=False,
            use_comm_manager=False,  # 关闭通信管理器,手动控制
            control_aim=control_aim
        )
        
        self.stats = PerformanceStats(use_mp=use_multiprocess)
        self.control_thread = None
        self.control_process = None
        self.current_trajectory = []
        self.trajectory_index = 0
        
    def connect(self) -> bool:
        """连接机械臂"""
        print(f"正在连接机械臂 (端口: {self.port})...")
        result = self.driver.connect()
        if result:
            print("✓ 连接成功!")
            # 启用低延迟模式
            self.driver.enable_low_latency_mode(True)
            print("✓ 低延迟模式已启用")
        return result
    
    def disconnect(self):
        """断开连接"""
        if self.use_multiprocess:
            self.stats._running.value = 0
            if self.control_process and self.control_process.is_alive():
                self.control_process.join(timeout=2.0)
                if self.control_process.is_alive():
                    self.control_process.terminate()
        else:
            self.stats.running = False
            if self.control_thread and self.control_thread.is_alive():
                self.control_thread.join(timeout=2.0)
        self.driver.disconnect()
        print("已断开连接")
    
    def _send_joint_command_sync(self, joint_angles_deg: List[float], speed_deg_s: float) -> Tuple[bool, Optional[List[int]], float]:
        """
        同步发送关节控制命令并等待响应 (高同步率版本)
        
        Args:
            joint_angles_deg: 目标角度 (度)
            speed_deg_s: 速度 (度/秒)
            
        Returns:
            (发送成功, 响应帧, 延迟ms)
        """
        # 转换为弧度
        joint_angles_rad = [a * DEG_TO_RAD for a in joint_angles_deg]
        
        # 使用SDK底层同步接口
        success, response, latency_ms = self.driver.set_joint_and_gripper_sync(
            joint_angles=joint_angles_rad,
            gripper_value=None,  # 不控制夹爪
            speed_deg_s=speed_deg_s,
            control_aim=self.control_aim,
            control_mode=self.driver.PATTERN_PV,
            response_timeout=self.response_timeout
        )
        
        return success, response, latency_ms
    
    def _send_joint_command_async(self, joint_angles_deg: List[float], speed_deg_s: float) -> Tuple[bool, Optional[List[int]]]:
        """
        异步发送关节控制命令 (原逻辑，用于对比测试)
        
        Args:
            joint_angles_deg: 目标角度 (度)
            speed_deg_s: 速度 (度/秒)
            
        Returns:
            (发送成功, 响应帧)
        """
        # 转换为弧度
        joint_angles_rad = [a * DEG_TO_RAD for a in joint_angles_deg]
        
        # 使用底层API构建并发送帧
        frame = self.driver._build_send_joint_frame(
            joint_angles=joint_angles_rad,
            gripper_value=None,
            speed_deg_s=speed_deg_s,
            control_aim=self.control_aim,
            control_mode=self.driver.PATTERN_PV
        )
        
        # 直接发送 (绕过comm_manager)
        send_success = self.driver.serial_comm.send_data(frame)
        
        if not send_success:
            return False, None
        
        # 尝试读取响应 (非阻塞)
        response = self.driver.serial_comm.read_frame()
        
        return True, response
    
    def _control_loop(self, duration: float, stats_mp=None):
        """高频控制循环 - 支持同步/异步模式和多进程
        
        Args:
            duration: 运行时长
            stats_mp: 多进程模式时的共享统计对象
        """
        interpolator = TrajectoryInterpolator(
            frequency=INTERPOLATION_FREQ,
            speed_deg_s=TARGET_SPEED_DEG_S
        )
        
        # 生成轨迹
        print("生成轨迹中...")
        trajectory = interpolator.generate_abc_trajectory()
        total_points = len(trajectory)
        print(f"✓ 轨迹生成完成: {total_points} 个点")
        
        # 控制参数
        target_interval = 1.0 / INTERPOLATION_FREQ
        trajectory_idx = 0
        cycle_count = 0
        
        # 统计变量 (本地)
        send_count = 0
        recv_count = 0
        error_count = 0
        
        # 延迟统计
        latency_history = deque(maxlen=1000)
        timeout_count = 0
        
        start_time = time.time()
        last_report_time = start_time
        last_second_send = 0
        last_second_recv = 0
        
        # 位置历史 (本地)
        position_history = deque(maxlen=600)
        
        # 多进程模式下使用共享状态
        use_mp = stats_mp is not None
        
        mode_str = "同步" if self.sync_mode else "异步"
        print(f"\n开始高频控制循环 ({mode_str}模式, 目标: {INTERPOLATION_FREQ} Hz, 时长: {duration}s)...\n")
        
        # 运行状态检查
        def is_running():
            if use_mp:
                return stats_mp._running.value == 1
            else:
                return self.stats.running
        
        while is_running() and (time.time() - start_time) < duration:
            loop_start = time.perf_counter()
            
            # 获取当前目标点
            target_point = trajectory[trajectory_idx]
            
            # 根据模式发送控制命令
            if self.sync_mode:
                # ===== 同步模式: 使用SDK底层同步接口 =====
                send_ok, response, latency_ms = self._send_joint_command_sync(
                    target_point, 
                    TARGET_SPEED_DEG_S
                )
                
                # 更新统计
                if send_ok:
                    send_count += 1
                    if response is not None:
                        recv_count += 1
                        latency_history.append(latency_ms)
                    else:
                        timeout_count += 1
                else:
                    error_count += 1
                
                # 记录当前位置
                elapsed = time.time() - start_time
                position_history.append((elapsed, target_point[0]))
            else:
                # ===== 异步模式: 原逻辑 =====
                send_ok, response = self._send_joint_command_async(
                    target_point, 
                    TARGET_SPEED_DEG_S
                )
                
                if send_ok:
                    send_count += 1
                    if response is not None:
                        recv_count += 1
                else:
                    error_count += 1
                
                elapsed = time.time() - start_time
                position_history.append((elapsed, target_point[0]))
            
            # 更新轨迹索引 (循环)
            trajectory_idx = (trajectory_idx + 1) % total_points
            if trajectory_idx == 0:
                cycle_count += 1
            
            # 每秒输出统计
            current_time = time.time()
            if current_time - last_report_time >= 1.0:
                elapsed = current_time - start_time
                
                # 计算瞬时频率 (本秒内)
                instant_send = send_count - last_second_send
                instant_recv = recv_count - last_second_recv
                last_second_send = send_count
                last_second_recv = recv_count
                
                # 平均频率
                sync_rate = (recv_count / send_count * 100) if send_count > 0 else 0
                
                # 平均延迟
                avg_latency = np.mean(latency_history) if len(latency_history) > 0 else 0
                
                # 多进程模式: 发送数据到绘图进程
                if use_mp:
                    plot_data = {
                        'elapsed': elapsed,
                        'instant_send': instant_send,
                        'instant_recv': instant_recv,
                        'sync_rate': sync_rate,
                        'avg_latency': avg_latency,
                        'cycle_count': cycle_count,
                        'timeout_count': timeout_count,
                        'position': target_point[0]
                    }
                    try:
                        stats_mp._plot_queue.put_nowait(plot_data)
                    except:
                        pass  # 队列满则丢弃
                else:
                    # 单进程模式: 直接更新 stats
                    with self.stats.lock:
                        self.stats.time_history.append(elapsed)
                        self.stats.send_fps_history.append(instant_send)
                        self.stats.recv_fps_history.append(instant_recv)
                        self.stats.sync_rate_history.append(sync_rate)
                        self.stats.send_count = send_count
                        self.stats.recv_count = recv_count
                
                if self.sync_mode:
                    print(f"[{elapsed:.1f}s] 发送: {instant_send} Hz | 接收: {instant_recv} Hz | "
                          f"同步: {sync_rate:.1f}% | 延迟: {avg_latency:.2f}ms | "
                          f"周期: {cycle_count} | 超时: {timeout_count}")
                else:
                    print(f"[{elapsed:.1f}s] 发送: {instant_send} Hz | 接收: {instant_recv} Hz | "
                          f"同步率: {sync_rate:.1f}% | 周期: {cycle_count} | 错误: {error_count}")
                last_report_time = current_time
            
            # 精确延时控制频率 (仅在异步模式下需要主动等待)
            if not self.sync_mode:
                elapsed_time = time.perf_counter() - loop_start
                if elapsed_time < target_interval:
                    # 自旋等待
                    target_time = loop_start + target_interval
                    while time.perf_counter() < target_time:
                        pass
        
        # 最终统计
        total_time = time.time() - start_time
        final_send_fps = send_count / total_time if total_time > 0 else 0
        final_recv_fps = recv_count / total_time if total_time > 0 else 0
        final_sync = (recv_count / send_count * 100) if send_count > 0 else 0
        
        # 延迟统计
        avg_latency = np.mean(latency_history) if len(latency_history) > 0 else 0
        max_latency = np.max(latency_history) if len(latency_history) > 0 else 0
        min_latency = np.min(latency_history) if len(latency_history) > 0 else 0
        
        # 发送结束信号到绘图进程
        if use_mp:
            try:
                stats_mp._plot_queue.put_nowait({'done': True})
            except:
                pass
        
        print("\n" + "="*70)
        print("高频控制测试结果".center(70))
        print("="*70)
        print(f"测试模式:       {'同步 (等待响应)' if self.sync_mode else '异步 (不等待)'}")
        print(f"测试时长:       {total_time:.2f} s")
        print(f"发送频率:       {final_send_fps:.2f} Hz (目标: {INTERPOLATION_FREQ} Hz)")
        print(f"接收频率:       {final_recv_fps:.2f} Hz")
        print(f"同步率:         {final_sync:.2f}%")
        print(f"完成周期:       {cycle_count}")
        print(f"总发送:         {send_count}")
        print(f"总接收:         {recv_count}")
        print(f"超时次数:       {timeout_count}")
        print(f"错误次数:       {error_count}")
        if self.sync_mode and len(latency_history) > 0:
            print(f"平均延迟:       {avg_latency:.3f} ms")
            print(f"最小延迟:       {min_latency:.3f} ms")
            print(f"最大延迟:       {max_latency:.3f} ms")
        print(f"频率达成率:     {(final_send_fps/INTERPOLATION_FREQ*100):.1f}%")
        print("="*70)
    
    def run(self, duration: float, stats_mp=None):
        """运行控制测试
        
        Args:
            duration: 运行时长
            stats_mp: 多进程共享统计对象 (绘图模式使用)
        """
        if stats_mp is not None:
            # 多进程模式: 在当前进程中直接运行
            self._control_loop(duration, stats_mp=stats_mp)
        else:
            # 单进程模式: 在线程中运行
            self.stats.running = True
            self.control_thread = threading.Thread(
                target=self._control_loop,
                args=(duration, None),
                daemon=True
            )
            self.control_thread.start()
            return self.control_thread


# ==================== 多进程控制函数 ====================

def _control_process_worker(port: str, baudrate: int, control_aim: int, 
                            sync_mode: bool, response_timeout: float,
                            duration: float, stats_mp):
    """
    控制进程工作函数 - 在独立进程中运行高频控制循环
    
    Args:
        port: 串口路径
        baudrate: 波特率  
        control_aim: 控制目标
        sync_mode: 同步模式
        response_timeout: 响应超时
        duration: 运行时长
        stats_mp: 共享的统计对象
    """
    # 在子进程中重新创建控制器
    controller = HighFrequencyController(
        port=port,
        baudrate=baudrate,
        control_aim=control_aim,
        sync_mode=sync_mode,
        response_timeout=response_timeout,
        use_multiprocess=True
    )
    
    try:
        if controller.connect():
            time.sleep(0.5)  # 等待连接稳定
            controller.run(duration, stats_mp=stats_mp)
        else:
            print("控制进程: 连接失败!")
    except Exception as e:
        print(f"控制进程异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        controller.driver.disconnect()
        print("控制进程: 已断开连接")


def run_with_plot(controller: HighFrequencyController, duration: float):
    """
    带实时绘图的运行模式 - 使用多进程隔离
    
    控制循环在独立进程中运行，绑图在主进程中运行。
    两者通过 multiprocessing.Queue 通信，避免 GIL 竞争。
    """
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    
    # 创建共享状态
    stats_mp = PerformanceStats(use_mp=True)
    
    # 启动控制进程
    control_process = mp.Process(
        target=_control_process_worker,
        args=(
            controller.port,
            controller.baudrate,
            controller.control_aim,
            controller.sync_mode,
            controller.response_timeout,
            duration,
            stats_mp
        ),
        daemon=True
    )
    control_process.start()
    print("✓ 控制进程已启动 (独立进程，不受绑图影响)")
    
    # 绘图数据历史 (主进程本地)
    time_history = deque(maxlen=600)
    send_fps_history = deque(maxlen=600)
    recv_fps_history = deque(maxlen=600)
    sync_rate_history = deque(maxlen=600)
    position_history = deque(maxlen=600)
    
    # 设置matplotlib
    plt.style.use('seaborn-v0_8-darkgrid')
    fig = plt.figure(figsize=(14, 10))
    fig.patch.set_facecolor('white')
    
    ax1 = plt.subplot(3, 1, 1)
    ax2 = plt.subplot(3, 1, 2)
    ax3 = plt.subplot(3, 1, 3)
    
    # 子图1: 通信频率
    ax1.set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    ax1.set_ylabel('Frequency (Hz)', fontsize=11, fontweight='bold')
    ax1.set_title('Communication Frequency (Target: 500 Hz)', fontsize=12, fontweight='bold', pad=10)
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.axhline(y=500, color='green', linestyle='--', linewidth=1, alpha=0.7, label='Target (500 Hz)')
    line1, = ax1.plot([], [], 'b-', linewidth=2, label='Send Frequency', alpha=0.8)
    line2, = ax1.plot([], [], 'r-', linewidth=2, label='Receive Frequency', alpha=0.8)
    ax1.legend(loc='upper right', fontsize=10, framealpha=0.9)
    
    # 子图2: 同步率
    ax2.set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    ax2.set_ylabel('Sync Rate (%)', fontsize=11, fontweight='bold')
    ax2.set_title('Communication Sync Rate', fontsize=12, fontweight='bold', pad=10)
    ax2.grid(True, alpha=0.3, linestyle='--')
    ax2.axhline(y=100, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    line3, = ax2.plot([], [], 'g-', linewidth=2, label='Sync Rate', alpha=0.8)
    ax2.legend(loc='lower right', fontsize=10, framealpha=0.9)
    
    # 子图3: 关节位置
    ax3.set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    ax3.set_ylabel('Joint 1 Position (deg)', fontsize=11, fontweight='bold')
    ax3.set_title('Joint 1 Trajectory (A→B→C→A)', fontsize=12, fontweight='bold', pad=10)
    ax3.grid(True, alpha=0.3, linestyle='--')
    line4, = ax3.plot([], [], 'm-', linewidth=1.5, label='Position', alpha=0.8)
    ax3.legend(loc='upper right', fontsize=10, framealpha=0.9)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # 控制循环是否结束
    control_done = [False]
    
    def animate(frame):
        # 从队列读取所有可用数据
        while True:
            try:
                data = stats_mp._plot_queue.get_nowait()
                if data.get('done'):
                    control_done[0] = True
                    break
                # 更新历史数据
                time_history.append(data['elapsed'])
                send_fps_history.append(data['instant_send'])
                recv_fps_history.append(data['instant_recv'])
                sync_rate_history.append(data['sync_rate'])
                position_history.append((data['elapsed'], data['position']))
            except:
                break
        
        # 检查控制进程是否结束
        if control_done[0] or not control_process.is_alive():
            ani.event_source.stop()
            return
        
        if len(time_history) < 2:
            return
        
        time_data = np.array(time_history)
        send_fps = np.array(send_fps_history)
        recv_fps = np.array(recv_fps_history)
        sync_rate = np.array(sync_rate_history)
        
        # 位置数据
        if len(position_history) > 1:
            pos_time = [p[0] for p in position_history]
            pos_val = [p[1] for p in position_history]
        else:
            pos_time, pos_val = [], []
        
        # 更新频率图
        line1.set_data(time_data, send_fps)
        line2.set_data(time_data, recv_fps)
        
        if len(time_data) > 0:
            ax1.set_xlim(max(0, time_data[-1] - 60), time_data[-1] + 2)
            fps_max = max(np.max(send_fps), 550) if len(send_fps) > 0 else 550
            fps_min = min(np.min(send_fps), 0) if len(send_fps) > 0 else 0
            ax1.set_ylim(max(0, fps_min - 50), fps_max + 50)
        
        # 更新同步率图
        line3.set_data(time_data, sync_rate)
        if len(time_data) > 0:
            ax2.set_xlim(max(0, time_data[-1] - 60), time_data[-1] + 2)
            ax2.set_ylim(max(90, np.min(sync_rate) - 2) if len(sync_rate) > 0 else 90, 101)
        
        # 更新位置图
        if len(pos_time) > 0:
            line4.set_data(pos_time, pos_val)
            ax3.set_xlim(max(0, pos_time[-1] - 10), pos_time[-1] + 1)
            ax3.set_ylim(-60, 60)
        
        # 更新标题
        current_send = send_fps[-1] if len(send_fps) > 0 else 0
        current_recv = recv_fps[-1] if len(recv_fps) > 0 else 0
        current_sync = sync_rate[-1] if len(sync_rate) > 0 else 0
        
        fig.suptitle(
            f'High-Frequency Control Performance Monitor (A→B→C→A @ {TARGET_SPEED_DEG_S}°/s)\n'
            f'Send: {current_send:.1f} Hz | Receive: {current_recv:.1f} Hz | Sync: {current_sync:.1f}%',
            fontsize=14, fontweight='bold'
        )
        
        return line1, line2, line3, line4
    
    ani = FuncAnimation(fig, animate, interval=100, blit=False, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        # 停止控制进程
        stats_mp._running.value = 0
        control_process.join(timeout=3.0)
        if control_process.is_alive():
            control_process.terminate()
            print("控制进程已强制终止")


def run_without_plot(controller: HighFrequencyController, duration: float):
    """无绘图模式运行"""
    control_thread = controller.run(duration)
    
    try:
        control_thread.join()
    except KeyboardInterrupt:
        print("\n用户中断")
        controller.stats.running = False
        control_thread.join(timeout=2.0)


def main():
    parser = argparse.ArgumentParser(
        description="高频控制通信频率测试 - 机械臂A→B→C→A运动",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 同步模式 (默认，高同步率)
  python 18_demo_fpv_move_test.py --port /dev/ttyACM1 --duration 60
  
  # 异步模式 (原逻辑，用于对比)
  python 18_demo_fpv_move_test.py --port /dev/ttyACM1 --async
  
  # 不显示图形
  python 18_demo_fpv_move_test.py --port /dev/ttyACM1 --no-plot
  
  # 调整响应超时 (更低延迟但可能降低同步率)
  python 18_demo_fpv_move_test.py --port /dev/ttyACM1 --timeout 0.0015

轨迹点定义:
  点A: [0, -45, -30, 0, 30, 0] 度
  点B: [45, -45, -85, 30, 0, 20] 度
  点C: [-45, -125, -55, -30, -60, 40] 度
        """
    )
    
    parser.add_argument('--port', type=str, default="/dev/ttyACM1",
                       help="串口端口 (默认: /dev/ttyACM1)")
    parser.add_argument('--baudrate', type=int, default=1000000,
                       help="波特率 (默认: 1000000)")
    parser.add_argument('--duration', type=float, default=60.0,
                       help="测试时长 (秒, 默认: 60)")
    parser.add_argument('--control-aim', type=str, default="teach",
                       choices=['teach', 'operation'],
                       help="控制目标: teach(示教臂) 或 operation(操作臂), 默认: teach")
    parser.add_argument('--no-plot', action='store_true',
                       help="不显示实时图形")
    parser.add_argument('--speed', type=float, default=50.0,
                       help="运动速度 (度/秒, 默认: 50)")
    parser.add_argument('--freq', type=int, default=500,
                       help="目标控制频率 (Hz, 默认: 500)")
    parser.add_argument('--async', dest='async_mode', action='store_true',
                       help="使用异步模式 (原逻辑，同步率较低)")
    parser.add_argument('--timeout', type=float, default=0.002,
                       help="响应超时时间 (秒, 默认: 0.002即2ms)")
    
    args = parser.parse_args()
    
    # 更新全局参数
    global TARGET_SPEED_DEG_S, INTERPOLATION_FREQ
    TARGET_SPEED_DEG_S = args.speed
    INTERPOLATION_FREQ = args.freq
    
    # 控制目标
    control_aim = 0x01 if args.control_aim == 'teach' else 0x02
    
    # 同步模式 (默认True，除非指定--async)
    sync_mode = not args.async_mode
    
    print("\n" + "="*70)
    print("高频控制通信频率测试".center(70))
    print("="*70)
    print(f"端口:           {args.port}")
    print(f"波特率:         {args.baudrate}")
    print(f"控制目标:       {'示教臂' if args.control_aim == 'teach' else '操作臂'} (0x{control_aim:02X})")
    print(f"目标频率:       {INTERPOLATION_FREQ} Hz")
    print(f"运动速度:       {TARGET_SPEED_DEG_S} °/s")
    print(f"测试时长:       {args.duration} s")
    print(f"通信模式:       {'同步 (等待响应)' if sync_mode else '异步 (不等待)'}")
    print(f"响应超时:       {args.timeout*1000:.1f} ms")
    print(f"显示图形:       {'否' if args.no_plot else '是'}")
    print("="*70)
    
    # 显示轨迹点
    print("\n轨迹点:")
    print(f"  点A: {POINT_A}")
    print(f"  点B: {POINT_B}")
    print(f"  点C: {POINT_C}")
    print(f"  路径: A → B → C → A (循环)")
    print()
    
    # 创建控制器
    controller = HighFrequencyController(
        port=args.port,
        baudrate=args.baudrate,
        control_aim=control_aim,
        sync_mode=sync_mode,
        response_timeout=args.timeout
    )
    
    try:
        # 连接
        if not controller.connect():
            print("连接失败!")
            return 1
        
        # 等待一下让连接稳定
        time.sleep(0.5)
        
        # 运行测试
        if args.no_plot:
            run_without_plot(controller, args.duration)
        else:
            run_with_plot(controller, args.duration)
        
        return 0
        
    except KeyboardInterrupt:
        print("\n用户中断测试")
        return 0
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        controller.disconnect()


if __name__ == "__main__":
    sys.exit(main())
