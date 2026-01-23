"""
Demo: Control robot joints using sliders with high frequency control (~200Hz)

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL 

Features:
- 7 sliders for 6 joints + 1 gripper control
- Range: -180° to 180° for joints, 0-100 for gripper
- High frequency control (~200Hz)
- Real-time joint position feedback display
"""

import alicia_m_sdk
from alicia_m_sdk.hardware import ServoDriver
import tkinter as tk
from tkinter import ttk
import threading
import time
import numpy as np
import signal
import sys
import os


class JointSliderController:
    """GUI controller for robot joint control using sliders."""
    
    def __init__(self, robot, speed_deg_s=200):
        """Initialize the slider controller.
        
        :param robot: Robot API instance
        :param speed_deg_s: Joint speed in degrees per second
        """
        self.robot = robot
        self.running = False
        self.control_thread = None
        self.shutdown_event = threading.Event()  # 关闭事件（默认未设置）
        
        # Control parameters - 500Hz control frequency
        self.control_frequency = 200  # Hz (降低到200Hz以确保稳定)
        self.control_interval = 1.0 / self.control_frequency  # 0.005 seconds
        
        # Speed parameter (in deg/s for all 6 joints)
        self.speed_deg_s = speed_deg_s
    
        
        # Joint limits (degrees)
        self.joint_min = [-150.0,-170.0, -170.0, -80.0, -80.0, -80.0]  # 6个关节的最小值
        self.joint_max = [ 150.0,   0.0,    0.0,  80.0,  80.0,  80.0] 
        self.gripper_min = 0.0
        self.gripper_max = 100.0
        
        # Current target values (degrees)
        self.target_joints = [0.0] * 7  # 6 joints + 1 gripper
        
        # Slider variables
        self.slider_vars = []
        
        # Create GUI
        self._create_gui()
        
    def _create_gui(self):
        """Create the GUI with sliders."""
        try:
            self.root = tk.Tk()
            self.root.title("机械臂关节滑动条控制 - Joint Slider Control")
            self.root.geometry("800x600")
            self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
        except Exception as e:
            print(f"✗ 创建GUI窗口失败: {e}")
            print("提示: 请确保在有图形界面的环境中运行此程序")
            print("     或使用 ssh -X 连接以启用X11转发")
            raise
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        title_label = ttk.Label(main_frame, text="机械臂关节滑动条控制", 
                                font=('Arial', 16, 'bold'))
        title_label.pack(pady=10)
        
        # Control frequency info
        freq_label = ttk.Label(main_frame, 
                               text=f"控制频率: {self.control_frequency} Hz | 速度: {self.speed_deg_s}°/s",
                               font=('Arial', 10))
        freq_label.pack(pady=5)
        
        # Sliders frame
        sliders_frame = ttk.Frame(main_frame)
        sliders_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        # Joint names
        joint_names = ["关节1 (J1)", "关节2 (J2)", "关节3 (J3)", 
                       "关节4 (J4)", "关节5 (J5)", "关节6 (J6)", "夹爪 (Gripper)"]
        
        # Create sliders for each joint
        for i, name in enumerate(joint_names):
            self._create_slider_row(sliders_frame, i, name)
        
        # Control buttons frame
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=10)
        
        # Start/Stop button
        self.control_button = ttk.Button(button_frame, text="开始控制", 
                                         command=self._toggle_control)
        self.control_button.pack(side=tk.LEFT, padx=5)
        
        # Reset button
        reset_button = ttk.Button(button_frame, text="归零位置", 
                                  command=self._reset_sliders)
        reset_button.pack(side=tk.LEFT, padx=5)
        
        # Home button
        home_button = ttk.Button(button_frame, text="回零位", 
                                 command=self._go_home)
        home_button.pack(side=tk.LEFT, padx=5)
        
        # Status frame
        status_frame = ttk.LabelFrame(main_frame, text="状态信息", padding="5")
        status_frame.pack(fill=tk.X, pady=10)
        
        # Current position display
        self.position_label = ttk.Label(status_frame, text="当前位置: --", 
                                        font=('Consolas', 10))
        self.position_label.pack(anchor=tk.W)
        
        # Target position display
        self.target_label = ttk.Label(status_frame, text="目标位置: --", 
                                      font=('Consolas', 10))
        self.target_label.pack(anchor=tk.W)
        
        # Control status
        self.status_label = ttk.Label(status_frame, text="状态: 未开始", 
                                      font=('Arial', 10), foreground='gray')
        self.status_label.pack(anchor=tk.W)
        
        # Start position update
        self._update_position_display()
        
    def _create_slider_row(self, parent, index, name):
        """Create a slider row for a joint.
        
        :param parent: Parent frame
        :param index: Joint index (0-6)
        :param name: Joint name
        """
        row_frame = ttk.Frame(parent)
        row_frame.pack(fill=tk.X, pady=5)
        
        # Joint name label
        name_label = ttk.Label(row_frame, text=name, width=15)
        name_label.pack(side=tk.LEFT, padx=5)
        
        # Slider variable
        if index < 6:
            # Joint slider - 使用各自的最小值和最大值
            var = tk.DoubleVar(value=0.0)
            slider = ttk.Scale(row_frame, from_=self.joint_min[index], to=self.joint_max[index],
                            orient=tk.HORIZONTAL, variable=var, length=400,
                            command=lambda val, idx=index: self._on_slider_change(idx, val))
        else:
            # Gripper slider: 0 to 100
            var = tk.DoubleVar(value=50.0)
            slider = ttk.Scale(row_frame, from_=self.gripper_min, to=self.gripper_max,
                            orient=tk.HORIZONTAL, variable=var, length=400,
                            command=lambda val, idx=index: self._on_slider_change(idx, val))
        slider.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        # Value display label
        value_label = ttk.Label(row_frame, text="0.0°", width=10)
        value_label.pack(side=tk.LEFT, padx=5)
        
        # Store references
        self.slider_vars.append({
            'var': var,
            'label': value_label,
            'slider': slider
        })
        
    def _on_slider_change(self, index, value):
        """Handle slider value change.
        
        :param index: Joint index
        :param value: New slider value
        """
        value = float(value)
        self.target_joints[index] = value
        
        # Update value label
        if index < 6:
            self.slider_vars[index]['label'].config(text=f"{value:.1f}°")
        else:
            self.slider_vars[index]['label'].config(text=f"{value:.1f}%")
            
    def _toggle_control(self):
        """Toggle control on/off."""
        if self.running:
            self._stop_control()
        else:
            self._start_control()
            
    def _start_control(self):
        """Start the control loop."""
        if self.running:
            return
            
        self.running = True
        self.control_button.config(text="停止控制")
        self.status_label.config(text="状态: 控制中...", foreground='green')
        
        # Start control thread
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        
    def _stop_control(self):
        """Stop the control loop."""
        self.running = False
        self.shutdown_event.set()  # 设置关闭事件
        self.control_button.config(text="开始控制")
        self.status_label.config(text="状态: 已停止", foreground='orange')
        
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=0.5)  # 缩短等待时间
            self.control_thread = None
        
        # 重置关闭事件，以便下次可以再次启动
        self.shutdown_event.clear()
            
    def _control_loop(self):
        """Main control loop running at 200Hz."""
        print("✓ 控制循环已启动")
        
        while self.running and not self.shutdown_event.is_set():
            loop_start = time.time()
            
            try:
                # Get current target values from sliders
                target = self.target_joints.copy()
                
                # Speed is in deg/s
                speed_deg_s = self.speed_deg_s
                
                # 使用 set_robot_state 方法发送控制命令
                # 注意：wait_for_completion=False 表示不等待到达目标位置，实现高频控制
                # 注意：target 已经是度数，所以使用 joint_format='deg'
                success = self.robot.set_robot_state(
                    target_joints=target[:6],  # 直接传入度数
                    gripper_value=target[6],
                    joint_format='deg',  # 明确指定输入是角度
                    wait_for_completion=False,
                    speed_deg_s=speed_deg_s
                )
                
                if not success:
                    print(f"⚠ 控制命令发送失败")
                
            except Exception as e:
                print(f"✗ 控制循环错误: {e}")
                import traceback
                traceback.print_exc()
                
            # Calculate sleep time to maintain control frequency
            elapsed = time.time() - loop_start
            sleep_time = max(0, self.control_interval - elapsed)
            
            if sleep_time > 0:
                # 使用 Event 的 wait 方法，这样可以立即响应关闭信号
                if self.shutdown_event.wait(sleep_time):
                    # 如果 wait 返回 True，说明事件被设置，应该退出
                    break
        
        print("✓ 控制循环已停止")
                
    def _reset_sliders(self):
        """Reset all sliders to zero position."""
        for i, slider_data in enumerate(self.slider_vars):
            if i < 6:
                slider_data['var'].set(0.0)
                slider_data['label'].config(text="0.0°")
                self.target_joints[i] = 0.0
            else:
                slider_data['var'].set(50.0)
                slider_data['label'].config(text="50.0%")
                self.target_joints[i] = 50.0
                
    def _go_home(self):
        """Move robot to home position."""
        try:
            # Set all sliders to zero
            self._reset_sliders()
            
            # If control is running, it will automatically send the zero position
            if not self.running:
                # Send zero position command using go_home method
                self.robot.go_home(speed_deg_s=self.speed_deg_s)
            
            self.status_label.config(text="状态: 正在回零位...", foreground='blue')
            
        except Exception as e:
            self.status_label.config(text=f"状态: 错误 - {e}", foreground='red')
            print(f"✗ 回零位错误: {e}")
            import traceback
            traceback.print_exc()
            
    def _update_position_display(self):
        """Update the position display labels at 20Hz (reduced from 10Hz for better feedback)."""
        try:
            # Get current joint positions
            current_joints = self.robot.get_robot_state("joint")
            
            if current_joints is not None:
                # Convert to degrees
                current_deg = [np.rad2deg(a) for a in current_joints]
                pos_str = " | ".join([f"J{i+1}:{v:6.1f}°" for i, v in enumerate(current_deg[:6])])
                self.position_label.config(text=f"当前位置: {pos_str}")
            else:
                self.position_label.config(text="当前位置: 无法读取")
                
            # Update target display
            target_str = " | ".join([f"J{i+1}:{v:6.1f}°" for i, v in enumerate(self.target_joints[:6])])
            self.target_label.config(text=f"目标位置: {target_str}")
            
        except Exception as e:
            # 静默处理错误，避免干扰GUI
            self.position_label.config(text="当前位置: 读取错误")
            
        # Schedule next update (20Hz for display - faster feedback)
        try:
            if hasattr(self, 'root') and self.root and self.root.winfo_exists():
                self.root.after(50, self._update_position_display)
        except tk.TclError:
            # 窗口已关闭，停止更新
            pass
            
    def _on_closing(self):
        """Handle window close event."""
        self._stop_control()
        self.shutdown_event.set()  # 确保关闭事件被设置
        
        try:
            self.root.quit()  # 退出主循环
            self.root.destroy()  # 销毁窗口
        except:
            pass
        
    def run(self):
        """Start the GUI main loop."""
        try:
            self.root.mainloop()
        except Exception as e:
            print(f"Tkinter主循环异常: {e}")
            import traceback
            traceback.print_exc()
            raise


def main(args):
    """Main function to run the slider control demo.
    
    :param args: Command line arguments
    """
    print("=" * 60)
    print("机械臂滑动条控制 - Joint Slider Control")
    print("=" * 60)
    print(f"控制频率: 200 Hz")
    print(f"关节范围: -180° ~ 180°")
    print(f"夹爪范围: 0% ~ 100%")
    print("按 Ctrl+C 可快速退出")
    print("=" * 60)
    
    # Initialize robot instance with control_aim and control_mode
    robot = alicia_m_sdk.create_robot(
        port=args.port,
        baudrate=args.baudrate,
        version=args.version,
        gripper_type=args.gripper_type,
        debug_mode=args.debug,
        control_aim=args.control_aim,
        control_mode=args.control_mode
    )
    
    controller = None

    def signal_handler(sig, frame):
        """处理 Ctrl+C 信号."""
        print("\n✗ 收到中断信号，正在关闭...")
        if controller:
            controller._on_closing()
        robot.disconnect()
        sys.exit(0)

    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)

    try:
        # Connect to robot
        if not robot.connect():
            print("✗ 连接失败，请检查串口设置")
            return
            
        print("✓ 机器人连接成功")
        
        # Create and run the slider controller
        try:
            controller = JointSliderController(robot, speed_deg_s=args.speed_deg_s)
            controller.run()
        except tk.TclError as e:
            print(f"\n✗ GUI显示错误: {e}")
            print("\n可能的解决方案:")
            print("1. 如果通过SSH连接，请使用: ssh -X user@host")
            print("2. 或在本地有图形界面的机器上运行此程序")
            print("3. 或使用VNC/远程桌面连接")
            raise

    except KeyboardInterrupt:
        print("\n✗ 用户中断")
    except tk.TclError as e:
        print(f"\n✗ GUI错误: {e}")
        print("\n这是一个需要图形界面的程序。")
        print("请确保:")
        print("  1. 在本地有图形界面的环境中运行")
        print("  2. 或使用 ssh -X 启用X11转发")
        print("  3. 或使用 VNC/远程桌面")
    except Exception as e:
        print(f"✗ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        robot.disconnect()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="机械臂滑动条控制示例")
    
    parser.add_argument('--port', type=str, default="/dev/ttyUSB0", 
                        help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  
                        help="波特率 (默认: 1000000)")
    parser.add_argument('--version', type=str, default="v1_1",  
                        help="机器人版本 (默认: v1_1)")
    parser.add_argument('--gripper_type', type=str, default="100mm",  
                        help="夹爪型号 (默认: 100mm)")
    parser.add_argument('--control-aim', type=str, default='operation', choices=['teach', 'operation'],
                        help='Control aim: teach (0x01示教臂) or operation (0x02操作臂) (默认: operation)')
    parser.add_argument('--control-mode', type=str, default='pv', choices=['pv', 'pvt', 'v', 'mit', 'mit_position', 'mit_speed', 'mit_torque'],
                        help='Control mode: pv, pvt, v, mit, mit_position, mit_speed, mit_torque (默认: pv)')
    parser.add_argument('--speed_deg_s', type=int, default=300, help="关节运动速度 (单位: 度/秒，默认: 300，范围: 10-400度/秒)")
    parser.add_argument('--debug', action='store_true',
                        help="启用调试模式")
    
    args = parser.parse_args()
    main(args)
