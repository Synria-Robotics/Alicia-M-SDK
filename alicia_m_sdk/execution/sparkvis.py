"""
SparkVis WebSocket Bridge for Real Robot Integration

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL v3.0

This module provides WebSocket-based bidirectional communication between
SparkVis UI and real Alicia-D robots, enabling real-time synchronization
and data logging capabilities.
"""

import asyncio
import json
import os
import signal
import sys
import time
from datetime import datetime
from typing import Dict, List, Optional

try:
    import websockets  # pip install websockets==13.1
except ImportError:
    print("请先安装 websockets: pip install websockets==13.1")
    raise


class SparkVisBridge:
    """WebSocket bridge for SparkVis UI and robot synchronization."""
    
    def __init__(
        self,
        robot,
        host: str = "localhost",
        port: int = 8765,
        output_file: Optional[str] = None,
        enable_robot_sync: bool = True,
        robot_sync_rate_hz: float = 500.0,  # 提升到500Hz
        log_source: str = "ui",  # ui | robot | both
        speed_deg_s: Optional[List[float]] = None  # 关节速度设置（度/秒）
    ):
        """Initialize SparkVis bridge.
        
        Args:
            robot: Alicia-M robot instance
            host: WebSocket server host
            port: WebSocket server port
            output_file: CSV output file path (optional)
            enable_robot_sync: Enable robot->UI state broadcasting
            robot_sync_rate_hz: Robot state broadcast frequency in Hz (default: 200Hz)
            log_source: Data logging source ('ui', 'robot', or 'both')
            
        """
        self.robot = robot
        self.host = host
        self.port = port
        self.enable_robot_sync = enable_robot_sync
        self.robot_sync_rate_hz = robot_sync_rate_hz
        self.robot_sync_interval = 1.0 / max(1e-3, robot_sync_rate_hz)
        self.log_source = log_source.lower()
        self.speed_deg_s = speed_deg_s  # 保存速度设置
        

        # WebSocket clients
        self.websocket_connections = set()
        self._pending_robot_state: Optional[Dict[str, float]] = None

        # 缓存上一次的关节目标值，防止增量更新时未包含的关节归零
        self.last_joint_targets = [0.0] * 6
        try:
            # 尝试初始化为当前机器人状态
            current_joints = self.robot.get_robot_state("joint")
            if current_joints and len(current_joints) >= 6:
                self.last_joint_targets = list(current_joints[:6])
        except Exception as e:
            print(f"[Log] 初始化关节目标值失败: {e}")

        # CSV logging
        self.output_file = output_file
        self.file_handle = None
        if self.output_file:
            try:
                os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)
                self.file_handle = open(self.output_file, 'w')
                self.file_handle.write('timestamp,joint1,joint2,joint3,joint4,joint5,joint6,gripper\n')
                self.file_handle.flush()
                print(f"[Log] 数据记录启用: {self.output_file}")
            except Exception as e:
                print(f"[Log] 打开CSV失败: {e}")
                self.file_handle = None

        # graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        print(f"[Exit] 收到信号 {signum}，正在优雅关闭...")
        self.cleanup()
        sys.exit(0)

    def cleanup(self):
        """Clean up resources."""
        if self.file_handle:
            try:
                self.file_handle.flush()
                self.file_handle.close()
                print("[Log] CSV 已保存")
            except Exception as e:
                print(f"[Log] 关闭CSV失败: {e}")
            finally:
                self.file_handle = None

    @staticmethod
    def _now_str(ts: Optional[float] = None) -> str:
        """Get formatted timestamp string."""
        return datetime.fromtimestamp(ts or time.time()).strftime('%H:%M:%S.%f')[:-3]

    def read_robot_state(self) -> Optional[Dict[str, float]]:
        """Read current robot joint states and gripper position."""
        try:
            joints = self.robot.get_robot_state("joint")  # 6 rad
            gripper_value = self.robot.get_robot_state("gripper")  # 0-100
            if joints is None or gripper_value is None:
                return None
            
            # Convert gripper to percentage [0..1] for UI
            try:
                gripper_pct = max(0.0, min(1.0, float(gripper_value) / 100.0))
            except Exception:
                gripper_pct = 0.0
                
            return {
                "Joint1": float(joints[0]),
                "Joint2": float(joints[1]),
                "Joint3": float(joints[2]),
                "Joint4": float(joints[3]),
                "Joint5": float(joints[4]),
                "Joint6": float(joints[5]),
                "gripper": float(gripper_pct),
            }
        except Exception:
            return None

    def apply_ui_joint_update(self, joint_values: Dict[str, float]):
        """Apply joint updates received from UI to robot (optimized for 500Hz)."""
        try:
            import numpy as np

            # 辅助函数：兼容 Joint1 和 joint1 两种键名 (优化: 缓存键名查找)
            def get_val(key_suffix, default=0.0):
                # 优先检查最常见的格式
                key_joint = f'Joint{key_suffix}'
                if key_joint in joint_values:
                    return float(joint_values[key_joint])
                # 备选格式
                for k in [f'joint{key_suffix}', f'J{key_suffix}', f'j{key_suffix}']:
                    if k in joint_values:
                        return float(joint_values[k])
                return default

            # 从 UI 获取关节目标值（弧度）
            # 使用 self.last_joint_targets 作为默认值，防止未包含的关节归零
            joints_rad = [
                get_val('1', self.last_joint_targets[0]),
                get_val('2', self.last_joint_targets[1]),
                get_val('3', self.last_joint_targets[2]),
                get_val('4', self.last_joint_targets[3]),
                get_val('5', self.last_joint_targets[4]),
                get_val('6', self.last_joint_targets[5]),
            ]
            
            # 更新缓存
            self.last_joint_targets = joints_rad
            
            gripper_val = joint_values.get('gripper', 0.0)

            # 🚀 关键优化: 使用 set_joint_and_gripper_combined() 
            # 这个方法内部已经针对高频控制优化，将关节和夹爪控制合并为一个串口命令
            # 避免了两次串口通信的开销
            
            # 如果夹爪值在消息中，使用合并命令
            if 'gripper' in joint_values:
                pct = max(0.0, min(1.0, float(gripper_val)))
                gripper_angle = pct * 100.0
                
                # 合并关节和夹爪命令（单次串口通信）
                self.robot.servo_driver.set_joint_and_gripper(
                    joint_angles=joints_rad,
                    gripper_value=gripper_angle,
                    speed_deg_s=self.speed_deg_s[0] if self.speed_deg_s else 2.5,  # 默认 2.5 deg/s ≈ 0.0436 rad/s
                )
            else:
                # 仅关节控制
                # 注意: target_joints 是位置(rad)，speed_deg_s 是速度(deg/s)，两者单位独立
                self.robot.set_robot_state(
                    target_joints=joints_rad,        # 关节目标位置(弧度)
                    joint_format='rad',              # 位置单位: 弧度
                    speed_deg_s=100,                 # 运动速度: 度/秒
                    wait_for_completion=False,
                )

            # 记录 UI 命令到 CSV (优化: 减少 flush 频率)
            if self.file_handle and self.log_source in ("ui", "both"):
                row = f"{self._now_str()},{','.join(map(str, joints_rad))},{gripper_val}\n"
                self.file_handle.write(row)
                # 优化: 每100次才 flush 一次，减少 I/O 开销
                if not hasattr(self, '_csv_write_count'):
                    self._csv_write_count = 0
                self._csv_write_count += 1
                if self._csv_write_count >= 100:
                    self.file_handle.flush()
                    self._csv_write_count = 0
                
        except Exception as e:
            print(f"[Log] 应用UI关节更新失败: {e}")

    async def broadcast_robot_state(self, joint_data: Dict[str, float]):
        """Broadcast robot state to all connected WebSocket clients."""
        if not self.websocket_connections:
            return
        message = {
            'type': 'robot_state_update',
            'joint_values': joint_data,
            'timestamp': time.time()
        }
        disconnected = set()
        for ws in self.websocket_connections.copy():
            try:
                await ws.send(json.dumps(message))
            except Exception as e:
                print(f"[Log] 广播机器人状态失败: {e}")
                disconnected.add(ws)
        self.websocket_connections -= disconnected

    async def websocket_handler(self, websocket, path):
        """Handle WebSocket connections and messages."""
        self.websocket_connections.add(websocket)

        # Initial sync: push current robot state
        if self.enable_robot_sync:
            robot_state = self.read_robot_state()
            if robot_state:
                await websocket.send(json.dumps({
                    'type': 'robot_state_update',
                    'joint_values': robot_state,
                    'timestamp': time.time()
                }))

        # Periodic task: Robot → UI broadcasting
        async def periodic_robot_sender():
            while websocket in self.websocket_connections:
                if self.enable_robot_sync:
                    state = self.read_robot_state()
                    if state:
                        # Log robot state
                        if self.file_handle and self.log_source in ("robot", "both"):
                            row = f"{self._now_str()},{state['Joint1']},{state['Joint2']},{state['Joint3']},{state['Joint4']},{state['Joint5']},{state['Joint6']},{state['gripper']}\n"
                            self.file_handle.write(row)
                            self.file_handle.flush()
                        try:
                            await websocket.send(json.dumps({
                                'type': 'robot_state_update',
                                'joint_values': state,
                                'timestamp': time.time()
                            }))
                        except Exception:
                            break
                await asyncio.sleep(self.robot_sync_interval)

        sender_task = asyncio.create_task(periodic_robot_sender())

        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    msg_type = data.get('type')

                    if msg_type == 'joint_update':
                        joint_values = data.get('joint_values', {})
                        self.apply_ui_joint_update(joint_values)

                        # 发送确认
                        await websocket.send(json.dumps({
                            'type': 'joint_update_ack',
                            'timestamp': time.time(),
                            'success': True
                        }))

                    elif msg_type == 'request_robot_state':
                        state = self.read_robot_state()
                        if state:
                            await websocket.send(json.dumps({
                                'type': 'robot_state_update',
                                'joint_values': state,
                                'timestamp': time.time()
                            }))
                except json.JSONDecodeError:
                    pass
                except Exception as e:
                    print(f"[Log] 处理消息错误: {e}")   
        except Exception as e:
            print(f"[Log] 处理消息错误: {e}")
        finally:
            if 'sender_task' in locals():
                sender_task.cancel()
            self.websocket_connections.discard(websocket)

    def start_server(self):
        """Start the WebSocket server (200Hz sync rate)."""
        print(f"🚀 WebSocket: ws://{self.host}:{self.port} ({self.robot_sync_rate_hz}Hz)")

        async def server():
            async with websockets.serve(self.websocket_handler, self.host, self.port):
                await asyncio.Future()

        try:
            asyncio.run(server())
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()