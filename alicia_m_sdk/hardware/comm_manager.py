# Copyright (c) 2025 Synria Robotics Co., Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

"""
CommunicationManager - 单线程串口通信管理器

设计目标：
1. 所有串口读写操作都由单一通信线程执行，消除竞争
2. 支持优先级队列：控制命令 > 查询命令
3. 支持同步等待响应机制
4. 保持与现有 ServoDriver 接口兼容
"""

import threading
import queue
import time
from typing import List, Optional, Dict, Callable, Any
from dataclasses import dataclass, field
from enum import IntEnum
from alicia_m_sdk.hardware.serial_comm import SerialComm
from alicia_m_sdk.utils.logger import logger


class CommandPriority(IntEnum):
    """命令优先级定义"""
    CRITICAL = 0    # 最高优先级：紧急停止等
    CONTROL = 1     # 控制命令：关节运动、夹爪控制
    QUERY = 2       # 查询命令：状态查询
    LOW = 3         # 低优先级：非紧急查询


@dataclass(order=True)
class CommandItem:
    """命令队列项，支持优先级排序"""
    priority: int
    timestamp: float = field(compare=False)  # 用于同优先级 FIFO
    cmd_id: int = field(compare=False)
    data: List[int] = field(compare=False)
    need_response: bool = field(compare=False, default=True)
    callback: Optional[Callable[[Optional[List[int]]], None]] = field(compare=False, default=None)
    response_event: Optional[threading.Event] = field(compare=False, default=None)
    response_data: Optional[List[int]] = field(compare=False, default=None)
    timeout: float = field(compare=False, default=0.05)  # 响应超时时间


class CommunicationManager:
    """
    单线程串口通信管理器
    
    核心原则：
    1. 只有通信线程可以操作串口（读/写）
    2. 其他线程通过队列提交命令
    3. 控制命令优先于查询命令
    4. 支持同步和异步两种模式
    
    使用示例：
        # 创建管理器
        comm_mgr = CommunicationManager(serial_comm, data_parser)
        comm_mgr.start()
        
        # 同步发送控制命令（等待响应）
        response = comm_mgr.send_command(
            data=control_cmd, 
            priority=CommandPriority.CONTROL,
            wait=True, 
            timeout=0.05
        )
        
        # 异步发送查询命令（不等待）
        comm_mgr.send_command(
            data=query_cmd, 
            priority=CommandPriority.QUERY,
            wait=False
        )
        
        # 停止
        comm_mgr.stop()
    """
    
    def __init__(self, 
                 serial_comm: SerialComm, 
                 data_parser,
                 response_timeout: float = 0.05,
                 debug_mode: bool = False):
        """
        初始化通信管理器
        
        Args:
            serial_comm: 串口通信实例
            data_parser: 数据解析器实例
            response_timeout: 默认响应超时时间（秒）
            debug_mode: 是否启用调试模式
        """
        self.serial_comm = serial_comm
        self.data_parser = data_parser
        self.response_timeout = response_timeout
        self.debug_mode = debug_mode
        
        # 命令队列（优先级队列）
        self._cmd_queue = queue.PriorityQueue()
        
        # 命令ID计数器
        self._cmd_id_counter = 0
        self._cmd_id_lock = threading.Lock()
        
        # 等待响应的命令映射：{cmd_id: CommandItem}
        self._pending_responses: Dict[int, CommandItem] = {}
        self._pending_lock = threading.Lock()
        
        # 通信线程控制
        self._comm_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._running = False
        
        # 状态统计
        self._stats = {
            "commands_sent": 0,
            "commands_success": 0,
            "commands_timeout": 0,
            "frames_received": 0,
            "queue_peak_size": 0,
        }
        self._stats_lock = threading.Lock()
        
        # 后台查询控制
        self._auto_query_enabled = False
        self._auto_query_interval = 0.02  # 50Hz 默认查询频率
        self._last_query_time = 0.0
        self._query_builder: Optional[Callable[[], List[int]]] = None
        
        # 用户命令时间戳（用于智能避让）
        self._last_user_cmd_time = 0.0
        self._user_cmd_cooldown = 0.01  # 用户命令后的冷却时间
    
    def start(self) -> bool:
        """启动通信管理线程"""
        if self._running:
            logger.warning("CommunicationManager already running")
            return True
        
        if not self.serial_comm.is_connected():
            logger.error("Serial port not connected, cannot start CommunicationManager")
            return False
        
        self._stop_event.clear()
        self._running = True
        
        self._comm_thread = threading.Thread(
            target=self._communication_loop,
            name="CommManager",
            daemon=True
        )
        self._comm_thread.start()
        
        logger.info("CommunicationManager started")
        return True
    
    def stop(self):
        """停止通信管理线程"""
        if not self._running:
            return
        
        self._stop_event.set()
        self._running = False
        
        # 清空队列中的等待命令
        self._clear_pending_commands()
        
        if self._comm_thread and self._comm_thread.is_alive():
            self._comm_thread.join(timeout=2.0)
        
        self._comm_thread = None
        logger.info("CommunicationManager stopped")
    
    def is_running(self) -> bool:
        """检查通信管理器是否运行中"""
        return self._running and self._comm_thread is not None and self._comm_thread.is_alive()
    
    def _get_next_cmd_id(self) -> int:
        """获取下一个命令ID（线程安全）"""
        with self._cmd_id_lock:
            self._cmd_id_counter += 1
            return self._cmd_id_counter
    
    def send_command(self,
                     data: List[int],
                     priority: CommandPriority = CommandPriority.CONTROL,
                     wait: bool = False,
                     timeout: Optional[float] = None,
                     callback: Optional[Callable[[Optional[List[int]]], None]] = None
                     ) -> Optional[List[int]]:
        """
        发送命令到通信队列
        
        Args:
            data: 命令数据（字节列表）
            priority: 命令优先级
            wait: 是否等待响应
            timeout: 响应超时时间（秒），None 使用默认值
            callback: 响应回调函数（异步模式）
            
        Returns:
            如果 wait=True，返回响应数据或 None（超时）
            如果 wait=False，返回 None
        """
        if not self._running:
            logger.warning("CommunicationManager not running, cannot send command")
            return None
        
        cmd_id = self._get_next_cmd_id()
        actual_timeout = timeout if timeout is not None else self.response_timeout
        
        # 更新用户命令时间戳
        if priority <= CommandPriority.CONTROL:
            self._last_user_cmd_time = time.perf_counter()
        
        # 创建命令项
        cmd_item = CommandItem(
            priority=priority,
            timestamp=time.perf_counter(),
            cmd_id=cmd_id,
            data=data,
            need_response=wait or callback is not None,
            callback=callback,
            response_event=threading.Event() if wait else None,
            response_data=None,
            timeout=actual_timeout
        )
        
        # 如果需要等待响应，注册到待处理映射
        if cmd_item.need_response:
            with self._pending_lock:
                self._pending_responses[cmd_id] = cmd_item
        
        # 放入队列
        self._cmd_queue.put(cmd_item)
        
        # 更新队列峰值统计
        queue_size = self._cmd_queue.qsize()
        with self._stats_lock:
            if queue_size > self._stats["queue_peak_size"]:
                self._stats["queue_peak_size"] = queue_size
        
        # 如果需要同步等待
        if wait and cmd_item.response_event:
            # 等待响应
            if cmd_item.response_event.wait(actual_timeout):
                # 响应已收到
                with self._pending_lock:
                    if cmd_id in self._pending_responses:
                        response = self._pending_responses[cmd_id].response_data
                        del self._pending_responses[cmd_id]
                        return response
            else:
                # 超时
                with self._pending_lock:
                    if cmd_id in self._pending_responses:
                        del self._pending_responses[cmd_id]
                with self._stats_lock:
                    self._stats["commands_timeout"] += 1
                if self.debug_mode:
                    logger.warning(f"Command {cmd_id} timeout after {actual_timeout}s")
                return None
        
        return None
    
    def send_control_command(self, data: List[int], wait: bool = True, timeout: float = 0.05) -> Optional[List[int]]:
        """
        发送控制命令（高优先级快捷方法）
        
        Args:
            data: 命令数据
            wait: 是否等待响应
            timeout: 超时时间
            
        Returns:
            响应数据或 None
        """
        return self.send_command(data, CommandPriority.CONTROL, wait, timeout)
    
    def send_query_command(self, data: List[int], wait: bool = False, timeout: float = 0.1) -> Optional[List[int]]:
        """
        发送查询命令（低优先级快捷方法）
        
        Args:
            data: 命令数据
            wait: 是否等待响应
            timeout: 超时时间
            
        Returns:
            响应数据或 None
        """
        return self.send_command(data, CommandPriority.QUERY, wait, timeout)
    
    def enable_auto_query(self, 
                          query_builder: Callable[[], List[int]], 
                          interval: float = 0.02):
        """
        启用自动后台查询
        
        Args:
            query_builder: 构建查询命令的函数
            interval: 查询间隔（秒）
        """
        self._query_builder = query_builder
        self._auto_query_interval = interval
        self._auto_query_enabled = True
        logger.info(f"Auto query enabled with interval {interval*1000:.1f}ms")
    
    def disable_auto_query(self):
        """禁用自动后台查询"""
        self._auto_query_enabled = False
        logger.info("Auto query disabled")
    
    def set_auto_query_interval(self, interval: float):
        """设置自动查询间隔"""
        self._auto_query_interval = interval
    
    def _communication_loop(self):
        """
        通信线程主循环
        
        职责：
        1. 从队列取命令（优先级排序）
        2. 发送命令到串口
        3. 等待并读取响应
        4. 分发响应给等待者
        5. 在空闲时执行自动查询
        """
        logger.info("Communication loop started")
        
        while not self._stop_event.is_set():
            try:
                cmd_item = None
                
                # 尝试从队列获取命令（非阻塞，短超时）
                try:
                    cmd_item = self._cmd_queue.get(timeout=0.001)
                except queue.Empty:
                    # 队列为空，检查是否需要自动查询
                    self._handle_auto_query()
                    continue
                
                if cmd_item is None:
                    continue
                
                # 发送命令
                send_success = self._send_and_receive(cmd_item)
                
                with self._stats_lock:
                    self._stats["commands_sent"] += 1
                    if send_success:
                        self._stats["commands_success"] += 1
                
            except Exception as e:
                logger.error(f"Communication loop exception: {e}")
                time.sleep(0.01)  # 避免错误循环
        
        logger.info("Communication loop stopped")
    
    def _send_and_receive(self, cmd_item: CommandItem) -> bool:
        """
        发送命令并等待响应
        
        Args:
            cmd_item: 命令项
            
        Returns:
            是否成功
        """
        # 发送命令
        if not self.serial_comm.send_data(cmd_item.data):
            logger.warning(f"Failed to send command {cmd_item.cmd_id}")
            self._notify_response(cmd_item.cmd_id, None)
            return False
        
        if self.debug_mode:
            hex_str = ' '.join(f'{b:02X}' for b in cmd_item.data)
            logger.debug(f"[TX] cmd_id={cmd_item.cmd_id} priority={cmd_item.priority} data={hex_str}")
        
        # 等待并读取响应
        timeout_start = time.perf_counter()
        response = None
        
        while (time.perf_counter() - timeout_start) < cmd_item.timeout:
            frame = self.serial_comm.read_frame()
            
            if frame is None:
                time.sleep(0.0001)  # 0.1ms 短暂休眠
                continue
            
            if frame == 9999999:
                logger.error("Severe serial communication error")
                break
            
            # 收到响应帧
            with self._stats_lock:
                self._stats["frames_received"] += 1
            
            if self.debug_mode:
                hex_str = ' '.join(f'{b:02X}' for b in frame)
                logger.debug(f"[RX] cmd_id={cmd_item.cmd_id} data={hex_str}")
            
            # 解析响应
            self.data_parser.parse_frame(frame)
            response = frame
            break
        
        # 通知等待者
        if cmd_item.need_response:
            self._notify_response(cmd_item.cmd_id, response)
        
        return response is not None
    
    def _notify_response(self, cmd_id: int, response: Optional[List[int]]):
        """通知等待响应的线程"""
        with self._pending_lock:
            if cmd_id in self._pending_responses:
                cmd_item = self._pending_responses[cmd_id]
                cmd_item.response_data = response
                
                # 触发等待事件
                if cmd_item.response_event:
                    cmd_item.response_event.set()
                
                # 调用回调
                if cmd_item.callback:
                    try:
                        cmd_item.callback(response)
                    except Exception as e:
                        logger.error(f"Response callback error: {e}")
    
    def _handle_auto_query(self):
        """处理自动后台查询"""
        if not self._auto_query_enabled or self._query_builder is None:
            time.sleep(0.001)  # 避免忙等待
            return
        
        current_time = time.perf_counter()
        
        # 智能避让：如果用户刚发送命令，暂缓查询
        time_since_user_cmd = current_time - self._last_user_cmd_time
        if time_since_user_cmd < self._user_cmd_cooldown:
            return
        
        # 检查查询间隔
        if (current_time - self._last_query_time) < self._auto_query_interval:
            return
        
        # 构建并发送查询命令
        try:
            query_data = self._query_builder()
            if query_data:
                # 直接发送，不通过队列（避免优先级反转）
                if self.serial_comm.send_data(query_data):
                    # 读取响应
                    timeout_start = time.perf_counter()
                    while (time.perf_counter() - timeout_start) < 0.03:
                        frame = self.serial_comm.read_frame()
                        if frame and frame != 9999999:
                            self.data_parser.parse_frame(frame)
                            with self._stats_lock:
                                self._stats["frames_received"] += 1
                            break
                        time.sleep(0.0001)
                
                self._last_query_time = current_time
        except Exception as e:
            logger.error(f"Auto query error: {e}")
    
    def _clear_pending_commands(self):
        """清空所有待处理的命令"""
        # 清空队列
        while not self._cmd_queue.empty():
            try:
                self._cmd_queue.get_nowait()
            except queue.Empty:
                break
        
        # 通知所有等待者（超时）
        with self._pending_lock:
            for cmd_id, cmd_item in self._pending_responses.items():
                if cmd_item.response_event:
                    cmd_item.response_event.set()
            self._pending_responses.clear()
    
    def get_stats(self) -> Dict[str, Any]:
        """获取统计信息"""
        with self._stats_lock:
            stats = self._stats.copy()
        
        stats["queue_size"] = self._cmd_queue.qsize()
        stats["pending_responses"] = len(self._pending_responses)
        stats["running"] = self._running
        stats["auto_query_enabled"] = self._auto_query_enabled
        
        return stats
    
    def reset_stats(self):
        """重置统计信息"""
        with self._stats_lock:
            self._stats = {
                "commands_sent": 0,
                "commands_success": 0,
                "commands_timeout": 0,
                "frames_received": 0,
                "queue_peak_size": 0,
            }
