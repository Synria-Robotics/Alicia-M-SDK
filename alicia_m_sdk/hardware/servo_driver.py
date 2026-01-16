import math
import time
import logging
import threading
from typing import List, Optional, Union, Tuple, Dict
import numpy as np

from alicia_m_sdk.hardware.serial_comm import SerialComm
from alicia_m_sdk.hardware.data_parser import DataParser, JointState
from alicia_m_sdk.utils.logger import logger


class ServoDriver:
    """Robot arm control module"""
    
    # Constant definitions
    
    RAD_TO_DEG = 180.0 / math.pi  # 弧度转角度系数
    DEG_TO_RAD = math.pi / 180.0  # 角度转弧度系数
    # Frame constants
    FRAME_HEADER = 0xAA
    FRAME_FOOTER = 0xFF

    # Command IDs
    CMD_JOINT = 0x06       # Arm joint angle feedback and control
    
    # ==================== 控制目标常量 (Control Aim) ====================
    # 功能码 = 0x80 + aim (写入模式)
    AIM_TEACH = 0x01        # 示教机械臂
    AIM_OPERATION = 0x02    # 操作机械臂
    AIM_HEAD = 0x04         # 机器人头部
    AIM_LOIN = 0x08         # 机器人腰部
    AIM_LEG = 0x10          # 机器人下肢
    AIM_UNDERPAN = 0x20     # 机器人底盘
    
    # ==================== 控制模式常量 (Control Pattern) ====================
    # 前缀格式: [数据类型标志 + 模式类型, 每电机字节数/2]
    # 数据类型标志: 0x00=写入, 0x80=反馈
    # 
    # 模式参数要求:
    # - PATTERN_PV:  需要全部参数 (位置 + 速度)，每电机4字节
    # - PATTERN_PVT: 需要全部参数 (位置 + 速度 + 扭矩)，每电机6字节
    # - PATTERN_V:   需要全部参数 (速度)，每电机2字节
    # - PATTERN_MIT_*: MIT模式，只发送单一参数，每电机2字节
    PATTERN_PV = (0x00, 0x02)      # 位置+速度模式，每电机4字节
    PATTERN_PVT = (0x00, 0x03)     # 位置+速度+扭矩模式，每电机6字节
    PATTERN_V = (0x01, 0x01)       # 速度模式，每电机2字节
    
    # MIT 模式 - 只发送单一参数，每电机2字节
    # 前缀1: 0x02 = MIT模式标志
    # 前缀2: 0x01 = 每电机2字节 (2/2=1)
    PATTERN_MIT_POSITION = (0x00, 0x01)  # MIT模式-仅位置，每电机2字节
    PATTERN_MIT_SPEED = (0x01, 0x01)     # MIT模式-仅速度，每电机2字节  
    PATTERN_MIT_TORQUE = (0x02, 0x01)    # MIT模式-仅扭矩，每电机2字节
    
    # 兼容旧代码：PATTERN_MIT 默认为扭矩模式
    PATTERN_MIT = (0x02, 0x01)           # MIT模式-仅扭矩（默认），每电机2字节
    
    # ==================== 数据类型标志 ====================
    DATA_FLAG_WRITE = 0x00         # 写入数据
    DATA_FLAG_FEEDBACK = 0x80      # 反馈数据
    
    # Gripper type configuration

    # Old values (0-4095 scale)
    # GRI_MAX_50MM = 3290
    # GRI_MAX_100MM = 3590
    
    # New values based on -12.5~12.5 rad -> 0~65535 mapping
    # 0 rad -> 32768
    # 2.64 rad -> 39688
    GRI_VAL_OPEN = 39688   # 2.64 rad
    GRI_VAL_CLOSE = 32768  # 0 rad
    
    # Raw instruction mapping table for information retrieval and control
    # 注意：这些指令是通用模板，实际使用时需要根据 control_aim 动态构建
    INFO_COMMAND_MAP: Dict[str, List[int]] = {
        # Get firmware version查询固件版（通用指令，不受control_aim影响）
        "version": [0xAA, 0x01, 0x7E, 0x01, 0xFE, 0x79, 0xFF],
        # Set current position as zero将当前姿态设为零位（通用指令）
        "zero_cali": [0xAA, 0x03, 0x00, 0x01, 0xFE, 0xA8, 0xFF], 
        # Torque on/off使能/关闭力矩（通用指令）
        "torque_on": [0xAA, 0x05, 0x00, 0x01, 0x01, 0xF9, 0xFF],
        "torque_off": [0xAA, 0x05, 0x00, 0x01, 0x00, 0x6F, 0xFF],
        # Joint information acquisition请求关节信息
        # 注意：此处为模板，实际使用时需根据 control_aim 动态构建
        # "joint": [0xAA, 0x06, 0x02, 0x02,0x00, 0x01, 0xCE, 0xFF],  # 旧的硬编码操作臂指令
    }
    
    # MIT模式初始化指令 - 必须在使用MIT控制模式前发送
    # 这是一个特殊的初始化包,让固件进入MIT控制状态
    # 格式: 功能码=0x81(AIM_TEACH写入), 模式=(0x00, 0x05)每电机10字节
    MIT_INIT_COMMAND = bytes.fromhex(
        "AA 06 81 48 00 05 "
        "FF 7F FF 07 FF 07 99 19 66 66 "  # Motor 1
        "FF 7F FF 07 FF 07 99 19 66 66 "  # Motor 2
        "FF 7F FF 07 FF 07 99 19 66 66 "  # Motor 3
        "FF 7F FF 07 FF 07 3D 0A 33 33 "  # Motor 4
        "FF 7F FF 07 FF 07 3D 0A 33 33 "  # Motor 5
        "FF 7F FF 07 FF 07 3D 0A 33 33 "  # Motor 6
        "FF 7F FF 07 FF 07 3D 0A 33 33 "  # Motor 7 (Gripper)
        "44 FF"
    )
    def __init__(self, port, baudrate=1000000, debug_mode=False, firmware_version=None, robot_type=None, gripper_type="100mm", auto_init_mit=False, control_aim=None, **kwargs):
        """
        接受额外的关键字参数以保持向后兼容（例如 firmware_version）。
        真实实现的初始化逻辑放在下面（或保留原有代码）。
        
        Args:
            auto_init_mit: 是否自动初始化MIT模式。默认False，需要手动运行00_config_mit_params.py配置。
                          设置为True则在首次使用MIT模式时自动发送初始化指令（会使电机回零点）。
            control_aim: 默认控制目标。默认为 AIM_OPERATION (0x02 操作臂)。
                        可选: AIM_TEACH (0x01 示教臂), AIM_OPERATION (0x02 操作臂)等
        """
        self.port = port
        self.baudrate = baudrate
        self.debug_mode = debug_mode
        # 保存可选信息，供外部使用或日志检查
        self.firmware_version = firmware_version
        self.robot_type = robot_type
        self.gripper_type = gripper_type
        self.auto_init_mit = auto_init_mit  # 是否自动初始化MIT模式
        # 设置默认控制目标为操作臂（AIM_OPERATION）
        self.default_control_aim = control_aim if control_aim is not None else self.AIM_OPERATION
        self._lock = threading.RLock()  # Use RLock to allow re-entry within same thread


        # Create serial communication module and data parser
        self.serial_comm = SerialComm(lock=self._lock, port=port, baudrate=baudrate, debug_mode=debug_mode)
        
        # ==================== 关节-电机方向映射表 ====================
        # 遵循【右手定则】：
        #   - 大拇指指向电机输出轴方向
        #   - 四指弯曲方向（正视输出轴时的逆时针方向）为：
        #     * 位置角度增加的正方向
        #     * 正速度的旋转方向  
        #     * 正扭矩的输出方向
        #

        self.joint_to_servo_map = [
            (0, 1.0),   # Joint 1: 正向安装
            (1, 1.0),  # Joint 2: 反向安装 (电机物理方向与URDF相反)
            (2, 1.0),  # Joint 3: 反向安装
            (3, 1.0),  # Joint 4: 反向安装
            (4, 1.0),   # Joint 5: 正向安装
            (5, 1.0),   # Joint 6: 正向安装
        ]
        
        # Extract directions for DataParser
        joint_directions = [mapping[1] for mapping in self.joint_to_servo_map]
        self.data_parser = DataParser(lock=self._lock, debug_mode=debug_mode, joint_directions=joint_directions)
        
        # Number of servos
        self.servo_count = 6
        self.joint_count = 6
        
        # State update thread related
        self._update_thread = None
        self.thread_update_interval = 0.010  # Update interval in seconds (10ms = 100Hz max query rate)
        self._stop_thread = threading.Event()
        self._pause_update = threading.Event()  # 用于暂停后台线程
        self._thread_running = False
        self._last_user_command_time = time.perf_counter()  # Initialize with current time (not 0.0)
        
        # MIT模式初始化状态标志
        # 默认假设用户已手动运行 00_config_mit_params.py 配置过MIT参数
        # 如果设置了 auto_init_mit=True，则在首次使用时自动配置（会使电机回零）
        self._mit_mode_initialized = not auto_init_mit  # auto_init_mit=False时默认为True（假设已配置）
        
        self.disconnect()

    def set_speed(self, speed_deg_s: float) -> bool:
        """Set robot motion speed (placeholder for now).
        
        :param speed_deg_s: Speed in degrees per second
        :return: True
        """
        # Currently just a placeholder or store it if needed
        # self.current_speed = speed_deg_s
        return True
    
    def wait_for_valid_state(self, timeout: float = 1.5) -> bool:
        """
        Wait for robot arm state to become valid

        :param timeout: Maximum waiting time (seconds)
        :return: Whether a valid state was received within timeout
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            js = self.data_parser.get_joint_state()
            if js and max(abs(a) for a in js.angles) > 1e-3:
                return True
            time.sleep(0.05)
        # print(f"[Timeout] No valid joint state received")
        return False

    def __del__(self):
        """
        析构函数不应抛异常（Python 在垃圾回收时会静默处理），
        因此这里捕获并吞掉所有异常；如果有运行线程，尝试优雅停止/join。
        """
        try:
            thread = getattr(self, "_update_thread", None)
            if thread is not None:
                try:
                    # 如果线程支持停止或 join，请根据实现调用
                    if hasattr(thread, "join"):
                        thread.join(timeout=0.1)
                except Exception:
                    # 不要在析构中抛出
                    pass
        except Exception:
            pass
    
    def connect(self) -> bool:
        """
        Connect to the robot arm.
        注意：连接成功后不会自动启动后台更新线程，需要手动调用 start_update_thread()。
        这样可以确保在启动后台线程之前先完成固件版本查询等初始化操作。
        
        :return: Whether connection is successful
        """
        result = self.serial_comm.connect()
        # 不再自动启动后台线程，由上层 API 在查询固件版本后手动启动
        return result
    
    def auto_detect_control_aim(self, timeout: float = 1.0) -> int:
        """
        自动检测机械臂类型(示教臂或操作臂)
        
        尝试使用当前的 default_control_aim 查询关节数据。
        如果收到的是全0数据(无效),则自动切换到另一种模式重试。
        
        :param timeout: 每次尝试的超时时间(秒)
        :return: 检测到的有效 control_aim (AIM_TEACH 或 AIM_OPERATION)
        """
        if self.debug_mode:
            logger.info(f"[Auto-Detect] Starting arm type detection (current: 0x{self.default_control_aim:02X})")
        
        # 先尝试当前设置的 control_aim
        aims_to_try = [self.default_control_aim]
        
        # 添加备选项
        if self.default_control_aim == self.AIM_OPERATION:
            aims_to_try.append(self.AIM_TEACH)
        else:
            aims_to_try.append(self.AIM_OPERATION)
        
        for aim in aims_to_try:
            if self.debug_mode:
                aim_name = "Teaching Arm" if aim == self.AIM_TEACH else "Operating Arm"
                logger.info(f"[Auto-Detect] Testing control_aim=0x{aim:02X} ({aim_name})")
            
            # 发送查询指令
            success = self.acquire_info("joint", wait=True, timeout=timeout, control_aim=aim)
            
            if not success:
                if self.debug_mode:
                    logger.warning(f"[Auto-Detect] Failed to query with control_aim=0x{aim:02X}")
                continue
            
            # 检查返回的数据是否有效
            joint_state = self.data_parser.get_joint_state()
            if joint_state and joint_state.angles:
                # 检查是否为全0数据(无效)或有效数据
                max_angle = max(abs(a) for a in joint_state.angles)
                
                # 全0数据判断: 所有角度的绝对值都小于0.1弧度(约5.7度)且非常接近0
                is_all_zero = max_angle < 0.001  # 0.001弧度 ≈ 0.057度
                
                # 异常数据判断: 如果角度接近-12.5弧度(-716.2度),也认为是无效数据
                is_invalid = any(abs(a + 12.5) < 0.1 for a in joint_state.angles)
                
                if not is_all_zero and not is_invalid:
                    # 找到有效数据
                    if self.debug_mode:
                        angles_deg = [round(a * 57.29578, 1) for a in joint_state.angles]
                        aim_name = "Teaching Arm" if aim == self.AIM_TEACH else "Operating Arm"
                        logger.info(f"[Auto-Detect] ✓ Valid data detected with control_aim=0x{aim:02X} ({aim_name})")
                        logger.info(f"[Auto-Detect]   Current angles: {angles_deg}°")
                    return aim
                else:
                    if self.debug_mode:
                        logger.warning(f"[Auto-Detect] Received invalid data (all zeros or extreme values), trying next...")
        
        # 如果都失败了,返回默认值
        logger.warning(f"[Auto-Detect] Failed to detect valid arm type, keeping default: 0x{self.default_control_aim:02X}")
        return self.default_control_aim
    
    def initialize_mit_mode(self, repeat_times: int = 2) -> bool:
        """
        初始化MIT控制模式。
        在使用任何MIT系列控制模式(PATTERN_MIT_POSITION/SPEED/TORQUE)前必须调用。
        
        :param repeat_times: 重复发送初始化指令的次数(默认2次,确保固件接收)
        :return: 是否成功初始化
        """
        if self._mit_mode_initialized:
            if self.debug_mode:
                logger.debug("MIT mode already initialized, skipping")
            return True
        
        if not self.serial_comm.serial_port or not self.serial_comm.serial_port.is_open:
            logger.error("Serial port not connected, cannot initialize MIT mode")
            return False
        
        try:
            if self.debug_mode:
                logger.info(f"Initializing MIT control mode (sending {repeat_times} times)...")
            
            for i in range(repeat_times):
                self.serial_comm.serial_port.write(self.MIT_INIT_COMMAND)
                if self.debug_mode:
                    logger.debug(f"  MIT init command sent ({i+1}/{repeat_times})")
                time.sleep(0.02)  # 20ms间隔
            
            # 等待固件处理完成
            time.sleep(0.1)
            
            self._mit_mode_initialized = True
            if self.debug_mode:
                logger.info("✓ MIT mode initialization complete")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize MIT mode: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the robot arm"""
        # Stop state update thread first
        self.stop_update_thread()
        self.serial_comm.disconnect()
        # 注意：断开连接时不重置 _mit_mode_initialized 标志
        # 因为MIT参数配置保存在STM32内存中，只要不断电就一直有效
        # self._mit_mode_initialized = False  # 注释掉，避免重复初始化
    
    def start_update_thread(self):
        """Start state update thread"""
        if self._update_thread is not None and self._thread_running:
            logger.info("State update thread is already running")
            return
        
        # Reset stop flag
        self._stop_thread.clear()
        self._thread_running = True
        
        # Create and start thread
        self._update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()
    
    def stop_update_thread(self):
        """Stop state update thread"""
        if self._update_thread is None or not self._thread_running:
            return
        
        # Set stop flag
        self._stop_thread.set()
        self._thread_running = False
        
        # Wait for thread to finish
        if self._update_thread.is_alive():
            self._update_thread.join(timeout=2.0)
        
        self._update_thread = None
    
    def is_update_thread_running(self) -> bool:
        """
        Check whether state update thread is running
        """
        return self._thread_running and self._update_thread is not None and self._update_thread.is_alive()
    
    def get_update_thread_status(self) -> Dict:
        """
        Get detailed status of the state update thread
        
        Returns:
            Dict: Dictionary containing thread status
        """
        return {
            "running": self.is_update_thread_running(),
            "enabled": self._thread_running,
            "thread_exists": self._update_thread is not None,
            "thread_alive": self._update_thread.is_alive() if self._update_thread else False,
            "stop_flag_set": self._stop_thread.is_set()
        }
    
    def _update_loop(self):
        """Main loop of state update thread (active query mode with smart coordination)
        
        Note: Alicia-M uses active query protocol - robot only responds to requests.
        Background thread periodically sends query commands to update joint state,
        but intelligently backs off when user is sending high-frequency control commands.
        """
        
        while not self._stop_thread.is_set():
            try:
                # Smart coordination: Check if user is sending commands frequently
                time_since_last_user_cmd = time.perf_counter() - self._last_user_command_time
                
                # If user sent command very recently (< 50ms), skip this query to avoid collision
                if time_since_last_user_cmd < 0.05:
                    time.sleep(self.thread_update_interval)
                    continue
                
                # Send joint query command
                query_cmd = self._build_joint_request_frame(control_aim=self.default_control_aim)
                
                # Send command with lock protection
                with self._lock:
                    success = self.serial_comm.send_data(query_cmd)
                
                if not success:
                    if self.debug_mode:
                        logger.warning("Failed to send joint query command")
                    time.sleep(self.thread_update_interval)
                    continue
                
                # Read response frames (with timeout)
                frames_read = 0
                max_frames_per_iteration = 3  # Limit frames to read per query
                timeout_start = time.perf_counter()
                read_timeout = 0.05  # 50ms timeout for reading response
                
                while frames_read < max_frames_per_iteration and not self._stop_thread.is_set():
                    # Check timeout
                    if time.perf_counter() - timeout_start > read_timeout:
                        break
                    
                    with self._lock:
                        frame = self.serial_comm.read_frame()
                    
                    if frame is None:
                        # No more frames available, small delay and retry
                        time.sleep(0.001)
                        continue
                    
                    # Handle severe communication error
                    if frame == 9999999:
                        try:
                            logger.error("Severe serial communication error detected, robot arm may be disconnected")
                        except Exception:
                            pass
                        break
                    
                    if frame:
                        self.data_parser.parse_frame(frame)
                        frames_read += 1
                        break  # Got response, move to next query
                
                # Sleep to control query frequency (avoid flooding)
                time.sleep(self.thread_update_interval)
                    
            except Exception as e:
                # Log error but continue loop for resilience
                try:
                    logger.error(f"State update thread exception: {str(e)}")
                except Exception:
                    pass
                # Sleep briefly before retrying to avoid rapid error loops
                time.sleep(self.thread_update_interval)
        
        self._thread_running = False


    #信息获取接口
    def acquire_info(self, info_type: str, wait: bool = False, timeout: float = 2.0, retry_interval: float = 0.2, control_aim: int = None) -> bool:
        """
        General information acquisition interface, selecting different commands by type.

        :param info_type: Type of information to acquire (version, zero_cali, torque_on, torque_off, joint, etc.)
        :param wait: If True, wait for the response to be received and parsed
        :param timeout: Maximum time to wait in seconds (only used if wait=True)
        :param retry_interval: Time interval between retry attempts in seconds (default 0.2s)
        :param control_aim: 控制目标 (AIM_TEACH=0x01, AIM_OPERATION=0x02)。
                        默认使用实例的 default_control_aim。
                        注意：只有 "joint" 和 "joint_gripper" 指令需要 control_aim，其他指令为通用指令。
        :return: True if successful
        """
        # Map joint_gripper to joint for hardware command
        actual_info_type = "joint" if info_type == "joint_gripper" else info_type
        
        if actual_info_type not in self.INFO_COMMAND_MAP and actual_info_type != "joint":
            raise ValueError(f"Unsupported info type: {info_type}")

        # Clear the corresponding event before sending request (if applicable)
        if info_type in self.data_parser._info_event_map:
            event = self.data_parser._info_event_map[info_type]
            event.clear()

        # 动态构建 joint 指令，其他指令使用预定义命令
        if actual_info_type == "joint":
            # 使用提供的 control_aim 或默认值
            aim = control_aim if control_aim is not None else self.default_control_aim
            command = self._build_joint_request_frame(control_aim=aim)
        else:
            command = self.INFO_COMMAND_MAP[actual_info_type]
        
        # If not waiting, just send once
        if not wait:
            success = self.serial_comm.send_data(command)
            return success

        # If waiting and has an event, implement retry logic
        if info_type in self.data_parser._info_event_map:
            event = self.data_parser._info_event_map[info_type]
            start_time = time.time()

            while time.time() - start_time < timeout:
                # Send command
                success = self.serial_comm.send_data(command)
                if not success:
                    logger.warning(f"Failed to send {info_type} command, retrying...")
                    time.sleep(retry_interval)
                    continue

                # Wait for response with a short timeout (retry_interval)
                remaining_time = timeout - (time.time() - start_time)
                wait_time = min(retry_interval, remaining_time)

                if event.wait(wait_time):
                    # Successfully received response
                    return True

            # Timeout exceeded
            logger.warning(f"Failed to get {info_type} within timeout period after multiple retries")
            return False
        else:
            # For commands without events, just send once
            success = self.serial_comm.send_data(command)
            return success
            
    def _build_joint_request_frame(self, control_aim: int = None) -> List[int]:
        """
        构建请求关节信息的指令帧
        
        Args:
            control_aim: 控制目标 (AIM_TEACH=0x01, AIM_OPERATION=0x02)
                        默认使用实例的 default_control_aim
        
        Returns:
            List[int]: 请求关节信息的指令帧
            
        Frame Structure:
            [0xAA] [0x06] [func_id] [0x02] [0x00] [0x01] [checksum] [0xFF]
            其中 func_id = control_aim (请求模式，高位为0)
        """
        if control_aim is None:
            control_aim = self.default_control_aim
        
        # 构建请求帧
        # frame[2] = control_aim (请求模式: 0x0x)
        # frame[3] = 0x02 (数据长度)
        # frame[4] = 0x00 (数据类型：位置数据)
        # frame[5] = 0x01 (偏移数量)
        frame = [0xAA, 0x06, control_aim, 0x02, 0x00, 0x01, 0x00, 0xFF]
        
        # 计算校验位 (frame[1] 到 frame[-2] 之间的数据)
        frame[-2] = self.serial_comm.calculate_checksum(frame[1:-2])
        
        if self.debug_mode:
            logger.info(f"[_build_joint_request_frame] Built request with control_aim=0x{control_aim:02X}: {' '.join(f'{b:02X}' for b in frame)}")
        
        return frame

    def set_gripper(self, value: float, control_aim: int = None, control_mode: tuple = None) -> bool:
        """
        Set gripper value (0-100).
        Wrapper for set_joint_and_gripper to maintain compatibility.
        """
        return self.set_joint_and_gripper(gripper_value=value, control_aim=control_aim, control_mode=control_mode)

    def set_joint_and_gripper(self, 
                              joint_angles: Optional[List[float]] = None,
                              gripper_value: Optional[float] = None,
                              speed_deg_s: Union[float, List[float], np.ndarray] = 57.3,
                              torque_nm: Union[float, List[float], np.ndarray] = 0.0,
                              gripper_speed_deg_s: Optional[float] = None,
                              control_aim: int = None,
                              control_mode: tuple = None) -> bool:
        """
        统一的关节和夹爪目标设置接口 - 支持所有控制模式
        
        Args:
            joint_angles: 目标关节角度列表 (弧度). None表示不控制关节
            gripper_value: 夹爪目标值 (0-100). None表示不控制夹爪
            speed_deg_s: 关节速度 (度/秒). 范围 [-573, +573] deg/s (对应 [-10, +10] rad/s).
                        可以是单个值(所有关节相同)或列表/数组(每关节独立，长度为6)
            torque_nm: 关节扭矩 (牛米). 范围 [-10.0, +10.0] Nm.
                      可以是单个值(所有关节相同)或列表/数组(每关节独立，长度为6)
            gripper_speed_deg_s: 夹爪速度 (度/秒). 范围 [-573, +573] deg/s. None使用默认值57.3 deg/s
            control_aim: 控制目标 (0x01=示教臂, 0x02=操作臂). None使用实例默认值
            control_mode: 控制模式元组. None使用实例默认值
                - PATTERN_PV:  必须提供 joint_angles + speed_deg_s
                - PATTERN_PVT: 必须提供 joint_angles + speed_deg_s + torque_nm
                - PATTERN_V:   必须提供 speed_deg_s
                - PATTERN_MIT: 可选提供 joint_angles / speed_deg_s / torque_nm 中的任意一个或多个
        """
        # 默认值
        if control_aim is None:
            control_aim = self.default_control_aim
        if control_mode is None:
            control_mode = self.PATTERN_PV
        
        # 检查是否使用MIT系列模式,如果是且未初始化,则先初始化
        is_mit_mode = control_mode in (
            self.PATTERN_MIT,
            self.PATTERN_MIT_POSITION,
            self.PATTERN_MIT_SPEED,
            self.PATTERN_MIT_TORQUE
        )
        
        # print(f"[DEBUG] set_joint_and_gripper called: is_mit_mode={is_mit_mode}, control_mode={control_mode}, _mit_mode_initialized={self._mit_mode_initialized}")
        
        if is_mit_mode and not self._mit_mode_initialized:
            # 只有当 auto_init_mit=True 时才自动初始化
            if self.auto_init_mit:
                # print("[DEBUG] MIT mode not initialized, initializing...")
                if self.debug_mode:
                    logger.info("Detected MIT mode usage, initializing MIT mode first...")
                if not self.initialize_mit_mode():
                    logger.error("Failed to initialize MIT mode, cannot proceed")
                    # print("[DEBUG] MIT mode initialization FAILED!")
                    return False
                # print("[DEBUG] MIT mode initialization SUCCESS!")
            else:
                # 提示用户需要手动配置MIT参数
                logger.warning("=" * 60)
                logger.warning("MIT mode requires parameter configuration first!")
                logger.warning("Please run: python examples/00_config_mit_params.py")
                logger.warning("")
                logger.warning("Or set auto_init_mit=True to auto-configure (will reset motors).")
                logger.warning("See examples/README_MIT_CONFIG.md for details.")
                logger.warning("=" * 60)
                return False

        # Basic validation for single float speed
        if isinstance(speed_deg_s, (int, float)):
            # Relaxed check to allow negative values for the new mapping [-573, 573]
            pass
        
        # print(f"[DEBUG] Building control frame with joint_angles={joint_angles}, gripper={gripper_value}, control_aim=0x{control_aim:02X}, control_mode={control_mode}")
        
        frame = self._build_send_joint_frame(
            joint_angles=joint_angles,
            gripper_value=gripper_value,
            speed_deg_s=speed_deg_s,
            torque_nm=torque_nm,
            gripper_speed_deg_s=gripper_speed_deg_s,
            control_aim=control_aim,
            control_mode=control_mode
        )

        # 仅在调试模式下打印发送的控制数据包（避免高频控制时的性能开销）
        if self.debug_mode:
            print(f"[TX] 发送控制数据包: {' '.join(f'{b:02X}' for b in frame)}")

        # Record user command time (for background thread coordination)
        self._last_user_command_time = time.perf_counter()
        
        # Send control command (background thread will handle any response)
        success = self.serial_comm.send_data(frame)
        
        if self.debug_mode and not success:
            logger.warning("Failed to send joint control command")
        
        return success
    
    
    def _build_send_joint_frame(self, 
                           joint_angles: Optional[List[float]] = None, 
                           gripper_value: Optional[float] = None, 
                                speed_deg_s: Union[float, List[float], np.ndarray] = 57.3,
                                torque_nm: Union[float, List[float], np.ndarray] = 0.0,
                                gripper_speed_deg_s: Optional[float] = None,
                           control_aim: int = None,
                           control_mode: tuple = None) -> List[int]:
        """
        构建关节+夹爪+速度控制帧 (支持多种控制模式)
        
        协议格式:
        [帧头 0xAA] [指令码 0x06] [功能码] [数据长度] [前缀1] [前缀2] [数据...] [校验位] [帧尾 0xFF]
        
        Args:
            joint_angles: 目标关节角度列表 (弧度). None表示不控制关节
            gripper_value: 夹爪目标值 (0-100). None表示不控制夹爪
            speed_deg_s: 关节速度 (度/秒). 范围 [-573, +573] deg/s (对应 [-10, +10] rad/s).
                        可以是单个值(所有关节相同)或列表(每关节独立)
            torque_nm: 关节扭矩 (牛米). 范围 [-10.0, +10.0] Nm.
                      可以是单个值(所有关节相同)或列表(每关节独立)
                      仅在PVT模式使用
            control_aim: 控制目标 (AIM_TEACH=0x01, AIM_OPERATION=0x02, AIM_HEAD=0x04, 
                         AIM_LOIN=0x08, AIM_LEG=0x10, AIM_UNDERPAN=0x20)
                         默认使用实例的 default_control_aim (默认为 AIM_OPERATION)
            control_mode: 控制模式元组 (PATTERN_PV, PATTERN_PVT, PATTERN_V, PATTERN_MIT)
                          格式: (模式类型, 每电机字节数/2)

        Returns:
            List[int]: 构建好的数据帧
        """
        # 默认值
        if control_aim is None:
            control_aim = self.default_control_aim
        if control_mode is None:
            control_mode = self.PATTERN_PV
        
        # 解析控制模式
        mode_type, bytes_per_motor_half = control_mode
        bytes_per_motor = bytes_per_motor_half * 2  # 每电机实际字节数
        
        # 计算数据长度
        # 6个关节 * 每电机字节数 + 1个夹爪 * 每电机字节数
        MOTOR_DATA_LEN = 7 * bytes_per_motor  # 7个电机(6关节+1夹爪)
        PREFIX_LEN = 2  # 前缀2字节
        DATA_LENGTH = PREFIX_LEN + MOTOR_DATA_LEN
        
        # 帧大小: 帧头 + 指令码 + 功能码 + 数据长度 + 数据 + 校验位 + 帧尾
        FRAME_SIZE = 1 + 1 + 1 + 1 + DATA_LENGTH + 1 + 1
        
        # Create frame
        frame = [0] * FRAME_SIZE
        frame[0] = self.FRAME_HEADER  # 0xAA
        frame[1] = self.CMD_JOINT     # 0x06
        
        # 功能码 = 0x80 + control_aim (写入模式)
        frame[2] = 0x80 | control_aim
        
        # 数据长度
        frame[3] = DATA_LENGTH
        
        # 前缀: [数据类型标志 + 模式类型, 每电机字节数/2]
        # 写入模式: 数据类型标志 = 0x00
        frame[4] = self.DATA_FLAG_WRITE | mode_type
        frame[5] = bytes_per_motor_half
        
        frame[-1] = self.FRAME_FOOTER  # 0xFF
        
        data_start = 6

        # Get current state for optional values
        current_state = self.data_parser.get_joint_state()

        if joint_angles is None:
            if current_state and current_state.angles:
                effective_joints = current_state.angles
            else:
                # Default to zero if no current state available
                effective_joints = [0.0] * self.joint_count
        else:
            if len(joint_angles) != self.joint_count:
                logger.error(f"Incorrect joint count: need {self.joint_count}, got {len(joint_angles)}")
                # Fall back to current or zeros to avoid crashing
                if current_state and current_state.angles:
                    effective_joints = current_state.angles
                else:
                    effective_joints = [0.0] * self.joint_count
            else:
                effective_joints = joint_angles

        # Handle speed (deg/s)
        # If gripper_speed_deg_s is provided, use it; otherwise use default or extract from speed_deg_s list
        if gripper_speed_deg_s is not None:
            gripper_speed_val = gripper_speed_deg_s
        else:
            gripper_speed_val = 57.3  # 默认夹爪速度 (deg/s, 约1.0 rad/s)

        if isinstance(speed_deg_s, (list, tuple, np.ndarray)) or (hasattr(speed_deg_s, '__len__') and not isinstance(speed_deg_s, str)):
            # Convert to list if it's a numpy array or other sequence
            try:
                speed_array = list(speed_deg_s)
            except TypeError:
                speed_array = [speed_deg_s]
            
            # If gripper_speed_deg_s is not provided and speed_array has 7 elements, use the 7th for gripper
            if gripper_speed_deg_s is None and len(speed_array) == 7:
                speed_list = speed_array[:6]
                gripper_speed_val = speed_array[6]
            elif len(speed_array) != self.joint_count:
                logger.warning(f"Speed list length {len(speed_array)} != joint count {self.joint_count}, using first value or default")
                speed_list = [speed_array[0]] * self.joint_count if speed_array else [57.3] * self.joint_count
            else:
                speed_list = speed_array
        else:
            speed_list = [speed_deg_s] * self.joint_count
            # Only use speed_deg_s for gripper if gripper_speed_deg_s is not provided
            if gripper_speed_deg_s is None:
                gripper_speed_val = speed_deg_s

        # Handle torque (N·m) - only used in PVT mode
        gripper_torque_val = 0.0  # 默认夹爪扭矩 (N·m)
        if isinstance(torque_nm, (list, tuple)) or hasattr(torque_nm, '__len__'):
            # Convert to list if it's a numpy array or other sequence
            try:
                torque_array = list(torque_nm)
            except TypeError:
                torque_array = [torque_nm]
            
            if len(torque_array) == 7:
                torque_list = torque_array[:6]
                gripper_torque_val = torque_array[6]
            elif len(torque_array) != self.joint_count:
                logger.warning(f"Torque list length {len(torque_array)} != joint count {self.joint_count}, using first value or default")
                torque_list = [torque_array[0]] * self.joint_count if torque_array else [0.0] * self.joint_count
            else:
                torque_list = torque_array
        else:
            torque_list = [torque_nm] * self.joint_count
            gripper_torque_val = torque_nm

        # 根据不同模式填充数据
        for joint_idx in range(6):
            angle_rad = effective_joints[joint_idx]
            # Get direction from mapping - 用于位置、速度、扭矩的方向校正
            # 遵循右手定则：大拇指指向电机输出轴，四指弯曲方向为正方向
            direction = self.joint_to_servo_map[joint_idx][1]
            hardware_value = self._rad_to_hardware_value(angle_rad, direction)
            
            offset = data_start + joint_idx * bytes_per_motor
            
            if control_mode == self.PATTERN_PV:
                # PV模式: 位置(2字节) + 速度(2字节) = 4字节
                speed_val = speed_list[joint_idx]
                speed_hw_value = self._value_to_hardware_value_speed(speed_val, direction)
                frame[offset] = hardware_value & 0xFF
                frame[offset + 1] = (hardware_value >> 8) & 0xFF
                frame[offset + 2] = speed_hw_value & 0xFF
                frame[offset + 3] = (speed_hw_value >> 8) & 0xFF
                
            elif control_mode == self.PATTERN_PVT:
                # PVT模式: 位置(2字节) + 速度(2字节) + 扭矩(2字节) = 6字节
                speed_val = speed_list[joint_idx]
                speed_hw_value = self._value_to_hardware_value_speed(speed_val, direction)
                torque_val = torque_list[joint_idx]
                torque_hw_value = self._value_to_hardware_value_torque(torque_val, direction)
                frame[offset] = hardware_value & 0xFF
                frame[offset + 1] = (hardware_value >> 8) & 0xFF
                frame[offset + 2] = speed_hw_value & 0xFF
                frame[offset + 3] = (speed_hw_value >> 8) & 0xFF
                frame[offset + 4] = torque_hw_value & 0xFF
                frame[offset + 5] = (torque_hw_value >> 8) & 0xFF
                
            elif control_mode == self.PATTERN_V:
                # V模式: 速度(2字节) = 2字节
                speed_val = speed_list[joint_idx]
                speed_hw_value = self._value_to_hardware_value_speed(speed_val, direction)
                frame[offset] = speed_hw_value & 0xFF
                frame[offset + 1] = (speed_hw_value >> 8) & 0xFF
                
            elif control_mode in (self.PATTERN_MIT, self.PATTERN_MIT_TORQUE):
                # MIT扭矩模式: 仅扭矩(2字节) = 2字节
                torque_val = torque_list[joint_idx]
                torque_hw_value = self._value_to_hardware_value_torque(torque_val, direction)
                frame[offset] = torque_hw_value & 0xFF
                frame[offset + 1] = (torque_hw_value >> 8) & 0xFF
                
            elif control_mode == self.PATTERN_MIT_POSITION:
                # MIT位置模式: 仅位置(2字节) = 2字节
                frame[offset] = hardware_value & 0xFF
                frame[offset + 1] = (hardware_value >> 8) & 0xFF
                
            elif control_mode == self.PATTERN_MIT_SPEED:
                # MIT速度模式: 仅速度(2字节) = 2字节
                speed_val = speed_list[joint_idx]
                speed_hw_value = self._value_to_hardware_value_speed(speed_val, direction)
                frame[offset] = speed_hw_value & 0xFF
                frame[offset + 1] = (speed_hw_value >> 8) & 0xFF
        
        # Gripper data (夹爪不需要方向校正，始终使用默认方向)
        gripper_offset = data_start + 6 * bytes_per_motor
        if gripper_value is not None:
            gripper_hw_value = self._value_to_hardware_value_grip(gripper_value, type=self.gripper_type)
        else:
            if current_state and current_state.gripper is not None:
                gripper_hw_value = self._value_to_hardware_value_grip(current_state.gripper, type=self.gripper_type)
            else:
                gripper_hw_value = 2048  # Default middle position

        if control_mode == self.PATTERN_PV:
            gripper_speed_hw_value = self._value_to_hardware_value_speed(gripper_speed_val)
            frame[gripper_offset] = gripper_hw_value & 0xFF
            frame[gripper_offset + 1] = (gripper_hw_value >> 8) & 0xFF
            frame[gripper_offset + 2] = gripper_speed_hw_value & 0xFF
            frame[gripper_offset + 3] = (gripper_speed_hw_value >> 8) & 0xFF
            
        elif control_mode == self.PATTERN_PVT:
            gripper_speed_hw_value = self._value_to_hardware_value_speed(gripper_speed_val)
            gripper_torque_hw_value = self._value_to_hardware_value_torque(gripper_torque_val)
            frame[gripper_offset] = gripper_hw_value & 0xFF
            frame[gripper_offset + 1] = (gripper_hw_value >> 8) & 0xFF
            frame[gripper_offset + 2] = gripper_speed_hw_value & 0xFF
            frame[gripper_offset + 3] = (gripper_speed_hw_value >> 8) & 0xFF
            frame[gripper_offset + 4] = gripper_torque_hw_value & 0xFF
            frame[gripper_offset + 5] = (gripper_torque_hw_value >> 8) & 0xFF
            
        elif control_mode == self.PATTERN_V:
            gripper_speed_hw_value = self._value_to_hardware_value_speed(gripper_speed_val)
            frame[gripper_offset] = gripper_speed_hw_value & 0xFF
            frame[gripper_offset + 1] = (gripper_speed_hw_value >> 8) & 0xFF
            
        elif control_mode in (self.PATTERN_MIT, self.PATTERN_MIT_TORQUE):
            # MIT扭矩模式: 仅扭矩(2字节) = 2字节
            gripper_torque_hw_value = self._value_to_hardware_value_torque(gripper_torque_val)
            frame[gripper_offset] = gripper_torque_hw_value & 0xFF
            frame[gripper_offset + 1] = (gripper_torque_hw_value >> 8) & 0xFF
            
        elif control_mode == self.PATTERN_MIT_POSITION:
            # MIT位置模式: 仅位置(2字节) = 2字节
            frame[gripper_offset] = gripper_hw_value & 0xFF
            frame[gripper_offset + 1] = (gripper_hw_value >> 8) & 0xFF
            
        elif control_mode == self.PATTERN_MIT_SPEED:
            # MIT速度模式: 仅速度(2字节) = 2字节
            gripper_speed_hw_value = self._value_to_hardware_value_speed(gripper_speed_val)
            frame[gripper_offset] = gripper_speed_hw_value & 0xFF
            frame[gripper_offset + 1] = (gripper_speed_hw_value >> 8) & 0xFF

        # 计算校验位
        frame[-2] = self.serial_comm.calculate_checksum(frame[1:-2])
        
        if self.debug_mode:
            # 详细调试输出
            if joint_angles is not None:
                angle_deg = [round(angle * self.RAD_TO_DEG, 2) for angle in joint_angles]
                logger.debug(f"Send frame - aim: 0x{control_aim:02X}, mode: {control_mode}, joints (deg): {angle_deg}, gripper: {gripper_value}")
            
            # 打印完整数据包（十六进制）
            hex_str = ' '.join([f'{b:02X}' for b in frame])
            logger.debug(f"TX Frame ({len(frame)} bytes): {hex_str}")
            
            # 打印扭矩数据详情
            if torque_nm is not None:
                torque_info = []
                for i in range(6):
                    direction = self.joint_to_servo_map[i][1]
                    torque_val = torque_list[i] if i < len(torque_list) else 0.0
                    hw_val = self._value_to_hardware_value_torque(torque_val, direction)
                    torque_info.append(f"J{i+1}:{torque_val:+.3f}Nm->0x{hw_val:03X}")
                logger.debug(f"Torque mapping: {', '.join(torque_info)}")
            
        return frame

    # 保留旧函数名作为兼容别名
    def _build_joint_frame(self, 
                           joint_angles: Optional[List[float]] = None, 
                           gripper_value: Optional[float] = None, 
                           speed_deg_s: Union[float, List[float]] = 57.3) -> List[int]:
        """
        [已废弃] 兼容旧版本的函数别名，请使用 _build_send_joint_frame
        
        注意: 速度参数现在使用度/秒 (deg/s)，范围 [-573, +573]
        """
        return self._build_send_joint_frame(
            joint_angles=joint_angles,
            gripper_value=gripper_value,
            speed_deg_s=speed_deg_s,
            torque_nm=0.0,
            control_aim=self.AIM_TEACH,
            control_mode=self.PATTERN_PV
        )

    
     #----------------------------------------------start-位置数据转换---------------------------------------
    
    def _rad_to_hardware_value(self, angle_rad: float, direction: float = 1.0) -> int:
        """
        Convert angle to hardware value using configurable range and bit width.
        Default mapping: [-12.5, +12.5] -> 16-bit [0, 65535]
        Args:
            angle_rad: Angle value (radians). If your physical range is
                        [-12.5, +12.5] radians, this will be linearly mapped
                        to 16-bit unsigned integer.
            direction: Motor direction coefficient (1.0 or -1.0)
        """
        # Apply direction mapping
        angle_rad = angle_rad * direction

        # Configurable range and bit width
        x_min = -12.5
        x_max = 12.5
        bits = 16

        # Clip to range
        if angle_rad < x_min or angle_rad > x_max:
            logger.warning(f"Angle out of range: {angle_rad:.4f}, will be clipped to [{x_min}, {x_max}]")
            angle_rad = max(x_min, min(x_max, angle_rad))

        return self._float_to_uint(angle_rad, x_min, x_max, bits)

    @staticmethod
    def _float_to_uint(x_float: float, x_min: float, x_max: float, bits: int) -> int:
        """Generic float -> unsigned int linear mapping.
        Maps [x_min, x_max] to [0, 2^bits - 1].
        """
        span = x_max - x_min
        if span <= 0:
            # Degenerate range, return zero
            return 0
        max_val = (1 << bits) - 1
        value = int((x_float - x_min) * (float(max_val)) / span)
        return max(0, min(max_val, value))

    @staticmethod
    def _uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
        """Generic unsigned int -> float linear mapping.
        Maps [0, 2^bits - 1] back to [x_min, x_max].
        """
        if x_max < x_min:
            x_max = x_min
        max_val = (1 << bits) - 1
        if x_int < 0:
            x_int = 0
        elif x_int > max_val:
            x_int = max_val
        span = x_max - x_min
        offset = x_min
        return float(x_int) * span / float(max_val) + offset
    

    #----------------------------------------------end-位置数据转换---------------------------------------
    

    #---------------------------------------------start-夹爪指令的转换----------------------------------
    
    def _value_to_hardware_value_grip(self, value: float, type: str="100mm") -> int:
        """
        :param value: Gripper value in 0-100
        :return: Hardware value
        """
        # Range check
        if value < 0:
            logger.warning(f"Gripper angle below range: {value:.2f}, will be clipped to 0")
            value = 0
        elif value > 100.0:
            logger.warning(f"Gripper angle above range: {value:.2f}, will be clipped to 100")
            value = 100.0

        # Mapping: 0% (Closed) -> GRI_VAL_CLOSE, 100% (Open) -> GRI_VAL_OPEN
        # 0% -> 32768 (0 rad)
        # 100% -> 39688 (2.64 rad)
        
        open_val = self.GRI_VAL_OPEN
        close_val = self.GRI_VAL_CLOSE
        
        # Calculate hardware value
        # Formula: hw = close_val - (value/100.0) * (close_val - open_val)
        ratio = (close_val - open_val) / 100.0
        hw_value = int(close_val - (value * ratio))
        
        # 范围限制
        min_val = min(open_val, close_val)
        max_val = max(open_val, close_val)
        return max(min_val, min(max_val, hw_value))
    
    #---------------------------------------------end-夹爪指令的转换----------------------------------

    #--------------------------------------------start-速度指令的转换--------------------------------
    
    def _value_to_hardware_value_speed(self, speed_deg_s: float, direction: float = 1.0) -> int:
        """
        将速度从度/秒转换为12位硬件值 [0, 4095]
        
        转换流程:
        1. 度/秒 -> 弧度/秒 (deg * π/180)
        2. 弧度/秒映射到12位: [-10.0, +10.0] rad/s -> [0, 4095]
        
        遵循右手定则：
        - 大拇指指向电机输出轴方向
        - 四指弯曲方向（逆时针）= 正速度方向
        
        映射关系:
        - -573 deg/s (-10.0 rad/s) -> 0
        -    0 deg/s (  0.0 rad/s) -> 2048 (中间值)
        - +573 deg/s (+10.0 rad/s) -> 4095
        
        :param speed_deg_s: 目标速度值 (度/秒)，有效范围 [-573, +573] deg/s
        :param direction: 电机方向系数 (1.0 或 -1.0)，用于适配电机安装方向
        :return: 原始 12位整数速度值 (0-4095)
        """
        # 确保输入是Python float类型，避免numpy数组导致的布尔值歧义
        speed_deg_s = float(speed_deg_s)
        direction = float(direction)
        
        # 步骤1: 度/秒 -> 弧度/秒
        speed_rad_s = speed_deg_s * self.DEG_TO_RAD
        
        # 应用方向系数
        speed_rad_s = speed_rad_s * direction
        
        # 步骤2: 映射到硬件范围
        x_min = -10.0  # rad/s
        x_max = 10.0   # rad/s
        bits = 12

        # 超出范围时进行裁剪
        if speed_rad_s < x_min or speed_rad_s > x_max:
            logger.warning(f"Speed out of range: {speed_deg_s:.1f} deg/s ({speed_rad_s:.3f} rad/s), valid range: [-573, +573] deg/s ([-10, +10] rad/s), will be clipped")
            speed_rad_s = max(x_min, min(x_max, speed_rad_s))

        return self._float_to_uint(speed_rad_s, x_min, x_max, bits)
    
    #--------------------------------------------end-速度指令的转换----------------------------------
    
    #--------------------------------------------start-扭矩指令的转换--------------------------------
    
    def _value_to_hardware_value_torque(self, torque_nm: float, direction: float = 1.0) -> int:
        """
        将扭矩从 [-12.0, +12.0] N·m 线性映射到 12位 [0, 4095]。
        
        遵循右手定则：
        - 大拇指指向电机输出轴方向
        - 四指弯曲方向（逆时针）= 正扭矩方向
        
        映射关系:
        - -12.0 N·m -> 0
        -   0.0 N·m -> 2048 (中间值)
        - +12.0 N·m -> 4095
        
        :param torque_nm: 目标扭矩值 (牛·米)，有效范围 [-12.0, +12.0]
        :param direction: 电机方向系数 (1.0 或 -1.0)，用于适配电机安装方向
        :return: 原始 12位整数扭矩值 (0-4095)
        """
        # 应用方向系数
        torque_nm = torque_nm * direction
        
        x_min = -12.0
        x_max = 12.0
        bits = 12

        # 超出范围时进行裁剪
        if torque_nm < x_min or torque_nm > x_max:
            logger.warning(f"Torque out of range: {torque_nm:.3f} N·m (valid [{x_min}, {x_max}]), will be clipped")
            torque_nm = max(x_min, min(x_max, torque_nm))

        return self._float_to_uint(torque_nm, x_min, x_max, bits)
    
    #--------------------------------------------end-扭矩指令的转换----------------------------------


    











# ServoDriver 总体作用

# ServoDriver 是机械臂的控制总模块，主要负责：

# 串口通信：通过 SerialComm 与机械臂主控板交互。

# 数据解析：通过 DataParser 解码、校验主控板发回的数据。

# 关节→舵机映射：机械臂有 6 个关节，但 9 个舵机，部分舵机反向。

# 构建控制帧：将用户指令（关节角、夹爪开合、速度、加速度、力矩使能等）打包成协议帧。

# 后台线程：持续读取机械臂状态，实现实时反馈和 UI 显示。

