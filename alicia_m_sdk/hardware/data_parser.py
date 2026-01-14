import math
import time
from typing import List, Dict, Optional, NamedTuple, Union
import threading
import copy
from alicia_m_sdk.utils.logger import logger


class JointState(NamedTuple):
    """Joint state data structure."""
    angles: List[float]  # Six joint angles (radians)
    gripper: float       # Gripper value
    timestamp: float     # Timestamp (seconds)
    run_status_text: str # Run status text


class DataParser:
    """Robot arm data parsing module."""
    # Constants
    DEG_TO_RAD = math.pi / 180.0  # Degrees to radians
    RAD_TO_DEG = 180.0 / math.pi  # Radians to degrees
    
    # ==================== Command ID 定义 (frame[1]) ====================
    CMD_VERSION = 0x01     # Firmware version feedback
    CMD_ZERO_POS = 0x03    # Set current position as zero
    CMD_TORQUE = 0x05      # Torque control
    CMD_JOINT = 0x06       # Joint angle feedback and control
    CMD_ERROR = 0xEE       # Error feedback
    
    # ==================== Func ID 标志位 (frame[2]) ====================
    # 高位标志: 0x0x = 请求数据(Ask), 0x8x = 写入数据(Write)
    FUNC_MASK_ASK = 0x00   # 请求数据标志 (0x0x)
    FUNC_MASK_WRITE = 0x80 # 写入数据标志 (0x8x)
    
    # 低位设备类型 (frame[2] & 0x7F)
    DEVICE_TEACHING_ARM = 0x01   # 示教机械臂
    DEVICE_OPERATING_ARM = 0x02  # 操作机械臂
    DEVICE_HEAD = 0x04           # 机器人头部
    DEVICE_WAIST = 0x08          # 机器人腰部
    DEVICE_LOWER_LIMB = 0x10     # 机器人下肢
    DEVICE_CHASSIS = 0x20        # 机器人底盘
    
    # ==================== Data Type 标志位 (frame[4]) ====================
    # 高位标志: 0x0x = 发送数据, 0x8x = 反馈数据
    DATA_MASK_SEND = 0x00     # 发送数据标志 (0x0x)
    DATA_MASK_FEEDBACK = 0x80 # 反馈数据标志 (0x8x)
    
    # 低位数据类型 (frame[4] & 0x7F)
    DATA_TYPE_POSITION = 0x00     # 当前位置
    DATA_TYPE_SPEED = 0x01        # 当前速度
    DATA_TYPE_TORQUE = 0x02       # 当前扭矩
    DATA_TYPE_POSITION_KP = 0x03  # 位置环 kp
    DATA_TYPE_SPEED_KP = 0x04     # 速度环 kp
    
    # ==================== 运行状态位定义 (Ask 模式反馈) ====================
    # 示教臂 (0x01) 状态位定义
    TEACH_STATUS_LOCKED = 0x01              # 锁定状态
    TEACH_STATUS_SYNC = 0x02                # 同步状态
    TEACH_STATUS_RESERVED1 = 0x04           # 待定1
    TEACH_STATUS_RESERVED2 = 0x08           # 待定2
    TEACH_STATUS_RESERVED3 = 0x10           # 待定3
    TEACH_STATUS_RESERVED4 = 0x20           # 待定4
    TEACH_STATUS_GRIPPER_TORQUE_LOCK = 0x40 # 夹具过高力矩的运动方向锁定
    TEACH_STATUS_MOTOR_ERR = 0x80           # 电机 ERR 状态
    
    # 操作臂 (0x02) 状态位定义
    OPER_STATUS_SINGLE_CLICK = 0x01         # 单击
    OPER_STATUS_DOUBLE_CLICK = 0x02         # 双击
    OPER_STATUS_LONG_PRESS = 0x04           # 长按
    OPER_STATUS_REPEAT_LONG_PRESS = 0x08    # 重复长按
    OPER_STATUS_RESERVED1 = 0x10            # 待定1
    OPER_STATUS_RESERVED2 = 0x20            # 待定2
    OPER_STATUS_GRIPPER_TORQUE_LOCK = 0x40  # 夹具过高力矩的运动方向锁定
    OPER_STATUS_MOTOR_ERR = 0x80            # 电机 ERR 状态
    
    # 兼容旧代码的常量（映射到示教臂状态）
    RUN_STATUS_LOCKED = 0x01        # 锁定状态
    RUN_STATUS_SYNC = 0x02          # 同步状态
    RUN_STATUS_TORQUE_LOCKED = 0x04 # 扭矩锁定状态（旧定义，保留兼容）
    RUN_STATUS_MOTOR_ERR = 0x08     # 电机 ERR 状态（旧定义，保留兼容）
    
    # ==================== 接收状态位定义 (Write 模式反馈) ====================
    RECV_STATUS_SUCCESS = 0x01  # 接收成功
    RECV_STATUS_FAILED = 0x02   # 接收失败
    
    # ==================== 其他常量 ====================
    GRI_MAX_50MM = 3290
    GRI_MAX_100MM = 3600
    
    def __init__(self, lock: threading.Lock, debug_mode: bool = False, gripper_type: str = "50mm", joint_directions: Optional[List[float]] = None):
        """
        Initialize data parser.
        
        Args:
            lock: Shared threading lock for concurrent access.
            debug_mode: Whether to enable debug logging.
            gripper_type: Gripper type, e.g. "50mm" or "100mm".
            joint_directions: List of direction coefficients for each joint (1.0 or -1.0).
        """
        self.debug_mode = debug_mode
        self.joint_directions = joint_directions or [1.0] * 6  # Default to all normal direction
        if gripper_type == "100mm":
            self.servo_value_limit = self.GRI_MAX_100MM
        else:
            self.servo_value_limit = self.GRI_MAX_50MM 
        # Store latest joint state

        #初始化关节状态和家爪状态
        self._joint_states = JointState([0.0]*6, 0.0, 0.0, "idle")
        
        #用于存储固件版本和完整的版本信息（如序列号、硬件版本、固件版本）
        self._firmware_version: Optional[str] = None
        # Full version information dict: serial, hardware, firmware
        self._version_info: Optional[Dict[str, str]] = None

        #存储传入的线程锁，用于保护共享资源（如关节状态、版本信息等）
        self._lock = lock

        #用于存储关节数据中的运行状态和对应的文本描述
        # Store run status from joint data
        self._run_status: Optional[int] = None
        self._run_status_text: Optional[str] = None
        self._device_type: Optional[str] = None  # 存储设备类型 (teaching_arm/operating_arm)
        self._device_code: Optional[int] = None  # 存储设备类型代码 (0x01/0x02)
        
        # 状态位详细存储（示教臂）
        self._teach_status = {
            "locked": False,                # 锁定状态
            "sync": False,                  # 同步状态
            "reserved1": False,             # 待定1
            "reserved2": False,             # 待定2
            "reserved3": False,             # 待定3
            "reserved4": False,             # 待定4
            "gripper_torque_lock": False,   # 夹具过高力矩的运动方向锁定
            "motor_err": False,             # 电机 ERR 状态
        }
        
        # 状态位详细存储（操作臂）
        self._oper_status = {
            "single_click": False,          # 单击
            "double_click": False,          # 双击
            "long_press": False,            # 长按
            "repeat_long_press": False,     # 重复长按
            "reserved1": False,             # 待定1
            "reserved2": False,             # 待定2
            "gripper_torque_lock": False,   # 夹具过高力矩的运动方向锁定
            "motor_err": False,             # 电机 ERR 状态
        }
        
        #事件同步，同步固件版本信息的接收和解析，关节状态的接收和解析，手爪状态的接收和解析
        # Event-based synchronization for async data acquisition
        # Events are set when corresponding data is received and parsed
        self._version_event = threading.Event()
        self._joint_event = threading.Event()
        self._gripper_event = threading.Event()
        
        #信息类型到事件的映射
        # Mapping from info type to corresponding event
        self._info_event_map = {
            "version": self._version_event,
            "joint": self._joint_event,
            "joint_gripper": self._joint_event,  # joint_gripper uses the same event as joint
            "gripper": self._gripper_event,
        }
        
    
    def parse_frame(self, frame: List[int]) -> Optional[Dict]:
        """
        Parse a full data frame.
        
        Args:
            frame: Complete data frame (byte list)
            
        Frame Structure:
            frame[0]: Header (0xAA)
            frame[1]: Command ID (CMD)
            frame[2]: Func ID (设备类型 + 请求/写入标志)
            frame[3]: Data Length (LEN)
            frame[4]: Data Type (数据类型 + 发送/反馈标志)
            frame[5]: Offset Count (偏移数量)
            frame[6...]: Data Payload
            frame[-2]: Checksum
            frame[-1]: Footer (0xFF)
        """
        cmd_id = frame[1]
        
        if cmd_id == self.CMD_VERSION:
            return self._parse_version_data(frame)
        
        elif cmd_id == self.CMD_JOINT:
            # ==================== 根据 frame[2] 判断请求/写入 ====================
            func_id = frame[2]
            is_write = (func_id & 0x80) != 0  # 高位为 1 表示写入数据 (0x8x)
            
            if is_write:
                # 写入数据响应 -> 进入 _parse_write_joint_data
                return self._parse_write_joint_data(frame)
            else:
                # 请求数据响应 -> 进入 _parse_ask_joint_data
                return self._parse_ask_joint_data(frame)

        elif cmd_id == self.CMD_ERROR:
            return self._parse_error_data(frame)
        
        else:
            if self.debug_mode:
                logger.debug(f"Unhandled command ID: 0x{cmd_id:02X}")
            return None
    
    def get_joint_state(self) -> Optional[JointState]:
        """
        Get current joint state.
        """
        with self._lock:
            js = self._joint_states
            # print("self joint states:", js)
            if js.angles is None or js.timestamp is None:
                logger.warning("Robot state has not been updated yet")
                return None
            return copy.deepcopy(self._joint_states)
        

    
    def get_version_info(self) -> Optional[Dict[str, str]]:
        """
        Get full version information.
        
        Returns:
            Optional[Dict[str, str]]: Dictionary with keys:
                - 'serial_number'
                - 'hardware_version'
                - 'firmware_version'
            or None if not available.
        """
        with self._lock:
            if self._version_info is None:
                return None
           
            return dict(self._version_info)
    
    def get_firmware_version(self) -> Optional[str]:
        """Get firmware version string."""
        with self._lock:
            return self._firmware_version
    
    def get_device_type(self) -> Optional[str]:
        """
        获取当前设备类型
        
        Returns:
            Optional[str]: 设备类型字符串 ("teaching_arm", "operating_arm", 等)
        """
        with self._lock:
            return self._device_type
    
    def get_device_code(self) -> Optional[int]:
        """
        获取当前设备类型代码
        
        Returns:
            Optional[int]: 设备类型代码 (0x01=示教臂, 0x02=操作臂, 等)
        """
        with self._lock:
            return self._device_code
    
    def get_run_status_byte(self) -> Optional[int]:
        """
        获取原始状态字节
        
        Returns:
            Optional[int]: 状态字节 (0x00-0xFF)
        """
        with self._lock:
            return self._run_status
    
    def get_run_status_text(self) -> Optional[str]:
        """
        获取状态文本描述
        
        Returns:
            Optional[str]: 状态文本，如 "locked,sync" 或 "single_click"
        """
        with self._lock:
            return self._run_status_text
    
    def get_teach_status(self) -> Dict[str, bool]:
        """
        获取示教臂的详细状态位
        
        Returns:
            Dict[str, bool]: 包含所有状态位的字典
                {
                    "locked": bool,
                    "sync": bool,
                    "reserved1": bool,
                    "reserved2": bool,
                    "reserved3": bool,
                    "reserved4": bool,
                    "gripper_torque_lock": bool,
                    "motor_err": bool
                }
        """
        with self._lock:
            return dict(self._teach_status)
    
    def get_oper_status(self) -> Dict[str, bool]:
        """
        获取操作臂的详细状态位
        
        Returns:
            Dict[str, bool]: 包含所有状态位的字典
                {
                    "single_click": bool,
                    "double_click": bool,
                    "long_press": bool,
                    "repeat_long_press": bool,
                    "reserved1": bool,
                    "reserved2": bool,
                    "gripper_torque_lock": bool,
                    "motor_err": bool
                }
        """
        with self._lock:
            return dict(self._oper_status)
    
    def get_current_status(self) -> Optional[Dict[str, bool]]:
        """
        根据当前设备类型，获取对应的详细状态位
        
        Returns:
            Optional[Dict[str, bool]]: 当前设备的状态位字典，如果设备类型未知则返回 None
        """
        with self._lock:
            if self._device_code == self.DEVICE_TEACHING_ARM:
                return dict(self._teach_status)
            elif self._device_code == self.DEVICE_OPERATING_ARM:
                return dict(self._oper_status)
            else:
                return None
    
    def get_all_status_info(self) -> Dict:
        """
        获取完整的状态信息（用于调试和监控）
        
        Returns:
            Dict: 包含所有状态信息的字典
        """
        with self._lock:
            return {
                "device_type": self._device_type,
                "device_code": f"0x{self._device_code:02X}" if self._device_code is not None else None,
                "run_status_byte": f"0x{self._run_status:02X}" if self._run_status is not None else None,
                "run_status_text": self._run_status_text,
                "teach_status": dict(self._teach_status),
                "oper_status": dict(self._oper_status),
                "current_status": self.get_current_status(),
            }
    
    def get_info(self, info_type: str):
        """
        Unified getter for parsed information, for cooperation with high-level APIs.
        
        :param info_type: 'joint_gripper' | 'joint' | 'gripper' | 'version'
        :return: Parsed data for the given type, or None if unavailable
        """
        with self._lock:
            if info_type == "joint_gripper":
                js = self._joint_states
                if js.angles is None or js.timestamp is None:
                    return None
                return copy.deepcopy(js)
            elif info_type == "joint":
                js = self._joint_states
                if js.angles is None or js.timestamp is None:
                    return None
                return list(js.angles)
            elif info_type == "gripper":
                js = self._joint_states
                if js.angles is None or js.timestamp is None:
                    return None
                return js.gripper
            elif info_type == "version":
                return dict(self._version_info) if self._version_info else None
            else:
                raise ValueError(f"Unsupported info type: {info_type}")

    def wait_for_info(self, info_type: str, timeout: float = 2.0) -> bool:
        """
        Wait for specified info type to be received and parsed.
        
        Args:
            info_type: Type of information to wait for. Supported types:
                - "version": Wait for version info
                - "joint": Wait for joint state
                - "joint_gripper": Wait for joint and gripper state (same as "joint")
                - "gripper": Wait for gripper state
            timeout: Maximum time to wait in seconds

        """
        if info_type not in self._info_event_map:
            raise ValueError(f"Unsupported info type: {info_type}. Supported types: {list(self._info_event_map.keys())}")
        
        event = self._info_event_map[info_type]
        return event.wait(timeout)
        


    def _update_joint_state(self,
                        angles: Optional[List[float]] = None,
                        gripper: Optional[float] = None,
                        run_status_text: Optional[str] = None):
        with self._lock:
            prev = self._joint_states
            self._joint_states = JointState(
                angles=angles if angles is not None else prev.angles,
                gripper=gripper if gripper is not None else prev.gripper,
                timestamp=time.time(),
                run_status_text=run_status_text if run_status_text is not None else prev.run_status_text
            )

    # ==================== 辅助方法：解析 frame[2] 设备类型 ====================
    def _parse_device_type(self, func_id: int) -> str:
        """
        解析 frame[2] 中的设备类型（低7位）
        
        Args:
            func_id: frame[2] 的值
            
        Returns:
            设备类型字符串
        """
        device_code = func_id & 0x7F  # 取低7位
        
        device_map = {
            self.DEVICE_TEACHING_ARM: "teaching_arm",      # 0x01 示教机械臂
            self.DEVICE_OPERATING_ARM: "operating_arm",    # 0x02 操作机械臂
            self.DEVICE_HEAD: "head",                      # 0x04 机器人头部
            self.DEVICE_WAIST: "waist",                    # 0x08 机器人腰部
            self.DEVICE_LOWER_LIMB: "lower_limb",          # 0x10 机器人下肢
            self.DEVICE_CHASSIS: "chassis",                # 0x20 机器人底盘
        }
        return device_map.get(device_code, f"unknown_device_0x{device_code:02X}")
    
    # ==================== 辅助方法：解析 frame[4] 数据类型 ====================
    def _parse_data_type(self, data_type_byte: int) -> str:
        """
        解析 frame[4] 中的数据类型（低7位）
        
        Args:
            data_type_byte: frame[4] 的值
            
        Returns:
            数据类型字符串
        """
        type_code = data_type_byte & 0x7F  # 取低7位
        
        type_map = {
            self.DATA_TYPE_POSITION: "position",       # 0x00 当前位置
            self.DATA_TYPE_SPEED: "speed",             # 0x01 当前速度
            self.DATA_TYPE_TORQUE: "torque",           # 0x02 当前扭矩
            self.DATA_TYPE_POSITION_KP: "position_kp", # 0x03 位置环 kp
            self.DATA_TYPE_SPEED_KP: "speed_kp",       # 0x04 速度环 kp
        }
        return type_map.get(type_code, f"unknown_type_0x{type_code:02X}")

    # ==================== 辅助方法：解析运行状态位 ====================
    def _parse_run_status(self, status_byte: int, device_code: int) -> str:
        """
        根据设备类型解析运行状态位，并更新内部状态存储
        
        Args:
            status_byte: 状态字节（8位）
            device_code: 设备类型代码（frame[2] 的低7位）
            
        Returns:
            状态描述字符串
        """
        status_parts = []
        
        # 示教臂 (0x01) 状态解析
        if device_code == self.DEVICE_TEACHING_ARM:
            with self._lock:
                self._teach_status["locked"] = bool(status_byte & self.TEACH_STATUS_LOCKED)
                self._teach_status["sync"] = bool(status_byte & self.TEACH_STATUS_SYNC)
                self._teach_status["reserved1"] = bool(status_byte & self.TEACH_STATUS_RESERVED1)
                self._teach_status["reserved2"] = bool(status_byte & self.TEACH_STATUS_RESERVED2)
                self._teach_status["reserved3"] = bool(status_byte & self.TEACH_STATUS_RESERVED3)
                self._teach_status["reserved4"] = bool(status_byte & self.TEACH_STATUS_RESERVED4)
                self._teach_status["gripper_torque_lock"] = bool(status_byte & self.TEACH_STATUS_GRIPPER_TORQUE_LOCK)
                self._teach_status["motor_err"] = bool(status_byte & self.TEACH_STATUS_MOTOR_ERR)
            
            if status_byte & self.TEACH_STATUS_LOCKED:
                status_parts.append("locked")
            if status_byte & self.TEACH_STATUS_SYNC:
                status_parts.append("sync")
            if status_byte & self.TEACH_STATUS_RESERVED1:
                status_parts.append("reserved1")
            if status_byte & self.TEACH_STATUS_RESERVED2:
                status_parts.append("reserved2")
            if status_byte & self.TEACH_STATUS_RESERVED3:
                status_parts.append("reserved3")
            if status_byte & self.TEACH_STATUS_RESERVED4:
                status_parts.append("reserved4")
            if status_byte & self.TEACH_STATUS_GRIPPER_TORQUE_LOCK:
                status_parts.append("gripper_torque_lock")
            if status_byte & self.TEACH_STATUS_MOTOR_ERR:
                status_parts.append("motor_err")
        
        # 操作臂 (0x02) 状态解析
        elif device_code == self.DEVICE_OPERATING_ARM:
            with self._lock:
                self._oper_status["single_click"] = bool(status_byte & self.OPER_STATUS_SINGLE_CLICK)
                self._oper_status["double_click"] = bool(status_byte & self.OPER_STATUS_DOUBLE_CLICK)
                self._oper_status["long_press"] = bool(status_byte & self.OPER_STATUS_LONG_PRESS)
                self._oper_status["repeat_long_press"] = bool(status_byte & self.OPER_STATUS_REPEAT_LONG_PRESS)
                self._oper_status["reserved1"] = bool(status_byte & self.OPER_STATUS_RESERVED1)
                self._oper_status["reserved2"] = bool(status_byte & self.OPER_STATUS_RESERVED2)
                self._oper_status["gripper_torque_lock"] = bool(status_byte & self.OPER_STATUS_GRIPPER_TORQUE_LOCK)
                self._oper_status["motor_err"] = bool(status_byte & self.OPER_STATUS_MOTOR_ERR)
            
            if status_byte & self.OPER_STATUS_SINGLE_CLICK:
                status_parts.append("single_click")
            if status_byte & self.OPER_STATUS_DOUBLE_CLICK:
                status_parts.append("double_click")
            if status_byte & self.OPER_STATUS_LONG_PRESS:
                status_parts.append("long_press")
            if status_byte & self.OPER_STATUS_REPEAT_LONG_PRESS:
                status_parts.append("repeat_long_press")
            if status_byte & self.OPER_STATUS_RESERVED1:
                status_parts.append("reserved1")
            if status_byte & self.OPER_STATUS_RESERVED2:
                status_parts.append("reserved2")
            if status_byte & self.OPER_STATUS_GRIPPER_TORQUE_LOCK:
                status_parts.append("gripper_torque_lock")
            if status_byte & self.OPER_STATUS_MOTOR_ERR:
                status_parts.append("motor_err")
        
        # 其他设备类型，使用通用解析
        else:
            # 使用通用的位描述
            for bit in range(8):
                if status_byte & (1 << bit):
                    status_parts.append(f"bit{bit}")
        
        return ",".join(status_parts) if status_parts else "idle"

    # ==================== 请求数据解析 (Ask 模式) ====================
    def _parse_ask_joint_data(self, frame: List[int]) -> Optional[Dict]:
        """
        解析请求数据响应帧 (frame[2] 为 0x0x 模式)
        
        Frame Structure:
            frame[0]: Header (0xAA)
            frame[1]: CMD_JOINT (0x06)
            frame[2]: Func ID = 0x0x (请求数据) + 设备类型
            frame[3]: Data Length (LEN)
            frame[4]: Data Type = 0x0x/0x8x (发送/反馈) + 数据类型
            frame[5]: Offset Count (偏移数量)
            frame[6...]: Data Payload
            frame[-2]: Checksum
            frame[-1]: Footer (0xFF)
            
        Ask 模式反馈的最后一字节运行状态定义:
            示教臂 (0x01):
                0x01: 锁定状态
                0x02: 同步状态
                0x04: 待定1
                0x08: 待定2
                0x10: 待定3
                0x20: 待定4
                0x40: 夹具过高力矩的运动方向锁定
                0x80: 电机 ERR 状态
            
            操作臂 (0x02):
                0x01: 单击
                0x02: 双击
                0x04: 长按
                0x08: 重复长按
                0x10: 待定1
                0x20: 待定2
                0x40: 夹具过高力矩的运动方向锁定
                0x80: 电机 ERR 状态
        """
        # ==================== 基础长度检查 ====================
        data_len = frame[3]
        expected_min_len = 4 + data_len + 2  # header+cmd+func+LEN + DATA + checksum+footer
        if len(frame) < expected_min_len:
            logger.warning(f"[Ask] Joint frame length mismatch: LEN={data_len}, frame_len={len(frame)}")
            return None

        # ==================== 解析 frame[2] 设备类型 ====================
        func_id = frame[2]
        device_type = self._parse_device_type(func_id)
        
        # ==================== 提取有效数据 ====================
        data_start = 4
        data_end = data_start + data_len
        data_bytes = frame[data_start:data_end]
        
        if data_len < 2:
            logger.warning(f"[Ask] Data too short: need at least 2 bytes, got {data_len}")
            return None
        
        # ==================== 解析 frame[4] 数据类型和发送/反馈标志 ====================
        data_type_byte = data_bytes[0]  # frame[4] 在 data_bytes 中是索引 0
        is_feedback = (data_type_byte & 0x80) != 0  # 高位为 1 表示反馈数据
        data_type = self._parse_data_type(data_type_byte)
        
        # ==================== 解析 frame[5] 偏移数量 ====================
        offset_count = data_bytes[1] if len(data_bytes) > 1 else 0  # frame[5] 在 data_bytes 中是索引 1
        
        if self.debug_mode:
            logger.debug(
                f"[Ask] Device: {device_type}, DataType: {data_type}, "
                f"IsFeedback: {is_feedback}, Offset: {offset_count}"
            )
        
        # ==================== 判断是否为反馈数据 ====================
        if not is_feedback:
            # 发送数据 (0x0x)，不需要处理
            if self.debug_mode:
                logger.debug(f"[Ask] Send data packet, no processing needed")
            return {
                "type": "ask_joint_send",
                "device_type": device_type,
                "data_type": data_type,
                "offset_count": offset_count,
                "timestamp": time.time(),
            }
        
        # ==================== 反馈数据处理 ====================
        # 有效数据从 data_bytes[2] 开始（跳过 frame[4] 和 frame[5]）
        payload = data_bytes[2:]
        
        if len(payload) < 12:
            logger.warning(f"[Ask] Payload too short: expect ≥12 bytes, got {len(payload)}")
            return None
        
        # 提取运行状态（有效数据最后一字节，即校验位前一位）
        run_status = payload[-1]
        
        # 解析运行状态位 (根据设备类型使用不同的状态定义)
        device_code = func_id & 0x7F  # 获取设备类型代码
        run_status_text = self._parse_run_status(run_status, device_code)
        
        # 提取电机数据（去掉最后一字节状态）
        motor_bytes = payload[:-1]
        
        # 解析 7 个电机（6 关节 + 1 夹爪）
        joint_values: List[float] = [0.0] * 6
        gripper_value = 0.0
        
        for i in range(7):
            idx = i * 2
            if idx + 2 > len(motor_bytes):
                break
            
            chunk = motor_bytes[idx : idx + 2]
            
            if i < 6:
                # 关节 0-5
                # Pass joint index to apply direction correction
                joint_values[i] = self._bytes_to_radians(chunk, joint_index=i)
            else:
                # 夹爪（索引 6）
                gripper_low = chunk[0]
                gripper_high = chunk[1]
                gripper_raw = (gripper_low & 0xFF) | ((gripper_high & 0xFF) << 8)
                ratio = (self.servo_value_limit - 2048) / 100
                val = 100 - ((gripper_raw - 2048) / ratio)
                gripper_value = round(max(0, min(val, 100)), 2)
        
        # 更新状态
        with self._lock:
            self._run_status = run_status
            self._run_status_text = run_status_text
            self._device_type = device_type
            self._device_code = device_code
        
        self._update_joint_state(angles=joint_values, gripper=gripper_value, run_status_text=run_status_text)
        self._joint_event.set()
        
        if self.debug_mode:
            degrees = [round(rad * self.RAD_TO_DEG, 2) for rad in joint_values]
            logger.debug(
                f"[Ask] Joint angles (deg): {degrees}, gripper={gripper_value}, "
                f"run_status=0x{run_status:02X}({run_status_text})"
            )
        
        return {
            "type": "ask_joint_feedback",
            "device_type": device_type,
            "data_type": data_type,
            "offset_count": offset_count,
            "angles": self._joint_states.angles,
            "gripper": self._joint_states.gripper,
            "run_status": run_status,
            "run_status_text": run_status_text,
            "timestamp": self._joint_states.timestamp,
        }

    # ==================== 写入数据解析 (Write 模式) ====================
    def _parse_write_joint_data(self, frame: List[int]) -> Optional[Dict]:
        """
        解析写入数据响应帧 (frame[2] 为 0x8x 模式)
        
        Frame Structure:
            frame[0]: Header (0xAA)
            frame[1]: CMD_JOINT (0x06)
            frame[2]: Func ID = 0x8x (写入数据) + 设备类型
            frame[3]: Data Length (LEN)
            frame[4]: Data Type = 0x0x/0x8x (发送/反馈) + 数据类型
            frame[5]: Offset Count (偏移数量)
            frame[6...]: Data Payload
            frame[-2]: Checksum
            frame[-1]: Footer (0xFF)
            
        Write 模式反馈的最后一字节接收状态定义:
            0x01: 接收成功 (recv_success)
            0x02: 接收失败 (recv_failed)
            
        注意: Write 模式反馈包可能很短，不对数据长度进行严格检查
        """
        # ==================== 基础帧结构检查（不检查数据长度） ====================
        data_len = frame[3]
        expected_min_len = 4 + data_len + 2  # header+cmd+func+LEN + DATA + checksum+footer
        if len(frame) < expected_min_len:
            logger.warning(f"[Write] Joint frame length mismatch: LEN={data_len}, frame_len={len(frame)}")
            return None

        # ==================== 解析 frame[2] 设备类型 ====================
        func_id = frame[2]
        device_type = self._parse_device_type(func_id)
        
        # ==================== 提取有效数据 ====================
        data_start = 4
        data_end = data_start + data_len
        data_bytes = frame[data_start:data_end]
        
        if data_len == 0:
            logger.warning(f"[Write] Empty data payload")
            return None
        
        # ==================== 解析 frame[4] 数据类型和发送/反馈标志 ====================
        data_type_byte = data_bytes[0]  # frame[4] 在 data_bytes 中是索引 0
        is_feedback = (data_type_byte & 0x80) != 0  # 高位为 1 表示反馈数据
        data_type = self._parse_data_type(data_type_byte)
        
        # ==================== 解析 frame[5] 偏移数量 ====================
        offset_count = data_bytes[1] if len(data_bytes) > 1 else 0
        
        if self.debug_mode:
            logger.debug(
                f"[Write] Device: {device_type}, DataType: {data_type}, "
                f"IsFeedback: {is_feedback}, Offset: {offset_count}"
            )
        
        # ==================== 判断是否为反馈数据 ====================
        if not is_feedback:
            # 发送数据 (0x0x)，不需要处理
            if self.debug_mode:
                logger.debug(f"[Write] Send data packet, no processing needed")
            return {
                "type": "write_joint_send",
                "device_type": device_type,
                "data_type": data_type,
                "offset_count": offset_count,
                "timestamp": time.time(),
            }
        
        # ==================== 反馈数据处理（短包，不检查长度） ====================
        # 提取接收状态（有效数据最后一字节，即校验位前一位）
        recv_status = data_bytes[-1]
        
        # 解析接收状态 (Write 模式特有)
        if recv_status == self.RECV_STATUS_SUCCESS:
            recv_status_text = "recv_success"
        elif recv_status == self.RECV_STATUS_FAILED:
            recv_status_text = "recv_failed"
        else:
            recv_status_text = f"unknown_recv_0x{recv_status:02X}"
        
        # 更新状态
        with self._lock:
            self._run_status = recv_status
            self._run_status_text = recv_status_text
        
        self._joint_event.set()
        
        if self.debug_mode:
            logger.debug(f"[Write] Recv status: {recv_status_text} (0x{recv_status:02X})")
        
        return {
            "type": "write_joint_feedback",
            "device_type": device_type,
            "data_type": data_type,
            "offset_count": offset_count,
            "recv_status": recv_status,
            "recv_status_text": recv_status_text,
            "timestamp": time.time(),
        }


    def _parse_error_data(self, frame: List[int]) -> Dict:
        """
        Parse error data frame (0xEE).
        
        Args:
            frame: Complete data frame
        """
        # Minimal length check
        if len(frame) < 7:
            logger.warning("Error frame too short")
            return None
        
        # Extract error code and parameter
        error_code = frame[3]
        error_param = frame[4]
        
        error_types = {
            0x00: "Header/footer or length error",
            0x01: "Checksum error",
            0x02: "Mode error",
            0x03: "Invalid ID",
        }
        
        error_message = error_types.get(error_code, f"Unknown error (0x{error_code:02X})")
        
        logger.warning(f"Device error: {error_message}, param: 0x{error_param:02X}")
        
        return {
            "type": "error_data",
            "error_code": error_code,
            "error_param": error_param,
            "error_message": error_message,
            "timestamp": time.time()
        }
    

    #解析版本信息
    def _parse_version_data(self, frame: List[int]) -> Dict:
        """
        Parse version data frame (CMD=0x01).
        
        Protocol:
        | 0xAA | 0x01 | 0xFE | LEN | DATA... | CHECKSUM | 0xFF |
        
        DATA layout (LEN = 0x18 = 24 bytes):
          - Serial number: 16 ASCII bytes
          - Hardware version: 4 ASCII bytes
          - Firmware version: 4 ASCII bytes
        """
        # Basic length check: header(1)+CMD(1)+func(1)+LEN(1)+DATA(LEN)+checksum(1)+footer(1)
        if len(frame) < 4 + frame[3] + 2:
            logger.warning(f"Version frame too short: expect ≥{4 + frame[3] + 2}, got {len(frame)}")
            return None

        data_len = frame[3]
        data_start = 4
        data_end = data_start + data_len
        data_bytes = frame[data_start:data_end]

        if data_len < 24:
            logger.warning(f"Version data length too short: expect 24, got {data_len}")
            return None

        # Split fields according to protocol
        serial_bytes = data_bytes[0:16]
        hardware_bytes = data_bytes[16:20]
        firmware_bytes = data_bytes[20:24]

        def _bytes_to_ascii(b: List[int]) -> str:
            try:
                return "".join(chr(x) for x in b).strip()
            except Exception as e:
                logger.error(f"Version ASCII parse exception: {e}")
                return ""

        def _bytes_to_int(b: List[int]) -> int:
            """Convert 4 bytes to integer (Little Endian)"""
            if len(b) < 4: return 0
            return (b[0] & 0xFF) | ((b[1] & 0xFF) << 8) | ((b[2] & 0xFF) << 16) | ((b[3] & 0xFF) << 24)

        def _int_to_version_str(val: int) -> str:
            """Convert integer to version string (e.g. 100 -> v1.0.0)"""
            s = str(val)
            if len(s) >= 3:
                # 100 -> 1.0.0
                return f"v{s[:-2]}.{s[-2]}.{s[-1]}"
            elif len(s) == 2:
                # 25 -> 2.5.0
                return f"v{s[0]}.{s[1]}.0"
            else:
                # 5 -> 5.0.0
                return f"v{s}.0.0"

        serial_number = _bytes_to_ascii(serial_bytes)
        
        hw_val = _bytes_to_int(hardware_bytes)
        hardware_version = _int_to_version_str(hw_val)
        
        fw_val = _bytes_to_int(firmware_bytes)
        firmware_version = _int_to_version_str(fw_val)

        # Store firmware version (for upper-level API)
        with self._lock:
            self._firmware_version = firmware_version
            self._version_info = {
                "serial_number": serial_number,
                "hardware_version": hardware_version,
                "firmware_version": firmware_version,
            }
        
        # Signal that version info has been received and parsed
        self._version_event.set()
        
        if self.debug_mode:
            logger.debug(
                f"Version parsed: SN='{serial_number}', HW='{hardware_version}' ({hw_val}), FW='{firmware_version}' ({fw_val})"
            )

        return {
            "type": "version_data",
            "serial_number": serial_number,
            "hardware_version": hardware_version,
            "firmware_version": firmware_version,
            "timestamp": time.time(),
        }
    
    def _bytes_to_radians(self, byte_array: List[int], joint_index: int = -1) -> float:
        """
        Convert 2-byte array (little endian) to radians.
        Mapping: 0 -> -12.5 rad, 32768 -> 0 rad, 65535 -> +12.5 rad
        
        注意：必须与 ServoDriver._rad_to_hardware_value() 使用相同的映射范围，
        否则发送和接收的角度会不一致。
        
        Args:
            byte_array: List of 2 bytes [low, high]
            joint_index: Index of the joint (0-5) to apply direction correction. 
                         If -1, no direction correction is applied.
        """
        if len(byte_array) != 2:
            logger.warning(f"Data length error: need 2 bytes, got {len(byte_array)}")
            return 0.0
        
        # Build 16-bit integer (little endian)
        hex_value = (byte_array[0] & 0xFF) | ((byte_array[1] & 0xFF) << 8)
        
        # Range check (0-65535)
        if hex_value < 0 or hex_value > 65535:
            logger.warning(f"Servo value out of range: {hex_value} (valid 0–65535)")
            hex_value = max(0, min(hex_value, 65535))
        
        # Map raw value to radians: 0–65535 -> [-12.5, +12.5] rad
        # 与 ServoDriver._rad_to_hardware_value() 保持一致
        x_min = -12.5
        x_max = 12.5
        max_val = 65535
        
        radians = (hex_value / max_val) * (x_max - x_min) + x_min
        
        # Apply direction correction if joint index is valid
        if 0 <= joint_index < 6:
            radians *= self.joint_directions[joint_index]
            
        return radians
        return x_min + (hex_value / max_val) * (x_max - x_min)

    
    def _value_to_radians(self, value: int) -> float:
        """
        Convert servo raw value to radians.
        Mapping: 0 -> -12.5 rad, 32768 -> 0 rad, 65535 -> +12.5 rad
        
        注意：必须与 ServoDriver._rad_to_hardware_value() 使用相同的映射范围。
        """
        if value < 0 or value > 65535:
            logger.warning(f"Servo value out of range: {value} (valid 0–65535)")
            value = max(0, min(value, 65535))
        
        # Map raw value to radians: 0–65535 -> [-12.5, +12.5] rad
        x_min = -12.5
        x_max = 12.5
        max_val = 65535
        return x_min + (value / max_val) * (x_max - x_min)
                

