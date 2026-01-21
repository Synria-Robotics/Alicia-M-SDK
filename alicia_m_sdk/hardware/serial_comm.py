import serial
import platform
import serial.tools.list_ports
import time
import os
from typing import List, Optional, Tuple
import threading
from datetime import datetime
from PyCRC.CRC32 import CRC32
import getpass
from alicia_m_sdk.utils.logger import logger
# READ_LENGTH = 50
DEFAULT_LENGTH = 6


class SerialComm:
    """Robot arm serial communication module"""
    
    PLATFORM_PRIORITIES = {
        "Darwin": ["cu.wchusbserial", "cu.SLAB_USBtoUART", "cu.usbserial", "cu.usbmodem", "ttyUSB", "COM"],
        "Linux": ["ttyUSB", "ttyACM", "ttyCH343USB", "ttyCH341USB", "cu.wchusbserial", 
                 "cu.SLAB_USBtoUART", "cu.usbserial", "cu.usbmodem", "COM"],
        "Windows": ["COM", "ttyUSB", "cu.usbserial", "cu.usbmodem"]
    }

    def __init__(self, port: str = "", baudrate: int = 1000000,
                 timeout: float = 1.0, debug_mode: bool = False, lock: Optional[threading.Lock] = None):
        """
        Args:
            port: Serial port name, leave empty to auto-search
            baudrate: Baud rate
            timeout: Timeout in seconds
            debug_mode: Whether to enable debug mode
            lock: Optional thread lock, auto-created if not provided
        """
        self.port_name = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.debug_mode = debug_mode
        self.serial_port = None
        self.last_log_time = 0
        self._last_print_time = 0
        self._lock = lock if lock is not None else threading.Lock()
        self._rx_buffer = bytearray()
        self._frames_processed = 0
        self._frames_dropped = 0

    def __del__(self):
        """Destructor, ensure serial port is closed"""
        self.disconnect()

    def connect(self) -> bool:
        """Connect to serial port device"""
        try:
            port = self.find_serial_port()
            if not port:
                logger.warning("No available serial port found")
                return False

            has_permission, error_msg = self._check_serial_permissions(port)
            if not has_permission:
                logger.error(error_msg)
                return False

            logger.info(f"Connecting to port: {port}")

            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()

            port = self._prefer_cu_port(port)

            if 'cu.usbserial' in port:
                logger.info(f"Current baudrate is {self.baudrate}, if communication is abnormal, try 1000000/1000000/921600")

            self.serial_port = serial.Serial(
                port=port, baudrate=self.baudrate, timeout=self.timeout,
                write_timeout=self.timeout, xonxoff=False, rtscts=False, dsrdtr=False
            )
            self._initialize_serial_port()

            if self.serial_port.is_open:
                logger.info("Serial port connection successful")
                return True
            return False
        except Exception as e:
            logger.error(f"Serial port connection exception: {str(e)}")
            return False


    def disconnect(self):
        """Disconnect serial port connection"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            logger.info("Serial port closed")

    def is_connected(self) -> bool:
        """Check if serial port is connected and open"""
        return self.serial_port is not None and self.serial_port.is_open

    #寻找可用串口设备
    def find_serial_port(self) -> str:
        """Find available serial port device"""
        current_time = time.time()
        should_log = (current_time - self.last_log_time) >= 2.0

        # Handle user-specified port
        if self.port_name:
            device_name = self._normalize_device_name(self.port_name, should_log)
            if self._is_device_accessible(device_name):
                logger.info(f"Using specified port: {device_name}")
                return device_name
            logger.warning(f"Specified port {device_name} is not available, will search for other devices")

        # Get serial port list
        try:
            ports = list(serial.tools.list_ports.comports())
        except Exception as e:
            if should_log:
                logger.error(f"Exception when listing ports: {str(e)}")
                self.last_log_time = current_time
            return ""

        if not ports:
            return ""

        if should_log:
            self.last_log_time = current_time

        # Find device by priority
        for key in self.PLATFORM_PRIORITIES.get(platform.system(), self.PLATFORM_PRIORITIES["Windows"]):
            for p in ports:
                if key in p.device:
                    device_name = self._normalize_device_name(p.device, should_log)
                    if self._is_device_accessible(device_name):
                        return device_name

        # macOS: try to map tty.* to cu.*
        if platform.system() == "Darwin":
            for p in ports:
                if p.device.startswith('/dev/tty.'):
                    cu_candidate = p.device.replace('/dev/tty.', '/dev/cu.')
                    if os.path.exists(cu_candidate) and os.access(cu_candidate, os.R_OK | os.W_OK):
                        if should_log:
                            logger.info(f"Map {p.device} to {cu_candidate}")
                        return cu_candidate

        if should_log:
            logger.warning("No available serial port device found (supports ttyUSB/ttyACM/ttyCH343USB/cu.usbserial/cu.usbmodem/COM)")
        return ""

    #-------------------------------------------------数据发送-----------------------------------------------------#
    def send_data(self, data: List[int]) -> bool:
        """
        Send data to serial port

        Args:
            data: Byte data list to send

        Returns:
            bool: Whether send is successful
        """
        with self._lock:
            try:
                # 1) 未连接则尝试连接
                if not self.serial_port or not self.serial_port.is_open:
                    logger.warning("Serial port not open, trying to reconnect")
                    if not self.connect():
                        logger.error("Cannot connect to serial port")
                        return False

                # 2) 将整数列表转换为字节串并写入串口
                data_bytes = bytes(data)
                bytes_written = self.serial_port.write(data_bytes)

                # # 3) 轻微延时后 flush 输出缓冲区
                # time.sleep(0.001)
                try:
                    self.serial_port.flush()
                except Exception:
                    print("flush error")
                    pass

                # 4) 校验写入字节数是否与期望一致
                if bytes_written != len(data):
                    logger.warning(f"Only wrote {bytes_written} bytes, should be {len(data)} bytes")
                    return False

                # 5) 调试输出
                if self.debug_mode:
                    self._hex_print("Send", data)

                return True

            except Exception as e:
                logger.error(f"Exception sending data: {str(e)}")
                return False


    #-------------------------------------------------数据接收-----------------------------------------------------#
    def read_frame(self) -> Optional[List[int]]:
        """
        Read one frame of data (non-blocking, returns None if no complete frame)

        Returns:
            Optional[List[int]]: Complete data frame, returns None if not available
        """
        try:
            # 1) 未连接则尝试连接
            if not self.serial_port or not self.serial_port.is_open:
                if not self.connect():
                    return None

            # 2) 没有待读数据则返回 None（非阻塞）
            if self.serial_port.in_waiting == 0:
                return None

            # 3) 分批读取到内部接收缓冲 _rx_buffer
            available_bytes = self.serial_port.in_waiting
            max_read_size = 80
            read_size = min(available_bytes, max_read_size)
            self._rx_buffer += self.serial_port.read(read_size)

            # 4) 基于协议的帧提取循环
            while len(self._rx_buffer) >= 6:
                # 防御：缓冲溢出则清空
                if len(self._rx_buffer) > 200:
                    self._rx_buffer.clear()
                    continue

                # 4.1) 同步到帧头 0xAA：若首字节不是 0xAA，丢弃一字节继续
                if self._rx_buffer[0] != 0xAA:
                    self._rx_buffer.pop(0)
                    continue

                # if self.debug_mode:
                #     print(f" Buffer size: {len(self._rx_buffer)} bytes, first bytes: {self._rx_buffer[:min(12, len(self._rx_buffer))]}")

                # 4.2) 读取长度字段 Len（下标 3）
                data_len = self._rx_buffer[3]
                frame_length = data_len + DEFAULT_LENGTH  # DEFAULT_LENGTH=6

                # 4.3) 若缓冲不足整个帧长度则等待更多数据
                if len(self._rx_buffer) < frame_length:
                    break

                candidate = self._rx_buffer[:frame_length]

                # 4.4) 校验帧尾 0xFF
                valid_tail = candidate[-1] == 0xFF
                if not valid_tail:
                    # 尾部不对：当前 0xAA 可能是假头，丢弃一字节继续同步
                    self._rx_buffer.pop(0)
                    continue

                # 4.5) 校验 CRC（算法见下文）
                if self._serial_data_check(candidate):
                    # 成功：从缓冲中移除该帧并返回
                    self._rx_buffer = self._rx_buffer[frame_length:]
                    if self.debug_mode:
                        self._hex_print("Recv", list(candidate))
                    
                    # 强制打印接收到的数据包
                    # print(f"[RX] 接收数据包: {' '.join(f'{b:02X}' for b in candidate)}")

                    return list(candidate)
                else:
                    # 失败：打印原始帧内容并丢弃一个字节，继续同步
                    logger.warning(f"CRC Error. Raw: {' '.join(f'{b:02X}' for b in candidate)}")
                    self._rx_buffer.pop(0)

            return None

        except Exception as e:
            logger.error(f"Exception reading data: {str(e)}")
            return None


    #-------------------------------------------------数据校验-----------------------------------------------------#
    def _serial_data_check(self, frame: bytearray) -> bool:
        """
        Verify CRC8 checksum using specific robot algorithm.
        Frame: [AA] [Cmd] [Func] [Len] [Data...] [CRC] [FF]
        """
        received_checksum = frame[-2]
        # Payload includes Cmd, Func, Len, and Data (everything between Header and CRC)
        payload_to_check = frame[1:-2]

        calculated_checksum = self.calculate_checksum(payload_to_check)
        return received_checksum == calculated_checksum


    def calculate_checksum(self, data) -> int:
        """
        Use CRC-32 and only use the last 8 bits by pycrc
        """
        crc_calculator = CRC32()
        crc = crc_calculator.calculate(bytes(data))
        return crc & 0xFF


    #-------------------------------------------------数据统计-----------------------------------------------------#    
    def get_processing_stats(self) -> dict:
        """
        Get frame processing statistics
        
        Returns:
            dict: Contains statistics of processed and dropped frames
        """
        return {
            "frames_processed": self._frames_processed,
            "frames_dropped": self._frames_dropped,
            "buffer_size": len(self._rx_buffer)
        }

    def _prefer_cu_port(self, port: str) -> str:
        """Convert macOS tty.* to cu.* if available"""
        if '/dev/tty.' in port:
            cu_candidate = port.replace('/dev/tty.', '/dev/cu.')
            if os.path.exists(cu_candidate) and os.access(cu_candidate, os.R_OK | os.W_OK):
                logger.info(f"Detected macOS port {port}, switching to {cu_candidate} for writing")
                return cu_candidate
        return port

    def _initialize_serial_port(self):
        """Initialize serial port buffers and handshake lines"""
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()
        self.serial_port.setDTR(True)  # Some controllers ignore TX when DTR is low
        self.serial_port.setRTS(False)


    def _normalize_device_name(self, device_name: str, should_log: bool = False) -> str:
        """Normalize device name for Windows COM port and Linux path"""
        # Windows: add prefix for COM ports > 9
        if platform.system() == "Windows" and device_name.startswith("COM"):
            try:
                port_num = int(device_name[3:])
                if port_num > 9 and not device_name.startswith("\\\\.\\"):
                    device_name = f"\\\\.\\{device_name}"
                    if should_log:
                        logger.info(f"Windows COM port number greater than 9, add prefix: {device_name}")
            except ValueError:
                pass
        
        # Linux: ensure /dev/ prefix
        if platform.system() == "Linux" and not device_name.startswith("/dev/"):
            if device_name.startswith(("tty", "cu")):
                device_name = f"/dev/{device_name}"
        
        return device_name

    def _check_serial_permissions(self, device_name: str) -> Tuple[bool, Optional[str]]:
        """Check serial port device permissions"""
        if platform.system() == "Windows":
            return True, None
        
        if not os.path.exists(device_name):
            return False, f"Device {device_name} does not exist"
        
        if not os.access(device_name, os.R_OK | os.W_OK):
            current_user = getpass.getuser()
            system = platform.system()
            solutions = {
                "Linux": (
                    f"  1. Add user '{current_user}' to dialout group:\n"
                    f"     sudo usermod -a -G dialout {current_user}\n"
                    f"  2. Log out and log back in, or run: newgrp dialout\n"
                    f"  3. Or temporarily use: sudo chmod 666 {device_name}\n"
                ),
                "Darwin": (
                    f"  1. Add user '{current_user}' to dialout or uucp group\n"
                    f"  2. Or temporarily use: sudo chmod 666 {device_name}\n"
                )
            }
            solution = solutions.get(system, f"  Temporarily use: sudo chmod 666 {device_name}\n")
            return False, f"Insufficient permissions: Cannot access serial port device {device_name}\nSolution:\n{solution}"
        
        return True, None

    def _is_device_accessible(self, device_name: str) -> bool:
        """Check if device exists and is accessible"""
        if platform.system() == "Windows" and device_name.startswith(("COM", "\\\\.\\COM")):
            return True
        if not os.path.exists(device_name):
            return False
        # Permission check is done in connect() for detailed error messages
        has_permission, error_msg = self._check_serial_permissions(device_name)
        if not has_permission and error_msg and self.debug_mode:
            logger.warning(error_msg)
        return True

    def _hex_print(self, title: str, data: List[int]):
        hex_buf = ' '.join(f"{b:02X}" for b in data)
        print(f"[{title}] {hex_buf}")

    # ==================== 高频同步通信接口 ====================
    
    def send_and_receive_sync(self, data: List[int], timeout: float = 0.003) -> Tuple[bool, Optional[List[int]], float]:
        """
        同步发送数据并等待响应 (高频控制专用)
        
        此方法实现了发送-等待-接收的同步机制，确保每次发送后等待MCU响应完成，
        避免数据"撞车"问题。适用于需要高同步率的高频控制场景。
        
        Args:
            data: 要发送的字节数据列表
            timeout: 等待响应的超时时间 (秒), 默认3ms
            
        Returns:
            Tuple[bool, Optional[List[int]], float]:
                - 发送是否成功
                - 响应帧 (如果收到) 或 None
                - 往返延迟 (毫秒)
        
        Example:
            >>> success, response, latency = serial_comm.send_and_receive_sync(frame, timeout=0.003)
            >>> if success and response:
            ...     print(f"同步成功, 延迟: {latency:.2f}ms")
        """
        with self._lock:
            try:
                # 1) 检查连接状态
                if not self.serial_port or not self.serial_port.is_open:
                    if not self.connect():
                        return False, None, 0.0
                
                # 2) 清空接收缓冲区 (丢弃之前未读取的残留数据)
                self.serial_port.reset_input_buffer()
                
                # 3) 记录发送时间
                send_start = time.perf_counter()
                
                # 4) 发送数据
                data_bytes = bytes(data)
                bytes_written = self.serial_port.write(data_bytes)
                
                # 5) 立即刷新输出缓冲
                try:
                    self.serial_port.flush()
                except Exception:
                    pass
                
                if bytes_written != len(data):
                    return False, None, 0.0
                
                # 6) 等待响应 (高效自旋等待)
                response = self._wait_for_response_fast(timeout)
                
                # 7) 计算往返延迟
                latency_ms = (time.perf_counter() - send_start) * 1000
                
                if self.debug_mode:
                    self._hex_print("Send", data)
                    if response:
                        self._hex_print("Recv", response)
                
                return True, response, latency_ms
                
            except Exception as e:
                logger.error(f"Exception in send_and_receive_sync: {str(e)}")
                return False, None, 0.0
    
    def _wait_for_response_fast(self, timeout: float = 0.003) -> Optional[List[int]]:
        """
        高效等待响应帧 (自旋等待实现)
        
        使用自旋等待而非 sleep，实现更精确的超时控制和更低的延迟。
        
        Args:
            timeout: 超时时间 (秒)
            
        Returns:
            响应帧列表 或 None (超时)
        """
        if not self.serial_port or not self.serial_port.is_open:
            return None
        
        start_time = time.perf_counter()
        buffer = []
        frame_started = False
        expected_length = 0
        
        while (time.perf_counter() - start_time) < timeout:
            # 检查是否有数据可读
            waiting = self.serial_port.in_waiting
            if waiting > 0:
                # 读取所有可用数据
                raw_data = self.serial_port.read(waiting)
                
                for byte in raw_data:
                    if not frame_started:
                        # 寻找帧头 0xAA
                        if byte == 0xAA:
                            buffer = [byte]
                            frame_started = True
                    else:
                        buffer.append(byte)
                        
                        # 第4个字节(index 3)是数据长度
                        if len(buffer) == 4:
                            # 帧长度 = 数据长度 + 固定6字节 (AA + Cmd + Func + Len + CRC + FF)
                            expected_length = buffer[3] + DEFAULT_LENGTH
                        
                        # 检查是否收到完整帧
                        if expected_length > 0 and len(buffer) >= expected_length:
                            # 检查帧尾
                            if buffer[-1] == 0xFF:
                                return buffer
                            else:
                                # 帧尾错误，重新开始寻找
                                frame_started = False
                                buffer = []
                                expected_length = 0
            # 不使用 sleep，直接自旋 (最低延迟)
        
        return None  # 超时
    
    def set_low_latency_mode(self, enable: bool = True):
        """
        设置低延迟模式
        
        在高频通信场景下，可以通过此方法优化串口参数以降低延迟。
        
        Args:
            enable: 是否启用低延迟模式
        """
        if not self.serial_port or not self.serial_port.is_open:
            return
        
        try:
            if enable:
                # 设置更小的超时时间
                # 注意: write_timeout 不能设置太短，否则高频写入会导致 Write timeout 错误
                # read timeout 可以很短 (1ms)，但 write timeout 需要预留更多时间 (10ms)
                self.serial_port.timeout = 0.001
                self.serial_port.write_timeout = 0.01  # 10ms，避免高频写入时超时
                
                # 禁用硬件流控 (如果还没有)
                self.serial_port.rtscts = False
                self.serial_port.dsrdtr = False
                self.serial_port.xonxoff = False
                
                # Linux: 尝试设置低延迟模式
                import platform
                if platform.system() == "Linux":
                    try:
                        import fcntl
                        import struct
                        # TIOCGSERIAL / TIOCSSERIAL 常量
                        TIOCGSERIAL = 0x541E
                        TIOCSSERIAL = 0x541F
                        # 尝试设置 low_latency 标志
                        # 注意：这可能需要 root 权限
                    except Exception:
                        pass  # 忽略错误，使用默认设置
                
                if self.debug_mode:
                    logger.info("Low latency mode enabled")
            else:
                # 恢复默认超时
                self.serial_port.timeout = self.timeout
                self.serial_port.write_timeout = self.timeout
                
                if self.debug_mode:
                    logger.info("Low latency mode disabled")
                    
        except Exception as e:
            logger.warning(f"Failed to set low latency mode: {e}")





