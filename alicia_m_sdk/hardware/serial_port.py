"""串口通信驱动

纯粹的串口 I/O 封装，不涉及任何协议逻辑。
串口初始化时设置 read_timeout，保证 read 操作不会无限阻塞，
这是读线程可安全退出的关键前提。
"""

import logging
from typing import Optional, List

import serial
import serial.tools.list_ports

from ..protocol.constants import FRAME_HEADER, FRAME_FOOTER

logger = logging.getLogger(__name__)


class SerialPort:
    """串口通信驱动

    提供串口连接、读写、端口发现等基础能力。
    帧解析采用基于长度的策略（非 0xFF 边界检测），因为数据区可能包含 0xFF。

    Args:
        port: 串口端口路径，空字符串表示自动发现
        baudrate: 波特率，默认 1Mbaud
    """

    # 串口读超时（秒）— 决定读线程最大响应延迟
    READ_TIMEOUT = 0.01  # 10ms，平衡灵敏度与 CPU 占用

    def __init__(self, port: str = "", baudrate: int = 1_000_000):
        self._port_name = port
        self._baudrate = baudrate
        self._serial: Optional[serial.Serial] = None
        self._rx_buffer = bytearray()

    @property
    def port_name(self) -> str:
        """当前配置或已连接的串口名。"""
        return self._port_name

    def set_port(self, port: str) -> None:
        """设置待连接串口名；仅允许在未连接时调用。"""
        if self.is_connected():
            raise serial.SerialException("串口已连接，不能切换端口")
        self._port_name = port

    def connect(self) -> bool:
        """连接串口

        如果 port 为空，自动发现可用端口。

        Returns:
            连接成功返回 True
        """
        port = self._port_name
        if not port:
            ports = self.find_ports()
            if not ports:
                logger.error("未找到可用串口设备")
                return False
            port = ports[0]
            logger.info(f"自动发现串口: {port}")

        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=self._baudrate,
                timeout=self.READ_TIMEOUT,
                write_timeout=0.1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            self.flush()
            self._port_name = port
            logger.info(f"串口已连接: {port} @ {self._baudrate} baud")
            return True
        except serial.SerialException as e:
            logger.error(f"串口连接失败: {e}")
            return False

    def disconnect(self) -> None:
        """断开串口连接"""
        if self._serial and self._serial.is_open:
            try:
                self.flush()
                self._serial.close()
            except Exception:
                pass
            logger.info("串口已断开")
        self._serial = None
        self._rx_buffer.clear()

    def flush(self) -> None:
        """清空串口缓冲区（硬件 UART + SDK 接收缓冲）

        用于模式切换、高频通信结束等场景，
        防止残留帧数据破坏后续协议解析。
        """
        if self._serial and self._serial.is_open:
            try:
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
            except Exception:
                pass
        self._rx_buffer.clear()

    def is_connected(self) -> bool:
        """检查连接状态"""
        return self._serial is not None and self._serial.is_open

    def write(self, data: bytes) -> None:
        """写入数据（由 Device.send_frame 的 write_lock 保护）

        Args:
            data: 完整的帧字节数据

        Raises:
            serial.SerialException: 写入失败
        """
        if not self.is_connected():
            raise serial.SerialException("串口未连接")
        self._serial.write(data)

    def read_frame(self) -> Optional[bytes]:
        """读取一帧完整数据（基于长度的帧解析）

        解析策略：
        1. 扫描帧头 0xAA（受 read_timeout 约束，无数据时返回 None）
        2. 读取 cmd_id(1B) + func_code(1B) + length(1B)
        3. 按 length 读取有效数据
        4. 读取 checksum(1B) + footer(1B)
        5. 验证 footer == 0xFF

        不依赖 0xFF 做帧边界（数据区可能包含 0xFF）。

        Returns:
            完整帧的原始字节（从帧头到帧尾），无数据或超时返回 None。
        """
        if not self.is_connected():
            return None

        try:
            # 尝试读取可用数据到缓冲区
            available = self._serial.in_waiting
            if available > 0:
                self._rx_buffer.extend(self._serial.read(min(available, 256)))
            elif len(self._rx_buffer) < 5:
                # 缓冲区不足一帧最小长度，阻塞等待（受 read_timeout 约束）
                data = self._serial.read(1)
                if data:
                    self._rx_buffer.extend(data)
                else:
                    return None  # 超时

            # 扫描帧头
            while len(self._rx_buffer) > 0:
                header_idx = self._rx_buffer.find(FRAME_HEADER)
                if header_idx < 0:
                    self._rx_buffer.clear()
                    return None
                if header_idx > 0:
                    # 丢弃帧头之前的数据
                    del self._rx_buffer[:header_idx]

                # 至少需要: 帧头(1) + cmd_id(1) + func_code(1) + length(1) = 4 字节
                if len(self._rx_buffer) < 4:
                    return None

                # 解析长度字段
                data_length = self._rx_buffer[3]

                # 完整帧长度: 帧头(1) + cmd_id(1) + func_code(1) + length(1)
                #            + data(N) + checksum(1) + footer(1)
                frame_length = 4 + data_length + 2

                if len(self._rx_buffer) < frame_length:
                    # 数据不足，等待下次读取
                    return None

                # 提取完整帧
                frame_bytes = bytes(self._rx_buffer[:frame_length])

                # 验证帧尾
                if frame_bytes[-1] != FRAME_FOOTER:
                    # 帧尾不匹配，丢弃帧头，继续搜索
                    del self._rx_buffer[:1]
                    continue

                # 帧完整，从缓冲区移除
                del self._rx_buffer[:frame_length]
                return frame_bytes

            return None

        except serial.SerialException:
            return None
        except Exception as e:
            logger.debug(f"读帧异常: {e}")
            return None

    @staticmethod
    def find_ports() -> List[str]:
        """发现可用的串口设备（跨平台）

        Returns:
            可用串口路径列表
        """
        ports = []
        for port_info in serial.tools.list_ports.comports():
            ports.append(port_info.device)
        return sorted(ports)
