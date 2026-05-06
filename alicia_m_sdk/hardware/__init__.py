"""硬件层：串口通信、设备抽象与协议编解码"""

from .serial_port import SerialPort
from .device import Device, StateCache
from .frame import Frame, crc32_check
from .codec import MessageCodec

__all__ = [
    'SerialPort', 'Device', 'StateCache',
    'Frame', 'crc32_check', 'MessageCodec',
]
