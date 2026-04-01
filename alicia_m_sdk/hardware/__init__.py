"""硬件层：串口通信与设备抽象"""

from .serial_port import SerialPort
from .device import Device, StateCache

__all__ = ['SerialPort', 'Device', 'StateCache']
