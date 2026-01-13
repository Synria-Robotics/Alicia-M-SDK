"""
MIT模式参数配置工具

Copyright (c) 2025 Synria Robotics Co., Ltd.
Licensed under GPL 

功能说明:
此脚本用于配置机械臂的MIT模式参数。
- 只需在STM32控制板上电后运行一次即可
- 配置参数后，电机会回到0点
- 之后可以正常使用MIT模式控制，无需重复配置

使用场景:
1. 首次使用MIT模式前运行一次
2. 需要修改MIT控制参数时运行
3. 控制板重新上电后运行一次

注意事项:
- 运行此脚本会让所有电机回到零点
- 确保机械臂周围没有障碍物
- 配置完成后可以断开，不影响后续MIT模式使用
"""

import serial
import platform
import serial.tools.list_ports
import time
import os
import argparse
from typing import Optional


class MITConfigTool:
    """MIT模式参数配置工具"""
    
    # MIT模式初始化指令
    # 格式: [AA][06][81][48][00][05][Motor1_10字节][Motor2_10字节]...[Motor7_10字节][CRC][FF]
    # 每个电机10字节数据包含: 位置(2) + 速度(2) + 扭矩(2) + Kp(2) + Kd(2)
    MIT_INIT_COMMAND = bytes.fromhex(
        "AA 06 82 48 00 05 "
        "FF 7F FF 07 FF 07 99 19 66 66 "  # Motor 1: pos=32767, spd=2047, trq=2047, kp=2621, kd=26214
        "FF 7F FF 07 FF 07 3D 0A 33 33 "  # Motor 2: pos=32767, spd=2047, trq=2047, kp=2621, kd=26214
        "FF 7F FF 07 FF 07 99 19 66 66 "  # Motor 3: pos=32767, spd=2047, trq=2047, kp=6553, kd=26214
        "FF 7F FF 07 FF 07 3D 0A 33 33 "  # Motor 4: pos=32767, spd=2047, trq=2047, kp=2621, kd=13107
        "FF 7F FF 07 FF 07 3D 0A 33 33 "  # Motor 5: pos=32767, spd=2047, trq=2047, kp=2621, kd=13107
        "FF 7F FF 07 FF 07 3D 0A 33 33 "  # Motor 6: pos=32767, spd=2047, trq=2047, kp=2621, kd=13107
        "FF 7F FF 07 FF 07 3D 0A 33 33 "  # Motor 7 (Gripper): pos=39688 (+2.64 rad), spd=2047, trq=2047, kp=2621, kd=13107
        "2F FF"
    )
    
    def __init__(self, port: str = "", baudrate: int = 1000000):
        self.port_name = port
        self.baudrate = baudrate 
        self.serial_port: Optional[serial.Serial] = None
    
    def find_serial_port(self) -> str:
        """查找可用的串口设备"""
        if self.port_name:
            if os.path.exists(self.port_name):
                return self.port_name
            print(f"⚠ 指定的串口 {self.port_name} 不存在，将自动搜索...")
        
        try:
            ports = list(serial.tools.list_ports.comports())
        except Exception as e:
            print(f"✗ 列举串口时出错: {e}")
            return ""
        
        if not ports:
            print("✗ 未找到任何串口设备")
            return ""
        
        # 优先级: ttyUSB > ttyACM > cu.usbserial > COM
        priorities = {
            "Linux": ["ttyUSB", "ttyACM"],
            "Darwin": ["cu.usbserial", "cu.SLAB_USBtoUART"],
            "Windows": ["COM"]
        }
        
        system = platform.system()
        for key in priorities.get(system, priorities["Linux"]):
            for p in ports:
                if key in p.device:
                    return p.device
        
        return ""
    
    def connect(self) -> bool:
        """连接串口"""
        port = self.find_serial_port()
        if not port:
            print("✗ 未找到可用的串口设备")
            return False
        
        try:
            print(f"正在连接串口: {port}")
            self.serial_port = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                timeout=1.0,
                write_timeout=1.0
            )
            
            if self.serial_port.is_open:
                print(f"✓ 串口连接成功: {port} @ {self.baudrate} bps")
                time.sleep(0.1)  # 等待串口稳定
                return True
            return False
        except Exception as e:
            print(f"✗ 串口连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("✓ 串口已断开")
    
    def send_mit_init(self, repeat_times: int = 5) -> bool:
        """
        发送MIT模式初始化指令
        
        Args:
            repeat_times: 重复发送次数(建议5次，确保STM32接收)
        
        Returns:
            是否发送成功
        """
        if not self.serial_port or not self.serial_port.is_open:
            print("✗ 串口未连接")
            return False
        
        try:
            print(f"\n正在发送MIT模式初始化指令 (重复 {repeat_times} 次)...")
            print(f"数据包: {self.MIT_INIT_COMMAND.hex(' ').upper()}")
            
            for i in range(repeat_times):
                self.serial_port.write(self.MIT_INIT_COMMAND)
                self.serial_port.flush()
                print(f"  [{i+1}/{repeat_times}] 发送完成")
                time.sleep(0.05)  # 50ms间隔
            
            print("\n✓ MIT模式参数配置完成")
            print("提示:")
            print("  - 电机应该已回到零点位置")
            print("  - 现在可以使用MIT模式进行控制")
            print("  - 除非重新上电，否则无需再次配置")
            return True
            
        except Exception as e:
            print(f"✗ 发送MIT初始化指令失败: {e}")
            return False


def main():
    parser = argparse.ArgumentParser(
        description="MIT模式参数配置工具 - 配置机械臂MIT控制模式参数",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用说明:
  1. 确保机械臂已上电且周围无障碍物
  2. 运行此脚本配置MIT模式参数
  3. 配置完成后，电机会回到零点
  4. 之后可以正常使用MIT模式，无需重复配置
  
示例:
  python 00_config_mit_params.py                    # 自动搜索串口
  python 00_config_mit_params.py --port /dev/ttyUSB0  # 指定串口
  python 00_config_mit_params.py --repeat 3         # 指定重复次数
        """
    )
    
    parser.add_argument(
        '--port',
        type=str,
        default="",
        help="串口设备 (例如: /dev/ttyUSB0, COM3)，留空则自动搜索"
    )
    parser.add_argument(
        '--baudrate',
        type=int,
        default=1000000,
        help="波特率 (默认: 1000000)"
    )
    parser.add_argument(
        '--repeat',
        type=int,
        default=5,
        help="重复发送次数 (默认: 5，建议范围 3-10)"
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("MIT模式参数配置工具")
    print("=" * 60)
    print("\n⚠ 注意事项:")
    print("  1. 运行此脚本会使电机回到零点")
    print("  2. 请确保机械臂周围没有障碍物")
    print("  3. 配置只需在上电后运行一次")
    print()
    
    # 等待用户确认
    try:
        input("按 Enter 键继续，或 Ctrl+C 取消...")
    except KeyboardInterrupt:
        print("\n\n✗ 用户取消操作")
        return
    
    # 创建配置工具实例
    tool = MITConfigTool(port=args.port, baudrate=args.baudrate)
    
    try:
        # 连接串口
        if not tool.connect():
            return
        
        # 发送MIT初始化指令
        success = tool.send_mit_init(repeat_times=args.repeat)
        
        if success:
            print("\n" + "=" * 60)
            print("✓ 配置成功完成")
            print("=" * 60)
        else:
            print("\n" + "=" * 60)
            print("✗ 配置失败")
            print("=" * 60)
    
    except KeyboardInterrupt:
        print("\n\n✗ 操作被用户中断")
    
    finally:
        tool.disconnect()


if __name__ == "__main__":
    main()
