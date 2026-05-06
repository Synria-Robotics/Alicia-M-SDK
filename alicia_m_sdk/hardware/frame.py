"""帧结构：Frame 数据类、帧组装/解析、CRC32 校验

帧格式:
┌──────┬───────┬───────┬──────────┬──────────┬───────┬──────┐
│ 帧头  │指令 ID │功能码  │有效数据长度│ 有效数据  │ 校验位 │ 帧尾  │
│ 0xAA │ 1字节  │ 1字节  │  1字节    │  N字节   │ 1字节  │ 0xFF │
└──────┴───────┴───────┴──────────┴──────────┴───────┴──────┘

校验位 = CRC32(指令ID + 功能码 + 有效数据长度 + 有效数据) & 0xFF

重要：帧解析采用基于长度的方式，而非 0xFF 边界检测，
因为有效数据中可能包含 0xFF 字节。
"""

import binascii
from dataclasses import dataclass

from .constants import FRAME_HEADER, FRAME_FOOTER


def crc32_check(data: bytes) -> int:
    """CRC32 校验，取低 8 位

    :param data, 需要校验的字节序列（指令ID + 功能码 + 有效数据长度 + 有效数据）
    :return, CRC32 结果的低 8 位（0x00~0xFF）
    """
    return binascii.crc32(data) & 0xFF


@dataclass
class Frame:
    """通信协议帧结构

    Attributes:
        cmd_id: 指令 ID（如 0x01, 0x06 等）
        func_code: 功能码（含读写方向位和部位标识）
        data: 有效数据（不含帧头帧尾、长度、校验位）
        checksum: CRC32 校验位（低 8 位），编码时自动计算
    """
    cmd_id: int
    func_code: int
    data: bytes = b''
    checksum: int = 0

    def encode(self) -> bytes:
        """将 Frame 组装为完整的二进制帧

        帧结构: [0xAA][cmd_id][func_code][length][data...][CRC8][0xFF]

        :return, 完整的二进制帧字节
        """
        length = len(self.data)
        # 构建校验数据：指令ID + 功能码 + 有效数据长度 + 有效数据
        check_payload = bytes([self.cmd_id, self.func_code, length]) + self.data
        self.checksum = crc32_check(check_payload)

        # 组装完整帧
        frame = bytes([
            FRAME_HEADER,
            self.cmd_id,
            self.func_code,
            length,
        ]) + self.data + bytes([
            self.checksum,
            FRAME_FOOTER,
        ])
        return frame

    @classmethod
    def decode(cls, raw: bytes) -> 'Frame':
        """从原始字节流中解析出一个 Frame

        基于长度字段解析，不依赖 0xFF 边界检测（数据中可能包含 0xFF）。
        包含 CRC 校验验证。

        :param raw, 原始字节流（必须包含完整帧：帧头 + 指令ID + 功能码 + 长度 + 数据 + 校验 + 帧尾）
        :return, 解析后的 Frame 对象
        """
        # 最小帧长度：帧头(1) + 指令ID(1) + 功能码(1) + 长度(1) + 校验(1) + 帧尾(1) = 6
        if len(raw) < 6:
            raise ValueError(f"帧数据过短: {len(raw)} 字节，最少需要 6 字节")

        # 验证帧头
        if raw[0] != FRAME_HEADER:
            raise ValueError(f"帧头错误: 期望 0x{FRAME_HEADER:02X}，实际 0x{raw[0]:02X}")

        cmd_id = raw[1]
        func_code = raw[2]
        data_length = raw[3]

        # 计算期望总帧长
        expected_total = 4 + data_length + 2  # 帧头+cmd+func+len + 数据 + 校验+帧尾
        if len(raw) < expected_total:
            raise ValueError(
                f"帧数据不完整: 期望 {expected_total} 字节，实际 {len(raw)} 字节"
            )

        # 提取数据
        data = raw[4:4 + data_length]

        # 提取校验位
        received_checksum = raw[4 + data_length]

        # 验证帧尾
        footer_pos = 4 + data_length + 1
        if raw[footer_pos] != FRAME_FOOTER:
            raise ValueError(
                f"帧尾错误: 期望 0x{FRAME_FOOTER:02X}，"
                f"实际 0x{raw[footer_pos]:02X}"
            )

        # CRC 校验
        check_payload = bytes([cmd_id, func_code, data_length]) + data
        expected_checksum = crc32_check(check_payload)
        if received_checksum != expected_checksum:
            raise ValueError(
                f"CRC 校验失败: 期望 0x{expected_checksum:02X}，"
                f"实际 0x{received_checksum:02X}"
            )

        return cls(
            cmd_id=cmd_id,
            func_code=func_code,
            data=bytes(data),
            checksum=received_checksum,
        )

    @staticmethod
    def expected_length(raw: bytes) -> int:
        """根据已接收的字节计算期望的完整帧长度

        用于硬件层的帧边界检测：先读取前 4 字节获取 length 字段，
        再按长度读取剩余数据。

        :param raw, 已接收的字节（至少 4 字节：帧头 + 指令ID + 功能码 + 长度）
        :return, 完整帧的总字节数
        """
        if len(raw) < 4:
            raise ValueError(f"需要至少 4 字节来确定帧长度，当前仅 {len(raw)} 字节")
        data_length = raw[3]
        return 4 + data_length + 2  # 头部(4) + 数据(N) + 校验(1) + 帧尾(1)

    def __repr__(self) -> str:
        data_hex = self.data.hex(' ') if self.data else '(空)'
        return (
            f"Frame(cmd=0x{self.cmd_id:02X}, func=0x{self.func_code:02X}, "
            f"len={len(self.data)}, data={data_hex}, crc=0x{self.checksum:02X})"
        )
