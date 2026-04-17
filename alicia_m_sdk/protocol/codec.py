"""编解码器：消息对象 <-> 二进制帧的双向转换

MessageCodec 是无状态的纯函数集合，负责将结构化消息对象转换为 Frame，
以及将 Frame 解析回消息对象。所有物理量到协议值的映射委托给 utils/conversion.py。

字节序: 所有多字节数据使用小端序（Little-Endian）。
12bit 数据存储方式: 2 字节，高 4 位保留置零，低 12 位为有效数据。
"""

import struct
from typing import List, Optional

from .constants import (
    CMD_VERSION, CMD_ZERO_RESET, CMD_TORQUE, CMD_JOINT_STATE,
    CMD_ENABLE, CMD_MOTOR_PARAM, CMD_ERROR,
    FUNC_READ_BIT, FUNC_WRITE_BIT,
    FUNC_VERSION_REQ, FUNC_VERSION_RESP,
    AIM_LEADER, AIM_FOLLOWER,
    ADDR_POSITION, ADDR_VELOCITY, ADDR_TORQUE, ADDR_KP, ADDR_KD,
    ADDR_LINEAR_VEL, ADDR_TEMPERATURE, LINEAR_VEL_CLEAR,
    PLACEHOLDER, ENABLE_ON, ENABLE_OFF, FEEDBACK_BIT,
    NUM_MOTORS,
    VERSION_SERIAL_LEN, VERSION_HARDWARE_LEN, VERSION_FIRMWARE_LEN,
    ERROR_DESCRIPTIONS,
)
from .frame import Frame
from .messages import (
    VersionRequest, VersionResponse,
    JointStateRequest, JointStateResponse,
    JointControlRequest, JointControlResponse,
    TorqueRequest, ZeroResetRequest,
    EnableRequest, MotorParamRequest, MotorParamReadRequest,
    ErrorResponse,
)
from alicia_m_sdk.utils.conversion import (
    encode_position, decode_position,
    encode_velocity, decode_velocity,
    encode_linear_velocity, decode_linear_velocity,
    encode_torque, decode_torque,
    encode_kp, decode_kp,
    encode_kd, decode_kd,
    encode_gripper, decode_gripper,
    decode_temperature,
)

# 夹爪电机索引
_GRIPPER_INDEX = NUM_MOTORS - 1  # M6 = index 6


class MessageCodec:
    """消息编解码器：消息对象 <-> Frame 的双向转换

    设计要点:
    - 无状态，所有方法均为纯函数，方便单元测试
    - 物理量 ↔ 协议值的映射调用 utils/conversion.py
    - 错误帧 (0xEE) 的解析在此统一处理
    """

    # ============================================================
    # 0x01 — 版本信息
    # ============================================================

    def encode_version_request(self) -> Frame:
        """编码版本查询请求

        请求帧: 功能码=0x7E, 数据=0xFE（占位符）

        Returns:
            编码后的 Frame 对象
        """
        return Frame(
            cmd_id=CMD_VERSION,
            func_code=FUNC_VERSION_REQ,
            data=bytes([PLACEHOLDER]),
        )

    def decode_version_response(self, frame: Frame) -> VersionResponse:
        """解码版本信息响应

        响应数据: 序列号(16B) + 硬件版本(4B) + 固件版本(4B) = 24 字节

        Args:
            frame: 接收到的 Frame 对象

        Returns:
            VersionResponse 消息对象

        Raises:
            ValueError: 指令ID不匹配或数据长度错误
        """
        self._check_cmd(frame, CMD_VERSION)
        expected_len = VERSION_SERIAL_LEN + VERSION_HARDWARE_LEN + VERSION_FIRMWARE_LEN
        if len(frame.data) < expected_len:
            raise ValueError(
                f"版本响应数据长度不足: 期望 {expected_len} 字节，实际 {len(frame.data)} 字节"
            )

        offset = 0
        # 序列号：16 字节 ASCII
        serial_raw = frame.data[offset:offset + VERSION_SERIAL_LEN]
        serial_number = serial_raw.decode('ascii', errors='replace').rstrip('\x00')
        offset += VERSION_SERIAL_LEN

        # 硬件版本：4 字节
        hw_raw = frame.data[offset:offset + VERSION_HARDWARE_LEN]
        hardware_version = self._parse_version_bytes(hw_raw)
        offset += VERSION_HARDWARE_LEN

        # 固件版本：4 字节
        fw_raw = frame.data[offset:offset + VERSION_FIRMWARE_LEN]
        firmware_version = self._parse_version_bytes(fw_raw)

        return VersionResponse(
            serial_number=serial_number,
            hardware_version=hardware_version,
            firmware_version=firmware_version,
        )

    # ============================================================
    # 0x06 — 关节状态获取（读取）
    # ============================================================

    def encode_joint_state_request(self, msg: JointStateRequest) -> Frame:
        """编码关节状态查询请求

        请求帧: [0xAA][0x06][aim][0x02][start_addr][addr_count][CRC8][0xFF]

        Args:
            msg: JointStateRequest 消息对象

        Returns:
            编码后的 Frame 对象
        """
        data = bytes([msg.start_addr, msg.addr_count])
        return Frame(
            cmd_id=CMD_JOINT_STATE,
            func_code=FUNC_READ_BIT | msg.aim,
            data=data,
        )

    def decode_joint_state_response(self, frame: Frame) -> JointStateResponse:
        """解码关节状态响应

        响应数据: [start_addr][addr_count][motor_data...][run_status]
        motor_data: 7 个电机 × addr_count 个地址 × 2 字节/地址

        Args:
            frame: 接收到的 Frame 对象

        Returns:
            JointStateResponse 消息对象

        Raises:
            ValueError: 数据格式错误
        """
        self._check_cmd(frame, CMD_JOINT_STATE)
        if len(frame.data) < 3:
            raise ValueError(f"关节状态响应数据过短: {len(frame.data)} 字节")

        start_addr = frame.data[0] & 0x7F  # 去除 bit7 反馈标志位
        addr_count = frame.data[1]

        # 计算电机数据区长度
        motor_bytes_per = addr_count * 2  # 每个电机 addr_count 个地址 × 2 字节
        motor_data_len = NUM_MOTORS * motor_bytes_per
        # 期望: 前缀(2) + 电机数据 + 运行状态(1)
        expected_data_len = 2 + motor_data_len + 1
        if len(frame.data) < expected_data_len:
            raise ValueError(
                f"关节状态响应数据不完整: 期望 {expected_data_len} 字节，"
                f"实际 {len(frame.data)} 字节"
            )

        # 解析电机数据
        motor_data: List[List[int]] = []
        offset = 2  # 跳过 start_addr 和 addr_count
        for _ in range(NUM_MOTORS):
            motor_values: List[int] = []
            for _ in range(addr_count):
                # 小端序读取 2 字节
                raw_value = struct.unpack_from('<H', frame.data, offset)[0]
                motor_values.append(raw_value)
                offset += 2
            motor_data.append(motor_values)

        # 运行状态（末尾 1 字节）
        run_status = frame.data[offset]

        return JointStateResponse(
            start_addr=start_addr,
            addr_count=addr_count,
            motor_data=motor_data,
            run_status=run_status,
        )

    # ============================================================
    # 0x06 — 关节控制（写入）
    # ============================================================

    def encode_joint_control(self, msg: JointControlRequest) -> Frame:
        """编码关节控制请求（写入目标状态）

        PV 帧: addr_count=2, 每电机 4 字节 (pos 16bit + vel 12bit)
        MIT 帧: addr_count=6, 每电机 12 字节 (pos + vel + torque + kp + kd + linear_vel)

        12bit 数据存储方式: 2 字节小端序，高 4 位保留置零，低 12 位为有效数据。

        Args:
            msg: JointControlRequest 消息对象，motor_data 为原始协议整数值

        Returns:
            编码后的 Frame 对象
        """
        # 构建数据区
        data = bytearray()
        data.append(msg.start_addr)
        data.append(msg.addr_count)

        for motor_values in msg.motor_data:
            for value in motor_values:
                # 所有值统一按 2 字节小端序写入
                # 12bit 数据的高 4 位已由调用方保证置零
                data.extend(struct.pack('<H', value & 0xFFFF))

        return Frame(
            cmd_id=CMD_JOINT_STATE,
            func_code=FUNC_WRITE_BIT | msg.aim,
            data=bytes(data),
        )

    def decode_joint_control_response(self, frame: Frame) -> JointControlResponse:
        """解码关节控制响应（写入结果反馈）

        响应帧: [start_addr|0x80][addr_count][result]
        result: 0x01=成功, 0x00=失败

        Args:
            frame: 接收到的 Frame 对象

        Returns:
            JointControlResponse 消息对象
        """
        self._check_cmd(frame, CMD_JOINT_STATE)
        if len(frame.data) < 3:
            raise ValueError(f"关节控制响应数据过短: {len(frame.data)} 字节")

        start_addr = frame.data[0] & 0x7F  # 去除 bit7 反馈标志位
        addr_count = frame.data[1]
        result = frame.data[2]

        return JointControlResponse(
            start_addr=start_addr,
            addr_count=addr_count,
            result=result,
        )

    # ============================================================
    # 0x05 — 力矩控制
    # ============================================================

    def encode_torque_request(self, msg: TorqueRequest) -> Frame:
        """编码力矩控制请求

        数据: 每个部位 2 字节（起始关节ID + 偏移数量）

        Args:
            msg: TorqueRequest 消息对象

        Returns:
            编码后的 Frame 对象
        """
        data = bytes([msg.start_joint, msg.joint_count])
        return Frame(
            cmd_id=CMD_TORQUE,
            func_code=FUNC_WRITE_BIT | msg.aim,
            data=data,
        )

    # ============================================================
    # 0x03 — 位姿重置
    # ============================================================

    def encode_zero_reset(self, msg: ZeroResetRequest) -> Frame:
        """编码位姿重置请求（设置当前位姿为零位）

        新固件协议: func_code 仅含部位标识（不带写入位）。
        数据前 2 字节 = 起始关节ID(0) + 关节数量；
        可选第 3 字节 = 调零方式（0x00=弱调零，0x01=强调零）。

        Args:
            msg: ZeroResetRequest 消息对象

        Returns:
            编码后的 Frame 对象
        """
        data = bytearray([msg.start_joint, msg.joint_count])
        if msg.reset_mode is not None:
            data.append(msg.reset_mode)
        return Frame(
            cmd_id=CMD_ZERO_RESET,
            func_code=msg.aim,
            data=bytes(data),
        )

    # ============================================================
    # 0x09 — 部位使能/失能
    # ============================================================

    def encode_enable_request(self, msg: EnableRequest) -> Frame:
        """编码使能/失能请求

        数据: 1 字节，0x01=使能，0x00=失能

        Args:
            msg: EnableRequest 消息对象

        Returns:
            编码后的 Frame 对象
        """
        data = bytes([ENABLE_ON if msg.enable else ENABLE_OFF])
        return Frame(
            cmd_id=CMD_ENABLE,
            func_code=FUNC_WRITE_BIT | msg.aim,
            data=data,
        )

    # ============================================================
    # 0x11 — 电机参数设置
    # ============================================================

    def encode_motor_param_request(self, msg: MotorParamRequest) -> Frame:
        """编码电机参数写入请求

        帧格式: [start_motor][motor_count][param_addr][param_value(4字节LE)]

        常见用法 — 模式切换:
            param_addr=0x0B, param_value=0x01(MIT) 或 0x02(PV)

        Args:
            msg: MotorParamRequest 消息对象

        Returns:
            编码后的 Frame 对象
        """
        data = bytearray()
        data.append(msg.start_motor)
        data.append(msg.motor_count)
        data.append(msg.param_addr)
        # 参数值: 4 字节小端序
        data.extend(struct.pack('<I', msg.param_value))
        return Frame(
            cmd_id=CMD_MOTOR_PARAM,
            func_code=FUNC_WRITE_BIT | msg.aim,
            data=bytes(data),
        )

    def encode_motor_param_read(self, msg: MotorParamReadRequest) -> Frame:
        """编码电机参数读取请求

        帧格式: [start_motor][motor_count][param_addr]
        读取时 func_code 不带 FUNC_WRITE_BIT。

        Args:
            msg: MotorParamReadRequest 消息对象

        Returns:
            编码后的 Frame 对象
        """
        data = bytes([msg.start_motor, msg.motor_count, msg.param_addr])
        return Frame(
            cmd_id=CMD_MOTOR_PARAM,
            func_code=msg.aim,
            data=data,
        )

    def decode_motor_param_read_response(self, frame: Frame) -> List[int]:
        """解码电机参数读取响应

        响应格式: [保留字节(3)][参数值(4字节LE)] × N 个电机
        前 3 字节为保留字段，后续按电机顺序返回 uint32 值。

        Args:
            frame: 接收到的 Frame 对象

        Returns:
            各电机的参数值列表
        """
        self._check_cmd(frame, CMD_MOTOR_PARAM)
        data = frame.data
        if len(data) < 3:
            return []
        values = []
        offset = 3  # 跳过保留字节
        while offset + 4 <= len(data):
            values.append(struct.unpack_from('<I', data, offset)[0])
            offset += 4
        return values

    # ============================================================
    # 0xEE — 错误反馈解码
    # ============================================================

    def decode_error_response(self, frame: Frame) -> ErrorResponse:
        """解码错误反馈响应

        错误类型由功能码字段标识，携带数据的含义取决于错误类型。

        Args:
            frame: 接收到的 Frame 对象

        Returns:
            ErrorResponse 消息对象
        """
        self._check_cmd(frame, CMD_ERROR)
        return ErrorResponse(
            error_code=frame.func_code,
            error_data=frame.data,
        )

    # ============================================================
    # 高级编码辅助：物理值 → 协议帧（含自动映射）
    # ============================================================

    def encode_pv_control(
        self,
        aim: int,
        positions: List[float],
        velocities: List[float],
    ) -> Frame:
        """编码 PV 模式控制帧（物理量输入）

        PV 帧: addr_count=2, 每电机 4 字节 (pos 16bit + vel 12bit)
        总数据长度: 2(前缀) + 7*4(电机数据) = 30 字节

        Args:
            aim: 目标部位（AIM_LEADER / AIM_FOLLOWER）
            positions: 7 个电机的目标位置 (rad)
            velocities: 7 个电机的有符号目标速度 (rad/s)

        Returns:
            编码后的 Frame 对象
        """
        self._validate_motor_list(positions, "positions")
        self._validate_motor_list(velocities, "velocities")

        motor_data: List[List[int]] = []
        for i in range(NUM_MOTORS):
            # M6（夹爪）: [0, 1000] → [0, 65535]，关节: [-12.5, +12.5] rad → [0, 65535]
            if i == _GRIPPER_INDEX:
                pos_raw = encode_gripper(positions[i])
            else:
                pos_raw = encode_position(positions[i])
            vel_raw = encode_velocity(velocities[i])
            motor_data.append([pos_raw, vel_raw])

        return self.encode_joint_control(JointControlRequest(
            aim=aim,
            start_addr=ADDR_POSITION,
            addr_count=2,
            motor_data=motor_data,
        ))

    def encode_mit_control(
        self,
        aim: int,
        positions: List[float],
        velocities: List[float],
        torques: List[float],
        kps: List[float],
        kds: List[float],
        linear_velocities: Optional[List[float]] = None,
    ) -> Frame:
        """编码 MIT 模式控制帧（物理量输入，全参数，始终 6 地址）

        始终发送 addr_count=6 的帧 (pos + vel + torque + kp + kd + linear_vel)。
        当 linear_velocities 为 None 时，线性轨迹速度填充清零信号 (0xFFFF)，
        通知固件禁用线性轨迹插值。

        Args:
            aim: 目标部位
            positions: 7 个电机的目标位置 (rad)
            velocities: 7 个电机的目标速度 (rad/s)
            torques: 7 个电机的前馈力矩 (N·m)
            kps: 7 个电机的 Kp 增益
            kds: 7 个电机的 Kd 增益
            linear_velocities: 7 个电机的线性轨迹插值速度 (rad/s)，
                None 时填充清零信号 (0xFFFF) 禁用插值

        Returns:
            编码后的 Frame 对象
        """
        self._validate_motor_list(positions, "positions")
        self._validate_motor_list(velocities, "velocities")
        self._validate_motor_list(torques, "torques")
        self._validate_motor_list(kps, "kps")
        self._validate_motor_list(kds, "kds")
        if linear_velocities is not None:
            self._validate_motor_list(linear_velocities, "linear_velocities")

        motor_data: List[List[int]] = []
        for i in range(NUM_MOTORS):
            # M6（夹爪）: [0, 1000] → [0, 65535]，关节: [-12.5, +12.5] rad → [0, 65535]
            if i == _GRIPPER_INDEX:
                pos_raw = encode_gripper(positions[i])
            else:
                pos_raw = encode_position(positions[i])
            vel_raw = encode_velocity(velocities[i])
            tor_raw = encode_torque(torques[i], motor_index=i)
            kp_raw = encode_kp(kps[i])
            kd_raw = encode_kd(kds[i])
            # 线性轨迹速度: 有值时编码 [0, 10] rad/s，无值时填充 0xFFFF 清零信号
            if linear_velocities is not None:
                lv_raw = encode_linear_velocity(linear_velocities[i])
            else:
                lv_raw = LINEAR_VEL_CLEAR
            motor_data.append([pos_raw, vel_raw, tor_raw, kp_raw, kd_raw, lv_raw])

        return self.encode_joint_control(JointControlRequest(
            aim=aim,
            start_addr=ADDR_POSITION,
            addr_count=6,
            motor_data=motor_data,
        ))

    # 0x06 地址 → (字段名, 解码函数) 映射
    # 按物理地址索引，支持任意 start_addr 的响应解码
    _ADDR_DECODERS = {
        ADDR_POSITION:   ("positions",   lambda raw, i: decode_gripper(raw) if i == _GRIPPER_INDEX else decode_position(raw)),
        ADDR_VELOCITY:   ("velocities",  lambda raw, i: decode_velocity(raw)),
        ADDR_TORQUE:     ("torques",     lambda raw, i: decode_torque(raw, motor_index=i)),
        ADDR_KP:         ("kps",         lambda raw, i: decode_kp(raw)),
        ADDR_KD:         ("kds",         lambda raw, i: decode_kd(raw)),
        ADDR_LINEAR_VEL: ("linear_vels", lambda raw, i: decode_linear_velocity(raw)),
        ADDR_TEMPERATURE:("temperatures",lambda raw, i: decode_temperature(raw)),
    }

    def decode_joint_state_physical(
        self,
        response: JointStateResponse,
    ) -> dict:
        """将关节状态响应的原始数据转换为物理量

        根据 start_addr 和 addr_count 自动选择对应的解码器。

        Args:
            response: JointStateResponse 消息对象

        Returns:
            字典，键为字段名（positions / velocities / torques / kps / kds /
            linear_vels / temperatures），值为各电机的物理量列表。
            附加 "run_status" 字段。
        """
        result: dict = {"run_status": response.run_status}

        for data_idx in range(response.addr_count):
            addr = response.start_addr + data_idx
            entry = self._ADDR_DECODERS.get(addr)
            if entry is None:
                continue
            field_name, decoder = entry
            values = []
            for motor_idx in range(len(response.motor_data)):
                raw = response.motor_data[motor_idx][data_idx]
                values.append(decoder(raw, motor_idx))
            result[field_name] = values

        return result

    # ============================================================
    # 帧分发：根据 cmd_id 自动选择解码方法
    # ============================================================

    def decode_frame(self, frame: Frame):
        """根据帧的指令 ID 自动分发到对应的解码方法

        Args:
            frame: 接收到的 Frame 对象

        Returns:
            对应的消息对象（VersionResponse / JointStateResponse /
            JointControlResponse / ErrorResponse）

        Raises:
            ValueError: 未知的指令 ID
        """
        if frame.cmd_id == CMD_VERSION:
            return self.decode_version_response(frame)
        elif frame.cmd_id == CMD_JOINT_STATE:
            # 区分状态响应和控制响应：检查 start_addr 的 bit7
            if len(frame.data) >= 1 and (frame.data[0] & FEEDBACK_BIT):
                return self.decode_joint_control_response(frame)
            else:
                return self.decode_joint_state_response(frame)
        elif frame.cmd_id == CMD_ERROR:
            return self.decode_error_response(frame)
        else:
            raise ValueError(f"未知或不支持解码的指令 ID: 0x{frame.cmd_id:02X}")

    # ============================================================
    # 内部辅助方法
    # ============================================================

    @staticmethod
    def _check_cmd(frame: Frame, expected_cmd: int) -> None:
        """校验帧的指令 ID 是否匹配"""
        if frame.cmd_id != expected_cmd:
            raise ValueError(
                f"指令 ID 不匹配: 期望 0x{expected_cmd:02X}，"
                f"实际 0x{frame.cmd_id:02X}"
            )

    @staticmethod
    def _validate_motor_list(values: List[float], name: str) -> None:
        """校验电机数据列表长度是否为 NUM_MOTORS"""
        if len(values) != NUM_MOTORS:
            raise ValueError(
                f"{name} 列表长度错误: 期望 {NUM_MOTORS}，实际 {len(values)}"
            )

    @staticmethod
    def _parse_version_bytes(raw: bytes) -> str:
        """解析版本号字节为可读字符串

        4 字节版本号: 解读为 uint32 小端序整数（固件版本号为数值，非 ASCII）。
        例如: b'\\x64\\x00\\x00\\x00' → 100 → "100"
        """
        import struct
        if len(raw) >= 4:
            value = struct.unpack_from('<I', raw)[0]
            return str(value)
        # 不足 4 字节时回退
        return '.'.join(str(b) for b in raw if b != 0)
