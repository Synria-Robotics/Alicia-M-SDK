"""设备抽象层

Device 是硬件层的核心，统一管理通信调度和状态缓存。

线程模型:
- 写入路径: 调用方线程直接写串口（仅 write_lock 保护）
- 读取线程: 专用后台线程持续读帧、解析响应、更新状态缓存（纯读取，不写串口）
- 状态轮询线程: 专用后台线程周期性发送状态查询帧，保证空闲时缓存不过期
"""

import logging
import time
import threading
from typing import Optional, List, Dict, Any

from .serial_port import SerialPort
from ..protocol.frame import Frame
from ..protocol.codec import MessageCodec
from ..protocol.messages import JointStateRequest, MotorParamReadRequest
from ..protocol.constants import (
    CMD_VERSION, CMD_JOINT_STATE, CMD_ERROR, CMD_MOTOR_PARAM,
    AIM_FOLLOWER, NUM_MOTORS,
    POLL_ADDR_BASIC, POLL_ADDR_EXTENDED,
    ERROR_DESCRIPTIONS,
)
from ..types.state import JointState, MitParams, RobotStatus, VersionInfo

logger = logging.getLogger(__name__)


class StateCache:
    """线程安全的状态缓存

    核心策略: 原子引用替换，避免 deepcopy + 长锁持有。
    - 写入端（读线程）: 构造新对象 → 原子替换引用
    - 读取端（API 线程）: 读引用获取快照，微秒级返回
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._joint_state: Optional[JointState] = None
        self._version: Optional[VersionInfo] = None
        self._robot_status: Optional[RobotStatus] = None
        # send_and_wait 机制: cmd_id → (Event, 响应Frame)
        self._pending_events: Dict[int, threading.Event] = {}
        self._pending_responses: Dict[int, Optional[Frame]] = {}

    def update_joint_state(self, state: JointState) -> None:
        """原子替换关节状态（读线程调用）"""
        with self._lock:
            self._joint_state = state

    def get_joint_state(self) -> Optional[JointState]:
        """获取当前状态快照（API 线程调用）"""
        with self._lock:
            return self._joint_state

    def update_version(self, version: VersionInfo) -> None:
        """原子替换版本信息"""
        with self._lock:
            self._version = version

    def get_version(self) -> Optional[VersionInfo]:
        """获取版本信息"""
        with self._lock:
            return self._version

    def update_robot_status(self, status: RobotStatus) -> None:
        """原子替换运行状态"""
        with self._lock:
            self._robot_status = status

    def get_robot_status(self) -> Optional[RobotStatus]:
        """获取运行状态"""
        with self._lock:
            return self._robot_status

    def register_pending(self, cmd_id: int) -> threading.Event:
        """注册等待响应的 Event（send_and_wait 用）"""
        event = threading.Event()
        with self._lock:
            self._pending_events[cmd_id] = event
            self._pending_responses[cmd_id] = None
        return event

    def resolve_pending(self, cmd_id: int, frame: Frame) -> None:
        """触发匹配的 pending Event，附带响应帧"""
        with self._lock:
            event = self._pending_events.pop(cmd_id, None)
            if event is not None:
                self._pending_responses[cmd_id] = frame
        if event:
            event.set()

    def get_pending_response(self, cmd_id: int) -> Optional[Frame]:
        """获取 send_and_wait 的响应帧"""
        with self._lock:
            return self._pending_responses.pop(cmd_id, None)


class Device:
    """机器人设备抽象：非阻塞通信、异步状态更新

    Args:
        serial_port: 串口驱动实例
        codec: 消息编解码器实例
    """

    def __init__(self, serial_port: SerialPort, codec: MessageCodec):
        self._port = serial_port
        self._codec = codec
        self._state_cache = StateCache()
        self._write_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._read_thread: Optional[threading.Thread] = None
        self._poll_thread: Optional[threading.Thread] = None
        self._last_write_time: float = 0.0
        self._aim: int = AIM_FOLLOWER
        self._poll_paused = threading.Event()  # 轮询暂停控制
        self._poll_addr_count: int = POLL_ADDR_BASIC  # 默认基础查询，兼容旧固件

    # ========== 属性 ==========

    @property
    def codec(self) -> MessageCodec:
        """获取编解码器"""
        return self._codec

    @property
    def aim(self) -> int:
        """获取当前控制目标部位"""
        return self._aim

    def set_aim(self, aim: int) -> None:
        """设置控制目标部位（connect 自动检测后调用）"""
        self._aim = aim

    def set_poll_addr_count(self, count: int) -> None:
        """设置轮询查询的地址数量

        Args:
            count: POLL_ADDR_BASIC(3) 或 POLL_ADDR_EXTENDED(7)
        """
        self._poll_addr_count = count

    def pause_polling(self) -> None:
        """暂停轮询线程（模式切换等操作期间使用）"""
        self._poll_paused.set()

    def resume_polling(self) -> None:
        """恢复轮询线程"""
        self._poll_paused.clear()

    def flush(self) -> None:
        """清空串口缓冲区（委托给底层串口驱动）"""
        self._port.flush()

    # ========== 生命周期 ==========

    def start(self) -> None:
        """启动读取线程 + 状态轮询线程"""
        self._stop_event.clear()
        self._read_thread = threading.Thread(
            target=self._read_loop, daemon=True, name="alicia-read"
        )
        self._poll_thread = threading.Thread(
            target=self._poll_loop, daemon=True, name="alicia-poll"
        )
        self._read_thread.start()
        self._poll_thread.start()
        logger.debug("后台线程已启动")

    def stop(self) -> None:
        """停止所有后台线程（串口 read_timeout 保证线程可退出）"""
        self._stop_event.set()
        if self._read_thread:
            self._read_thread.join(timeout=2.0)
            self._read_thread = None
        if self._poll_thread:
            self._poll_thread.join(timeout=1.0)
            self._poll_thread = None
        logger.debug("后台线程已停止")

    # ========== 写入路径（非阻塞）==========

    # 连续写入最小间隔 (秒): 防止轮询帧与控制帧背靠背到达固件引发 CRC 错误
    _MIN_WRITE_GAP = 0.001

    def send_frame(self, frame: Frame) -> None:
        """发送原始帧（fire-and-forget，不等待响应）"""
        with self._write_lock:
            gap = time.perf_counter() - self._last_write_time
            if gap < self._MIN_WRITE_GAP:
                time.sleep(self._MIN_WRITE_GAP - gap)
            self._port.write(frame.encode())
            self._last_write_time = time.perf_counter()

    # ========== 高层发送接口 ==========

    def send_pv(self, aim: int, positions: List[float],
                velocities: List[float]) -> None:
        """发送 PV 控制帧（pos+vel，fire-and-forget）

        Args:
            aim: 目标部位 (AIM_LEADER / AIM_FOLLOWER)
            positions: 7 个电机的目标位置 (rad)
            velocities: 7 个电机的有符号速度 (rad/s)
        """
        frame = self._codec.encode_pv_control(aim, positions, velocities)
        self.send_frame(frame)

    def send_mit(
        self,
        aim: int,
        params: List[MitParams],
        linear_velocities: Optional[List[float]] = None,
    ) -> None:
        """发送 MIT 全参数帧（fire-and-forget，始终 6 地址）

        Args:
            aim: 目标部位
            params: 7 个电机的 MIT 参数
            linear_velocities: 线性轨迹插值速度 (rad/s)，
                None 时填充清零信号 (0xFFFF) 禁用插值
        """
        # 解包 MitParams 为 codec 所需的独立列表
        positions = [p.pos_ref for p in params]
        velocities = [p.vel_ref for p in params]
        torques = [p.t_ref for p in params]
        kps = [p.kp if p.kp is not None else 0.0 for p in params]
        kds = [p.kd if p.kd is not None else 0.0 for p in params]
        frame = self._codec.encode_mit_control(
            aim, positions, velocities, torques, kps, kds,
            linear_velocities=linear_velocities,
        )
        self.send_frame(frame)

    def send_and_wait(self, frame: Frame, expected_cmd: int,
                      timeout: float = 1.0) -> Optional[Frame]:
        """发送请求帧并等待响应

        仅用于低频操作：版本查询、模式切换等。
        通过 Event 机制等待读线程收到匹配响应，不阻塞串口。

        Args:
            frame: 要发送的请求帧
            expected_cmd: 期望响应的指令 ID
            timeout: 等待超时（秒）

        Returns:
            匹配的响应帧，超时返回 None
        """
        event = self._state_cache.register_pending(expected_cmd)
        self.send_frame(frame)
        event.wait(max(timeout, 0))
        return self._state_cache.get_pending_response(expected_cmd)

    # ========== 一次性查询 ==========

    def query_motor_params(
        self, param_addr: int, timeout: float = 1.0
    ) -> Optional[List[int]]:
        """读取所有电机的指定参数

        Args:
            param_addr: 参数地址 (如 MOTOR_PARAM_CTRL_MODE=0x0B)
            timeout: 等待超时 (秒)

        Returns:
            各电机的参数值列表（uint32），超时返回 None
        """
        query = self._codec.encode_motor_param_read(MotorParamReadRequest(
            aim=self._aim,
            start_motor=1,
            motor_count=NUM_MOTORS,
            param_addr=param_addr,
        ))
        resp = self.send_and_wait(query, CMD_MOTOR_PARAM, timeout=timeout)
        if resp is None:
            return None
        return self._codec.decode_motor_param_read_response(resp)

    # ========== 状态访问（读缓存，无 I/O）==========

    @property
    def joint_state(self) -> Optional[JointState]:
        """获取最新关节状态（从缓存原子读取）"""
        return self._state_cache.get_joint_state()

    @property
    def robot_status(self) -> Optional[RobotStatus]:
        """获取最新运行状态"""
        return self._state_cache.get_robot_status()

    @property
    def version_info(self) -> Optional[VersionInfo]:
        """获取版本信息"""
        return self._state_cache.get_version()

    # ========== 读取线程（内部）==========

    def _read_loop(self) -> None:
        """读取线程主循环

        职责: 持续读取串口帧 → 解析 → 更新状态缓存。
        绝不写串口，避免与写入路径竞争。
        """
        while not self._stop_event.is_set():
            raw = self._port.read_frame()
            if raw:
                try:
                    parsed = Frame.decode(raw)
                    self._dispatch_response(parsed)
                except Exception as e:
                    logger.debug(f"帧解析失败: {e}")

    # ========== 状态轮询线程（内部）==========

    def _poll_loop(self) -> None:
        """状态轮询线程：周期性发送 0x06 读取帧获取关节状态

        这是状态缓存的唯一数据来源。
        退避机制: 最近有写入时短暂跳过，避免与高频控制竞争带宽。
        """
        POLL_INTERVAL = 0.005    # 5ms → 200Hz（空闲时）
        WRITE_COOLDOWN = 0.003   # 3ms 写冷却

        while not self._stop_event.is_set():
            # 暂停检查
            if self._poll_paused.is_set():
                self._stop_event.wait(POLL_INTERVAL)
                continue

            elapsed = time.perf_counter() - self._last_write_time
            if elapsed > WRITE_COOLDOWN:
                try:
                    query = self._codec.encode_joint_state_request(
                        JointStateRequest(
                            aim=self._aim,
                            start_addr=0x00,
                            addr_count=self._poll_addr_count,
                        )
                    )
                    self.send_frame(query)
                except Exception as e:
                    logger.debug(f"轮询查询发送失败: {e}")

            self._stop_event.wait(POLL_INTERVAL)

    # ========== 响应分发 ==========

    def _dispatch_response(self, frame: Frame) -> None:
        """根据指令 ID 分发响应到对应处理器"""
        cmd_id = frame.cmd_id

        # 尝试触发 send_and_wait 的 pending event
        self._state_cache.resolve_pending(cmd_id, frame)

        if cmd_id == CMD_JOINT_STATE:
            self._handle_joint_state(frame)
        elif cmd_id == CMD_VERSION:
            self._handle_version(frame)
        elif cmd_id == CMD_ERROR:
            self._handle_error(frame)
        # 其他指令的响应（0x03, 0x05, 0x09, 0x11）通过 send_and_wait 处理

    def _handle_joint_state(self, frame: Frame) -> None:
        """处理 0x06 关节状态响应"""
        try:
            response = self._codec.decode_joint_state_response(frame)
            if response is None:
                return

            # 写入响应仅含 result 字节，不含状态数据
            if response.motor_data is None or len(response.motor_data) == 0:
                return

            # 解码为物理量字典（codec 已处理 M6 夹爪的特殊映射）
            phys = self._codec.decode_joint_state_physical(response)

            positions = phys.get('positions', [0.0] * NUM_MOTORS)
            velocities = phys.get('velocities')
            torques = phys.get('torques')
            kps = phys.get('kps')
            kds = phys.get('kds')
            linear_vels = phys.get('linear_vels')
            temperatures = phys.get('temperatures')

            # 构造 JointState（positions[0:6]=关节角度, positions[6]=夹爪值）
            # 其余字段保留全部 7 个电机数据（含夹爪）
            joint_state = JointState(
                angles=positions[:6] if len(positions) >= 6 else positions,
                gripper=positions[6] if len(positions) >= 7 else 0.0,
                timestamp=time.time(),
                run_status=phys.get('run_status', 0),
                velocities=velocities,
                torques=torques,
                kps=kps,
                kds=kds,
                linear_vels=linear_vels,
                temperatures=temperatures,
            )
            self._state_cache.update_joint_state(joint_state)

            # 解析运行状态字节
            run_status = phys.get('run_status', 0)
            if run_status is not None:
                status = self._parse_run_status(run_status)
                self._state_cache.update_robot_status(status)

        except Exception as e:
            logger.debug(f"关节状态解析失败: {e}")

    def _handle_version(self, frame: Frame) -> None:
        """处理 0x01 版本信息响应"""
        try:
            version_resp = self._codec.decode_version_response(frame)
            if version_resp:
                info = VersionInfo(
                    serial_number=version_resp.serial_number,
                    hardware_version=version_resp.hardware_version,
                    firmware_version=version_resp.firmware_version,
                    product_type=version_resp.serial_number[:2] if len(version_resp.serial_number) >= 2 else "",
                    device_type=version_resp.serial_number[2:3] if len(version_resp.serial_number) >= 3 else "",
                )
                self._state_cache.update_version(info)
        except Exception as e:
            logger.debug(f"版本信息解析失败: {e}")

    def _handle_error(self, frame: Frame) -> None:
        """处理 0xEE 错误反馈"""
        if len(frame.data) >= 1:
            error_type = frame.func_code
            error_data = frame.data[0] if frame.data else 0
            desc = ERROR_DESCRIPTIONS.get(error_type, f"未知错误(0x{error_type:02X})")
            logger.warning("固件检测得到的数据帧有误: %s (type=0x%02X, data=0x%02X)",
                           desc, error_type, error_data)

    @staticmethod
    def _parse_run_status(status_byte: int) -> RobotStatus:
        """解析运行状态字节"""
        return RobotStatus(
            is_locked=bool(status_byte & 0x01),
            is_synced=bool(status_byte & 0x02),
            has_motor_error=bool(status_byte & 0x80),
            gripper_torque_locked=bool(status_byte & 0x40),
            single_click=bool(status_byte & 0x01),
            double_click=bool(status_byte & 0x02),
            long_press=bool(status_byte & 0x04),
        )
