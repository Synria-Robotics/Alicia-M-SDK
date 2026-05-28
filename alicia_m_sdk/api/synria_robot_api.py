"""
SynriaRobotAPI — Alicia-M 机器人 SDK 主接口
"""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Union, Any, Mapping

import numpy as np

from ..types.state import MitParams
from ..types.config import RobotConfig
from ..types.enums import ControlMode
from ..types.exceptions import (
    ConnectionError, RobotStateError,
)
from ..hardware.codec import MessageCodec
from ..hardware.constants import (
    CMD_VERSION,
    MOTOR_PARAM_CTRL_MODE, CTRL_MODE_NAMES,
    CMD_GRIPPER_PARAM,
    POLL_ADDR_BASIC, POLL_ADDR_EXTENDED,
)
from ..hardware.serial_port import SerialPort
from ..hardware.device import Device
from ..execution.joint_control import JointController
from ..execution.trajectory_executor import (
    TrajectoryExecutor,
    execute_joint_trajectory as _execute_joint_trajectory,
    resolve_default_traj_save_dir as _resolve_default_traj_save_dir,
    save_trajectory_csv as _save_trajectory_csv,
)
from ..execution.joint_mapping import convert_joints_rad_from_alicia_d_to_alicia_m
from ..execution.teleoperation import Teleoperation
from ..diagnostics import DiagnosticResult, run_diagnostic as _run_diagnostic
from ..user_settings import UserSettings
from ..user_settings import get_user_settings as _get_user_settings
from ..user_settings import set_gripper_type as _set_gripper_type
from ..user_settings import gripper_type_config_value
from ..gripper_params import (
    GripperParamResult,
    make_read_gripper_params_frame,
    make_write_gripper_params_frame,
    parse_gripper_params_response,
)
from ..integrations.robocore import kinematics as kin_module
from ..integrations.robocore import planning as plan_module
from ..utils.beauty_logger import logger
from ..utils.version import supports_min_version
from ..utils.demo_runtime import (
    auto_generate_waypoints as _auto_generate_waypoints,
    load_waypoints_interactive as _load_waypoints_interactive,
    make_mapped_teleop_state_printer as _make_mapped_teleop_state_printer,
    manual_record_waypoints as _manual_record_waypoints,
    manual_record_waypoints_with_torque_off as _manual_record_waypoints_with_torque_off,
    print_diagnostic_response as _print_diagnostic_response,
    print_joint_state as _print_joint_state,
    print_robot_state as _print_robot_state,
)
from .._internal import connection as _connection


class SynriaRobotAPI:
    """Alicia-M 机器人 SDK 主接口

    提供面向用户的统一控制入口，内部委托各子模块执行具体逻辑。

    :param config, 机器人配置
    :param robot_model, RoboCore RobotModel 实例（由 create_robot 传入）
    """

    def __init__(self, config: RobotConfig, robot_model=None):
        self._config = config
        self._serial_port = SerialPort(config.port, config.baudrate)
        self._codec = MessageCodec()
        self._device = Device(self._serial_port, self._codec)
        # control_mode 为 None 时暂用 PV，connect() 中会检测固件模式并同步
        self._joint_ctrl = JointController(
            self._device,
            control_mode=config.control_mode or "pv",
            joint_limits_lower=config.joint_limits_lower,
            joint_limits_upper=config.joint_limits_upper,
        )
        self._traj_executor = TrajectoryExecutor(self._device)
        self._robot_model = robot_model
        self._connected = False
        # Tracks whether the caller requested automatic version detection.
        # "auto" means we must resolve the URDF version after the handshake;
        # any other value means the model was already loaded (or deliberately
        # skipped) before connect() was called.
        self._model_version_mode: str = config.version
        # Resolved synriard version string, e.g. "v1_1".  Set after a
        # successful auto-detection or when an explicit version is used.
        self._resolved_model_version: Optional[str] = (
            None if config.version == "auto" else config.version
        )

    def __enter__(self) -> "SynriaRobotAPI":
        """Enter a managed robot session, connecting if needed."""
        if not self.is_connected():
            self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        """Always disconnect when leaving a managed robot session."""
        self.disconnect()
        return False

    def __del__(self):
        try:
            self.disconnect()
        except Exception:
            pass

    @property
    def robot_model(self):
        """获取 RoboCore 机器人模型实例"""
        return self._robot_model

    @property
    def resolved_model_version(self) -> Optional[str]:
        """实际加载的 URDF 版本字符串，例如 ``'v1_1'``。

        * 当 ``version="auto"`` 时，连接成功后由固件硬件版本自动填充。
        * 当 ``version`` 为显式值时，等同于 ``config.version``。
        * 连接前或自动检测失败时返回 ``None``。
        """
        return self._resolved_model_version

    @property
    def control_mode(self) -> ControlMode:
        """获取当前控制模式"""
        return self._joint_ctrl.mode

    @property
    def connected_port(self) -> str:
        """获取当前连接的串口名。"""
        return self._serial_port.port_name

    # ========== 连接管理 ==========

    def connect(self, timeout: float = 5.0) -> bool:
        """连接机器人

        总超时覆盖整个连接流程：串口打开 + 后台线程启动 + 自动检测 + 首次状态获取。

        :param timeout, 总超时（秒）
        :return, 连接成功返回 True
        """
        if self.is_connected():
            return True

        if self._config.port:
            self._connect_once(timeout)
            logger.info(f"Connected to serial port: {self.connected_port}")
            return True

        ports = SerialPort.find_ports()
        if not ports:
            raise ConnectionError("No serial ports found")

        ports = list(reversed(ports))
        logger.info(f"Found serial ports: {', '.join(ports)}")
        errors = []
        for port in ports:
            logger.info(f"Trying serial port: {port}")
            try:
                self._serial_port.set_port(port)
                self._connect_once(timeout)
                logger.info(f"Auto-connected to serial port: {self.connected_port}")
                return True
            except Exception as exc:
                errors.append(f"{port}: {exc}")
                logger.debug(f"Serial port {port} auto-detection failed: {exc}")
                self.disconnect()

        detail = "; ".join(errors)
        raise ConnectionError(f"Failed to auto-detect Alicia-M serial port. Candidates: {', '.join(ports)}. {detail}")

    def _connect_once(self, timeout: float) -> bool:
        """在当前 SerialPort.port_name 上完成一次 Alicia-M 握手。"""
        try:
            robot_model, resolved_version = _connection.connect_once(
                self._serial_port, self._device, self._codec,
                self._joint_ctrl, self._config,
                model_version_mode=self._model_version_mode,
                timeout=timeout,
            )
        except Exception:
            self.disconnect()
            raise
        if robot_model is not None:
            self._robot_model = robot_model
            self._resolved_model_version = resolved_version
        self._connected = True
        logger.info(f"Robot connected: {self.connected_port}")
        return True

    def disconnect(self) -> None:
        """断开连接：停止后台线程 → 关闭串口"""
        was_connected = self._connected or self._serial_port.is_connected()
        self._device.stop()
        self._serial_port.disconnect()
        self._connected = False
        if was_connected:
            logger.info("Robot disconnected")

    def is_connected(self) -> bool:
        """检查连接状态"""
        return self._connected and self._serial_port.is_connected()

    # ========== 状态查询 ==========

    def get_robot_state(self, info_type: str = "joint_gripper") -> Any:
        """获取机器人状态

        缓存类（从轮询状态缓存读取，微秒级返回）:
            - "joint": 6个关节角度 (rad)
            - "joint_gripper": 关节角度 + 夹爪值
            - "velocity": 关节速度
            - "torque": 关节力矩
            - "linear_vels": 插补速度
            - "temperatures": 线圈温度
            - "all": 完整 JointState 对象
            - "version": 版本信息
            - "status": 运行状态

        一次性查询（发送请求等待响应）:
            - "control_mode": 各电机控制模式 (0x11)

        :param info_type, 查询类型
        :return, 对应类型的状态数据，不可用时返回 None
        """
        state = self._device.joint_state

        if info_type == "joint":
            return list(state.angles) if state else None
        elif info_type == "joint_gripper":
            if state is None:
                return None
            return {
                'angles': list(state.angles),
                'gripper': state.gripper,
            }
        elif info_type == "velocity":
            return list(state.velocities) if state and state.velocities else None
        elif info_type == "torque":
            return list(state.torques) if state and state.torques else None
        elif info_type == "all":
            return state
        elif info_type == "linear_vels":
            return list(state.linear_vels) if state and state.linear_vels else None
        elif info_type == "temperatures":
            return list(state.temperatures) if state and state.temperatures else None
        elif info_type == "control_mode":
            return self._query_control_modes()
        elif info_type == "version":
            return self._device.version_info
        elif info_type == "status":
            return self._device.robot_status
        else:
            logger.warning(f"未实现的状态查询类型: {info_type}")
            return None

    def get_pose(self) -> Optional[Dict]:
        """获取末端位姿（通过 FK 计算）

        :return, 位姿字典 {transform, position, rotation, euler_xyz, quaternion_xyzw}
        """
        if self._robot_model is None:
            logger.warning("未加载机器人模型，无法计算位姿")
            return None
        state = self._device.joint_state
        if state is None:
            return None
        return kin_module.compute_forward_kinematics(
            self._robot_model, list(state.angles)
        )

    def compute_forward_kinematics(
        self,
        joints: List[float],
        joint_format: str = "rad",
    ) -> Dict:
        """@brief 对指定关节角计算正运动学。

        @param joints 关节角列表。
        @param joint_format 关节角单位，支持 "rad" 或 "deg"。
        @return 位姿字典。
        """
        if self._robot_model is None:
            raise RobotStateError("未加载机器人模型")
        q = [math.radians(a) for a in joints] if joint_format == "deg" else list(joints)
        return kin_module.compute_forward_kinematics(self._robot_model, q)

    def get_firmware_version(self, timeout: float = 5.0) -> Optional[str]:
        """获取固件版本

        :param timeout, 查询超时
        :return, 固件版本字符串，如 "v1.1.0"
        """
        frame = self._codec.encode_version_request()
        self._device.send_and_wait(frame, CMD_VERSION, timeout=timeout)
        info = self._device.version_info
        return info.firmware_version if info else None

    # ========== 关节控制 ==========

    def set_robot_state(
        self,
        target_joints: Optional[List[float]] = None,
        gripper_value: Optional[float] = None,
        joint_format: str = 'deg',
        speed: float = 40,
        gripper_speed: float = 100,
        wait_for_completion: bool = True,
        use_interpolation: bool = True,
        kp: Optional[Union[float, List[float]]] = None,
        kd: Optional[Union[float, List[float]]] = None,
        torque: Optional[Union[float, List[float]]] = None,
        vel_ref: Optional[Union[float, List[float]]] = None,
        **kwargs,
    ) -> bool:
        """点位运动：设置关节角度和/或夹爪位置

        自动根据当前模式选择实现：
        - PV: 发送 pos+vel 帧，固件自行到达
        - MIT: 默认使用线性轨迹插值，发送 6 地址单帧
          (pos+vel+torque+kp+kd+linear_vel)，vel=0，由 linear_vel 控制插值速度

        MIT 控制律: tau = kp * (pos_ref - pos_cur) + kd * (vel_ref - vel_cur) + t_ref

        :param target_joints, 目标角度，6 个关节
        :param gripper_value, 夹爪值 [0, 1000]
        :param joint_format, 角度格式 'deg' / 'rad'
        :param speed, 运动速度 [0, 400]
        :param gripper_speed, 夹爪速度 [0, 400]
        :param wait_for_completion, 是否等待到达
        :param use_interpolation, MIT 模式是否使用线性轨迹插值（PV 模式忽略）
        :param kp, MIT 位置增益 [0, 500]（PV 模式忽略）。 None=使用默认值，float=广播至所有电机， List[float] 长度 6(仅关节) 或 7(含夹爪) 逐电机设置。
        :param kd, MIT 速度增益 [0, 5]（PV 模式忽略）。格式同 kp。
        :param torque, MIT 前馈力矩 (N·m)（PV 模式忽略）。 None=默认 0，float=广播，List[float] 逐电机设置。
        :param vel_ref, MIT 目标速度 (rad/s)（PV 模式忽略）。格式同 torque。
        :return, 是否成功到达目标
        """
        # 角度转换
        if target_joints is not None and joint_format == 'deg':
            target_joints = [math.radians(a) for a in target_joints]

        # 仅控制夹爪（无关节目标）→ 走专用夹爪路径
        if target_joints is None:
            if gripper_value is not None:
                return self._joint_ctrl.move_gripper(
                    gripper_value, speed=gripper_speed,
                    wait=wait_for_completion,
                    kp=kp, kd=kd, torque=torque, vel_ref=vel_ref,
                )
            return True  # 无目标，无操作

        mode = self._joint_ctrl.mode

        if mode == ControlMode.PV:
            return self._joint_ctrl.move_pv(
                target_joints=target_joints,
                speed=speed,
                gripper=gripper_value,
                gripper_speed=gripper_speed,
                wait=wait_for_completion,
            )
        else:
            return self._joint_ctrl.move_mit(
                target_joints=target_joints,
                speed=speed,
                gripper=gripper_value,
                gripper_speed=gripper_speed,
                wait=wait_for_completion,
                use_interpolation=use_interpolation,
                kp=kp,
                kd=kd,
                torque=torque,
                vel_ref=vel_ref,
            )

    def go_home(self, speed: float = 40, gripper_speed: float = 100) -> bool:
        """回零位"""
        return self._joint_ctrl.go_home(speed=speed)

    def set_gripper_target(
        self,
        command: Optional[str] = None,
        value: Optional[float] = None,
        wait_for_completion: bool = True,
    ) -> bool:
        """控制夹爪

        :param command, "open" / "close" / None（使用 value）
        :param value, 夹爪目标值 [0, 1000]
        :param wait_for_completion, 是否等待
        """
        if command == "open":
            value = 1000.0
        elif command == "close":
            value = 0.0
        if value is None:
            return False
        return self._joint_ctrl.move_gripper(value, wait=wait_for_completion)

    def set_linear_interpolation_velocity(
        self,
        velocity_rad_s: Union[float, List[float]] = 2.0,
    ) -> bool:
        """一次性设置固件线性轨迹插值速度.

        发送一帧只包含线性插值速度（0x06/addr=0x05）的数据帧。
        速度单位为 rad/s，可传标量广播到 7 个电机，或传长度 7 的列表。
        """
        return self._joint_ctrl.set_linear_interpolation_velocity(
            velocity_rad_s,
        )

    # ========== MIT 专用接口 ==========

    def send_mit_command(
        self,
        joint_params: List[MitParams],
        gripper: Optional[float] = None,
    ) -> None:
        """MIT 全参数直接发送（低延迟）

        每帧发送 6 地址 MIT 帧（线性速度填清零信号），不等待、不插值。
        调用方需自行维持高频发送（≥200Hz）。

        :param joint_params, 7 个电机的 MIT 参数
        :param gripper, 夹爪值，None 使用 joint_params[6].pos_ref
        """
        self._joint_ctrl.send_mit(joint_params, gripper=gripper)

    def initialize_mit_gains(
        self,
        kp: Optional[Union[float, List[float]]] = None,
        kd: Optional[Union[float, List[float]]] = None,
        torque: Optional[Union[float, List[float]]] = None,
        vel_ref: Optional[Union[float, List[float]]] = None,
        duration: float = 1.0,
        frequency_hz: float = 50.0,
        read_timeout: float = 1.0,
    ) -> bool:
        """@brief 在 MIT 运动开始前线性初始化 Kp/Kd。

        @details
        SDK 会读取当前机械臂 Kp/Kd，与目标增益逐电机比较，并保持当前
        关节/夹爪位置不变，将 Kp/Kd 线性过渡到目标值。建议在 demo、
        遥操作或产品流程第一次进入 MIT 运动前调用一次。

        @param kp 目标位置增益 [0, 500]。None 使用默认值；标量广播；
            长度 6/7 的列表表示逐电机设置。
        @param kd 目标速度增益 [0, 5]。格式同 kp。
        @param torque 预热帧使用的前馈力矩。None 表示 0。
        @param vel_ref 预热帧使用的速度参考。None 表示 0。
        @param duration 线性过渡时长，单位秒。
        @param frequency_hz 过渡帧发送频率，单位 Hz。
        @param read_timeout 读取当前 Kp/Kd 的最长等待时间，单位秒。
        @return True 表示初始化完成。
        """
        result = self._joint_ctrl.initialize_mit_gains(
            kp=kp,
            kd=kd,
            torque=torque,
            vel_ref=vel_ref,
            duration=duration,
            frequency_hz=frequency_hz,
            read_timeout=read_timeout,
        )
        if result:
            print("MIT 阻抗增益初始化结束", flush=True)
        return result

    # ========== 系统控制 ==========

    def torque_control(
        self, command: str, joints: Optional[List[int]] = None
    ) -> bool:
        """力矩开关（仅 MIT 模式）

        :param command, "off"=卸力, "on"=恢复
        :param joints, 关节索引列表，None=全部
        """
        if command == "off":
            return self._joint_ctrl.torque_off(joints)
        elif command == "on":
            return self._joint_ctrl.torque_on(joints)
        return False

    def enable_robot(self) -> bool:
        """使能机器人（发送 0x09 使能指令）"""
        return self._joint_ctrl.enable()

    def disable_robot(self) -> bool:
        """失能机器人"""
        return self._joint_ctrl.disable()

    def switch_mode(self, mode: str) -> bool:
        """切换控制模式（仅关节 M0-M5，夹爪 M6 固件锁定 MIT）

        流程: 失能 → 切换模式 → 使能。
        切到 MIT 后关节可自由活动；切回 PV 后关节锁定在当前位置。
        夹爪电机始终保持 MIT 模式，不受模式切换影响。

        :param mode, "pv" / "mit"
        """
        ctrl_mode = ControlMode.PV if mode.lower() == "pv" else ControlMode.MIT
        return self._joint_ctrl.switch_mode(ctrl_mode)

    def set_extended_polling(self, enabled: bool) -> None:
        """切换状态轮询模式

        基础模式（默认）: 仅查询角度、速度、力矩，兼容所有固件版本
        扩展模式: 额外查询 kp、kd、插补速度、温度，仅新固件支持

        :param enabled, True=扩展查询, False=基础查询
        """
        count = POLL_ADDR_EXTENDED if enabled else POLL_ADDR_BASIC
        self._device.set_poll_addr_count(count)
        logger.info(f"状态轮询模式: {'扩展 (7地址)' if enabled else '基础 (3地址)'}")

    def set_zero_position(self, mode: str = "strong") -> bool:
        """设置当前位姿为零位。

        :param mode, "strong"=强调零（默认）, "weak"=弱调零（固件 >= 1.0.6）
        """
        mode_key = mode.lower()
        if mode_key == "weak":
            version = self.get_firmware_version(timeout=1.0)
            if not supports_min_version(version, (1, 0, 6)):
                raise RobotStateError(
                    f"弱调零需要固件版本 >= v1.0.6，当前版本: {version or '未知'}"
                )
        return self._joint_ctrl.set_zero_position(mode=mode_key)

    def run_diagnostic(self, timeout: float = 3.0) -> DiagnosticResult:
        """运行自检并返回结构化结果。"""
        return _run_diagnostic(self._device, timeout=timeout)

    def print_diagnostic_response(self, result: DiagnosticResult) -> bool:
        """@brief 打印自检响应快照。

        @param result run_diagnostic 返回的结构化结果。
        @return True 表示响应正常，False 表示超时或错误响应。
        """
        return _print_diagnostic_response(result)

    def get_user_settings(self, timeout: float = 1.0) -> Optional[UserSettings]:
        """读取全部个性化设置。"""
        return _get_user_settings(self._device, timeout=timeout)

    def set_gripper_type(
        self,
        gripper_type,
        timeout: float = 3.0,
        readback: bool = True,
    ) -> bool:
        """写入夹爪类型配置。

        gripper_type 支持规范配置值 0/2、示例选项 10/40、字符串 small/large/50mm/100mm，
        以及 GripperType 枚举。
        """
        return _set_gripper_type(
            self._device,
            gripper_type,
            timeout=timeout,
            readback=readback,
        )

    def confirm_gripper_type(self, settings: UserSettings, expected_value: int) -> bool:
        """@brief 判断读回的个性化设置是否确认夹爪类型。

        @param settings 读回的个性化设置。
        @param expected_value 期望夹爪类型配置值。
        @return True 表示读回值与期望值一致。
        """
        if len(settings.values) < 2:
            return False
        actual_value = settings.values[1]
        return gripper_type_config_value(actual_value) == gripper_type_config_value(expected_value)

    def send_gripper_param_frame(self, frame, timeout: float = 1.0):
        """发送 0x17 夹爪夹持参数帧并等待响应。

        这是为低层夹爪参数 demo 保留的过渡 API，避免示例直接访问内部 Device。
        """
        return self._device.send_and_wait(frame, CMD_GRIPPER_PARAM, timeout=timeout)

    def get_gripper_params(
        self,
        mask: int = 0,
        aim: Union[str, int] = "follower",
        timeout: float = 1.0,
    ) -> Optional[GripperParamResult]:
        """@brief 通过 0x17 读取夹爪夹持参数。

        @param mask 参数掩码，0 表示读取全部参数。
        @param aim "follower"、"leader"、AIM_FOLLOWER 或 AIM_LEADER。
        @param timeout 响应超时时间，单位秒。
        @return 解析后的响应；超时时返回 None。
        """
        frame = make_read_gripper_params_frame(aim=aim, mask=mask)
        response = self.send_gripper_param_frame(frame, timeout=timeout)
        if response is None:
            return None
        return parse_gripper_params_response(response)

    def set_gripper_params(
        self,
        values: Mapping[Union[str, int], float],
        aim: Union[str, int] = "follower",
        timeout: float = 1.0,
        readback: bool = False,
        readback_delay: float = 0.2,
        gripper_type=None,
        save: bool = False,
    ) -> Optional[Union[GripperParamResult, Dict[str, GripperParamResult]]]:
        """@brief 通过 0x17 写入夹爪夹持参数。

        @param values 参数值，键可以是公开名称或协议掩码位。
        @param aim "follower"、"leader"、AIM_FOLLOWER 或 AIM_LEADER。
        @param timeout 响应超时时间，单位秒。
        @param readback 写入后是否读回全部夹爪参数。
        @param readback_delay 写入响应与读回之间的等待时间，单位秒。
        @param gripper_type 夹爪类型；None 表示按大小夹爪合并范围校验，
            "auto" 表示先读取用户设置中的夹爪类型再按精确范围校验。
        @param save True 时请求设备掉电保存当前完整夹爪参数配置。
        @return 写入响应；启用读回时返回包含 write/readback 的字典；超时时返回 None。
        @note save 默认为 False，避免用户误触掉电保存；写入参数仍会立即生效。
        @note save=True 时设备可能需要写入非易失存储，SDK 会使用至少 3 秒的响应等待时间。
        """
        resolved_gripper_type = gripper_type
        if isinstance(gripper_type, str) and gripper_type.lower() == "auto":
            settings = self.get_user_settings(timeout=timeout)
            resolved_gripper_type = settings.gripper_type if settings and settings.gripper_type is not None else None
        effective_timeout = max(timeout, 3.0) if save else timeout
        frame = make_write_gripper_params_frame(
            values,
            aim=aim,
            gripper_type=resolved_gripper_type,
            save=save,
        )
        response = self.send_gripper_param_frame(frame, timeout=effective_timeout)
        if response is None:
            return None
        write_result = parse_gripper_params_response(response)
        if not readback:
            return write_result

        if readback_delay > 0:
            time.sleep(readback_delay)
        readback_result = self.get_gripper_params(mask=0, aim=aim, timeout=effective_timeout)
        if readback_result is None:
            return {"write": write_result}
        return {"write": write_result, "readback": readback_result}

    # ========== 运动学 ==========

    def set_pose(
        self,
        target_pose,
        method: str = 'dls',
        execute: bool = True,
        speed: float = 7,
        **ik_params,
    ) -> Dict:
        """通过逆运动学移动到目标位姿

        :param target_pose, 目标位姿（4x4矩阵 / [x,y,z,qx,qy,qz,qw]）
        :param method, IK 方法
        :param execute, 是否执行运动
        :param speed, 运动速度
        :return, IK 结果字典
        """
        if self._robot_model is None:
            raise RobotStateError("未加载机器人模型")

        state = self._device.joint_state
        q_init = list(state.angles) if state else None

        result = kin_module.compute_inverse_kinematics(
            self._robot_model, target_pose, q_init=q_init,
            method=method, **ik_params,
        )

        if execute and result.get('success'):
            q = result['q'].tolist()
            self.set_robot_state(target_joints=q, joint_format='rad', speed=speed)
            result['motion_executed'] = True
        else:
            result['motion_executed'] = False

        return result

    # ========== 轨迹规划与执行 ==========

    def plan_joint_trajectory(
        self, waypoints, planner_type: str = 'b_spline', **kwargs
    ) -> Dict:
        """规划关节空间轨迹"""
        return plan_module.plan_joint_trajectory(waypoints, planner_type, **kwargs)

    def plan_cartesian_trajectory(self, waypoints, **kwargs) -> Dict:
        """规划笛卡尔空间轨迹"""
        return plan_module.plan_cartesian_trajectory(waypoints, **kwargs)

    def move_joint_trajectory(
        self, q_end, duration: float = 2.0, method: str = 'cubic', **kwargs
    ) -> bool:
        """执行平滑关节轨迹

        :param q_end, 目标关节角度 (rad)
        :param duration, 运动时长
        :param method, 插值方法
        """
        state = self._device.joint_state
        if state is None:
            return False
        q_start = list(state.angles) + [state.gripper]
        q_end_full = list(q_end) + [state.gripper] if len(q_end) == 6 else list(q_end)

        traj = plan_module.plan_joint_trajectory(
            [q_start, q_end_full],
            planner_type='multi_segment',
            duration=duration,
            segment_type=method,
        )
        if not traj.get('success'):
            return False

        return self._traj_executor.execute_pv(
            traj['timestamps'], traj['positions'], traj['velocities']
        )

    def execute_planned_joint_trajectory(
        self,
        traj,
        speed: float = 100.0,
        track_hz: Optional[float] = None,
    ):
        """@brief 执行已经规划好的关节轨迹。

        @param traj plan_joint_trajectory 返回的轨迹字典。
        @param speed 执行显示和控制使用的速度参数。
        @param track_hz 关节反馈记录频率；None 表示不记录。
        @return (ok, tracking)，ok 表示执行是否完成，tracking 为可选反馈记录。
        """
        return _execute_joint_trajectory(self, traj, speed, track_hz=track_hz)

    def save_joint_trajectory_csv(self, traj, output_dir=None):
        """@brief 保存轨迹 CSV 文件。

        @param traj 轨迹字典。
        @param output_dir 输出目录；None 时使用工具默认目录。
        @return 保存后的 CSV 路径。
        """
        return _save_trajectory_csv(traj, output_dir=output_dir)

    def default_trajectory_save_dir(self, anchor_file):
        """@brief 获取默认轨迹保存目录。

        @param anchor_file 用于定位项目根目录的文件路径。
        @return 默认保存目录。
        """
        return _resolve_default_traj_save_dir(anchor_file)

    def create_mapped_teleoperation(
        self,
        leader,
        frequency_hz: float = 100.0,
        follower_speed: float = 200.0,
        use_interpolation: bool = False,
        kp=None,
        kd=None,
        torque=None,
        vel_ref=None,
    ) -> Teleoperation:
        """@brief 创建 Alicia-D 到 Alicia-M 的 URDF 限位映射遥操作控制器。

        @param leader Alicia-D 示教臂实例。
        @param frequency_hz 控制循环频率。
        @param follower_speed 操作臂运动速度参数。
        @param use_interpolation MIT 模式是否启用线性轨迹插值。
        @param kp MIT 位置增益。
        @param kd MIT 速度增益。
        @param torque MIT 前馈力矩。
        @param vel_ref MIT 速度参考。
        @return Teleoperation 控制器实例。
        """
        return Teleoperation(
            leader=leader,
            follower=self,
            frequency_hz=frequency_hz,
            follower_speed=follower_speed,
            joint_mapper=convert_joints_rad_from_alicia_d_to_alicia_m,
            use_interpolation=use_interpolation,
            kp=kp,
            kd=kd,
            torque=torque,
            vel_ref=vel_ref,
        )

    def move_cartesian_linear(
        self, target_pose, duration: float = 2.0, **kwargs
    ) -> bool:
        """执行笛卡尔直线轨迹"""
        current_pose = self.get_pose()
        if current_pose is None:
            return False

        current_pos = current_pose['position'].tolist()
        current_quat = current_pose['quaternion_xyzw'].tolist()
        start = current_pos + current_quat

        traj = plan_module.plan_cartesian_trajectory(
            [start, target_pose], duration=duration, **kwargs
        )
        if not traj.get('success'):
            return False

        # 笛卡尔轨迹需要逐点 IK
        return self._execute_cartesian_traj(traj)

    def solve_ik_for_trajectory(
        self, target_poses, q_init=None, **kwargs
    ) -> Dict:
        """批量 IK 求解"""
        if self._robot_model is None:
            raise RobotStateError("未加载机器人模型")
        results = []
        q_current = q_init
        for pose in target_poses:
            r = kin_module.compute_inverse_kinematics(
                self._robot_model, pose, q_init=q_current, **kwargs
            )
            results.append(r)
            if r.get('success'):
                q_current = r['q'].tolist()
        return {'results': results, 'num_solved': sum(1 for r in results if r.get('success'))}

    def manual_record_waypoints(self):
        """@brief 交互式记录当前机械臂关节路点。

        @return 路点数组 [N, 6]，取消或路点不足时返回 None。
        """
        return _manual_record_waypoints(self)

    def manual_record_waypoints_with_torque_off(self):
        """@brief 切换到 MIT 并卸力后，交互式拖动示教记录路点。

        @return 路点数组 [N, 6]，取消或路点不足时返回 None。
        """
        return _manual_record_waypoints_with_torque_off(self)

    def auto_generate_waypoints(
        self,
        num_waypoints: int = 5,
        joint_scale: float = 0.6,
        use_current_joints: bool = False,
        seed: Optional[int] = 666,
    ):
        """@brief 自动生成关节空间路点。

        @param num_waypoints 路点数量，至少为 2。
        @param joint_scale robot_model.random_q 的缩放系数。
        @param use_current_joints 是否使用当前关节角作为首个路点。
        @param seed 随机种子。
        @return 路点数组 [N, 6]。
        """
        return _auto_generate_waypoints(
            self, self._robot_model,
            num_waypoints=num_waypoints,
            joint_scale=joint_scale,
            use_current_joints=use_current_joints,
            seed=seed,
        )

    def load_waypoints(self, path: str):
        """@brief 从文件加载关节路点。

        @param path 路点文件路径；为空时会交互式询问。
        @return (waypoints, meta) 或 None。
        """
        return _load_waypoints_interactive(path)

    def make_mapped_teleop_state_printer(self, frequency_hz: float):
        """@brief 创建遥操作映射状态打印回调。

        @param frequency_hz 遥操作循环频率。
        @return 可传入 Teleoperation.set_state_callback 的回调。
        """
        return _make_mapped_teleop_state_printer(frequency_hz)

    def print_compact_state(self) -> None:
        """@brief 按 pos/vel/tor 紧凑格式打印当前状态。"""
        _print_robot_state(self)

    # ========== 状态打印 ==========

    def print_state(self, continuous: bool = False, output_format: str = "deg") -> None:
        """打印当前状态"""
        _print_joint_state(self._device, continuous=continuous, output_format=output_format)

    # ========== 内部方法 ==========

    def _query_control_modes(self) -> Optional[List[dict]]:
        """读取各电机控制模式 (0x11 addr=0x0B)"""
        values = self._device.query_motor_params(MOTOR_PARAM_CTRL_MODE)
        if values is None:
            return None
        return [
            {"value": v, "name": CTRL_MODE_NAMES.get(v, f"未知({v})")}
            for v in values
        ]

    def _execute_cartesian_traj(self, traj: Dict) -> bool:
        """执行笛卡尔轨迹（逐点 IK → PV 轨迹执行）"""
        return plan_module.execute_cartesian_trajectory(
            self._robot_model, traj, self._device.joint_state, self._traj_executor
        )
