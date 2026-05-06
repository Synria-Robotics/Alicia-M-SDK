"""关节级运动控制

提供 PV 模式和 MIT 模式的独立控制方法，以及两种模式共用的通用方法。

设计原则:
- PV = 「发后不管」: 发送目标+速度，固件自行到达
- MIT = 「持续控制」: 每帧发送完整阻抗参数，SDK 持有控制权
- 模式切换流程: 失能 → 切换 → 使能，固件侧处理目标位置初始化
- 输入超限时优先裁剪+警告（而非直接拒绝）
"""

import time
from typing import List, Optional, Union

from ..hardware.device import Device
from ..hardware.constants import (
    CMD_TORQUE, CMD_ENABLE, CMD_MOTOR_PARAM, CMD_ZERO_RESET,
    CTRL_MODE_MIT, CTRL_MODE_PV,
    MOTOR_PARAM_CTRL_MODE,
    NUM_JOINTS, NUM_MOTORS,
    DEFAULT_KP_LARGE, DEFAULT_KD_LARGE,
    DEFAULT_KP_SMALL, DEFAULT_KD_SMALL,
    ZERO_RESET_STRONG,
)
from ..hardware.messages import (
    TorqueRequest, EnableRequest, MotorParamRequest, ZeroResetRequest,
)
from ..types.enums import ControlMode
from ..types.state import MitParams
from ..types.exceptions import RobotStateError, ValidationError
from ..utils.conversion import speed_user_to_firmware
from ..utils.validation import (
    validate_joint_angles, validate_speed, validate_gripper_value,
)
from ..utils.beauty_logger import logger

# MIT 默认增益查表（按电机索引）
# M0~M2 大关节, M3~M6 小关节（含夹爪）
_DEFAULT_KP = [DEFAULT_KP_LARGE] * 3 + [DEFAULT_KP_SMALL] * 4
_DEFAULT_KD = [DEFAULT_KD_LARGE] * 3 + [DEFAULT_KD_SMALL] * 4

# 关节限位默认值 (rad)
_DEFAULT_JOINT_LIMITS_LOWER = [-2.80, -2.80, -2.80, -2.80, -2.80, -2.80]
_DEFAULT_JOINT_LIMITS_UPPER = [2.80, 2.80, 2.80, 2.80, 2.80, 2.80]


def _fill_mit_defaults(motor_index: int, kp: Optional[float],
                       kd: Optional[float]) -> tuple:
    """填充 MIT 参数的默认 Kp/Kd

    策略:
    - None → 使用电机编号对应的安全默认值
    - 0 → 保持为 0（零力矩 / 卸力），不做填充
    - 具体数值 → 直接使用

    :param motor_index, 电机编号 (0~6)
    :param kp, 用户传入的 Kp，None 表示使用默认
    :param kd, 用户传入的 Kd，None 表示使用默认
    :return, (filled_kp, filled_kd) 填充后的增益值
    """
    filled_kp = _DEFAULT_KP[motor_index] if kp is None else kp
    filled_kd = _DEFAULT_KD[motor_index] if kd is None else kd
    return filled_kp, filled_kd


def _normalize_mit_param(
    value: Optional[Union[float, List[Optional[float]]]],
    name: str,
    default: Optional[float] = None,
) -> List[Optional[float]]:
    """标准化 MIT 参数为逐电机列表（长度 NUM_MOTORS）

    支持三种输入形式:
    - None: 所有电机使用默认值
    - 标量: 广播至所有电机（6 关节 + 夹爪）
    - 列表: 长度 6（仅关节，夹爪使用默认值）或 7（含夹爪），
            列表中的 None 元素表示该电机使用默认值

    :param value, MIT 参数值 — None / 标量 / 列表
    :param name, 参数名称（用于异常消息）
    :param default, 列表中 None 元素的替换值，None 表示保留 None（由下游填充）
    :return, 长度 NUM_MOTORS 的列表，None 元素表示使用默认值
    """
    if value is None:
        return [default] * NUM_MOTORS
    if isinstance(value, (int, float)):
        return [float(value)] * NUM_MOTORS
    value = list(value)
    if len(value) not in (NUM_JOINTS, NUM_MOTORS):
        raise ValidationError(
            f"{name} 列表长度错误: 期望 {NUM_JOINTS}(仅关节) 或 "
            f"{NUM_MOTORS}(含夹爪), 实际 {len(value)}"
        )
    result = [float(v) if v is not None else default for v in value]
    if len(result) == NUM_JOINTS:
        result.append(default)  # 夹爪使用默认值
    return result


class JointController:
    """关节级运动控制

    提供 PV 模式和 MIT 模式的独立控制方法，以及两种模式共用的通用方法。
    模式切换采用失能→切换→使能流程，固件侧处理目标位置初始化。

    :param device, 设备抽象层实例（通过 Device 发送指令，读取状态缓存）
    :param control_mode, 初始控制模式 ('pv' / 'mit')
    :param joint_limits_lower, 各关节角度下限 (rad)，None 使用默认值
    :param joint_limits_upper, 各关节角度上限 (rad)，None 使用默认值
    """

    def __init__(
        self,
        device: Device,
        control_mode: Union[str, ControlMode] = ControlMode.PV,
        joint_limits_lower: Optional[List[float]] = None,
        joint_limits_upper: Optional[List[float]] = None,
    ):
        self._device = device
        # 统一为 ControlMode 枚举
        if isinstance(control_mode, str):
            self._mode = ControlMode(control_mode.lower())
        else:
            self._mode = control_mode
        self._joint_limits_lower = joint_limits_lower or _DEFAULT_JOINT_LIMITS_LOWER
        self._joint_limits_upper = joint_limits_upper or _DEFAULT_JOINT_LIMITS_UPPER

    # ========== 属性 ==========

    @property
    def mode(self) -> ControlMode:
        """获取当前控制模式"""
        return self._mode

    @mode.setter
    def mode(self, value: ControlMode) -> None:
        """同步内部模式标记（仅更新状态，不发送切换指令）

        用于 connect() 阶段将 SDK 内部状态与固件实际模式对齐。
        如需实际切换固件模式，请使用 switch_mode()。
        """
        self._mode = value

    # ========== PV 模式 ==========

    def move_pv(
        self,
        target_joints: List[float],
        speed: float,
        gripper: Optional[float] = None,
        gripper_speed: Optional[float] = None,
        wait: bool = True,
        timeout: float = 15.0,
    ) -> bool:
        """PV 模式点位运动

        发送目标位置+速度，固件内部做加减速插值。
        速度为无量纲值，SDK 根据运动方向自动计算有符号速度。

        :param target_joints, 目标角度 (rad), 6 个关节
        :param speed, 运动速度，无量纲 [0, 400]，映射到 [0, 10] rad/s
        :param gripper, 夹爪目标值 [0, 1000]，None 表示不控制
        :param gripper_speed, 夹爪速度 [0, 400]
        :param wait, 是否阻塞等待到达
        :param timeout, 等待超时 (秒)
        :return, True=到达目标 / False=超时
        """
        # 参数校验与裁剪
        target_joints = list(target_joints)
        if len(target_joints) != NUM_JOINTS:
            raise ValidationError(
                f"目标关节数量错误: 期望 {NUM_JOINTS}, 实际 {len(target_joints)}"
            )
        target_joints = validate_joint_angles(
            target_joints, self._joint_limits_lower, self._joint_limits_upper
        )
        speeds = validate_speed(speed, NUM_JOINTS)

        # 读取当前位置
        current = self._get_current_angles()

        # 计算有符号速度
        velocities = self._compute_signed_velocities(
            target_joints, current, speeds
        )
        positions = list(target_joints)

        # 夹爪处理
        if gripper is not None:
            gripper = validate_gripper_value(gripper)
            positions.append(gripper)
            g_speed = gripper_speed if gripper_speed is not None else speed
            g_speeds = validate_speed(g_speed, 1)
            # 夹爪速度方向
            state = self._device.joint_state
            current_gripper = state.gripper if state else 0.0
            g_sign = 1.0 if gripper > current_gripper else -1.0
            g_mag = speed_user_to_firmware(g_speeds[0])
            velocities.append(g_sign * g_mag)
        else:
            # 夹爪保持不动: 位置=当前, 速度=0
            state = self._device.joint_state
            current_gripper = state.gripper if state else 0.0
            positions.append(current_gripper)
            velocities.append(0.0)

        # 发送 PV 帧
        self._device.send_pv(self._device.aim, positions, velocities)
        logger.debug(f"PV 帧已发送: pos={positions}, vel={velocities}")

        if wait:
            t0 = time.perf_counter()
            joint_ok = self._wait_for_target(target_joints, timeout=timeout)
            if gripper is not None:
                # 关节与夹爪并行运动，用剩余超时（至少 1s）等待夹爪
                remaining = max(timeout - (time.perf_counter() - t0), 1.0)
                gripper_ok = self._wait_for_gripper(gripper, timeout=remaining)
                return joint_ok and gripper_ok
            return joint_ok
        return True

    # ========== MIT 模式 ==========

    def send_mit(
        self,
        joint_params: List[MitParams],
        gripper: Optional[float] = None,
    ) -> None:
        """MIT 全参数直接发送（标准 MIT 接口）

        发送一帧 6 地址 MIT 帧（线性速度填清零信号）。不做等待，不做插值。
        用于遥操作、力控、轨迹回放等需要持续高频发帧的场景。

        :param joint_params, 7 个电机的 MIT 参数 (pos, vel, torque, kp, kd)
        :param gripper, 夹爪目标值 [0, 1000]，为 None 时使用 joint_params[6].pos_ref
        """
        if len(joint_params) != NUM_MOTORS:
            raise ValidationError(
                f"MIT 参数数量错误: 期望 {NUM_MOTORS}, 实际 {len(joint_params)}"
            )

        # 填充默认 Kp/Kd
        filled_params = []
        for i, p in enumerate(joint_params):
            kp, kd = _fill_mit_defaults(i, p.kp, p.kd)
            filled_params.append(MitParams(
                pos_ref=p.pos_ref,
                vel_ref=p.vel_ref,
                t_ref=p.t_ref,
                kp=kp,
                kd=kd,
            ))

        # 如果指定了夹爪值，覆盖最后一个电机的 pos_ref
        if gripper is not None:
            gripper = validate_gripper_value(gripper)
            last = filled_params[NUM_JOINTS]
            filled_params[NUM_JOINTS] = MitParams(
                pos_ref=gripper,
                vel_ref=last.vel_ref,
                t_ref=last.t_ref,
                kp=last.kp,
                kd=last.kd,
            )

        self._device.send_mit(self._device.aim, filled_params)
        logger.debug(f"MIT 帧已发送: {filled_params}")

    def move_mit(
        self,
        target_joints: List[float],
        speed: float,
        gripper: Optional[float] = None,
        gripper_speed: Optional[float] = None,
        wait: bool = True,
        timeout: float = 15.0,
        use_interpolation: bool = True,
        kp: Optional[Union[float, List[float]]] = None,
        kd: Optional[Union[float, List[float]]] = None,
        torque: Optional[Union[float, List[float]]] = None,
        vel_ref: Optional[Union[float, List[float]]] = None,
    ) -> bool:
        """MIT 模式点位运动

        始终发送 6 地址帧 (pos+vel+torque+kp+kd+linear_vel)，支持两种运动方式:
        - 线性轨迹插值 (use_interpolation=True, 默认):
          linear_vel 为用户指定的插值速度，固件按该速度线性到达目标。
        - 直接 PD 控制 (use_interpolation=False):
          linear_vel 填充清零信号 (0xFFFF)，禁用固件插值，
          由 PD 控制器驱动关节趋近目标，发送后立即返回（不等待到达）。

        MIT 控制律: tau = kp * (pos_ref - pos_cur) + kd * (vel_ref - vel_cur) + t_ref

        :param target_joints, 目标角度 (rad), 6 个关节
        :param speed, 运动速度 [0, 400]，仅 use_interpolation=True 时生效
        :param gripper, 夹爪目标值 [0, 1000]，None 表示不控制
        :param gripper_speed, 夹爪速度 [0, 400]，仅 use_interpolation=True 时生效
        :param wait, 是否阻塞等待到达，仅 use_interpolation=True 时生效
        :param timeout, 等待超时 (秒)
        :param use_interpolation, 是否使用线性轨迹插值
        :param kp, 位置增益 [0, 500]。None=使用默认值， float=广播至所有电机，List[float] 长度 6 或 7 逐电机设置。
        :param kd, 速度增益 [0, 5]。格式同 kp。
        :param torque, 前馈力矩 (N·m)。None=默认 0， float=广播，List[float] 逐电机设置。
        :param vel_ref, 目标速度 (rad/s)。格式同 torque。
        :return, True=到达目标 / False=超时
        """
        # 参数校验
        target_joints = list(target_joints)
        if len(target_joints) != NUM_JOINTS:
            raise ValidationError(
                f"目标关节数量错误: 期望 {NUM_JOINTS}, 实际 {len(target_joints)}"
            )
        target_joints = validate_joint_angles(
            target_joints, self._joint_limits_lower, self._joint_limits_upper
        )
        if gripper is not None:
            gripper = validate_gripper_value(gripper)

        # 步骤1: 计算线性轨迹插值速度（仅插值模式）
        # 所有电机始终设正值插值速度，禁止设 0（固件将 0 视为"不插值、直接跳到
        # 目标位置"，在阶段切换时因 PD 稳态误差引起的微小位置差会导致瞬间跳变抖动）
        linear_vels = None
        if use_interpolation:
            speeds = validate_speed(speed, NUM_JOINTS)
            linear_vels = [speed_user_to_firmware(speeds[i])
                           for i in range(NUM_JOINTS)]
            # 夹爪线性速度
            g_speed = gripper_speed if gripper_speed is not None else speed
            g_speeds = validate_speed(g_speed, 1)
            linear_vels.append(speed_user_to_firmware(g_speeds[0]))

        # 步骤2: 标准化 MIT 参数为逐电机列表
        # kp/kd: None 保留，由 _fill_mit_defaults 按电机编号填充安全默认值
        # torque/vel_ref: None → 0.0，无需按电机区分
        kps = _normalize_mit_param(kp, "kp")
        kds = _normalize_mit_param(kd, "kd")
        torques = _normalize_mit_param(torque, "torque", default=0.0)
        vel_refs = _normalize_mit_param(vel_ref, "vel_ref", default=0.0)

        # 步骤3: 构建 MIT 帧（始终 6 地址）
        #   插值模式: 线性速度为用户指定值
        #   直接 PD:  线性速度填充清零信号 (0xFFFF)
        mit_params = []
        for i in range(NUM_JOINTS):
            filled_kp, filled_kd = _fill_mit_defaults(i, kps[i], kds[i])
            mit_params.append(MitParams(
                pos_ref=target_joints[i],
                vel_ref=vel_refs[i],
                t_ref=torques[i],
                kp=filled_kp,
                kd=filled_kd,
            ))
        # 夹爪电机
        gripper_target = gripper if gripper is not None else (
            self._device.joint_state.gripper
            if self._device.joint_state else 0.0
        )
        gripper_kp, gripper_kd = _fill_mit_defaults(
            NUM_JOINTS, kps[NUM_JOINTS], kds[NUM_JOINTS],
        )
        mit_params.append(MitParams(
            pos_ref=gripper_target,
            vel_ref=vel_refs[NUM_JOINTS],
            t_ref=torques[NUM_JOINTS],
            kp=gripper_kp,
            kd=gripper_kd,
        ))
        self._device.send_mit(
            self._device.aim, mit_params, linear_velocities=linear_vels,
        )
        logger.debug(
            f"MIT 帧已发送: target={target_joints}, interpolation={use_interpolation}"
        )

        # 步骤4: 等待到达（仅插值模式；直接 PD 控制不精确收敛，不等待）
        # MIT PD 控制器存在稳态误差，容差需大于 PV 模式
        reached = True
        if use_interpolation and wait:
            t0 = time.perf_counter()
            reached = self._wait_for_target(
                target_joints, tolerance=0.15, timeout=timeout,
            )
            if gripper is not None:
                remaining = max(timeout - (time.perf_counter() - t0), 1.0)
                gripper_ok = self._wait_for_gripper(gripper, timeout=remaining)
                reached = reached and gripper_ok

        return reached

    # ========== 通用方法 ==========

    def go_home(self, speed: float = 40.0) -> bool:
        """回零位（自动适配当前模式）

        :param speed, 运动速度 [0, 400]
        :return, True=到达零位 / False=超时
        """
        zero_joints = [0.0] * NUM_JOINTS
        if self._mode == ControlMode.PV:
            return self.move_pv(zero_joints, speed=speed, gripper=None)
        else:
            return self.move_mit(zero_joints, speed=speed, gripper=None)

    def move_gripper(
        self,
        value: float,
        speed: float = 100.0,
        wait: bool = True,
        timeout: float = 5.0,
        kp: Optional[Union[float, List[float]]] = None,
        kd: Optional[Union[float, List[float]]] = None,
        torque: Optional[Union[float, List[float]]] = None,
        vel_ref: Optional[Union[float, List[float]]] = None,
    ) -> bool:
        """控制夹爪

        夹爪电机固件锁定为 MIT 模式，始终通过 MIT 全参数帧控制。
        关节保持当前位置不动，夹爪通过 6 地址单帧线性轨迹插值到达目标。

        注意: 始终发送 6 地址帧，不使用插值时线性速度填充清零信号 (0xFFFF)。

        :param value, 夹爪目标值 [0=关闭, 1000=打开]
        :param speed, 夹爪速度 [0, 400]
        :param wait, 是否阻塞等待到达
        :param timeout, 等待超时 (秒)
        :param kp, 位置增益 [0, 500]。None=使用默认值， float=广播至所有电机，List[float] 长度 6 或 7 逐电机设置。
        :param kd, 速度增益 [0, 5]。格式同 kp。
        :param torque, 前馈力矩 (N·m)。None=默认 0， float=广播，List[float] 逐电机设置。
        :param vel_ref, 目标速度 (rad/s)。格式同 torque。
        :return, True=到达目标 / False=超时
        """
        value = validate_gripper_value(value)

        # 标准化 MIT 参数为逐电机列表
        kps = _normalize_mit_param(kp, "kp")
        kds = _normalize_mit_param(kd, "kd")
        torques = _normalize_mit_param(torque, "torque", default=0.0)
        vel_refs = _normalize_mit_param(vel_ref, "vel_ref", default=0.0)

        # 读取当前关节位置，保持不变
        current = self._get_current_angles()

        # 夹爪固件锁定 MIT，始终发送 MIT 全参数帧
        # 关节部分保持当前位置，torque/vel_ref 强制归零以防意外运动
        mit_params = []
        for i in range(NUM_JOINTS):
            filled_kp, filled_kd = _fill_mit_defaults(i, kps[i], kds[i])
            mit_params.append(MitParams(
                pos_ref=current[i],
                vel_ref=0.0,
                t_ref=0.0,
                kp=filled_kp,
                kd=filled_kd,
            ))
        # 夹爪电机：使用用户指定的全部 MIT 参数
        gripper_kp, gripper_kd = _fill_mit_defaults(
            NUM_JOINTS, kps[NUM_JOINTS], kds[NUM_JOINTS],
        )
        mit_params.append(MitParams(
            pos_ref=value,
            vel_ref=vel_refs[NUM_JOINTS],
            t_ref=torques[NUM_JOINTS],
            kp=gripper_kp,
            kd=gripper_kd,
        ))

        # 线性轨迹插值：所有电机始终设正值速度，禁止设 0（防止阶段切换抖动）
        g_speeds = validate_speed(speed, 1)
        g_vel = speed_user_to_firmware(g_speeds[0])
        linear_vels = [g_vel] * NUM_JOINTS + [g_vel]

        self._device.send_mit(
            self._device.aim, mit_params, linear_velocities=linear_vels,
        )

        if wait:
            return self._wait_for_gripper(value, timeout=timeout)
        return True

    def _wait_for_gripper(
        self,
        target: float,
        tolerance: float = 50.0,
        timeout: float = 5.0,
    ) -> bool:
        """轮询状态缓存等待夹爪到达目标

        :param target, 夹爪目标值 [0, 1000]
        :param tolerance, 到达判定阈值
        :param timeout, 超时时间 (秒)
        :return, True=到达目标 / False=超时
        """
        POLL_INTERVAL = 0.02
        deadline = time.perf_counter() + timeout

        while time.perf_counter() < deadline:
            state = self._device.joint_state
            if state is not None:
                if abs(state.gripper - target) < tolerance:
                    return True
            time.sleep(POLL_INTERVAL)

        logger.warning(f"等待夹爪到达目标超时 ({timeout:.1f}s)")
        return False

    # ========== 力矩与使能 ==========

    def torque_off(self, joints: Optional[List[int]] = None) -> bool:
        """卸载力矩（MIT 零阻抗实现）

        通过发送 MIT 全参数帧，将目标关节 kp/kd/t_ref/vel_ref 置零，
        并保持当前位置为 pos_ref，达到可拖动的零阻抗效果。

        :param joints, 需要卸力的关节索引列表 (0~5)，None 表示全部关节（含夹爪）
        :return, True=指令发送成功
        """
        if self._mode != ControlMode.MIT:
            self.switch_mode(ControlMode.MIT)

        state = self._device.joint_state
        if state is None:
            raise RobotStateError("无状态缓存，无法执行 torque_off")

        q_cur = list(state.angles)
        g_cur = state.gripper
        selected = set(range(NUM_MOTORS)) if joints is None else set(int(j) for j in joints)
        # If caller requests all joints (0~5), include gripper as well for full zero-impedance.
        if joints is not None and all(j in selected for j in range(NUM_JOINTS)):
            selected.add(NUM_JOINTS)

        kps = []
        kds = []
        torques = []
        vel_refs = []
        for i in range(NUM_MOTORS):
            if i in selected:
                kps.append(0.0)
                kds.append(0.0)
            else:
                kps.append(None)
                kds.append(None)
            torques.append(0.0)
            vel_refs.append(0.0)

        self.move_mit(
            target_joints=q_cur,
            speed=5.0,
            gripper=g_cur,
            wait=False,
            use_interpolation=False,
            kp=kps,
            kd=kds,
            torque=torques,
            vel_ref=vel_refs,
        )
        logger.info(f"卸力完成(MIT零阻抗): joints={joints or '全部'}")
        return True

    def torque_on(self, joints: Optional[List[int]] = None) -> bool:
        """恢复力矩（MIT 默认阻抗实现）

        通过发送 MIT 全参数帧，将目标关节 kp/kd 恢复为默认值，
        并保持当前位置为 pos_ref，避免恢复时产生突跳。

        :param joints, 需要恢复力矩的关节索引列表，None 表示全部（含夹爪）
        :return, True=成功恢复
        """
        if self._mode != ControlMode.MIT:
            self.switch_mode(ControlMode.MIT)

        state = self._device.joint_state
        if state is None:
            raise RobotStateError("无状态缓存，无法执行 torque_on")

        q_cur = list(state.angles)
        g_cur = state.gripper
        selected = set(range(NUM_MOTORS)) if joints is None else set(int(j) for j in joints)
        if joints is not None and all(j in selected for j in range(NUM_JOINTS)):
            selected.add(NUM_JOINTS)

        kps = []
        kds = []
        torques = []
        vel_refs = []
        for i in range(NUM_MOTORS):
            if i in selected:
                kps.append(None)
                kds.append(None)
            else:
                kps.append(0.0)
                kds.append(0.0)
            torques.append(0.0)
            vel_refs.append(0.0)

        self.move_mit(
            target_joints=q_cur,
            speed=5.0,
            gripper=g_cur,
            wait=False,
            use_interpolation=False,
            kp=kps,
            kd=kds,
            torque=torques,
            vel_ref=vel_refs,
        )
        logger.info(f"力矩恢复完成(MIT默认阻抗): joints={joints or '全部'}")
        return True

    def enable(self) -> bool:
        """使能机器人（发送 0x09 指令，任何模式可用）

        :return, True=使能成功
        """
        frame = self._device.codec.encode_enable_request(EnableRequest(
            aim=self._device.aim,
            enable=True,
        ))
        self._device.send_frame(frame)
        time.sleep(0.1)  # 等待固件处理使能

        logger.info("使能完成")
        return True

    def disable(self) -> bool:
        """失能机器人（发送 0x09 指令，任何模式可用）

        :return, True=失能成功
        """
        frame = self._device.codec.encode_enable_request(EnableRequest(
            aim=self._device.aim,
            enable=False,
        ))
        self._device.send_frame(frame)
        time.sleep(0.1)  # 等待固件处理失能

        logger.info("失能完成")
        return True

    def switch_mode(self, mode: Union[str, ControlMode]) -> bool:
        """切换控制模式（发送 0x11 指令, addr=0x0B）

        仅切换关节电机 M0-M5，夹爪电机 M6 固件锁定为 MIT 模式不可切换。

        切换流程:
        1. 失能所有电机
        2. 发送模式切换指令
        3. 读回验证（失败重试一次）
        4. 使能所有电机

        :param mode, 目标控制模式 ('pv' / 'mit' 或 ControlMode 枚举)
        :return, True=切换成功
        """
        if isinstance(mode, str):
            mode = ControlMode(mode.lower())

        if mode == self._mode:
            logger.debug(f"已处于 {mode.value} 模式，无需切换")
            return True

        logger.warning(f"即将切换控制模式: {self._mode.value} → {mode.value}")

        # 步骤1: 失能所有电机
        self.disable()

        # 暂停轮询，避免与模式切换指令交叉
        self._device.pause_polling()

        try:
            ctrl_mode_value = CTRL_MODE_MIT if mode == ControlMode.MIT else CTRL_MODE_PV

            # 清空串口缓冲区，防止残留帧干扰模式切换验证
            self._device.flush()

            # 步骤2: 发送模式切换指令（M0-M5，夹爪 M6 固件锁定 MIT 不参与）
            self._send_mode_switch_command(ctrl_mode_value)
            time.sleep(0.5)

            # 步骤3: 读回验证
            if not self._verify_mode_switch(ctrl_mode_value):
                logger.warning("模式切换未完全生效，重试一次")
                self._send_mode_switch_command(ctrl_mode_value)
                time.sleep(0.5)

            # 更新内部模式
            old_mode = self._mode
            self._mode = mode
        finally:
            self._device.resume_polling()

        # 步骤4: 使能所有电机，等待轮询获取使能后的新鲜状态
        self.enable()
        time.sleep(0.2)

        logger.info(f"控制模式已切换: {old_mode.value} → {mode.value}")
        return True

    def _send_mode_switch_command(self, ctrl_mode_value: int) -> None:
        """发送模式切换指令（仅切换关节 M0-M5，夹爪 M6 固件锁定 MIT）"""
        frame = self._device.codec.encode_motor_param_request(MotorParamRequest(
            aim=self._device.aim,
            start_motor=1,
            motor_count=NUM_JOINTS,
            param_addr=MOTOR_PARAM_CTRL_MODE,
            param_value=ctrl_mode_value,
        ))
        self._device.send_frame(frame)

    def _verify_mode_switch(self, expected_value: int) -> bool:
        """读回验证关节电机 M0-M5 的控制模式是否与预期一致（夹爪 M6 固件锁定 MIT，不参与验证）

        内部重试多次查询，容忍固件模式切换后的短暂响应延迟。
        """
        for attempt in range(3):
            # 每次查询前清空残留帧，避免旧响应干扰匹配
            self._device.flush()
            values = self._device.query_motor_params(MOTOR_PARAM_CTRL_MODE, timeout=2.0)
            if values is None or len(values) < NUM_JOINTS:
                logger.debug(f"模式验证第 {attempt + 1} 次查询未获得有效响应")
                time.sleep(0.5)
                continue
            # 检查所有关节电机是否已切换
            mismatched = [i for i in range(NUM_JOINTS) if values[i] != expected_value]
            if not mismatched:
                return True
            for i in mismatched:
                logger.debug(
                    f"电机 M{i} 模式未切换: 期望 0x{expected_value:02X}, 实际 0x{values[i]:02X}"
                )
            time.sleep(0.5)

        logger.warning("模式验证失败: 多次查询均未确认切换成功")
        return False

    def set_zero_position(self) -> bool:
        """设置当前位姿为零位（发送 0x03 指令）

        :return, True=设置成功
        """
        frame = self._device.codec.encode_zero_reset(ZeroResetRequest(
            aim=self._device.aim,
            start_joint=0,
            joint_count=NUM_MOTORS,
            reset_mode=ZERO_RESET_STRONG,
        ))
        self._device.send_frame(frame)
        time.sleep(0.1)  # 等待固件处理零位标定

        logger.info("强调零完成")
        return True

    # ========== 等待到达 ==========

    def _wait_for_target(
        self,
        target: List[float],
        tolerance: float = 0.05,
        timeout: float = 10.0,
    ) -> bool:
        """轮询状态缓存等待到达目标位置

        实现策略: 固定间隔轮询（非 busy-wait），读状态缓存比较误差。
        不阻塞串口通信 — 只读内存中的 StateCache，不发 I/O。

        :param target, 目标关节角度 (rad), 6 个关节
        :param tolerance, 到达判定阈值 (rad)
        :param timeout, 超时时间 (秒)
        :return, True=到达目标 / False=超时
        """
        POLL_INTERVAL = 0.01  # 10ms 轮询间隔
        deadline = time.perf_counter() + timeout

        while time.perf_counter() < deadline:
            state = self._device.joint_state
            if state is not None and len(state.angles) >= len(target):
                if all(
                    abs(state.angles[i] - target[i]) < tolerance
                    for i in range(len(target))
                ):
                    return True
            time.sleep(POLL_INTERVAL)

        logger.warning(f"等待到达目标超时 ({timeout:.1f}s)")
        return False

    # ========== 内部辅助方法 ==========

    def _get_current_angles(self) -> List[float]:
        """读取当前关节角度（从状态缓存）

        :return, 6 个关节的当前角度 (rad)
        """
        state = self._device.joint_state
        if state is None:
            raise RobotStateError(
                "无法获取当前关节状态: 状态缓存为空（串口未连接或尚未收到状态数据）"
            )
        return list(state.angles[:NUM_JOINTS])

    def _compute_signed_velocities(
        self,
        target: List[float],
        current: List[float],
        speeds: List[float],
    ) -> List[float]:
        """计算有符号速度列表（仅关节，不含夹爪）

        根据目标与当前位置的差值确定运动方向，
        用户速度 [0, 400] → 固件速度 [0, 10] rad/s → 加方向符号。

        :param target, 目标角度 (rad), 6 个关节
        :param current, 当前角度 (rad), 6 个关节
        :param speeds, 用户速度列表 [0, 400], 6 个关节
        :return, 有符号速度列表 (rad/s), 6 个元素
        """
        velocities = []
        for i in range(NUM_JOINTS):
            magnitude = speed_user_to_firmware(speeds[i])
            diff = target[i] - current[i]
            if abs(diff) < 1e-6:
                sign = 1.0
            else:
                sign = 1.0 if diff > 0 else -1.0
            velocities.append(sign * magnitude)
        return velocities

