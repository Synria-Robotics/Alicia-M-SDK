"""关节级运动控制

提供 PV 模式和 MIT 模式的独立控制方法，以及两种模式共用的通用方法。

设计原则:
- PV = 「发后不管」: 发送目标+速度，固件自行到达
- MIT = 「持续控制」: 每帧发送完整阻抗参数，SDK 持有控制权
- 所有可能导致关节突变的操作，内置「位置锁定首帧」安全序列
- 输入超限时优先裁剪+警告（而非直接拒绝）
"""

import logging
import time
from typing import List, Optional, Union

from ..hardware.device import Device
from ..protocol.constants import (
    CMD_TORQUE, CMD_ENABLE, CMD_MOTOR_PARAM, CMD_ZERO_RESET,
    CTRL_MODE_MIT, CTRL_MODE_PV,
    MOTOR_PARAM_CTRL_MODE,
    NUM_JOINTS, NUM_MOTORS,
    DEFAULT_KP_LARGE, DEFAULT_KD_LARGE,
    DEFAULT_KP_SMALL, DEFAULT_KD_SMALL,
)
from ..protocol.messages import (
    TorqueRequest, EnableRequest, MotorParamRequest, ZeroResetRequest,
)
from ..types.enums import ControlMode
from ..types.state import JointState, MitParams
from ..types.exceptions import RobotStateError, ValidationError
from ..utils.conversion import speed_user_to_firmware
from ..utils.validation import (
    validate_joint_angles, validate_speed, validate_gripper_value,
)

logger = logging.getLogger(__name__)

# 方向映射: 电机正方向与关节正方向的对应关系
DIRECTION_MAP = [1, -1, 1, 1, -1, 1, 1]

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

    Args:
        motor_index: 电机编号 (0~6)
        kp: 用户传入的 Kp，None 表示使用默认
        kd: 用户传入的 Kd，None 表示使用默认

    Returns:
        (filled_kp, filled_kd) 填充后的增益值
    """
    filled_kp = _DEFAULT_KP[motor_index] if kp is None else kp
    filled_kd = _DEFAULT_KD[motor_index] if kd is None else kd
    return filled_kp, filled_kd


class JointController:
    """关节级运动控制

    提供 PV 模式和 MIT 模式的独立控制方法，以及两种模式共用的通用方法。
    所有安全序列由本类内部自动执行，用户无需手动处理。

    Args:
        device: 设备抽象层实例（通过 Device 发送指令，读取状态缓存）
        control_mode: 初始控制模式 ('pv' / 'mit')
        joint_limits_lower: 各关节角度下限 (rad)，None 使用默认值
        joint_limits_upper: 各关节角度上限 (rad)，None 使用默认值
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

    # ========== PV 模式 ==========

    def move_pv(
        self,
        target_joints: List[float],
        speed: float,
        gripper: Optional[float] = None,
        gripper_speed: Optional[float] = None,
        wait: bool = True,
        timeout: float = 10.0,
    ) -> bool:
        """PV 模式点位运动

        发送目标位置+速度，固件内部做加减速插值。
        速度为无量纲值，SDK 根据运动方向自动计算有符号速度。

        Args:
            target_joints: 目标角度 (rad), 6 个关节
            speed: 运动速度，无量纲 [0, 400]，映射到 [0, 10] rad/s
            gripper: 夹爪目标值 [0, 1000]，None 表示不控制
            gripper_speed: 夹爪速度 [0, 400]
            wait: 是否阻塞等待到达
            timeout: 等待超时 (秒)

        Returns:
            True=到达目标 / False=超时
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

        # 读取当前位置用于计算速度方向
        current = self._get_current_angles()

        # 构建 7 电机的位置和速度列表（含夹爪）
        positions = self._apply_direction_map(target_joints)
        velocities = self._compute_signed_velocities(
            target_joints, current, speeds
        )

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
        logger.debug("PV 帧已发送: pos=%s, vel=%s", positions, velocities)

        # 等待到达
        if wait:
            return self._wait_for_target(target_joints, timeout=timeout)
        return True

    # ========== MIT 模式 ==========

    def send_mit(
        self,
        joint_params: List[MitParams],
        gripper: Optional[float] = None,
    ) -> None:
        """MIT 全参数直接发送（标准 MIT 接口）

        发送一帧包含全部 5 参数的 MIT 帧。不做等待，不做插值。
        用于遥操作、力控、轨迹回放等需要持续高频发帧的场景。

        Args:
            joint_params: 7 个电机的 MIT 参数 (pos, vel, torque, kp, kd)
            gripper: 夹爪目标值 [0, 1000]，为 None 时使用 joint_params[6].pos_ref
        """
        if len(joint_params) != NUM_MOTORS:
            raise ValidationError(
                f"MIT 参数数量错误: 期望 {NUM_MOTORS}, 实际 {len(joint_params)}"
            )

        # 填充默认 Kp/Kd 并应用方向映射
        filled_params = []
        for i, p in enumerate(joint_params):
            kp, kd = _fill_mit_defaults(i, p.kp, p.kd)
            # 对关节（非夹爪）应用方向映射
            pos = p.pos_ref * DIRECTION_MAP[i] if i < NUM_JOINTS else p.pos_ref
            filled_params.append(MitParams(
                pos_ref=pos,
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
        logger.debug("MIT 帧已发送: %s", filled_params)

    def move_mit(
        self,
        target_joints: List[float],
        speed: float,
        gripper: Optional[float] = None,
        gripper_speed: Optional[float] = None,
        wait: bool = True,
        timeout: float = 10.0,
    ) -> bool:
        """MIT 模式点位运动（简易接口，通过线性轨迹速度实现）

        在 MIT 模式下实现类似 PV 的「发后等待」体验:
        1. 设定线性轨迹速度 (addr=0x05)
        2. 发送全参数 MIT 帧 (pos=目标, vel=0, t=0, kp=默认, kd=默认)
        3. 等待到达目标
        4. 清零线性轨迹速度（防止残留）

        Args:
            target_joints: 目标角度 (rad), 6 个关节
            speed: 运动速度，无量纲 [0, 400]
            gripper: 夹爪目标值 [0, 1000]，None 表示不控制
            gripper_speed: 夹爪速度 [0, 400]
            wait: 是否阻塞等待到达
            timeout: 等待超时 (秒)

        Returns:
            True=到达目标 / False=超时
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
        speeds = validate_speed(speed, NUM_JOINTS)

        # 读取当前位置用于计算速度方向
        current = self._get_current_angles()

        # 步骤1: 计算有符号线性轨迹速度并发送
        linear_vels = self._compute_signed_velocities(
            target_joints, current, speeds
        )
        # 夹爪线性速度
        if gripper is not None:
            gripper = validate_gripper_value(gripper)
            state = self._device.joint_state
            current_gripper = state.gripper if state else 0.0
            g_speed = gripper_speed if gripper_speed is not None else speed
            g_speeds = validate_speed(g_speed, 1)
            g_sign = 1.0 if gripper > current_gripper else -1.0
            g_mag = speed_user_to_firmware(g_speeds[0])
            linear_vels.append(g_sign * g_mag)
        else:
            linear_vels.append(0.0)

        self._device.send_linear_velocity(self._device.aim, linear_vels)

        # 步骤2: 构建全参数 MIT 帧 (pos=目标, vel=0, t=0, kp=默认, kd=默认)
        mit_params = []
        for i in range(NUM_JOINTS):
            kp, kd = _fill_mit_defaults(i, None, None)
            mit_params.append(MitParams(
                pos_ref=target_joints[i] * DIRECTION_MAP[i],
                vel_ref=0.0,
                t_ref=0.0,
                kp=kp,
                kd=kd,
            ))
        # 夹爪电机
        gripper_target = gripper if gripper is not None else (
            self._device.joint_state.gripper
            if self._device.joint_state else 0.0
        )
        gripper_kp, gripper_kd = _fill_mit_defaults(NUM_JOINTS, None, None)
        mit_params.append(MitParams(
            pos_ref=gripper_target,
            vel_ref=0.0,
            t_ref=0.0,
            kp=gripper_kp,
            kd=gripper_kd,
        ))
        self._device.send_mit(self._device.aim, mit_params)
        logger.debug("MIT 线性速度运动已启动: target=%s, vel=%s",
                     target_joints, linear_vels)

        # 步骤3: 等待到达
        reached = True
        if wait:
            reached = self._wait_for_target(target_joints, timeout=timeout)

        # 步骤4: 清零线性轨迹速度（防止残留影响后续运动）
        zero_vels = [0.0] * NUM_MOTORS
        self._device.send_linear_velocity(self._device.aim, zero_vels)
        logger.debug("MIT 线性速度已清零")

        return reached

    # ========== 通用方法 ==========

    def go_home(self, speed: float = 15.0) -> bool:
        """回零位（自动适配当前模式）

        Args:
            speed: 运动速度 [0, 400]

        Returns:
            True=到达零位 / False=超时
        """
        zero_joints = [0.0] * NUM_JOINTS
        if self._mode == ControlMode.PV:
            return self.move_pv(zero_joints, speed=speed, gripper=None)
        else:
            return self.move_mit(zero_joints, speed=speed, gripper=None)

    def move_gripper(self, value: float, wait: bool = True) -> bool:
        """控制夹爪

        Args:
            value: 夹爪目标值 [0=关闭, 1000=打开]
            wait: 是否阻塞等待到达

        Returns:
            True=到达目标 / False=超时
        """
        value = validate_gripper_value(value)

        # 读取当前关节位置，保持不变
        current = self._get_current_angles()

        if self._mode == ControlMode.PV:
            # PV: 发送当前位置（不移动关节），仅控制夹爪
            positions = self._apply_direction_map(current)
            velocities = [0.0] * NUM_JOINTS  # 关节速度为 0
            # 夹爪
            positions.append(value)
            state = self._device.joint_state
            current_gripper = state.gripper if state else 0.0
            g_sign = 1.0 if value > current_gripper else -1.0
            velocities.append(g_sign * speed_user_to_firmware(200.0))
            self._device.send_pv(self._device.aim, positions, velocities)
        else:
            # MIT: 发送全参数帧，关节保持当前位置
            mit_params = self._build_hold_position_mit(current)
            # 覆盖夹爪 pos_ref
            gripper_kp, gripper_kd = _fill_mit_defaults(NUM_JOINTS, None, None)
            mit_params[NUM_JOINTS] = MitParams(
                pos_ref=value,
                vel_ref=0.0,
                t_ref=0.0,
                kp=gripper_kp,
                kd=gripper_kd,
            )
            self._device.send_mit(self._device.aim, mit_params)

        if wait:
            # 等待夹爪到达（简单延时，夹爪无精确位置反馈）
            time.sleep(1.0)
        return True

    # ========== 力矩与使能 ==========

    def torque_off(self, joints: Optional[List[int]] = None) -> bool:
        """卸载力矩（仅 MIT 模式，发送 0x05 指令）

        原理: 将指定关节的 kp=kd 置零，使其自由运动。
        未指定的关节保持原有 kp/kd，继续锁定。
        如果当前为 PV 模式，先自动切换到 MIT。

        Args:
            joints: 需要卸力的关节索引列表 (0~5)，None 表示全部关节

        Returns:
            True=指令发送成功
        """
        # 如果不在 MIT 模式，自动切换
        if self._mode != ControlMode.MIT:
            self.switch_mode(ControlMode.MIT)

        # 确定卸力关节范围
        if joints is None:
            start_joint = 0
            joint_count = NUM_MOTORS  # 包含夹爪
        else:
            start_joint = min(joints)
            joint_count = max(joints) - start_joint + 1

        # 发送 0x05 卸力指令
        frame = self._device.codec.encode_torque_request(TorqueRequest(
            aim=self._device.aim,
            start_joint=start_joint,
            joint_count=joint_count,
        ))
        resp = self._device.send_and_wait(frame, CMD_TORQUE, timeout=1.0)
        if resp is None:
            logger.warning("卸力指令响应超时")
            return False

        logger.info("卸力完成: joints=%s", joints or "全部")
        return True

    def torque_on(self, joints: Optional[List[int]] = None) -> bool:
        """恢复力矩（仅 MIT 模式，发送 0x05 指令）

        安全序列:
        1. 读取当前关节位置
        2. 发送 0x05 恢复指令（固件内部恢复 kp/kd）
        3. 立即以当前位置发送 MIT 帧（防止弹回旧位置）

        Args:
            joints: 需要恢复力矩的关节索引列表，None 表示全部

        Returns:
            True=成功恢复
        """
        # 步骤1: 读取当前位置
        current = self._get_current_angles()

        # 步骤2: 发送 0x05 恢复力矩指令
        if joints is None:
            start_joint = 0
            joint_count = NUM_MOTORS
        else:
            start_joint = min(joints)
            joint_count = max(joints) - start_joint + 1

        frame = self._device.codec.encode_torque_request(TorqueRequest(
            aim=self._device.aim,
            start_joint=start_joint,
            joint_count=joint_count,
        ))
        resp = self._device.send_and_wait(frame, CMD_TORQUE, timeout=1.0)
        if resp is None:
            logger.warning("恢复力矩指令响应超时")
            return False

        # 步骤3: 安全序列 — 以当前位置发送首帧（防止突跳）
        self._send_position_latch_mit(current)

        logger.info("力矩恢复完成: joints=%s", joints or "全部")
        return True

    def enable(self) -> bool:
        """使能机器人（发送 0x09 指令，任何模式可用）

        安全序列:
        1. 发送 0x09 使能指令
        2. 读取当前关节位置
        3. 以当前位置发送首帧（防止突跳到旧目标位置）

        Returns:
            True=使能成功
        """
        # 步骤1: 发送使能指令
        frame = self._device.codec.encode_enable_request(EnableRequest(
            aim=self._device.aim,
            enable=True,
        ))
        resp = self._device.send_and_wait(frame, CMD_ENABLE, timeout=1.0)
        if resp is None:
            logger.warning("使能指令响应超时")
            return False

        # 步骤2: 等待状态缓存更新
        time.sleep(0.05)

        # 步骤3: 读取当前位置并发送安全首帧
        current = self._get_current_angles()
        self._send_safety_first_frame(current)

        logger.info("使能完成")
        return True

    def disable(self) -> bool:
        """失能机器人（发送 0x09 指令，任何模式可用）

        Returns:
            True=失能成功
        """
        frame = self._device.codec.encode_enable_request(EnableRequest(
            aim=self._device.aim,
            enable=False,
        ))
        resp = self._device.send_and_wait(frame, CMD_ENABLE, timeout=1.0)
        if resp is None:
            logger.warning("失能指令响应超时")
            return False

        logger.info("失能完成")
        return True

    def switch_mode(self, mode: Union[str, ControlMode]) -> bool:
        """切换控制模式（发送 0x11 指令, addr=0x0B）

        安全序列:
        1. 读取当前关节位置
        2. 发送 0x11 切换模式
        3. 以当前位置发送首帧（防止突跳）
           - 切到 MIT: 发送 pos=当前位置, kp=默认, kd=默认 的全参数帧
           - 切到 PV: 发送 pos=当前位置, vel=0 的 PV 帧

        Args:
            mode: 目标控制模式 ('pv' / 'mit' 或 ControlMode 枚举)

        Returns:
            True=切换成功
        """
        if isinstance(mode, str):
            mode = ControlMode(mode.lower())

        if mode == self._mode:
            logger.debug("已处于 %s 模式，无需切换", mode.value)
            return True

        # 步骤1: 读取当前位置
        current = self._get_current_angles()

        # 步骤2: 发送 0x11 模式切换指令
        ctrl_mode_value = CTRL_MODE_MIT if mode == ControlMode.MIT else CTRL_MODE_PV
        frame = self._device.codec.encode_motor_param_request(MotorParamRequest(
            aim=self._device.aim,
            start_motor=0,
            motor_count=NUM_MOTORS,
            param_addr=MOTOR_PARAM_CTRL_MODE,
            param_value=ctrl_mode_value,
        ))
        resp = self._device.send_and_wait(frame, CMD_MOTOR_PARAM, timeout=1.0)
        if resp is None:
            logger.warning("模式切换指令响应超时")
            return False

        # 更新内部模式
        old_mode = self._mode
        self._mode = mode

        # 步骤3: 安全首帧（根据新模式格式发送）
        self._send_safety_first_frame(current)

        logger.info("控制模式已切换: %s → %s", old_mode.value, mode.value)
        return True

    def set_zero_position(self) -> bool:
        """设置当前位姿为零位（发送 0x03 指令）

        Returns:
            True=设置成功
        """
        frame = self._device.codec.encode_zero_reset(ZeroResetRequest(
            aim=self._device.aim,
            start_joint=0,
            joint_count=NUM_MOTORS,
        ))
        resp = self._device.send_and_wait(frame, CMD_ZERO_RESET, timeout=1.0)
        if resp is None:
            logger.warning("零位标定指令响应超时")
            return False

        logger.info("零位标定完成")
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

        Args:
            target: 目标关节角度 (rad), 6 个关节
            tolerance: 到达判定阈值 (rad)
            timeout: 超时时间 (秒)

        Returns:
            True=到达目标 / False=超时
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

        logger.warning("等待到达目标超时 (%.1fs)", timeout)
        return False

    # ========== 内部辅助方法 ==========

    def _get_current_angles(self) -> List[float]:
        """读取当前关节角度（从状态缓存）

        Returns:
            6 个关节的当前角度 (rad)

        Raises:
            RobotStateError: 状态缓存不可用（串口未连接或状态尚未获取）
        """
        state = self._device.joint_state
        if state is None:
            raise RobotStateError(
                "无法获取当前关节状态: 状态缓存为空（串口未连接或尚未收到状态数据）"
            )
        return list(state.angles[:NUM_JOINTS])

    def _apply_direction_map(self, joints: List[float]) -> List[float]:
        """对 6 个关节角度应用方向映射

        Args:
            joints: 6 个关节角度 (rad)

        Returns:
            应用方向映射后的角度列表（仅关节，不含夹爪）
        """
        return [joints[i] * DIRECTION_MAP[i] for i in range(NUM_JOINTS)]

    def _compute_signed_velocities(
        self,
        target: List[float],
        current: List[float],
        speeds: List[float],
    ) -> List[float]:
        """计算有符号速度列表（仅关节，不含夹爪）

        用户速度 [0, 400] → 固件速度 [0, 10] rad/s → 加方向符号

        Args:
            target: 目标角度 (rad), 6 个关节
            current: 当前角度 (rad), 6 个关节
            speeds: 用户速度列表 [0, 400], 6 个关节

        Returns:
            有符号速度列表 (rad/s), 6 个元素
        """
        velocities = []
        for i in range(NUM_JOINTS):
            magnitude = speed_user_to_firmware(speeds[i])
            # 方向符号: 根据目标与当前的差值决定
            diff = target[i] - current[i]
            if abs(diff) < 1e-6:
                sign = 1.0
            else:
                sign = 1.0 if diff > 0 else -1.0
            # 应用方向映射（电机正方向可能与关节正方向相反）
            velocities.append(sign * magnitude * DIRECTION_MAP[i])
        return velocities

    def _build_hold_position_mit(
        self, current_angles: List[float]
    ) -> List[MitParams]:
        """构建「保持当前位置」的 MIT 参数列表

        用于安全首帧: pos=当前位置, vel=0, t=0, kp=默认, kd=默认

        Args:
            current_angles: 当前 6 个关节角度 (rad)

        Returns:
            7 个电机的 MitParams 列表
        """
        params = []
        for i in range(NUM_JOINTS):
            kp, kd = _fill_mit_defaults(i, None, None)
            params.append(MitParams(
                pos_ref=current_angles[i] * DIRECTION_MAP[i],
                vel_ref=0.0,
                t_ref=0.0,
                kp=kp,
                kd=kd,
            ))
        # 夹爪: 保持当前位置
        state = self._device.joint_state
        current_gripper = state.gripper if state else 0.0
        gripper_kp, gripper_kd = _fill_mit_defaults(NUM_JOINTS, None, None)
        params.append(MitParams(
            pos_ref=current_gripper,
            vel_ref=0.0,
            t_ref=0.0,
            kp=gripper_kp,
            kd=gripper_kd,
        ))
        return params

    def _send_position_latch_mit(self, current_angles: List[float]) -> None:
        """发送 MIT 位置锁定帧（安全首帧）

        以当前位置为目标，vel=0, t=0, kp/kd=默认值。
        用于恢复力矩、使能、模式切换后防止关节突跳。

        Args:
            current_angles: 当前 6 个关节角度 (rad)
        """
        params = self._build_hold_position_mit(current_angles)
        self._device.send_mit(self._device.aim, params)
        logger.debug("MIT 安全首帧已发送 (位置锁定)")

    def _send_position_latch_pv(self, current_angles: List[float]) -> None:
        """发送 PV 位置锁定帧（安全首帧）

        以当前位置为目标，vel=0。
        用于使能、模式切换后防止关节突跳。

        Args:
            current_angles: 当前 6 个关节角度 (rad)
        """
        positions = self._apply_direction_map(current_angles)
        velocities = [0.0] * NUM_JOINTS
        # 夹爪
        state = self._device.joint_state
        current_gripper = state.gripper if state else 0.0
        positions.append(current_gripper)
        velocities.append(0.0)

        self._device.send_pv(self._device.aim, positions, velocities)
        logger.debug("PV 安全首帧已发送 (位置锁定)")

    def _send_safety_first_frame(self, current_angles: List[float]) -> None:
        """根据当前模式发送安全首帧

        Args:
            current_angles: 当前 6 个关节角度 (rad)
        """
        if self._mode == ControlMode.MIT:
            self._send_position_latch_mit(current_angles)
        else:
            self._send_position_latch_pv(current_angles)
