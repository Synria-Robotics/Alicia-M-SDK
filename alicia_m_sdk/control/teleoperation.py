"""遥操作模块 — 示教臂跟随控制

使用 Alicia-D 伺服示教臂（leader）实时控制
Alicia-M 电机操作臂（follower）跟随运动。

支持 PV 和 MIT 两种控制模式，由 follower 当前模式自动决定:
- PV: 每帧发送 pos+vel，固件做插值
- MIT: 每帧发送 6 地址单帧（含线性轨迹插值速度），固件按指定速度平滑跟随

控制循环在后台线程以固定频率运行，每帧:
    1. 读取 leader 关节/夹爪状态
    2. 经符号/偏移映射转换
    3. 发送到 follower（不等待到达）
"""

import time
import threading
import logging
from typing import Optional, Union, List, Callable

from ..control.joint_control import _normalize_mit_param
from ..utils.timing import precise_sleep

logger = logging.getLogger(__name__)


class Teleoperation:
    """实时主从遥操作控制器

    从 leader 臂读取关节状态，经符号/偏移映射后发送到 follower 臂。
    自动适配 follower 当前控制模式（PV / MIT）。

    Args:
        leader: 示教臂实例（Alicia-D SynriaRobotAPI），需提供
            ``get_robot_state("joint_gripper")``（返回含 ``.angles``、``.gripper`` 属性的对象）
            和 ``torque_control("off"/"on")``
        follower: 操作臂实例（Alicia-M SynriaRobotAPI），PV 或 MIT 模式均可
        frequency_hz: 控制循环频率 (Hz)
        follower_speed: follower 运动速度 [0, 400]，映射到 [0, 10] rad/s。
            默认 400（最大速度，实时跟随）。PV 模式作为关节速度，
            MIT 插值模式作为线性轨迹插值速度
        gripper_scale: leader → follower 夹爪缩放系数。
            两臂均为 0-1000 量程时使用默认 1.0
        joint_signs: 6 个关节的符号乘数 (+1/-1)，补偿 leader/follower 关节方向差异
        joint_offsets_rad: 6 个关节的弧度偏移量，加到映射后的 follower 关节值上
        joint_mapper: 自定义关节映射函数，签名 ``(List[float]) -> List[float]``。
            输入为 leader 关节角度（弧度），输出为 follower 关节角度（弧度）。
            设置后将替代 joint_signs / joint_offsets_rad 的默认映射逻辑
        use_interpolation: MIT 模式是否使用线性轨迹插值（默认 False）。
            启用时发送 6 地址帧，运动更平滑但可能降低重力负载关节的刚度
        kp: MIT 位置增益 [0, 500]。None=使用默认值，
            float=广播至所有电机，List[float] 长度 6 或 7 逐电机设置。
        kd: MIT 速度增益 [0, 5]。格式同 kp。
        torque: MIT 前馈力矩 (N·m)。None=默认 0，
            float=广播，List[float] 逐电机设置。
        vel_ref: MIT 目标速度 (rad/s)。格式同 torque。
    """

    def __init__(
        self,
        leader,
        follower,
        frequency_hz: float = 60.0,
        follower_speed: float = 400.0,
        gripper_scale: float = 1.0,
        joint_signs: Optional[List[float]] = None,
        joint_offsets_rad: Optional[List[float]] = None,
        joint_mapper: Optional[Callable[[List[float]], List[float]]] = None,
        use_interpolation: bool = False,
        kp: Optional[Union[float, List[float]]] = None,
        kd: Optional[Union[float, List[float]]] = None,
        torque: Optional[Union[float, List[float]]] = None,
        vel_ref: Optional[Union[float, List[float]]] = None,
    ):
        self.leader = leader
        self.follower = follower
        self.frequency_hz = frequency_hz
        self.follower_speed = follower_speed
        self.gripper_scale = gripper_scale
        self.joint_signs = joint_signs or [1.0] * 6
        self.joint_offsets_rad = joint_offsets_rad or [0.0] * 6
        self.joint_mapper = joint_mapper
        self.use_interpolation = use_interpolation
        # 预标准化 MIT 参数为逐电机列表，避免控制循环中每帧重复转换
        self.kp = _normalize_mit_param(kp, "kp")
        self.kd = _normalize_mit_param(kd, "kd")
        self.torque = _normalize_mit_param(torque, "torque", default=0.0)
        self.vel_ref = _normalize_mit_param(vel_ref, "vel_ref", default=0.0)

        self._running = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._on_state_callback: Optional[Callable] = None
        self._loop_count = 0
        self._error_count = 0

    def set_state_callback(self, callback: Callable) -> None:
        """注册每帧状态回调

        Args:
            callback: 回调函数，签名 ``(leader_joints, leader_gripper, loop_count)``
        """
        self._on_state_callback = callback

    # ========== 内部映射 ==========

    def _map_joints(self, leader_joints: List[float]) -> List[float]:
        """leader → follower 关节角度映射

        若设置了 joint_mapper 则使用自定义映射，否则使用符号翻转 + 偏移。
        """
        if self.joint_mapper is not None:
            return self.joint_mapper(leader_joints)
        return [
            sign * angle + offset
            for angle, sign, offset in zip(
                leader_joints, self.joint_signs, self.joint_offsets_rad
            )
        ]

    def _map_gripper(self, leader_gripper: float) -> float:
        """leader → follower 夹爪值映射（缩放 + 裁剪）"""
        return max(0.0, min(1000.0, leader_gripper * self.gripper_scale))

    # ========== 控制循环 ==========

    def _control_loop(self) -> None:
        """后台控制循环主体

        插值模式策略：首帧发送 6 地址帧设定固件线性插值速度，
        后续帧仅发送 5 地址帧更新目标位置，固件保留插值速度持续生效，
        避免每帧重置插值状态导致 PD 刚度丧失。
        """
        interval = 1.0 / self.frequency_hz
        mode_str = self.follower.control_mode.value.upper()
        interp_sent = False  # 插值首帧是否已发送

        logger.info(
            "遥操作控制循环启动: %.0f Hz, %s 模式, 插值=%s",
            self.frequency_hz, mode_str, self.use_interpolation,
        )

        while self._running.is_set():
            loop_start = time.perf_counter()
            try:
                # 读取 leader 关节 + 夹爪
                state = self.leader.get_robot_state("joint_gripper")
                if state is None or state.angles is None:
                    self._error_count += 1
                    continue

                # 映射到 follower 空间
                follower_joints = self._map_joints(state.angles)
                follower_gripper = self._map_gripper(state.gripper)

                # 插值模式：首帧发 6 地址帧设速度，后续 5 地址帧更新位置
                if self.use_interpolation and not interp_sent:
                    use_interp_this_frame = True
                    interp_sent = True
                else:
                    use_interp_this_frame = False

                # 发送到 follower（PV/MIT 由 follower 当前模式自动路由）
                self.follower.set_robot_state(
                    target_joints=follower_joints,
                    gripper_value=follower_gripper,
                    joint_format='rad',
                    speed=self.follower_speed,
                    gripper_speed=self.follower_speed,
                    wait_for_completion=False,
                    use_interpolation=use_interp_this_frame,
                    kp=self.kp,
                    kd=self.kd,
                    torque=self.torque,
                    vel_ref=self.vel_ref,
                )

                self._loop_count += 1
                if self._on_state_callback:
                    self._on_state_callback(
                        state.angles, state.gripper, self._loop_count
                    )

            except Exception as e:
                self._error_count += 1
                if self._error_count <= 5:
                    logger.warning("遥操作循环异常: %s", e)
                elif self._error_count == 6:
                    logger.warning("后续遥操作异常已静默")

            elapsed = time.perf_counter() - loop_start
            precise_sleep(interval - elapsed)

        logger.info(
            "遥操作控制循环停止 (已发送 %d 帧, %d 次异常)",
            self._loop_count, self._error_count,
        )

    # ========== 公开接口 ==========

    def start(self) -> None:
        """启动遥操作

        禁用 leader 力矩使其可自由拖动，
        follower 开始实时跟随 leader 关节位置。
        """
        if self._running.is_set():
            logger.warning("遥操作已在运行中")
            return

        logger.info("禁用 leader 力矩以自由拖动...")
        self.leader.torque_control('off')

        self._loop_count = 0
        self._error_count = 0
        self._running.set()
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """停止遥操作并恢复 leader 力矩"""
        if not self._running.is_set():
            return

        self._running.clear()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

        logger.info("恢复 leader 力矩...")
        self.leader.torque_control('on')

    def run_interactive(self) -> None:
        """交互式运行: 启动遥操作，等待用户按 Enter 或 Ctrl+C 停止"""
        self.start()
        try:
            input("\n遥操作运行中，按 Enter 停止...\n")
        except KeyboardInterrupt:
            print()
        finally:
            self.stop()
