"""拖动示教

流程:
1. 确保 MIT 模式 → 发送 0x05 卸力 (kp=kd=0)
2. 后台录制线程以固定间隔读取关节位置
3. 用户手动拖动机械臂
4. 停止录制 → 发送 0x05 恢复力矩

回放使用 MIT 全参数帧高频发送。
"""

import logging
import time
import threading
from typing import List, Optional

from ..hardware.device import Device
from ..protocol.constants import NUM_JOINTS, NUM_MOTORS
from ..types.state import MitParams
from ..types.exceptions import RobotStateError
from ..utils.timing import precise_sleep
from .joint_control import JointController, _fill_mit_defaults

logger = logging.getLogger(__name__)


class DragTeaching:
    """拖动示教

    提供路点录制和回放功能。录制期间卸载关节力矩，
    用户手动拖动机械臂，SDK 以固定间隔记录关节位置。
    回放时通过 MIT 全参数帧高频发送录制的路点。

    Args:
        device: 设备抽象层实例
        joint_controller: 关节控制器实例（用于力矩开关和模式切换）
    """

    def __init__(self, device: Device, joint_controller: JointController):
        self._device = device
        self._joint_ctrl = joint_controller
        self._recording = False
        self._stop_event = threading.Event()
        self._record_thread: Optional[threading.Thread] = None
        self._waypoints: List[List[float]] = []
        self._lock = threading.Lock()

    @property
    def is_recording(self) -> bool:
        """是否正在录制"""
        return self._recording

    @property
    def waypoint_count(self) -> int:
        """已录制的路点数量"""
        with self._lock:
            return len(self._waypoints)

    def start_recording(self, interval: float = 0.05) -> None:
        """进入无力矩状态并开始录制路点

        步骤:
        1. 确保 MIT 模式
        2. 卸载全部关节力矩 (torque_off)
        3. 启动后台录制线程，以 interval 间隔记录关节位置

        Args:
            interval: 录制间隔 (秒)，默认 0.05s (20Hz)

        Raises:
            RobotStateError: 已在录制中
        """
        if self._recording:
            raise RobotStateError("已在录制中，请先调用 stop_recording()")

        # 清空之前的路点
        with self._lock:
            self._waypoints = []

        # 确保 MIT 模式并卸力
        self._joint_ctrl.torque_off(joints=None)

        # 启动录制线程
        self._stop_event.clear()
        self._recording = True
        self._record_thread = threading.Thread(
            target=self._record_loop,
            args=(interval,),
            daemon=True,
            name="alicia-teaching-record",
        )
        self._record_thread.start()
        logger.info("拖动示教录制开始 (间隔 %.3fs)", interval)

    def stop_recording(self) -> List[List[float]]:
        """停止录制，恢复力矩，返回路点列表

        Returns:
            路点列表，每个路点为 [j0, j1, j2, j3, j4, j5, gripper] (7 个值)
        """
        if not self._recording:
            logger.warning("当前未在录制，直接返回空列表")
            with self._lock:
                return list(self._waypoints)

        # 停止录制线程
        self._stop_event.set()
        if self._record_thread:
            self._record_thread.join(timeout=2.0)
            self._record_thread = None
        self._recording = False

        # 恢复力矩
        self._joint_ctrl.torque_on(joints=None)

        with self._lock:
            waypoints = list(self._waypoints)

        logger.info("录制停止: 共 %d 个路点", len(waypoints))
        return waypoints

    def replay(
        self,
        waypoints: List[List[float]],
        hz: float = 200,
    ) -> bool:
        """回放录制的路点（MIT 全参数帧高频发送）

        以指定频率逐帧发送 MIT 全参数帧。
        每个路点包含 6 关节角度 + 夹爪值。

        Args:
            waypoints: 路点列表，每个路点为 7 个值 [j0..j5, gripper]
            hz: 回放频率 (Hz)，默认 200Hz

        Returns:
            True=回放完成 / False=被中断
        """
        if not waypoints:
            logger.warning("回放路点为空")
            return True

        frame_interval = 1.0 / hz
        n_frames = len(waypoints)
        logger.info("MIT 回放开始: %d 帧, %.0fHz, 预计 %.2fs",
                     n_frames, hz, n_frames * frame_interval)

        self._stop_event.clear()

        for idx, wp in enumerate(waypoints):
            if self._stop_event.is_set():
                logger.info("回放被中断 (帧 %d/%d)", idx, n_frames)
                return False

            t_frame_start = time.perf_counter()

            # 构建 MIT 参数 (pos=路点位置, vel=0, t=0, kp=默认, kd=默认)
            params = []
            for motor_idx in range(NUM_JOINTS):
                kp, kd = _fill_mit_defaults(motor_idx, None, None)
                params.append(MitParams(
                    pos_ref=wp[motor_idx],
                    vel_ref=0.0,
                    t_ref=0.0,
                    kp=kp,
                    kd=kd,
                ))

            # 夹爪
            gripper_val = wp[NUM_JOINTS] if len(wp) > NUM_JOINTS else 0.0
            gripper_kp, gripper_kd = _fill_mit_defaults(NUM_JOINTS, None, None)
            params.append(MitParams(
                pos_ref=gripper_val,
                vel_ref=0.0,
                t_ref=0.0,
                kp=gripper_kp,
                kd=gripper_kd,
            ))

            # 发送 MIT 帧
            self._device.send_mit(self._device.aim, params)

            # 精确等待到下一帧
            elapsed = time.perf_counter() - t_frame_start
            remaining = frame_interval - elapsed
            if remaining > 0:
                precise_sleep(remaining)

        logger.info("MIT 回放完成")
        return True

    # ========== 内部方法 ==========

    def _record_loop(self, interval: float) -> None:
        """录制线程主循环

        以固定间隔读取状态缓存中的关节位置并记录。

        Args:
            interval: 录制间隔 (秒)
        """
        while not self._stop_event.is_set():
            t_start = time.perf_counter()

            state = self._device.joint_state
            if state is not None:
                # 记录 6 关节角度 + 夹爪 = 7 个值
                waypoint = list(state.angles[:NUM_JOINTS]) + [state.gripper]
                with self._lock:
                    self._waypoints.append(waypoint)
            else:
                logger.debug("录制: 状态缓存为空，跳过本帧")

            # 等待到下一个采样时刻
            elapsed = time.perf_counter() - t_start
            remaining = interval - elapsed
            if remaining > 0:
                self._stop_event.wait(remaining)
