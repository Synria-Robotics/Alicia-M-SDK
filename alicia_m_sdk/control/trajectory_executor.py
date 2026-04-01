"""轨迹回放执行器

PV 模式: 逐帧发送 pos+vel，固件负责电机级平滑。
MIT 模式: 不需要独立的轨迹回放（直接使用 send_mit 高频发帧）。
"""

import logging
import time
import threading
from typing import Optional

import numpy as np

from ..hardware.device import Device
from ..protocol.constants import NUM_JOINTS, NUM_MOTORS
from ..types.exceptions import ValidationError, MotionError
from ..utils.timing import precise_sleep

logger = logging.getLogger(__name__)

# 方向映射（与 joint_control 保持一致）
DIRECTION_MAP = [1, -1, 1, 1, -1, 1, 1]


class TrajectoryExecutor:
    """轨迹回放执行器

    PV 模式: 逐帧发送 pos+vel，固件负责电机级平滑。

    Args:
        device: 设备抽象层实例
    """

    def __init__(self, device: Device):
        self._device = device
        self._running = False
        self._stop_event = threading.Event()

    @property
    def is_running(self) -> bool:
        """是否正在执行轨迹"""
        return self._running

    def execute_pv(
        self,
        timestamps: np.ndarray,
        positions: np.ndarray,
        velocities: np.ndarray,
    ) -> bool:
        """PV 模式轨迹执行

        按时间戳逐帧发送 pos+vel 控制帧，通过精确定时同步轨迹节拍。

        Args:
            timestamps: 时间序列 (s)，形状 [N]，单调递增
            positions: 关节位置序列 [N, 7] (rad)，含夹爪
                       7 列分别对应 6 关节 + 1 夹爪
            velocities: 关节速度序列 [N, 7] (rad/s)，含符号
                       7 列分别对应 6 关节 + 1 夹爪速度

        Returns:
            True=轨迹执行完成 / False=被中断

        Raises:
            ValidationError: 输入数据格式不正确
        """
        # 输入校验
        n_frames = len(timestamps)
        if n_frames == 0:
            raise ValidationError("轨迹为空: timestamps 长度为 0")

        if positions.shape != (n_frames, NUM_MOTORS):
            raise ValidationError(
                f"positions 形状错误: 期望 ({n_frames}, {NUM_MOTORS}), "
                f"实际 {positions.shape}"
            )
        if velocities.shape != (n_frames, NUM_MOTORS):
            raise ValidationError(
                f"velocities 形状错误: 期望 ({n_frames}, {NUM_MOTORS}), "
                f"实际 {velocities.shape}"
            )

        # 重置停止标志
        self._stop_event.clear()
        self._running = True
        logger.info("PV 轨迹执行开始: %d 帧, 时长 %.2fs",
                     n_frames, timestamps[-1] - timestamps[0])

        try:
            t_start = time.perf_counter()

            for frame_idx in range(n_frames):
                # 检查停止信号
                if self._stop_event.is_set():
                    logger.info("轨迹执行被中断 (帧 %d/%d)",
                                frame_idx, n_frames)
                    return False

                # 等待到该帧的时间戳
                target_time = t_start + timestamps[frame_idx]
                now = time.perf_counter()
                if target_time > now:
                    precise_sleep(target_time - now)

                # 构建该帧数据（应用方向映射）
                pos_frame = []
                vel_frame = []
                for motor_idx in range(NUM_MOTORS):
                    pos = float(positions[frame_idx, motor_idx])
                    vel = float(velocities[frame_idx, motor_idx])
                    # 对关节应用方向映射（夹爪不映射）
                    if motor_idx < NUM_JOINTS:
                        pos *= DIRECTION_MAP[motor_idx]
                        vel *= DIRECTION_MAP[motor_idx]
                    pos_frame.append(pos)
                    vel_frame.append(vel)

                # 发送 PV 帧
                self._device.send_pv(
                    self._device.aim, pos_frame, vel_frame
                )

            elapsed = time.perf_counter() - t_start
            logger.info("PV 轨迹执行完成: 实际耗时 %.3fs", elapsed)
            return True

        except Exception as e:
            logger.error("轨迹执行异常: %s", e)
            raise MotionError(f"轨迹执行失败: {e}") from e

        finally:
            self._running = False

    def stop(self) -> None:
        """紧急停止当前轨迹

        设置停止标志，execute_pv 将在下一帧检测到并返回 False。
        立即发送一帧零速度帧以停止运动。
        """
        self._stop_event.set()

        # 读取当前位置并发送零速度帧（尽快停止）
        state = self._device.joint_state
        if state is not None:
            positions = []
            for i in range(NUM_JOINTS):
                positions.append(state.angles[i] * DIRECTION_MAP[i])
            positions.append(state.gripper)
            velocities = [0.0] * NUM_MOTORS
            self._device.send_pv(self._device.aim, positions, velocities)
            logger.info("紧急停止: 已发送零速度帧")
        else:
            logger.warning("紧急停止: 无法读取当前位置，仅设置停止标志")
