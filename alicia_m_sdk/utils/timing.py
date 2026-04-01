"""时间工具

提供高精度休眠和帧率统计功能。
用于控制循环中的精确定时和性能监控。
"""

import time
from collections import deque
from typing import Deque


def precise_sleep(duration: float) -> None:
    """高精度休眠（混合策略）

    对于较长的等待时间，先使用 time.sleep 让出 CPU，
    剩余的短时间段使用忙等待（busy-wait）确保亚毫秒精度。

    策略：
        - duration > 2ms: 前段使用 sleep（预留 1ms 余量），尾段忙等待
        - duration <= 2ms: 全程忙等待
        - duration <= 0: 立即返回

    Args:
        duration: 休眠时长（秒），如 0.001 表示 1ms
    """
    if duration <= 0:
        return

    target = time.perf_counter() + duration

    # 对于较长的等待，先 sleep 让出 CPU 时间片
    # 预留 1ms 余量给忙等待段，避免 sleep 的抖动导致超时
    sleep_threshold = 0.002  # 2ms
    sleep_margin = 0.001     # 1ms 余量

    if duration > sleep_threshold:
        time.sleep(duration - sleep_margin)

    # 忙等待到精确时刻
    while time.perf_counter() < target:
        pass


class FPSCounter:
    """帧率统计器

    使用滑动窗口法统计最近 N 帧的平均帧率。
    适用于控制循环的性能监控。

    Args:
        window_size: 统计窗口大小（帧数），默认 60 帧

    用法::

        fps = FPSCounter()
        while running:
            # ... 执行一帧逻辑 ...
            fps.tick()
            print(f"FPS: {fps.get_fps():.1f}")
    """

    def __init__(self, window_size: int = 60):
        self._window_size = window_size
        self._timestamps = deque(maxlen=window_size)  # type: Deque[float]

    def tick(self) -> None:
        """记录当前帧的时间戳

        每帧调用一次，内部自动维护滑动窗口。
        """
        self._timestamps.append(time.perf_counter())

    def get_fps(self) -> float:
        """获取当前帧率 (Hz)

        基于滑动窗口内的时间间隔计算平均帧率。
        窗口内不足 2 帧时返回 0.0。

        Returns:
            平均帧率 (Hz)
        """
        n = len(self._timestamps)
        if n < 2:
            return 0.0

        elapsed = self._timestamps[-1] - self._timestamps[0]
        if elapsed <= 0:
            return 0.0

        return (n - 1) / elapsed

    def reset(self) -> None:
        """重置统计数据"""
        self._timestamps.clear()
