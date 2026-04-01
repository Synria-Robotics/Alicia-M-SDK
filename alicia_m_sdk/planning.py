"""规划接口：对 RoboCore 轨迹规划的薄封装

提供关节空间和笛卡尔空间的轨迹规划功能，
规划器采用延迟导入策略，避免不使用规划功能时的导入开销。

所有规划函数均为同步阻塞调用，在发帧前完成离线预计算，
不影响实时控制通信。

性能参考：
- 关节轨迹规划: 约 10~50ms
- 笛卡尔轨迹规划: 约 20~100ms（含 IK 求解）
"""

from typing import Dict, List, Any, Optional

import numpy as np


def plan_joint_trajectory(
    waypoints: List[List[float]],
    planner_type: str = 'b_spline',
    **kwargs,
) -> Dict[str, Any]:
    """关节空间轨迹规划

    在关节空间中对给定路点进行轨迹插值，生成平滑的时间序列
    （时间戳、位置、速度、加速度）。

    Args:
        waypoints: 路点列表，每个路点为关节角度列表 (rad)
            形状: [N, num_joints]，至少需要 2 个路点
        planner_type: 规划器类型，可选值：
            - 'b_spline': B 样条插值（默认，平滑性好）
            - 'multi_segment': 多段多项式插值
        **kwargs: 传递给规划器的额外参数：
            - duration (float): 总时长 (s)，默认根据路点间距自动估算
            - frequency (float): 采样频率 (Hz)，默认 200.0
            - order (int): B 样条阶数，默认 5（仅 b_spline）
            - segment_type (str): 分段类型 "cubic"/"quintic"（仅 multi_segment）

    Returns:
        包含以下键的字典：
        - 'success': 规划是否成功 (bool)
        - 'timestamps': 时间序列 (ndarray, shape [M])
        - 'positions': 关节位置序列 (ndarray, shape [M, num_joints])
        - 'velocities': 关节速度序列 (ndarray, shape [M, num_joints])
        - 'accelerations': 关节加速度序列 (ndarray, shape [M, num_joints])
        - 'duration': 轨迹总时长 (float, s)
        - 'num_points': 采样点数 (int)
        - 'planner_type': 使用的规划器类型 (str)

    Raises:
        ValueError: 路点数量不足或格式错误
        ImportError: robocore.planning 不可用
    """
    # 参数校验
    if len(waypoints) < 2:
        raise ValueError(f"路点数量不足: 至少需要 2 个路点，当前 {len(waypoints)} 个")

    # 延迟导入规划模块
    from robocore.planning import BSplinePlanner, MultiSegmentPlanner

    waypoints_array = np.array(waypoints, dtype=np.float64)

    # 提取规划参数
    duration = kwargs.pop('duration', None)
    frequency = kwargs.pop('frequency', 200.0)

    # 选择规划器
    if planner_type == 'b_spline':
        order = kwargs.pop('order', 5)
        planner = BSplinePlanner(order=order, **kwargs)
    elif planner_type == 'multi_segment':
        segment_type = kwargs.pop('segment_type', 'quintic')
        planner = MultiSegmentPlanner(segment_type=segment_type, **kwargs)
    else:
        raise ValueError(f"不支持的规划器类型: '{planner_type}'，可选: 'b_spline', 'multi_segment'")

    # 执行规划
    result = planner.plan(
        waypoints=waypoints_array,
        duration=duration,
        frequency=frequency,
    )

    # 标准化返回格式
    if isinstance(result, dict):
        return {
            'success': result.get('success', True),
            'timestamps': np.asarray(result.get('timestamps', [])),
            'positions': np.asarray(result.get('positions', [])),
            'velocities': np.asarray(result.get('velocities', [])),
            'accelerations': np.asarray(result.get('accelerations', [])),
            'duration': float(result.get('duration', 0.0)),
            'num_points': int(result.get('num_points', 0)),
            'planner_type': planner_type,
        }

    # 兼容: 规划器直接返回元组 (timestamps, positions, velocities, accelerations)
    timestamps, positions, velocities, accelerations = result
    timestamps = np.asarray(timestamps)
    positions = np.asarray(positions)
    velocities = np.asarray(velocities)
    accelerations = np.asarray(accelerations)

    return {
        'success': True,
        'timestamps': timestamps,
        'positions': positions,
        'velocities': velocities,
        'accelerations': accelerations,
        'duration': float(timestamps[-1] - timestamps[0]) if len(timestamps) > 0 else 0.0,
        'num_points': len(timestamps),
        'planner_type': planner_type,
    }


def plan_cartesian_trajectory(
    waypoints: List[List[float]],
    **kwargs,
) -> Dict[str, Any]:
    """笛卡尔空间轨迹规划

    在笛卡尔空间中对给定末端位姿路点进行轨迹插值，
    生成平滑的末端位姿序列（位置 + 姿态）。

    Args:
        waypoints: 末端位姿路点列表，每个路点为:
            - 长度 6: [x, y, z, rx, ry, rz]（位置+欧拉角）
            - 长度 7: [x, y, z, qx, qy, qz, qw]（位置+四元数）
            至少需要 2 个路点
        **kwargs: 传递给规划器的额外参数：
            - duration (float): 总时长 (s)
            - frequency (float): 采样频率 (Hz)，默认 200.0
            - interpolation (str): 姿态插值方法 "slerp"/"linear"

    Returns:
        包含以下键的字典：
        - 'success': 规划是否成功 (bool)
        - 'timestamps': 时间序列 (ndarray, shape [M])
        - 'positions': 末端位置序列 (ndarray, shape [M, 3])
        - 'orientations': 末端姿态序列 (ndarray, shape [M, 4]，四元数 xyzw)
        - 'poses': 完整位姿序列 (ndarray, shape [M, 7]，pos+quat)
        - 'duration': 轨迹总时长 (float, s)
        - 'num_points': 采样点数 (int)

    Raises:
        ValueError: 路点数量不足或格式错误
        ImportError: robocore.planning 不可用
    """
    # 参数校验
    if len(waypoints) < 2:
        raise ValueError(f"路点数量不足: 至少需要 2 个路点，当前 {len(waypoints)} 个")

    # 延迟导入规划模块
    from robocore.planning import SplineCurvePlanner

    waypoints_array = np.array(waypoints, dtype=np.float64)

    # 提取规划参数
    duration = kwargs.pop('duration', None)
    frequency = kwargs.pop('frequency', 200.0)

    # 创建笛卡尔规划器并执行
    planner = SplineCurvePlanner(**kwargs)
    result = planner.plan(
        waypoints=waypoints_array,
        duration=duration,
        frequency=frequency,
    )

    # 标准化返回格式
    if isinstance(result, dict):
        return {
            'success': result.get('success', True),
            'timestamps': np.asarray(result.get('timestamps', [])),
            'positions': np.asarray(result.get('positions', [])),
            'orientations': np.asarray(result.get('orientations', [])),
            'poses': np.asarray(result.get('poses', [])),
            'duration': float(result.get('duration', 0.0)),
            'num_points': int(result.get('num_points', 0)),
        }

    # 兼容: 规划器直接返回元组 (timestamps, poses)
    timestamps, poses = result
    timestamps = np.asarray(timestamps)
    poses = np.asarray(poses)

    return {
        'success': True,
        'timestamps': timestamps,
        'positions': poses[:, :3] if poses.ndim == 2 and poses.shape[1] >= 3 else poses,
        'orientations': poses[:, 3:7] if poses.ndim == 2 and poses.shape[1] >= 7 else np.array([]),
        'poses': poses,
        'duration': float(timestamps[-1] - timestamps[0]) if len(timestamps) > 0 else 0.0,
        'num_points': len(timestamps),
    }
