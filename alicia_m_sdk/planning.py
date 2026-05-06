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


def _estimate_num_points(duration: Optional[float], frequency: float, default_points: int = 100) -> int:
    """Convert duration/frequency to planner num_points."""
    if duration is None:
        return int(default_points)
    return max(2, int(round(float(duration) * float(frequency))))


def _rpy_xyz_to_matrix(rpy: np.ndarray) -> np.ndarray:
    """Convert xyz Euler angles to rotation matrix."""
    rx, ry, rz = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    rx_m = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    ry_m = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    rz_m = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    return rz_m @ ry_m @ rx_m


def _quat_xyzw_to_matrix(quat: np.ndarray) -> np.ndarray:
    """Convert xyzw quaternion to rotation matrix."""
    x, y, z, w = [float(v) for v in quat]
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def _pose_vectors_to_transforms(waypoints_array: np.ndarray) -> np.ndarray:
    """Convert [N,6]/[N,7] pose vectors to [N,4,4] transforms."""
    n, d = waypoints_array.shape
    transforms = np.tile(np.eye(4, dtype=np.float64), (n, 1, 1))
    transforms[:, :3, 3] = waypoints_array[:, :3]
    if d == 7:
        for i in range(n):
            transforms[i, :3, :3] = _quat_xyzw_to_matrix(waypoints_array[i, 3:7])
    else:  # d == 6
        for i in range(n):
            transforms[i, :3, :3] = _rpy_xyz_to_matrix(waypoints_array[i, 3:6])
    return transforms


def _rotation_matrix_to_quat_xyzw(rot: np.ndarray) -> np.ndarray:
    """Convert rotation matrix to xyzw quaternion."""
    try:
        from robocore.transform import matrix_to_quaternion
        return np.asarray(matrix_to_quaternion(rot), dtype=np.float64)
    except Exception:
        m = rot
        trace = float(m[0, 0] + m[1, 1] + m[2, 2])
        if trace > 0.0:
            s = np.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (m[2, 1] - m[1, 2]) / s
            y = (m[0, 2] - m[2, 0]) / s
            z = (m[1, 0] - m[0, 1]) / s
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
        return np.array([x, y, z, w], dtype=np.float64)


def _transforms_to_pose_vectors(poses_4x4: np.ndarray) -> np.ndarray:
    """Convert [N,4,4] transforms to [N,7] pose vectors."""
    n = poses_4x4.shape[0]
    poses_7 = np.zeros((n, 7), dtype=np.float64)
    poses_7[:, :3] = poses_4x4[:, :3, 3]
    for i in range(n):
        poses_7[i, 3:] = _rotation_matrix_to_quat_xyzw(poses_4x4[i, :3, :3])
    return poses_7


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
    num_points = int(kwargs.pop('num_points', _estimate_num_points(duration, frequency, 100)))

    # 选择规划器
    if planner_type == 'b_spline':
        # 兼容旧参数名 order，新版 RoboCore 使用 degree
        degree = kwargs.pop('degree', kwargs.pop('order', 5))
        planner = BSplinePlanner(degree=degree, **kwargs)
        plan_kwargs = {
            'duration': duration,
            'num_points': num_points,
        }
    elif planner_type == 'multi_segment':
        # 兼容旧参数名 segment_type，新版 RoboCore 使用 method
        method = kwargs.pop('method', kwargs.pop('segment_type', 'quintic'))
        planner = MultiSegmentPlanner(method=method, **kwargs)
        duration_per_segment = kwargs.pop('duration_per_segment', 1.0)
        num_points_per_segment = int(kwargs.pop('num_points_per_segment', 50))
        plan_kwargs = {
            'durations': duration_per_segment,
            'num_points_per_segment': num_points_per_segment,
        }
    else:
        raise ValueError(f"不支持的规划器类型: '{planner_type}'，可选: 'b_spline', 'multi_segment'")

    # 执行规划
    result = planner.plan(
        waypoints=waypoints_array,
        **plan_kwargs,
    )

    # 标准化返回格式
    if isinstance(result, dict):
        timestamps = np.asarray(result.get('timestamps', result.get('t', [])))
        positions = np.asarray(result.get('positions', result.get('q', [])))
        velocities = np.asarray(result.get('velocities', result.get('qd', [])))
        accelerations = np.asarray(result.get('accelerations', result.get('qdd', [])))
        duration_out = float(result.get('duration', 0.0))
        if duration_out <= 0.0 and len(timestamps) > 0:
            duration_out = float(timestamps[-1] - timestamps[0])
        return {
            'success': result.get('success', True),
            'timestamps': timestamps,
            'positions': positions,
            'velocities': velocities,
            'accelerations': accelerations,
            'duration': duration_out,
            'num_points': int(result.get('num_points', len(timestamps))),
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
    if waypoints_array.ndim == 2 and waypoints_array.shape[1] in (6, 7):
        waypoints_array = _pose_vectors_to_transforms(waypoints_array)

    # 提取规划参数
    duration = kwargs.pop('duration', None)
    frequency = kwargs.pop('frequency', 200.0)
    num_points = int(kwargs.pop('num_points', _estimate_num_points(duration, frequency, 100)))

    # 创建笛卡尔规划器并执行
    planner = SplineCurvePlanner(**kwargs)
    result = planner.plan(
        waypoints=waypoints_array,
        duration=duration,
        num_points=num_points,
    )

    # 标准化返回格式
    if isinstance(result, dict):
        timestamps = np.asarray(result.get('timestamps', result.get('t', [])))
        positions = np.asarray(result.get('positions', []))
        orientations = np.asarray(result.get('orientations', []))
        poses = np.asarray(result.get('poses', []))
        if poses.ndim == 3 and poses.shape[1:] == (4, 4):
            poses = _transforms_to_pose_vectors(poses)
            positions = poses[:, :3]
            orientations = poses[:, 3:]
        duration_out = float(result.get('duration', 0.0))
        if duration_out <= 0.0 and len(timestamps) > 0:
            duration_out = float(timestamps[-1] - timestamps[0])
        return {
            'success': result.get('success', True),
            'timestamps': timestamps,
            'positions': positions,
            'orientations': orientations,
            'poses': poses,
            'duration': duration_out,
            'num_points': int(result.get('num_points', len(timestamps))),
        }

    # 兼容: 规划器直接返回元组 (timestamps, poses)
    timestamps, poses = result
    timestamps = np.asarray(timestamps)
    poses = np.asarray(poses)
    if poses.ndim == 3 and poses.shape[1:] == (4, 4):
        poses = _transforms_to_pose_vectors(poses)

    return {
        'success': True,
        'timestamps': timestamps,
        'positions': poses[:, :3] if poses.ndim == 2 and poses.shape[1] >= 3 else poses,
        'orientations': poses[:, 3:7] if poses.ndim == 2 and poses.shape[1] >= 7 else np.array([]),
        'poses': poses,
        'duration': float(timestamps[-1] - timestamps[0]) if len(timestamps) > 0 else 0.0,
        'num_points': len(timestamps),
    }
