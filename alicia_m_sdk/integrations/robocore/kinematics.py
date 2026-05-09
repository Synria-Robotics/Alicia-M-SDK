"""运动学接口：对 RoboCore FK/IK/Jacobian 的薄封装

提供正运动学、逆运动学和雅可比矩阵计算的便捷入口。
所有函数均为同步阻塞调用，不涉及线程操作。

性能参考：
- FK: < 1ms（矩阵乘法）
- IK: 约 5~50ms（迭代求解，取决于起始猜测质量）
- Jacobian: < 1ms

RoboCore 调用不在控制环路内——控制帧发送是纯协议层操作，
与运动学解算完全隔离。轨迹执行时，所有 FK/IK 在发帧前已完成预计算。
"""

from typing import Dict, List, Optional, Any

import numpy as np

# RoboCore 延迟导入：运动学功能仅在 RoboCore 可用时才能使用
_rc = None
_fk = None
_ik = None
_jac = None
_to_numpy = None
_matrix_to_euler = None
_matrix_to_quaternion = None


def _rpy_xyz_to_matrix(rpy):
    """Convert xyz Euler angles to rotation matrix.

    :param rpy: Euler xyz angles
    :return: Rotation matrix 3x3
    """
    rx, ry, rz = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    rx_m = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=np.float64)
    ry_m = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=np.float64)
    rz_m = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz_m @ ry_m @ rx_m


def _quat_xyzw_to_matrix(quat):
    """Convert xyzw quaternion to rotation matrix.

    :param quat: Quaternion xyzw
    :return: Rotation matrix 3x3
    """
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


def _normalize_target_pose(target_pose):
    """Normalize pose input to 4x4 or [B,4,4] matrix form.

    :param target_pose: Pose in 4x4 / [x,y,z,rpy] / [x,y,z,quat]
    :return: Pose matrix (4x4 or [B,4,4])
    """
    arr = np.asarray(target_pose, dtype=np.float64)
    if arr.shape == (4, 4):
        return arr
    if arr.ndim == 3 and arr.shape[1:] == (4, 4):
        return arr
    if arr.ndim == 1 and arr.shape[0] in (6, 7):
        T = np.eye(4, dtype=np.float64)
        T[:3, 3] = arr[:3]
        if arr.shape[0] == 7:
            T[:3, :3] = _quat_xyzw_to_matrix(arr[3:7])
        else:
            T[:3, :3] = _rpy_xyz_to_matrix(arr[3:6])
        return T
    if arr.ndim == 2 and arr.shape[1] in (6, 7):
        out = np.tile(np.eye(4, dtype=np.float64), (arr.shape[0], 1, 1))
        out[:, :3, 3] = arr[:, :3]
        for i in range(arr.shape[0]):
            if arr.shape[1] == 7:
                out[i, :3, :3] = _quat_xyzw_to_matrix(arr[i, 3:7])
            else:
                out[i, :3, :3] = _rpy_xyz_to_matrix(arr[i, 3:6])
        return out
    return arr


def _ensure_robocore():
    """确保 RoboCore 已导入

    兼容 synria-robocore 2.5.0rc2 的三种后端（numpy / torch / cpp）。
    robocore.utils.backend.to_numpy 在所有后端下均可将原生张量转换为 numpy
    ndarray，包括 2.5.0rc2 新增的 C++ CppTensor 类型，无需额外处理。
    """
    global _rc, _fk, _ik, _jac, _to_numpy, _matrix_to_euler, _matrix_to_quaternion
    if _rc is not None:
        return
    try:
        import robocore as rc
        from robocore.kinematics import forward_kinematics, inverse_kinematics, jacobian
        from robocore.transform import matrix_to_euler, matrix_to_quaternion
        from robocore.utils.backend import to_numpy
        _rc = rc
        _fk = forward_kinematics
        _ik = inverse_kinematics
        _jac = jacobian
        _to_numpy = to_numpy
        _matrix_to_euler = matrix_to_euler
        _matrix_to_quaternion = matrix_to_quaternion
    except ImportError:
        raise ImportError(
            "RoboCore 未安装，运动学功能不可用。"
            "请执行: pip install 'synria-robocore==2.5.0rc2'"
        )


def compute_forward_kinematics(robot_model, q: List[float]) -> Dict[str, Any]:
    """正运动学：关节角度 -> 末端位姿

    将关节角度映射到末端执行器的齐次变换矩阵，同时提取位置、旋转矩阵、
    欧拉角和四元数等多种位姿表示。

    :param robot_model, RoboCore 的 RobotModel 实例（由 create_robot 初始化）
    :param q, 关节角度列表 (rad)，长度应与模型自由度一致
    :return, 包含以下键的字典： - 'transform': 4x4 齐次变换矩阵 (ndarray) - 'position': 末端位置 [x, y, z] (ndarray, 单位 m) - 'rotation': 3x3 旋转矩阵 (ndarray) - 'euler_xyz': 欧拉角 [rx, ry, rz] (rad) - 'quaternion_xyzw': 四元数 [qx, qy, qz, qw]
    """
    _ensure_robocore()
    T = _fk(robot_model, q, return_end=True)
    T_np = _to_numpy(T)
    position = T_np[:3, 3].copy()
    rotation = T_np[:3, :3].copy()

    return {
        'transform': T_np,
        'position': position,
        'rotation': rotation,
        'euler_xyz': _matrix_to_euler(rotation),
        'quaternion_xyzw': _matrix_to_quaternion(rotation),
    }


def compute_inverse_kinematics(
    robot_model,
    target_pose,
    q_init: Optional[List[float]] = None,
    method: str = 'dls',
    **kwargs,
) -> Dict[str, Any]:
    """逆运动学：目标位姿 -> 关节角度

    默认使用阻尼最小二乘法 (DLS) 求解，支持多起点搜索以提高全局收敛概率。

    :param robot_model, RoboCore 的 RobotModel 实例
    :param target_pose, 目标位姿，支持以下格式： - 4x4 齐次变换矩阵 (ndarray) - 长度为 7 的列表/数组 [x, y, z, qx, qy, qz, qw]（位置+四元数） - 长度为 6 的列表/数组 [x, y, z, rx, ry, rz]（位置+欧拉角）
    :param q_init, 初始猜测关节角度 (rad)，None 使用零位或当前位姿
    :param method, IK 求解方法，可选值： - 'dls': 阻尼最小二乘法（默认，稳定性好） - 'jacobian_transpose': 雅可比转置法（速度快但精度较低） - 'levenberg_marquardt': LM 法（收敛性好但计算稍慢）
    :param **kwargs, 传递给 RoboCore inverse_kinematics 的额外参数： - max_iterations (int): 最大迭代次数，默认 100 - tolerance (float): 收敛阈值，默认 1e-6 - damping (float): DLS 阻尼系数，默认 0.01 - num_initial_guesses (int): 多起点搜索数量，默认 1
    :return, 包含以下键的字典： - 'success': 是否收敛 (bool) - 'q': 求解结果关节角度 (ndarray, rad) - 'residual': 末端位姿残差范数 - 'iterations': 实际迭代次数 - 'method': 使用的求解方法名称
    """
    _ensure_robocore()
    ik_kwargs = dict(kwargs)
    ik_kwargs.setdefault('method', method)
    # Compatible aliases across RoboCore versions.
    if 'max_iterations' in ik_kwargs and 'max_iters' not in ik_kwargs:
        ik_kwargs['max_iters'] = ik_kwargs.pop('max_iterations')
    if 'tolerance' in ik_kwargs:
        tol = ik_kwargs.pop('tolerance')
        ik_kwargs.setdefault('pos_tol', tol)
        ik_kwargs.setdefault('ori_tol', tol)

    target_pose_norm = _normalize_target_pose(target_pose)
    result = _ik(robot_model, target_pose_norm, q0=q_init, **ik_kwargs)

    if isinstance(result, dict):
        output = {
            'success': result.get('success', False),
            'q': _to_numpy(result.get('q', np.zeros(0))),
            'residual': float(result.get('residual', float('inf'))),
            'iterations': int(result.get('iterations', 0)),
            'method': method,
        }
    else:
        q_result = _to_numpy(result)
        output = {
            'success': True,
            'q': q_result,
            'residual': 0.0,
            'iterations': 0,
            'method': method,
        }

    return output


def compute_jacobian(robot_model, q: List[float]) -> np.ndarray:
    """雅可比矩阵计算

    计算给定关节构型下的几何雅可比矩阵，表示关节速度到末端线速度和角速度的映射关系。

    :param robot_model, RoboCore 的 RobotModel 实例
    :param q, 关节角度列表 (rad)
    :return, 6xN 雅可比矩阵 (ndarray)，其中 N 为自由度数 前 3 行为线速度雅可比，后 3 行为角速度雅可比
    """
    _ensure_robocore()
    J = _jac(robot_model, q)
    return _to_numpy(J)
