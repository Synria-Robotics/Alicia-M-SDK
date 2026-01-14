# Copyright (c) 2025 Synria Robotics Co., Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""Shared utilities for trajectory planning demos.

This module provides common functions for waypoint loading, saving, and recording
that are shared between joint space and Cartesian space trajectory demos.
"""

import numpy as np
import json
import os
import time
from typing import Optional, Tuple, List, Callable, Any

from robocore.utils.beauty_logger import beauty_print, beauty_print_array
try:
    from robocore.utils.backend import to_numpy
except ImportError:
    # Fallback if to_numpy is not available
    def to_numpy(x):
        if hasattr(x, 'numpy'):
            return x.numpy()
        return np.asarray(x)
from robocore.transform import make_transform, quaternion_to_matrix, rpy_to_matrix, matrix_to_quaternion


def record_waypoints_manual(controller,
                            get_state_fn: Optional[Callable] = None,
                            format_fn: Optional[Callable] = None) -> List[Any]:
    """
    General manual waypoint recording function.

    :param controller: Robot controller
    :param get_state_fn: Custom state getter function, returns data to record. If None, uses default joint+gripper
    :param format_fn: Custom formatting function for log output. If None, uses default format
    :return: List of recorded waypoints
    """
    print("\n=== 手动记录模式 ===")
    print("关闭扭矩后，拖动到目标位置按回车记录")

    input("按回车开始...")
    controller.torque_control('off')
    print("[安全] 扭矩已关闭，可以拖动机械臂")

    waypoints = []

    try:
        while True:
            cmd = input(f"\n拖动到位置后按回车记录第{len(waypoints) + 1}个点，输入'q'结束: ").strip()
            if cmd.lower() == 'q':
                break

            # 使用自定义或默认的状态获取函数
            if get_state_fn:
                state = get_state_fn(controller)
            else:
                # 默认：记录关节角度和夹爪状态（使用统一API一次性获取）
                robot_state = controller.get_robot_state("joint_gripper")
                if robot_state is not None:
                    joints = robot_state.angles
                    gripper = robot_state.gripper
                else:
                    joints = None
                    gripper = 0.0
                state = {"t": time.time(), "q": joints, "grip": gripper} if joints is not None else None

            if state:
                waypoints.append(state)

                # 使用自定义或默认的格式化函数输出日志
                if format_fn:
                    format_fn(len(waypoints), state)  # Call function, don't print return value
                else:
                    # 默认格式
                    if isinstance(state, dict) and 'q' in state:
                        print(f"[记录] 第{len(waypoints)}个点: 关节{[round(j, 3) for j in state['q']]}, 夹爪{state.get('grip', 0):.3f}")
                    else:
                        print(f"[记录] 第{len(waypoints)}个点")

    finally:
        controller.torque_control('on')
        print("[安全] 扭矩已重新开启")

    return waypoints


def load_joint_waypoints_from_file(file_path: str) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Load joint waypoints from JSON file.
    
    :param file_path: Path to JSON file containing joint waypoints
    :return: Tuple of (waypoints_array, gripper_values) where:
             - waypoints_array: Array of joint angles [n_waypoints, n_dof]
             - gripper_values: Array of gripper values [n_waypoints] or None if not present
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Waypoint file not found: {file_path}")
    
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    waypoints = []
    gripper_values = []
    has_gripper = False
    
    # Check if data is a list
    if not isinstance(data, list):
        raise ValueError("JSON file must contain a list of waypoints")
    
    for i, wp in enumerate(data):
        if isinstance(wp, list):
            # Old format: list of joint angles only
            waypoints.append(np.array(wp, dtype=np.float64))
            gripper_values.append(None)
        elif isinstance(wp, dict):
            # New format: dict with 'joints' and optionally 'gripper'
            if 'joints' not in wp:
                raise ValueError(f"Waypoint {i+1}: dict must contain 'joints' key")
            waypoints.append(np.array(wp['joints'], dtype=np.float64))
            if 'gripper' in wp:
                gripper_values.append(float(wp['gripper']))
                has_gripper = True
            else:
                gripper_values.append(None)
        else:
            raise ValueError(f"Waypoint {i+1}: must be a list of joint angles or a dict with 'joints' key")
    
    if len(waypoints) < 2:
        raise ValueError("At least 2 waypoints are required")
    
    # Convert to numpy array and check consistency
    waypoints_array = np.array(waypoints)
    n_dof = waypoints_array.shape[1]
    
    # Check all waypoints have same DOF
    for i, wp in enumerate(waypoints):
        if len(wp) != n_dof:
            raise ValueError(f"Waypoint {i+1}: has {len(wp)} joints, expected {n_dof}")
    
    # Convert gripper values to array if all waypoints have gripper info
    if has_gripper:
        gripper_array = np.array([g if g is not None else 0.0 for g in gripper_values], dtype=np.float64)
        return waypoints_array, gripper_array
    else:
        return waypoints_array, None


def save_joint_waypoints_to_file(waypoints: np.ndarray, file_path: str, gripper_values: Optional[np.ndarray] = None):
    """Save joint waypoints to JSON file.
    
    :param waypoints: Array of joint angles [n_waypoints, n_dof]
    :param file_path: Path to save JSON file
    :param gripper_values: Optional array of gripper values [n_waypoints]
    """
    # Convert to list format for JSON serialization
    waypoints_list = []
    for i, wp in enumerate(waypoints):
        wp_list = wp.tolist() if isinstance(wp, np.ndarray) else list(wp)
        if gripper_values is not None and i < len(gripper_values):
            # New format: dict with joints and gripper
            waypoints_list.append({
                'joints': wp_list,
                'gripper': float(gripper_values[i])
            })
        else:
            # Old format: list of joint angles only
            waypoints_list.append(wp_list)
    
    with open(file_path, 'w') as f:
        json.dump(waypoints_list, f, indent=2)
    
    gripper_info = f" (with gripper)" if gripper_values is not None else ""
    beauty_print(f"Saved {len(waypoints)} waypoints{gripper_info} to {file_path}", type="success")


def record_joint_waypoints_manual(robot) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Record joint waypoints by manually dragging robot.
    
    :param robot: Robot controller instance
    :return: Tuple of (waypoints_array, gripper_values) where:
             - waypoints_array: Array of joint angles [n_waypoints, n_dof]
             - gripper_values: Array of gripper values [n_waypoints]
    """
    # Define custom state getter for joint waypoints
    def get_joint_state(controller):
        # Get joint and gripper together in a single call (more efficient)
        robot_state = controller.get_robot_state("joint_gripper")
        if robot_state is None:
            beauty_print("✗ 无法获取当前关节角度和夹爪状态", type="warning")
            return None
        
        joints = to_numpy(robot_state.angles)
        gripper = robot_state.gripper
        return {"q": joints, "grip": float(gripper)}
    
    # Define custom formatter for joint waypoints
    def format_joint_state(count, state):
        joints = state.get('q')
        gripper = state.get('grip', 0.0)
        beauty_print(f"[记录] 第{count}个点:")
        print(f"  关节角度 (rad): {beauty_print_array(joints)}")
        print(f"  关节角度 (deg): {beauty_print_array(np.rad2deg(joints))}")
        print(f"  夹爪状态: {gripper:.1f} (0-100, 0=闭合, 1000=张开)")
        return None  # Formatting is done via print
    
    # Use the general recording function
    beauty_print("\n=== 手动记录模式 ===", type="module", centered=False)
    beauty_print("关闭扭矩后，拖动到目标位置按回车记录")
    
    recorded_data = record_waypoints_manual(robot, get_state_fn=get_joint_state, format_fn=format_joint_state)
    
    if not recorded_data or len(recorded_data) < 2:
        beauty_print("至少需要2个waypoint才能生成轨迹", type="warning")
        return None, None
    
    # Convert to numpy arrays
    waypoints = []
    gripper_values = []
    for point in recorded_data:
        if point and 'q' in point:
            waypoints.append(to_numpy(point['q']))
            gripper_values.append(float(point.get('grip', 0.0)))
    
    return np.array(waypoints), np.array(gripper_values)


def load_cartesian_waypoints_from_file(file_path: str) -> List[np.ndarray]:
    """Load Cartesian waypoints from JSON file.
    
    :param file_path: Path to JSON file containing waypoints
    :return: List of 4x4 transformation matrices
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Waypoint file not found: {file_path}")
    
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    waypoints = []
    
    # Check if data is a list
    if not isinstance(data, list):
        raise ValueError("JSON file must contain a list of waypoints")
    
    for i, wp in enumerate(data):
        if isinstance(wp, list) and len(wp) == 4 and all(isinstance(row, list) and len(row) == 4 for row in wp):
            # Direct 4x4 matrix format
            waypoints.append(np.array(wp, dtype=np.float64))
        elif isinstance(wp, dict):
            # Position + orientation format
            if 'position' not in wp:
                raise ValueError(f"Waypoint {i+1}: missing 'position' field")
            
            pos = np.array(wp['position'], dtype=np.float64)
            if len(pos) != 3:
                raise ValueError(f"Waypoint {i+1}: position must be [x, y, z]")
            
            # Handle orientation
            if 'orientation' in wp:
                ori = wp['orientation']
                if isinstance(ori, list):
                    if len(ori) == 4:
                        # Quaternion [x, y, z, w]
                        R = quaternion_to_matrix(np.array(ori, dtype=np.float64))
                    elif len(ori) == 3:
                        # RPY [roll, pitch, yaw]
                        R = rpy_to_matrix(ori[0], ori[1], ori[2])
                    else:
                        raise ValueError(f"Waypoint {i+1}: orientation must be quaternion [x,y,z,w] or RPY [roll,pitch,yaw]")
                else:
                    raise ValueError(f"Waypoint {i+1}: orientation must be a list")
            else:
                # Default orientation: identity rotation
                R = np.eye(3)
            
            # Create 4x4 transform matrix
            T = make_transform(R, pos)
            waypoints.append(T)
        else:
            raise ValueError(f"Waypoint {i+1}: invalid format. Must be 4x4 matrix or dict with 'position' and optional 'orientation'")
    
    if len(waypoints) < 2:
        raise ValueError("At least 2 waypoints are required")
    
    return waypoints


def save_cartesian_waypoints_to_file(waypoints: np.ndarray, file_path: str):
    """Save Cartesian waypoints to JSON file.
    
    :param waypoints: Array of 4x4 transformation matrices [n_waypoints, 4, 4]
    :param file_path: Path to save JSON file
    """
    # Convert to dict format with position and orientation (quaternion) separately
    waypoints_list = []
    for wp in waypoints:
        wp = to_numpy(wp) if not isinstance(wp, np.ndarray) else wp
        
        # Extract position (translation)
        position = wp[:3, 3].tolist()
        
        # Extract rotation matrix and convert to quaternion
        rotation = wp[:3, :3]
        quaternion = matrix_to_quaternion(rotation).tolist()  # [x, y, z, w]
        
        # Save as dict with position and orientation
        waypoints_list.append({
            'position': position,
            'orientation': quaternion
        })
    
    with open(file_path, 'w') as f:
        json.dump(waypoints_list, f, indent=2)
    
    beauty_print(f"Saved {len(waypoints)} Cartesian waypoints to {file_path}", type="success")


def record_cartesian_waypoints_manual(robot) -> Optional[np.ndarray]:
    """Record Cartesian waypoints by manually dragging robot.
    
    :param robot: Robot controller instance
    :return: Array of 4x4 transformation matrices [n_waypoints, 4, 4]
    """
    beauty_print("\n=== 手动记录模式 ===", type="module", centered=False)
    beauty_print("关闭扭矩后，拖动到目标位置按回车记录")
    
    input("按回车开始...")
    robot.torque_control('off')
    beauty_print("[安全] 扭矩已关闭，可以拖动机械臂", type="warning")
    
    waypoints = []
    
    try:
        while True:
            cmd = input(f"\n拖动到位置后按回车记录第{len(waypoints) + 1}个点，输入'q'结束: ").strip()
            if cmd.lower() == 'q':
                break
            
            # Get current end-effector pose
            pose_info = robot.get_pose()
            if pose_info is None:
                beauty_print("✗ 无法获取当前末端执行器位姿", type="warning")
                continue
            
            # Get 4x4 transformation matrix
            T = to_numpy(pose_info['transform'])
            waypoints.append(T)
            
            # Extract position and orientation for display
            pos = pose_info['position']
            quat = pose_info['quaternion_xyzw']
            
            beauty_print(f"[记录] 第{len(waypoints)}个点:")
            print(f"  位置 (m): {beauty_print_array(pos)}")
            print(f"  四元数 (xyzw): {beauty_print_array(quat)}")
    
    finally:
        robot.torque_control('on')
        beauty_print("[安全] 扭矩已重新开启", type="success")
    
    if len(waypoints) < 2:
        beauty_print("至少需要2个waypoint才能生成轨迹", type="warning")
        return None
    
    return np.array(waypoints)


def handle_waypoint_recording(robot, args, waypoint_type: str = 'joint') -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Handle waypoint recording mode (common logic for both joint and Cartesian).
    
    :param robot: Robot controller instance
    :param args: Command line arguments
    :param waypoint_type: 'joint' or 'cartesian'
    :return: Tuple of (waypoints, gripper_values) where gripper_values is None for Cartesian
    """
    waypoints = None
    gripper_waypoints = None
    
    # Record by default unless --no-record is specified or --waypoints-file is provided
    should_record = not args.no_record and args.waypoints_file is None
    if should_record:
        if waypoint_type == 'joint':
            beauty_print("[0] Recording Joint Waypoints", type="module", centered=False)
            waypoints, gripper_waypoints = record_joint_waypoints_manual(robot)
        else:  # cartesian
            beauty_print("[0] Recording Cartesian Waypoints", type="module", centered=False)
            waypoints = record_cartesian_waypoints_manual(robot)
        
        if waypoints is None:
            robot.disconnect()
            return None, None
        
        # Save to file if specified
        if args.save_file:
            if waypoint_type == 'joint':
                save_joint_waypoints_to_file(waypoints, args.save_file, gripper_waypoints)
            else:
                save_cartesian_waypoints_to_file(waypoints, args.save_file)
            beauty_print(f"Waypoints saved to {args.save_file}", type="success")
        else:
            beauty_print("No save file specified. Use --save-file to save waypoints.", type="warning")
        
        # Ask if user wants to execute trajectory
        user_input = input("\nExecute trajectory with recorded waypoints? (y/n): ").strip().lower()
        if user_input != 'y':
            beauty_print("Exiting without execution.", type="info")
            robot.disconnect()
            return None, None
    
    return waypoints, gripper_waypoints


def load_or_generate_joint_waypoints(robot, robot_model, args) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Load or generate joint waypoints.
    
    :param robot: Robot controller instance
    :param robot_model: Robot model instance
    :param args: Command line arguments
    :return: Tuple of (waypoints, gripper_waypoints)
    """
    from robocore.utils.backend import to_numpy
    
    if args.waypoints_file:
        beauty_print(f"Loading waypoints from file: {args.waypoints_file}")
        waypoints, gripper_waypoints = load_joint_waypoints_from_file(args.waypoints_file)
        beauty_print(f"Successfully loaded {len(waypoints)} waypoints from file")
        if gripper_waypoints is not None:
            beauty_print(f"Gripper values loaded: {len(gripper_waypoints)} waypoints")
        return waypoints, gripper_waypoints
    
    # Generate random waypoints
    beauty_print(f"Generating {args.num_waypoints} random waypoints within joint limits...")
    waypoints = []
    gripper_waypoints = []
    
    # Get current joint angles and gripper as starting point (optional)
    if args.use_current_joints:
        # Get joint and gripper together in a single call (more efficient)
        robot_state = robot.get_robot_state("joint_gripper")
        if robot_state is not None:
            q_start = robot_state.angles
            g_start = robot_state.gripper
            waypoints.append(to_numpy(q_start))
            gripper_waypoints.append(float(g_start) if g_start is not None else 500.0)
            beauty_print(f"Using current joint angles and gripper as first waypoint:")
            print(f"  Current joints (rad): {beauty_print_array(q_start)}")
            print(f"  Current joints (deg): {beauty_print_array(np.rad2deg(q_start))}")
            print(f"  Current gripper: {g_start:.1f} (0-100)" if g_start is not None else "  Current gripper: N/A")
            num_random = args.num_waypoints - 1
        else:
            beauty_print("✗ 无法获取当前关节角度，使用随机生成", type="warning")
            num_random = args.num_waypoints
    else:
        num_random = args.num_waypoints
    
    # Generate random waypoints
    for i in range(num_random):
        waypoint_seed = args.seed + i if args.seed is not None else None
        q = to_numpy(robot_model.random_q(seed=waypoint_seed, scale=args.joint_scale))
        waypoints.append(q)
        # Random gripper value between 0 and 100
        if waypoint_seed is not None:
            np.random.seed(waypoint_seed)
        gripper_waypoints.append(float(np.random.uniform(0, 100)))
    
    return np.array(waypoints), np.array(gripper_waypoints) if gripper_waypoints else None


def load_or_generate_cartesian_waypoints(robot_model, args) -> np.ndarray:
    """Load or generate Cartesian waypoints.
    
    :param robot_model: Robot model instance
    :param args: Command line arguments
    :return: Array of waypoint poses [n_waypoints, 4, 4]
    """
    from robocore.utils.backend import to_numpy
    
    if args.waypoints_file:
        beauty_print(f"Loading waypoints from file: {args.waypoints_file}")
        waypoints_list = load_cartesian_waypoints_from_file(args.waypoints_file)
        waypoints = np.array([to_numpy(wp) for wp in waypoints_list])
        beauty_print(f"Successfully loaded {len(waypoints)} waypoints from file")
        return waypoints
    
    # Generate random poses
    beauty_print(f"Generating {args.num_waypoints} random waypoints within workspace...")
    waypoints_list = robot_model.random_pose_batch(
        batch_size=args.num_waypoints,
        seed=args.seed,
        scale=args.workspace_scale
    )
    return np.array([to_numpy(wp) for wp in waypoints_list])


def display_joint_waypoints(waypoints: np.ndarray, gripper_waypoints: Optional[np.ndarray] = None):
    """Display joint waypoints information.
    
    :param waypoints: Array of joint angles [n_waypoints, n_dof]
    :param gripper_waypoints: Optional array of gripper values
    """
    beauty_print(f"Waypoints: {len(waypoints)}")
    for i, wp in enumerate(waypoints):
        print(f"  Waypoint {i+1}: {beauty_print_array(wp)} (rad)")
        print(f"              {beauty_print_array(np.rad2deg(wp))} (deg)")
        if gripper_waypoints is not None and i < len(gripper_waypoints):
            print(f"              夹爪: {gripper_waypoints[i]:.1f} (0-100)")


def display_cartesian_waypoints(waypoints: np.ndarray):
    """Display Cartesian waypoints information.
    
    :param waypoints: Array of waypoint poses [n_waypoints, 4, 4]
    """
    beauty_print(f"Waypoints: {len(waypoints)}")
    for i, wp in enumerate(waypoints):
        pos = wp[:3, 3]
        print(f"  Waypoint {i+1}: position = {beauty_print_array(pos)}")


def display_joint_trajectory_stats(trajectory: dict):
    """Display joint trajectory statistics.
    
    :param trajectory: Trajectory dictionary with 't', 'q', 'qd', 'qdd', etc.
    """
    beauty_print(f"Trajectory generated:")
    print(f"  Duration: {trajectory['t'][-1]:.3f} s")
    print(f"  Points: {len(trajectory['t'])}")
    print(f"  Max velocity: {np.max(np.abs(trajectory['qd'])):.3f} rad/s")
    print(f"  Max acceleration: {np.max(np.abs(trajectory['qdd'])):.3f} rad/s²")
    if 'qddd' in trajectory:
        print(f"  Max jerk: {np.max(np.abs(trajectory['qddd'])):.3f} rad/s³")
    
    # Display gripper trajectory if available
    gripper_trajectory = trajectory.get('gripper', None)
    if gripper_trajectory is not None:
        beauty_print(f"Gripper trajectory interpolated: {len(gripper_trajectory)} points")
        print(f"  Gripper range: [{np.min(gripper_trajectory):.1f}, {np.max(gripper_trajectory):.1f}]")


def display_cartesian_trajectory_stats(trajectory: dict):
    """Display Cartesian trajectory statistics.
    
    :param trajectory: Trajectory dictionary with 't', 'positions', 'orientations', 'velocities', etc.
    """
    beauty_print(f"Trajectory generated:")
    print(f"  Duration: {trajectory['t'][-1]:.3f} s")
    print(f"  Points: {len(trajectory['t'])}")
    print(f"  Max linear velocity: {np.max(np.linalg.norm(trajectory['velocities'][:, :3], axis=1)):.3f} m/s")
    print(f"  Max angular velocity: {np.max(np.linalg.norm(trajectory['velocities'][:, 3:], axis=1)):.3f} rad/s")


def verify_cartesian_waypoints(trajectory: dict, waypoints: np.ndarray):
    """Verify that trajectory passes through waypoints.
    
    :param trajectory: Trajectory dictionary
    :param waypoints: Array of waypoint poses [n_waypoints, 4, 4]
    """
    from robocore.transform.se3 import get_rotation
    from robocore.transform.conversions import matrix_to_axis_angle
    
    beauty_print("Verifying waypoints are passed through:")
    waypoint_positions = trajectory.get('waypoints', np.array([wp[:3, 3] for wp in waypoints]))
    trajectory_positions = trajectory['positions']
    trajectory_orientations = trajectory['orientations']
    
    # Check first waypoint
    error_first_pos = np.linalg.norm(trajectory_positions[0] - waypoint_positions[0])
    R1 = trajectory_orientations[0]
    R2 = get_rotation(waypoints[0])
    R_diff = R1 @ R2.T
    axis, angle = matrix_to_axis_angle(R_diff)
    error_first_ori = abs(angle)
    status_first = "✓" if error_first_pos < 1e-5 and error_first_ori < 1e-5 else "✗"
    print(f"  Waypoint 1 (first, t=0.000s): {status_first} pos_err={error_first_pos:.6e} m, ori_err={error_first_ori:.6e} rad")
    
    # Check last waypoint
    error_last_pos = np.linalg.norm(trajectory_positions[-1] - waypoint_positions[-1])
    R1 = trajectory_orientations[-1]
    R2 = get_rotation(waypoints[-1])
    R_diff = R1 @ R2.T
    axis, angle = matrix_to_axis_angle(R_diff)
    error_last_ori = abs(angle)
    status_last = "✓" if error_last_pos < 1e-5 and error_last_ori < 1e-5 else "✗"
    print(f"  Waypoint {len(waypoints)} (last, t={trajectory['t'][-1]:.3f}s): {status_last} pos_err={error_last_pos:.6e} m, ori_err={error_last_ori:.6e} rad")
    
    # Check intermediate waypoints
    if len(waypoints) > 2:
        u_interp = trajectory['t'] / trajectory['t'][-1]
        u_waypoints = np.zeros(len(waypoints))
        for i in range(1, len(waypoints)):
            u_waypoints[i] = u_waypoints[i - 1] + np.linalg.norm(waypoint_positions[i] - waypoint_positions[i - 1])
        u_waypoints = u_waypoints / u_waypoints[-1] if u_waypoints[-1] > 0 else np.linspace(0, 1, len(waypoints))
        
        for wp_idx in range(1, len(waypoints) - 1):
            u_wp = u_waypoints[wp_idx]
            closest_idx = np.argmin(np.abs(u_interp - u_wp))
            actual_pos = trajectory_positions[closest_idx]
            expected_pos = waypoint_positions[wp_idx]
            error = np.linalg.norm(actual_pos - expected_pos)
            status = "✓" if error < 1e-2 else "✗"
            print(f"  Waypoint {wp_idx+1} (intermediate, u={u_wp:.3f}): {status} error={error:.6f} m")
            if error >= 1e-2:
                print(f"    Expected: {beauty_print_array(expected_pos)}")
                print(f"    Actual:   {beauty_print_array(actual_pos)}")


def display_ik_results(ik_result: dict, trajectory: dict, num_samples: int = 3):
    """Display IK solving results and statistics.
    
    :param ik_result: IK result dictionary from solve_ik_for_trajectory
    :param trajectory: Trajectory dictionary
    :param num_samples: Number of sample results to display
    """
    joint_angles = ik_result['joint_angles']
    ik_results = ik_result['ik_results']
    success_count = ik_result['statistics']['successful']
    success_rate = ik_result['success_rate']
    
    beauty_print(f"IK sequential computation completed:")
    print(f"  Total time: {ik_result['statistics']['ik_time']:.4f} s")
    print(f"  Average time per pose: {ik_result['statistics']['avg_time_per_pose']*1000:.4f} ms")
    
    # Display sample results
    beauty_print(f"First {num_samples} IK results:")
    for i in range(min(num_samples, len(ik_results))):
        result = ik_results[i]
        print(f"  Point {i+1}: success={result['success']}, "
              f"pos_err={result.get('pos_err', 0.0):.6e}, "
              f"ori_err={result.get('ori_err', 0.0):.6e}, "
              f"iters={result.get('iters', 0)}")
    
    # Display joint angle statistics
    beauty_print(f"Joint Angle Statistics:")
    print(f"  Shape: {joint_angles.shape}")
    print(f"  Min values: {beauty_print_array(np.min(joint_angles, axis=0))}")
    print(f"  Max values: {beauty_print_array(np.max(joint_angles, axis=0))}")
    print(f"  Mean values: {beauty_print_array(np.mean(joint_angles, axis=0))}")
    
    # Display sample joint angles
    beauty_print(f"Sample Joint Angles (first 5 points):")
    for i in range(min(5, len(joint_angles))):
        print(f"  Point {i+1} (t={trajectory['t'][i]:.3f}s): {beauty_print_array(joint_angles[i])}")
    
    if len(joint_angles) > 5:
        beauty_print(f"Sample Joint Angles (last 5 points):")
        for i in range(max(0, len(joint_angles)-5), len(joint_angles)):
            print(f"  Point {i+1} (t={trajectory['t'][i]:.3f}s): {beauty_print_array(joint_angles[i])}")
    
    # Display error statistics
    if success_count > 0:
        pos_errors = [r['pos_err'] for r in ik_results if r['success']]
        ori_errors = [r['ori_err'] for r in ik_results if r['success']]
        beauty_print(f"Error Statistics (successful cases):")
        print(f"  Position error - Mean: {np.mean(pos_errors):.6e} m, Max: {np.max(pos_errors):.6e} m")
        print(f"  Orientation error - Mean: {np.mean(ori_errors):.6e} rad, Max: {np.max(ori_errors):.6e} rad")
    
    beauty_print(f"IK Success Rate: {success_count}/{len(ik_results)} ({success_rate*100:.1f}%)")


def plot_trajectory(trajectory: dict, waypoints: np.ndarray, plot_type: str = 'joint', 
                   joint_angles: Optional[np.ndarray] = None, ik_results: Optional[list] = None):
    """Plot trajectory visualization.
    
    :param trajectory: Trajectory dictionary
    :param waypoints: Waypoints array
    :param plot_type: 'joint' or 'cartesian'
    :param joint_angles: Optional joint angles for Cartesian plotting
    :param ik_results: Optional IK results for Cartesian plotting
    """
    try:
        import matplotlib.pyplot as plt
        if plot_type == 'joint':
            from robocore.planning import plot_joint_trajectory
            plot_joint_trajectory(trajectory, waypoints)
        else:  # cartesian
            from robocore.planning import plot_cartesian_with_ik
            plot_cartesian_with_ik(trajectory, waypoints, joint_angles, ik_results)
        plt.show(block=False)
        plt.pause(0.1)
    except ImportError:
        beauty_print("matplotlib not installed. Skipping plots.", type="warning")
