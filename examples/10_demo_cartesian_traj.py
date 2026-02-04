#!/usr/bin/env python3
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

"""Cartesian Spline Trajectory Planning with Inverse Kinematics

This demo demonstrates:
1. Generating a smooth spline trajectory in Cartesian space through multiple waypoints
2. Solving inverse kinematics for all poses in the trajectory (batch IK)
3. Executing the trajectory on the robot
"""

import numpy as np
import argparse

import alicia_m_sdk
from alicia_m_sdk.execution import CartesianTrajectoryExecutor
from alicia_m_sdk.hardware import ServoDriver
import robocore as rc
from robocore.transform import make_transform
from robocore.utils.beauty_logger import beauty_print, beauty_print_array
from robocore.utils.backend import to_numpy

from alicia_m_sdk.utils.trajectory_utils import (
    handle_waypoint_recording,
    load_or_generate_cartesian_waypoints,
    display_cartesian_waypoints,
    display_cartesian_trajectory_stats,
    verify_cartesian_waypoints,
    display_ik_results,
    plot_trajectory
)


def main(args):
    """Main function for Cartesian space trajectory planning and execution."""
    # [0] Initialize robot connection
    beauty_print("Cartesian Spline Planning with IK Batch Solver", type="module")
    
    robot = alicia_m_sdk.create_robot(
        port=args.port,
        version=args.version,
        base_link=args.base_link,
        end_link=args.end_link,
        control_aim=args.control_aim,
        control_mode="mit" if args.use_mit_mode else "pv"
    )
    rc.set_backend(args.backend, device=args.device)
    robot_model = robot.robot_model

    # [1] Handle waypoint recording or loading/generation
    waypoints, _ = handle_waypoint_recording(robot, args, waypoint_type='cartesian')
    if waypoints is None:
        # Load or generate waypoints
        try:
            waypoints = load_or_generate_cartesian_waypoints(robot_model, args)
        except Exception as e:
            beauty_print(f"Failed to load/generate waypoints: {e}", type="error")
            robot.disconnect()
            return
    
    display_cartesian_waypoints(waypoints)

    # [2] Generate Cartesian trajectory
    beauty_print("[2] Generating Spline Trajectory", type="module", centered=False)
    
    trajectory = robot.plan_cartesian_trajectory(
        waypoints=waypoints,
        duration=args.duration,
        num_points=args.num_points,
        backend='numpy'  # Use numpy for smooth cubic spline interpolation
    )

    display_cartesian_trajectory_stats(trajectory)
    verify_cartesian_waypoints(trajectory, waypoints)

    # [3] Extract poses and solve IK
    if 'poses' in trajectory:
        target_poses = trajectory['poses']
    else:
        # Build poses from positions and orientations
        positions = trajectory['positions']
        orientations = trajectory['orientations']
        target_poses = np.array([make_transform(orientations[i], positions[i]) 
                                for i in range(len(positions))])

    beauty_print("[3] Solving Inverse Kinematics", type="module", centered=False)
    
    # Get initial joint configuration
    q0 = robot.get_robot_state("joint") if args.init_strategy == 'current' else None
    actual_strategy = 'random' if (q0 is not None and args.init_strategy == 'current') else args.init_strategy
    
    if q0 is not None:
        beauty_print(f"Using current joint angles as initial guess:")
        print(f"  Current joints (rad): {beauty_print_array(q0)}")
        print(f"  Current joints (deg): {beauty_print_array(np.rad2deg(q0))}")

    # Solve IK for all poses
    ik_result = robot.solve_ik_for_trajectory(
        target_poses=target_poses,
        q_init=q0,
        method=args.method,
        max_iters=args.max_iters,
        pos_tol=args.pos_tol,
        ori_tol=args.ori_tol,
        num_initial_guesses=args.num_inits,
        initial_guess_strategy=actual_strategy,
        initial_guess_scale=args.init_scale,
        random_seed=args.seed,
        backend=args.backend,
        use_previous_solution=True
    )

    display_ik_results(ik_result, trajectory)
    beauty_print("✓ Cartesian spline planning with IK batch solver completed!", type="success")

    joint_angles = ik_result['joint_angles']
    ik_results = ik_result['ik_results']
    success_rate = ik_result['success_rate']

    # [4] Plot trajectory (optional)
    if args.plot:
        beauty_print("[4] Plotting Trajectory", type="module", centered=False)
        plot_trajectory(trajectory, waypoints, plot_type='cartesian', 
                       joint_angles=joint_angles, ik_results=ik_results)
    
    input("\nPress Enter to start trajectory execution...")

    # [5] Execute trajectory
    beauty_print("[5] Executing Trajectory on Robot", type="module", centered=False)
    
    executor = CartesianTrajectoryExecutor(
        robot=robot,
        speed_deg_s=args.speed_deg_s,
        tolerance=0.5,
        timeout=args.timeout,
        progress_interval=50,
        initial_delay=0.1,
        wait_for_completion=False,
        use_timing=True,
        use_mit_mode=args.use_mit_mode,
        playback_hz=args.playback_hz
    )
    
    exec_result = executor.execute(
        joint_angles=joint_angles,
        trajectory_times=to_numpy(trajectory['t']),
        gripper_values=None,
        initial_tolerance=0.5,
        ik_success_rate=success_rate,
        min_success_rate=0.8
    )
    
    if exec_result.get('cancelled', False):
        robot.disconnect()
        return

    robot.disconnect()
    return {
        'trajectory': trajectory,
        'joint_angles': joint_angles,
        'ik_results': ik_results,
        'success_rate': success_rate,
        'waypoints': waypoints
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Cartesian Spline Planning with IK Batch Solver')
    
    # Robot connection
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--version', type=str, default="v1_1", help="机械臂版本 (可选: v1_0, v1_1，默认: v1_1)")
    parser.add_argument('--base_link', type=str, default="base_link", help="基座链路名称")
    parser.add_argument('--end_link', type=str, default="tool0", help="末端执行器链路名称")
    
    # Motor-specific control settings
    parser.add_argument('--control-aim', type=str, default='operation', choices=['teach', 'operation'],
                        help='Control aim: teach (0x01示教臂) or operation (0x02操作臂) (默认: operation)')
    parser.add_argument('--control-mode', type=str, default='pv', choices=['pv', 'pvt', 'v', 'mit', 'mit_position', 'mit_speed', 'mit_torque'],
                        help='Control mode: pv, pvt, v, mit, mit_position, mit_speed, mit_torque (默认: pv)')
    parser.add_argument('--use-mit-mode', action='store_true', 
                        help='Use MIT position mode for high-frequency playback (motor-specific)')
    parser.add_argument('--playback-hz', type=float, default=200.0,
                        help='Playback frequency in Hz for MIT position mode (default: 200Hz)')
    
    # Waypoint settings
    parser.add_argument('--no-record', action='store_true', help='Disable recording mode')
    parser.add_argument('--save-file', type=str, default=None, help='Path to save recorded waypoints')
    parser.add_argument('--waypoints-file', type=str, default=None, 
                        help='Path to JSON file with waypoints (4x4 matrix or dict with position/orientation)')
    parser.add_argument('--num-waypoints', type=int, default=2, help='Number of waypoints for random generation')
    parser.add_argument('--workspace-scale', type=float, default=0.6, help='Workspace scale (0.0-1.0)')
    
    # Trajectory planning
    parser.add_argument('--duration', type=float, default=1.0, help='Trajectory duration (seconds)')
    parser.add_argument('--num-points', type=int, default=10, help='Number of trajectory points')
    
    # IK settings
    parser.add_argument('--method', type=str, default='dls', choices=['dls', 'pinv', 'transpose'], help='IK method')
    parser.add_argument('--max-iters', type=int, default=100, help='Maximum IK iterations')
    parser.add_argument('--pos-tol', type=float, default=1e-2, help='Position tolerance (m)')
    parser.add_argument('--ori-tol', type=float, default=1e-2, help='Orientation tolerance (rad)')
    parser.add_argument('--init-scale', type=float, default=0.6, help='Initial guess scale (0.0-1.0)')
    parser.add_argument('--num-inits', type=int, default=5, help='Number of initial guesses')
    parser.add_argument('--init-strategy', type=str, default='current',
                        choices=['zero', 'random', 'sobol', 'latin', 'center', 'uniform', 'current'],
                        help='Initial guess strategy')
    parser.add_argument('--seed', type=int, default=666, help='Random seed')
    
    # Execution
    parser.add_argument('--speed_deg_s', type=int, default=20, help="关节运动速度 (单位: 度/秒，默认: 20，范围: 10-400度/秒)")
    parser.add_argument('--timeout', type=float, default=10.0, help='Timeout per command (seconds)')
    
    # Other
    parser.add_argument('--backend', type=str, default='numpy', choices=['numpy', 'torch'], help='Backend')
    parser.add_argument('--device', type=str, default='cpu', help='Device')
    parser.add_argument('--plot', action='store_false', help='Plot trajectory visualization')
    
    main(parser.parse_args())
