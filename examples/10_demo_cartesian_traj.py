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

"""Cartesian Spline Trajectory Planning with Inverse Kinematics (MoveIt-style Plan → Execute)

This demo demonstrates:
1. Defining Cartesian waypoints (record, random, or load from file)
2. Planning a smooth spline trajectory + batch IK solving
3. Reviewing the plan (stats + optional plot)
4. Executing the trajectory on the robot

Per-segment control: duration and interpolation points are specified per segment
(between two consecutive waypoints), then summed for the global Cartesian spline.
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
    # ── [0] Connect ──────────────────────────────────────────────
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

    # ── [1] Waypoints ────────────────────────────────────────────
    waypoints, _ = handle_waypoint_recording(robot, args, waypoint_type='cartesian')
    if waypoints is None:
        try:
            waypoints = load_or_generate_cartesian_waypoints(robot_model, args)
        except Exception as e:
            beauty_print(f"Failed to load/generate waypoints: {e}", type="error")
            robot.disconnect()
            return
    
    display_cartesian_waypoints(waypoints)
    n_segments = max(len(waypoints) - 1, 1)

    # ── [2] Plan (Cartesian spline + IK) ─────────────────────────
    beauty_print("[2] Planning Cartesian Spline Trajectory", type="module", centered=False)

    total_duration = args.duration_per_segment * n_segments
    total_points = args.num_points_per_segment * n_segments
    print(f"  Segments       : {n_segments}")
    print(f"  Per-segment    : {args.duration_per_segment:.1f}s, {args.num_points_per_segment} pts")
    print(f"  Total planned  : {total_duration:.1f}s, {total_points} pts")
    print(f"  Cmd frequency  : {total_points / total_duration:.0f} Hz")

    trajectory = robot.plan_cartesian_trajectory(
        waypoints=waypoints,
        duration=total_duration,
        num_points=total_points,
        backend='numpy'
    )

    display_cartesian_trajectory_stats(trajectory)
    verify_cartesian_waypoints(trajectory, waypoints)

    # ── [3] Solve IK ────────────────────────────────────────────
    if 'poses' in trajectory:
        target_poses = trajectory['poses']
    else:
        positions = trajectory['positions']
        orientations = trajectory['orientations']
        target_poses = np.array([make_transform(orientations[i], positions[i]) 
                                for i in range(len(positions))])

    beauty_print("[3] Solving Inverse Kinematics", type="module", centered=False)
    
    q0 = robot.get_robot_state("joint") if args.init_strategy == 'current' else None
    actual_strategy = 'random' if (q0 is not None and args.init_strategy == 'current') else args.init_strategy
    
    if q0 is not None:
        beauty_print(f"Using current joint angles as initial guess:")
        print(f"  Current joints (rad): {beauty_print_array(q0)}")
        print(f"  Current joints (deg): {beauty_print_array(np.rad2deg(q0))}")

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

    joint_angles = ik_result['joint_angles']
    ik_results = ik_result['ik_results']
    success_rate = ik_result['success_rate']

    # ── [4] Review (optional plot) ───────────────────────────────
    if args.plot:
        beauty_print("[4] Plotting Trajectory", type="module", centered=False)
        plot_trajectory(trajectory, waypoints, plot_type='cartesian', 
                       joint_angles=joint_angles, ik_results=ik_results)
    
    beauty_print("Plan complete. Review the trajectory above.", type="success")
    input("\nPress Enter to execute on robot (Ctrl+C to cancel)...")

    # ── [5] Execute ──────────────────────────────────────────────
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
    parser = argparse.ArgumentParser(
        description='Cartesian Spline Planning with IK Batch Solver (MoveIt-style)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  # Per-segment control (default): 2 waypoints, 5s per segment, 100 pts per segment
  python 10_demo_cartesian_traj.py --no-record

  # Custom per-segment timing
  python 10_demo_cartesian_traj.py --no-record --duration-per-segment 3.0 --num-points-per-segment 150
""")
    
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
    
    # Trajectory planning (per-segment)
    parser.add_argument('--duration-per-segment', type=float, default=5.0,
                        help='Duration per segment in seconds (default: 5.0)')
    parser.add_argument('--num-points-per-segment', type=int, default=100,
                        help='Interpolation points per segment (default: 100)')
    
    # IK settings
    parser.add_argument('--method', type=str, default='dls', choices=['dls', 'pinv', 'transpose'], help='IK method')
    parser.add_argument('--max-iters', type=int, default=100, help='Maximum IK iterations')
    parser.add_argument('--pos-tol', type=float, default=1e-2, help='Position tolerance (m)')
    parser.add_argument('--ori-tol', type=float, default=1e-2, help='Orientation tolerance (rad)')
    parser.add_argument('--init-scale', type=float, default=0.6, help='Initial guess scale (0.0-1.0)')
    parser.add_argument('--num-inits', type=int, default=2, help='Number of initial guesses')
    parser.add_argument('--init-strategy', type=str, default='current',
                        choices=['zero', 'random', 'sobol', 'latin', 'center', 'uniform', 'current'],
                        help='Initial guess strategy')
    parser.add_argument('--seed', type=int, default=666, help='Random seed')
    
    # Execution
    parser.add_argument('--speed_deg_s', type=int, default=100, help="关节运动速度 (单位: 度/秒，默认: 100，范围: 10-400度/秒)")
    parser.add_argument('--timeout', type=float, default=10.0, help='Timeout per command (seconds)')
    
    # Other
    parser.add_argument('--backend', type=str, default='numpy', choices=['numpy', 'torch'], help='Backend')
    parser.add_argument('--device', type=str, default='cpu', help='Device')
    parser.add_argument('--plot', action='store_false', help='Plot trajectory visualization')
    
    main(parser.parse_args())
