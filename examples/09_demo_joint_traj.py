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

"""Joint Space Trajectory Planning and Execution (MoveIt-style Plan → Execute)

This demo demonstrates:
1. Defining waypoints (record, random, or load from file)
2. Planning smooth trajectory with per-segment duration and interpolation
3. Reviewing the plan (stats + optional plot)
4. Executing the trajectory on the robot

Default planner: multi_segment (quintic polynomial per segment)
  - Each segment between two waypoints has independent duration and point count
  - Smooth C2-continuous transitions at waypoints (zero velocity/acceleration)
"""

import numpy as np
import argparse

import alicia_m_sdk
from alicia_m_sdk.execution import JointTrajectoryExecutor
from alicia_m_sdk.hardware import ServoDriver
import robocore as rc
from robocore.utils.beauty_logger import beauty_print
from robocore.utils.backend import to_numpy

from alicia_m_sdk.utils.trajectory_utils import (
    handle_waypoint_recording,
    load_or_generate_joint_waypoints,
    display_joint_waypoints,
    display_joint_trajectory_stats,
    plot_trajectory
)


def main(args):
    """Main function for joint space trajectory planning and execution."""
    # ── [0] Connect ──────────────────────────────────────────────
    beauty_print("Joint Space Trajectory Planning", type="module")
    
    robot = alicia_m_sdk.create_robot(
        port=args.port,
        version=args.version,
        base_link=args.base_link,
        end_link=args.end_link,
        control_aim=args.control_aim,
        control_mode=args.control_mode
    )
    rc.set_backend(args.backend, device=args.device)
    robot_model = robot.robot_model

    # ── [1] Waypoints ────────────────────────────────────────────
    waypoints, gripper_waypoints = handle_waypoint_recording(robot, args, waypoint_type='joint')
    if waypoints is None:
        try:
            waypoints, gripper_waypoints = load_or_generate_joint_waypoints(robot, robot_model, args)
        except Exception as e:
            beauty_print(f"Failed to load/generate waypoints: {e}", type="error")
            robot.disconnect()
            return
    
    display_joint_waypoints(waypoints, gripper_waypoints)
    n_segments = len(waypoints) - 1

    # ── [2] Plan ─────────────────────────────────────────────────
    beauty_print("[2] Planning Trajectory", type="module", centered=False)

    if args.planner == 'multi_segment':
        planner_name = f"Multi-Segment ({args.segment_method})"
        total_duration = args.duration_per_segment * n_segments
        total_points = args.num_points_per_segment * n_segments
        print(f"  Planner        : {planner_name}")
        print(f"  Segments       : {n_segments}")
        print(f"  Per-segment    : {args.duration_per_segment:.1f}s, {args.num_points_per_segment} pts")
        print(f"  Total planned  : {total_duration:.1f}s, {total_points} pts")
        print(f"  Cmd frequency  : {total_points / total_duration:.0f} Hz")
    else:
        planner_name = f"B-Spline (degree={args.bspline_degree})"
        print(f"  Planner        : {planner_name}")
        print(f"  Duration       : {args.duration:.1f}s")
        print(f"  Points         : {args.num_points}")

    trajectory = robot.plan_joint_trajectory(
        waypoints=waypoints,
        planner_type=args.planner,
        duration=args.duration if args.planner == 'b_spline' else None,
        num_points=args.num_points if args.planner == 'b_spline' else None,
        bspline_degree=args.bspline_degree,
        segment_method=args.segment_method,
        duration_per_segment=args.duration_per_segment if args.planner == 'multi_segment' else None,
        num_points_per_segment=args.num_points_per_segment if args.planner == 'multi_segment' else None,
        gripper_waypoints=gripper_waypoints
    )

    display_joint_trajectory_stats(trajectory)
    gripper_trajectory = trajectory.get('gripper', None)

    # ── [3] Review (optional plot) ───────────────────────────────
    if args.plot:
        beauty_print("[3] Plotting Trajectory", type="module", centered=False)
        plot_trajectory(trajectory, waypoints, plot_type='joint')
    
    beauty_print("Plan complete. Review the trajectory above.", type="success")
    input("\nPress Enter to execute on robot (Ctrl+C to cancel)...")

    # ── [4] Execute ──────────────────────────────────────────────
    beauty_print("[4] Executing Trajectory on Robot", type="module", centered=False)
    
    executor = JointTrajectoryExecutor(
        robot=robot,
        speed=args.speed,
        tolerance=0.5,
        timeout=args.timeout,
        progress_interval=50,
        initial_delay=2.0,
        wait_for_completion=False,
        use_timing=True,
        use_mit_mode=args.use_mit_mode,
        playback_hz=args.playback_hz
    )
    
    executor.execute(
        joint_angles=to_numpy(trajectory['q']),
        trajectory_times=to_numpy(trajectory['t']),
        gripper_values=gripper_trajectory,
        initial_tolerance=0.1,
    )

    robot.disconnect()
    return {'trajectory': trajectory, 'waypoints': waypoints}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Joint Space Trajectory Planning and Execution (MoveIt-style)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  # Per-segment control (default): 6 waypoints, 3s per segment, 100 pts per segment
  python 09_demo_joint_traj.py --no-record

  # Custom per-segment timing
  python 09_demo_joint_traj.py --no-record --duration-per-segment 4.0 --num-points-per-segment 150

  # Global B-Spline mode (single smooth curve through all waypoints)
  python 09_demo_joint_traj.py --no-record --planner b_spline --duration 10.0 --num-points 500
""")
    
    # Robot connection
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--version', type=str, default="v1_1", help="机械臂版本 (可选: v1_0, v1_1，默认: v1_1)")
    parser.add_argument('--base_link', type=str, default="base_link", help="基座链路名称")
    parser.add_argument('--end_link', type=str, default="tool0", help="末端执行器链路名称")
    
    # Motor-specific control settings
    parser.add_argument('--control-aim', type=str, default='operation', choices=['teach', 'operation'],
                        help='Control aim: teach (0x01示教臂) or operation (0x02操作臂) (默认: operation)')
    parser.add_argument('--control-mode', type=str, default='pv', choices=['pv', 'mit'],
                        help='Control mode: pv or mit (默认: pv)')
    parser.add_argument('--playback-hz', type=float, default=200.0,
                        help='Playback frequency in Hz for MIT position mode (default: 200Hz)')
    
    # Waypoint settings
    parser.add_argument('--no-record', action='store_true', help='Disable recording mode')
    parser.add_argument('--save-file', type=str, default=None, help='Path to save recorded waypoints')
    parser.add_argument('--waypoints-file', type=str, default=None, help='Path to JSON file with waypoints')
    parser.add_argument('--num-waypoints', type=int, default=6, help='Number of waypoints for random generation')
    parser.add_argument('--joint-scale', type=float, default=0.6, help='Scale factor for random joints (0.0-1.0)')
    parser.add_argument('--use-current-joints', action='store_true', help='Use current joints as first waypoint')
    
    # Trajectory planning (per-segment by default)
    parser.add_argument('--planner', type=str, default='multi_segment', choices=['b_spline', 'multi_segment'],
                        help='Planner type (default: multi_segment)')
    parser.add_argument('--duration-per-segment', type=float, default=3.0,
                        help='Duration per segment in seconds (multi_segment, default: 3.0)')
    parser.add_argument('--num-points-per-segment', type=int, default=100,
                        help='Interpolation points per segment (multi_segment, default: 100)')
    parser.add_argument('--segment-method', type=str, default='quintic', choices=['cubic', 'quintic'],
                        help='Polynomial method per segment (default: quintic)')
    # B-Spline fallback (global)
    parser.add_argument('--duration', type=float, default=10.0, help='Total trajectory duration (b_spline)')
    parser.add_argument('--num-points', type=int, default=500, help='Total interpolation points (b_spline)')
    parser.add_argument('--bspline-degree', type=int, default=5, choices=[3, 5], help='B-Spline degree')
    
    # Execution
    parser.add_argument('--speed', type=int, default=70, help="关节运动速度 (默认: 70，范围: 0-400)")
    parser.add_argument('--timeout', type=float, default=10.0, help='Timeout per command (seconds)')
    parser.add_argument('--use-mit-mode', action='store_true', help='Use MIT mode for execution')
    
    # Other
    parser.add_argument('--backend', type=str, default='numpy', choices=['numpy', 'torch'], help='Backend')
    parser.add_argument('--device', type=str, default='cpu', help='Device')
    parser.add_argument('--seed', type=int, default=666, help='Random seed')
    parser.add_argument('--plot', action='store_false', help='Plot trajectory visualization')
    
    main(parser.parse_args())
