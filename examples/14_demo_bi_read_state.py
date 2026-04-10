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

"""
Demo: Read and print dual-arm robot state

Features:
- Read left/right arm joint angles and gripper state
- Read left/right end-effector pose
- Optional extended state (joint velocities and torques)
- Support single-shot or continuous printing
- Auto-detect two serial ports when omitted
"""

import argparse
import glob
import time

import alicia_m_sdk
from alicia_m_sdk.utils.logger import logger


def discover_candidate_ports():
    """Discover likely robot serial ports."""
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    deduped = []
    for port in ports:
        if port not in deduped:
            deduped.append(port)
    return deduped


def resolve_ports(args):
    """Resolve left/right ports from CLI args or local auto-detection."""
    discovered = discover_candidate_ports()

    left_port = args.left_port
    right_port = args.right_port

    if left_port and right_port:
        if left_port == right_port:
            raise ValueError("left and right ports must be different")
        return left_port, right_port

    remaining = [port for port in discovered if port not in {left_port, right_port}]

    if left_port is None and right_port is None:
        if len(remaining) < 2:
            raise ValueError(
                "could not auto-detect two ports; please pass --left-port and --right-port"
            )
        left_port, right_port = remaining[:2]
        return left_port, right_port

    if left_port is None:
        if not remaining:
            raise ValueError("could not auto-detect left port")
        left_port = remaining[0]
    elif right_port is None:
        if not remaining:
            raise ValueError("could not auto-detect right port")
        right_port = remaining[0]

    if left_port == right_port:
        raise ValueError("left and right ports must be different")

    return left_port, right_port


def create_reader_robot(port, args):
    """Create a robot instance configured for read-only state monitoring."""
    robot = alicia_m_sdk.create_robot(
        port=port,
        version=args.version,
        base_link=args.base_link,
        end_link=args.end_link,
        control_aim=args.control_aim,
        control_mode=args.control_mode,
        skip_mit_init=True,
    )

    if not robot.is_connected():
        raise RuntimeError(f"failed to connect to robot on port {port}")

    if not args.disable_extended_state:
        robot.servo_driver.set_extended_state(enabled=True)

    return robot


def format_joint_values(values, output_format):
    """Format joint values for display."""
    if values is None:
        return "N/A"

    if output_format == "deg":
        converted = [round(value * 180.0 / 3.141592653589793, 2) for value in values]
        return f"{converted} deg"

    converted = [round(value, 4) for value in values]
    return f"{converted} rad"


def format_pose(pose):
    """Format pose dictionary for display."""
    if pose is None:
        return "N/A"

    position = [round(value, 5) for value in pose["position"]]
    quaternion = [round(value, 4) for value in pose["quaternion_xyzw"]]
    return f"xyz={position}, quat={quaternion}"


def format_arm_snapshot(name, robot, output_format):
    """Build a printable state snapshot for one arm."""
    joint_state = robot.get_robot_state("joint_gripper")
    pose = robot.get_pose()

    if joint_state is None:
        return f"[{name}] state unavailable"

    lines = [
        f"[{name}]",
        f"  joints  : {format_joint_values(joint_state.angles, output_format)}",
        # f"  gripper : {round(joint_state.gripper, 1)}",
        # f"  status  : {joint_state.run_status_text}",
        # f"  pose    : {format_pose(pose)}",
    ]

    if joint_state.velocities is not None:
        lines.append(f"  vel     : {format_joint_values(joint_state.velocities, output_format)}")

    if joint_state.torques is not None:
        torques = [round(value, 3) for value in joint_state.torques]
        lines.append(f"  torque  : {torques} Nm")

    return "\n".join(lines)


def print_dual_state(left_robot, right_robot, args):
    """Print one synchronized dual-arm snapshot."""
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    print("\n" + "=" * 100)
    print(f"[{timestamp}] Dual-arm state")
    print(format_arm_snapshot("LEFT ", left_robot, args.format))
    print(format_arm_snapshot("RIGHT", right_robot, args.format))


def main(args):
    """Read and print left/right arm state."""
    left_port, right_port = resolve_ports(args)
    logger.info(f"Using left arm port : {left_port}")
    logger.info(f"Using right arm port: {right_port}")

    left_robot = None
    right_robot = None

    try:
        left_robot = create_reader_robot(left_port, args)
        right_robot = create_reader_robot(right_port, args)

        if not args.disable_extended_state:
            time.sleep(0.1)

        if args.single:
            print_dual_state(left_robot, right_robot, args)
            return

        if args.fps <= 0:
            raise ValueError("--fps must be > 0 for continuous mode")

        interval = 1.0 / args.fps
        logger.info(f"Start continuous dual-arm state printing, press Ctrl+C to stop (target FPS: {args.fps})")

        while True:
            start_time = time.perf_counter()
            print_dual_state(left_robot, right_robot, args)
            elapsed = time.perf_counter() - start_time
            sleep_time = max(0.0, interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        logger.info("Stopped dual-arm state printing")
    except Exception as exc:
        logger.error(f"Failed to read dual-arm state: {exc}")
        raise
    finally:
        if left_robot is not None:
            left_robot.disconnect()
        if right_robot is not None:
            right_robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Read dual-arm robot state")

    parser.add_argument("--left-port", type=str, default=None, help="left arm serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--right-port", type=str, default=None, help="right arm serial port, e.g. /dev/ttyACM1")
    parser.add_argument("--version", type=str, default="v1_1", help="robot version, e.g. v1_0 or v1_1")
    parser.add_argument("--base_link", type=str, default="base_link", help="robot base link name")
    parser.add_argument("--end_link", type=str, default="link6", help="robot end-effector link name")
    parser.add_argument(
        "--control-aim",
        type=str,
        default="operation",
        choices=["teach", "operation"],
        help="control aim for both arms",
    )
    parser.add_argument(
        "--control-mode",
        type=str,
        default="pv",
        choices=["pv", "mit"],
        help="control mode for both arms",
    )
    parser.add_argument(
        "--format",
        type=str,
        default="deg",
        choices=["deg", "rad"],
        help="joint display format",
    )
    parser.add_argument(
        "--single",
        action="store_true",
        help="print one dual-arm snapshot and exit",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=5.0,
        help="continuous printing frequency",
    )
    parser.add_argument(
        "--disable-extended-state",
        action="store_true",
        help="disable velocity and torque reading",
    )

    main(parser.parse_args())
