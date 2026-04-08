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
Demo: Synchronous dual-arm joint control

Features:
- Control left and right arms at the same time
- Auto-detect two serial ports when omitted
- Reuse the standard set_robot_state() joint + gripper command
- Optional right-arm mirroring for symmetric bimanual motion
"""

import argparse
import glob
import threading
import time
from concurrent.futures import ThreadPoolExecutor

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
        return remaining[0], remaining[1]

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


def create_control_robot(port, args):
    """Create one robot instance for dual-arm motion."""
    robot = alicia_m_sdk.create_robot(
        port=port,
        version=args.version,
        base_link=args.base_link,
        end_link=args.end_link,
        control_aim=args.control_aim,
        control_mode=args.control_mode,
    )

    if not robot.is_connected():
        raise RuntimeError(f"failed to connect to robot on port {port}")

    return robot


def build_targets(base_target_deg, mirror_right):
    """Build left/right target joints from one reference pose."""
    left_target = list(base_target_deg)
    right_target = list(base_target_deg)

    if mirror_right:
        right_target[0] = -right_target[0]

    return left_target, right_target


def run_synchronized_step(
    left_robot,
    right_robot,
    left_target,
    right_target,
    left_gripper,
    right_gripper,
    args,
    title,
):
    """Start both arms together and wait until both complete."""
    logger.info(title)
    start_event = threading.Event()

    def _move_one(robot, target_joints, gripper_value):
        start_event.wait()
        return robot.set_robot_state(
            target_joints=target_joints,
            gripper_value=gripper_value,
            joint_format="deg",
            speed=args.speed,
            gripper_speed=args.gripper_speed,
            wait_for_completion=True,
            timeout=args.timeout,
        )

    with ThreadPoolExecutor(max_workers=2) as executor:
        left_future = executor.submit(_move_one, left_robot, left_target, left_gripper)
        right_future = executor.submit(_move_one, right_robot, right_target, right_gripper)
        start_event.set()

        left_ok = left_future.result()
        right_ok = right_future.result()

    if not left_ok or not right_ok:
        raise RuntimeError(
            f"synchronized step failed (left={left_ok}, right={right_ok})"
        )


def main(args):
    """Control two robot arms with synchronized joint-space motions."""
    left_port, right_port = resolve_ports(args)
    logger.info(f"Using left arm port : {left_port}")
    logger.info(f"Using right arm port: {right_port}")

    left_robot = None
    right_robot = None

    try:
        left_robot = create_control_robot(left_port, args)
        right_robot = create_control_robot(right_port, args)

        home_left, home_right = build_targets([0, 0, 0, 0, 0, 0], mirror_right=False)
        ready_left, ready_right = build_targets(
            [15.0, -15.0, -15.0, 15.0, 0.0, 0.0],
            mirror_right=args.mirror_right,
        )
        reach_left, reach_right = build_targets(
            [-15.0, -15.0, -15.0, 15.0, 0.0, 0.0],
            mirror_right=args.mirror_right,
        )

        print("\n===== Step 1: 双臂回到 Home 位置 =====")
        run_synchronized_step(
            left_robot,
            right_robot,
            left_target=home_left,
            right_target=home_right,
            left_gripper=None,
            right_gripper=None,
            args=args,
            title="Step 1: move both arms to home",
        )
        time.sleep(args.pause)

        print("\n===== Step 2: 双臂同步运动到准备姿态，并张开夹爪 =====")
        run_synchronized_step(
            left_robot,
            right_robot,
            left_target=ready_left,
            right_target=ready_right,
            left_gripper=args.open_gripper,
            right_gripper=args.open_gripper,
            args=args,
            title="Step 2: move both arms to ready pose and open grippers",
        )
        time.sleep(args.pause)

        print("\n===== Step 3: 双臂同步前伸，并闭合夹爪 =====")
        run_synchronized_step(
            left_robot,
            right_robot,
            left_target=reach_left,
            right_target=reach_right,
            left_gripper=args.close_gripper,
            right_gripper=args.close_gripper,
            args=args,
            title="Step 3: move both arms forward and close grippers",
        )
        time.sleep(args.pause)

        print("\n===== Step 4: 双臂同步回到 Home，并保持夹爪闭合 =====")
        run_synchronized_step(
            left_robot,
            right_robot,
            left_target=home_left,
            right_target=home_right,
            left_gripper=args.close_gripper,
            right_gripper=args.close_gripper,
            args=args,
            title="Step 4: return both arms home",
        )

    except KeyboardInterrupt:
        logger.info("\n✗ Processing interrupted")
    finally:
        if left_robot is not None:
            left_robot.disconnect()
        if right_robot is not None:
            right_robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="双臂同步关节运动控制示例")

    parser.add_argument("--left-port", type=str, default=None, help="左臂串口端口，例如 /dev/ttyACM0")
    parser.add_argument("--right-port", type=str, default=None, help="右臂串口端口，例如 /dev/ttyACM1")
    parser.add_argument("--version", type=str, default="v1_1", help="机械臂版本 (默认: v1_1)")
    parser.add_argument("--base_link", type=str, default="base_link", help="基座链路名称")
    parser.add_argument("--end_link", type=str, default="link6", help="末端执行器链路名称")
    parser.add_argument(
        "--control-aim",
        type=str,
        default="operation",
        choices=["teach", "operation"],
        help="双臂控制目标 (默认: operation)",
    )
    parser.add_argument(
        "--control-mode",
        type=str,
        default="pv",
        choices=["pv", "mit"],
        help="双臂控制模式 (默认: pv)",
    )
    parser.add_argument("--speed", type=int, default=15, help="关节运动速度 (范围: 0-400)")
    parser.add_argument("--gripper-speed", type=int, default=40, help="夹爪速度 (范围: 0-400)")
    parser.add_argument("--timeout", type=float, default=100.0, help="单步等待超时时间（秒）")
    parser.add_argument("--pause", type=float, default=2.0, help="每个动作之间的暂停时间（秒）")
    parser.add_argument(
        "--mirror-right",
        action="store_true",
        help="将右臂第1关节做镜像，适用于左右对称摆放的双臂",
    )
    parser.add_argument("--open-gripper", type=int, default=1000, help="张开夹爪目标值 (默认: 1000)")
    parser.add_argument("--close-gripper", type=int, default=0, help="闭合夹爪目标值 (默认: 0)")

    main(parser.parse_args())
