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
Demo: Bridge MuJoCo interactive dual-arm IK to Alicia-M SDK

This demo links the MuJoCo interactive bimanual controller with two real arms:
- MuJoCo solves left/right IK in real time
- The demo subscribes to MuJoCo's current joint solutions every frame
- Joint targets are streamed to the left/right SDK robot instances

Typical flow:
1. Read current left/right arm joints from hardware
2. Use those joints to initialize the MuJoCo interactive scene
3. Drag MuJoCo targets
4. Stream the resulting joint values to the real bimanual arms
"""

import argparse
import glob
import sys
import time
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

SDK_ROOT = Path(__file__).resolve().parents[1]
ROBOCORE_ROOT = SDK_ROOT.parents[1] / "Bessica" / "RoboCore"
if str(ROBOCORE_ROOT) not in sys.path:
    sys.path.insert(0, str(ROBOCORE_ROOT))
if str(SDK_ROOT) not in sys.path:
    sys.path.insert(0, str(SDK_ROOT))

import alicia_m_sdk
from alicia_m_sdk.utils.logger import logger
from robocore.bridge.sim.mujoco.interactive_dual_arm import InteractiveDualArmIK
from synriard import get_model_path  # type: ignore[import-not-found]

INITIAL_JOINT_STATE = {
    "joint1_l": 1.57,
    "joint2_l": -2.35,
    "joint3_l": -1.57,
    "joint4_l": 0.0,
    "joint5_l": 0.0,
    "joint6_l": 0.0,
    "joint7_l": 0.0,
    "joint8_l": 0.0,
    "joint1_r": -1.57,
    "joint2_r": -2.35,
    "joint3_r": -1.57,
    "joint4_r": 0.0,
    "joint5_r": 0.0,
    "joint6_r": 0.0,
    "joint7_r": 0.0,
    "joint8_r": 0.0,
}


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
    """Create one robot instance for streaming joint targets."""
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


class DualArmSDKStreamer:
    """Stream MuJoCo joint targets to the real left/right arms."""

    def __init__(self, left_robot, right_robot, args, left_limits, right_limits):
        self.left_robot = left_robot
        self.right_robot = right_robot
        self.args = args
        self.left_limits = left_limits
        self.right_limits = right_limits
        self.last_send_time = 0.0
        self.last_left = None
        self.last_right = None
        self.executor = ThreadPoolExecutor(max_workers=2)

    def _send_one(self, robot, target_joints):
        return robot.set_robot_state(
            target_joints=target_joints,
            gripper_value=None,
            joint_format="rad",
            speed=self.args.speed,
            gripper_speed=self.args.gripper_speed,
            wait_for_completion=False,
            timeout=self.args.timeout,
        )

    def maybe_send(self, left_target, right_target):
        """Rate-limit and stream new joint commands to both arms."""
        now = time.time()
        min_interval = 1.0 / self.args.send_hz if self.args.send_hz > 0 else 0.0

        left_target = np.asarray(left_target[:6], dtype=float)
        right_target = np.asarray(right_target[:6], dtype=float)
        left_clipped = np.clip(left_target, self.left_limits[:, 0], self.left_limits[:, 1])
        right_clipped = np.clip(right_target, self.right_limits[:, 0], self.right_limits[:, 1])

        if np.any(np.abs(left_clipped - left_target) > 1e-9):
            logger.warning("Left target exceeded XML joint limits, clipped before SDK send")
        if np.any(np.abs(right_clipped - right_target) > 1e-9):
            logger.warning("Right target exceeded XML joint limits, clipped before SDK send")

        changed = (
            self.last_left is None
            or self.last_right is None
            or np.max(np.abs(left_clipped - self.last_left)) >= self.args.joint_epsilon
            or np.max(np.abs(right_clipped - self.last_right)) >= self.args.joint_epsilon
        )

        if not changed:
            return

        if min_interval > 0 and (now - self.last_send_time) < min_interval:
            return

        left_future = self.executor.submit(self._send_one, self.left_robot, left_clipped.tolist())
        right_future = self.executor.submit(self._send_one, self.right_robot, right_clipped.tolist())
        left_ok = left_future.result()
        right_ok = right_future.result()

        if not left_ok or not right_ok:
            raise RuntimeError(f"failed to stream joints (left={left_ok}, right={right_ok})")

        self.last_send_time = now
        self.last_left = left_clipped.copy()
        self.last_right = right_clipped.copy()

    def close(self):
        """Release worker threads used for command streaming."""
        self.executor.shutdown(wait=True)


class InteractiveDualArmSDKBridge(InteractiveDualArmIK):
    """MuJoCo interactive dual-arm controller with SDK streaming."""

    def __init__(self, *args, streamer=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.streamer = streamer

    def step(self):
        """Run one MuJoCo step, then publish solved joints to hardware."""
        super().step()
        if self.streamer is not None:
            self.streamer.maybe_send(self.q_left, self.q_right)


def load_arm_joint_limits_from_mjcf(mjcf_path):
    """Load joint1..joint6 limits for left/right arms from MJCF."""
    tree = ET.parse(mjcf_path)
    root = tree.getroot()

    limits = {}
    for joint in root.findall(".//joint"):
        name = joint.get("name")
        range_text = joint.get("range")
        if not name or not range_text:
            continue

        if name.startswith("joint") and (name.endswith("_l") or name.endswith("_r")):
            parts = range_text.strip().split()
            if len(parts) != 2:
                continue
            lo = float(parts[0])
            hi = float(parts[1])
            limits[name] = (min(lo, hi), max(lo, hi))

    left = []
    right = []
    for idx in range(1, 7):
        lname = f"joint{idx}_l"
        rname = f"joint{idx}_r"
        if lname not in limits or rname not in limits:
            raise RuntimeError(f"missing joint limits in MJCF for {lname} or {rname}")
        left.append(limits[lname])
        right.append(limits[rname])

    return np.asarray(left, dtype=float), np.asarray(right, dtype=float)


def main(args):
    """Run interactive MuJoCo dual-arm control and stream it to real hardware."""
    left_port, right_port = resolve_ports(args)
    logger.info(f"Using left arm port : {left_port}")
    logger.info(f"Using right arm port: {right_port}")

    left_robot = None
    right_robot = None
    streamer = None

    try:
        left_robot = create_control_robot(left_port, args)
        right_robot = create_control_robot(right_port, args)

        initial_joint_state = dict(INITIAL_JOINT_STATE)
        logger.info("Using fixed MuJoCo initial joint state from demo_mujoco_bi_relativem")

        mjcf_path = get_model_path(
            "Alicia_M",
            version=args.version,
            variant="bi_interactive",
            model_format="mjcf",
        )
        left_limits, right_limits = load_arm_joint_limits_from_mjcf(mjcf_path)
        logger.info("Loaded joint1..joint6 limits from MJCF and enabled clipping before SDK send")

        streamer = DualArmSDKStreamer(left_robot, right_robot, args, left_limits, right_limits)
        controller = InteractiveDualArmSDKBridge(
            mjcf_path,
            args.left_end,
            args.right_end,
            initial_joint_state=initial_joint_state,
            streamer=streamer,
        )

        # Push the initial state once so SDK and MuJoCo start from the same command source.
        streamer.maybe_send(controller.q_left, controller.q_right)
        controller.run(mode=args.mode)

    except KeyboardInterrupt:
        logger.info("Interactive bridge interrupted by user")
    finally:
        if streamer is not None:
            streamer.close()
        if left_robot is not None:
            left_robot.disconnect()
        if right_robot is not None:
            right_robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MuJoCo ↔ Alicia-M SDK dual-arm interactive bridge")

    parser.add_argument("--left-port", type=str, default=None, help="left arm serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--right-port", type=str, default=None, help="right arm serial port, e.g. /dev/ttyACM1")
    parser.add_argument("--version", type=str, default="v1_1", help="robot version, e.g. v1_1")
    parser.add_argument("--base_link", type=str, default="base_link", help="robot base link name")
    parser.add_argument("--end_link", type=str, default="link6", help="SDK end-effector link name")
    parser.add_argument("--left-end", type=str, default="tool0_site_l", help="MuJoCo left end-effector reference")
    parser.add_argument("--right-end", type=str, default="tool0_site_r", help="MuJoCo right end-effector reference")
    parser.add_argument(
        "--mode",
        type=str,
        default="relative",
        choices=["independent", "relative", "mirror"],
        help="MuJoCo interactive control mode",
    )
    parser.add_argument(
        "--control-aim",
        type=str,
        default="operation",
        choices=["teach", "operation"],
        help="control aim for both real arms",
    )
    parser.add_argument(
        "--control-mode",
        type=str,
        default="pv",
        choices=["pv", "mit"],
        help="control mode for both real arms",
    )
    parser.add_argument("--speed", type=int, default=15, help="joint speed sent to SDK")
    parser.add_argument("--gripper-speed", type=int, default=40, help="gripper speed sent to SDK")
    parser.add_argument("--timeout", type=float, default=2.0, help="SDK command timeout")
    parser.add_argument("--send-hz", type=float, default=20.0, help="joint streaming rate to hardware")
    parser.add_argument(
        "--joint-epsilon",
        type=float,
        default=0.01,
        help="minimum joint change in rad before sending a new command",
    )

    main(parser.parse_args())
