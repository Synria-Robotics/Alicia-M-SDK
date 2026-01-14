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
Control utility functions for joint validation and trajectory planning.
"""

from typing import List, Tuple, Optional, Union, Any
import numpy as np


def check_and_clip_joint_limits(
    joints: List[float],
    joint_limits: Union[List[Tuple[float, float]], Any]
) -> Tuple[List[float], List[Tuple[str, float, float]]]:
    """Check and clip joint angles to stay within limits.

    :param joints: List of joint angles in radians
    :param joint_limits: Joint limits, can be a list of (min, max) tuples or robot_model.joint_limits
    :return: Tuple of (clipped_joints, violations) where violations is a list of (joint_name, original, clipped)
    """
    joints = list(joints)
    violations = []
    
    # Handle different joint_limits formats
    limits_list = None
    
    # Try to extract limits from robot_model-like object
    if hasattr(joint_limits, '_actuated'):
        # robot_model or robot_model.joint_limits that has _actuated attribute
        limits_list = []
        for js in joint_limits._actuated:
            lo, hi = -np.pi, np.pi  # Default limits
            if js.limit:
                if js.limit[0] is not None:
                    lo = js.limit[0]
                if js.limit[1] is not None:
                    hi = js.limit[1]
            limits_list.append((lo, hi))
    elif isinstance(joint_limits, (list, tuple)) and len(joint_limits) > 0:
        # List of (min, max) tuples
        if isinstance(joint_limits[0], (list, tuple)) and len(joint_limits[0]) == 2:
            limits_list = joint_limits
        else:
            # Single tuple or different format
            limits_list = [joint_limits] * len(joints) if len(joints) > 0 else []
    elif hasattr(joint_limits, '__iter__') and not isinstance(joint_limits, str):
        # Try to iterate (might be a custom object with iterable limits)
        try:
            limits_list = list(joint_limits)
            # Check if it's a list of tuples
            if len(limits_list) > 0 and isinstance(limits_list[0], (list, tuple)) and len(limits_list[0]) == 2:
                pass  # Already in correct format
            else:
                limits_list = None
        except (TypeError, ValueError):
            limits_list = None
    
    # If still no valid format, use default limits
    if limits_list is None:
        limits_list = [(-np.pi, np.pi)] * len(joints)
    
    # Ensure we have limits for all joints
    if len(limits_list) < len(joints):
        limits_list.extend([(-np.pi, np.pi)] * (len(joints) - len(limits_list)))
    
    # Clip joints and record violations
    clipped_joints = []
    for i, (joint_val, (lo, hi)) in enumerate(zip(joints, limits_list)):
        original = joint_val
        clipped = np.clip(joint_val, lo, hi)
        clipped_joints.append(float(clipped))
        
        if abs(original - clipped) > 1e-6:
            joint_name = f"joint_{i}"
            violations.append((joint_name, float(original), float(clipped)))
    
    return clipped_joints, violations


def validate_joint_list(
    joints: Union[List[float], np.ndarray],
    num_joints: Optional[int] = None,
    joint_limits: Optional[Union[List[Tuple[float, float]], Any]] = None
) -> Tuple[bool, Optional[str]]:
    """Validate joint list format and values.

    :param joints: List or array of joint angles
    :param num_joints: Expected number of joints (optional)
    :param joint_limits: Joint limits for validation (optional)
    :return: Tuple of (is_valid, error_message)
    """
    if joints is None:
        return False, "Joint list is None"
    
    try:
        joints_list = list(joints)
    except (TypeError, ValueError):
        return False, "Joint list cannot be converted to list"
    
    if len(joints_list) == 0:
        return False, "Joint list is empty"
    
    if num_joints is not None and len(joints_list) != num_joints:
        return False, f"Expected {num_joints} joints, got {len(joints_list)}"
    
    # Check for NaN or Inf values
    for i, val in enumerate(joints_list):
        if not np.isfinite(val):
            return False, f"Joint {i} has invalid value: {val}"
    
    # Check limits if provided
    if joint_limits is not None:
        clipped, violations = check_and_clip_joint_limits(joints_list, joint_limits)
        if violations:
            violation_msgs = [f"{name}: {orig:.3f}" for name, orig, _ in violations]
            return False, f"Joints exceed limits: {', '.join(violation_msgs)}"
    
    return True, None


def compute_steps_and_delay(
    start: Union[float, List[float]],
    end: Union[float, List[float]],
    duration: float,
    max_step_size: Optional[float] = None,
    min_delay: float = 0.001
) -> Tuple[int, float]:
    """Compute number of steps and delay time for trajectory execution.

    :param start: Start value(s)
    :param end: End value(s)
    :param duration: Total duration in seconds
    :param max_step_size: Maximum step size (optional, for position-based calculation)
    :param min_delay: Minimum delay between steps in seconds
    :return: Tuple of (num_steps, delay_time)
    """
    if duration <= 0:
        return 1, min_delay
    
    # If max_step_size is provided, use it to calculate steps
    if max_step_size is not None and max_step_size > 0:
        try:
            start_arr = np.array(start) if not isinstance(start, (int, float)) else np.array([start])
            end_arr = np.array(end) if not isinstance(end, (int, float)) else np.array([end])
            max_distance = np.max(np.abs(end_arr - start_arr))
            num_steps = max(1, int(np.ceil(max_distance / max_step_size)))
        except (TypeError, ValueError):
            num_steps = max(1, int(np.ceil(duration / min_delay)))
    else:
        # Time-based calculation
        num_steps = max(1, int(np.ceil(duration / min_delay)))
    
    # Calculate delay to match duration
    delay = duration / num_steps if num_steps > 1 else duration
    delay = max(min_delay, delay)
    
    # Recalculate steps based on actual delay
    num_steps = max(1, int(np.ceil(duration / delay)))
    
    return num_steps, delay
