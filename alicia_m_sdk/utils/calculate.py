from typing import List
import numpy as np



def calculate_movement_duration(current_joints: List[float], target_joints: List[float], speed_rad_s: float) -> float:
    """
    Calculates the required time for the movement to complete based on the stored speed.
        
    :param current_joints: The starting joint angles in radians.
    :param target_joints: The destination joint angles in radians.
    :param speed_rad_s: The joint speed in radians per second.
    :return: The estimated duration of the movement in seconds.
    """
    if not current_joints or not target_joints or len(current_joints) != len(target_joints):
        print("Warning: Invalid joint angles provided. Returning default delay of 1.0s.")
        return 1.0
    angular_velocity = max(0.001, speed_rad_s)
    angle_distances = np.abs(np.array(target_joints) - np.array(current_joints))
    max_angle_distance = np.max(angle_distances)
    duration = max_angle_distance / angular_velocity + 0.5
    return duration


