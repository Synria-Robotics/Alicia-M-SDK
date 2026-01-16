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
单位转换工具模块
统一管理角度和速度的单位转换
"""

import math
from typing import Union, List

# ==================== 转换常量 ====================
DEG_TO_RAD = math.pi / 180.0  # 度 -> 弧度
RAD_TO_DEG = 180.0 / math.pi  # 弧度 -> 度


# ==================== 角度转换 ====================

def deg_to_rad(angle_deg: Union[float, List[float]]) -> Union[float, List[float]]:
    """
    将角度从度转换为弧度
    
    Args:
        angle_deg: 角度(度),可以是单个值或列表
        
    Returns:
        角度(弧度),类型与输入相同
    """
    if isinstance(angle_deg, (list, tuple)):
        return [a * DEG_TO_RAD for a in angle_deg]
    return angle_deg * DEG_TO_RAD


def rad_to_deg(angle_rad: Union[float, List[float]]) -> Union[float, List[float]]:
    """
    将角度从弧度转换为度
    
    Args:
        angle_rad: 角度(弧度),可以是单个值或列表
        
    Returns:
        角度(度),类型与输入相同
    """
    if isinstance(angle_rad, (list, tuple)):
        return [a * RAD_TO_DEG for a in angle_rad]
    return angle_rad * RAD_TO_DEG


# ==================== 速度转换 ====================

def speed_deg_to_rad(speed_deg_s: Union[float, List[float]]) -> Union[float, List[float]]:
    """
    将速度从度/秒转换为弧度/秒
    
    这是用户API与硬件驱动之间的标准转换接口:
    - 用户API接收: speed_deg_s (度/秒) 
    - 硬件驱动需要: speed_rad_s (弧度/秒)
    
    Args:
        speed_deg_s: 速度(度/秒),可以是单个值或列表
                    推荐范围: 0-360 deg/s
        
    Returns:
        速度(弧度/秒),类型与输入相同
        
    Examples:
        >>> speed_deg_to_rad(180)  # 半圆/秒
        3.14159...
        >>> speed_deg_to_rad([90, 180, 45])
        [1.5707..., 3.1415..., 0.7853...]
    """
    if isinstance(speed_deg_s, (list, tuple)):
        return [s * DEG_TO_RAD for s in speed_deg_s]
    return speed_deg_s * DEG_TO_RAD


def speed_rad_to_deg(speed_rad_s: Union[float, List[float]]) -> Union[float, List[float]]:
    """
    将速度从弧度/秒转换为度/秒
    
    Args:
        speed_rad_s: 速度(弧度/秒),可以是单个值或列表
        
    Returns:
        速度(度/秒),类型与输入相同
        
    Examples:
        >>> speed_rad_to_deg(math.pi)  # 半圆/秒
        180.0
        >>> speed_rad_to_deg([1.0, 2.0, 0.5])
        [57.29..., 114.59..., 28.64...]
    """
    if isinstance(speed_rad_s, (list, tuple)):
        return [s * RAD_TO_DEG for s in speed_rad_s]
    return speed_rad_s * RAD_TO_DEG


# ==================== 验证和限制 ====================

def validate_speed_deg_s(speed_deg_s: float, 
                         min_speed: float = 0.0, 
                         max_speed: float = 360.0,
                         clip: bool = True) -> float:
    """
    验证并可选地限制速度值(度/秒)
    
    Args:
        speed_deg_s: 速度(度/秒)
        min_speed: 最小速度(度/秒),默认0
        max_speed: 最大速度(度/秒),默认360
        clip: 是否将超出范围的值裁剪到有效范围,默认True
        
    Returns:
        验证/裁剪后的速度值
        
    Raises:
        ValueError: 如果速度超出范围且clip=False
    """
    if clip:
        return max(min_speed, min(speed_deg_s, max_speed))
    else:
        if speed_deg_s < min_speed or speed_deg_s > max_speed:
            raise ValueError(
                f"Speed {speed_deg_s} deg/s is out of range "
                f"[{min_speed}, {max_speed}] deg/s"
            )
        return speed_deg_s


def validate_speed_rad_s(speed_rad_s: float,
                        min_speed: float = -10.0,
                        max_speed: float = 10.0,
                        clip: bool = True) -> float:
    """
    验证并可选地限制速度值(弧度/秒)
    
    Args:
        speed_rad_s: 速度(弧度/秒)
        min_speed: 最小速度(弧度/秒),默认-10.0
        max_speed: 最大速度(弧度/秒),默认10.0
        clip: 是否将超出范围的值裁剪到有效范围,默认True
        
    Returns:
        验证/裁剪后的速度值
        
    Raises:
        ValueError: 如果速度超出范围且clip=False
    """
    if clip:
        return max(min_speed, min(speed_rad_s, max_speed))
    else:
        if speed_rad_s < min_speed or speed_rad_s > max_speed:
            raise ValueError(
                f"Speed {speed_rad_s} rad/s is out of range "
                f"[{min_speed}, {max_speed}] rad/s"
            )
        return speed_rad_s


# ==================== 便捷组合函数 ====================

def convert_and_validate_speed(speed_deg_s: Union[float, List[float]],
                               max_deg_s: float = 360.0,
                               max_rad_s: float = 10.0,
                               clip: bool = True) -> Union[float, List[float]]:
    """
    组合函数:将速度从度/秒转换为弧度/秒,并验证范围
    
    这是推荐的用户API到硬件驱动的转换函数
    
    Args:
        speed_deg_s: 速度(度/秒),可以是单个值或列表
        max_deg_s: 最大速度限制(度/秒),默认360
        max_rad_s: 最大速度限制(弧度/秒),默认10.0
        clip: 是否裁剪超出范围的值,默认True
        
    Returns:
        验证后的速度(弧度/秒),类型与输入相同
        
    Examples:
        >>> convert_and_validate_speed(180)
        3.14159...
        >>> convert_and_validate_speed(500, clip=True)  # 裁剪到360
        6.28318...  # 等于360度/秒
    """
    if isinstance(speed_deg_s, (list, tuple)):
        # 处理列表
        validated_rad_list = []
        for s in speed_deg_s:
            # 先验证度值
            s_validated = validate_speed_deg_s(s, 0.0, max_deg_s, clip=clip)
            # 转换为弧度
            s_rad = s_validated * DEG_TO_RAD
            # 再次验证弧度值(双重保险)
            s_rad_validated = validate_speed_rad_s(s_rad, -max_rad_s, max_rad_s, clip=clip)
            validated_rad_list.append(s_rad_validated)
        return validated_rad_list
    else:
        # 处理单个值
        # 先验证度值
        speed_validated = validate_speed_deg_s(speed_deg_s, 0.0, max_deg_s, clip=clip)
        # 转换为弧度
        speed_rad = speed_validated * DEG_TO_RAD
        # 再次验证弧度值(双重保险)
        return validate_speed_rad_s(speed_rad, -max_rad_s, max_rad_s, clip=clip)
