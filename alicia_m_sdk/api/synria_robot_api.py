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
SynriaRobotAPI - User-level API

Responsibilities:
- Provide concise unified user interface
- High-level motion command encapsulation
- State query interface
- System control functions
- Parameter validation and error handling
"""

import time
from typing import List, Optional, Dict, Union, Tuple, Any
import numpy as np
import json
import os
# Import from robocore for kinematics and planning
from robocore.kinematics import inverse_kinematics
from robocore.modeling import RobotModel
from robocore.transform import make_transform
from robocore.transform.conversions import quaternion_to_matrix
from robocore.kinematics import forward_kinematics
from robocore.transform import matrix_to_euler, matrix_to_quaternion
from ..hardware import ServoDriver
from ..hardware.data_parser import JointState
from ..execution import HardwareExecutor, JointInterpolator
from ..utils.logger import logger
from ..utils.control_utils import compute_steps_and_delay, validate_joint_list, check_and_clip_joint_limits
from ..utils.calculate import calculate_movement_duration


class SynriaRobotAPI:
    """Synria robot arm API - provides unified user interface"""

    def __init__(self,
                 servo_driver: ServoDriver,
                 robot_model: RobotModel,
                 auto_connect: bool = True):
        """Initialize robot API.

        :param servo_driver: Servo driver instance (low-level hardware)
        :param robot_model: Pre-loaded robot model (RoboCore RobotModel)
        :param auto_connect: Automatically connect on initialization
        """
        self.servo_driver = servo_driver
        self.data_parser = servo_driver.data_parser  # Direct access to data parser
        self.robot_model = robot_model
        self.debug_mode = servo_driver.debug_mode  # Access debug mode from servo driver

        # Higher-level helpers
        self.robot_type = None

        # Access control_aim from servo_driver (motor-specific)
        self.control_aim = getattr(servo_driver, 'default_control_aim', ServoDriver.AIM_OPERATION)
        # Set default control_mode to PATTERN_PV (position+velocity mode)
        # Note: ServoDriver defaults to PATTERN_PV if control_mode is None
        self.control_mode = ServoDriver.PATTERN_PV

        # Create execution layer components
        self.hardware_executor = HardwareExecutor(servo_driver)
        # Default parameters
        self.home_angles = [0.0] * 6

        if auto_connect:
            self.connect()


    
    # ==================== Connection Management ====================
    
    def connect(self) -> bool:
        """Connect to robot and detect firmware version."""
        if self.is_connected():
            # Ensure update thread is running if already connected
            if not self.servo_driver.is_update_thread_running():
                self.servo_driver.start_update_thread()
            return True

        result = self.servo_driver.connect()
        if result:
            try:
                # Start background update thread for continuous state updates
                self.servo_driver.start_update_thread()
                
                # Give background thread a moment to start (avoid race condition)
                time.sleep(0.1)
                
                # Auto-detect arm type (teach vs operation) and update control_aim
                detected_aim = self.servo_driver.auto_detect_control_aim(timeout=1.0)
                if detected_aim != self.servo_driver.default_control_aim:
                    # Update control_aim if detection found a different valid type
                    old_aim_name = "Teaching Arm" if self.servo_driver.default_control_aim == ServoDriver.AIM_TEACH else "Operating Arm"
                    new_aim_name = "Teaching Arm" if detected_aim == ServoDriver.AIM_TEACH else "Operating Arm"
                    logger.info(f"[Auto-Detect] Switched control_aim: {old_aim_name} (0x{self.servo_driver.default_control_aim:02X}) → {new_aim_name} (0x{detected_aim:02X})")
                    self.servo_driver.default_control_aim = detected_aim
                    self.control_aim = detected_aim
                
                # Initialize state
                self.get_robot_state("joint_gripper")
                self._robot_type()
                logger.info("Synria Robot Connected successfully.")
                return True
            except Exception as e:
                logger.error(f"Hardware initialization failed after serial connection: {e}")
                return False
        return False
    
    def disconnect(self):
        """Disconnect from robot and stop update threads."""
        self.servo_driver.stop_update_thread()
        self.servo_driver.disconnect()
    
    def is_connected(self) -> bool:
        """Check if robot is connected.
        """
        return self.servo_driver.serial_comm.is_connected()

    # ==================== Get Robot Information ====================

    def get_robot_state(self, info_type: str = "joint_gripper", timeout: float = 1.0) -> Optional[Union[JointState, Dict, List[float], str, float]]:
        """
        Unified API to get robot state information.
        
        :param info_type: Type of information to get. Options:
            - "joint_gripper": Returns JointState (arm joint angles, gripper value, timestamp, run_status_text)
            - "joint": Returns List[float] of arm joint angles (radians) only
            - "gripper": Returns float gripper value (0-100) only
            - "version": Returns Dict with serial_number, hardware_version, firmware_version
            - "temperature": Returns List[float] of temperatures in Celsius
            - "velocity": Returns List[float] of velocities in degrees per second
            - "gripper_type": Returns str (e.g., "50mm" or "100mm") or None if unavailable
            - "self_check": Returns Dict with self-check data (or None if failed)
        :param timeout: Maximum time to wait for response in seconds
        :return: Requested data or None if failed
        """
        # Special handling for gripper_type: try cache first, then hardware query
        if info_type == "gripper_type":
            return self._get_gripper_type_with_cache(timeout)

        # Joint and gripper data are continuously updated by background thread
        # Just read from cache (no need to send query command)
        if info_type in ("joint_gripper", "joint", "gripper"):
            # Give background thread a chance to update if data is stale
            result = self.data_parser.get_info(info_type)
            if result is None:
                # Cache not ready yet, wait briefly for background thread
                time.sleep(0.05)
                result = self.data_parser.get_info(info_type)
            return result

        # Other info types map directly to hardware commands
        if not self.servo_driver.acquire_info(info_type, wait=True, timeout=timeout):
            logger.error(f"Failed to get {info_type} data within timeout period")
            return None

        result = self.data_parser.get_info(info_type)

        return result

    def _get_gripper_type_with_cache(self, timeout: float = 1.0) -> Optional[str]:
        """Get gripper type with caching support.
        
        :param timeout: Maximum time to wait for response in seconds
        :return: Gripper type string or None if unavailable
        """
        # M-SDK (servo motor) doesn't support querying gripper_type from hardware
        # Return the gripper_type that was passed during initialization
        # This is stored in servo_driver or can be accessed from robot configuration
        if hasattr(self.servo_driver, 'gripper_type'):
            return self.servo_driver.gripper_type
        # If not available, return None (hardware doesn't support querying)
        return None

    def _robot_type(self) -> str:
        """Detect robot type from serial number."""
        if self.robot_type is not None:
            return self.robot_type

        version = self.get_robot_state("version")
        if version is None:
            return None

        serial_number = version.get("serial_number", "")
        # For M-SDK, detect based on serial number pattern
        if serial_number.startswith("AMF"):
            self.robot_type = "follower"
        elif serial_number.startswith("AML"):
            self.robot_type = "leader"

        return self.robot_type

    def get_pose(self) -> Optional[Union[List[float], Dict]]:
        """Get current end-effector pose.

        :return: Dictionary with position, rotation, euler_xyz, quaternion_xyzw, transform, or None if failed
        """
        joint_angles = self.get_robot_state("joint")
        if joint_angles is None:
            logger.error("无法获取关节角度")
            return None

        try:
            T_fk = forward_kinematics(
                self.robot_model,
                joint_angles,
                return_end=True
            )
        except Exception as e:
            logger.error(f"Forward kinematics failed: {e}")
            return None

        position_fk = T_fk[:3, 3]
        rotation_fk = T_fk[:3, :3]
        euler_fk = matrix_to_euler(rotation_fk, seq='xyz')
        quat_fk = matrix_to_quaternion(rotation_fk)

        return {
            'transform': T_fk,
            'position': position_fk,
            'rotation': rotation_fk,
            'euler_xyz': euler_fk,
            'quaternion_xyzw': quat_fk
        }

    # ==================== Robot Control ====================

    def go_home(self, speed_deg_s: Union[int, float, List[float], np.ndarray] = 20, gripper_speed_deg_s: Optional[float] = 57.3):
        """Move robot to home position (all joints at 0 degrees) and wait until completion.

        :param speed_deg_s: Speed in degrees per second. Can be int/float (same for all joints) or list/array (per-joint speeds, range [-573, +573] deg/s), default 20
        :param gripper_speed_deg_s: Gripper speed in degrees per second (range [-573, +573] deg/s), default 57.3
        """
        home_joints = [0.0] * 6
        self.set_robot_state(
            target_joints=home_joints, 
            gripper_value=None, 
            speed_deg_s=speed_deg_s,
            gripper_speed_deg_s=gripper_speed_deg_s,
            wait_for_completion=True
        )
    
    def set_home(self, speed_deg_s: Union[int, float, List[float], np.ndarray] = 20, gripper_speed_deg_s: Optional[float] = 57.3):
        """DEPRECATED: Use go_home() instead. This method will be removed in future versions.
        
        Move robot to home position (all joints at 0 degrees) and wait until completion.

        :param speed_deg_s: Speed in degrees per second. Can be int/float (same for all joints) or list/array (per-joint speeds, range [-573, +573] deg/s), default 20
        :param gripper_speed_deg_s: Gripper speed in degrees per second (range [-573, +573] deg/s), default 57.3
        """
        import warnings
        warnings.warn(
            "set_home() is deprecated and will be removed in future versions. Use go_home() instead.",
            DeprecationWarning,
            stacklevel=2
        )
        self.go_home(speed_deg_s=speed_deg_s, gripper_speed_deg_s=gripper_speed_deg_s)

    def set_robot_state(self,
                        target_joints: Optional[List[float]] = None,
                        gripper_value: Optional[int] = None,
                        joint_format: str = 'deg',
                        speed_deg_s: Union[int, float, List[float], np.ndarray] = 20,
                        gripper_speed_deg_s: Optional[float] = 57.3,
                        tolerance: float = 0.1,
                        timeout: float = 10.0,
                        wait_for_completion: bool = True,
                        control_aim: int = None,
                        control_mode: tuple = None) -> bool:
        """Set joint angles and/or gripper in a single combined command.

        :param target_joints: Optional target joint angles. If None, keeps current
        :param gripper_value: Optional gripper value (0-100). If None, keeps current
        :param joint_format: Unit format for joints, 'rad' or 'deg'
        :param speed_deg_s: Speed in degrees per second. Can be int/float (same for all joints) or list/array (per-joint speeds, range [-573, +573] deg/s), default 20
        :param gripper_speed_deg_s: Gripper speed in degrees per second (range [-573, +573] deg/s), default 57.3
        :param tolerance: Rad, acceptable abs distance to target for joints
        :param timeout: Seconds, maximum wait time
        :param wait_for_completion: If True, wait until target reached
        :param control_aim: Control target (ServoDriver.AIM_TEACH, AIM_OPERATION, etc.). If None, uses instance default
        :param control_mode: Control mode (ServoDriver.PATTERN_PV, PATTERN_MIT_POSITION, etc.). If None, uses instance default
        :return: True if successful, False otherwise
        
        Note: Due to hardware protocol constraints, joint and gripper data must be sent together.
        When target_joints is None, the current joint positions from the background query will be used.
        If no valid joint data is available, zeros (HOME position) will be used as fallback.
        """
        # Convert joint format if needed
        if target_joints is not None:
            if joint_format == 'deg':
                target_joints = [a * np.pi / 180.0 for a in target_joints]
        else:
            # 当 target_joints 为 None 时，尝试使用当前关节位置
            # 这样可以避免意外将机械臂移动到其他位置
            current_state = self.servo_driver.data_parser.get_joint_state()
            if current_state and current_state.angles:
                angles = current_state.angles
                # 验证数据有效性
                is_valid = True
                # 检查1：全零是无效的（未初始化的默认值）
                if all(abs(a) < 0.001 for a in angles):
                    is_valid = False
                # 检查2：接近 -12.5 rad 是无效的（原始值0转换后的结果）
                if all(abs(a - (-12.5)) < 0.1 for a in angles):
                    is_valid = False
                
                if is_valid:
                    target_joints = angles
                    logger.debug(f"Using current joint positions: {[f'{a*57.2958:.1f}°' for a in angles]}")
                else:
                    logger.warning("No valid joint data available, gripper-only command will use current position from hardware")
                    # 让底层使用 [0.0]*6 作为默认值，这会发送 HOME 位置
                    # 用户需要知道这个行为！

        # Use provided control_aim/control_mode or instance defaults
        effective_control_aim = control_aim if control_aim is not None else self.control_aim
        effective_control_mode = control_mode if control_mode is not None else self.control_mode
        
        # 速度参数直接传递,无需转换 (现在全SDK统一使用deg/s)

        # Use unified method with control mode parameters
        success = self.servo_driver.set_joint_and_gripper(
            joint_angles=target_joints,
            gripper_value=gripper_value,
            speed_deg_s=speed_deg_s,
            gripper_speed_deg_s=gripper_speed_deg_s,
            control_aim=effective_control_aim,
            control_mode=effective_control_mode
        )

        if not success:
            logger.error("Failed to set robot target")
            return False

        # Wait for completion if requested
        if wait_for_completion and (target_joints is not None or gripper_value is not None):
            # Store control_mode temporarily for _wait_for_joint_target
            original_control_mode = self.control_mode
            if effective_control_mode is not None:
                self.control_mode = effective_control_mode
            
            joint_result = True
            gripper_result = True
            
            try:
                # Wait for joints if target_joints is provided
                if target_joints is not None:
                    joint_result = self._wait_for_joint_target(
                        target_joints=target_joints,
                        tolerance_deg=tolerance * 180.0 / np.pi,  # Convert rad to deg
                        timeout=timeout,
                        log_prefix="等待关节接近目标"
                    )
                
                # Wait for gripper if gripper_value is provided
                if gripper_value is not None:
                    gripper_tolerance = 5.0  # Increase tolerance to 5% for more reliable completion
                    gripper_timeout = min(4.0, timeout)  # Use max 4 seconds for gripper wait
                    start_time = time.time()
                    
                    # Give hardware some time to start responding
                    time.sleep(0.05)
                    
                    # Check gripper position with timeout
                    gripper_reached = False
                    while time.time() - start_time < gripper_timeout:
                        current_gripper = self.get_robot_state("gripper")
                        if current_gripper is not None:
                            if abs(current_gripper - gripper_value) <= gripper_tolerance:
                                gripper_reached = True
                                break
                        time.sleep(0.05)
                    
                    # If we didn't verify position but command was sent, still consider success
                    if not gripper_reached:
                        final_check = self.get_robot_state("gripper")
                        if final_check is None:
                            # State unavailable, but command was sent
                            gripper_result = True
                        else:
                            # Check one more time with tolerance
                            gripper_result = abs(final_check - gripper_value) <= gripper_tolerance
                    else:
                        gripper_result = True
            finally:
                # Restore original control_mode
                self.control_mode = original_control_mode
            
            return joint_result and gripper_result

        # Either waiting was not requested, or no targets were provided.
        return True

    # ==================== Gripper Control ====================
    
    def set_gripper_target(self,
                       command: Optional[str] = None,
                       value: Optional[float] = None,
                       wait_for_completion: bool = True,
                       timeout: float = 4.0,
                       tolerance: float = 1.0) -> bool:
        """Control gripper position.

        :param command: Command string, 'open' or 'close'
        :param value: Gripper value, 0 (closed) to 100 (open)
        :param wait_for_completion: Wait until gripper reaches target
        :param timeout: Maximum wait time in seconds
        :param tolerance: Acceptable difference to target value
        :return: True if successful
        """
        if command is not None and value is not None:
            logger.error("command 与 value 参数不可同时指定")
            return False
        
        if command is not None:
            if command == "open":
                value = 100.0  # 打开对应100
            elif command == "close":
                value = 0.0    # 关闭对应0
            else:
                logger.error("command 参数必须是 'open' 或 'close'")
                return False
        
        if value is None:
            logger.error("必须提供 command 或 value 参数")
            return False
        
        # 发送夹爪命令
        success = self.servo_driver.set_gripper(value, control_aim=self.control_aim, control_mode=self.control_mode)
        if not success:
            logger.error("夹爪命令发送失败")
            return False
        
        if wait_for_completion:
            start_time = time.time()
            while time.time() - start_time < timeout:
                current_gripper = self.get_robot_state("gripper")
                if current_gripper is not None:
                    if abs(current_gripper - value) <= tolerance:
                        # logger.info(f"夹爪已到达目标开合度: {value:.1f}")
                        return True
                self.servo_driver.set_gripper(value, control_aim=self.control_aim, control_mode=self.control_mode)
                time.sleep(0.1)
            
            # logger.warning("夹爪运动等待超时")
            return False
        
        return True


    def set_pose(self,
                        target_pose: List[float],
                        backend: str = 'numpy',
                        method: str = 'dls',
                        display: bool = True,
                        tolerance: float = 1e-4,
                        max_iters: int = 100,
                        multi_start: int = 0,
                        use_random_init: bool = False,
                        speed_deg_s: Union[int, float, List[float], np.ndarray] = 10,
                        gripper_speed_deg_s: Optional[float] = 57.3,
                        execute: bool = True) -> Dict:
        """Move end-effector to target pose using inverse kinematics.

        :param target_pose: Target pose as [x, y, z, qx, qy, qz, qw]
        :param backend: Computation backend, 'numpy' or 'torch'
        :param method: IK solver method, 'dls', 'pinv', or 'transpose'
        :param display: Display solution details
        :param tolerance: Position and orientation tolerance
        :param max_iters: Maximum number of iterations
        :param multi_start: Number of multi-start attempts, 0 to disable
        :param use_random_init: Use random initial guess instead of current pose
        :param speed_deg_s: Motion speed in degrees per second. Can be int/float (same for all joints) or list/array (per-joint speeds, range [-573, +573] deg/s)
        :param gripper_speed_deg_s: Gripper speed in degrees per second (range [-573, +573] deg/s), default 57.3
        :param execute: Execute motion if True
        :return: Dictionary with success, q, iters, pos_err, ori_err, message
        """
        # Convert pose to transformation matrix
        position = np.array(target_pose[:3])
        quaternion = np.array(target_pose[3:])
        rotation_matrix = quaternion_to_matrix(quaternion)
        pose_matrix = make_transform(rotation_matrix, position)

        # Get initial guess
        if use_random_init:
            # Generate random initial guess within joint limits
            q_init = self._generate_random_q(scale=0.5)
            if display:
                logger.info("使用随机初始值")
        else:
            q_init = self.get_robot_state("joint")
            if q_init is None:
                return {
                    'success': False,
                    'message': '无法获取当前关节角度',
                    'q': None
                }

        if display:
            logger.info(f"初始关节角度 (rad): {[f'{q:+.4f}' for q in q_init]}")
            logger.info(f"初始关节角度 (deg): {[f'{np.rad2deg(q):+.2f}' for q in q_init]}")
            logger.info(f"正在求解IK (方法: {method}, 最大迭代: {max_iters})...")

        # Solve inverse kinematics
        ik_result = inverse_kinematics(
            self.robot_model,
            pose_matrix,
            q_init,
            backend=backend,
            method=method,
            max_iters=max_iters,
            pos_tol=tolerance,
            ori_tol=tolerance,
            multi_start=multi_start,
            multi_noise=0.3,
            use_analytic_jacobian=True
        )

        if ik_result['success']:
            if display:
                logger.info("✓ IK 求解成功!")
                logger.info(f"  迭代次数: {ik_result['iters']}")
                logger.info(f"  位置误差: {ik_result['pos_err']:.6e} m")
                logger.info(f"  姿态误差: {ik_result['ori_err']:.6e} rad")
                logger.info(f"  关节角度 (rad): {[f'{q:+.4f}' for q in ik_result['q']]}")
                logger.info(f"  关节角度 (deg): {[f'{np.rad2deg(q):+.2f}' for q in ik_result['q']]}")

            # Execute motion if requested
            if execute:
                result = self.set_robot_state(
                    target_joints=ik_result['q'],
                    joint_format='rad',
                    speed_deg_s=speed_deg_s,
                    gripper_speed_deg_s=gripper_speed_deg_s,
                    wait_for_completion=True
                )
                ik_result['motion_executed'] = result
            else:
                ik_result['motion_executed'] = False
                if display:
                    logger.info("  (未执行运动，execute=False)")

            return ik_result
        else:
            error_msg = ik_result.get('message', '未知错误')
            if display:
                logger.error(f"✗ IK 求解失败: {error_msg}")
                logger.error(f"  迭代次数: {ik_result.get('iters', 'N/A')}")
                logger.error(f"  位置误差: {ik_result.get('pos_err', float('inf')):.6e} m")
                logger.error(f"  姿态误差: {ik_result.get('ori_err', float('inf')):.6e} rad")

            return ik_result
    
    def set_pose_target(self, 
                       target_pose: List[float], 
                       backend: str = 'numpy', 
                       method: str = 'dls', 
                       display: bool = True, 
                       tolerance: float = 1e-3, 
                       max_iters: int = 1000,
                       multi_start: int = 0,
                       use_random_init: bool = False,
                       speed_factor: float = 1.0,
                       execute: bool = True,
                       joint_limits: Optional[Tuple[List[float], List[float]]] = None) -> Dict:
        """Move end-effector to target pose using inverse kinematics.
        
        .. deprecated:: 1.0.0
           Use :meth:`set_pose` instead. This method is kept for backward compatibility.

        :param target_pose: Target pose as [x, y, z, qx, qy, qz, qw]
        :param backend: Computation backend, 'numpy' or 'torch'
        :param method: IK solver method, 'dls', 'pinv', or 'transpose'
        :param display: Display solution details
        :param tolerance: Position and orientation tolerance
        :param max_iters: Maximum number of iterations
        :param multi_start: Number of multi-start attempts, 0 to disable
        :param use_random_init: Use random initial guess instead of current pose
        :param speed_factor: Motion speed multiplier
        :param execute: Execute motion if True
        :param joint_limits: Custom joint limits (deprecated, not used)
        :return: Dictionary with success, q, iters, pos_err, ori_err, message
        """
        import warnings
        warnings.warn(
            "set_pose_target() is deprecated. Use set_pose() instead.",
            DeprecationWarning,
            stacklevel=2
        )
        
        # Convert speed_factor to speed_deg_s
        # Default speed is typically around 5-10 deg/s
        default_speed_deg_s = 10
        speed_deg_s = int(default_speed_deg_s * speed_factor)
        
        # Use new set_pose method with converted parameters
        return self.set_pose(
            target_pose=target_pose,
            backend=backend,
            method=method,
            display=display,
            tolerance=tolerance,
            max_iters=max_iters,
            multi_start=multi_start,
            use_random_init=use_random_init,
            speed_deg_s=speed_deg_s,
            execute=execute
        )


    
    def get_firmware_version(self, timeout=5.0, send_interval=0.2):
        """Query robot firmware version.
        使用事件等待机制：发送指令后等待后台线程解析回复。

        :param timeout: Total time in seconds to keep trying
        :param send_interval: Time in seconds between each attempt
        :return: Firmware version string, or None if query fails
        """
        start_time = time.time()
        
        # Check if the firmware version is already in the json file
        if os.path.exists(os.path.join(os.path.dirname(__file__), "firmware_version.json")):
            with open(os.path.join(os.path.dirname(__file__), "firmware_version.json"), "r") as f:
                firmware_version = json.load(f)["firmware_version"]
                self.firmware_version = firmware_version
        else:
            firmware_version = None
        
        if firmware_version:
            return firmware_version

        # Use acquire_info with wait=True to let background thread handle the response
        try:
            while (time.time() - start_time) < timeout:
                try:
                    # Use acquire_info which will send command and wait for event
                    success = self.servo_driver.acquire_info("version", wait=True, timeout=send_interval)
                    
                    if success:
                        version = self.data_parser.get_firmware_version()
                        if version and version != "未知版本":
                            # Save it into a json file
                            with open(os.path.join(os.path.dirname(__file__), "firmware_version.json"), "w") as f:
                                json.dump({"firmware_version": version}, f)
                            return version  # Success, return the version immediately

                except Exception as e:
                    logger.error(f"An error occurred during a read attempt: {e}")

                time.sleep(send_interval)
        except Exception as e:
            logger.error(f"Failed to get firmware version: {e}")

        return None
    
    # ==================== Advanced Trajectory Methods ====================
    
    def move_joint_trajectory(self,
                             q_end: List[float],
                             duration: float = 2.0,
                             method: str = 'cubic',
                             num_points: int = 100,
                             visualize: bool = False) -> bool:
        """Execute smooth joint trajectory to target position.

        :param q_end: Target joint angles in radians
        :param duration: Trajectory duration in seconds
        :param method: Interpolation method, 'linear', 'cubic', or 'quintic'
        :param num_points: Number of trajectory waypoints
        :param visualize: Enable trajectory visualization
        :return: True if successful
        """
        q_start = self.get_robot_state("joint")
        if q_start is None:
            logger.error("无法获取当前关节角度")
            return False
        
        q_start = np.array(q_start)
        q_end = np.array(q_end)
        
        # 检查关节限位
        q_end, violations = check_and_clip_joint_limits(
            joints=q_end.tolist(),
            joint_limits=self.robot_model.joint_limits
        )
        q_end = np.array(q_end)
        
        for joint_name, original, clipped in violations:
            logger.warning(f"{joint_name} 超出限制：{original:.2f} -> {clipped:.2f}")
        
        # 生成轨迹
        logger.info(f"使用 {method} 插值生成关节轨迹 (时长: {duration}s, 点数: {num_points})")
        
        if method == 'linear':
            _, q, _, _ = linear_joint_trajectory(q_start, q_end, duration, num_points)
        elif method == 'cubic':
            _, q, _, _ = cubic_polynomial_trajectory(q_start, q_end, duration, num_points)
        elif method == 'quintic':
            _, q, _, _ = quintic_polynomial_trajectory(q_start, q_end, duration, num_points)
        else:
            logger.error(f"不支持的插值方法: {method}")
            return False
        
        # 执行轨迹
        delay = duration / num_points
        self.hardware_executor.delay = delay
        
        result = self.hardware_executor.execute(
            joint_traj=q.tolist(),
            visualize=visualize
        )
        
        return result if result is not None else True
    
    def move_cartesian_linear(self,
                             target_pose: List[float],
                             duration: float = 2.0,
                             num_points: int = 50,
                             ik_method: str = 'dls',
                             visualize: bool = False) -> bool:
        """Execute linear Cartesian trajectory to target pose.

        :param target_pose: Target pose as [x, y, z, qx, qy, qz, qw]
        :param duration: Trajectory duration in seconds
        :param num_points: Number of trajectory waypoints
        :param ik_method: IK solver method
        :param visualize: Enable trajectory visualization
        :return: True if successful
        """
        # 获取当前位姿
        current_pose_dict = self.get_pose()
        if current_pose_dict is None:
            logger.error("无法获取当前位姿")
            return False
        
        pose_start = current_pose_dict['transform']
        
        # 构建目标位姿矩阵
        position = np.array(target_pose[:3])
        quaternion = np.array(target_pose[3:])
        rotation_matrix = quaternion_to_matrix(quaternion)
        pose_end = make_transform(rotation_matrix, position)
        
        # 获取当前关节角度作为IK初始猜测
        q_init = self.get_robot_state("joint")
        if q_init is None:
            logger.error("无法获取当前关节角度")
            return False
        q_init = np.array(q_init)
        
        logger.info(f"生成笛卡尔直线轨迹 (时长: {duration}s, 点数: {num_points})")
        
        # 生成轨迹
        try:
            _, _, q = linear_cartesian_trajectory(
                self.robot_model,
                pose_start,
                pose_end,
                duration,
                num_points=num_points,
                q_init=None,
                ik_backend='numpy',
                ik_method=ik_method,
                max_iters=500,
                pos_tol=1e-3,
                ori_tol=1e-3
            )
        except Exception as e:
            logger.error(f"轨迹规划失败: {e}")
            return False
        
        # 执行轨迹
        delay = duration / num_points
        self.hardware_executor.delay = delay
        
        logger.info(f"执行笛卡尔轨迹 (总点数: {len(q)})")

        result = self.hardware_executor.execute(
            joint_traj=q.tolist(),
            visualize=visualize, 
        )
        
        return result if result is not None else True

    def plan_joint_trajectory(
        self,
        waypoints: np.ndarray,
        planner_type: str = 'b_spline',
        duration: Optional[float] = None,
        num_points: int = 800,
        bspline_degree: int = 5,
        segment_method: str = 'quintic',
        duration_per_segment: Optional[float] = None,
        num_points_per_segment: int = 100,
        gripper_waypoints: Optional[np.ndarray] = None
    ) -> Dict[str, Any]:
        """Plan joint space trajectory through waypoints.
        
        :param waypoints: Array of joint waypoints [n_waypoints, n_dof] in radians
        :param planner_type: Planner type, 'b_spline' or 'multi_segment'
        :param duration: Total trajectory duration in seconds (for B-Spline)
        :param num_points: Number of points in trajectory (for B-Spline)
        :param bspline_degree: B-Spline degree, 3 (cubic) or 5 (quintic)
        :param segment_method: Multi-segment method, 'cubic' or 'quintic'
        :param duration_per_segment: Duration per segment in seconds (for Multi-Segment)
        :param num_points_per_segment: Number of points per segment (for Multi-Segment)
        :param gripper_waypoints: Optional array of gripper values [n_waypoints] (0-1000)
        :return: Dictionary with trajectory data including 't', 'q', 'qd', 'qdd', and optionally 'gripper'
        """
        from robocore.planning import BSplinePlanner, MultiSegmentPlanner
        from robocore.utils.backend import to_numpy

        waypoints = to_numpy(waypoints)
        if waypoints.ndim == 1:
            waypoints = waypoints.reshape(1, -1)

        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints")

        # Create planner
        if planner_type == 'b_spline':
            planner = BSplinePlanner(degree=bspline_degree)
        elif planner_type == 'multi_segment':
            planner = MultiSegmentPlanner(method=segment_method)
        else:
            raise ValueError(f"Unknown planner type: {planner_type}. Must be 'b_spline' or 'multi_segment'")

        # Plan trajectory
        if planner_type == 'b_spline':
            trajectory = planner.plan(
                waypoints=waypoints,
                duration=duration,
                num_points=num_points
            )
        else:  # multi_segment
            if duration_per_segment is None:
                duration_per_segment = 1.0
            trajectory = planner.plan(
                waypoints=waypoints,
                durations=duration_per_segment,
                num_points_per_segment=num_points_per_segment
            )

        # Interpolate gripper values if provided
        if gripper_waypoints is not None:
            gripper_waypoints = to_numpy(gripper_waypoints)
            t_waypoints = np.linspace(0, trajectory['t'][-1], len(gripper_waypoints))
            t_traj = to_numpy(trajectory['t'])
            # Use linear interpolation for gripper
            gripper_trajectory = np.interp(t_traj, t_waypoints, gripper_waypoints)
            # Clip to valid range [0, 1000]
            gripper_trajectory = np.clip(gripper_trajectory, 0, 1000)
            trajectory['gripper'] = gripper_trajectory

        # Add waypoints to trajectory for reference
        trajectory['waypoints'] = waypoints

        return trajectory

    def plan_cartesian_trajectory(
        self,
        waypoints: np.ndarray,
        duration: Optional[float] = None,
        num_points: int = 100,
        backend: str = 'numpy'
    ) -> Dict[str, Any]:
        """Plan Cartesian space spline trajectory through waypoints.
        
        :param waypoints: Array of waypoint poses [n_waypoints, 4, 4] (transformation matrices)
                          or [n_waypoints, 3] (positions only, will use identity orientation)
        :param duration: Total trajectory duration in seconds (optional, auto-estimated if None)
        :param num_points: Number of points in trajectory
        :param backend: Computation backend, 'numpy' or 'torch' (numpy recommended for smooth splines)
        :return: Dictionary with 't', 'poses', 'positions', 'orientations', 'velocities', 'accelerations'
        """
        import robocore as rc
        from robocore.planning import SplineCurvePlanner
        from robocore.utils.backend import to_numpy

        # Set backend for planning
        rc.set_backend(backend)

        # Ensure waypoints are numpy arrays
        if isinstance(waypoints, list):
            waypoints = np.array([to_numpy(wp) for wp in waypoints])
        else:
            waypoints = to_numpy(waypoints)

        # Create planner
        planner = SplineCurvePlanner()

        # Plan trajectory
        trajectory = planner.plan(
            waypoints=waypoints,
            duration=duration,
            num_points=num_points
        )

        # Convert to numpy if needed
        for key in ['t', 'positions', 'orientations', 'velocities', 'accelerations']:
            if key in trajectory:
                trajectory[key] = to_numpy(trajectory[key])

        if 'poses' in trajectory:
            trajectory['poses'] = np.array([to_numpy(pose) for pose in trajectory['poses']])

        # Add waypoint positions for reference
        if waypoints.ndim == 3 and waypoints.shape[1:] == (4, 4):
            waypoint_positions = np.array([wp[:3, 3] for wp in waypoints])
        elif waypoints.ndim == 2 and waypoints.shape[1] == 3:
            waypoint_positions = waypoints
        else:
            waypoint_positions = None

        if waypoint_positions is not None:
            trajectory['waypoints'] = waypoint_positions

        return trajectory

    def solve_ik_for_trajectory(
        self,
        target_poses: np.ndarray,
        q_init: Optional[List[float]] = None,
        method: str = 'dls',
        max_iters: int = 100,
        pos_tol: float = 1e-2,
        ori_tol: float = 1e-2,
        num_initial_guesses: int = 5,
        initial_guess_strategy: str = 'random',
        initial_guess_scale: float = 0.6,
        random_seed: Optional[int] = None,
        backend: str = 'numpy',
        use_previous_solution: bool = True
    ) -> Dict[str, Any]:
        """Solve inverse kinematics for a sequence of Cartesian poses.
        
        :param target_poses: Array of target poses [n_poses, 4, 4] (transformation matrices)
        :param q_init: Initial joint configuration (uses current joints if None)
        :param method: IK solver method, 'dls', 'pinv', or 'transpose'
        :param max_iters: Maximum IK iterations per pose
        :param pos_tol: Position tolerance in meters
        :param ori_tol: Orientation tolerance in radians
        :param num_initial_guesses: Number of initial guesses for multi-start
        :param initial_guess_strategy: Initial guess strategy ('zero', 'random', 'sobol', 'latin', 'center', 'uniform')
        :param initial_guess_scale: Scale factor for initial guesses (0.0 to 1.0)
        :param random_seed: Random seed for reproducibility
        :param backend: Computation backend, 'numpy' or 'torch'
        :param use_previous_solution: If True, use previous solution as initial guess (ensures continuity)
        :return: Dictionary with 'joint_angles', 'ik_results', 'success_rate', 'statistics'
        """
        import robocore as rc
        from robocore.kinematics.ik import inverse_kinematics
        from robocore.utils.backend import to_numpy
        import time

        # Set backend if specified (inverse_kinematics uses global backend)
        if backend is not None:
            rc.set_backend(backend)

        target_poses = to_numpy(target_poses)
        n_poses = len(target_poses)

        # Get initial joint configuration
        if q_init is None:
            q_init = self.get_robot_state("joint")
            if q_init is None:
                raise ValueError("Cannot get current joint angles. Please provide q_init.")

        q_init = np.array(q_init)
        q_current = q_init.copy()

        # Solve IK for each pose
        ik_results = []
        joint_angles = []
        success_count = 0

        start_time = time.time()

        for i, target_pose in enumerate(target_poses):
            # Use previous solution as initial guess if enabled
            if use_previous_solution and i > 0:
                q0 = q_current
                # Use fewer initial guesses for subsequent poses (we have a good initial guess)
                num_inits = min(5, num_initial_guesses) if num_initial_guesses > 1 else 1
                strategy = 'random'
            else:
                q0 = q_init if i == 0 else q_current
                num_inits = num_initial_guesses
                strategy = initial_guess_strategy if i == 0 else 'random'

            # Solve IK (max_iters, pos_tol, ori_tol are passed via solver_kwargs)
            result = inverse_kinematics(
                self.robot_model,
                target_pose,
                q0=q0,
                method=method,
                num_initial_guesses=num_inits,
                initial_guess_strategy=strategy,
                initial_guess_scale=initial_guess_scale,
                random_seed=random_seed,
                max_iters=max_iters,
                pos_tol=pos_tol,
                ori_tol=ori_tol
            )

            ik_results.append(result)

            if result['success']:
                joint_angles.append(result['q'])
                q_current = np.array(result['q'])
                success_count += 1
            else:
                # Use previous solution if available, otherwise use zeros
                if len(joint_angles) > 0:
                    joint_angles.append(joint_angles[-1])
                else:
                    joint_angles.append(np.zeros(len(self.robot_model._chain_actuated)))

        ik_time = time.time() - start_time

        joint_angles = np.array(joint_angles)
        success_rate = success_count / n_poses if n_poses > 0 else 0.0

        # Calculate statistics
        pos_errors = [r['pos_err'] for r in ik_results if r['success']]
        ori_errors = [r['ori_err'] for r in ik_results if r['success']]

        statistics = {
            'total_poses': n_poses,
            'successful': success_count,
            'failed': n_poses - success_count,
            'success_rate': success_rate,
            'ik_time': ik_time,
            'avg_time_per_pose': ik_time / n_poses if n_poses > 0 else 0.0,
            'avg_pos_error': np.mean(pos_errors) if pos_errors else None,
            'max_pos_error': np.max(pos_errors) if pos_errors else None,
            'avg_ori_error': np.mean(ori_errors) if ori_errors else None,
            'max_ori_error': np.max(ori_errors) if ori_errors else None
        }

        return {
            'joint_angles': joint_angles,
            'ik_results': ik_results,
            'success_rate': success_rate,
            'statistics': statistics
        }
    
    # ==================== System Control ====================
    def set_acceleration(self, acceleration: int = 1) -> bool:
        """Set robot acceleration.

        :param acceleration: Acceleration value
        :return: True if successful
        """
        return self.servo_driver.set_acceleration(acceleration)

    def set_speed(self, speed_deg_s: float) -> bool:
        """Set robot motion speed.

        :param speed_deg_s: Speed in degrees per second
        :return: True if successful
        """
        self.speed_deg_s = speed_deg_s
        return self.servo_driver.set_speed(speed_deg_s)
    
    
    def torque_control(self, command: str) -> bool:
        """Enable or disable robot torque.

        :param command: Command string, 'on' or 'off'
        :return: True if successful
        """
        if command == "on":
            logger.info("开启扭矩")
            return self.servo_driver.enable_torque()
        elif command == "off":
            logger.info("关闭扭矩")
            return self.servo_driver.disable_torque()
        else:
            logger.error("command 参数必须是 'on' 或 'off'")
            return False
    
    def zero_calibration(self) -> bool:
        """Execute zero position calibration procedure.

        :return: True if calibration successful
        """
        logger.warning("此操作将更改出厂零点位置，请谨慎操作")
        logger.info("开始归零校准,机械臂将失去扭矩")
        logger.info("按下回车继续, Ctrl+C 取消...")
        input()
        # 关闭扭矩
        if not self.torque_control('off'):
            logger.error("扭矩关闭失败")
            return False
        logger.info("请手动拖动机械臂到零点位置，然后按回车继续...")
        input()
        
        # 重新开启扭矩
        if not self.torque_control('on'):
            logger.error("扭矩开启失败")
            return False
        
        # 执行零点校准
        result = self.servo_driver.set_zero_position()
        if result:
            logger.info("归零校准成功")
        else:
            logger.error("归零校准失败")
        
        return result
    

    

    
    def print_state(self, continuous: bool = False, output_format: str = "deg", robot_type: str = "follower"):
        """Print current robot state.

        :param continuous: Print continuously if True, once if False
        :param output_format: Angle format, 'deg' or 'rad'
        """

        def _print_once(robot_type):

            joints_raw = self.get_robot_state("joint")
            if robot_type != "follower" and joints_raw is not None:
                # For non-follower, get additional button states if needed
                joint_state = self.get_robot_state("joint_gripper")
                if isinstance(joint_state, JointState):
                    joints_raw = (joints_raw, getattr(joint_state, 'button1', None), getattr(joint_state, 'button2', None))

            gripper = self.get_robot_state("gripper")

            pose = None

            if joints_raw is None:

                logger.warning("无法获取关节状态")

                return
            # Normalize angles and (optionally) buttons depending on robot_type
            if robot_type == "follower":
                joints = joints_raw
                button1 = button2 = None
                pose = self.get_pose()
            else:
                joints = joints_raw[0]
                button1 = joints_raw[1]
                button2 = joints_raw[2]

            # Format joints for printing
            if output_format == 'deg':
                joint_out = [round(angle * 180.0 / np.pi, 2) for angle in joints]
                unit = "°"
            else:
                joint_out = [round(angle, 3) for angle in joints]
                unit = "rad"
            logger.info(f"关节角度（{unit}）：{joint_out} 夹爪开合度：{gripper}")
            if robot_type != "follower":
                logger.info(f"同步键：{button1}, 锁定键：{button2}")

            if pose is not None:
                quaternion = pose['quaternion_xyzw']
                position = pose['position']
                logger.info(f"位置(xyz /m): {[round(p, 5) for p in position]}, 四元数(qx, qy, qz, qw): {[round(q, 3) for q in quaternion]}")
                print("\n")
        if continuous:
            logger.info("开始连续状态打印，按 Ctrl+C 停止")
            try:
                while True:
                    _print_once(robot_type)
                    time.sleep(0.06)
            except KeyboardInterrupt:
                logger.info("停止连续状态打印")
        else:
            _print_once(robot_type)
    

    def _generate_random_q(self, scale: float = 0.5) -> List[float]:
        """Generate random joint configuration within limits.

        :param scale: Range scale factor within joint limits
        :return: Random joint angles in radians
        """
        rng = np.random.default_rng()
        q = [0.0] * self.robot_model.num_dof()

        for js in self.robot_model._actuated:
            lo, hi = -1.0, 1.0
            if js.limit:
                if js.limit[0] is not None:
                    lo = js.limit[0]
                if js.limit[1] is not None:
                    hi = js.limit[1]
            mid = 0.5 * (lo + hi)
            span = 0.5 * (hi - lo) * scale
            q[js.index] = float(rng.uniform(mid - span, mid + span))

        return q


    def _wait_for_joint_target(self,
                               target_joints: List[float],
                               tolerance_deg: float = 5.0,
                               timeout: float = 120.0,
                               log_prefix: str = "等待关节接近目标") -> bool:
        """Wait until all joints reach target angles.
        
        基于单片机反馈的实际位置与目标位置进行比较，
        当所有关节的误差都在容差范围内时，判断电机到位。
        
        对于MIT模式,会持续发送目标位置指令以保持运动。

        :param target_joints: Target joint angles in radians
        :param tolerance_deg: Degrees, acceptable abs distance to target for all joints (default ±5°)
        :param timeout: Seconds, maximum wait time (default 120s)
        :param log_prefix: Log message prefix
        :return: True if target reached, False if timeout
        """
        start_time = time.time()
        RAD_TO_DEG = 180.0 / np.pi
        
        # 将目标角度转换为度，用于比较
        target_joints_deg = [a * RAD_TO_DEG for a in target_joints]
        
        # 检查是否为MIT模式
        is_mit_mode = self.control_mode in (
            self.servo_driver.PATTERN_MIT,
            self.servo_driver.PATTERN_MIT_POSITION,
            self.servo_driver.PATTERN_MIT_SPEED,
            self.servo_driver.PATTERN_MIT_TORQUE
        )
        
        if is_mit_mode:
            logger.info(f"{log_prefix}... (MIT模式-持续发送, 容差: ±{tolerance_deg}°, 超时: {timeout}s)")
            # MIT模式说明：在MIT模式下没有实时位置反馈，只有控制指令的接收状态反馈
            # 因此我们基于时间等待，而不是位置误差判断
            print(f"[MIT模式] 目标角度: {[round(a, 1) for a in target_joints_deg[:min(len(target_joints_deg), 6)]]}°")
            print(f"[MIT模式] 注意: MIT模式无实时位置反馈，将持续发送控制指令 {timeout}秒")
        else:
            logger.info(f"{log_prefix}... (容差: ±{tolerance_deg}°, 超时: {timeout}s)")
        
        last_send_time = 0
        send_interval = 0.005  # MIT模式发送频率: 200Hz

        while time.time() - start_time < timeout:
            # MIT模式需要持续发送目标位置
            if is_mit_mode and (time.time() - last_send_time) >= send_interval:
                self.servo_driver.set_joint_and_gripper(
                    joint_angles=target_joints,
                    gripper_value=None,  # 保持当前夹爪状态
                    speed_deg_s=self.speed_deg_s if hasattr(self, 'speed_deg_s') else 57.3,
                    torque_nm=0.0,
                    control_aim=self.control_aim,
                    control_mode=self.control_mode
                )
                last_send_time = time.time()
            
            # MIT模式：没有实时位置反馈，基于时间判断
            if is_mit_mode:
                elapsed = time.time() - start_time
                # 简单的时间判断：假设电机需要一定时间到达目标
                # 可以根据目标角度与起始角度的差值来估算所需时间
                # 这里使用一个简单的策略：持续发送一段时间后认为已到达
                estimated_time = min(timeout * 0.5, 3.0)  # 最多等待3秒或超时时间的一半
                if elapsed >= estimated_time:
                    print(f"[MIT模式] 已持续发送 {elapsed:.2f}s，假定已到达目标")
                    logger.info(f"✓ MIT模式控制完成 (耗时: {elapsed:.2f}s)")
                    return True
                # 短暂休眠
                time.sleep(0.05)
                continue
            
            # 非MIT模式：后台线程会持续更新关节状态，这里直接读取最新数据
            current_joints = self.get_robot_state("joint")
            
            if current_joints is not None:
                # 将当前角度转换为度
                current_joints_deg = [a * RAD_TO_DEG for a in current_joints]
                
                # 只比较前6个关节（排除夹爪）
                joints_to_compare = min(len(current_joints_deg), len(target_joints_deg), 6)
                
                # 计算每个关节的误差（度）
                errors_deg = [abs(cur - tgt) for cur, tgt in zip(current_joints_deg[:joints_to_compare], target_joints_deg[:joints_to_compare])]
                max_error = max(errors_deg) if errors_deg else 0
                
                # 每秒打印一次当前状态（使用 print 确保输出）
                elapsed = time.time() - start_time
                if int(elapsed) % 1 == 0 and int(elapsed) != getattr(self, '_last_print_time', -1):
                    self._last_print_time = int(elapsed)
                    print(f"[位置检测] 目标: {[round(a, 1) for a in target_joints_deg[:joints_to_compare]]}°")
                    print(f"[位置检测] 当前: {[round(a, 1) for a in current_joints_deg[:joints_to_compare]]}°")
                    print(f"[位置检测] 误差: {[round(e, 1) for e in errors_deg]}°, 最大误差: {max_error:.1f}°, 容差: {tolerance_deg}°")
                
                # 判断所有关节是否都在容差范围内
                if all(err <= tolerance_deg for err in errors_deg):
                    logger.info(f"✓ 已到达目标位置 (耗时: {elapsed:.2f}s, 最大误差: {max_error:.2f}°)")
                    return True
            
            # 短暂休眠，避免 CPU 空转，同时等待后台线程更新数据
            # time.sleep(0.1)

        # 超时，打印当前状态
        current_joints = self.get_robot_state("joint")
        if current_joints is not None:
            current_joints_deg = [a * RAD_TO_DEG for a in current_joints]
            joints_to_compare = min(len(current_joints_deg), len(target_joints_deg), 6)
            errors_deg = [abs(cur - tgt) for cur, tgt in zip(current_joints_deg[:joints_to_compare], target_joints_deg[:joints_to_compare])]
            logger.warning(f"等待关节到目标附近超时 ({timeout}s)")
            logger.warning(f"  目标角度 (deg): {[round(a, 2) for a in target_joints_deg[:joints_to_compare]]}")
            logger.warning(f"  当前角度 (deg): {[round(a, 2) for a in current_joints_deg[:joints_to_compare]]}")
            logger.warning(f"  误差 (deg): {[round(e, 2) for e in errors_deg]}")
        else:
            logger.warning("等待关节到目标附近超时，且无法获取当前关节状态")
        
        return False
    

    def __del__(self):
        try:
            self.disconnect()
        except Exception as e:
            logger.error(f"SynriaRobotAPI destructor exception: {e}")