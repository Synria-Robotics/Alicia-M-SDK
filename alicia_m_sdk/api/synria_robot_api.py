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
from typing import List, Optional, Dict, Union, Tuple
import numpy as np
import json
import os
# Import from robocore for kinematics and planning
from robocore.kinematics import inverse_kinematics
from robocore.modeling import RobotModel
from robocore.transform import make_transform, quaternion_to_matrix
from robocore.kinematics import forward_kinematics
from robocore.transform import matrix_to_euler, matrix_to_quaternion
from robocore.planning.trajectory import (
    cubic_polynomial_trajectory,
    quintic_polynomial_trajectory,
    linear_joint_trajectory,
    linear_cartesian_trajectory,
)
from ..hardware import ServoDriver
from ..execution import HardwareExecutor, JointInterpolator
from ..utils.logger import logger
from robocore.utils.control_utils import compute_steps_and_delay, validate_joint_list, check_and_clip_joint_limits
from ..utils.calculate import calculate_movement_duration


class SynriaRobotAPI:
    """Synria robot arm API - provides unified user interface"""
    
    def __init__(self,
                 servo_driver: ServoDriver,
                 robot_model: RobotModel,
                 firmware_version: None,
                 speed_rad_s: float = 0.087266,
                 control_aim: int = None,
                 control_mode: tuple = None):
        """Initialize robot API.

        :param servo_driver: Servo driver instance
        :param robot_model: Robot model from RoboCore
        :param firmware_version: Firmware version string
        :param speed_rad_s: Default speed in radians per second
        :param control_aim: 控制目标 (AIM_TEACH, AIM_OPERATION, etc.)
        :param control_mode: 控制模式 (PATTERN_PV, PATTERN_PVT, PATTERN_V, PATTERN_MIT)
        """
        # 核心组件
        self.servo_driver = servo_driver
        self.data_parser = servo_driver.data_parser  # Direct access to data parser
        self.robot_model = robot_model
        self.speed_rad_s = speed_rad_s
        self.firmware_version = firmware_version
        self.firmware_new = False
        
        # 控制目标和模式 (使用 ServoDriver 的常量)
        self.control_aim = control_aim if control_aim is not None else ServoDriver.AIM_TEACH
        self.control_mode = control_mode if control_mode is not None else ServoDriver.PATTERN_PV

        # 创建各层组件
        self.hardware_executor = HardwareExecutor(servo_driver)
        # 默认参数
        self.home_angles = [0.0] * 6


    
    # ==================== Connection Management ====================
    
    def connect(self) -> bool:
        """Connect to robot and detect firmware version.
        
        初始化顺序：
        1. 连接串口
        2. 查询固件版本（一问一答）
        3. 根据固件版本配置参数
        4. 启动后台更新线程

        :return: True if connection successful
        """
        result = self.servo_driver.connect()
        if not result:
            return False
        
        # 1. 先查询固件版本（此时后台线程尚未启动，不会有指令冲突）
        if not self.firmware_version:
            self.firmware_version = self.get_firmware_version(timeout=2.0)
            logger.info(f"Detected firmware version: {self.firmware_version}")

        # 2. 根据固件版本配置参数
        if self.firmware_version and (self.firmware_version.startswith("1.") or self.firmware_version.startswith("v1.")):
            self.set_speed(self.speed_rad_s)
            self.servo_driver.firmware_new = True
            self.servo_driver.data_parser.firmware_new = True
            self.firmware_new = True

        # 3. 最后启动后台更新线程
        self.servo_driver.start_update_thread()
        self.servo_driver.wait_for_valid_state()

        return True
    
    def disconnect(self):
        """Disconnect from robot and stop update threads."""
        self.servo_driver.stop_update_thread()
        self.servo_driver.disconnect()
    
    def is_connected(self) -> bool:
        """Check if robot is connected.

        :return: True if connection is active
        """
        return self.servo_driver.serial_comm.is_connected()
    
    # ==================== Robot Control ====================                         
    

    def set_home(self, speed_factor: float = 1, tolerance: float = 0.03, timeout: float = 10.0):
        """Move robot to home position and wait until near zero.

        :param speed_factor: Speed multiplier for motion
        :param tolerance: Degrees, acceptable abs distance to zero for all joints (default 5°)
        :param timeout: Seconds, maximum wait time
        """
        time.sleep(0.1)

        # 转换容差为度（如果是弧度制则转换）
        tolerance_deg = tolerance * 180.0 / np.pi if tolerance < 1.0 else tolerance
        tolerance_deg = max(tolerance_deg, 5.0)

        if self.firmware_new:
            self.set_joint_target(self.home_angles, joint_format='rad', tolerance=tolerance_deg, timeout=timeout)
        else:
            self.set_joint_target_interplotation(self.home_angles, joint_format='rad', speed_factor=speed_factor)
            # Active feedback: block until joints are near zero
        return self._wait_for_joint_target(
                target_joints=self.home_angles,
                tolerance_deg=tolerance_deg,
                timeout=timeout,
                log_prefix="等待关节接近零位"
            )


    def set_joint_target(self,
                         target_joints: List[float],
                         joint_format: str = 'rad',
                         tolerance: float = 0.03,
                         timeout: Optional[float] = None,
                         wait_for_completion: bool = True,
                         speeds: Optional[Union[float, List[float]]] = None,
                         torques: Optional[Union[float, List[float]]] = None) -> bool:
        """Move robot to target joint angles and optionally wait until near target.

        :param target_joints: Target joint angles
        :param joint_format: Unit format, 'rad' or 'deg'
        :param tolerance: Rad, acceptable abs distance to target for all joints
        :param timeout: Seconds, maximum wait time; default derived from distance and speed
        :param wait_for_completion: If True, wait until joints reach target; if False, return immediately after sending command
        :param speeds: Optional speed(s) for the movement (rad/s). Can be a single float or a list of 6 floats.
                       If None, uses self.speed_rad_s.
        :param torques: Optional torque(s) for the movement (N·m). Can be a single float or a list of 6 floats.
                        Used in PATTERN_PVT and PATTERN_MIT modes.
        :return: True if command sent successfully (and reached target if wait_for_completion=True)
        """
        # Handle 7-element input for joints (6 joints + 1 gripper)
        gripper_val = None
        if len(target_joints) == 7:
            gripper_val = target_joints[6]
            target_joints = target_joints[:6]

        if joint_format == 'deg':
            target_joints = [a * np.pi / 180.0 for a in target_joints]
        
        # Use provided speeds or default speed
        current_speed = speeds if speeds is not None else self.speed_rad_s
        
        # Use provided torques or default (0.0)
        current_torque = torques if torques is not None else 0.0
        
        # Speed is already in rad/s, no conversion needed
        speed_rad_s = current_speed

        sucess = self.servo_driver.set_joint_and_gripper(
            joint_angles=target_joints, 
            gripper_value=gripper_val, 
            speed_rad_s=speed_rad_s,
            torque_nm=current_torque,
            control_aim=self.control_aim,
            control_mode=self.control_mode
        )

        if not sucess:
            return False

        # If not waiting, return immediately after successful command send
        if not wait_for_completion:
            return True

        # calculate the delay for the next movement based on speed
        calc_speed = self.speed_rad_s
        if isinstance(current_speed, (int, float)):
            calc_speed = current_speed
        elif isinstance(current_speed, list) and len(current_speed) > 0:
             calc_speed = sum(abs(s) for s in current_speed) / len(current_speed)

        current_joints = self.get_joints()
        delay = calculate_movement_duration(current_joints, target_joints, calc_speed)

        # Active feedback wait until close to target (or timeout)
        # 默认超时时间 = 预估时间 * 2 + 10秒缓冲，确保有足够时间
        max_wait = timeout if timeout is not None else max(30.0, delay * 2.0 + 10.0)
        
        # tolerance 参数转换为度（如果是弧度制输入则转换）
        tolerance_deg = tolerance * 180.0 / np.pi if tolerance < 1.0 else tolerance
        # 默认容差为 5 度
        tolerance_deg = max(tolerance_deg, 5.0)
        
        return self._wait_for_joint_target(
            target_joints=target_joints,
            tolerance_deg=tolerance_deg,
            timeout=max_wait,
            log_prefix="等待关节接近目标"
        )
    
    def set_joint_target_no_wait(self,
                                 target_joints: List[float],
                                 joint_format: str = 'rad',
                                 speeds: Optional[Union[float, List[float]]] = None,
                                 torques: Optional[Union[float, List[float]]] = None) -> bool:
        """Move robot to target joint angles without waiting for completion.
        
        This is a non-blocking version of set_joint_target. The command is sent
        to the robot and the function returns immediately without waiting for
        the robot to reach the target position.
        
        :param target_joints: Target joint angles
        :param joint_format: Unit format, 'rad' or 'deg'
        :param speeds: Optional speed(s) for the movement (rad/s). If None, uses self.speed_rad_s.
        :param torques: Optional torque(s) for the movement (N·m). Used in PATTERN_PVT and PATTERN_MIT modes.
        :return: True if command sent successfully, False otherwise
        """
        if joint_format == 'deg':
            target_joints = [a * np.pi / 180.0 for a in target_joints]
        
        # Use provided speeds or default speed
        current_speed = speeds if speeds is not None else self.speed_rad_s
        current_torque = torques if torques is not None else 0.0
        
        # Speed is already in rad/s, no conversion needed
        speed_rad_s = current_speed
        
        return self.servo_driver.set_joint_and_gripper(
            joint_angles=target_joints, 
            speed_rad_s=speed_rad_s,
            torque_nm=current_torque,
            control_aim=self.control_aim,
            control_mode=self.control_mode
        )

    
    def set_joint_target_interplotation(self,
              target_joints: List[float],
              joint_format: str = 'rad',
              speed_factor: float = 1.0,
              T_default: float = 4.0,
              n_steps_ref: int = 200,
              visualize: bool = False) -> bool:
        """Move robot to target joint angles with interpolation.

        :param target_joints: Target joint angles
        :param joint_format: Unit format, 'rad' or 'deg'
        :param speed_factor: Speed multiplier
        :param T_default: Default interpolation duration in seconds
        :param n_steps_ref: Reference number of interpolation steps
        :param visualize: Enable trajectory visualization
        :return: True if motion started successfully
        """
        logger.info("[moveJ] 开始执行关节空间插值移动")
        
        if target_joints is None:
            logger.error("[moveJ] 请提供 target_joints 参数")
            return False
        
        # Handle 7-element input (6 joints + 1 gripper)
        if len(target_joints) == 7:
            target_joints = target_joints[:6]
        
        # 参数验证和转换
        joint_format = joint_format.lower()
        if joint_format not in ['rad', 'deg']:
            logger.error(f"[moveJ] 不支持的 joint_format: '{joint_format}'，应为 'rad' 或 'deg'")
            return False
        
        # 支持角度制输入
        if joint_format == 'deg':
            logger.info("[moveJ] 输入角度单位为 degree，将转换为 rad")
            target_joints = [a * np.pi / 180.0 for a in target_joints]
        
        # 验证关节列表
        validate_joint_list(target_joints)
        
        # 检查关节限位并修正
        target_joints, violations = check_and_clip_joint_limits(
            joints=target_joints,
            joint_limits=self.robot_model.joint_limits
        )
        
        for joint_name, original, clipped in violations:
            logger.warning(
                f"[moveJ] {joint_name} 超出限制：{original:.2f} -> 已截断为 {clipped:.2f}"
            )
        
        # 获取当前状态（直接从servo_driver获取，带重试机制）
        cur_angles = self.get_joints()
        
        # 插值步数与延迟
        steps, delay = compute_steps_and_delay(
            speed_factor=speed_factor,
            T_default=T_default,
            n_steps_ref=n_steps_ref
        )
        interploter = JointInterpolator()
        # 规划轨迹
        joint_traj = interploter.plan(
            start_angles=cur_angles,
            target_angles=target_joints,
            steps=steps
        )
        
        # 显示起始和目标角度
        if joint_format == 'deg':
            display_cur = [round(a * 180.0 / np.pi, 1) for a in cur_angles]
            display_target = [round(a * 180.0 / np.pi, 1) for a in target_joints]
            unit = "°"
        else:
            display_cur = [round(a, 3) for a in cur_angles]
            display_target = [round(a, 3) for a in target_joints]
            unit = "rad"
        
        # 日志输出
        logger.info(f"[moveJ] 起始角度 ({unit}): {display_cur}")
        logger.info(f"[moveJ] 目标角度 ({unit}): {display_target}")
        logger.info(f"[moveJ] 插值步数: {steps}，单步延迟: {delay:.3f}s，预计总耗时: {steps * delay:.1f}s")
        
        # 执行轨迹
        self.hardware_executor.delay = delay
        result = self.hardware_executor.execute(
            joint_traj=joint_traj,
            visualize=visualize
        )
        
        return result if result is not None else True
    

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
                current_gripper = self.get_gripper()
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
                        speed_deg_s: int = 10,
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
        :param speed_deg_s: Motion speed in degrees per second
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
            q_init = self.get_joints()
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
                # Convert speed_deg_s to speed_rad_s for M-SDK
                speed_rad_s = speed_deg_s * np.pi / 180.0
                if self.firmware_new:
                    result = self.set_joint_target(
                        ik_result['q'], 
                        joint_format='rad',
                        speeds=speed_rad_s
                    )
                else:
                    # For old firmware, use speed_factor based on default speed
                    speed_factor = speed_rad_s / self.speed_rad_s if self.speed_rad_s > 0 else 1.0
                    result = self.set_joint_target_interplotation(
                        ik_result['q'], 
                        joint_format='rad',
                        speed_factor=speed_factor
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
        # Default speed_rad_s is typically around 0.087266 rad/s (5 deg/s)
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

    def get_joints(self, type: str = "follower") -> Optional[Union[List[float], Tuple[List[float], bool, bool]]]:
        """Get current joint angles.
        :param type: 'follower' -> return angles only; other values -> (angles, button1, button2)
        :return: Angles list, or (angles, button1, button2) if requested; None if unavailable
        """
        joint_state = self.data_parser.get_joint_state()
        if joint_state:
            if type == "follower":
                return joint_state.angles
            return (joint_state.angles, joint_state.button1, joint_state.button2)
        return None
    
    def get_pose(self) -> Optional[Union[List[float], Dict]]:
        """Get current end-effector pose.

        :return: Dictionary with position, rotation, euler_xyz, quaternion_xyzw, transform, or None if failed
        """

        joint_angles = self.get_joints()
        if joint_angles is None:
            logger.error("无法获取关节角度")
            return None

        T_fk = forward_kinematics(
            self.robot_model,
            joint_angles,
            return_end=True
        )

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

    
    def get_gripper(self) -> Optional[float]:
        """Get current gripper position.

        :return: Gripper value from 0 (closed) to 100 (open)
        """

        joint_state = self.data_parser.get_joint_state()
        if joint_state:
            return joint_state.gripper
        return None

    
    def get_firmware_version(self, timeout=5.0, send_interval=0.2):
        """Query robot firmware version.
        使用一问一答机制：发送指令后等待单片机回复。

        :param timeout: Total time in seconds to keep trying
        :param send_interval: Time in seconds between each attempt
        :return: Firmware version string, or None if query fails
        """
        command = [0xAA, 0x01, 0x7E, 0x01, 0xFE, 0x79, 0xFF]
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

        # 如果后台线程正在运行，先暂停它
        was_thread_running = self.servo_driver.is_update_thread_running()
        if was_thread_running:
            self.servo_driver._pause_update.set()

        try:
            while (time.time() - start_time) < timeout:
                try:
                    # 使用一问一答机制：发送指令并等待回复
                    frame = self.servo_driver.serial_comm.send_and_wait_response(command, timeout=1.0)
                    if frame:
                        # 解析接收到的数据帧
                        self.data_parser.parse_frame(frame)
                    
                    version = self.data_parser.get_firmware_version()
                    if version and version != "未知版本":
                        # Save it into a json file
                        with open(os.path.join(os.path.dirname(__file__), "firmware_version.json"), "w") as f:
                            json.dump({"firmware_version": version}, f)
                        return version  # Success, return the version immediately

                except Exception as e:
                    logger.error(f"An error occurred during a read attempt: {e}")

                time.sleep(send_interval)
        finally:
            # 恢复后台线程
            if was_thread_running:
                self.servo_driver._pause_update.clear()

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
        q_start = self.get_joints()
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
        q_init = self.get_joints()
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
    

    
    # ==================== System Control ====================
    def set_acceleration(self, acceleration: int = 1) -> bool:
        """Set robot acceleration.

        :param acceleration: Acceleration value
        :return: True if successful
        """
        return self.servo_driver.set_acceleration(acceleration)

    def set_speed(self, speed_rad_s: float) -> bool:
        """Set robot motion speed.

        :param speed_rad_s: Speed in radians per second
        :return: True if successful
        """
        self.speed_rad_s = speed_rad_s
        return self.servo_driver.set_speed(speed_rad_s)
    
    
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


            joints_raw = self.get_joints(robot_type)

            gripper = self.get_gripper()

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
                logger.info(f"位置(xyz /m): {[round(p, 3) for p in position]}, 四元数(qx, qy, qz, qw): {[round(q, 3) for q in quaternion]}")
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
        send_interval = 0.02  # MIT模式发送频率: 50Hz

        while time.time() - start_time < timeout:
            # MIT模式需要持续发送目标位置
            if is_mit_mode and (time.time() - last_send_time) >= send_interval:
                self.servo_driver.set_joint_and_gripper(
                    joint_angles=target_joints,
                    gripper_value=None,  # 保持当前夹爪状态
                    speed_rad_s=self.speed_rad_s if hasattr(self, 'speed_rad_s') else 1.0,
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
            current_joints = self.get_joints()
            
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
            time.sleep(0.1)

        # 超时，打印当前状态
        current_joints = self.get_joints()
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