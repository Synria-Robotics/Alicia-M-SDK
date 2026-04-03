"""SynriaRobotAPI — Alicia-M 机器人 SDK 主接口

精简的门面（Facade）类，将具体逻辑委托给控制层、运动学接口、规划接口。
通过 create_robot() 工厂函数创建实例。
"""

from __future__ import annotations

import math
import time
import logging
import warnings
from typing import Dict, List, Optional, Any

import numpy as np

from ..types.state import JointState, MitParams
from ..types.config import RobotConfig
from ..types.enums import ControlMode, ControlAim
from ..types.exceptions import (
    ConnectionError, TimeoutError, RobotStateError,
)
from ..protocol.codec import MessageCodec
from ..protocol.constants import (
    AIM_LEADER, AIM_FOLLOWER, CMD_VERSION, FUNC_WRITE_BIT,
)
from ..hardware.serial_port import SerialPort
from ..hardware.device import Device
from ..control.joint_control import JointController
from ..control.trajectory_executor import TrajectoryExecutor
from ..control.teaching import DragTeaching
from .. import kinematics as kin_module
from .. import planning as plan_module

logger = logging.getLogger(__name__)


class SynriaRobotAPI:
    """Alicia-M 机器人 SDK 主接口

    提供面向用户的统一控制入口，内部委托各子模块执行具体逻辑。

    Args:
        config: 机器人配置
        robot_model: RoboCore RobotModel 实例（由 create_robot 传入）
    """

    def __init__(self, config: RobotConfig, robot_model=None):
        self._config = config
        self._serial_port = SerialPort(config.port, config.baudrate)
        self._codec = MessageCodec()
        self._device = Device(self._serial_port, self._codec)
        self._joint_ctrl = JointController(
            self._device,
            control_mode=config.control_mode,
            joint_limits_lower=config.joint_limits_lower,
            joint_limits_upper=config.joint_limits_upper,
        )
        self._traj_executor = TrajectoryExecutor(self._device)
        self._teaching = DragTeaching(self._device, self._joint_ctrl)
        self._robot_model = robot_model
        self._connected = False

    @property
    def robot_model(self):
        """获取 RoboCore 机器人模型实例"""
        return self._robot_model

    # ========== 连接管理 ==========

    def connect(self, timeout: float = 5.0) -> bool:
        """连接机器人

        总超时覆盖整个连接流程：串口打开 + 后台线程启动 + 自动检测 + 首次状态获取。

        Args:
            timeout: 总超时（秒）

        Returns:
            连接成功返回 True

        Raises:
            ConnectionError: 连接失败
        """
        deadline = time.time() + timeout

        # 1. 打开串口
        if not self._serial_port.connect():
            raise ConnectionError("串口连接失败，请检查设备连接和端口权限")

        # 2. 启动后台线程
        self._device.start()

        # 3. 自动检测控制目标
        if self._config.control_aim:
            aim_str = self._config.control_aim.lower()
            aim = AIM_LEADER if aim_str == "leader" else AIM_FOLLOWER
            self._device.set_aim(aim)
        else:
            # 通过版本查询自动检测
            self._auto_detect_aim(max(deadline - time.time(), 0.5))

        # 4. 等待首次状态缓存填充
        poll_deadline = min(deadline, time.time() + 2.0)
        while time.time() < poll_deadline:
            if self._device.joint_state is not None:
                break
            time.sleep(0.05)

        # 5. 查询固件版本
        remaining = max(deadline - time.time(), 0.5)
        self.get_firmware_version(timeout=remaining)

        self._connected = True
        logger.info("机器人连接成功")
        return True

    def disconnect(self) -> None:
        """断开连接：停止后台线程 → 关闭串口"""
        self._device.stop()
        self._serial_port.disconnect()
        self._connected = False
        logger.info("机器人已断开")

    def is_connected(self) -> bool:
        """检查连接状态"""
        return self._connected and self._serial_port.is_connected()

    # ========== 状态查询 ==========

    def get_robot_state(self, info_type: str = "joint_gripper") -> Any:
        """获取机器人状态

        Args:
            info_type: 查询类型
                - "joint": 6个关节角度 (rad)
                - "joint_gripper": 关节角度 + 夹爪值
                - "velocity": 关节速度
                - "torque": 关节力矩
                - "all": 完整 JointState 对象
                - "version": 版本信息
                - "status": 运行状态

        Returns:
            对应类型的状态数据
        """
        state = self._device.joint_state

        if info_type == "joint":
            return list(state.angles) if state else None
        elif info_type == "joint_gripper":
            if state is None:
                return None
            return {
                'angles': list(state.angles),
                'gripper': state.gripper,
            }
        elif info_type == "velocity":
            return list(state.velocities) if state and state.velocities else None
        elif info_type == "torque":
            return list(state.torques) if state and state.torques else None
        elif info_type == "all":
            return state
        elif info_type == "version":
            return self._device.version_info
        elif info_type == "status":
            return self._device.robot_status
        else:
            # 未实现的类型（预留扩展）
            logger.warning(f"未实现的状态查询类型: {info_type}")
            return None

    def get_pose(self) -> Optional[Dict]:
        """获取末端位姿（通过 FK 计算）

        Returns:
            位姿字典 {transform, position, rotation, euler_xyz, quaternion_xyzw}
        """
        if self._robot_model is None:
            logger.warning("未加载机器人模型，无法计算位姿")
            return None
        state = self._device.joint_state
        if state is None:
            return None
        return kin_module.compute_forward_kinematics(
            self._robot_model, list(state.angles)
        )

    def get_firmware_version(self, timeout: float = 5.0) -> Optional[str]:
        """获取固件版本

        Args:
            timeout: 查询超时

        Returns:
            固件版本字符串，如 "v1.1.0"
        """
        frame = self._codec.encode_version_request()
        resp = self._device.send_and_wait(frame, CMD_VERSION, timeout=timeout)
        info = self._device.version_info
        return info.firmware_version if info else None

    # ========== 关节控制 ==========

    def set_robot_state(
        self,
        target_joints: Optional[List[float]] = None,
        gripper_value: Optional[float] = None,
        joint_format: str = 'deg',
        speed: float = 15,
        gripper_speed: float = 40,
        wait_for_completion: bool = True,
        **kwargs,
    ) -> bool:
        """点位运动：设置关节角度和/或夹爪位置

        自动根据当前模式选择实现：
        - PV: 发送 pos+vel 帧，固件自行到达
        - MIT: 通过线性轨迹速度 + 全参数帧

        Args:
            target_joints: 目标角度，6 个关节
            gripper_value: 夹爪值 [0, 1000]
            joint_format: 角度格式 'deg' / 'rad'
            speed: 运动速度 [0, 400]
            gripper_speed: 夹爪速度 [0, 400]
            wait_for_completion: 是否等待到达

        Returns:
            是否成功到达目标
        """
        # 角度转换
        if target_joints is not None and joint_format == 'deg':
            target_joints = [math.radians(a) for a in target_joints]

        # 仅控制夹爪（无关节目标）→ 走专用夹爪路径
        if target_joints is None:
            if gripper_value is not None:
                return self._joint_ctrl.move_gripper(
                    gripper_value, speed=gripper_speed,
                    wait=wait_for_completion,
                )
            return True  # 无目标，无操作

        mode = self._joint_ctrl._mode

        if mode == ControlMode.PV or mode == "pv":
            return self._joint_ctrl.move_pv(
                target_joints=target_joints,
                speed=speed,
                gripper=gripper_value,
                gripper_speed=gripper_speed,
                wait=wait_for_completion,
            )
        else:
            return self._joint_ctrl.move_mit(
                target_joints=target_joints,
                speed=speed,
                gripper=gripper_value,
                gripper_speed=gripper_speed,
                wait=wait_for_completion,
            )

    def go_home(self, speed: float = 15, gripper_speed: float = 40) -> bool:
        """回零位"""
        return self._joint_ctrl.go_home(speed=speed)

    def set_gripper_target(
        self,
        command: Optional[str] = None,
        value: Optional[float] = None,
        wait_for_completion: bool = True,
    ) -> bool:
        """控制夹爪

        Args:
            command: "open" / "close" / None（使用 value）
            value: 夹爪目标值 [0, 1000]
            wait_for_completion: 是否等待
        """
        if command == "open":
            value = 1000.0
        elif command == "close":
            value = 0.0
        if value is None:
            return False
        return self._joint_ctrl.move_gripper(value, wait=wait_for_completion)

    # ========== MIT 专用接口 ==========

    def send_mit_command(
        self,
        joint_params: List[MitParams],
        gripper: Optional[float] = None,
    ) -> None:
        """MIT 全参数直接发送（低延迟）

        每帧发送完整的 5 参数 MIT 数据，不等待、不插值。
        调用方需自行维持高频发送（≥200Hz）。

        Args:
            joint_params: 7 个电机的 MIT 参数
            gripper: 夹爪值，None 使用 joint_params[6].pos_ref
        """
        self._joint_ctrl.send_mit(joint_params, gripper=gripper)

    # ========== 系统控制 ==========

    def torque_control(
        self, command: str, joints: Optional[List[int]] = None
    ) -> bool:
        """力矩开关（仅 MIT 模式）

        Args:
            command: "off"=卸力, "on"=恢复
            joints: 关节索引列表，None=全部
        """
        if command == "off":
            return self._joint_ctrl.torque_off(joints)
        elif command == "on":
            return self._joint_ctrl.torque_on(joints)
        return False

    def enable_robot(self) -> bool:
        """使能机器人（含安全序列防止突跳）"""
        return self._joint_ctrl.enable()

    def disable_robot(self) -> bool:
        """失能机器人"""
        return self._joint_ctrl.disable()

    def switch_mode(self, mode: str) -> bool:
        """切换控制模式

        注意: 切换瞬间固件会短暂失能再使能，机械臂会因重力下坠。
        切到 MIT 后关节可自由活动；切回 PV 后关节锁定在当前位置。

        Args:
            mode: "pv" / "mit"
        """
        ctrl_mode = ControlMode.PV if mode.lower() == "pv" else ControlMode.MIT
        return self._joint_ctrl.switch_mode(ctrl_mode)

    def set_zero_position(self) -> bool:
        """设置当前位姿为零位"""
        return self._joint_ctrl.set_zero_position()

    # ========== 运动学 ==========

    def set_pose(
        self,
        target_pose,
        method: str = 'dls',
        execute: bool = True,
        speed: float = 7,
        **ik_params,
    ) -> Dict:
        """通过逆运动学移动到目标位姿

        Args:
            target_pose: 目标位姿（4x4矩阵 / [x,y,z,qx,qy,qz,qw]）
            method: IK 方法
            execute: 是否执行运动
            speed: 运动速度

        Returns:
            IK 结果字典
        """
        if self._robot_model is None:
            raise RobotStateError("未加载机器人模型")

        state = self._device.joint_state
        q_init = list(state.angles) if state else None

        result = kin_module.compute_inverse_kinematics(
            self._robot_model, target_pose, q_init=q_init,
            method=method, **ik_params,
        )

        if execute and result.get('success'):
            q = result['q'].tolist()
            self.set_robot_state(target_joints=q, joint_format='rad', speed=speed)
            result['motion_executed'] = True
        else:
            result['motion_executed'] = False

        return result

    # ========== 轨迹规划与执行 ==========

    def plan_joint_trajectory(
        self, waypoints, planner_type: str = 'b_spline', **kwargs
    ) -> Dict:
        """规划关节空间轨迹"""
        return plan_module.plan_joint_trajectory(waypoints, planner_type, **kwargs)

    def plan_cartesian_trajectory(self, waypoints, **kwargs) -> Dict:
        """规划笛卡尔空间轨迹"""
        return plan_module.plan_cartesian_trajectory(waypoints, **kwargs)

    def move_joint_trajectory(
        self, q_end, duration: float = 2.0, method: str = 'cubic', **kwargs
    ) -> bool:
        """执行平滑关节轨迹

        Args:
            q_end: 目标关节角度 (rad)
            duration: 运动时长
            method: 插值方法
        """
        state = self._device.joint_state
        if state is None:
            return False
        q_start = list(state.angles) + [state.gripper]
        q_end_full = list(q_end) + [state.gripper] if len(q_end) == 6 else list(q_end)

        traj = plan_module.plan_joint_trajectory(
            [q_start, q_end_full],
            planner_type='multi_segment',
            duration=duration,
            segment_type=method,
        )
        if not traj.get('success'):
            return False

        return self._traj_executor.execute_pv(
            traj['timestamps'], traj['positions'], traj['velocities']
        )

    def move_cartesian_linear(
        self, target_pose, duration: float = 2.0, **kwargs
    ) -> bool:
        """执行笛卡尔直线轨迹"""
        current_pose = self.get_pose()
        if current_pose is None:
            return False

        current_pos = current_pose['position'].tolist()
        current_quat = current_pose['quaternion_xyzw'].tolist()
        start = current_pos + current_quat

        traj = plan_module.plan_cartesian_trajectory(
            [start, target_pose], duration=duration, **kwargs
        )
        if not traj.get('success'):
            return False

        # 笛卡尔轨迹需要逐点 IK
        return self._execute_cartesian_traj(traj)

    def solve_ik_for_trajectory(
        self, target_poses, q_init=None, **kwargs
    ) -> Dict:
        """批量 IK 求解"""
        if self._robot_model is None:
            raise RobotStateError("未加载机器人模型")
        results = []
        q_current = q_init
        for pose in target_poses:
            r = kin_module.compute_inverse_kinematics(
                self._robot_model, pose, q_init=q_current, **kwargs
            )
            results.append(r)
            if r.get('success'):
                q_current = r['q'].tolist()
        return {'results': results, 'num_solved': sum(1 for r in results if r.get('success'))}

    # ========== 示教 ==========

    def start_teaching(self, interval: float = 0.05) -> None:
        """开始拖动示教录制"""
        self._teaching.start_recording(interval)

    def stop_teaching(self) -> List[List[float]]:
        """停止示教，返回路点"""
        return self._teaching.stop_recording()

    def replay_teaching(self, waypoints: List[List[float]], hz: float = 200) -> bool:
        """回放示教轨迹"""
        return self._teaching.replay(waypoints, hz)

    # ========== 状态打印 ==========

    def print_state(self, continuous: bool = False, output_format: str = "deg") -> None:
        """打印当前状态"""
        try:
            from robocore.utils.beauty_logger import beauty_print, beauty_print_array
        except ImportError:
            beauty_print = print
            beauty_print_array = lambda arr, **kw: str(arr)

        def _print_once():
            state = self._device.joint_state
            if state is None:
                beauty_print("未获取到状态数据", type="warning")
                return
            angles = list(state.angles)
            if output_format == "deg":
                angles_display = [math.degrees(a) for a in angles]
                unit = "deg"
            else:
                angles_display = list(angles)
                unit = "rad"

            beauty_print(f"关节角度 ({unit}):")
            print(f"  {beauty_print_array(angles_display, precision=2)}")
            beauty_print(f"夹爪: {state.gripper:.0f}")
            if state.velocities:
                beauty_print("速度 (rad/s):")
                print(f"  {beauty_print_array(state.velocities, precision=3)}")
            if state.torques:
                beauty_print("力矩 (N·m):")
                print(f"  {beauty_print_array(state.torques, precision=3)}")

        if continuous:
            try:
                while True:
                    _print_once()
                    print("---")
                    time.sleep(0.1)
            except KeyboardInterrupt:
                pass
        else:
            _print_once()

    # ========== 废弃方法 ==========

    def set_home(self, **kwargs):
        """已废弃，请使用 go_home()"""
        warnings.warn("set_home() 已废弃，请使用 go_home()", DeprecationWarning, stacklevel=2)
        return self.go_home(**kwargs)

    def set_pose_target(self, **kwargs):
        """已废弃，请使用 set_pose()"""
        warnings.warn("set_pose_target() 已废弃，请使用 set_pose()", DeprecationWarning, stacklevel=2)
        return self.set_pose(**kwargs)

    # ========== 内部方法 ==========

    def _auto_detect_aim(self, timeout: float) -> None:
        """自动检测控制目标（示教臂/操作臂）"""
        frame = self._codec.encode_version_request()
        resp = self._device.send_and_wait(frame, CMD_VERSION, timeout=timeout)
        info = self._device.version_info
        if info and info.device_type:
            dt = info.device_type.upper()
            if dt in ('L', 'LEADER'):
                self._device.set_aim(AIM_LEADER)
                logger.info("检测到示教臂 (Leader)")
            else:
                self._device.set_aim(AIM_FOLLOWER)
                logger.info("检测到操作臂 (Follower)")
        else:
            self._device.set_aim(AIM_FOLLOWER)
            logger.info("未检测到设备类型，默认操作臂")

    def _execute_cartesian_traj(self, traj: Dict) -> bool:
        """执行笛卡尔轨迹（逐点 IK → PV 轨迹执行）"""
        if self._robot_model is None:
            return False
        poses = traj.get('poses', [])
        if len(poses) == 0:
            return False

        state = self._device.joint_state
        q_current = list(state.angles) if state else None
        joint_positions = []

        for pose in poses:
            r = kin_module.compute_inverse_kinematics(
                self._robot_model, pose, q_init=q_current
            )
            if not r.get('success'):
                logger.warning("笛卡尔轨迹 IK 求解失败")
                return False
            q = r['q'].tolist()
            joint_positions.append(q + [state.gripper if state else 0.0])
            q_current = q

        positions = np.array(joint_positions)
        timestamps = traj['timestamps']
        # 数值微分计算速度
        dt = np.diff(timestamps)
        velocities = np.zeros_like(positions)
        velocities[1:] = np.diff(positions, axis=0) / dt[:, np.newaxis]

        return self._traj_executor.execute_pv(timestamps, positions, velocities)
