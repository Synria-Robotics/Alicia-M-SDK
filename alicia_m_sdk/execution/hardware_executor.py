import time
from typing import List, Optional
import numpy as np

from alicia_m_sdk.hardware import ServoDriver
from alicia_m_sdk.utils.logger import logger
from alicia_m_sdk.utils.vislab import plot_joint_angles
from .drag_teaching import record_waypoints_manual

class HardwareExecutor:
    """
    硬件轨迹执行器 - 支持MIT位置模式的高频轨迹回放
    
    功能:
    1. 执行关节空间轨迹（支持PV模式和MIT位置模式）
    2. MIT位置模式用于高频轨迹回放（200Hz+）
    3. 支持轨迹可视化和交互确认
    """
    
    def __init__(self, joint_controller: ServoDriver):
        self.joint_controller = joint_controller
        self.delay = 0.002  # 默认延迟2ms (500Hz)

    def execute(self, 
                joint_traj: List[List[float]], 
                visualize: bool = False,
                gripper_traj: List[float] = None,
                interaction: bool = False,
                speed: float = 4.0,
                use_mit_mode: bool = False,
                playback_hz: float = 50.0,
                ):
        """
        执行关节轨迹
        
        :param joint_traj: 关节轨迹，列表的列表 [[j1,j2,j3,j4,j5,j6], ...]
        :param visualize: 是否在执行前绘图
        :param gripper_traj: 夹爪轨迹（可选），与关节轨迹对齐
        :param interaction: 是否等待用户确认
        :param speed: PV模式下的关节速度 (deg/s)
        :param use_mit_mode: 是否使用MIT位置模式（用于高频回放）
        :param playback_hz: MIT模式下的回放频率 (Hz)
        :return: True if executed, False if cancelled
        """
        traj_np = np.array(joint_traj)

        if visualize:
            plot_joint_angles(traj_np)

        if interaction:
            logger.module("[executor]按下回车执行轨迹，按下 q 取消：")
            usr_input = input()

            if usr_input.lower() == 'q':
                logger.info("[executor]取消执行轨迹")
                return False
        
        if use_mit_mode:
            # 使用MIT位置模式回放（高频）
            return self._execute_mit_position_mode(
                joint_traj, 
                gripper_traj, 
                playback_hz
            )
        else:
            # 使用PV模式回放（低频，带插值）
            return self._execute_pv_mode(
                joint_traj, 
                gripper_traj, 
                speed
            )
    
    def _execute_pv_mode(self,
                        joint_traj: List[List[float]],
                        gripper_traj: Optional[List[float]],
                        speed: float) -> bool:
        """
        使用PV模式执行轨迹（低频，带速度控制）

        :param joint_traj: 关节轨迹
        :param gripper_traj: 夹爪轨迹
        :param speed: 关节速度 (deg/s)
        :return: True if executed successfully
        """
        logger.info(f"[PV模式] 执行轨迹，共 {len(joint_traj)} 个点")

        for idx, point in enumerate(joint_traj):
            # Select corresponding gripper value (if provided)
            g = None
            if gripper_traj is not None and idx < len(gripper_traj):
                g = gripper_traj[idx]

            # Use combined joint + gripper command
            self.joint_controller.set_joint_and_gripper(
                joint_angles=point,
                gripper_value=g,
                speed=speed,
                control_aim=self.joint_controller.default_control_aim,
            )
            time.sleep(self.delay)

        logger.info("[PV模式] 轨迹执行完成")
        return True
    
    def _execute_mit_position_mode(self, 
                                   joint_traj: List[List[float]], 
                                   gripper_traj: Optional[List[float]], 
                                   playback_hz: float) -> bool:
        """
        使用MIT位置模式执行轨迹（高频回放）
        
        此模式用于回放通过重力补偿录制的高频轨迹（如200Hz采样率）
        MIT位置模式每个电机只发送位置数据（2字节），通信效率高
        
        :param joint_traj: 关节轨迹
        :param gripper_traj: 夹爪轨迹
        :param playback_hz: 回放频率 (Hz)
        :return: True if executed successfully
        """
        logger.info(f"[MIT位置模式] 执行轨迹，共 {len(joint_traj)} 个点")
        logger.info(f"[MIT位置模式] 回放频率: {playback_hz} Hz")
        
        # 初始化MIT模式（如果尚未初始化则执行，已初始化则跳过）
        if not self.joint_controller._mit_mode_initialized:
            if not self.joint_controller.initialize_mit_mode(repeat_times=3):
                logger.error("[MIT位置模式] 初始化失败")
                return False
        
        dt = 1.0 / playback_hz
        
        for idx, point in enumerate(joint_traj):
            start_time = time.time()
            
            # 夹爪值
            g = None
            if gripper_traj is not None and idx < len(gripper_traj):
                g = gripper_traj[idx]
            
            # 使用MIT位置模式发送
            self.joint_controller.set_joint_and_gripper(
                joint_angles=point,
                gripper_value=g,
                speed=0.0,  # MIT位置模式不使用速度
                torque_nm=0.0,    # MIT位置模式不使用扭矩
                control_aim=self.joint_controller.default_control_aim,
                control_mode=self.joint_controller.PATTERN_MIT
            )
            
            # 控制回放频率
            elapsed = time.time() - start_time
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # 周期性打印进度
            if idx % int(playback_hz) == 0:  # 每秒打印一次
                progress = (idx + 1) / len(joint_traj) * 100
                logger.info(f"[MIT位置模式] 进度: {progress:.1f}% ({idx+1}/{len(joint_traj)})")
        
        logger.info("[MIT位置模式] 轨迹执行完成")
        return True
        


class CartesianWaypointPlanner:
    """Cartesian waypoint planner for recording and executing paths"""
    
    def __init__(self, robot):
        """
        :param robot: Robot API instance
        """
        self.robot = robot
    
    def get_current_waypoint(self) -> Optional[List[float]]:
        """
        :return: [x, y, z, qx, qy, qz, qw, gripper] or None
        """
        pose = self.robot.get_pose()
        if pose is None:
            return None
        
        pos = pose['position'].tolist()
        quat = pose['quaternion_xyzw'].tolist()
        gripper = self.robot.get_robot_state("gripper") or 0.0
        
        return pos + quat + [gripper]
    
    def record_teaching_waypoints(self) -> List[List[float]]:
        """
        :return: Waypoints as [x, y, z, qx, qy, qz, qw, gripper]
        """
        logger.info("=== 教学模式：手动记录路径点 ===")
        
        # 定义状态获取函数（获取笛卡尔位姿）
        def get_cartesian_state(_):
            return self.get_current_waypoint()
        
        # 定义日志格式化函数
        def format_waypoint(count, waypoint):
            if waypoint and len(waypoint) >= 8:
                return (f"[记录] 路径点 {count}: "
                       f"位置={[round(p, 4) for p in waypoint[:3]]}, "
                       f"夹爪={waypoint[7]:.3f}")
            return f"[记录] 路径点 {count}"
        
        # 使用共享函数记录路径点
        waypoints = record_waypoints_manual(
            controller=self.robot,
            get_state_fn=get_cartesian_state,
            format_fn=format_waypoint
        )
        
        return waypoints
    
    
    def execute_trajectory(self,
                          waypoints: List[List[float]],
                          move_duration: float = 3.0,
                          num_points: int = 150,
                          ik_method: str = 'dls',
                          visualize: bool = False,
                          step_by_step: bool = False,
                          step_delay: float = 0.2):
        """
        :param waypoints: Waypoints as [x, y, z, qx, qy, qz, qw, gripper]
        :param move_duration: Movement time per waypoint in seconds
        :param num_points: Interpolation points per segment
        :param ik_method: IK method
        :param visualize: Whether to visualize
        :param step_by_step: Whether to execute step by step
        :param step_delay: Delay between steps in seconds
        """
        if not waypoints:
            logger.error("没有路径点可执行")
            return
        
        logger.info("\n=== 执行笛卡尔轨迹 ===")
        logger.info(f"共 {len(waypoints)} 个路径点")
        logger.info(f"{'逐步' if step_by_step else '连续'}执行模式...")
        
        for i, waypoint in enumerate(waypoints):
            logger.info(f"执行路径点 {i+1}/{len(waypoints)}...")
            
            # 分离位姿和夹爪
            pose = waypoint[:7]  # [x, y, z, qx, qy, qz, qw]
            gripper = waypoint[7] if len(waypoint) > 7 else 0.0
            
            # 执行笛卡尔运动
            self.robot.move_cartesian_linear(
                target_pose=pose,
                duration=move_duration,
                num_points=num_points,
                ik_method=ik_method,
                visualize=visualize
            )
            
            # 设置夹爪（使用统一接口，仅控制夹爪）
            self.robot.set_robot_state(gripper_value=gripper, wait_for_completion=False)
            time.sleep(step_delay)
            
            # 逐步执行模式下等待用户确认
            if step_by_step and i < len(waypoints) - 1:
                input("按 Enter 继续下一个路径点...")
        
        logger.info("✓ 轨迹执行完成!")

        # ...existing code...
class JointInterpolator:
    """
    简单关节轨迹插值器（占位实现）。
    提供一个 plan(start_angles, target_angles, steps) 方法，返回插值后的关节列表。
    """
    def __init__(self):
        pass

    def plan(self, start_angles: List[float], target_angles: List[float], steps: int) -> List[List[float]]:
        """
        Plan a linear trajectory between start and target angles.
        
        :param start_angles: Starting joint angles
        :param target_angles: Target joint angles
        :param steps: Number of interpolation steps
        :return: List of joint angle lists
        """
        import numpy as _np
        
        if steps < 2:
            return [start_angles, target_angles]
            
        start = _np.array(start_angles)
        end = _np.array(target_angles)
        
        # Generate linear interpolation
        # linspace returns shape (steps, n_joints)
        traj = _np.linspace(start, end, steps)
        
        return traj.tolist()

    def interpolate(self, joint_traj, num_points_per_segment: int = 50):
        """
        joint_traj: List[List[float]] 每个点为关节角列表
        num_points_per_segment: 每段插值点数（包含起点不含终点）
        返回: List[List[float]] 插值后的关节轨迹
        """
        import numpy as _np

        if not joint_traj:
            return []

        joint_traj_np = _np.asarray(joint_traj, dtype=_np.float64)
        n_points, n_joints = joint_traj_np.shape
        if n_points < 2:
            return joint_traj

        result = []
        for i in range(n_points - 1):
            a = joint_traj_np[i]
            b = joint_traj_np[i + 1]
            # num_points_per_segment 包含起点，不包含终点
            for t in _np.linspace(0.0, 1.0, num_points_per_segment, endpoint=False):
                result.append(((1 - t) * a + t * b).tolist())
        # 添加最后一个点
        result.append(joint_traj_np[-1].tolist())
        return result
# ...existing code...