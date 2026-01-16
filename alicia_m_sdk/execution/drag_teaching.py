import os
import json
import time
import threading
from typing import List, Dict, Any, Optional, Callable
from datetime import datetime
import numpy as np
import xml.etree.ElementTree as ET


# ==================== URDF解析与重力补偿计算 ====================

def get_rotation_matrix_from_rpy(roll, pitch, yaw):
    """根据 Roll, Pitch, Yaw 计算旋转矩阵 (XYZ 顺序)"""
    Rx = np.array([
        [1, 0, 0, 0],
        [0, np.cos(roll), -np.sin(roll), 0],
        [0, np.sin(roll), np.cos(roll), 0],
        [0, 0, 0, 1]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch), 0],
        [0, 1, 0, 0],
        [-np.sin(pitch), 0, np.cos(pitch), 0],
        [0, 0, 0, 1]
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0, 0],
        [np.sin(yaw), np.cos(yaw), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return Rz @ Ry @ Rx


def get_transform_matrix(xyz, rpy):
    """根据 xyz 和 rpy 获取 4x4 变换矩阵"""
    T = get_rotation_matrix_from_rpy(rpy[0], rpy[1], rpy[2])
    T[0, 3] = xyz[0]
    T[1, 3] = xyz[1]
    T[2, 3] = xyz[2]
    return T


def get_joint_transform(axis, angle, joint_type):
    """根据关节类型、轴和角度计算局部变换矩阵"""
    T = np.eye(4)
    ax, ay, az = axis
    c = np.cos(angle)
    s = np.sin(angle)
    
    if joint_type == 'revolute' or joint_type == 'continuous':
        length = np.sqrt(ax*ax + ay*ay + az*az)
        if length > 1e-6:
            ax, ay, az = ax/length, ay/length, az/length
        
        C = 1 - c
        R = np.array([
            [ax*ax*C + c,    ax*ay*C - az*s, ax*az*C + ay*s, 0],
            [ay*ax*C + az*s, ay*ay*C + c,    ay*az*C - ax*s, 0],
            [az*ax*C - ay*s, az*ay*C + ax*s, az*az*C + c,    0],
            [0,              0,              0,              1]
        ])
        return R
        
    elif joint_type == 'prismatic':
        T[0, 3] = ax * angle
        T[1, 3] = ay * angle
        T[2, 3] = az * angle
        return T
        
    return T


class RobotDynamicsModel:
    """机器人动力学模型 - 基于URDF的重力补偿计算"""
    
    def __init__(self, urdf_path: str):
        self.tree = ET.parse(urdf_path)
        self.root = self.tree.getroot()
        self.joints = {}
        self.links = {}
        self.root_link_name = None
        self.parse_urdf()
        self.precompute_fixed_transforms()
        
        # 目标关节列表 (对应6个关节电机)
        self.target_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 有效计算连杆 (包含夹爪部件)
        self.valid_links = {'base_link', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 
                           'grasp_base', 'link7', 'link8'}
    
    def parse_string_to_float(self, s):
        return [float(x) for x in s.split()]
    
    def parse_urdf(self):
        """解析URDF文件"""
        # 解析 Link
        for link in self.root.findall('link'):
            name = link.get('name')
            com_xyz = [0, 0, 0]
            mass = 0.0
            inertial = link.find('inertial')
            if inertial is not None:
                origin = inertial.find('origin')
                if origin is not None and origin.get('xyz'):
                    com_xyz = self.parse_string_to_float(origin.get('xyz'))
                mass_elem = inertial.find('mass')
                if mass_elem is not None:
                    mass = float(mass_elem.get('value'))
            self.links[name] = {'children': [], 'com': com_xyz, 'mass': mass}

        # 解析 Joint
        child_links = set()
        for joint in self.root.findall('joint'):
            name = joint.get('name')
            j_type = joint.get('type')
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            
            origin = joint.find('origin')
            xyz = [0, 0, 0]
            rpy = [0, 0, 0]
            if origin is not None:
                if origin.get('xyz'): xyz = self.parse_string_to_float(origin.get('xyz'))
                if origin.get('rpy'): rpy = self.parse_string_to_float(origin.get('rpy'))
            
            axis = [1, 0, 0]
            axis_elem = joint.find('axis')
            if axis_elem is not None:
                axis = self.parse_string_to_float(axis_elem.get('xyz'))

            self.joints[name] = {
                'type': j_type,
                'parent': parent,
                'child': child,
                'xyz': xyz,
                'rpy': rpy,
                'axis': axis
            }
            
            if parent in self.links:
                self.links[parent]['children'].append(name)
            child_links.add(child)

        all_links = set(self.links.keys())
        root_links = list(all_links - child_links)
        if root_links:
            self.root_link_name = root_links[0]
    
    def precompute_fixed_transforms(self):
        """预计算关节的原点变换矩阵，减少运行时计算量"""
        for name, joint in self.joints.items():
            joint['T_origin'] = get_transform_matrix(joint['xyz'], joint['rpy'])
            joint['axis_vec'] = np.array(joint['axis'] + [0])
    
    def compute_gravity_torques(self, joint_angles: List[float]) -> List[float]:
        """
        计算重力补偿扭矩
        :param joint_angles: 关节角度列表 [j1, j2, j3, j4, j5, j6] (弧度)
        :return: 扭矩列表 [t1, t2, t3, t4, t5, t6] (Nm)
        """
        # 构建角度字典
        angle_dict = {name: val for name, val in zip(self.target_joints, joint_angles)}
        
        # 1. 正运动学 (FK)
        link_transforms = {}
        joint_global_info = {}
        
        stack = [(self.root_link_name, np.eye(4))]
        
        while stack:
            curr_link, parent_T = stack.pop()
            link_transforms[curr_link] = parent_T
            
            if curr_link in self.links:
                for joint_name in self.links[curr_link]['children']:
                    joint = self.joints[joint_name]
                    child_link = joint['child']
                    
                    T_origin = joint['T_origin']
                    T_joint_frame_global = parent_T @ T_origin
                    
                    axis_global = T_joint_frame_global @ joint['axis_vec']
                    joint_global_info[joint_name] = {
                        'axis': axis_global[:3],
                        'origin': T_joint_frame_global[:3, 3]
                    }
                    
                    angle = angle_dict.get(joint_name, 0.0)
                    T_joint = get_joint_transform(joint['axis'], angle, joint['type'])
                    
                    child_T = parent_T @ T_origin @ T_joint
                    stack.append((child_link, child_T))

        # 2. 逆动力学 (RNE - Gravity only)
        torques = {}
        g_vec = np.array([0, 0, -9.81])
        
        def recursive_dynamics(link_name):
            link_data = self.links[link_name]
            mass = link_data['mass']
            
            T_link = link_transforms[link_name]
            com_local = np.array(link_data['com'] + [1])
            com_global = (T_link @ com_local)[:3]
            
            F_self = -(mass * g_vec)
            
            # Find parent joint info
            parent_joint_name = None
            for j_name, j_val in self.joints.items():
                if j_val['child'] == link_name:
                    parent_joint_name = j_name
                    break
            
            joint_origin = np.zeros(3)
            if parent_joint_name:
                joint_origin = joint_global_info[parent_joint_name]['origin']
                
            r_com = com_global - joint_origin
            M_self = np.cross(r_com, F_self)
            
            total_force = F_self
            total_moment = M_self
            
            for child_joint_name in link_data['children']:
                child_joint = self.joints[child_joint_name]
                child_link_name = child_joint['child']
                
                if child_link_name not in self.valid_links:
                    continue
                
                F_child, M_child = recursive_dynamics(child_link_name)
                
                child_joint_origin = joint_global_info[child_joint_name]['origin']
                r_child = child_joint_origin - joint_origin
                
                total_force += F_child
                total_moment += M_child + np.cross(r_child, F_child)
                
            if parent_joint_name:
                axis = joint_global_info[parent_joint_name]['axis']
                joint_type = self.joints[parent_joint_name]['type']
                
                torque = 0.0
                norm = np.linalg.norm(axis)
                if norm > 1e-6:
                    axis_normalized = axis / norm
                else:
                    axis_normalized = axis
                
                if joint_type in ['revolute', 'continuous']:
                    torque = np.dot(total_moment, axis_normalized)
                elif joint_type == 'prismatic':
                    torque = np.dot(total_force, axis_normalized)
                
                # 针对 joint6 的微调增益 (补偿线缆/摩擦/模型误差)
                if parent_joint_name == 'joint6':
                    torque *= 1.2

                torques[parent_joint_name] = torque
                
            return total_force, total_moment

        recursive_dynamics(self.root_link_name)
        
        # 返回有序列表
        return [torques.get(name, 0.0) for name in self.target_joints]


# ==================== 手动路径点记录函数 ====================

def record_waypoints_manual(controller, 
                            get_state_fn: Optional[Callable] = None,
                            format_fn: Optional[Callable] = None) -> List[Any]:
    """
    通用手动路径点记录函数 - 基于MIT扭矩模式的重力补偿
    
    注意：此函数不再简单关闭扭矩，而是使用重力补偿让机械臂呈现"零重力"状态
    
    Args:
        controller: 机器人控制器 (需要有servo_driver属性)
        get_state_fn: 自定义状态获取函数，返回要记录的数据。如果为None，使用默认的关节+夹爪
        format_fn: 自定义格式化函数，用于日志输出。如果为None，使用默认格式
        
    Returns:
        记录的路径点列表
    """
    print("\n=== 手动记录模式 (重力补偿) ===")
    print("系统将启用重力补偿，机械臂将呈现\"零重力\"状态")
    print("您可以自由拖动机械臂到目标位置后按回车记录")
    
    input("按回车开始...")
    
    # 注意：不再调用 torque_control('off')
    # 改为使用MIT扭矩模式进行重力补偿
    print("[提示] 请在调用此函数前确保已进入重力补偿模式")
    print("[提示] 或者使用 GravityCompensationTeaching 类")
    
    waypoints = []
    
    try:
        while True:
            cmd = input(f"\n拖动到位置后按回车记录第{len(waypoints) + 1}个点，输入'q'结束: ").strip()
            if cmd.lower() == 'q':
                break
            
            # 使用自定义或默认的状态获取函数
            if get_state_fn:
                state = get_state_fn(controller)
            else:
                # 默认：记录关节角度和夹爪状态
                joints = controller.get_joints()
                try:
                    gripper = float(controller.get_gripper())
                except:
                    gripper = 0.0
                state = {"t": time.time(), "q": joints, "grip": gripper}
            
            if state:
                waypoints.append(state)
                
                # 使用自定义或默认的格式化函数输出日志
                if format_fn:
                    print(format_fn(len(waypoints), state))
                else:
                    # 默认格式
                    if isinstance(state, dict) and 'q' in state:
                        print(f"[记录] 第{len(waypoints)}个点: 关节{[round(j, 3) for j in state['q']]}, 夹爪{state.get('grip', 0):.3f}")
                    else:
                        print(f"[记录] 第{len(waypoints)}个点")
                        
    finally:
        print("[完成] 手动记录结束")
        
    return waypoints


class SimpleDragTeaching:
    """简化的拖动示教类 - 直接记录关节状态"""
    
    def __init__(self, args, controller):
        self.args = args
        self.controller = controller

    def setup(self):
        """初始化设置"""
        if self.args.save_motion:
            print(f"动作名: {self.args.save_motion}")
        if self.args.mode == 'auto':
            print(f"采样频率: {self.args.sample_hz} Hz")
        print("=" * 30)
        
    def manual_mode(self) -> List[Dict[str, Any]]:
        """手动模式 - 记录关键点"""
        return record_waypoints_manual(self.controller)
        
    def auto_mode(self) -> List[Dict[str, Any]]:
        """自动模式 - 连续记录"""
        print("\n=== 自动模式 ===")
        print("关闭扭矩后拖动机械臂，系统自动记录轨迹")
        
        input("按回车开始...")
        self.controller.torque_control('off')
        print("[安全] 扭矩已关闭，可以拖动机械臂")
        
        # 记录变量
        trajectory = []
        recording = threading.Event()
        
        def record_loop():
            """后台记录线程"""
            dt = 1.0 / self.args.sample_hz
            start_time = time.time()
            
            while recording.is_set():
                try:
                    current_time = time.time() - start_time
                    joints = self.controller.get_joints()
                    try:
                        gripper = float(self.controller.get_gripper())
                    except:
                        gripper = 0.0
                        
                    point = {
                        "t": current_time,
                        "q": joints,
                        "grip": gripper
                    }
                    trajectory.append(point)
                    
                except Exception as e:
                    print(f"[警告] 记录失败: {e}")
                    
                time.sleep(dt)
        
        try:
            input("开始拖动，按回车开始记录...")
            recording.set()
            thread = threading.Thread(target=record_loop, daemon=True)
            thread.start()
            print(f"[记录中] 频率 {self.args.sample_hz} Hz...")
            
            input("按回车停止记录...")
            recording.clear()
            thread.join(timeout=1.0)
            
        finally:
            self.controller.torque_control('on')
            print("[安全] 扭矩已重新开启")
            
        print(f"[完成] 记录了 {len(trajectory)} 个点")
        return trajectory
    
    def replay_only_mode(self) -> Optional[List[Dict[str, Any]]]:
        """仅回放模式 - 加载已有数据"""
        print("\n=== 仅回放模式 ===")
        
        # 检查动作名是否提供
        if not self.args.save_motion:
            print("[错误] 回放模式需要指定动作名")
            print("请使用 --save-motion <motion_name> 参数")
            print("或使用 --list-motions 查看可用动作")
            return None
        
        # 加载数据
        save_dir = os.path.join("example_motions", self.args.save_motion)
        traj_path = os.path.join(save_dir, "joint_traj.json")
        meta_path = os.path.join(save_dir, "meta.json")
        
        # 检查目录和文件是否存在
        if not os.path.exists(save_dir):
            print(f"[错误] 未找到动作目录: {save_dir}")
            print("\n可用的动作:")
            available = list_available_motions()
            if available:
                for motion in available[:5]:  # 只显示前5个
                    print(f"  - {motion}")
                if len(available) > 5:
                    print(f"  ... 还有 {len(available)-5} 个动作")
                print(f"\n使用 --list-motions 查看完整列表")
            else:
                print("  未找到任何已录制的动作")
                print("  请先使用 auto 或 manual 模式录制动作")
            return None
            
        if not os.path.exists(traj_path):
            print(f"[错误] 未找到轨迹文件: {traj_path}")
            print("该动作可能损坏或不完整")
            return None
            
        if not os.path.exists(meta_path):
            print(f"[警告] 未找到元数据文件: {meta_path}")
            print("将使用默认设置继续加载")
        
        # 读取数据和元信息
        try:
            with open(traj_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            print(f"[错误] 读取轨迹文件失败: {e}")
            return None
            
        # 读取元信息
        meta = {}
        if os.path.exists(meta_path):
            try:
                with open(meta_path, 'r', encoding='utf-8') as f:
                    meta = json.load(f)
            except Exception as e:
                print(f"[警告] 读取元数据失败: {e}")
        
        # 更新模式为原始记录模式
        original_mode = meta.get('mode', 'auto')
        print(f"[加载] 轨迹: {len(data)}个点")
        print(f"[加载] 原始模式: {original_mode}")
        print(f"[加载] 创建时间: {meta.get('created_at', 'Unknown')}")
        print(f"[加载] 采样频率: {meta.get('sample_hz', 'Unknown')} Hz")
        
        # 临时保存原始参数并设置为原始模式
        self._original_mode = self.args.mode
        self.args.mode = original_mode
        
        return data
        
    def save_data(self, data: List[Dict[str, Any]]) -> Optional[str]:
        """保存数据"""
        if not data:
            print("[保存] 没有数据")
            return None
            
        # 创建保存目录
        save_dir = os.path.join("example_motions", self.args.save_motion)
        os.makedirs(save_dir, exist_ok=True)
        
        # 保存关节轨迹
        traj_path = os.path.join(save_dir, "joint_traj.json")
        with open(traj_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
            
        # 保存元信息
        meta = {
            "motion": self.args.save_motion,
            "mode": self.args.mode,  # 保存记录时的模式
            "created_at": datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            "sample_hz": self.args.sample_hz,
            "count": len(data),
            "description": "拖动示教轨迹"
        }
        
        meta_path = os.path.join(save_dir, "meta.json")
        with open(meta_path, 'w', encoding='utf-8') as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
            
        print(f"[保存] 轨迹: {traj_path}")
        print(f"[保存] 元数据: {meta_path}")
        
        return save_dir
        
    def replay(self, data: List[Dict[str, Any]]):
        """根据模式回放轨迹"""
        if not data:
            return
            
        print(f"\n=== 轨迹回放 ===")
        print(f"回放模式: {self.args.mode}")
        replay = input(f"是否回放轨迹（{len(data)}个点）？(y/n): ").strip().lower()
        if replay != 'y':
            return
            
        print("[回放] 开始...")
        
        if self.args.mode == 'auto':
            # 自动模式：使用直接设置，快速回放
            print("[回放] 使用直接设置模式（快速）")
            for i, point in enumerate(data):
                try:
                    # 获取夹爪值
                    gripper_value = point.get("grip", 0.0)
                    
                    # 使用combined control直接设置关节和夹爪，无插值
                    # speed_rad_s: 30 rad/s ≈ 0.524 rad/s
                    self.controller.servo_driver.set_joint_and_gripper(
                        joint_angles=point["q"],
                        gripper_value=gripper_value,
                        speed_rad_s=0.524  # 高速回放 (约30 rad/s)
                    )
                    
                    print(f"[回放] {i+1}/{len(data)}")
                    time.sleep(0.02)  # 20ms延时，快速回放
                    
                except Exception as e:
                    print(f"[错误] 回放第{i+1}点失败: {e}")
                    
        elif self.args.mode == 'manual':
            # 手动模式：使用插值运动，平滑回放
            print("[回放] 使用插值运动模式（平滑）")
            for i, point in enumerate(data):
                try:
                    firmware_new = self.controller.firmware_new
                    if firmware_new:
                        self.controller.set_joint_target(point["q"])
                    else:
                        self.controller.set_joint_target_interplotation(point["q"], joint_format='rad', speed_factor=1.0, T_default=0.5, n_steps_ref=50)
                    
                    # 设置夹爪
                    gripper_value = point.get("grip", 0.0)
                    if gripper_value is not None:
                        try:
                            # 夹爪值已经是0-100范围，直接使用
                            self.controller.set_gripper_target(value=gripper_value, wait_for_completion=False)
                        except:
                            pass
                    
                    print(f"[回放] {i+1}/{len(data)}")
                    
                except Exception as e:
                    print(f"[错误] 回放第{i+1}点失败: {e}")
        
        print("[回放] 完成")
    
    
    def run(self):
        """运行主程序"""
        try:
            # 对于replay_only模式，在setup之前先验证动作是否存在
            if self.args.mode == 'replay_only':
                if not self.args.save_motion:
                    print("[错误] 回放模式必须指定 --save-motion 参数")
                    print("使用 --list-motions 查看可用动作")
                    return
                
                # 检查动作是否存在
                save_dir = os.path.join("example_motions", self.args.save_motion)
                if not os.path.exists(save_dir):
                    print(f"[错误] 动作 '{self.args.save_motion}' 不存在")
                    print("\n提示:")
                    available = list_available_motions()
                    if available:
                        print("可用的动作:")
                        for motion in available[:3]:
                            print(f"  {motion}")
                        if len(available) > 3:
                            print(f"  ... 还有 {len(available)-3} 个")
                    else:
                        print("未找到任何已录制的动作，请先录制")
                    return
            
            self.setup()
            
            # 根据模式执行不同操作
            if self.args.mode == 'manual':
                data = self.manual_mode()
            elif self.args.mode == 'auto':
                data = self.auto_mode()
            elif self.args.mode == 'replay_only':
                data = self.replay_only_mode()
            else:
                raise ValueError(f"不支持的模式: {self.args.mode}")
                
            if data:
                # 只有非replay_only模式才保存数据
                if self.args.mode != 'replay_only':
                    save_dir = self.save_data(data)
                    
                    if save_dir:
                        print(f"\n[完成] 拖动示教完成！")
                        print(f"数据已保存到: {save_dir}")
                else:
                    print(f"\n[完成] 加载轨迹完成！")
                    
                # 回放
                self.replay(data)
                
                # 恢复原始模式（对于replay_only）
                if hasattr(self, '_original_mode'):
                    self.args.mode = self._original_mode
                    
            else:
                print("[完成] 未记录数据或加载失败")
                
        except KeyboardInterrupt:
            print("\n[中断] 用户中断")
        except Exception as e:
            print(f"[错误] 运行失败: {e}")




def list_available_motions() -> List[str]:
    """列出所有可用的动作"""
    motions_dir = "example_motions"
    if not os.path.exists(motions_dir):
        return []
    
    motions = []
    for item in os.listdir(motions_dir):
        motion_path = os.path.join(motions_dir, item)
        if os.path.isdir(motion_path):
            # 检查是否有轨迹数据
            traj_file = os.path.join(motion_path, "joint_traj.json")
            if os.path.exists(traj_file):
                motions.append(item)
    
    return sorted(motions)


def print_available_motions():
    """打印所有可用的动作"""
    motions = list_available_motions()
    
    print("=== 可用的动作列表 ===")
    if not motions:
        print("未找到任何已录制的动作")
        print("请先使用 auto 或 manual 模式录制动作")
        return
    
    print(f"在 example_motions/ 目录下找到 {len(motions)} 个动作:")
    
    for i, motion in enumerate(motions, 1):
        motion_dir = os.path.join("example_motions", motion)
        meta_path = os.path.join(motion_dir, "meta.json")
        
        # 读取动作信息
        info = f"{i:2d}. {motion}"
        if os.path.exists(meta_path):
            try:
                with open(meta_path, 'r', encoding='utf-8') as f:
                    meta = json.load(f)
                mode = meta.get('mode', 'unknown')
                created = meta.get('created_at', 'unknown')
                count = meta.get('count', 0)
                info += f" (模式: {mode}, 点数: {count}, 创建: {created})"
            except:
                pass
        
        print(info)
    
    print(f"\n使用示例:")
    print(f"python 07_demo_drag_teaching.py --port /dev/ttyUSB0 --mode replay_only --save-motion {motions[0]}")


# ==================== 基于MIT扭矩模式的重力补偿示教类 ====================

class GravityCompensationTeaching:
    """
    基于MIT扭矩模式的重力补偿拖动示教类
    
    功能:
    1. 使用URDF模型实时计算重力补偿扭矩
    2. 通过MIT扭矩模式发送补偿扭矩，实现\"零重力\"拖动
    3. 以200Hz频率记录关节角度
    4. 保存轨迹供后续回放
    
    使用:
        teaching = GravityCompensationTeaching(
            controller=robot,
            urdf_path="path/to/urdf",
            sample_hz=200
        )
        teaching.start_gravity_compensation()  # 启动重力补偿
        trajectory = teaching.record_trajectory(duration=10.0)  # 录制10秒
        teaching.stop_gravity_compensation()  # 停止补偿
        teaching.save_trajectory(trajectory, "motion_name")
    """
    
    def __init__(self, controller, urdf_path: str, sample_hz: float = 200.0, 
                 torque_scale: float = 1.0, debug: bool = False):
        """
        :param controller: Robot API实例 (需要有servo_driver属性)
        :param urdf_path: URDF文件路径
        :param sample_hz: 采样频率 (Hz)，建议200Hz
        :param torque_scale: 扭矩缩放系数，用于微调补偿强度
        :param debug: 是否启用调试模式
        """
        self.controller = controller
        self.servo_driver = controller.servo_driver
        self.sample_hz = sample_hz
        self.torque_scale = torque_scale
        self.debug = debug
        
        # 加载动力学模型
        self.dynamics_model = RobotDynamicsModel(urdf_path)
        
        # 线程控制
        self._compensation_running = False
        self._compensation_thread = None
        self._stop_event = threading.Event()
        
        print(f"[初始化] 重力补偿示教系统")
        print(f"  - URDF: {urdf_path}")
        print(f"  - 采样频率: {sample_hz} Hz")
        print(f"  - 扭矩缩放: {torque_scale}")
    
    def start_gravity_compensation(self) -> bool:
        """
        启动重力补偿线程
        
        :return: 是否成功启动
        """
        if self._compensation_running:
            print("[警告] 重力补偿已经在运行")
            return True
        
        # 初始化MIT扭矩模式
        print("[初始化] 切换到MIT扭矩模式...")
        if not self.servo_driver.initialize_mit_mode(repeat_times=3):
            print("[错误] MIT模式初始化失败")
            return False
        
        # 启动后台补偿线程
        self._stop_event.clear()
        self._compensation_running = True
        self._compensation_thread = threading.Thread(
            target=self._gravity_compensation_loop, 
            daemon=True
        )
        self._compensation_thread.start()
        
        print("[启动] 重力补偿已启动，机械臂现在处于\"零重力\"状态")
        print("[提示] 您现在可以自由拖动机械臂")
        
        return True
    
    def stop_gravity_compensation(self):
        """停止重力补偿线程"""
        if not self._compensation_running:
            return
        
        print("[停止] 正在停止重力补偿...")
        self._stop_event.set()
        self._compensation_running = False
        
        if self._compensation_thread and self._compensation_thread.is_alive():
            self._compensation_thread.join(timeout=2.0)
        
        # 发送零扭矩
        zero_torques = [0.0] * 6
        self.servo_driver._send_joint_frame_internal(
            joint_angles=None,
            gripper_value=None,
            speed_deg_s=0.0,
            torque_nm=zero_torques,
            control_aim=self.servo_driver.AIM_OPERATION,
            control_mode=self.servo_driver.PATTERN_MIT_TORQUE
        )
        
        print("[完成] 重力补偿已停止")
    
    def _gravity_compensation_loop(self):
        """重力补偿后台线程"""
        dt = 1.0 / self.sample_hz
        
        while not self._stop_event.is_set():
            start_time = time.time()
            
            try:
                # 1. 获取当前关节角度
                current_joints = self.controller.get_joints()
                if current_joints is None or len(current_joints) < 6:
                    time.sleep(dt)
                    continue
                
                # 2. 计算重力补偿扭矩
                gravity_torques = self.dynamics_model.compute_gravity_torques(current_joints)
                
                # 3. 应用扭矩缩放
                scaled_torques = [t * self.torque_scale for t in gravity_torques]
                
                # 4. 发送MIT扭矩命令
                self.servo_driver._send_joint_frame_internal(
                    joint_angles=None,
                    gripper_value=None,
                    speed_deg_s=0.0,
                    torque_nm=scaled_torques,
                    control_aim=self.servo_driver.AIM_OPERATION,
                    control_mode=self.servo_driver.PATTERN_MIT_TORQUE
                )
                
                if self.debug:
                    print(f"[补偿] 扭矩: {[f'{t:.3f}' for t in scaled_torques]}")
                
            except Exception as e:
                print(f"[错误] 重力补偿失败: {e}")
            
            # 控制循环频率
            elapsed = time.time() - start_time
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def record_trajectory(self, duration: Optional[float] = None, 
                         manual_stop: bool = True) -> List[Dict[str, Any]]:
        """
        录制轨迹
        
        :param duration: 录制时长(秒)，None表示手动停止
        :param manual_stop: 是否等待用户按回车停止
        :return: 轨迹数据列表
        """
        if not self._compensation_running:
            print("[错误] 请先启动重力补偿 (start_gravity_compensation)")
            return []
        
        print("\n=== 开始录制轨迹 ===")
        if duration:
            print(f"录制时长: {duration}秒")
        else:
            print("手动录制，按回车停止")
        
        trajectory = []
        recording = threading.Event()
        recording.set()
        
        def record_loop():
            """录制线程"""
            dt = 1.0 / self.sample_hz
            start_time = time.time()
            
            while recording.is_set():
                current_time = time.time() - start_time
                
                # 检查时长限制
                if duration and current_time >= duration:
                    break
                
                try:
                    joints = self.controller.get_joints()
                    if joints is None:
                        continue
                    
                    try:
                        gripper = float(self.controller.get_gripper())
                    except:
                        gripper = 0.0
                    
                    point = {
                        "t": current_time,
                        "q": joints,
                        "grip": gripper
                    }
                    trajectory.append(point)
                    
                except Exception as e:
                    if self.debug:
                        print(f"[警告] 记录失败: {e}")
                
                time.sleep(dt)
        
        # 启动录制线程
        record_thread = threading.Thread(target=record_loop, daemon=True)
        record_thread.start()
        
        print(f"[录制中] 采样频率 {self.sample_hz} Hz...")
        
        if manual_stop:
            input("拖动机械臂，按回车停止录制...")
            recording.clear()
        
        record_thread.join(timeout=2.0)
        
        print(f"[完成] 录制了 {len(trajectory)} 个点 ({len(trajectory)/self.sample_hz:.2f}秒)")
        return trajectory
    
    def save_trajectory(self, trajectory: List[Dict[str, Any]], 
                       motion_name: str) -> Optional[str]:
        """
        保存轨迹到文件
        
        :param trajectory: 轨迹数据
        :param motion_name: 动作名称
        :return: 保存路径，失败返回None
        """
        if not trajectory:
            print("[错误] 轨迹为空，无法保存")
            return None
        
        # 创建保存目录
        save_dir = os.path.join("example_motions", motion_name)
        os.makedirs(save_dir, exist_ok=True)
        
        # 保存轨迹数据
        traj_path = os.path.join(save_dir, "joint_traj.json")
        with open(traj_path, 'w', encoding='utf-8') as f:
            json.dump(trajectory, f, ensure_ascii=False, indent=2)
        
        # 保存元信息
        meta = {
            "motion": motion_name,
            "mode": "gravity_compensation",
            "created_at": datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            "sample_hz": self.sample_hz,
            "count": len(trajectory),
            "duration": len(trajectory) / self.sample_hz,
            "torque_scale": self.torque_scale,
            "description": "重力补偿拖动示教轨迹"
        }
        
        meta_path = os.path.join(save_dir, "meta.json")
        with open(meta_path, 'w', encoding='utf-8') as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        
        print(f"[保存] 轨迹文件: {traj_path}")
        print(f"[保存] 元数据: {meta_path}")
        print(f"[信息] 点数: {len(trajectory)}, 时长: {meta['duration']:.2f}秒")
        
        return save_dir
    
    def run_interactive(self, motion_name: str):
        """
        交互式录制流程
        
        :param motion_name: 动作名称
        """
        try:
            print("\n" + "="*50)
            print("  重力补偿拖动示教系统")
            print("="*50)
            print(f"动作名称: {motion_name}")
            print(f"采样频率: {self.sample_hz} Hz")
            print()
            
            # 1. 启动重力补偿
            if not self.start_gravity_compensation():
                print("[错误] 无法启动重力补偿")
                return
            
            time.sleep(0.5)  # 等待系统稳定
            
            # 2. 录制轨迹
            trajectory = self.record_trajectory(duration=None, manual_stop=True)
            
            if not trajectory:
                print("[错误] 未录制任何数据")
                return
            
            # 3. 保存轨迹
            save_dir = self.save_trajectory(trajectory, motion_name)
            
            if save_dir:
                print(f"\n[完成] 拖动示教完成！")
                print(f"数据已保存到: {save_dir}")
            
        except KeyboardInterrupt:
            print("\n[中断] 用户中断")
        except Exception as e:
            print(f"[错误] 运行失败: {e}")
            if self.debug:
                import traceback
                traceback.print_exc()
        finally:
            # 确保停止重力补偿
            self.stop_gravity_compensation()

