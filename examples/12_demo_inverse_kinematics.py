"""12_demo_inverse_kinematics.py — 逆运动学

演示逆运动学（IK）求解：
  1. 读取当前末端位姿 -> IK 求解 -> 打印关节角度
  2. 给定目标位姿 -> IK 求解 -> 可选执行运动
"""

import math
import time
import numpy as np
import alicia_m_sdk
from alicia_m_sdk import forward_kinematics, inverse_kinematics, RobotModel
from robocore.utils.beauty_logger import beauty_print, beauty_print_array
from robocore.transform import matrix_to_euler, matrix_to_quaternion
from robocore.transform.conversions import quaternion_to_matrix
from robocore.utils.backend import to_numpy


# 默认目标位姿: 位置 (m) + 四元数 (xyzw)
DEFAULT_TARGET_POSE = [+0.20, -0.0, +0.22, 0.0, 0.707, 0.0, 0.707]
# DEFAULT_TARGET_POSE = [0, 0.20, +0.22, 0.0, 0.707, 0.0, 0.707]


def main():
    beauty_print("Demo: 逆运动学 (IK)", type="module")

    # 创建并连接机器人
    robot = alicia_m_sdk.create_robot(control_mode="pv")
    beauty_print("机器人连接成功", type="success")

    try:
        robot_model = robot.robot_model

        # === 1. 读取当前位姿 -> IK -> 打印关节角度 ===
        beauty_print("1. 当前位姿的逆运动学验证", type="module")

        # 获取当前末端位姿
        pose_info = robot.get_pose()
        if pose_info is not None:
            position = pose_info['position']
            euler_xyz = pose_info['euler_xyz']
            quat_xyzw = pose_info['quaternion_xyzw']
            T_current = pose_info['transform']

            beauty_print(f"  当前位置 (m):     {beauty_print_array(position, precision=5)}", type="info")
            beauty_print(f"  当前欧拉角 (deg): {beauty_print_array(np.rad2deg(euler_xyz), precision=2)}", type="info")
            beauty_print(f"  当前四元数 (xyzw): {beauty_print_array(quat_xyzw, precision=6)}", type="info")

            # 以当前关节角度为初始猜测求解 IK
            q_current = robot.get_robot_state("joint")
            start_time = time.time()
            ik_result = inverse_kinematics(
                robot_model,
                T_current,
                q_current,
                method='dls',
                max_iters=1000,
                pos_tol=1e-2,
                ori_tol=1e-2,
                num_initial_guesses=5,
                use_analytic_jacobian=True,
            )
            elapsed = (time.time() - start_time) * 1000.0

            q_ik = to_numpy(ik_result['q'])
            q_ik_deg = np.rad2deg(q_ik)
            beauty_print(f"  IK 求解结果 (deg): {beauty_print_array(q_ik_deg, precision=2)}", type="info")
            beauty_print(f"  IK 求解结果 (rad): {beauty_print_array(q_ik, precision=4)}", type="info")
            beauty_print(f"  求解成功: {ik_result.get('success', False)}", type="info")
            beauty_print(f"  位置误差: {ik_result.get('pos_err', 'N/A')}", type="info")
            beauty_print(f"  姿态误差: {ik_result.get('ori_err', 'N/A')}", type="info")
            beauty_print(f"  计算耗时: {elapsed:.2f} ms", type="info")
        else:
            beauty_print("  无法获取当前位姿", type="warning")

        # === 2. 给定目标位姿 -> IK -> 可选执行 ===
        beauty_print("2. 给定目标位姿的逆运动学求解", type="module")

        target = DEFAULT_TARGET_POSE
        beauty_print(f"  目标位置 (m):     [{target[0]:.4f}, {target[1]:.4f}, {target[2]:.4f}]", type="info")
        beauty_print(f"  目标四元数 (xyzw): [{target[3]:.4f}, {target[4]:.4f}, {target[5]:.4f}, {target[6]:.4f}]", type="info")

        # 构建目标变换矩阵
        T_target = np.eye(4)
        T_target[:3, 3] = target[:3]
        T_target[:3, :3] = quaternion_to_matrix(target[3:])

        # 求解 IK
        start_time = time.time()
        ik_result = inverse_kinematics(
            robot_model,
            T_target,
            None,  # 无初始猜测，使用多起点
            method='dls',
            max_iters=500,
            pos_tol=1e-2,
            ori_tol=1e-2,
            num_initial_guesses=10,
            initial_guess_strategy='random',
            use_analytic_jacobian=True,
        )
        elapsed = (time.time() - start_time) * 1000.0

        q_ik = to_numpy(ik_result['q'])
        q_ik_deg = np.rad2deg(q_ik)
        ik_success = ik_result.get('success', False)
        if isinstance(ik_success, list):
            ik_success = ik_success[0] if ik_success else False

        beauty_print(f"  IK 求解结果 (deg): {beauty_print_array(q_ik_deg, precision=2)}", type="info")
        beauty_print(f"  IK 求解结果 (rad): {beauty_print_array(q_ik, precision=4)}", type="info")
        beauty_print(f"  求解成功: {ik_success}", type="info")
        beauty_print(f"  位置误差: {ik_result.get('pos_err', 'N/A')}", type="info")
        beauty_print(f"  姿态误差: {ik_result.get('ori_err', 'N/A')}", type="info")
        beauty_print(f"  计算耗时: {elapsed:.2f} ms", type="info")

        # 可选：执行运动到 IK 解
        if ik_success:
            beauty_print("IK 求解成功，是否执行运动?", type="info")
            beauty_print("  输入 y + Enter 执行，其他键跳过", type="info")
            try:
                user_input = input("  > ")
                if user_input.strip().lower() == 'y':
                    beauty_print("正在移动到目标位姿...", type="info")
                    robot.set_robot_state(
                        target_joints=q_ik.tolist(),
                        joint_format='rad',
                        speed=7,
                        wait_for_completion=True,
                    )
                    beauty_print("已到达目标位姿", type="success")
                else:
                    beauty_print("跳过运动执行", type="info")
            except EOFError:
                beauty_print("跳过运动执行", type="info")
        else:
            beauty_print("IK 求解失败，无法执行运动", type="warning")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
