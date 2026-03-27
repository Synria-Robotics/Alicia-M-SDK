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
Demo: Inverse kinematics

Features:
- Specify target end-effector pose
- Solve for joint angles
- Move to target position
"""

import argparse
import time

import numpy as np
import robocore as rc
from robocore.kinematics.ik import inverse_kinematics
from robocore.transform.conversions import quaternion_to_matrix
from robocore.utils.backend import to_numpy
from robocore.utils.beauty_logger import beauty_print, beauty_print_array

import alicia_m_sdk
from alicia_m_sdk.hardware import ServoDriver



def main(args):
    robot = alicia_m_sdk.create_robot(
        port=args.port,
        version=args.version,
        base_link=args.base_link,
        end_link=args.end_link,
        control_aim=args.control_aim,
        control_mode=args.control_mode
    )

    # Set backend
    rc.set_backend(args.backend)

    # Get robot model
    robot_model = robot.robot_model

    # Get initial guess
    if args.init_strategy == 'current':
        q0 = robot.get_robot_state("joint")
        if q0 is None:
            print("✗ 无法获取当前关节角度")
            robot.disconnect()
            return
        # Use 'random' strategy with current joints as base
        actual_strategy = 'random'
        print(f"当前关节角度 (弧度): {beauty_print_array(q0)}")
        print(f"当前关节角度 (角度): {beauty_print_array(np.rad2deg(q0))}")
    else:
        q0 = None
        actual_strategy = args.init_strategy

    # Convert pose to transformation matrix
    T_target = np.zeros((4, 4))
    T_target[:3, 3] = args.end_pose[:3]
    T_target[3, 3] = 1.0
    T_target[:3, :3] = quaternion_to_matrix(args.end_pose[3:])

    print(f"\n目标位姿:")
    print(f"  位置: [{args.end_pose[0]:.6f}, {args.end_pose[1]:.6f}, {args.end_pose[2]:.6f}] m")
    print(f"  四元数: [{args.end_pose[3]:.6f}, {args.end_pose[4]:.6f}, {args.end_pose[5]:.6f}, {args.end_pose[6]:.6f}]")

    if args.num_inits > 1:
        print(f"\n多起点尝试: {args.num_inits}, 策略: {actual_strategy}")
    elif args.init_strategy == 'current':
        print(f"\n使用当前关节角度作为初始猜测")

    # Solve inverse kinematics
    start_time = time.time()
    ik_result = inverse_kinematics(
        robot_model,
        T_target,
        q0,
        method=args.method,
        max_iters=args.max_iters,
        pos_tol=args.pos_tol,
        ori_tol=args.ori_tol,
        num_initial_guesses=args.num_inits,
        initial_guess_strategy=actual_strategy,
        initial_guess_scale=args.init_scale,
        random_seed=args.seed,
        use_analytic_jacobian=True
    )
    elapsed_time = time.time() - start_time

    # Convert to numpy
    q_ik = to_numpy(ik_result['q'])
    
    beauty_print("详细求解结果", type="module")
    print(f"  成功: {ik_result['success']}")

    # Extract error information (handle both single value and list)
    iters = ik_result.get('iters', 0)
    pos_err = ik_result.get('pos_err', float('inf'))
    ori_err = ik_result.get('ori_err', float('inf'))
    err_norm = ik_result.get('err_norm', None)

    # Handle list results (from batch processing)
    if isinstance(iters, list):
        iters = iters[0] if iters else 0
    if isinstance(pos_err, list):
        pos_err = pos_err[0] if pos_err else float('inf')
    if isinstance(ori_err, list):
        ori_err = ori_err[0] if ori_err else float('inf')
    if isinstance(err_norm, list):
        err_norm = err_norm[0] if err_norm else None

    # Check if IK succeeded
    ik_success = ik_result.get('success', False)
    if isinstance(ik_success, list):
        ik_success = ik_success[0] if ik_success else False

    if ik_success or args.force_execute:
        # Success case - show detailed results
        print(f"  迭代次数: {iters}")
        print(f"  位置误差: {pos_err:.6e} m")
        print(f"  姿态误差: {ori_err:.6e} rad")
        if err_norm is not None:
            print(f"  总误差: {err_norm:.6e}")
        print(f"  计算时间: {elapsed_time * 1000:.4f} ms")
        beauty_print("关节角度 (弧度):")
        print(f"  q_ik = {beauty_print_array(q_ik)}")
        beauty_print("关节角度 (角度):")
        print(f"  q_ik = {beauty_print_array(np.rad2deg(q_ik))}")

        # Execute motion if requested
        if args.execute or args.force_execute:
            print("\n执行移动到目标位置...")
            success = robot.set_robot_state(
                target_joints=q_ik,
                joint_format='rad',
                speed=args.speed,
                wait_for_completion=True,
                timeout=10
            )
            if success:
                beauty_print("✓ 机械臂已移动到目标位置")
            else:
                beauty_print("✗ 移动失败")
        else:
            beauty_print("(未执行移动)")
    else:
        # Failed case - show detailed error information
        print(f"  迭代次数: {iters}/{args.max_iters}")
        print(f"  位置误差: {pos_err:.6e} m (容差: {args.pos_tol:.6e} m)")
        print(f"  姿态误差: {ori_err:.6e} rad (容差: {args.ori_tol:.6e} rad)")
        if err_norm is not None:
            print(f"  总误差: {err_norm:.6e}")
        print(f"  计算时间: {elapsed_time * 1000:.4f} ms")
        beauty_print("关节角度 (弧度):")
        print(f"  q_ik = {beauty_print_array(q_ik)}")
        beauty_print("关节角度 (角度):")
        print(f"  q_ik = {beauty_print_array(np.rad2deg(q_ik))}")

        # Provide suggestions
        print("\n  可能的原因:")
        if iters >= args.max_iters:
            print("    - 达到最大迭代次数，可能目标位姿不可达或初始猜测不佳")
        if pos_err > args.pos_tol * 10:
            print(f"    - 位置误差较大 ({pos_err:.6e} m)，目标位置可能超出工作空间")
        if ori_err > args.ori_tol * 10:
            print(f"    - 姿态误差较大 ({ori_err:.6e} rad)，目标姿态可能不可达")

        print("\n  建议:")
        print("    - 尝试增加 --num-inits 参数（例如: --num-inits 10）")
        print("    - 尝试不同的 --init-strategy（例如: --init-strategy random）")
        print("    - 尝试增加 --max-iters 参数")
        print("    - 检查目标位姿是否在工作空间内")

        if 'message' in ik_result:
            print(f"\n  错误信息: {ik_result['message']}")
    print("=" * 60 + "\n")
    
    robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="逆向运动学示例")
    
    # Robot connection settings
    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--speed', type=int, default=7,  help="关节运动速度 (默认: 7，范围: 0-400)")
    parser.add_argument('--version', type=str, default="v1_1", help="机械臂版本 (可选: v1_0, v1_1，默认: v1_1)")
    parser.add_argument('--base_link', type=str, default="base_link", help="基座链路名称, world 或 base_link等")
    parser.add_argument('--end_link', type=str, default="link6", help="末端执行器链路名称, tool0 或 link6等")
    parser.add_argument('--control-aim', type=str, default='operation', choices=['teach', 'operation'],
                        help='Control aim: teach or operation (默认: operation)')
    parser.add_argument('--control-mode', type=str, default='pv',
                        choices=['pv', 'mit'],
                        help='Control mode (默认: pv)')

    # IK Configuration
    parser.add_argument('--end-pose', type=float, nargs=7, 
                        default=[0.1, 0., 0.42201, 0, 0.707,0, 0.707],
                       help='目标位姿 (7个浮点数: px py pz qx qy qz qw)')
    parser.add_argument('--method', type=str, default='dls', 
                       choices=['dls', 'pinv', 'transpose'],
                       help='IK方法: dls(阻尼最小二乘), pinv(伪逆), transpose(雅可比转置)')
    parser.add_argument('--max-iters', type=int, default=500,  help='最大迭代次数 (默认: 500)')
    parser.add_argument('--pos-tol', type=float, default=1e-3, help='位置容差 (默认: 1e-3)')
    parser.add_argument('--ori-tol', type=float, default=1e-3, help='姿态容差 (默认: 1e-3)')
    parser.add_argument('--num-inits', type=int, default=10,  help='初始猜测数量 (默认: 10)')
    parser.add_argument('--init-strategy', type=str, default='random',
                        choices=['zero', 'random', 'sobol', 'latin', 'center', 'uniform', 'current'],
                        help='初始猜测策略 (默认: current=使用当前关节角度)')
    parser.add_argument('--init-scale', type=float, default=1.0,
                        help='关节限制缩放因子 (0.0到1.0, 默认: 1.0)')
    parser.add_argument('--seed', type=int, default=None,
                        help='随机种子 (默认: None)')
    parser.add_argument('--backend', type=str, default='numpy',
                        choices=['numpy', 'torch'],
                        help='计算后端 (默认: numpy)')
    parser.add_argument('--execute', action='store_true', help='执行移动到求解的位置')
    parser.add_argument('--force-execute', action='store_true', help='强制执行移动到求解的位置，不考虑是否成功')
    
    args = parser.parse_args()
    
    main(args)
