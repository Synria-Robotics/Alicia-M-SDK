#!/usr/bin/env python3
"""@file sop_void_grip_test.py
@brief Alicia-M 虚空识别与虚空夹 SOP 安全测试命令行程序。
@details
本脚本用于复现客户给定时序表中的末端移动、末端旋转与夹爪开合动作。
识别步骤为“虚空识别”：只等待节拍时间，不访问相机、不读取图像、不产生视觉结果。
夹爪步骤为“虚空夹”：只执行夹爪开合动作，不要求工作空间内存在物品，也不检测夹持力。

安全策略采用“先验证、再执行、失败即停”：
- 末端笛卡尔移动按固定长度拆分为多个小段；
- 每个中间目标先调用 IK 预检，确认模型可达、残差可接受、关节不越限；
- 每段执行后读取设备状态，若出现电机错误或状态异常立即终止；
- 任何步骤失败都会尝试失能机械臂并断开串口。

注意：IK 可达不等同于碰撞安全。本脚本不做环境碰撞检测、不感知桌面、
工装、线缆或人员位置；真实运行前必须清空工作空间并人工确认安全。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Iterable, Optional

import numpy as np

import alicia_m_sdk


PASS = "PASS"
FAIL = "FAIL"

DEFAULT_STEP_MM = 50.0
DEFAULT_SPEED = 10.0
DEFAULT_GRIPPER_SPEED = 80.0
DEFAULT_ROTATE_DEG = 90.0
DEFAULT_IK_RESIDUAL_TOL = 0.05
DEFAULT_LOG_DIR = Path(__file__).resolve().parent / "log"
DEFAULT_INITIAL_JOINTS_DEG = [0.0, -90.0, -45.0, 0.0, 45.0, 0.0]
DEFAULT_INITIAL_SPEED = 5.0

# Alicia_M_v1_1_follower.urdf 中 6 个旋转关节的限位，单位 rad。
# 这里保留为脚本内的通用安全边界，不绑定任何本机绝对 URDF 路径。
DEFAULT_JOINT_LIMITS_RAD: tuple[tuple[float, float], ...] = (
    (-2.7475, 2.7475),
    (-3.14, 0.0),
    (-3.14, 0.0),
    (-1.57, 1.57),
    (-1.57, 1.57),
    (-2.791, 2.791),
)


@dataclass(frozen=True)
class SopAction:
    """@brief 单个 SOP 动作定义。
    @details
    SOP 时序表会被展开为一组不可变动作。每个动作只描述“做什么”，
    具体的安全预检、拆段、执行和失败处理由 `VoidGripSopRunner` 统一负责。
    @param index 时序表中的序号或脚本内部顺序号。
    @param name 面向操作者显示的动作名称。
    @param kind 动作类型，支持 `recognition`、`translate`、`rotate`、`gripper`。
    @param duration_s 节拍等待时间，单位秒；为 0 时不额外等待。
    @param axis 平移动作使用的基坐标轴，或旋转动作使用的工具坐标轴。
    @param distance_mm 平移动作行程，单位 mm，正负号表示方向。
    @param angle_deg 旋转动作角度，单位 deg，正负号表示方向。
    @param gripper_value 夹爪目标值，范围 0~1000。
    """

    index: int
    name: str
    kind: str
    duration_s: float = 0.0
    axis: Optional[str] = None
    distance_mm: float = 0.0
    angle_deg: float = 0.0
    gripper_value: Optional[float] = None


@dataclass
class VoidGripSopConfig:
    """@brief 虚空识别与虚空夹 SOP 运行配置。
    @details
    配置集中保存所有会影响连接、模型、运动速度、分段粒度和人工确认策略的参数。
    其中 `urdf_path` 是可选覆盖项：默认不使用固定路径，而是依赖 SDK 在连接后通过
    `synriard` 自动加载 RobotModel；只有用户显式传入路径时才用本地 URDF 构建模型。
    @param port Alicia-M 串口名，空字符串表示 SDK 自动扫描。
    @param mode 控制模式，支持 `pv` 或 `mit`。
    @param version 模型版本提示，默认 `auto`。
    @param variant 模型变体，默认 `follower`。
    @param urdf_path 可选本地 URDF 覆盖路径。
    @param speed 末端运动速度参数。
    @param gripper_speed 夹爪运动速度参数。
    @param rotate_deg 默认末端旋转角度。
    @param step_mm 笛卡尔大位移拆段长度。
    @param ik_residual_tol IK 残差上限。
    @param require_confirmation 是否在真实动作前要求人工输入 YES。
    @param dry_run 是否只打印动作流程而不连接设备。
    """

    port: str = ""
    mode: str = "pv"
    version: str = "auto"
    variant: str = "follower"
    backend: str = "numpy"
    urdf_path: str = ""
    speed: float = DEFAULT_SPEED
    initial_speed: float = DEFAULT_INITIAL_SPEED
    initial_joints_deg: list[float] | None = field(
        default_factory=lambda: list(DEFAULT_INITIAL_JOINTS_DEG)
    )
    skip_initial_pose: bool = False
    mit_disable_interpolation: bool = False
    gripper_speed: float = DEFAULT_GRIPPER_SPEED
    rotate_deg: float = DEFAULT_ROTATE_DEG
    step_mm: float = DEFAULT_STEP_MM
    ik_residual_tol: float = DEFAULT_IK_RESIDUAL_TOL
    require_confirmation: bool = False
    dry_run: bool = False
    log_dir: str = ""


class SopSafetyError(RuntimeError):
    """@brief SOP 安全检查或动作执行失败。
    @details
    该异常表示当前 SOP 不应继续执行。调用方捕获后必须停止后续动作，
    并尽力执行失能和断开连接。
    """


def _axis_vector(axis: str) -> np.ndarray:
    """@brief 将轴名称转换为三维单位向量。
    @param axis 坐标轴名称，支持 `x`、`y`、`z`。
    @return 三维单位向量。
    @throws ValueError 当轴名称不支持时抛出。
    """
    key = axis.lower()
    if key == "x":
        return np.array([1.0, 0.0, 0.0], dtype=float)
    if key == "y":
        return np.array([0.0, 1.0, 0.0], dtype=float)
    if key == "z":
        return np.array([0.0, 0.0, 1.0], dtype=float)
    raise ValueError(f"unsupported axis: {axis}")


def _rotation_matrix(axis: str, angle_deg: float) -> np.ndarray:
    """@brief 构造工具坐标系旋转矩阵。
    @details
    旋转矩阵用于右乘到当前末端姿态，因此动作含义是“绕当前工具坐标轴旋转”。
    @param axis 工具坐标轴，支持 `x`、`y`、`z`。
    @param angle_deg 旋转角度，单位 deg。
    @return 3x3 旋转矩阵。
    @throws ValueError 当轴名称不支持时抛出。
    """
    angle = math.radians(angle_deg)
    c = math.cos(angle)
    s = math.sin(angle)
    key = axis.lower()
    if key == "x":
        return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)
    if key == "y":
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)
    if key == "z":
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    raise ValueError(f"unsupported axis: {axis}")


def _as_bool(value: Any) -> bool:
    """@brief 将 SDK 返回的不同布尔形式统一解释为布尔值。
    @param value SDK 调用返回值。
    @return True 表示成功，False 表示失败。
    """
    return bool(value)


def _ik_success(value: Any) -> bool:
    """@brief 兼容不同 RoboCore 返回类型的 IK 成功字段。
    @param value `set_pose(..., execute=False)` 返回字典中的 `success` 值。
    @return True 表示 IK 收敛成功。
    """
    if isinstance(value, (list, tuple, np.ndarray)):
        return bool(np.asarray(value).all())
    return bool(value)


def _extract_transform(pose_info: Any) -> np.ndarray:
    """@brief 从 SDK 位姿结果中提取 4x4 齐次变换矩阵。
    @param pose_info `robot.get_pose()` 返回的位姿字典或矩阵。
    @return 4x4 齐次变换矩阵副本。
    @throws SopSafetyError 当位姿为空或格式错误时抛出。
    """
    if pose_info is None:
        raise SopSafetyError("无法读取当前末端位姿；请确认 RobotModel/URDF 已加载")
    transform = pose_info.get("transform") if isinstance(pose_info, dict) else pose_info
    arr = np.asarray(transform, dtype=float)
    if arr.shape != (4, 4):
        raise SopSafetyError(f"末端位姿格式错误，期望 4x4，实际 {arr.shape}")
    return arr.copy()


def _safe_joint_list(values: Any) -> list[float]:
    """@brief 将 IK 结果中的关节角转换为普通浮点列表。
    @param values IK 返回的关节角数组或列表。
    @return 6 个关节角，单位 rad。
    @throws SopSafetyError 当结果长度不足 6 时抛出。
    """
    arr = np.asarray(values, dtype=float).reshape(-1)
    if arr.size < 6:
        raise SopSafetyError(f"IK 返回关节数量不足，期望至少 6，实际 {arr.size}")
    return [float(v) for v in arr[:6]]


def parse_urdf_revolute_limits(path: Path) -> tuple[tuple[float, float], ...]:
    """@brief 从 URDF 中解析前 6 个旋转关节限位。
    @details
    该函数只解析 `<joint type="revolute">` 的 `<limit lower="..." upper="...">`，
    用于 IK 预检后的关节解安全判断。它不解析碰撞几何，也不做自碰撞或环境碰撞检测。
    @param path URDF 文件路径。
    @return 6 个 `(lower, upper)` 关节限位元组，单位 rad。
    @throws SopSafetyError 当 URDF 不可读、关节数量不足或限位字段缺失时抛出。
    """
    try:
        root = ET.parse(path).getroot()
    except Exception as exc:
        raise SopSafetyError(f"解析 URDF 失败: {path}: {exc}") from exc
    limits: list[tuple[float, float]] = []
    for joint in root.findall("joint"):
        if joint.attrib.get("type") != "revolute":
            continue
        limit = joint.find("limit")
        if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
            raise SopSafetyError(f"URDF 旋转关节缺少 lower/upper 限位: {joint.attrib.get('name', '<unnamed>')}")
        limits.append((float(limit.attrib["lower"]), float(limit.attrib["upper"])))
        if len(limits) == 6:
            return tuple(limits)
    raise SopSafetyError(f"URDF 旋转关节数量不足，期望 6，实际 {len(limits)}")


def build_default_actions(rotate_deg: float) -> list[SopAction]:
    """@brief 构造图片时序表对应的默认 SOP 动作。
    @details
    行程参数保持原始表格含义：X/Y/Z 平移使用基坐标系，末端旋转使用工具坐标系 Z 轴。
    识别和夹爪均为虚空动作，不依赖相机或物体。
    @param rotate_deg 两次末端旋转使用的角度，单位 deg。
    @return SOP 动作列表。
    """
    return [
        SopAction(1, "虚空识别", "recognition", duration_s=0.5),
        SopAction(2, "末端 X 移动 +150mm", "translate", axis="x", distance_mm=150.0),
        SopAction(3, "末端 Y 移动 +800mm", "translate", axis="y", distance_mm=800.0),
        SopAction(4, "末端旋转", "rotate", duration_s=0.3, axis="z", angle_deg=rotate_deg),
        SopAction(5, "末端下降接近 -200mm", "translate", duration_s=0.3, axis="z", distance_mm=-200.0),
        SopAction(6, "虚空夹爪闭合", "gripper", duration_s=0.2, gripper_value=0.0),
        SopAction(9, "末端上升 +200mm", "translate", duration_s=0.3, axis="z", distance_mm=200.0),
        SopAction(10, "末端旋转", "rotate", duration_s=0.2, axis="z", angle_deg=rotate_deg),
        SopAction(11, "末端 Y 移动 +200mm", "translate", axis="y", distance_mm=200.0),
        SopAction(12, "末端下降 -150mm", "translate", duration_s=0.3, axis="z", distance_mm=-150.0),
        SopAction(13, "虚空夹爪打开", "gripper", duration_s=0.2, gripper_value=1000.0),
        SopAction(14, "机械臂上升 +150mm", "translate", duration_s=0.3, axis="z", distance_mm=150.0),
        SopAction(15, "末端 Y 移动 +200mm", "translate", axis="y", distance_mm=200.0),
    ]


def build_parser() -> argparse.ArgumentParser:
    """@brief 构造命令行参数解析器。
    @return 已配置好的 `argparse.ArgumentParser`。
    """
    parser = argparse.ArgumentParser(description="Run Alicia-M void recognition and void gripper SOP.")
    parser.add_argument("--port", type=str, default="", help="Serial port, for example COM37.")
    parser.add_argument("--mode", choices=["pv", "mit"], default="pv", help="Control mode.")
    parser.add_argument("--urdf-path", type=str, default="", help="Optional local URDF override path.")
    parser.add_argument("--version", type=str, default="auto", help="Model version passed to create_robot.")
    parser.add_argument("--variant", type=str, default="follower", help="Model variant passed to create_robot.")
    parser.add_argument("--backend", choices=["auto", "numpy", "torch"], default="numpy", help="RoboCore backend.")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED, help="Motion speed.")
    parser.add_argument(
        "--initial-speed",
        type=float,
        default=DEFAULT_INITIAL_SPEED,
        help="Slow speed used when moving to the initial joint pose.",
    )
    parser.add_argument(
        "--initial-joints-deg",
        type=str,
        default=",".join(f"{value:g}" for value in DEFAULT_INITIAL_JOINTS_DEG),
        help="Initial joint pose in deg, comma-separated 6 values.",
    )
    parser.add_argument(
        "--skip-initial-pose",
        action="store_true",
        help="Do not move to the predefined initial joint pose before Cartesian SOP.",
    )
    parser.add_argument(
        "--mit-disable-interpolation",
        action="store_true",
        help="MIT initial move uses direct PD path; linear velocity field is disabled with 0xFFFF.",
    )
    parser.add_argument("--gripper-speed", type=float, default=DEFAULT_GRIPPER_SPEED, help="Gripper speed.")
    parser.add_argument("--rotate-deg", type=float, default=DEFAULT_ROTATE_DEG, help="Tool Z rotation angle.")
    parser.add_argument("--step-mm", type=float, default=DEFAULT_STEP_MM, help="Cartesian segment length in mm.")
    parser.add_argument("--ik-residual-tol", type=float, default=DEFAULT_IK_RESIDUAL_TOL)
    parser.add_argument("--require-confirmation", action="store_true", help="Require YES before real motion.")
    parser.add_argument("--dry-run", action="store_true", help="Print SOP only; do not connect or move.")
    return parser


def config_from_args(args: argparse.Namespace) -> VoidGripSopConfig:
    """@brief 将命令行参数转换为 SOP 配置。
    @param args `argparse` 解析后的命名空间。
    @return `VoidGripSopConfig` 配置对象。
    @throws ValueError 当速度、分段长度或残差阈值非法时抛出。
    """
    if args.speed <= 0:
        raise ValueError("--speed must be positive")
    if args.gripper_speed <= 0:
        raise ValueError("--gripper-speed must be positive")
    if args.initial_speed <= 0:
        raise ValueError("--initial-speed must be positive")
    if args.step_mm <= 0:
        raise ValueError("--step-mm must be positive")
    if args.ik_residual_tol <= 0:
        raise ValueError("--ik-residual-tol must be positive")
    initial_joints = _parse_float_list(args.initial_joints_deg, 6, "--initial-joints-deg")
    return VoidGripSopConfig(
        port=args.port,
        mode=args.mode,
        version=args.version,
        variant=args.variant,
        backend=args.backend,
        urdf_path=args.urdf_path,
        speed=args.speed,
        initial_speed=args.initial_speed,
        initial_joints_deg=initial_joints,
        skip_initial_pose=args.skip_initial_pose,
        mit_disable_interpolation=args.mit_disable_interpolation,
        gripper_speed=args.gripper_speed,
        rotate_deg=args.rotate_deg,
        step_mm=args.step_mm,
        ik_residual_tol=args.ik_residual_tol,
        require_confirmation=args.require_confirmation,
        dry_run=args.dry_run,
    )


def _parse_float_list(text: str, expected_len: int, name: str) -> list[float]:
    """@brief 解析固定长度的逗号分隔浮点数列表。
    @param text 命令行传入的逗号分隔字符串。
    @param expected_len 期望元素数量。
    @param name 参数名称，用于错误消息。
    @return 解析后的浮点数列表。
    @throws ValueError 当元素数量不符合要求或包含非数字内容时抛出。
    """
    try:
        values = [float(item.strip()) for item in text.split(",") if item.strip()]
    except ValueError as exc:
        raise ValueError(f"{name} must contain numbers") from exc
    if len(values) != expected_len:
        raise ValueError(f"{name} must contain {expected_len} comma-separated numbers")
    return values


class VoidGripSopRunner:
    """@brief 虚空识别与虚空夹 SOP 执行器。
    @details
    执行器负责连接设备、加载或覆盖 RobotModel、预检每个末端目标、执行动作、
    读取状态和安全退出。它不会调用相机，也不会判断物体是否被夹住。
    @param config SOP 运行配置。
    @param robot_factory 机器人对象工厂，默认使用 `alicia_m_sdk.create_robot`。
    @param sleeper 等待函数，默认使用 `time.sleep`；单元测试可传入空函数。
    """

    def __init__(
        self,
        config: VoidGripSopConfig,
        robot_factory: Callable[..., Any] = alicia_m_sdk.create_robot,
        sleeper: Callable[[float], None] = time.sleep,
    ):
        self.config = config
        self.robot_factory = robot_factory
        self.sleeper = sleeper
        self.robot: Any = None
        self.actions = build_default_actions(config.rotate_deg)
        self._step_counter = 0
        self._executed_segments = 0
        self._completed_actions: list[int] = []
        self._gripper_actions = 0
        self._joint_limits = DEFAULT_JOINT_LIMITS_RAD

    def run(self) -> int:
        """@brief 执行一次完整 SOP。
        @return 进程退出码；0 表示 PASS，1 表示 FAIL。
        """
        started_at = datetime.now().astimezone()
        status = FAIL
        failure_reason = ""
        try:
            self._print_plan()
            if self.config.dry_run:
                print("DRY-RUN: 未连接设备，未下发任何动作。")
                status = PASS
                return 0
            self._confirm_if_required()
            self._connect()
            self._override_robot_model_if_requested()
            self._require_robot_model()
            self._prepare_control_mode()
            self._check_robot_state()
            self._move_to_initial_pose()
            for action in self.actions:
                self._run_action(action)
            print(f"SOP result: {PASS}")
            status = PASS
            return 0
        except (KeyboardInterrupt, SopSafetyError, ValueError) as exc:
            failure_reason = str(exc)
            print(f"SOP result: {FAIL}")
            print(f"原因: {failure_reason}")
            return 1
        finally:
            self._safe_shutdown()
            self._write_summary_log(started_at, datetime.now().astimezone(), status, failure_reason)

    def _print_plan(self) -> None:
        """@brief 打印即将执行的 SOP 动作清单。
        @details
        该输出用于让操作者在确认前看到完整动作方向、行程和虚空步骤。
        它不代表动作已经通过 IK 或设备状态检查。
        """
        print("Alicia-M 虚空识别 + 虚空夹 SOP")
        print(f"模式: {self.config.mode.upper()}  串口: {self.config.port or 'auto'}")
        print(f"速度: {self.config.speed}  夹爪速度: {self.config.gripper_speed}")
        if self.config.skip_initial_pose:
            print("初始位姿: 跳过")
        else:
            print(f"初始位姿: {self.config.initial_joints_deg} deg, 速度: {self.config.initial_speed}")
        print(f"分段: {self.config.step_mm}mm  旋转: {self.config.rotate_deg}deg")
        print(f"URDF 覆盖: {self.config.urdf_path or '未指定，使用 SDK/synriard 自动模型'}")
        print("动作表:")
        for action in self.actions:
            print(f"  {action.index:>2}. {action.name}")

    def _confirm_if_required(self) -> None:
        """@brief 根据配置执行人工安全确认。
        @details
        该确认只降低误启动风险，不替代现场安全围栏、急停和人员避让。
        @throws SopSafetyError 操作者未输入 `YES` 时抛出。
        """
        if not self.config.require_confirmation:
            return
        answer = input("确认工作空间已清空且急停可用，输入 YES 开始真实动作: ").strip()
        if answer != "YES":
            raise SopSafetyError("操作者未确认安全区域")

    def _connect(self) -> None:
        """@brief 创建并连接 Alicia-M 机器人对象。
        @details
        默认依赖 SDK 在连接后根据硬件版本通过 `synriard` 自动加载 RobotModel。
        @throws SopSafetyError 当连接或对象创建失败时抛出。
        """
        try:
            self.robot = self.robot_factory(
                port=self.config.port,
                version=self.config.version,
                variant=self.config.variant,
                control_mode=self.config.mode,
                sync_control_mode=True,
                auto_connect=True,
                backend=self.config.backend,
            )
        except Exception as exc:
            raise SopSafetyError(f"连接机器人失败: {exc}") from exc

    def _override_robot_model_if_requested(self) -> None:
        """@brief 按需使用本地 URDF 覆盖 RobotModel。
        @details
        默认不使用固定本机路径。只有用户显式传入 `--urdf-path` 时，才尝试用该 URDF
        构建 RoboCore RobotModel，并覆盖到当前 robot 的内部模型上，供 `get_pose()` 和
        `set_pose()` 使用。
        @throws SopSafetyError 当路径不存在或 RoboCore 构建失败时抛出。
        """
        if not self.config.urdf_path:
            return
        path = Path(self.config.urdf_path)
        if not path.exists():
            raise SopSafetyError(f"URDF 路径不存在: {path}")
        try:
            import robocore as rc
            from robocore.modeling import RobotModel

            rc.set_backend(self.config.backend)
            model = RobotModel(str(path), base_link="base_link", end_link="tool0")
            self._joint_limits = parse_urdf_revolute_limits(path)
        except Exception as exc:
            raise SopSafetyError(f"使用 URDF 构建 RobotModel 失败: {exc}") from exc
        setattr(self.robot, "_robot_model", model)

    def _require_robot_model(self) -> None:
        """@brief 确认机器人模型可用于 FK/IK。
        @details
        没有 RobotModel 时无法对末端目标做 IK 预检，因此必须在真实动作前失败退出。
        @throws SopSafetyError 当模型不可用或当前位姿不可读时抛出。
        """
        if getattr(self.robot, "robot_model", None) is None:
            raise SopSafetyError("RobotModel 不可用；请安装 synriard/robocore 或传入 --urdf-path")
        _extract_transform(self.robot.get_pose())

    def _prepare_control_mode(self) -> None:
        """@brief 切换并准备目标控制模式。
        @details
        PV 模式直接用于点位执行；MIT 模式在切换后初始化阻抗增益，避免首次动作时
        Kp/Kd 跳变过大。
        @throws SopSafetyError 当模式切换或 MIT 增益初始化失败时抛出。
        """
        current = getattr(getattr(self.robot, "control_mode", None), "value", None)
        if current != self.config.mode:
            if not _as_bool(self.robot.switch_mode(self.config.mode)):
                raise SopSafetyError(f"切换到 {self.config.mode.upper()} 模式失败")
        if self.config.mode == "mit":
            if not _as_bool(self.robot.initialize_mit_gains()):
                raise SopSafetyError("MIT 阻抗增益初始化失败")

    def _move_to_initial_pose(self) -> None:
        """@brief 慢速移动到 SOP 初始关节位姿。
        @details
        该步骤用于避免直接从任意当前姿态做笛卡尔相对移动，从而降低 IK 初值差、
        奇异位形或姿态不一致导致的失败概率。默认初始位姿为
        `[0, -90, -45, 0, 45, 0] deg`，用于表达“1/2/3 竖直且 3 关节夹角为
        45 度”的预备姿态；现场如需更精确位姿，可通过 `--initial-joints-deg`
        覆盖。PV 模式使用低速点位运动；MIT 模式默认使用固件线性轨迹插值，
        只有传入 `--mit-disable-interpolation` 时才关闭插值，此时底层线性速度字段由
        SDK 填充 0xFFFF。
        @throws SopSafetyError 当初始位姿缺失、运动失败或运动后状态异常时抛出。
        """
        if self.config.skip_initial_pose:
            return
        if self.config.initial_joints_deg is None:
            raise SopSafetyError("初始关节位姿未配置")
        print(f"[init] 慢速移动到初始位姿 {self.config.initial_joints_deg} deg")
        kwargs: dict[str, Any] = {}
        if self.config.mode == "mit":
            kwargs.update(
                {
                    "use_interpolation": not self.config.mit_disable_interpolation,
                    "kp": [150.0] * 7,
                    "kd": [2.0] * 7,
                    "torque": [0.0] * 7,
                    "vel_ref": [0.0] * 7,
                }
            )
        ok = self.robot.set_robot_state(
            target_joints=self.config.initial_joints_deg,
            joint_format="deg",
            speed=self.config.initial_speed,
            wait_for_completion=True,
            **kwargs,
        )
        if not _as_bool(ok):
            raise SopSafetyError("移动到初始位姿失败")
        self._check_robot_state()

    def _check_robot_state(self) -> None:
        """@brief 检查基础设备状态。
        @details
        当前检查包含：状态是否可读、RobotStatus 是否报告电机错误、JointState 运行状态
        是否置位电机错误位。本函数不做环境碰撞检测。
        @throws SopSafetyError 当状态不可读或检测到错误位时抛出。
        """
        state = self.robot.get_robot_state("all")
        if state is None:
            raise SopSafetyError("无法读取机器人状态")
        status = self.robot.get_robot_state("status")
        if getattr(status, "has_motor_error", False):
            raise SopSafetyError("RobotStatus 报告电机错误")
        if getattr(state, "run_status", 0) & 0x80:
            raise SopSafetyError(f"JointState run_status 电机错误位已置位: 0x{state.run_status:02X}")

    def _run_action(self, action: SopAction) -> None:
        """@brief 执行单个 SOP 动作。
        @param action 待执行的 SOP 动作。
        @throws SopSafetyError 当动作类型未知、预检失败或执行失败时抛出。
        """
        print(f"[{action.index}] {action.name}")
        if action.kind == "recognition":
            self._wait_action(action.duration_s)
        elif action.kind == "translate":
            self._run_translation(action)
        elif action.kind == "rotate":
            self._run_rotation(action)
        elif action.kind == "gripper":
            self._run_gripper(action)
        else:
            raise SopSafetyError(f"未知动作类型: {action.kind}")
        self._check_robot_state()
        self._completed_actions.append(action.index)

    def _wait_action(self, duration_s: float) -> None:
        """@brief 执行节拍等待。
        @details
        用于虚空识别和表格中的等待时间。该函数不访问相机，也不查询视觉模块。
        @param duration_s 等待时间，单位秒。
        """
        if duration_s > 0:
            self.sleeper(duration_s)

    def _run_translation(self, action: SopAction) -> None:
        """@brief 执行基坐标系末端平移。
        @details
        平移会按 `step_mm` 拆成多个中间目标。每个目标都先做 IK 预检，
        预检通过后才下发真实运动。
        @param action 平移动作定义。
        @throws SopSafetyError 当 IK 失败、关节越限或执行失败时抛出。
        """
        if action.axis is None:
            raise SopSafetyError("平移动作缺少 axis")
        start = _extract_transform(self.robot.get_pose())
        direction = _axis_vector(action.axis)
        total_m = action.distance_mm / 1000.0
        segments = max(1, math.ceil(abs(action.distance_mm) / self.config.step_mm))
        for index in range(1, segments + 1):
            target = start.copy()
            target[:3, 3] = start[:3, 3] + direction * (total_m * index / segments)
            self._precheck_and_execute_pose(target, f"{action.name} segment {index}/{segments}")
        self._wait_action(action.duration_s)

    def _run_rotation(self, action: SopAction) -> None:
        """@brief 执行工具坐标系末端旋转。
        @details
        旋转通过右乘工具坐标系旋转矩阵实现。默认绕工具 Z 轴旋转 `rotate_deg`。
        @param action 旋转动作定义。
        @throws SopSafetyError 当 IK 失败、关节越限或执行失败时抛出。
        """
        if action.axis is None:
            raise SopSafetyError("旋转动作缺少 axis")
        start = _extract_transform(self.robot.get_pose())
        target = start.copy()
        target[:3, :3] = start[:3, :3] @ _rotation_matrix(action.axis, action.angle_deg)
        self._precheck_and_execute_pose(target, action.name)
        self._wait_action(action.duration_s)

    def _run_gripper(self, action: SopAction) -> None:
        """@brief 执行虚空夹爪动作。
        @details
        该函数只下发夹爪位置命令，不检测是否存在物体、不检查夹持力。
        @param action 夹爪动作定义。
        @throws SopSafetyError 当夹爪目标缺失或 SDK 返回失败时抛出。
        """
        if action.gripper_value is None:
            raise SopSafetyError("夹爪动作缺少 gripper_value")
        ok = self.robot.set_gripper_target(value=action.gripper_value, wait_for_completion=True)
        if not _as_bool(ok):
            raise SopSafetyError(f"夹爪动作失败: target={action.gripper_value}")
        self._gripper_actions += 1
        self._wait_action(action.duration_s)

    def _precheck_and_execute_pose(self, target: np.ndarray, label: str) -> None:
        """@brief 对单个末端目标执行 IK 预检并下发运动。
        @details
        安全检查包含 IK 是否成功、残差是否低于阈值、6 个关节角是否处于限位内。
        这些检查不能证明路径无碰撞，只能证明目标位姿在当前模型下可达且关节解合理。
        @param target 4x4 末端目标位姿。
        @param label 日志中显示的动作标签。
        @throws SopSafetyError 当预检或执行失败时抛出。
        """
        self._step_counter += 1
        result = self.robot.set_pose(
            target,
            method="dls",
            execute=False,
            speed=self.config.speed,
            max_iters=5000,
            pos_tol=1e-2,
            ori_tol=1e-2,
            num_initial_guesses=12,
            initial_guess_strategy="current",
            use_analytic_jacobian=True,
        )
        if not isinstance(result, dict) or not _ik_success(result.get("success", False)):
            raise SopSafetyError(f"IK 预检失败: {label}")
        residual = float(result.get("residual", 0.0))
        if residual > self.config.ik_residual_tol:
            raise SopSafetyError(
                f"IK 残差过大: {label}, residual={residual:.6f}, limit={self.config.ik_residual_tol:.6f}"
            )
        self._assert_joints_in_limits(_safe_joint_list(result.get("q", [])), label)
        executed = self.robot.set_pose(
            target,
            method="dls",
            execute=True,
            speed=self.config.speed,
            max_iters=5000,
            pos_tol=1e-2,
            ori_tol=1e-2,
            num_initial_guesses=12,
            initial_guess_strategy="current",
            use_analytic_jacobian=True,
        )
        if not isinstance(executed, dict) or not _as_bool(executed.get("motion_executed", False)):
            raise SopSafetyError(f"末端动作执行失败: {label}")
        self._executed_segments += 1

    def _assert_joints_in_limits(self, joints: Iterable[float], label: str) -> None:
        """@brief 检查 IK 关节解是否处于安全限位内。
        @details
        限位默认来自 Alicia_M v1.1 follower URDF 的 6 个旋转关节。
        如果未来需要适配不同硬件版本，应在此处扩展为从指定 URDF 解析。
        @param joints 6 个关节角，单位 rad。
        @param label 当前动作标签。
        @throws SopSafetyError 当任一关节越限时抛出。
        """
        for index, (value, (lower, upper)) in enumerate(zip(joints, self._joint_limits)):
            if value < lower or value > upper:
                raise SopSafetyError(
                    f"IK 关节解越限: {label}, J{index + 1}={value:.4f}, limit=[{lower:.4f}, {upper:.4f}]"
                )

    def _safe_shutdown(self) -> None:
        """@brief 尝试安全失能并断开串口。
        @details
        该函数吞掉清理阶段异常，避免掩盖前面真正的失败原因。
        """
        if self.robot is None:
            return
        try:
            self.robot.disable_robot()
        except Exception:
            pass
        try:
            self.robot.disconnect()
        except Exception:
            pass

    def _write_summary_log(
        self,
        started_at: datetime,
        finished_at: datetime,
        status: str,
        failure_reason: str,
    ) -> None:
        """@brief 写出按当前时间命名的 SOP 总结日志。
        @details
        日志目录固定为脚本所在目录下的 `log/`，不存在时自动创建。日志内容以最终总结数据为主，
        包括运行结果、失败原因、关键配置、动作完成情况、IK 预检次数和真实执行段数。
        该函数不记录逐帧轨迹或大体积状态快照，避免日志过大。
        @param started_at SOP 开始时间，带本地时区。
        @param finished_at SOP 结束时间，带本地时区。
        @param status 最终状态，取值 `PASS` 或 `FAIL`。
        @param failure_reason 失败原因；PASS 时为空字符串。
        """
        log_dir = Path(self.config.log_dir) if self.config.log_dir else DEFAULT_LOG_DIR
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = started_at.strftime("%Y%m%d_%H%M%S")
        path = log_dir / f"{stamp}.json"
        summary = {
            "status": status,
            "failure_reason": failure_reason,
            "started_at": started_at.isoformat(timespec="seconds"),
            "finished_at": finished_at.isoformat(timespec="seconds"),
            "duration_s": round((finished_at - started_at).total_seconds(), 3),
            "port": self.config.port or "auto",
            "mode": self.config.mode,
            "version": self.config.version,
            "variant": self.config.variant,
            "backend": self.config.backend,
            "urdf_source": self.config.urdf_path or "sdk_synriard_auto",
            "speed": self.config.speed,
            "initial_speed": self.config.initial_speed,
            "initial_joints_deg": self.config.initial_joints_deg,
            "skip_initial_pose": self.config.skip_initial_pose,
            "mit_disable_interpolation": self.config.mit_disable_interpolation,
            "gripper_speed": self.config.gripper_speed,
            "rotate_deg": self.config.rotate_deg,
            "step_mm": self.config.step_mm,
            "ik_residual_tol": self.config.ik_residual_tol,
            "require_confirmation": self.config.require_confirmation,
            "dry_run": self.config.dry_run,
            "actions_total": len(self.actions),
            "actions_completed": len(self._completed_actions),
            "completed_action_indices": list(self._completed_actions),
            "ik_prechecks": self._step_counter,
            "executed_segments": self._executed_segments,
            "gripper_actions_completed": self._gripper_actions,
        }
        path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"SOP summary log: {path}")


def main(argv: Optional[list[str]] = None) -> int:
    """@brief 命令行入口。
    @param argv 可选参数列表；为 None 时读取 `sys.argv[1:]`。
    @return 进程退出码；0 表示 PASS，1 表示 FAIL。
    """
    parser = build_parser()
    try:
        config = config_from_args(parser.parse_args(argv))
    except ValueError as exc:
        parser.error(str(exc))
    return VoidGripSopRunner(config).run()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
