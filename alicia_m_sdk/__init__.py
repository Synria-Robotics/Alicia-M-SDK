"""Alicia-M-SDK: Synria 云擎系列机械臂 Python SDK

提供对 Alicia-M 6-DOF 机械臂的完整控制能力，包括：
- PV（位置-速度）模式：固件侧加减速插值
- MIT（阻抗控制）模式：全参数逐帧控制
- 正/逆运动学（基于 RoboCore）
- 轨迹规划与执行
- 拖动示教与回放

快速开始::

    import alicia_m_sdk

    robot = alicia_m_sdk.create_robot()
    robot.set_robot_state(target_joints=[0, 30, 0, 0, -30, 0], speed=15)
    robot.disconnect()
"""

from typing import Literal, Optional

__version__ = "1.1.1rc1"

# === 核心类型 ===
from .api.synria_robot_api import SynriaRobotAPI
from .types.state import JointState, MitParams, RobotStatus, VersionInfo
from .diagnostics import DiagnosticResult, DiagnosticArmSnapshot
from .user_settings import UserSettings
from .gripper_params import GripperParamResult, GripperParamSpec
from .execution import JointController, Teleoperation, TrajectoryExecutor
from .types.config import RobotConfig
from .types.enums import ControlAim, ControlMode, GripperType
from .types.exceptions import (
    AliciaSDKError,
    ConnectionError,
    TimeoutError,
    ProtocolError,
    ValidationError,
    RobotStateError,
    HardwareFaultError,
    MotionError,
)
from .utils.beauty_logger import logger, LogLevel
def create_robot(
    port: str = "",
    version: str = "v1_1",
    variant: Optional[str] = None,
    control_aim: Optional[str] = None,
    control_mode: Optional[str] = None,
    baudrate: int = 1_000_000,
    backend: Literal["numpy", "torch", "cpp"] = "cpp",
    debug_mode: bool = False,
    auto_connect: bool = True,
    extended_polling: bool = False,
    **kwargs,
) -> SynriaRobotAPI:
    """创建机器人实例的工厂函数

    初始化顺序（与 Alicia-D-SDK 保持一致）:
    1. 设置 RoboCore 计算后端
    2. 加载机器人 URDF 模型（立即初始化，避免首次调用延迟）
    3. 创建 SynriaRobotAPI 实例
    4. 自动连接（可选）

    :param port, 串口端口路径，空字符串表示自动发现
    :param version, 机器人硬件版本 ("v1_0", "v1_1")
    :param variant, 变体标识（None=自动检测）
    :param control_aim, 控制目标 ("leader"/"follower"/None=自动检测)
    :param control_mode, 控制模式 ("pv"/"mit"/None=检测固件当前模式)
    :param baudrate, 串口波特率
    :param backend, RoboCore 计算后端 ("numpy"/"torch"/"cpp")；"cpp" 需要 synria-robocore 2.5.0+ 且已编译 C++ 扩展
    :param debug_mode, 调试模式（启用 DEBUG 级别日志）
    :param auto_connect, 是否自动连接
    :param extended_polling, 扩展轮询（查询插补速度、温度等，需新固件支持）
    :return, SynriaRobotAPI 实例
    """
    if debug_mode:
        logger.set_min_level(LogLevel.DEBUG)
    else:
        logger.set_min_level(LogLevel.INFO)

    # 1. 设置 RoboCore 后端（2.5.0rc2：'cpp' 是合法值，不会抛 ValueError）
    robot_model = None
    try:
        import robocore as rc
        rc.set_backend(backend)

        # 2. 加载机器人模型
        from robocore.modeling import RobotModel as RobotModelClass
        from synriard import get_model_path
        # synriard 的 Alicia_M 模型必须指定 variant
        model_variant = variant if variant else "follower"
        model_path = get_model_path(
            "Alicia_M", version=version,
            variant=model_variant, model_format="urdf",
        )
        robot_model = RobotModelClass(
            str(model_path),
            base_link="base_link",
            end_link="tool0",
        )
    except ImportError:
        # RoboCore 或 synriard 不可用时，运动学功能不可用
        logger.warning("RoboCore / synriard is not installed; kinematics and planning are unavailable")
    except Exception as e:
        logger.warning(f"Failed to load robot model: {e}")

    # 3. 创建实例
    config = RobotConfig(
        port=port,
        version=version,
        variant=variant,
        control_aim=control_aim,
        control_mode=control_mode,
        baudrate=baudrate,
        auto_connect=auto_connect,
        backend=backend,
        debug_mode=debug_mode,
    )
    robot = SynriaRobotAPI(config, robot_model=robot_model)

    # 4. 自动连接
    if auto_connect:
        robot.connect()

    # 5. 按需启用扩展轮询
    if extended_polling:
        robot.set_extended_polling(True)

    return robot


__all__ = [
    # 工厂函数
    'create_robot',
    # 核心类
    'SynriaRobotAPI',
    # 类型
    'JointState', 'MitParams', 'RobotStatus', 'VersionInfo',
    'RobotConfig',
    'ControlAim', 'ControlMode', 'GripperType',
    'DiagnosticResult', 'DiagnosticArmSnapshot', 'UserSettings',
    'GripperParamResult', 'GripperParamSpec',
    # 高级控制入口
    'JointController', 'Teleoperation', 'TrajectoryExecutor',
    # 异常
    'AliciaSDKError', 'ConnectionError', 'TimeoutError',
    'ProtocolError', 'ValidationError', 'RobotStateError',
    'HardwareFaultError', 'MotionError',
]
