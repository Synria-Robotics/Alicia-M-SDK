"""Alicia-M 连接握手辅助函数

将原 SynriaRobotAPI 中的内部握手逻辑提取至独立模块，以保持 API 类轻薄。
所有函数接受显式依赖引用并通过返回值传递结果，不持有对 API 实例的反向引用，
便于独立测试。
"""

from __future__ import annotations

import time
from typing import Any, Optional, Tuple

from ..hardware.constants import (
    AIM_LEADER, AIM_FOLLOWER, CMD_VERSION,
    MOTOR_PARAM_CTRL_MODE, CTRL_MODE_MIT, CTRL_MODE_PV,
    NUM_JOINTS,
)
from ..types.config import RobotConfig
from ..types.enums import ControlMode
from ..types.exceptions import ConnectionError
from ..utils.beauty_logger import logger
from ..utils.model_resolver import resolve_model_version, load_robot_model as _load_robot_model


# ─── Aim detection ────────────────────────────────────────────────────────────

def auto_detect_aim(device, codec, timeout: float) -> None:
    """通过版本帧自动检测设备类型并在 *device* 上设置 aim。

    安全策略：自动检测到 Leader 时强制回退为 Follower 并发出 warning，
    以防止 SDK 控制指令（模式切换、使能等）误发往示教臂。
    如确需连接 Leader，请通过 ``create_robot(control_aim='leader')`` 显式指定。
    """
    frame = codec.encode_version_request()
    device.send_and_wait(frame, CMD_VERSION, timeout=timeout)
    info = device.version_info
    if info and info.device_type:
        dt = info.device_type.upper()
        if dt in ('L', 'LEADER'):
            logger.warning(
                "检测到示教臂 (Leader)，但未显式指定 control_aim='leader'。"
                "为防止误操作示教臂固件，已强制设为 Follower 模式。"
                "如确需连接 Leader，请通过 create_robot(control_aim='leader') 显式指定"
            )
            device.set_aim(AIM_FOLLOWER)
        else:
            device.set_aim(AIM_FOLLOWER)
            logger.info("检测到操作臂 (Follower)")
    else:
        device.set_aim(AIM_FOLLOWER)
        logger.info("未检测到设备类型，默认操作臂")


# ─── Control-mode detection & sync ────────────────────────────────────────────

def detect_firmware_mode(device) -> Optional[ControlMode]:
    """带重试地查询固件实际控制模式。

    仅检查关节电机 M0-M5 的一致性；夹爪 M6 在固件侧锁定为 MIT，不参与判断。
    超时或混合模式时返回 ``None``。
    """
    _MODE_MAP = {CTRL_MODE_MIT: ControlMode.MIT, CTRL_MODE_PV: ControlMode.PV}
    for attempt in range(3):
        device.flush()
        values = device.query_motor_params(MOTOR_PARAM_CTRL_MODE, timeout=2.0)
        if values is not None and len(values) >= NUM_JOINTS:
            break
        logger.debug(f"模式检测第 {attempt + 1} 次查询未获得有效响应")
        time.sleep(0.5)
    else:
        return None

    joint_values = values[:NUM_JOINTS]
    if any(v != joint_values[0] for v in joint_values):
        logger.warning(f"检测到关节电机混合控制模式: {joint_values}，将强制同步")
        return None
    return _MODE_MAP.get(joint_values[0])


def sync_control_mode(device, joint_ctrl, config: RobotConfig) -> None:
    """检测固件控制模式并将 SDK 内部 joint_ctrl 状态与之同步。

    当 ``config.control_mode`` 为空时跟随固件当前模式；否则若固件与期望不一致则强制切换。
    查询完全失败时抛出 ``ConnectionError``。
    """
    firmware_mode = detect_firmware_mode(device)
    requested = config.control_mode
    desired = ControlMode(requested.lower()) if requested else firmware_mode

    if desired is None:
        raise ConnectionError("无法检测固件控制模式，请检查串口连接和固件状态")

    if firmware_mode == desired:
        joint_ctrl.mode = desired
        logger.info(f"固件控制模式: {desired.value.upper()}")
    else:
        opposite = ControlMode.MIT if desired == ControlMode.PV else ControlMode.PV
        joint_ctrl.mode = firmware_mode if firmware_mode is not None else opposite
        logger.info(f"切换固件模式 → {desired.value.upper()}")
        joint_ctrl.switch_mode(desired)


# ─── URDF model loading ───────────────────────────────────────────────────────

def load_model_from_hw_version(
    hw_version: str,
    config: RobotConfig,
) -> Tuple[Any, Optional[str]]:
    """将固件上报的硬件版本号解析为对应的 URDF 机器人模型。

    成功时返回 ``(robot_model, model_version)``；失败时返回 ``(None, None)``
    并在 logger 中记录 warning（不抛异常，确保连接流程不被打断）。
    """
    try:
        model_version = resolve_model_version(hw_version)
        model_variant = config.variant if config.variant else "follower"
        robot_model = _load_robot_model(
            version=model_version,
            variant=model_variant,
            backend=config.backend,
            base_link=config.base_link,
            end_link=config.end_link,
        )
        if robot_model is not None:
            logger.info(
                f"Auto-detected hardware v{hw_version} → "
                f"loaded Alicia_M {model_version} ({model_variant}) URDF"
            )
            return robot_model, model_version
    except (ValueError, RuntimeError) as exc:
        logger.warning(f"Auto URDF loading failed (hw_version={hw_version!r}): {exc}")
    return None, None


# ─── Full connection handshake ────────────────────────────────────────────────

def connect_once(
    serial_port,
    device,
    codec,
    joint_ctrl,
    config: RobotConfig,
    model_version_mode: str,
    timeout: float,
) -> Tuple[Any, Optional[str]]:
    """在当前 serial_port 上执行完整的 Alicia-M 六步握手。

    返回 ``(robot_model, resolved_version)``：
    - ``model_version_mode != "auto"`` 时两者均为 ``None``（模型已由调用方预先加载）。
    - ``model_version_mode == "auto"`` 但检测失败时两者也为 ``None``。

    任何步骤失败时均抛出异常，调用方负责在 except 块中调用 ``disconnect()``。
    """
    deadline = time.time() + timeout

    # 1. 打开串口。
    if not serial_port.connect():
        raise ConnectionError("Failed to open serial port; check device connection and permissions")

    # 2. 启动后台读线程和状态轮询线程。
    device.start()

    # 3. 设置或自动检测控制目标（示教臂/操作臂）。
    if config.control_aim:
        aim_str = config.control_aim.lower()
        aim = AIM_LEADER if aim_str == "leader" else AIM_FOLLOWER
        device.set_aim(aim)
    else:
        auto_detect_aim(device, codec, max(deadline - time.time(), 0.5))

    # 4. 验证固件版本响应；无响应则跳过当前串口。
    remaining = max(deadline - time.time(), 0.5)
    frame = codec.encode_version_request()
    device.send_and_wait(frame, CMD_VERSION, timeout=remaining)
    version_info = device.version_info
    firmware_version = version_info.firmware_version if version_info else None
    if firmware_version is None:
        raise ConnectionError("No Alicia-M firmware version response received")

    # 4.5 根据硬件版本自动加载 URDF（仅当 version="auto" 时执行）。
    robot_model: Any = None
    resolved_version: Optional[str] = None
    if model_version_mode == "auto":
        if version_info and version_info.hardware_version:
            robot_model, resolved_version = load_model_from_hw_version(
                version_info.hardware_version, config
            )
        else:
            logger.warning(
                "Hardware version is unavailable after firmware handshake; "
                "robot model will not be loaded. Set version explicitly to bypass auto-detection."
            )

    # 5. 等待首次关节状态缓存填充。
    poll_deadline = min(deadline, time.time() + 2.0)
    while time.time() < poll_deadline:
        if device.joint_state is not None:
            break
        time.sleep(0.05)

    # 6. 检测固件控制模式并同步 SDK 内部状态。
    sync_control_mode(device, joint_ctrl, config)

    return robot_model, resolved_version
