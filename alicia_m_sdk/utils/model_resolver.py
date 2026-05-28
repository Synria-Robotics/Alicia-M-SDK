"""model_resolver.py — 硬件版本 → URDF 模型版本映射

将固件上报的整数硬件版本号（如 100、101）解析为 synriard 版本字符串，
并封装机器人模型的加载流程，供 create_robot() 和 SynriaRobotAPI 调用。

版本号规则（4 字节小端序 uint32）：
  100, 101  → Alicia-M v1.1  → synriard "v1_1"
  >=102     → Alicia-M v1.2  → synriard "v1_2"
"""

from __future__ import annotations

from typing import Optional

from .beauty_logger import logger

__all__ = [
    "HW_VERSION_MAP",
    "resolve_model_version",
    "load_robot_model",
]

# ---------------------------------------------------------------------------
# Hardware-version → synriard version string mapping
# Key  : VersionInfo.hardware_version (uint32 decoded as decimal string)
# Value: synriard version argument accepted by get_model_path()
# ---------------------------------------------------------------------------
HW_VERSION_MAP: dict[str, str] = {
    "100": "v1_1",   # Alicia-M v1.1
    "101": "v1_1",   # Alicia-M v1.1
    "102+": "v1_2",  # Alicia-M v1.2 and later compatible hardware
}

_HW_VERSION_V1_1_MIN = 100
_HW_VERSION_V1_2_MIN = 102


def resolve_model_version(hw_version: str) -> str:
    """将固件硬件版本号解析为 synriard 版本字符串。

    :param hw_version: ``VersionInfo.hardware_version``，如 ``"100"``、``"101"`` 或 ``"102"``
    :returns: synriard 版本字符串，如 ``"v1_1"``
    :raises ValueError: 版本号不在已知映射表中
    """
    try:
        version_num = int(str(hw_version).strip())
    except (TypeError, ValueError):
        version_num = -1

    if _HW_VERSION_V1_1_MIN <= version_num < _HW_VERSION_V1_2_MIN:
        return "v1_1"
    if version_num >= _HW_VERSION_V1_2_MIN:
        return "v1_2"

    known = ", ".join(f"{k} → {v}" for k, v in HW_VERSION_MAP.items())
    raise ValueError(
        f"Unknown hardware version: '{hw_version}'. "
        f"Known mappings: [{known}]. "
        f"Please upgrade alicia-m-sdk or synriard to support this hardware."
    )


def load_robot_model(
    version: str,
    variant: str,
    backend: str,
    base_link: str = "base_link",
    end_link: str = "tool0",
):
    """加载 synriard URDF 模型并构建 RoboCore RobotModel 实例。

    :param version: synriard 版本字符串，如 ``"v1_1"``
    :param variant: URDF 变体，如 ``"follower"`` 或 ``"vertical"``
    :param backend: RoboCore 计算后端，``"cpp"`` / ``"numpy"`` / ``"torch"``
    :param base_link: 运动链基座链接名
    :param end_link: 运动链末端链接名
    :returns: RoboCore ``RobotModel`` 实例；依赖库缺失时返回 ``None``
    :raises RuntimeError: synriard 中找不到对应版本/变体的 URDF
    :raises ImportError: robocore 或 synriard 未安装（已内部捕获并返回 None）
    """
    try:
        import robocore as rc
        from robocore.modeling import RobotModel as _RobotModel
        from synriard import get_model_path
    except ImportError as exc:
        logger.warning(
            f"Cannot load robot model: {exc}. "
            "Kinematics and planning will be unavailable."
        )
        return None

    try:
        model_path = get_model_path(
            "Alicia_M",
            version=version,
            variant=variant,
            model_format="urdf",
        )
    except (ValueError, AttributeError) as exc:
        raise RuntimeError(
            f"URDF not found in synriard for Alicia_M {version} / {variant}. "
            f"Ensure synriard >= 1.2.2 is installed and contains this version. "
            f"Detail: {exc}"
        ) from exc

    try:
        rc.set_backend(backend)
        return _RobotModel(str(model_path), base_link=base_link, end_link=end_link)
    except Exception as exc:
        raise RuntimeError(
            f"Failed to build RobotModel from '{model_path}': {exc}"
        ) from exc
