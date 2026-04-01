"""SDK 统一异常体系

层级化异常类，禁止将原始协议错误或 Python 原生异常无包装地抛给用户。
异常消息必须可读、包含关键上下文。
"""

from __future__ import annotations

__all__ = [
    'AliciaSDKError',
    'ConnectionError',
    'TimeoutError',
    'ProtocolError',
    'ValidationError',
    'RobotStateError',
    'HardwareFaultError',
    'MotionError',
]


class AliciaSDKError(Exception):
    """SDK 基础异常，所有 SDK 异常的父类

    所有由 Alicia-M-SDK 主动抛出的异常都继承自此类，
    用户可通过捕获此异常统一处理所有 SDK 错误。
    """
    pass


class ConnectionError(AliciaSDKError):
    """串口连接失败或已断开

    触发场景：
    - 串口未连接时发送指令
    - 串口打开失败（端口不存在、权限不足、被占用）
    - 自动发现未找到可用端口
    """
    pass


class TimeoutError(AliciaSDKError):
    """通信超时

    触发场景：
    - send_and_wait 等待响应超时
    - connect 流程超时（串口打开 + 自动检测 + 首次状态获取）
    """
    pass


class ProtocolError(AliciaSDKError):
    """协议层错误

    触发场景：
    - CRC32 校验失败
    - 帧格式异常（帧头/帧尾不匹配、长度字段错误）
    - 消息解码失败（未知指令 ID、数据长度不符预期）
    """
    pass


class ValidationError(AliciaSDKError):
    """参数校验失败

    触发场景：
    - 关节角度超限
    - 速度超范围
    - 参数类型错误
    - 夹爪值超范围

    异常消息示例：
        "关节3超限: 目标=3.40 rad, 限位=[-2.80, 2.80] rad"
    """
    pass


class RobotStateError(AliciaSDKError):
    """机器人状态不满足操作前提

    触发场景：
    - 未使能就发运动指令
    - PV 模式下调用 torque_off
    - 控制模式不匹配
    """
    pass


class HardwareFaultError(AliciaSDKError):
    """固件返回的硬件错误（0xEE 指令映射）

    通过 error_code 属性可获取原始错误码，用于精确定位硬件层故障。

    Attributes:
        error_code: 固件返回的原始错误码
    """

    # 错误码描述映射
    _ERROR_DESCRIPTIONS = {
        0x00: "帧头/帧尾校验错误",
        0x01: "数据长度校验错误",
        0x02: "CRC32 校验不通过",
        0x03: "系统模式错误（机械臂类型检测失败）",
        0x04: "电机角度限位中",
        0x05: "motorData 偏移与部位数量不符",
        0x06: "insID 偏移超过最大地址",
    }

    def __init__(self, error_code: int, detail: str = ""):
        self.error_code = error_code
        # 拼接可读的错误消息
        desc = self._ERROR_DESCRIPTIONS.get(error_code, "未知错误")
        msg = "硬件错误 [0x{:02X}]: {}".format(error_code, desc)
        if detail:
            msg += " - {}".format(detail)
        super().__init__(msg)


class MotionError(AliciaSDKError):
    """运动执行异常

    触发场景：
    - 轨迹执行超时
    - 未到达目标位置
    - 运动过程中检测到硬件错误
    """
    pass
