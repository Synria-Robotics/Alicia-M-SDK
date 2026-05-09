"""个性化设置 demo 的辅助逻辑。

入口脚本只负责交互流程；0x02 帧构造、发送和响应解析放在这里。
"""

import struct
import time

from alicia_m_sdk.protocol.frame import Frame
from robocore.utils.beauty_logger import beauty_print


CMD_USER_SETTINGS = 0x02
CMD_ERROR = 0xEE
FUNC_READ_ALL_SETTINGS = 0x07
FUNC_WRITE_GRIPPER_TYPE = 0x82
WRITE_ACCEPTED = 0x81

GRIPPER_TYPE_VALUES = {
    0: "默认小夹爪",
    2: "大夹爪",
}

GRIPPER_TYPE_OPTIONS = {
    10: 0,
    40: 2,
}

GRIPPER_TYPE_OPTION_LABELS = {
    10: "小夹爪",
    40: "大夹爪",
}

SETTING_NAMES = [
    "开机动作配置",
    "夹爪类型配置",
    "定时上传开关",
]

ERROR_TYPE_NAMES = {
    0x00: "帧头或帧尾错误",
    0x01: "长度错误",
    0x02: "校验错误",
    0x03: "窗口错误",
    0x04: "角度越界",
    0x05: "数据长度错误",
    0x06: "地址错误",
    0x07: "当前窗口模式不允许",
    0x08: "线性轨迹状态异常",
    0xEE: "模式切换被拒绝",
}


def make_read_settings_frame() -> Frame:
    """构造读取全部个性化设置帧。"""
    return Frame(cmd_id=CMD_USER_SETTINGS, func_code=FUNC_READ_ALL_SETTINGS)


def make_write_gripper_type_frame(gripper_type: int) -> Frame:
    """构造写入夹爪类型配置帧。"""
    return Frame(
        cmd_id=CMD_USER_SETTINGS,
        func_code=FUNC_WRITE_GRIPPER_TYPE,
        data=struct.pack("<I", gripper_type),
    )


def send_user_settings_frame(robot, frame: Frame, timeout: float, warn_on_timeout: bool = True):
    """发送 0x02 个性化设置帧并等待响应。"""
    beauty_print(f"TX: {format_bytes(frame.encode())}", type="info")
    response = send_and_wait_user_settings_or_error(robot, frame, timeout)
    if response is None:
        if warn_on_timeout:
            beauty_print("未收到 0x02 个性化设置响应", type="warning")
        return None
    beauty_print(f"RX: {format_bytes(response.encode())}", type="info")
    if response.cmd_id == CMD_ERROR:
        print_error_response(response)
        return None
    return response


def send_and_wait_user_settings_or_error(robot, frame: Frame, timeout: float):
    """等待 0x02 响应；若下位机返回 0xEE，也立即返回。"""
    device = robot._device
    state_cache = device._state_cache
    settings_event = state_cache.register_pending(CMD_USER_SETTINGS)
    error_event = state_cache.register_pending(CMD_ERROR)

    try:
        device.send_frame(frame)
        deadline = time.perf_counter() + max(timeout, 0)
        while time.perf_counter() <= deadline:
            if settings_event.is_set():
                return state_cache.get_pending_response(CMD_USER_SETTINGS)
            if error_event.is_set():
                return state_cache.get_pending_response(CMD_ERROR)
            time.sleep(0.01)
        return None
    finally:
        clear_pending_response(state_cache, CMD_USER_SETTINGS)
        clear_pending_response(state_cache, CMD_ERROR)


def clear_pending_response(state_cache, cmd_id: int):
    """清理未命中的 pending，避免后续后台响应触发旧等待。"""
    with state_cache._lock:
        state_cache._pending_events.pop(cmd_id, None)
        state_cache._pending_responses.pop(cmd_id, None)


def print_settings_response(frame: Frame):
    """打印读取个性化设置的响应。"""
    values = parse_settings_response(frame)
    if values is None:
        return

    for index, value in enumerate(values):
        name = SETTING_NAMES[index] if index < len(SETTING_NAMES) else f"配置项{index}"
        extra = gripper_type_label(value) if index == 1 else str(value)
        beauty_print(f"  {name}: {extra}", type="info")


def parse_settings_response(frame: Frame):
    """解析读取个性化设置的响应。"""
    if frame.func_code != FUNC_READ_ALL_SETTINGS:
        beauty_print(f"非读取响应功能码: 0x{frame.func_code:02X}", type="warning")
        return None
    if len(frame.data) % 4 != 0:
        beauty_print(f"响应数据长度异常: {len(frame.data)} 字节", type="warning")
        beauty_print(f"DATA: {format_bytes(frame.data)}", type="info")
        return None

    return struct.unpack(f"<{len(frame.data) // 4}I", frame.data)


def confirm_gripper_type(frame: Frame, expected_value: int) -> bool:
    """通过读回配置确认夹爪类型是否已写入。"""
    values = parse_settings_response(frame)
    if values is None or len(values) < 2:
        beauty_print("无法通过读回确认夹爪类型配置", type="warning")
        return False

    actual_value = values[1]
    ok = gripper_type_config_value(actual_value) == gripper_type_config_value(expected_value)
    msg_type = "success" if ok else "warning"
    beauty_print(
        f"夹爪类型写入{'已确认' if ok else '未确认'}: {gripper_type_label(actual_value)}",
        type=msg_type,
    )
    return ok


def print_write_response(frame: Frame) -> bool:
    """打印写入夹爪类型配置的响应。"""
    if frame.func_code != FUNC_WRITE_GRIPPER_TYPE:
        beauty_print(f"非写入响应功能码: 0x{frame.func_code:02X}", type="warning")
        return False
    if len(frame.data) != 1:
        beauty_print(f"写入响应数据长度异常: {len(frame.data)} 字节", type="warning")
        beauty_print(f"DATA: {format_bytes(frame.data)}", type="info")
        return False

    accepted = frame.data[0] == WRITE_ACCEPTED
    msg_type = "success" if accepted else "warning"
    beauty_print(f"写入请求: {'已接收' if accepted else '未确认'} (0x{frame.data[0]:02X})", type=msg_type)
    return accepted


def print_error_response(frame: Frame):
    """打印 0xEE 错误反馈。"""
    error_type = frame.func_code
    extra = frame.data[0] if frame.data else None
    error_name = ERROR_TYPE_NAMES.get(error_type, "未知错误")
    beauty_print(f"个性化设置返回错误: {error_name} (0x{error_type:02X})", type="warning")
    if extra is not None:
        beauty_print(f"  附加信息: 0x{extra:02X}", type="info")


def parse_gripper_type(text: str) -> int:
    """解析并限制夹爪类型配置值。"""
    option = int(text, 0)
    if option not in GRIPPER_TYPE_OPTIONS:
        choices = ", ".join(str(item) for item in sorted(GRIPPER_TYPE_OPTIONS))
        raise ValueError(f"夹爪类型只支持: {choices}")
    return GRIPPER_TYPE_OPTIONS[option]


def gripper_type_label(value: int) -> str:
    """返回夹爪类型配置的显示文本。"""
    config_value = gripper_type_config_value(value)
    label = GRIPPER_TYPE_VALUES[config_value]
    if value in GRIPPER_TYPE_VALUES:
        return f"{value} ({label})"
    return f"{value} (按 bit1 解析为{label}，建议重新写入规范配置值 {config_value})"


def gripper_type_config_value(value: int) -> int:
    """按协议 bit1 解析夹爪类型配置值。"""
    return 2 if value & 0x02 else 0


def gripper_type_option_label(text: str) -> str:
    """返回用户输入选项对应的显示文本。"""
    option = int(text, 0)
    value = GRIPPER_TYPE_OPTIONS[option]
    return f"{option} ({GRIPPER_TYPE_OPTION_LABELS[option]} -> 配置值 {value})"


def format_bytes(data: bytes) -> str:
    """格式化字节数组，便于打印 TX/RX。"""
    return " ".join(f"{byte:02X}" for byte in data)
