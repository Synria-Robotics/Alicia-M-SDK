"""自检 demo 的辅助逻辑。

入口脚本只负责交互流程；版本判断、固定帧发送和结果打印放在这里。
"""

import time
from typing import Optional

from alicia_m_sdk.protocol.frame import Frame
from robocore.utils.beauty_logger import beauty_print


DIAGNOSTIC_REQUEST_HEX = "AA FE 02 01 FE BC FF"
DIAGNOSTIC_CMD_ID = 0xFE
ERROR_CMD_ID = 0xEE
DIAGNOSTIC_SUPPORTED_VERSION = (1, 0, 6)
DIAGNOSTIC_BLOCK_LEN = 15
DIAGNOSTIC_JOINT_COUNT = 7

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

ERROR_EXTRA_LABELS = {
    0x00: "收到的帧长度",
    0x01: "收到的帧长度",
    0x02: "下位机计算出的校验字节",
    0x03: "预留信息",
    0x04: "越界关节编号",
    0x05: "当前数据长度或数量",
    0x06: "非法地址或越界值",
    0x07: "请求功能码",
    0x08: "预留信息",
}

MODE_NAMES = {
    0x0: "普通状态",
    0x1: "控制协议状态",
    0x2: "重力补偿状态",
    0x3: "双臂同步状态",
    0x4: "固件升级状态",
    0x5: "控制锁定状态",
}

MOTOR_STATE_NAMES = {
    0x0: "失能",
    0x1: "使能",
    0x4: "电机轴异常",
    0x8: "超压",
    0x9: "欠压",
    0xA: "过电流",
    0xB: "MOS 过温",
    0xC: "电机线圈过温",
    0xD: "通讯丢失",
    0xE: "过载",
}

CONTROL_MODE_NAMES = {
    0x00: "未读取或未初始化",
    0x01: "mit模式",
    0x02: "pv模式",
    0x03: "速度模式",
    0x04: "位置、速度、电流混合模式",
}


def supports_diagnostic(version):
    """判断固件是否支持自检 demo。"""
    parsed = parse_firmware_version(version)
    return parsed == DIAGNOSTIC_SUPPORTED_VERSION


def parse_firmware_version(version):
    """解析固件版本号，兼容 106、1.0.6 和 v1.0.6。"""
    if not version:
        return None

    text = str(version).strip().lower().lstrip("v")
    if text.isdigit():
        value = int(text)
        return value // 100, (value // 10) % 10, value % 10

    parts = text.split(".")
    if len(parts) == 3 and all(part.isdigit() for part in parts):
        return tuple(int(part) for part in parts)

    return None


def send_diagnostic(robot, timeout: float) -> Optional[Frame]:
    """发送自检固定帧并等待响应。"""
    request = Frame.decode(bytes.fromhex(DIAGNOSTIC_REQUEST_HEX))
    beauty_print(f"TX: {DIAGNOSTIC_REQUEST_HEX}", type="info")
    response = send_and_wait_diagnostic_or_error(robot, request, timeout)
    if response is None:
        beauty_print("未收到自检响应", type="warning")
        return None
    beauty_print(f"RX: {format_bytes(response.encode())}", type="info")
    return response


def send_and_wait_diagnostic_or_error(robot, request: Frame, timeout: float) -> Optional[Frame]:
    """等待自检响应；若下位机返回 0xEE，也交给 demo 显示。"""
    device = robot._device
    state_cache = device._state_cache
    diagnostic_event = state_cache.register_pending(DIAGNOSTIC_CMD_ID)
    error_event = state_cache.register_pending(ERROR_CMD_ID)

    try:
        device.send_frame(request)
        deadline = time.perf_counter() + max(timeout, 0)
        while time.perf_counter() <= deadline:
            if diagnostic_event.is_set():
                return state_cache.get_pending_response(DIAGNOSTIC_CMD_ID)
            if error_event.is_set():
                return state_cache.get_pending_response(ERROR_CMD_ID)
            time.sleep(0.01)
        return None
    finally:
        clear_pending_response(state_cache, DIAGNOSTIC_CMD_ID)
        clear_pending_response(state_cache, ERROR_CMD_ID)


def clear_pending_response(state_cache, cmd_id: int):
    """清理未命中的 pending，避免后续后台响应触发旧等待。"""
    with state_cache._lock:
        state_cache._pending_events.pop(cmd_id, None)
        state_cache._pending_responses.pop(cmd_id, None)


def print_diagnostic_response(frame: Frame) -> bool:
    """打印自检响应快照。"""
    if frame.cmd_id == ERROR_CMD_ID:
        print_error_response(frame)
        return False

    data = frame.data
    if len(data) not in (DIAGNOSTIC_BLOCK_LEN, DIAGNOSTIC_BLOCK_LEN * 2):
        beauty_print(f"响应数据长度异常: {len(data)} 字节", type="warning")
        beauty_print(f"DATA: {format_bytes(data)}", type="info")
        return False

    block_count = len(data) // DIAGNOSTIC_BLOCK_LEN
    arm_names = diagnostic_arm_names(frame.func_code, block_count)
    for block_index in range(block_count):
        offset = block_index * DIAGNOSTIC_BLOCK_LEN
        block = data[offset:offset + DIAGNOSTIC_BLOCK_LEN]
        arm_name = arm_names[block_index]
        beauty_print(f"{arm_name}自检快照:", type="info")
        beauty_print(f"  通信状态位图: 0x{block[0]:02X}", type="info")
        beauty_print(f"  通信状态: {format_comm_bitmap(block[0])}", type="info")
        beauty_print(f"  电机状态: {format_bytes(block[1:8])}", type="info")
        beauty_print(f"  状态含义: {format_motor_states(block[1:8])}", type="info")
        beauty_print(f"  电机模式: {format_bytes(block[8:15])}", type="info")
        beauty_print(f"  模式含义: {format_control_modes(block[8:15])}", type="info")
    return True


def print_error_response(frame: Frame):
    """按 0xEE 错误反馈格式打印错误帧。"""
    error_type = frame.func_code
    extra = frame.data[0] if frame.data else None
    error_name = ERROR_TYPE_NAMES.get(error_type, "未知错误")
    beauty_print(f"自检返回错误: {error_name} (0x{error_type:02X})", type="warning")

    if extra is None:
        beauty_print("  附加信息: 缺失", type="warning")
        return

    if error_type == 0xEE:
        current_mode = (extra & 0xF0) >> 4
        target_mode = extra & 0x0F
        beauty_print(
            f"  当前模式: 0x{current_mode:X} {mode_name(current_mode)}",
            type="info",
        )
        beauty_print(
            f"  目标模式: 0x{target_mode:X} {mode_name(target_mode)}",
            type="info",
        )
        return

    extra_label = ERROR_EXTRA_LABELS.get(error_type, "附加信息")
    beauty_print(f"  {extra_label}: 0x{extra:02X}", type="info")


def diagnostic_arm_names(func_code: int, block_count: int):
    """按响应功能码低 7 位和数据块顺序返回机械臂名称。"""
    arm_mask = func_code & 0x7F
    names = []
    if arm_mask & 0x01:
        names.append("示教臂")
    if arm_mask & 0x02:
        names.append("操作臂")
    if len(names) == block_count:
        return names
    return ["操作臂" if block_count == 1 else f"机械臂{index + 1}" for index in range(block_count)]


def format_comm_bitmap(comm_bitmap: int) -> str:
    """格式化 7 个关节的通信快照位。"""
    states = []
    for joint_index in range(DIAGNOSTIC_JOINT_COUNT):
        state = "收到" if comm_bitmap & (1 << joint_index) else "未收到"
        states.append(f"J{joint_index + 1}={state}")
    return ", ".join(states)


def format_motor_states(states: bytes) -> str:
    """格式化 J1~J7 电机状态码。"""
    return ", ".join(
        f"J{index + 1}={code_name(value, MOTOR_STATE_NAMES)}"
        for index, value in enumerate(states)
    )


def format_control_modes(modes: bytes) -> str:
    """格式化 J1~J7 控制模式。"""
    return ", ".join(
        f"J{index + 1}={code_name(value, CONTROL_MODE_NAMES)}"
        for index, value in enumerate(modes)
    )


def code_name(value: int, names) -> str:
    """返回带原始码的可读名称。"""
    return f"{names.get(value, '未知')} (0x{value:02X})"


def mode_name(mode: int) -> str:
    """返回 0xEE 模式码名称。"""
    return MODE_NAMES.get(mode, "未知状态")


def format_bytes(data: bytes) -> str:
    """格式化字节数组，便于打印 TX/RX。"""
    return " ".join(f"{byte:02X}" for byte in data)
