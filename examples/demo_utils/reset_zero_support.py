"""零位标定 demo 的辅助逻辑。

入口脚本只负责交互流程；版本判断、状态打印和调零协议细节放在这里。
"""

import time

from alicia_m_sdk.hardware.constants import NUM_MOTORS, ZERO_RESET_WEAK
from alicia_m_sdk.hardware.messages import ZeroResetRequest


PRINT_INTERVAL = 0.2
MIN_WEAK_ZERO_VERSION = (1, 0, 6)


def send_strong_zero_position(robot):
    """发送强调零指令。"""
    robot.set_zero_position()


def send_weak_zero_position(robot):
    """发送弱调零指令；调用前必须先确认固件版本 >= 1.0.6。"""
    device = robot._device
    frame = device.codec.encode_zero_reset(ZeroResetRequest(
        aim=device.aim,
        start_joint=0,
        joint_count=NUM_MOTORS,
        reset_mode=ZERO_RESET_WEAK,
    ))
    device.send_frame(frame)
    time.sleep(0.1)


def supports_weak_zero(version):
    """判断固件是否支持弱调零。"""
    parsed = parse_firmware_version(version)
    return parsed is not None and parsed >= MIN_WEAK_ZERO_VERSION


def parse_firmware_version(version):
    """解析固件整数版本号，例如 106 表示 1.0.6。"""
    if not version:
        return None
    text = str(version).strip()
    if not text.isdigit():
        return None
    value = int(text)
    return value // 100, (value // 10) % 10, value % 10


def print_robot_state(robot):
    """打印当前关节位置、速度和力矩。"""
    state = robot.get_robot_state("all")
    if state is None:
        print("pos=N/A vel=N/A tor=N/A", flush=True)
        return

    pos = list(state.angles) + [state.gripper]
    print(
        f"pos={format_values(pos)} "
        f"vel={format_values(state.velocities)} "
        f"tor={format_values(state.torques)}",
        flush=True,
    )


def format_values(values, precision=4):
    """格式化状态数组，便于连续打印。"""
    if values is None:
        return "N/A"
    return "[" + ", ".join(f"{value:.{precision}f}" for value in values) + "]"
