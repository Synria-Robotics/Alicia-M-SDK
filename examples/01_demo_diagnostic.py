"""01_demo_diagnostic.py — 自检功能

演示自检功能的调用方式。
注意: 底层自检功能尚未更新，当前以占位形式实现。
"""

import argparse

import alicia_m_sdk
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from demo_utils.demo_common import add_port_argument
from alicia_m_sdk.utils.beauty_logger import beauty_print, beauty_print_array


def main():
    beauty_print("Demo: 自检功能（底层待更新）", type="module")

    parser = argparse.ArgumentParser(description="Run Alicia-M diagnostic demo.")
    add_port_argument(parser)
    args = parser.parse_args()

    # 创建并连接机器人
    robot = alicia_m_sdk.create_robot(port=args.port)
    beauty_print("机器人连接成功", type="success")

    try:
        # --- 执行自检 ---
        beauty_print("正在执行自检...", type="info")
        self_check = robot.get_robot_state("self_check")

        if self_check is not None:
            beauty_print("自检结果:", type="info")
            # 自检返回的数据结构取决于固件实现
            # 预期包含各关节电机健康状态
            if isinstance(self_check, dict):
                for key, value in self_check.items():
                    beauty_print(f"  {key}: {value}", type="info")
            else:
                beauty_print(f"  原始数据: {self_check}", type="info")
            beauty_print("自检完成", type="success")
        else:
            beauty_print("自检功能暂未实现（底层待更新）", type="warning")
            beauty_print("预留接口: robot.get_robot_state('self_check')", type="info")

    except KeyboardInterrupt:
        beauty_print("\n用户中断", type="warning")
    finally:
        robot.disconnect()
        beauty_print("已断开连接", type="info")


if __name__ == "__main__":
    main()
