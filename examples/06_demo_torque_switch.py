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
Demo: Robot torque control (on/off switch)

Warning:
- Ensure no obstacles around the robot arm before operation
- When torque is disabled, manually support the robot arm to prevent it from falling
"""

import alicia_m_sdk
from alicia_m_sdk.utils.logger import logger


def main(args):
    """Execute robot torque on/off switch demo.

    :param args: Command line arguments containing port
    """
    robot = alicia_m_sdk.create_robot(
        port=args.port,
        version=args.version,
        base_link=args.base_link,
        end_link=args.end_link,
        control_aim=args.control_aim,
        control_mode=args.control_mode,
    )

    try:
        logger.warning("请用手扶住机械臂！ / Please manually hold the robot arm!")
        input("按回车关闭力矩 / Press Enter to disable torque...")
        robot.torque_control('off')
        logger.info("力矩已关闭 / Torque disabled.")

        input("按回车重新开启力矩 / Press Enter to re-enable torque...")
        robot.torque_control('on')
        logger.info("力矩已开启 / Torque re-enabled.")

    except KeyboardInterrupt:
        logger.info("\n操作已取消 / Operation cancelled")

    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        robot.disconnect()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Robot torque switch demo")

    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--version', type=str, default="v1_1", help="机械臂版本 (可选: v1_0, v1_1，默认: v1_1)")
    parser.add_argument('--base_link', type=str, default="base_link", help="基座链路名称")
    parser.add_argument('--end_link', type=str, default="link6", help="末端执行器链路名称")
    parser.add_argument('--control-aim', type=str, default='operation', choices=['teach', 'operation'],
                        help='Control aim: teach (0x01示教臂) or operation (0x02操作臂) (默认: operation)')
    parser.add_argument('--control-mode', type=str, default='pv', choices=['pv', 'pvt', 'v', 'mit', 'mit_position', 'mit_speed', 'mit_torque'],
                        help='Control mode: pv, pvt, v, mit, mit_position, mit_speed, mit_torque (默认: pv)')

    main(parser.parse_args())
