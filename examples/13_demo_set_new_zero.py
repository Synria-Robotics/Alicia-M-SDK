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
Demo: Set new zero configuration (zero calibration)

Warning:
- Ensure no obstacles around the robot arm before calibration
- When torque is disabled, manually support the robot arm to prevent it from falling
- 一旦设置新零点，无法恢复至出厂零点！
  Once the new zero point is set, it CANNOT be restored to the factory zero point!
  If you want to restore it, you need to purchase the calibration tool!
"""

import alicia_m_sdk
from alicia_m_sdk.utils.logger import logger


def main(args):
    """Execute zero calibration demo.

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
        logger.warning("此操作将更改出厂零点位置，请谨慎操作！")
        logger.warning("WARNING: This will permanently change the factory zero position!")
        logger.warning("请用手扶住机械臂！ / Please manually hold the robot arm!")

        input("按回车关闭力矩（然后手动拖动机械臂到零点位置）/ Press Enter to disable torque...")
        robot.torque_control('off')
        logger.info("力矩已关闭，请将机械臂拖动到目标零点位置 / Torque disabled, drag arm to desired zero position")

        input("按回车开启力矩并设置当前位置为新零点 / Press Enter to enable torque and set new zero...")
        robot.torque_control('on')
        logger.info("力矩已开启 / Torque re-enabled.")

        result = robot.servo_driver.set_zero_position()
        if result:
            logger.info("零点校准成功 / Zero calibration successful!")
        else:
            logger.error("零点校准失败 / Zero calibration failed!")

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
    parser = argparse.ArgumentParser(description="Robot zero calibration demo")

    parser.add_argument('--port', type=str, default="", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--version', type=str, default="v1_1", help="机械臂版本 (可选: v1_0, v1_1，默认: v1_1)")
    parser.add_argument('--base_link', type=str, default="base_link", help="基座链路名称")
    parser.add_argument('--end_link', type=str, default="link6", help="末端执行器链路名称")
    parser.add_argument('--control-aim', type=str, default='operation', choices=['teach', 'operation'],
                        help='Control aim: teach (0x01示教臂) or operation (0x02操作臂) (默认: operation)')
    parser.add_argument('--control-mode', type=str, default='pv', choices=['pv', 'mit'],
                        help='Control mode: pv or mit (默认: pv)')

    main(parser.parse_args())
