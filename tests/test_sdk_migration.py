import math
import struct
import unittest
from unittest.mock import Mock, patch

import numpy as np

import alicia_m_sdk
from alicia_m_sdk.api.synria_robot_api import SynriaRobotAPI
from alicia_m_sdk.execution.trajectory_executor import TrajectoryExecutor
from alicia_m_sdk.hardware.device import Device
from alicia_m_sdk.types.enums import GripperType

from alicia_m_sdk.diagnostics import (
    DIAGNOSTIC_BLOCK_LEN,
    DIAGNOSTIC_REQUEST,
    parse_diagnostic_response,
    supports_diagnostic,
)
from alicia_m_sdk.execution.joint_mapping import (
    convert_joints_deg_from_alicia_d_to_alicia_m,
    convert_joints_rad_from_alicia_d_to_alicia_m,
)
from alicia_m_sdk.hardware.codec import MessageCodec
from alicia_m_sdk.hardware.constants import (
    AIM_FOLLOWER,
    CMD_DIAGNOSTIC,
    CMD_ERROR,
    CMD_USER_SETTINGS,
    FUNC_READ_ALL_SETTINGS,
    FUNC_WRITE_GRIPPER_TYPE,
    NUM_MOTORS,
    USER_SETTING_WRITE_ACCEPT,
    ZERO_RESET_STRONG,
    ZERO_RESET_WEAK,
)
from alicia_m_sdk.hardware.frame import Frame
from alicia_m_sdk.hardware.messages import ZeroResetRequest
from alicia_m_sdk.types.config import RobotConfig
from alicia_m_sdk.user_settings import (
    gripper_type_config_value,
    is_write_accepted,
    make_read_settings_frame,
    make_write_gripper_type_frame,
    normalize_gripper_type,
    parse_settings_response,
)
from alicia_m_sdk.utils.version import parse_firmware_version, supports_min_version


class VersionHelpersTest(unittest.TestCase):
    def test_runtime_version_matches_project_version(self):
        self.assertEqual(alicia_m_sdk.__version__, "1.1.0")

    def test_parse_supported_version_formats(self):
        self.assertEqual(parse_firmware_version(106), (1, 0, 6))
        self.assertEqual(parse_firmware_version("106"), (1, 0, 6))
        self.assertEqual(parse_firmware_version("1.0.6"), (1, 0, 6))
        self.assertEqual(parse_firmware_version("v1.0.6"), (1, 0, 6))
        self.assertEqual(parse_firmware_version("v1.1.0"), (1, 1, 0))
        self.assertIsNone(parse_firmware_version("bad"))

    def test_min_version_checks(self):
        self.assertTrue(supports_min_version("v1.1.0", (1, 0, 6)))
        self.assertTrue(supports_diagnostic("1.0.6"))
        self.assertTrue(supports_diagnostic("1.1.0"))
        self.assertFalse(supports_diagnostic("1.0.5"))


class DiagnosticParsingTest(unittest.TestCase):
    def test_diagnostic_request_matches_protocol_frame(self):
        self.assertEqual(
            DIAGNOSTIC_REQUEST.encode().hex(" ").upper(),
            "AA FE 02 01 FE BC FF",
        )

    def test_parse_single_arm_response(self):
        block = bytes([0x7F]) + bytes([0, 1, 4, 8, 9, 0xA, 0xE]) + bytes([1, 2, 3, 4, 0, 1, 2])
        frame = Frame(cmd_id=CMD_DIAGNOSTIC, func_code=AIM_FOLLOWER, data=block)
        result = parse_diagnostic_response(frame)
        self.assertTrue(result.ok)
        self.assertEqual(len(result.snapshots), 1)
        self.assertEqual(result.snapshots[0].arm_name, "操作臂")
        self.assertEqual(result.snapshots[0].comm_bitmap, 0x7F)
        self.assertEqual(result.snapshots[0].motor_states[2], 4)
        self.assertEqual(result.snapshots[0].control_modes[1], 2)

    def test_parse_double_arm_response(self):
        block = bytes([0x01]) + bytes(range(1, DIAGNOSTIC_BLOCK_LEN))
        frame = Frame(cmd_id=CMD_DIAGNOSTIC, func_code=0x03, data=block + block)
        result = parse_diagnostic_response(frame)
        self.assertTrue(result.ok)
        self.assertEqual([snapshot.arm_name for snapshot in result.snapshots], ["示教臂", "操作臂"])

    def test_parse_error_response(self):
        frame = Frame(cmd_id=CMD_ERROR, func_code=0x07, data=bytes([0xFE]))
        result = parse_diagnostic_response(frame)
        self.assertFalse(result.ok)
        self.assertEqual(result.error_code, 0x07)
        self.assertEqual(result.error_data, bytes([0xFE]))

    def test_invalid_diagnostic_length_raises(self):
        frame = Frame(cmd_id=CMD_DIAGNOSTIC, func_code=AIM_FOLLOWER, data=b"\x00")
        with self.assertRaises(ValueError):
            parse_diagnostic_response(frame)


class UserSettingsTest(unittest.TestCase):
    def test_settings_frame_builders(self):
        self.assertEqual(
            make_read_settings_frame(),
            Frame(cmd_id=CMD_USER_SETTINGS, func_code=FUNC_READ_ALL_SETTINGS),
        )
        self.assertEqual(
            make_write_gripper_type_frame(40),
            Frame(
                cmd_id=CMD_USER_SETTINGS,
                func_code=FUNC_WRITE_GRIPPER_TYPE,
                data=struct.pack("<I", 2),
            ),
        )

    def test_parse_settings_response(self):
        data = (1).to_bytes(4, "little") + (2).to_bytes(4, "little") + (0).to_bytes(4, "little")
        frame = Frame(cmd_id=CMD_USER_SETTINGS, func_code=FUNC_READ_ALL_SETTINGS, data=data)
        settings = parse_settings_response(frame)
        self.assertEqual(settings.values, [1, 2, 0])
        self.assertEqual(settings.startup_action, 1)
        self.assertEqual(settings.gripper_type, 2)
        self.assertEqual(settings.timed_upload_enabled, 0)

    def test_gripper_type_helpers(self):
        self.assertEqual(normalize_gripper_type("10"), 0)
        self.assertEqual(normalize_gripper_type("40"), 2)
        self.assertEqual(normalize_gripper_type("large"), 2)
        self.assertEqual(normalize_gripper_type(GripperType.MM_100), 2)
        self.assertEqual(gripper_type_config_value(6), 2)
        self.assertEqual(gripper_type_config_value(4), 0)

    def test_gripper_type_enum_metadata(self):
        self.assertEqual(GripperType.MM_50.value, "50mm")
        self.assertEqual(GripperType.MM_50.firmware_value, 0)
        self.assertEqual(GripperType.MM_100.firmware_value, 2)
        self.assertEqual(GripperType.MM_50.option_value, 10)
        self.assertEqual(GripperType.MM_100.option_value, 40)
        self.assertIs(GripperType.parse("100mm"), GripperType.MM_100)
        self.assertIs(GripperType.parse(10), GripperType.MM_50)

    def test_advanced_api_exports_are_available(self):
        self.assertIs(alicia_m_sdk.JointController, __import__("alicia_m_sdk.execution").execution.JointController)
        self.assertIs(alicia_m_sdk.Teleoperation, __import__("alicia_m_sdk.execution").execution.Teleoperation)

    def test_write_response_acceptance(self):
        accepted = Frame(
            cmd_id=CMD_USER_SETTINGS,
            func_code=FUNC_WRITE_GRIPPER_TYPE,
            data=bytes([USER_SETTING_WRITE_ACCEPT]),
        )
        rejected = Frame(
            cmd_id=CMD_USER_SETTINGS,
            func_code=FUNC_WRITE_GRIPPER_TYPE,
            data=b"\x00",
        )
        self.assertTrue(is_write_accepted(accepted))
        self.assertFalse(is_write_accepted(rejected))


class DeviceStatusTest(unittest.TestCase):
    def test_follower_status_does_not_report_button_events(self):
        status = Device._parse_run_status(0xC3, device_type="F")
        self.assertTrue(status.is_locked)
        self.assertTrue(status.is_synced)
        self.assertTrue(status.has_motor_error)
        self.assertTrue(status.gripper_torque_locked)
        self.assertFalse(status.single_click)
        self.assertFalse(status.double_click)
        self.assertFalse(status.long_press)

    def test_leader_status_reports_button_events(self):
        status = Device._parse_run_status(0x07, device_type="L")
        self.assertFalse(status.is_locked)
        self.assertFalse(status.is_synced)
        self.assertTrue(status.single_click)
        self.assertTrue(status.double_click)
        self.assertTrue(status.long_press)


class TrajectoryExecutorTest(unittest.TestCase):
    def test_execute_pv_uses_supplied_positions_and_velocities(self):
        device = Mock()
        device.aim = AIM_FOLLOWER
        device.joint_state = None
        executor = TrajectoryExecutor(device)
        timestamps = np.array([0.0])
        positions = np.array([[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 700.0]])
        velocities = np.array([[1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 25.0]])

        self.assertTrue(executor.execute_pv(timestamps, positions, velocities))

        device.send_pv.assert_called_once_with(
            AIM_FOLLOWER,
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 700.0],
            [1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 25.0],
        )


class DeviceCommandTest(unittest.TestCase):
    def test_send_linear_velocity_uses_codec_frame(self):
        port = Mock()
        codec = Mock()
        frame = Frame(cmd_id=0x06, func_code=AIM_FOLLOWER, data=b"frame")
        codec.encode_linear_velocity_control.return_value = frame
        device = Device(port, codec)
        device.send_frame = Mock()

        velocities = [1.0] * NUM_MOTORS
        device.send_linear_velocity(AIM_FOLLOWER, velocities)

        codec.encode_linear_velocity_control.assert_called_once_with(AIM_FOLLOWER, velocities)
        device.send_frame.assert_called_once_with(frame)


class SynriaRobotAPILifecycleTest(unittest.TestCase):
    def test_context_manager_connects_and_disconnects(self):
        with patch.object(SynriaRobotAPI, "connect", return_value=True) as connect:
            robot = SynriaRobotAPI(RobotConfig(auto_connect=False))
            robot.disconnect = Mock()
            robot.is_connected = Mock(return_value=False)

            with robot as managed:
                self.assertIs(managed, robot)

            connect.assert_called_once()
            robot.disconnect.assert_called_once()

    def test_context_manager_does_not_reconnect_existing_connection(self):
        with patch.object(SynriaRobotAPI, "connect", return_value=True) as connect:
            robot = SynriaRobotAPI(RobotConfig(auto_connect=False))
            robot.disconnect = Mock()
            robot.is_connected = Mock(return_value=True)

            with robot:
                pass

            connect.assert_not_called()
            robot.disconnect.assert_called_once()


class JointMappingTest(unittest.TestCase):
    def test_zero_mapping_and_reversed_joints(self):
        mapped = convert_joints_deg_from_alicia_d_to_alicia_m([0, 0, 0, 10, 0, 10])
        self.assertEqual(mapped[0], 0)
        self.assertAlmostEqual(mapped[1], -89.95)
        self.assertAlmostEqual(mapped[3], -10)
        self.assertAlmostEqual(mapped[5], -10)

    def test_proportional_joint_and_clamping(self):
        mapped = convert_joints_deg_from_alicia_d_to_alicia_m([999, 0, 999, 0, 0, 0])
        self.assertEqual(mapped[0], 157.5)
        self.assertEqual(mapped[2], -179.9)

    def test_rad_mapping(self):
        mapped = convert_joints_rad_from_alicia_d_to_alicia_m([0, 0, 0, 0, 0, 0])
        self.assertEqual(len(mapped), 6)
        self.assertAlmostEqual(mapped[1], math.radians(-89.95))


class ZeroResetFrameTest(unittest.TestCase):
    def test_zero_reset_modes_encode_expected_data(self):
        codec = MessageCodec()
        strong = codec.encode_zero_reset(ZeroResetRequest(
            aim=AIM_FOLLOWER,
            start_joint=0,
            joint_count=NUM_MOTORS,
            reset_mode=ZERO_RESET_STRONG,
        ))
        weak = codec.encode_zero_reset(ZeroResetRequest(
            aim=AIM_FOLLOWER,
            start_joint=0,
            joint_count=NUM_MOTORS,
            reset_mode=ZERO_RESET_WEAK,
        ))
        self.assertEqual(strong.data, bytes([0, NUM_MOTORS, ZERO_RESET_STRONG]))
        self.assertEqual(weak.data, bytes([0, NUM_MOTORS, ZERO_RESET_WEAK]))


if __name__ == "__main__":
    unittest.main()
