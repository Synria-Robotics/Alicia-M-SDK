import math
import importlib
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
    CMD_GRIPPER_PARAM,
    CMD_USER_SETTINGS,
    EXACT_ZERO_12BIT,
    FUNC_READ_ALL_SETTINGS,
    FUNC_WRITE_GRIPPER_TYPE,
    NUM_MOTORS,
    USER_SETTING_WRITE_ACCEPT,
    ZERO_RESET_STRONG,
    ZERO_RESET_WEAK,
)
from alicia_m_sdk.hardware.frame import Frame
from alicia_m_sdk.hardware.messages import ZeroResetRequest
from alicia_m_sdk.gripper_params import (
    GRIPPER_PARAM_BY_NAME,
    make_read_gripper_params_frame,
    make_write_gripper_params_frame,
    parse_gripper_params_response,
)
from alicia_m_sdk.types.config import RobotConfig
from alicia_m_sdk.types.state import JointState
from alicia_m_sdk.user_settings import (
    gripper_type_config_value,
    is_write_accepted,
    make_read_settings_frame,
    make_write_gripper_type_frame,
    normalize_gripper_type,
    parse_settings_response,
)
from alicia_m_sdk.utils.version import parse_firmware_version, supports_min_version
from alicia_m_sdk.utils.model_resolver import resolve_model_version
from alicia_m_sdk.utils.conversion import encode_torque, encode_velocity


class VersionHelpersTest(unittest.TestCase):
    def test_runtime_version_matches_project_version(self):
        self.assertEqual(alicia_m_sdk.__version__, "1.1.1rc1")

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

    def test_hardware_version_model_mapping(self):
        self.assertEqual(resolve_model_version("100"), "v1_1")
        self.assertEqual(resolve_model_version("101"), "v1_1")
        self.assertEqual(resolve_model_version("102"), "v1_2")
        self.assertEqual(resolve_model_version("103"), "v1_2")
        with self.assertRaises(ValueError):
            resolve_model_version("99")


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


class GripperParamsProtocolTest(unittest.TestCase):
    def test_read_all_frame_matches_protocol_example(self):
        frame = make_read_gripper_params_frame(aim="follower", mask=0)
        self.assertEqual(frame.encode().hex(" ").upper(), "AA 17 02 00 65 FF")

    def test_write_selected_torque_frame_matches_protocol_example(self):
        frame = make_write_gripper_params_frame(
            {"target_force": 35.0, "hold_torque": 2.5},
            aim="follower",
        )
        self.assertEqual(
            frame.encode().hex(" ").upper(),
            "AA 17 82 09 09 00 00 0C 42 00 00 20 40 B3 FF",
        )

    def test_write_saved_frame_matches_protocol_example(self):
        frame = make_write_gripper_params_frame(
            {"target_force": 2.0},
            aim="follower",
            save=True,
        )
        self.assertEqual(
            frame.encode().hex(" ").upper(),
            "AA 17 82 06 01 00 00 00 40 01 BD FF",
        )

    def test_robot_set_gripper_params_passes_save_flag(self):
        robot = SynriaRobotAPI(RobotConfig(auto_connect=False))
        response = Frame.decode(bytes.fromhex("AA 17 82 03 01 01 01 7C FF"))
        robot.send_gripper_param_frame = Mock(return_value=response)
        with patch("alicia_m_sdk.api.synria_robot_api.make_write_gripper_params_frame") as build_frame:
            build_frame.return_value = Frame(cmd_id=0x17, func_code=0x82, data=b"\x01\x00\x00\x00\x40\x01")
            result = robot.set_gripper_params(
                {"target_force": 2.0},
                aim="follower",
                save=True,
                gripper_type=None,
            )

        build_frame.assert_called_once_with(
            {"target_force": 2.0},
            aim="follower",
            gripper_type=None,
            save=True,
        )
        robot.send_gripper_param_frame.assert_called_once_with(
            build_frame.return_value,
            timeout=3.0,
        )
        self.assertTrue(result.write_ok)

    def test_parse_write_ack_matches_protocol_example(self):
        frame = Frame.decode(bytes.fromhex("AA 17 82 03 01 09 01 74 FF"))
        result = parse_gripper_params_response(frame)
        self.assertEqual(result.target_byte, 0x01)
        self.assertEqual(result.mask, 0x09)
        self.assertTrue(result.write_ok)
        self.assertEqual(result.values, {})

    def test_parse_read_all_response_matches_protocol_example(self):
        frame = Frame.decode(bytes.fromhex(
            "AA 17 82 22 01 FF 00 00 0C 42 00 00 A0 3F "
            "00 00 20 C0 00 00 20 40 9A 99 19 3F CD CC "
            "CC 3E 00 00 A0 41 33 33 B3 3E 8C FF"
        ))
        result = parse_gripper_params_response(frame)
        self.assertEqual(result.target_byte, 0x01)
        self.assertEqual(result.mask, 0xFF)
        self.assertAlmostEqual(result.values["target_force"], 35.0)
        self.assertAlmostEqual(result.values["open_feedforward"], 1.25)
        self.assertAlmostEqual(result.values["close_feedforward"], -2.5)
        self.assertAlmostEqual(result.values["hold_torque"], 2.5)
        self.assertAlmostEqual(result.values["force_kp"], 0.6, places=6)
        self.assertAlmostEqual(result.values["force_ki"], 0.4, places=6)
        self.assertAlmostEqual(result.values["integral_limit"], 20.0)
        self.assertAlmostEqual(result.values["close_torque_scale"], 0.35, places=6)

    def test_new_torque_aliases_are_accepted(self):
        self.assertIs(GRIPPER_PARAM_BY_NAME["open_feedforward_torque"], GRIPPER_PARAM_BY_NAME["open_feedforward"])
        self.assertIs(GRIPPER_PARAM_BY_NAME["close_feedforward_torque"], GRIPPER_PARAM_BY_NAME["close_feedforward"])
        self.assertIs(GRIPPER_PARAM_BY_NAME["max_hold_torque"], GRIPPER_PARAM_BY_NAME["hold_torque"])
        canonical = make_write_gripper_params_frame({"hold_torque": 2.5}, aim="follower")
        alias = make_write_gripper_params_frame({"max_hold_torque": 2.5}, aim="follower")
        self.assertEqual(alias.encode(), canonical.encode())

    def test_gripper_type_specific_ranges_are_enforced(self):
        make_write_gripper_params_frame({"hold_torque": 8.0}, aim="follower", gripper_type="large")
        with self.assertRaises(ValueError):
            make_write_gripper_params_frame({"hold_torque": 8.0}, aim="follower", gripper_type="small")

    def test_broad_protocol_ranges_are_enforced_without_gripper_type(self):
        make_write_gripper_params_frame({"target_force": 120.0}, aim="follower")
        with self.assertRaises(ValueError):
            make_write_gripper_params_frame({"target_force": 121.0}, aim="follower")


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

    def test_send_and_wait_pauses_polling_until_response(self):
        port = Mock()
        codec = Mock()
        device = Device(port, codec)
        request = Frame(cmd_id=CMD_GRIPPER_PARAM, func_code=0x82, data=b"\x01\x00\x00\x00\x40\x01")
        response = Frame(cmd_id=CMD_GRIPPER_PARAM, func_code=0x82, data=b"\x01\x01\x01")

        def complete_pending(_frame):
            device._state_cache.resolve_pending(CMD_GRIPPER_PARAM, response)

        device.send_frame = Mock(side_effect=complete_pending)

        with patch.object(device, "pause_polling", wraps=device.pause_polling) as pause:
            with patch.object(device, "resume_polling", wraps=device.resume_polling) as resume:
                result = device.send_and_wait(request, CMD_GRIPPER_PARAM, timeout=0.1)

        self.assertIs(result, response)
        pause.assert_called_once()
        resume.assert_called_once()
        self.assertFalse(device._poll_paused.is_set())

    def test_send_and_wait_keeps_preexisting_poll_pause(self):
        port = Mock()
        codec = Mock()
        device = Device(port, codec)
        device.pause_polling()
        request = Frame(cmd_id=CMD_GRIPPER_PARAM, func_code=0x82, data=b"\x01\x00\x00\x00\x40\x01")
        response = Frame(cmd_id=CMD_GRIPPER_PARAM, func_code=0x82, data=b"\x01\x01\x01")

        def complete_pending(_frame):
            device._state_cache.resolve_pending(CMD_GRIPPER_PARAM, response)

        device.send_frame = Mock(side_effect=complete_pending)

        with patch.object(device, "resume_polling", wraps=device.resume_polling) as resume:
            result = device.send_and_wait(request, CMD_GRIPPER_PARAM, timeout=0.1)

        self.assertIs(result, response)
        resume.assert_not_called()
        self.assertTrue(device._poll_paused.is_set())


class MessageCodecExactZeroTest(unittest.TestCase):
    def test_pv_zero_velocity_uses_exact_zero_12bit(self):
        codec = MessageCodec()
        frame = codec.encode_pv_control(
            AIM_FOLLOWER,
            [0.0] * NUM_MOTORS,
            [0.0] * NUM_MOTORS,
        )

        for motor_index in range(NUM_MOTORS):
            velocity_raw = struct.unpack_from("<H", frame.data, 2 + motor_index * 4 + 2)[0]
            self.assertEqual(velocity_raw, EXACT_ZERO_12BIT)

    def test_pv_nonzero_velocity_uses_standard_mapping(self):
        codec = MessageCodec()
        frame = codec.encode_pv_control(
            AIM_FOLLOWER,
            [0.0] * NUM_MOTORS,
            [1.0] * NUM_MOTORS,
        )

        for motor_index in range(NUM_MOTORS):
            velocity_raw = struct.unpack_from("<H", frame.data, 2 + motor_index * 4 + 2)[0]
            self.assertEqual(velocity_raw, encode_velocity(1.0))

    def test_mit_zero_velocity_and_torque_use_exact_zero_12bit(self):
        codec = MessageCodec()
        frame = codec.encode_mit_control(
            AIM_FOLLOWER,
            [0.0] * NUM_MOTORS,
            [0.0] * NUM_MOTORS,
            [0.0] * NUM_MOTORS,
            [150.0] * NUM_MOTORS,
            [2.0] * NUM_MOTORS,
            linear_velocities=[0.0] * NUM_MOTORS,
        )

        for motor_index in range(NUM_MOTORS):
            base = 2 + motor_index * 12
            velocity_raw = struct.unpack_from("<H", frame.data, base + 2)[0]
            torque_raw = struct.unpack_from("<H", frame.data, base + 4)[0]
            linear_velocity_raw = struct.unpack_from("<H", frame.data, base + 10)[0]
            self.assertEqual(velocity_raw, EXACT_ZERO_12BIT)
            self.assertEqual(torque_raw, EXACT_ZERO_12BIT)
            self.assertEqual(linear_velocity_raw, EXACT_ZERO_12BIT)

    def test_mit_nonzero_velocity_and_torque_use_standard_mapping(self):
        codec = MessageCodec()
        frame = codec.encode_mit_control(
            AIM_FOLLOWER,
            [0.0] * NUM_MOTORS,
            [1.0] * NUM_MOTORS,
            [1.0] * NUM_MOTORS,
            [150.0] * NUM_MOTORS,
            [2.0] * NUM_MOTORS,
            linear_velocities=[1.0] * NUM_MOTORS,
        )

        for motor_index in range(NUM_MOTORS):
            base = 2 + motor_index * 12
            velocity_raw = struct.unpack_from("<H", frame.data, base + 2)[0]
            torque_raw = struct.unpack_from("<H", frame.data, base + 4)[0]
            linear_velocity_raw = struct.unpack_from("<H", frame.data, base + 10)[0]
            self.assertEqual(velocity_raw, encode_velocity(1.0))
            self.assertEqual(torque_raw, encode_torque(1.0, motor_index))
            self.assertNotEqual(linear_velocity_raw, EXACT_ZERO_12BIT)

    def test_linear_velocity_zero_uses_exact_zero_12bit(self):
        codec = MessageCodec()
        frame = codec.encode_linear_velocity_control(
            AIM_FOLLOWER,
            [0.0] * NUM_MOTORS,
        )

        for motor_index in range(NUM_MOTORS):
            linear_velocity_raw = struct.unpack_from("<H", frame.data, 2 + motor_index * 2)[0]
            self.assertEqual(linear_velocity_raw, EXACT_ZERO_12BIT)


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


class RoboCoreIntegrationLayoutTest(unittest.TestCase):
    def test_root_robocore_forwards_are_not_exported(self):
        self.assertNotIn("RobotModel", alicia_m_sdk.__all__)
        self.assertFalse(hasattr(alicia_m_sdk, "forward_kinematics"))
        with self.assertRaises(ModuleNotFoundError):
            importlib.import_module("alicia_m_sdk.kinematics")
        with self.assertRaises(ModuleNotFoundError):
            importlib.import_module("alicia_m_sdk.planning")

    def test_robocore_adapters_export_from_integration_package(self):
        module = importlib.import_module("alicia_m_sdk.integrations.robocore")
        self.assertTrue(hasattr(module, "compute_forward_kinematics"))
        self.assertTrue(hasattr(module, "compute_inverse_kinematics"))
        self.assertTrue(hasattr(module, "plan_joint_trajectory"))
        self.assertTrue(hasattr(module, "plan_cartesian_trajectory"))

    def test_get_pose_uses_robocore_integration_adapter(self):
        robot = SynriaRobotAPI(RobotConfig(auto_connect=False), robot_model=object())
        robot._device._state_cache.update_joint_state(JointState(
            angles=[0.0] * 6,
            gripper=0.0,
            timestamp=0.0,
            run_status=0,
        ))
        expected = {"position": np.array([1.0, 2.0, 3.0])}
        with patch(
            "alicia_m_sdk.api.synria_robot_api.kin_module.compute_forward_kinematics",
            return_value=expected,
        ) as compute_fk:
            self.assertIs(robot.get_pose(), expected)

        compute_fk.assert_called_once_with(robot.robot_model, [0.0] * 6)

    def test_planning_methods_use_robocore_integration_adapter(self):
        robot = SynriaRobotAPI(RobotConfig(auto_connect=False))
        joint_result = {"success": True, "kind": "joint"}
        cart_result = {"success": True, "kind": "cartesian"}
        with patch(
            "alicia_m_sdk.api.synria_robot_api.plan_module.plan_joint_trajectory",
            return_value=joint_result,
        ) as plan_joint:
            self.assertIs(robot.plan_joint_trajectory([[0.0] * 6, [0.1] * 6]), joint_result)
        with patch(
            "alicia_m_sdk.api.synria_robot_api.plan_module.plan_cartesian_trajectory",
            return_value=cart_result,
        ) as plan_cartesian:
            self.assertIs(robot.plan_cartesian_trajectory([[0.0] * 7, [0.1] * 7]), cart_result)

        plan_joint.assert_called_once()
        plan_cartesian.assert_called_once()


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
