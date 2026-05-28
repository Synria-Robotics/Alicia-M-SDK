import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

from alicia_m_sdk.diagnostics import DiagnosticArmSnapshot, DiagnosticResult
from alicia_m_sdk.hardware.frame import Frame
from alicia_m_sdk.sop import FAIL, PASS, SopConfig, SopRunner
from alicia_m_sdk.types.enums import ControlMode
from alicia_m_sdk.types.state import JointState, RobotStatus, VersionInfo


def make_state(
    angles=None,
    gripper=0.0,
    velocities=None,
    torques=None,
    temperatures=None,
):
    return JointState(
        angles=angles if angles is not None else [0.0] * 6,
        gripper=gripper,
        timestamp=0.0,
        run_status=0,
        velocities=velocities,
        torques=torques,
        temperatures=temperatures,
    )


def make_diagnostic(ok=True):
    if not ok:
        return DiagnosticResult(frame=None, snapshots=[])
    return DiagnosticResult(
        frame=Frame(cmd_id=0xFE, func_code=0x02, data=b"\x00" * 15),
        snapshots=[
            DiagnosticArmSnapshot(
                arm_name="follower",
                comm_bitmap=0x7F,
                motor_states=[0x1] * 7,
                control_modes=[0x2] * 7,
            )
        ],
    )


class FakeRobot:
    def __init__(self, state=None, diagnostic=None, status=None):
        self.connected_port = "COM37"
        self.control_mode = ControlMode.PV
        self._state = state if state is not None else make_state()
        self._diagnostic = diagnostic if diagnostic is not None else make_diagnostic()
        self._status = status
        self.disconnect = Mock()
        self.enable_robot = Mock(return_value=True)
        self.disable_robot = Mock(return_value=True)
        self.switch_mode = Mock(return_value=True)
        self.go_home = Mock(return_value=True)

    def get_robot_state(self, info_type="all"):
        if info_type == "all":
            return self._state
        if info_type == "version":
            return VersionInfo(
                serial_number="SN-FW",
                hardware_version="v1.0.0",
                firmware_version="v1.0.6",
                product_type="AM",
                device_type="F",
            )
        if info_type == "status":
            return self._status
        if info_type == "control_mode":
            return [{"value": 2, "name": "pv"}] * 7
        return None

    def run_diagnostic(self, timeout=3.0):
        return self._diagnostic

    def set_robot_state(self, target_joints, joint_format="deg", **kwargs):
        if joint_format == "deg":
            import math

            self._state.angles = [math.radians(value) for value in target_joints]
        else:
            self._state.angles = list(target_joints)
        return True

    def set_gripper_target(self, value=None, **kwargs):
        self._state.gripper = value
        return True


class SopRunnerTest(unittest.TestCase):
    def run_with_robot(self, robot, **kwargs):
        output_dir = tempfile.mkdtemp()
        config = SopConfig(
            output_dir=output_dir,
            serial="AM-TEST",
            zero_gripper=0.0,
            **kwargs,
        )
        return SopRunner(config, robot_factory=Mock(return_value=robot)).run()

    def test_quick_profile_passes_and_writes_reports(self):
        report = self.run_with_robot(FakeRobot())

        self.assertEqual(report.status, PASS)
        self.assertTrue(report.report_paths["json"].endswith(".json"))
        self.assertTrue(Path(report.report_paths["json"]).exists())
        self.assertTrue(Path(report.report_paths["csv"]).exists())
        self.assertTrue(Path(report.report_paths["markdown"]).exists())
        self.assertEqual(report.steps[0].name, "safety_config_check")
        self.assertEqual(report.steps[-1].name, "safe_shutdown_disable")

    def test_connection_failure_generates_fail_report(self):
        output_dir = tempfile.mkdtemp()
        factory = Mock(side_effect=RuntimeError("no device"))
        report = SopRunner(SopConfig(output_dir=output_dir), robot_factory=factory).run()

        self.assertEqual(report.status, FAIL)
        self.assertEqual(report.steps[1].name, "connect_and_identify")
        self.assertIn("no device", report.steps[1].error)

    def test_zero_position_failure_short_circuits_motion(self):
        robot = FakeRobot(state=make_state(angles=[0.1, 0.0, 0.0, 0.0, 0.0, 0.0]))
        report = self.run_with_robot(robot)

        self.assertEqual(report.status, FAIL)
        self.assertEqual(
            [step.name for step in report.steps],
            ["safety_config_check", "connect_and_identify", "zero_position_check", "safe_shutdown_disable"],
        )
        self.assertEqual(report.steps[2].status, FAIL)
        self.assertTrue(report.steps[2].metrics["failures"])

    def test_diagnostic_failure_generates_fail(self):
        robot = FakeRobot(diagnostic=make_diagnostic(ok=False))
        report = self.run_with_robot(robot)

        self.assertEqual(report.status, FAIL)
        self.assertEqual(report.steps[-2].name, "firmware_diagnostic")

    def test_motion_failure_generates_fail(self):
        robot = FakeRobot()
        robot.set_robot_state = Mock(return_value=False)
        report = self.run_with_robot(robot)

        self.assertEqual(report.status, FAIL)
        self.assertEqual(report.steps[-2].name, "small_range_motion_check")

    def test_profile_aging_parses_and_runs_cycles(self):
        report = self.run_with_robot(
            FakeRobot(state=make_state(temperatures=[30.0] * 7)),
            profile="aging",
            aging_cycles=2,
            aging_interval_s=0.0,
        )

        self.assertEqual(report.status, PASS)
        self.assertEqual(report.steps[-2].name, "aging_cycle_check")
        self.assertEqual(report.steps[-2].metrics["completed_cycles"], 2)

    def test_unsafe_speed_fails_before_connecting(self):
        output_dir = tempfile.mkdtemp()
        factory = Mock(return_value=FakeRobot())
        report = SopRunner(
            SopConfig(output_dir=output_dir, speed=99.0, max_safe_speed=40.0),
            robot_factory=factory,
        ).run()

        self.assertEqual(report.status, FAIL)
        self.assertEqual([step.name for step in report.steps], ["safety_config_check"])
        factory.assert_not_called()

    def test_pre_motion_motor_error_stops_before_enable(self):
        robot = FakeRobot(status=RobotStatus(has_motor_error=True))
        report = self.run_with_robot(robot)

        self.assertEqual(report.status, FAIL)
        self.assertEqual(report.steps[-2].name, "pre_motion_safety_gate")
        robot.enable_robot.assert_not_called()


if __name__ == "__main__":
    unittest.main()
