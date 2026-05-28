import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

import numpy as np

from scripts.sop_void_grip_test import (
    SopAction,
    SopSafetyError,
    VoidGripSopConfig,
    VoidGripSopRunner,
    parse_urdf_revolute_limits,
)


class ModeValue:
    def __init__(self, value="pv"):
        self.value = value


class FakeStatus:
    has_motor_error = False


class FakeState:
    run_status = 0


class FakeRobot:
    def __init__(self, mode="pv", ik_success=True, execute_success=True):
        self.control_mode = ModeValue(mode)
        self.robot_model = object()
        self.pose = np.eye(4, dtype=float)
        self.ik_success = ik_success
        self.execute_success = execute_success
        self.switch_mode = Mock(return_value=True)
        self.initialize_mit_gains = Mock(return_value=True)
        self.disable_robot = Mock(return_value=True)
        self.disconnect = Mock()
        self.set_gripper_target = Mock(return_value=True)
        self.set_robot_state = Mock(return_value=True)
        self.set_pose_calls = []

    def get_pose(self):
        return {"transform": self.pose.copy()}

    def get_robot_state(self, info_type="all"):
        if info_type == "all":
            return FakeState()
        if info_type == "status":
            return FakeStatus()
        return None

    def set_pose(self, target, execute=True, **kwargs):
        self.set_pose_calls.append({"target": np.asarray(target).copy(), "execute": execute})
        if not execute:
            return {
                "success": self.ik_success,
                "q": np.array([-0.1, -1.0, -1.0, 0.0, 0.0, 0.0], dtype=float),
                "residual": 0.0,
            }
        if self.execute_success:
            self.pose = np.asarray(target).copy()
        return {"success": True, "motion_executed": self.execute_success}


class VoidGripSopTest(unittest.TestCase):
    def make_config(self, **kwargs):
        kwargs.setdefault("log_dir", tempfile.mkdtemp())
        return VoidGripSopConfig(**kwargs)

    def test_dry_run_does_not_connect(self):
        factory = Mock(return_value=FakeRobot())
        config = self.make_config(dry_run=True, port="COM37")

        code = VoidGripSopRunner(config, robot_factory=factory, sleeper=lambda _: None).run()

        self.assertEqual(code, 0)
        factory.assert_not_called()

    def test_default_does_not_require_fixed_urdf_path(self):
        robot = FakeRobot()
        factory = Mock(return_value=robot)
        config = self.make_config(step_mm=1000.0)

        code = VoidGripSopRunner(config, robot_factory=factory, sleeper=lambda _: None).run()

        self.assertEqual(code, 0)
        factory.assert_called_once()
        robot.set_robot_state.assert_called()

    def test_explicit_missing_urdf_path_fails(self):
        robot = FakeRobot()
        factory = Mock(return_value=robot)
        missing = str(Path(tempfile.gettempdir()) / "missing_alicia_m.urdf")
        config = self.make_config(urdf_path=missing)

        code = VoidGripSopRunner(config, robot_factory=factory, sleeper=lambda _: None).run()

        self.assertEqual(code, 1)
        robot.disable_robot.assert_called()
        robot.disconnect.assert_called()

    def test_y_800mm_splits_into_16_segments_at_50mm(self):
        robot = FakeRobot()
        runner = VoidGripSopRunner(self.make_config(step_mm=50.0), sleeper=lambda _: None)
        runner.robot = robot

        runner._run_translation(
            SopAction(3, "末端 Y 移动 +800mm", "translate", axis="y", distance_mm=800.0)
        )

        execute_calls = [call for call in robot.set_pose_calls if call["execute"]]
        precheck_calls = [call for call in robot.set_pose_calls if not call["execute"]]
        self.assertEqual(len(precheck_calls), 16)
        self.assertEqual(len(execute_calls), 16)
        self.assertAlmostEqual(execute_calls[-1]["target"][1, 3], 0.8)

    def test_ik_failure_does_not_execute_motion(self):
        robot = FakeRobot(ik_success=False)
        runner = VoidGripSopRunner(self.make_config(step_mm=50.0), sleeper=lambda _: None)
        runner.robot = robot

        with self.assertRaises(SopSafetyError):
            runner._run_translation(
                SopAction(2, "末端 X 移动 +150mm", "translate", axis="x", distance_mm=150.0)
            )

        self.assertTrue(robot.set_pose_calls)
        self.assertTrue(all(not call["execute"] for call in robot.set_pose_calls))

    def test_execute_failure_stops_and_shutdown_runs(self):
        robot = FakeRobot(execute_success=False)
        factory = Mock(return_value=robot)
        config = self.make_config(step_mm=1000.0)

        code = VoidGripSopRunner(config, robot_factory=factory, sleeper=lambda _: None).run()

        self.assertEqual(code, 1)
        robot.disable_robot.assert_called()
        robot.disconnect.assert_called()

    def test_mit_mode_switches_and_initializes_gains(self):
        robot = FakeRobot(mode="pv")
        factory = Mock(return_value=robot)
        config = self.make_config(mode="mit", step_mm=1000.0)

        code = VoidGripSopRunner(config, robot_factory=factory, sleeper=lambda _: None).run()

        self.assertEqual(code, 0)
        robot.switch_mode.assert_called_once_with("mit")
        robot.initialize_mit_gains.assert_called_once()
        init_kwargs = robot.set_robot_state.call_args.kwargs
        self.assertTrue(init_kwargs["use_interpolation"])

    def test_mit_initial_pose_can_disable_interpolation(self):
        robot = FakeRobot(mode="mit")
        factory = Mock(return_value=robot)
        config = self.make_config(mode="mit", mit_disable_interpolation=True, step_mm=1000.0)

        code = VoidGripSopRunner(config, robot_factory=factory, sleeper=lambda _: None).run()

        self.assertEqual(code, 0)
        init_kwargs = robot.set_robot_state.call_args.kwargs
        self.assertFalse(init_kwargs["use_interpolation"])

    def test_skip_initial_pose_does_not_move_joints_first(self):
        robot = FakeRobot()
        factory = Mock(return_value=robot)
        config = self.make_config(skip_initial_pose=True, step_mm=1000.0)

        code = VoidGripSopRunner(config, robot_factory=factory, sleeper=lambda _: None).run()

        self.assertEqual(code, 0)
        robot.set_robot_state.assert_not_called()

    def test_summary_log_is_created_with_final_data(self):
        robot = FakeRobot()
        factory = Mock(return_value=robot)
        log_dir = Path(tempfile.mkdtemp()) / "nested" / "log"
        config = self.make_config(dry_run=True, port="COM37", log_dir=str(log_dir))

        code = VoidGripSopRunner(config, robot_factory=factory, sleeper=lambda _: None).run()

        self.assertEqual(code, 0)
        logs = list(log_dir.glob("*.json"))
        self.assertEqual(len(logs), 1)
        data = json.loads(logs[0].read_text(encoding="utf-8"))
        self.assertEqual(data["status"], "PASS")
        self.assertEqual(data["port"], "COM37")
        self.assertTrue(data["dry_run"])
        self.assertEqual(data["initial_joints_deg"], [0.0, -90.0, -45.0, 0.0, 45.0, 0.0])
        self.assertEqual(data["actions_total"], 13)
        self.assertEqual(data["ik_prechecks"], 0)

    def test_parse_urdf_revolute_limits_reads_first_six_joints(self):
        urdf = ["<robot name='test'>"]
        for index in range(6):
            urdf.append(
                f"<joint name='Joint{index + 1}' type='revolute'>"
                f"<limit lower='-{index + 1}' upper='{index + 1}' effort='1' velocity='1'/>"
                "</joint>"
            )
        urdf.append("</robot>")
        path = Path(tempfile.mkdtemp()) / "test.urdf"
        path.write_text("".join(urdf), encoding="utf-8")

        limits = parse_urdf_revolute_limits(path)

        self.assertEqual(limits[0], (-1.0, 1.0))
        self.assertEqual(limits[-1], (-6.0, 6.0))


if __name__ == "__main__":
    unittest.main()
