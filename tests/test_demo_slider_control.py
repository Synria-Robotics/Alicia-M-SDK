import importlib.util
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import Mock


def load_demo_module():
    root = Path(__file__).resolve().parents[1]
    examples_dir = root / "examples"
    if str(examples_dir) not in sys.path:
        sys.path.insert(0, str(examples_dir))
    module_path = root / "examples" / "19_demo_slider_control.py"
    spec = importlib.util.spec_from_file_location("demo_slider_control", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


demo = load_demo_module()


class SliderControlHelpersTest(unittest.TestCase):
    def test_parse_urdf_revolute_limits_reads_first_six_joints(self):
        urdf = ["<robot name='test'>"]
        urdf.append("<joint name='fixed' type='fixed'/>")
        for index in range(7):
            urdf.append(
                f"<joint name='Joint{index + 1}' type='revolute'>"
                f"<limit lower='-{index + 1}' upper='{index + 1}' effort='1' velocity='1'/>"
                "</joint>"
            )
        urdf.append("</robot>")
        path = Path(tempfile.mkdtemp()) / "test.urdf"
        path.write_text("".join(urdf), encoding="utf-8")

        limits = demo.parse_urdf_revolute_limits(path)

        self.assertEqual(len(limits), 6)
        self.assertEqual(limits[0], (-1.0, 1.0))
        self.assertEqual(limits[-1], (-6.0, 6.0))

    def test_parse_urdf_revolute_limits_rejects_missing_file(self):
        missing = Path(tempfile.gettempdir()) / "missing_alicia_m_test.urdf"

        with self.assertRaises(FileNotFoundError):
            demo.parse_urdf_revolute_limits(missing)

    def test_parse_urdf_revolute_limits_rejects_missing_limit_bounds(self):
        path = Path(tempfile.mkdtemp()) / "bad.urdf"
        path.write_text(
            "<robot name='test'>"
            "<joint name='Joint1' type='revolute'><limit lower='-1'/></joint>"
            "</robot>",
            encoding="utf-8",
        )

        with self.assertRaisesRegex(ValueError, "missing lower/upper"):
            demo.parse_urdf_revolute_limits(path)

    def test_parse_urdf_revolute_limits_rejects_too_few_revolute_joints(self):
        path = Path(tempfile.mkdtemp()) / "short.urdf"
        path.write_text(
            "<robot name='test'>"
            "<joint name='Joint1' type='revolute'><limit lower='-1' upper='1'/></joint>"
            "</robot>",
            encoding="utf-8",
        )

        with self.assertRaisesRegex(ValueError, "expected at least 6"):
            demo.parse_urdf_revolute_limits(path)

    def test_clamp_joints_deg_clips_each_joint(self):
        values = [-200.0, -20.0, 0.0, 3.0, 40.0, 250.0]
        limits = [(-100.0, 100.0), (-10.0, 10.0), (-1.0, 1.0), (0.0, 5.0), (0.0, 30.0), (-180.0, 180.0)]

        clipped = demo.clamp_joints_deg(values, limits)

        self.assertEqual(clipped, [-100.0, -10.0, 0.0, 3.0, 30.0, 180.0])

    def test_rad_limits_to_deg_converts_six_limits(self):
        limits = [(-demo.math.pi, demo.math.pi)] * 6

        converted = demo.rad_limits_to_deg(limits)

        self.assertEqual(converted, [(-180.0, 180.0)] * 6)

    def test_slider_window_builds_widgets_and_flushes_latest_targets(self):
        original_tkinter = sys.modules.get("tkinter")
        fake_tkinter = build_fake_tkinter()
        sys.modules["tkinter"] = fake_tkinter
        try:
            root = FakeRoot()
            robot = Mock()
            robot.set_robot_state.return_value = True
            robot.disconnect.return_value = None
            window = demo.SliderControlWindow(
                root=root,
                robot=robot,
                initial_joints_deg=[0.0, -10.0, -20.0, 0.0, 0.0, 0.0],
                initial_gripper=500.0,
                joint_limits_deg=[(-180.0, 180.0)] * 6,
                speed=15.0,
                gripper_speed=100.0,
                send_interval_ms=120,
            )

            self.assertEqual(len(window.joint_vars), 6)
            window.joint_vars[0].set(20.0)
            window._on_joint_change(0)
            window.gripper_var.set(800.0)
            window._on_gripper_change()

            self.assertEqual(root.after_calls, 1)
            root.run_pending()

            self.assertEqual(robot.set_robot_state.call_count, 2)
            joint_call = robot.set_robot_state.call_args_list[0]
            gripper_call = robot.set_robot_state.call_args_list[1]
            self.assertEqual(joint_call.kwargs["target_joints"][0], 20.0)
            self.assertFalse(joint_call.kwargs["wait_for_completion"])
            self.assertEqual(gripper_call.kwargs["gripper_value"], 800.0)
            self.assertFalse(gripper_call.kwargs["wait_for_completion"])
        finally:
            if original_tkinter is None:
                sys.modules.pop("tkinter", None)
            else:
                sys.modules["tkinter"] = original_tkinter


class FakeRoot:
    def __init__(self):
        self.after_calls = 0
        self._pending_callback = None

    def title(self, _title):
        pass

    def protocol(self, _name, _callback):
        pass

    def columnconfigure(self, _column, weight=0):
        pass

    def rowconfigure(self, _row, weight=0):
        pass

    def after(self, _delay, callback):
        self.after_calls += 1
        self._pending_callback = callback
        return "after-1"

    def after_cancel(self, _callback_id):
        self._pending_callback = None

    def run_pending(self):
        callback = self._pending_callback
        self._pending_callback = None
        callback()

    def destroy(self):
        pass


class FakeWidget:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.grid_calls = []

    def grid(self, **kwargs):
        self.grid_calls.append(kwargs)

    def columnconfigure(self, _column, weight=0):
        pass


class FakeVar:
    def __init__(self, value=None):
        self._value = value

    def get(self):
        return self._value

    def set(self, value):
        self._value = value


def build_fake_tkinter():
    return types.SimpleNamespace(
        Frame=FakeWidget,
        Label=FakeWidget,
        Scale=FakeWidget,
        StringVar=FakeVar,
        DoubleVar=FakeVar,
        HORIZONTAL="horizontal",
    )


if __name__ == "__main__":
    unittest.main()
