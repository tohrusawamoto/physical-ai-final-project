"""Focused checks for plan validation and the pre-execution approval boundary.

These tests use the Python standard library. ROS objects are replaced with
test doubles for cleanup checks; actual ROS, Gazebo, message delivery, and
robot motion are not tested.
"""

import builtins
import contextlib
import io
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

import run_robot


@contextlib.contextmanager
def fake_ros(subscribers=1, interrupt_on_motion=False):
    """Test cleanup ordering without importing ROS or sending any messages."""
    events = []
    handlers = {}
    state = {"ok": False}

    class Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)

    class Publisher:
        def get_subscription_count(self):
            return subscribers

        def publish(self, message):
            if not state["ok"]:
                raise AssertionError("ROS must still be alive when publishing the stop")
            events.append(("publish", message.linear.x, message.angular.z))
            if interrupt_on_motion and (message.linear.x or message.angular.z):
                handlers[run_robot.signal.SIGINT](run_robot.signal.SIGINT, None)

    publisher = Publisher()
    node = types.SimpleNamespace(
        create_publisher=lambda *args, **kwargs: publisher,
        destroy_node=lambda: events.append(("destroy",)),
    )
    rclpy = types.ModuleType("rclpy")
    rclpy.__path__ = []
    rclpy.init = lambda **kwargs: state.update(ok=True)
    rclpy.ok = lambda: state["ok"]
    rclpy.create_node = lambda *args: node
    rclpy.spin_once = lambda *args, **kwargs: None

    def shutdown():
        events.append(("shutdown",))
        state["ok"] = False

    rclpy.shutdown = shutdown
    signals = types.ModuleType("rclpy.signals")
    signals.SignalHandlerOptions = types.SimpleNamespace(NO=0)
    geometry = types.ModuleType("geometry_msgs")
    geometry.__path__ = []
    messages = types.ModuleType("geometry_msgs.msg")
    messages.Twist = Twist
    modules = {"rclpy": rclpy, "rclpy.signals": signals,
               "geometry_msgs": geometry, "geometry_msgs.msg": messages}
    with patch.dict(run_robot.sys.modules, modules), \
            patch.object(run_robot.signal, "signal", side_effect=lambda number, handler: handlers.update({number: handler})), \
            patch.object(run_robot.time, "sleep"), \
            contextlib.redirect_stdout(io.StringIO()):
        yield events


class PlanValidationTests(unittest.TestCase):
    def test_all_allowed_actions_and_velocity_directions(self):
        plan = run_robot.parse_plan(json.dumps([
            {"action": action, "duration": 1.5}
            for action in ("forward", "left", "right", "stop")
        ]))
        self.assertEqual(len(plan), 4)
        self.assertEqual(run_robot.VELOCITIES["forward"], (0.1, 0.0))
        self.assertGreater(run_robot.VELOCITIES["left"][1], 0)
        self.assertLess(run_robot.VELOCITIES["right"][1], 0)
        self.assertEqual(run_robot.VELOCITIES["stop"], (0.0, 0.0))

    def test_rejects_wrong_root_empty_plan_and_too_many_commands(self):
        values = [{}, None, "forward", [],
                  [{"action": "stop", "duration": 1}] * 21]
        for value in values:
            with self.subTest(value=value), self.assertRaises(ValueError):
                run_robot.parse_plan(json.dumps(value))

    def test_rejects_non_numeric_non_finite_or_out_of_range_duration(self):
        for value in (True, False, "1", None, 0, -0.0, -1, 10.001,
                      float("nan"), float("inf"), -float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                run_robot.parse_plan(json.dumps([{"action": "forward", "duration": value}]))
        with self.assertRaises(ValueError):
            run_robot.parse_plan('[{"action":"forward","duration":1e309}]')

    def test_limits_accept_boundaries_and_reject_excess_total(self):
        self.assertEqual(len(run_robot.parse_plan(json.dumps(
            [{"action": "stop", "duration": 10}] * 6
        ))), 6)
        self.assertEqual(len(run_robot.parse_plan(json.dumps(
            [{"action": "stop", "duration": 0.1}] * 20
        ))), 20)
        with self.assertRaises(ValueError):
            run_robot.parse_plan(json.dumps([{"action": "forward", "duration": 10}] * 7))

    def test_rejects_unknown_actions_missing_fields_and_extra_fields(self):
        for item in ({"action": "backward", "duration": 1},
                     {"action": "__import__('os')", "duration": 1},
                     {"action": True, "duration": 1},
                     {"action": "stop"},
                     {"action": "forward", "duration": 1, "speed": 100},
                     ["forward", 1]):
            with self.subTest(item=item), self.assertRaises(ValueError):
                run_robot.parse_plan(json.dumps([item]))

    def test_rejects_duplicate_keys_and_non_json_wrappers(self):
        for text in ('[{"action":"stop","action":"forward","duration":1}]',
                     '[{"action":"stop","duration":1,"duration":2}]',
                     '```json\n[{"action":"stop","duration":1}]\n```',
                     '[{"action":"stop","duration":1}] trailing text'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                run_robot.parse_plan(text)

    def test_file_size_limit_is_enforced(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "large.json"
            path.write_bytes(b" " * (run_robot.MAX_PLAN_BYTES + 1))
            with self.assertRaises(ValueError):
                run_robot.load_plan(path)

    def test_entire_plan_is_checked_before_approval_or_ros(self):
        # A valid first command must not execute before a bad later command is found.
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "bad.json"
            path.write_text('[{"action":"forward","duration":1},'
                            '{"action":"launch_shell","duration":1}]', encoding="utf-8")
            with patch("builtins.input") as approval, patch.object(run_robot, "run_ros_plan") as ros:
                with contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(run_robot.main([str(path)]), 2)
                approval.assert_not_called()
                ros.assert_not_called()

    def test_validate_only_does_not_import_ros_or_ask_for_approval(self):
        original_import = builtins.__import__

        def import_guard(name, *args, **kwargs):
            if name == "rclpy" or name.startswith("rclpy.") or name.startswith("geometry_msgs"):
                raise AssertionError("Validation must not import ROS")
            return original_import(name, *args, **kwargs)

        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "valid.json"
            path.write_text('[{"action":"stop","duration":1}]', encoding="utf-8")
            with patch("builtins.__import__", side_effect=import_guard), patch("builtins.input") as approval:
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(run_robot.main([str(path), "--validate-only"]), 0)
                approval.assert_not_called()

    def test_declining_and_noninteractive_input_never_run_ros(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "valid.json"
            path.write_text('[{"action":"forward","duration":1}]', encoding="utf-8")
            with patch.object(run_robot, "run_ros_plan") as ros:
                with patch.object(run_robot.sys.stdin, "isatty", return_value=True), patch("builtins.input", return_value="NO"):
                    with contextlib.redirect_stdout(io.StringIO()):
                        self.assertEqual(run_robot.main([str(path)]), 1)
                with patch.object(run_robot.sys.stdin, "isatty", return_value=False), patch("builtins.input") as approval:
                    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                        self.assertEqual(run_robot.main([str(path)]), 2)
                    approval.assert_not_called()
                ros.assert_not_called()

    def test_missing_subscriber_sends_nothing(self):
        with fake_ros(subscribers=0) as events, patch.object(run_robot, "SUBSCRIBER_TIMEOUT", 0.0):
            with self.assertRaises(RuntimeError):
                run_robot.run_ros_plan((run_robot.Command("forward", 1.0),), "/cmd_vel")
        self.assertFalse(any(event[0] == "publish" for event in events))
        self.assertEqual(events[-2:], [("destroy",), ("shutdown",)])

    def test_interrupt_skips_rest_and_stops_before_destroying_ros(self):
        with fake_ros(interrupt_on_motion=True) as events:
            result = run_robot.run_ros_plan((
                run_robot.Command("forward", 1.0), run_robot.Command("right", 1.0)
            ), "/cmd_vel")
        self.assertEqual(result, 130)
        publications = [event for event in events if event[0] == "publish"]
        self.assertEqual(publications[0], ("publish", 0.1, 0.0))
        self.assertEqual(publications[1:], [("publish", 0.0, 0.0)] * run_robot.STOP_MESSAGES)
        self.assertEqual(events[-2:], [("destroy",), ("shutdown",)])


if __name__ == "__main__":
    unittest.main()
