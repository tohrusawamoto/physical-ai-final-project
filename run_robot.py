#!/usr/bin/env python3
"""Run a human-approved JSON motion plan in a ROS 2 Humble simulation.

Usage:
    python3 run_robot.py commands.json --validate-only
    python3 run_robot.py commands.json

The JSON is an array of {"action": "forward|left|right|stop", "duration": seconds}.
This script does not call an AI service. Save the AI's proposed JSON, inspect it,
and explicitly approve execution at the terminal.

Simulation demonstration only. Time-based motion is open-loop: actual distance
and angle depend on the simulator and its speed. There is no obstacle detection.
Stop other teleoperation/controllers publishing to the same velocity topic.
Final zero-velocity messages are best effort, not an emergency-stop guarantee;
SIGKILL, a network failure, or a crashed process can prevent their delivery.
"""

import argparse
import json
import math
from pathlib import Path
import re
import signal
import sys
import threading
import time
from dataclasses import dataclass


MAX_COMMANDS = 20
MAX_DURATION = 10.0
MAX_TOTAL_DURATION = 60.0
MAX_PLAN_BYTES = 64 * 1024
PUBLISH_HZ = 20.0
SUBSCRIBER_TIMEOUT = 5.0
STOP_MESSAGES = 6
VELOCITIES = {
    "forward": (0.10, 0.0),
    "left": (0.0, 0.35),
    "right": (0.0, -0.35),
    "stop": (0.0, 0.0),
}


@dataclass(frozen=True)
class Command:
    action: str
    duration: float


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON key: " + key)
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError("Non-finite JSON number is not allowed: " + value)


def parse_plan(text):
    """Validate the entire plan before any ROS import, confirmation, or motion."""
    if len(text.encode("utf-8")) > MAX_PLAN_BYTES:
        raise ValueError("Plan must be at most 64 KiB.")
    try:
        raw = json.loads(
            text, object_pairs_hook=_unique_object, parse_constant=_reject_constant
        )
    except (json.JSONDecodeError, RecursionError) as exc:
        raise ValueError("Invalid JSON: " + str(exc)) from exc
    if not isinstance(raw, list) or not 1 <= len(raw) <= MAX_COMMANDS:
        raise ValueError("Plan must be a JSON array with 1 to 20 commands.")

    commands = []
    for index, item in enumerate(raw, start=1):
        prefix = "Command {}: ".format(index)
        if not isinstance(item, dict) or set(item) != {"action", "duration"}:
            raise ValueError(prefix + "use exactly the keys action and duration.")
        action = item["action"]
        if not isinstance(action, str) or action not in VELOCITIES:
            raise ValueError(prefix + "action must be forward, left, right, or stop.")
        duration = item["duration"]
        # bool is a subclass of int; check exact types to reject true/false.
        if type(duration) not in (int, float):
            raise ValueError(prefix + "duration must be a JSON number.")
        if not 0 < duration <= MAX_DURATION or not math.isfinite(duration):
            raise ValueError(prefix + "duration must be finite and greater than 0, at most 10.")
        commands.append(Command(action, float(duration)))

    if math.fsum(command.duration for command in commands) > MAX_TOTAL_DURATION:
        raise ValueError("Total planned duration must be at most 60 seconds.")
    return tuple(commands)


def load_plan(path):
    with Path(path).open("rb") as source:
        content = source.read(MAX_PLAN_BYTES + 1)
    if len(content) > MAX_PLAN_BYTES:
        raise ValueError("Plan must be at most 64 KiB.")
    return parse_plan(content.decode("utf-8-sig"))


def describe_plan(commands, topic):
    print("Validated plan; topic: {}".format(topic), flush=True)
    print("Step  Action     Seconds    Linear m/s    Angular rad/s")
    for index, command in enumerate(commands, start=1):
        linear, angular = VELOCITIES[command.action]
        print("{:>4}  {:<8}  {:>7.2f}    {:>10.2f}    {:>13.2f}".format(
            index, command.action, command.duration, linear, angular
        ))
    print("Total planned command time: {:.2f} seconds".format(
        math.fsum(command.duration for command in commands)
    ), flush=True)


def run_ros_plan(commands, topic):
    """Publish only after approval; keep ROS alive for the final stop burst."""
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.signals import SignalHandlerOptions
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Python modules are unavailable. Run this inside the ROS container "
            "after sourcing /opt/ros/humble/setup.bash."
        ) from exc

    stop_requested = threading.Event()
    old_handlers = {}
    node = None
    publisher = None
    initialized = False
    execution_started = False
    interval = 1.0 / PUBLISH_HZ

    def request_stop(signum, frame):
        # Do not shut down rclpy here: the finally block needs a live publisher.
        stop_requested.set()

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, request_stop)
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node("human_confirmed_json_robot")
        publisher = node.create_publisher(Twist, topic, 1)

        print("Waiting up to 5 seconds for a compatible velocity subscriber...", flush=True)
        deadline = time.monotonic() + SUBSCRIBER_TIMEOUT
        while publisher.get_subscription_count() == 0:
            if stop_requested.is_set():
                return 130
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "No compatible subscriber on {}. No velocity was sent. "
                    "Check the simulator, ROS_DOMAIN_ID, topic name, and message type."
                    .format(topic)
                )
            rclpy.spin_once(node, timeout_sec=0.05)
        if stop_requested.is_set():
            return 130

        # A subscriber can also be a monitoring tool; discovery alone does not
        # establish that a robot exists or is safe to command.
        execution_started = True
        for index, command in enumerate(commands, start=1):
            if stop_requested.is_set():
                break
            linear, angular = VELOCITIES[command.action]
            message = Twist()
            message.linear.x = linear
            message.angular.z = angular
            print("[{}/{}] {} for {:.2f} s".format(
                index, len(commands), command.action, command.duration
            ), flush=True)
            deadline = time.monotonic() + command.duration
            while not stop_requested.is_set() and time.monotonic() < deadline:
                if not rclpy.ok():
                    raise RuntimeError("ROS context stopped during execution.")
                if publisher.get_subscription_count() == 0:
                    raise RuntimeError("Velocity subscriber disappeared; aborting the plan.")
                publisher.publish(message)
                remaining = deadline - time.monotonic()
                if remaining > 0:
                    stop_requested.wait(min(interval, remaining))
        return 130 if stop_requested.is_set() else 0
    finally:
        # A malformed plan, declined approval, or absent subscriber sends nothing.
        if execution_started and publisher is not None and rclpy.ok():
            try:
                for _ in range(STOP_MESSAGES):
                    publisher.publish(Twist())
                    time.sleep(interval)
                print("Final zero-velocity messages sent (best effort).", flush=True)
            except Exception as exc:
                print("Could not finish sending zero velocity: {}".format(exc), file=sys.stderr)
        try:
            if node is not None:
                node.destroy_node()
        finally:
            try:
                if initialized and rclpy.ok():
                    rclpy.shutdown()
            finally:
                for signum, handler in old_handlers.items():
                    signal.signal(signum, handler)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", nargs="?", default="commands.json", type=Path)
    parser.add_argument("--validate-only", action="store_true",
                        help="Validate and print the plan; never import ROS or publish.")
    parser.add_argument("--topic", default="/cmd_vel", help="Twist topic (default: /cmd_vel).")
    args = parser.parse_args(argv)
    try:
        if re.fullmatch(r"/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*", args.topic) is None:
            raise ValueError("Use an absolute topic such as /cmd_vel or /robot/cmd_vel.")
        commands = load_plan(args.plan)
    except (OSError, UnicodeError, ValueError) as exc:
        print("Validation failed: {}".format(exc), file=sys.stderr)
        return 2

    describe_plan(commands, args.topic)
    if args.validate_only:
        print("Validation only. No ROS initialization or velocity publication.")
        return 0
    if not sys.stdin.isatty():
        print("Execution requires an interactive terminal for human approval.", file=sys.stderr)
        return 2
    print("Check the simulated robot and its surroundings. Motion is open-loop.")
    try:
        approval = input("Type EXECUTE to run this plan, or anything else to cancel: ")
    except (EOFError, KeyboardInterrupt):
        print("\nCancelled. No velocity was sent.")
        return 130
    if approval.strip() != "EXECUTE":
        print("Cancelled. No velocity was sent.")
        return 1
    try:
        result = run_ros_plan(commands, args.topic)
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    except Exception as exc:
        print("Execution failed: {}".format(exc), file=sys.stderr)
        return 3
    if result == 130:
        print("Interrupted; the remaining commands were skipped.")
    else:
        print("Plan finished.")
    return result


if __name__ == "__main__":
    sys.exit(main())
