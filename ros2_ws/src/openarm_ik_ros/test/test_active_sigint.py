#!/usr/bin/env python3
"""Exercise bounded SIGINT shutdown at queued, active, settling, and completed phases."""
import os
import signal
import subprocess
import sys
import time

import rclpy
from openarm_control_msgs.action import MoveJoint
from rclpy.action import ActionClient


PHASES = ("queued", "started", "settling", "completed")


def spin_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return True
    return predicate()


def stop_process(process):
    started = time.monotonic()
    os.killpg(process.pid, signal.SIGINT)
    try:
        output, _ = process.communicate(timeout=2.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        output, _ = process.communicate()
        raise AssertionError("active adapter exceeded the two-second SIGINT bound")
    assert time.monotonic() - started < 2.0
    assert process.returncode == 0, output
    assert "terminate called" not in output
    assert "goal does not exist" not in output


def run_phase(phase):
    process = subprocess.Popen(
        ["ros2", "run", "openarm_ik_ros", "openarm_ik_ros_node"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    rclpy.init()
    node = rclpy.create_node(f"openarm_sigint_{phase}")
    client = ActionClient(node, MoveJoint, "/openarm_ik/move_joint")
    try:
        assert client.wait_for_server(timeout_sec=5.0)
        feedback = []
        goal = MoveJoint.Goal()
        goal.stamp = node.get_clock().now().to_msg()
        goal.joint_name = "openarm_right_joint1"
        goal.target_rad = 0.8
        goal_future = client.send_goal_async(
            goal, feedback_callback=lambda value: feedback.append(value.feedback.measured_progress)
        )
        if phase == "queued":
            time.sleep(0.01)
            stop_process(process)
            return

        assert spin_until(node, goal_future.done, 3.0)
        handle = goal_future.result()
        assert handle is not None and handle.accepted
        result_future = handle.get_result_async()
        if phase == "started":
            assert spin_until(node, lambda: any(0.0 < item < 0.8 for item in feedback), 5.0)
        elif phase == "settling":
            assert spin_until(
                node,
                lambda: not result_future.done() and any(item >= 0.98 for item in feedback),
                15.0,
            )
        else:
            assert spin_until(node, result_future.done, 15.0)
        stop_process(process)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        node.destroy_node()
        rclpy.shutdown()


def main():
    if len(sys.argv) == 2:
        assert sys.argv[1] in PHASES
        run_phase(sys.argv[1])
        return
    for index, phase in enumerate(PHASES):
        environment = os.environ.copy()
        environment["ROS_DOMAIN_ID"] = str(180 + (os.getpid() + index) % 50)
        completed = subprocess.run(
            [sys.executable, __file__, phase],
            env=environment,
            text=True,
            capture_output=True,
            timeout=25.0,
            check=False,
        )
        assert completed.returncode == 0, (
            f"{phase} SIGINT phase failed\nstdout:\n{completed.stdout}\nstderr:\n{completed.stderr}"
        )


if __name__ == "__main__":
    main()
