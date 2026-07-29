#!/usr/bin/env python3
"""Exercise measured state, action CLI, authority, diagnostics, TF ownership, and shutdown."""
import os
import signal
import subprocess
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Pose, PoseArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


EXPECTED_NAMES = [
    *(f"openarm_left_joint{index}" for index in range(1, 8)),
    *(f"openarm_right_joint{index}" for index in range(1, 8)),
]


def wait_for(node, predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise AssertionError("timed out waiting for ROS contract event")


def fields(message):
    return {item.key: item.value for item in message.status[0].values}


def main():
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    launch = subprocess.Popen(
        ["ros2", "launch", "openarm_ik_ros", "openarm_ik_rviz.launch.py", "rviz:=false"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=environment,
    )
    os.environ["ROS_DOMAIN_ID"] = environment["ROS_DOMAIN_ID"]
    rclpy.init()
    node = rclpy.create_node("openarm_virtual_control_contract_test")
    states = []
    diagnostics = []
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(JointState, "/joint_states", states.append, qos)
    node.create_subscription(DiagnosticArray, "/openarm_ik/diagnostics", diagnostics.append, qos)
    try:
        wait_for(node, lambda: len(states) >= 3 and diagnostics)
        assert states[-1].name == EXPECTED_NAMES
        assert len(states[-1].position) == 14
        assert len(states[-1].velocity) == 14
        assert len(states[-1].effort) == 14
        assert states[-1].header.stamp.sec > 0
        assert len(node.get_publishers_info_by_topic("/joint_states")) == 1
        assert len(node.get_publishers_info_by_topic("/tf")) == 1
        assert len(node.get_publishers_info_by_topic("/tf_static")) == 1
        report = fields(diagnostics[-1])
        level = diagnostics[-1].status[0].level
        if isinstance(level, bytes):
            level = int.from_bytes(level)
        assert level == 1, (level, report)
        assert report["backend"] == "virtual"
        assert report["capability_bits"] == "7"
        assert report["collision_checked"] == "false"
        assert report["state_source"] == "oa_snapshot_encoder_feedback"
        assert report["physical_motion_authorized"] == "false"
        assert report["left_fresh_mask"] == "127"
        assert report["right_fresh_mask"] == "127"

        duplicate = subprocess.run(
            ["ros2", "run", "openarm_ik_ros", "openarm_ik_ros_node"],
            env=environment,
            text=True,
            capture_output=True,
            timeout=5.0,
            check=False,
        )
        assert duplicate.returncode != 0
        assert "another local JointState authority" in duplicate.stderr

        invalid_name = subprocess.run(
            ["ros2", "run", "openarm_ik_ros", "openarm_control_cli", "move-joint",
             "unknown_joint", "0.1"],
            env=environment,
            text=True,
            capture_output=True,
            timeout=5.0,
            check=False,
        )
        assert invalid_name.returncode == 4
        assert "goal rejected" in invalid_name.stderr

        invalid_frame = subprocess.run(
            ["ros2", "run", "openarm_ik_ros", "openarm_control_cli", "move-paired-tcp",
             "missing_frame", "0.2", "0.3", "0.85", "0.2", "-0.3", "0.85"],
            env=environment,
            text=True,
            capture_output=True,
            timeout=5.0,
            check=False,
        )
        assert invalid_frame.returncode == 7
        assert "transform_unavailable" in invalid_frame.stderr

        legacy = PoseArray()
        legacy.header.stamp = node.get_clock().now().to_msg()
        legacy.header.frame_id = "openarm_body_link0"
        left = Pose()
        left.position.x = 0.20
        left.position.y = 0.30
        left.position.z = 0.85
        right = Pose()
        right.position.x = 0.20
        right.position.y = -0.30
        right.position.z = 0.85
        legacy.poses = [left, right]
        legacy_stamp_ns = (
            legacy.header.stamp.sec * 1_000_000_000 + legacy.header.stamp.nanosec
        )
        legacy_owner = f"legacy:{legacy_stamp_ns}"
        legacy_publisher = node.create_publisher(PoseArray, "/openarm_ik/paired_xyz", qos)
        wait_for(node, lambda: legacy_publisher.get_subscription_count() == 1)
        legacy_publisher.publish(legacy)
        wait_for(
            node,
            lambda: diagnostics
            and fields(diagnostics[-1]).get("active_owner") == legacy_owner
            and fields(diagnostics[-1]).get("executing") == "true",
        )
        rejected_during_legacy = subprocess.run(
            ["ros2", "run", "openarm_ik_ros", "openarm_control_cli", "move-joint",
             "openarm_left_joint4", "0.1"],
            env=environment,
            text=True,
            capture_output=True,
            timeout=5.0,
            check=False,
        )
        assert rejected_during_legacy.returncode == 4
        wait_for(
            node,
            lambda: diagnostics
            and fields(diagnostics[-1]).get("last_action") == "deprecated_paired_xyz"
            and fields(diagnostics[-1]).get("last_goal_id") == legacy_owner
            and fields(diagnostics[-1]).get("committed") == "true"
            and fields(diagnostics[-1]).get("active_owner") == ""
            and fields(diagnostics[-1]).get("adapter_state") == "idle",
            timeout=35.0,
        )
        legacy_report = fields(diagnostics[-1])
        assert legacy_report["request_stamp_ns"] == str(legacy_stamp_ns)
        assert legacy_report["outcome"] == "completed"
        assert int(legacy_report["result_left_plan_seed_feedback_seq"]) > 0
        assert int(legacy_report["result_right_plan_seed_feedback_seq"]) > 0
        assert int(legacy_report["result_plan_duration_ns"]) > 0
        assert int(legacy_report["result_left_terminal_feedback_seq"]) >= int(
            legacy_report["result_left_plan_seed_feedback_seq"]
        )
        assert int(legacy_report["result_right_terminal_feedback_seq"]) >= int(
            legacy_report["result_right_plan_seed_feedback_seq"]
        )

        before_sequence = int(report["left_feedback_seq"])
        command = subprocess.run(
            ["ros2", "run", "openarm_ik_ros", "openarm_control_cli", "move-joint",
             "openarm_left_joint4", "0.2"],
            env=environment,
            text=True,
            capture_output=True,
            timeout=15.0,
            check=False,
        )
        assert command.returncode == 0, command.stderr
        assert "completed command_id=" in command.stdout
        wait_for(
            node,
            lambda: diagnostics and fields(diagnostics[-1]).get("committed") == "true"
            and states and abs(states[-1].position[3] - 0.2) < 5e-4,
        )
        assert int(fields(diagnostics[-1])["left_feedback_seq"]) > before_sequence
        assert abs(states[-1].position[3] - 0.2) < 5e-4
    finally:
        node.destroy_node()
        rclpy.shutdown()
        started = time.monotonic()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait()
            raise AssertionError("headless launch exceeded the two-second shutdown bound")
        assert time.monotonic() - started < 2.0
        assert launch.returncode == 0


if __name__ == "__main__":
    main()
