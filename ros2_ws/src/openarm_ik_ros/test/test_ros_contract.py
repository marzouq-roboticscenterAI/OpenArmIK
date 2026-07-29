#!/usr/bin/env python3
"""Exercise the standard PoseArray adapter contract against a real headless launch."""
import os
import signal
import subprocess
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Pose, PoseArray
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


EXPECTED_NAMES = [
    *(f"openarm_left_joint{index}" for index in range(1, 8)),
    *(f"openarm_right_joint{index}" for index in range(1, 8)),
    "openarm_left_finger_joint1",
    "openarm_right_finger_joint1",
]


def values(status):
    return {entry.key: entry.value for entry in status.values}


def wait_for(node, predicate, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise AssertionError("timed out waiting for ROS contract event")


def paired_message(left, right, node):
    message = PoseArray()
    message.header.frame_id = "world"
    message.header.stamp = node.get_clock().now().to_msg()
    for xyz in (left, right):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        message.poses.append(pose)
    return message


def main():
    launch = subprocess.Popen(
        ["ros2", "launch", "openarm_ik_ros", "openarm_ik_rviz.launch.py", "rviz:=false"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=os.environ.copy(),
    )
    rclpy.init()
    node = rclpy.create_node("openarm_ik_ros_contract_test")
    states, diagnostics, transforms = [], [], {}
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    static_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    node.create_subscription(JointState, "/joint_states", states.append, qos)
    node.create_subscription(DiagnosticArray, "/openarm_ik/diagnostics", diagnostics.append, qos)

    def receive_tf(message: TFMessage):
        for transform in message.transforms:
            transforms[transform.child_frame_id] = transform

    node.create_subscription(TFMessage, "/tf", receive_tf, qos)
    node.create_subscription(TFMessage, "/tf_static", receive_tf, static_qos)
    tf_buffer = Buffer()
    listener = TransformListener(tf_buffer, node)
    publisher = node.create_publisher(PoseArray, "/openarm_ik/paired_xyz", qos)
    try:
        wait_for(node, lambda: states and tf_buffer.can_transform("world", "openarm_body_link0", Time()))
        assert states[-1].name == EXPECTED_NAMES
        assert len(node.get_publishers_info_by_topic("/joint_states")) == 1
        assert len(node.get_publishers_info_by_topic("/tf")) == 1
        assert publisher.get_subscription_count() == 1
        assert tf_buffer.lookup_transform("world", "openarm_body_link0", Time()).header.frame_id == "world"

        publisher.publish(paired_message((0.20, 0.30, 0.85), (0.20, -0.30, 0.85), node))
        wait_for(node, lambda: diagnostics and values(diagnostics[-1].status[0]).get("committed") == "true")
        report = values(diagnostics[-1].status[0])
        assert report["achieved_available"] == "true"
        assert report["redundancy_policy"] == "continuity-v1"
        assert report["backend"] == "virtual"
        assert report["collision_checked"] == "false"
        left_expected = tuple(map(float, report["left_achieved_hand_tcp_xyz"].split(",")))
        right_expected = tuple(map(float, report["right_achieved_hand_tcp_xyz"].split(",")))

        def tcp_matches_diagnostic():
            if not tf_buffer.can_transform("world", "openarm_left_hand_tcp", Time()) or \
                    not tf_buffer.can_transform("world", "openarm_right_hand_tcp", Time()):
                return False
            left = tf_buffer.lookup_transform("world", "openarm_left_hand_tcp", Time()).transform.translation
            right = tf_buffer.lookup_transform("world", "openarm_right_hand_tcp", Time()).transform.translation
            return all(abs(actual - expected) < 1e-6 for actual, expected in zip(
                (left.x, left.y, left.z), left_expected)) and all(
                abs(actual - expected) < 1e-6 for actual, expected in zip(
                    (right.x, right.y, right.z), right_expected))

        wait_for(
            node,
            tcp_matches_diagnostic,
        )
        left_tcp = tf_buffer.lookup_transform("world", "openarm_left_hand_tcp", Time()).transform.translation
        right_tcp = tf_buffer.lookup_transform("world", "openarm_right_hand_tcp", Time()).transform.translation
        for actual, expected in zip(
            (left_tcp.x, left_tcp.y, left_tcp.z),
            left_expected,
        ):
            assert abs(actual - expected) < 1e-6
        for actual, expected in zip(
            (right_tcp.x, right_tcp.y, right_tcp.z),
            right_expected,
        ):
            assert abs(actual - expected) < 1e-6
        committed_state = list(states[-1].position)

        publisher.publish(paired_message((10.0, 10.0, 10.0), (0.20, -0.30, 0.85), node))
        wait_for(node, lambda: diagnostics and values(diagnostics[-1].status[0]).get("committed") == "false")
        failure = values(diagnostics[-1].status[0])
        assert failure["achieved_available"] == "false"
        assert "left_residual_m" not in failure
        assert "left_achieved_hand_tcp_matrix" not in failure
        time.sleep(0.2)
        rclpy.spin_once(node, timeout_sec=0.1)
        assert list(states[-1].position) == committed_state
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait()


if __name__ == "__main__":
    main()
