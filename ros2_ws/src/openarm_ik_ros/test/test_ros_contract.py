#!/usr/bin/env python3
"""Exercise measured state, actions, live portal routing, authority, TF, and shutdown."""
import http.client
import json
import os
import re
import signal
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Pose, PoseArray
from openarm_control_msgs.action import MovePairedTcpScaled
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


EXPECTED_NAMES = [
    *(f"openarm_left_joint{index}" for index in range(1, 8)),
    *(f"openarm_right_joint{index}" for index in range(1, 8)),
]
EXPECTED_WORLD_CHILDREN = [
    "openarm_body_link0",
    *(f"openarm_{side}_link{index}" for side in ("left", "right") for index in range(8)),
    *(f"openarm_{side}_{suffix}" for side in ("left", "right")
      for suffix in ("hand", "hand_tcp", "left_finger", "right_finger")),
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


def scaled_goal(node, scale, left=(0.30, 0.22, 0.30), right=(0.30, -0.22, 0.30)):
    goal = MovePairedTcpScaled.Goal()
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.header.frame_id = "openarm_body_link0"
    goal.left_tcp_m.x, goal.left_tcp_m.y, goal.left_tcp_m.z = left
    goal.right_tcp_m.x, goal.right_tcp_m.y, goal.right_tcp_m.z = right
    goal.motion_limit_scale = scale
    return goal


def portal_request(port, method, target, payload=None, csrf=None):
    body = None if payload is None else json.dumps(payload, separators=(",", ":"))
    headers = {"Host": f"127.0.0.1:{port}"}
    if body is not None:
        headers.update({
            "Content-Type": "application/json",
            "Origin": f"http://127.0.0.1:{port}",
            "Sec-Fetch-Site": "same-origin",
            "X-CSRF-Token": csrf,
        })
    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=2.0)
    try:
        connection.request(method, target, body=body, headers=headers)
        response = connection.getresponse()
        return response.status, response.read().decode("utf-8")
    finally:
        connection.close()


def stop_process_group(process, label, errors):
    """Reap a test process group without allowing cleanup errors to escape early."""
    started = time.monotonic()
    try:
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            errors.append(f"{label} exceeded the two-second shutdown bound")
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=2.0)
        if time.monotonic() - started >= 2.0:
            errors.append(f"{label} missed the two-second shutdown bound")
        if process.returncode != 0:
            errors.append(f"{label} exited with status {process.returncode}")
    except Exception as error:
        errors.append(f"{label} cleanup failed: {error}")
        try:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
        except (OSError, ProcessLookupError):
            pass
        try:
            process.wait(timeout=2.0)
        except Exception:
            pass


def main():
    environment = os.environ.copy()
    environment.setdefault("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
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
    tf_listener = None
    portal = None
    try:
        tf_buffer = Buffer(node=node)
        tf_listener = TransformListener(tf_buffer, node, spin_thread=False)
        wait_for(node, lambda: len(states) >= 3 and diagnostics)
        assert states[-1].name == EXPECTED_NAMES
        assert not any("finger" in name for name in states[-1].name)
        assert len(states[-1].position) == 14
        assert len(states[-1].velocity) == 14
        assert len(states[-1].effort) == 14
        assert states[-1].header.stamp.sec > 0
        assert len(node.get_publishers_info_by_topic("/joint_states")) == 1
        assert len(node.get_publishers_info_by_topic("/tf")) == 1
        assert len(node.get_publishers_info_by_topic("/tf_static")) == 1
        wait_for(
            node,
            lambda: all(tf_buffer.can_transform("world", frame, rclpy.time.Time())
                        for frame in EXPECTED_WORLD_CHILDREN),
        )
        assert len(EXPECTED_WORLD_CHILDREN) == 25
        report = fields(diagnostics[-1])
        level = diagnostics[-1].status[0].level
        if isinstance(level, bytes):
            level = int.from_bytes(level)
        assert level == 1, (level, report)
        assert report["backend"] == "virtual"
        assert report["runtime_authority"] == "openarm_runtime"
        assert report["capability_bits"] == "3576"
        assert report["runtime_state_source"] == "oa_runtime_snapshot_encoder_feedback"
        assert len(report["runtime_coordinate_identity_sha256"]) == 64
        assert report["collision_checked"] == "false"
        assert report["state_source"] == "oa_snapshot_encoder_feedback"
        assert report["physical_motion_authorized"] == "false"
        assert report["physical_motion_capability"] == "false"
        assert report["physical_discovery_endpoint_exposed"] == "false"
        assert report["single_xyz_capability"] == "false"
        assert report["manifest_state"] == "4"
        assert report["manifest_authenticated"] == "false"
        assert report["manifest_checkpoint_authorized"] == "false"
        assert report["persistence_status"] == "built_in_immutable_manifest_not_persisted"
        assert report["calibration_status"] == "runtime_capable_ros_endpoint_not_exposed"
        assert report["discovery_status"] == "virtual_exact_inventory"
        assert report["inventory_interface_count"] == "2"
        assert report["inventory_motor_count"] == "14"
        assert report["inventory_unresolved_assignment"] == "0"
        assert report["left_fresh_mask"] == "127"
        assert report["right_fresh_mask"] == "127"

        portal_port = 31000 + os.getpid() % 10000
        portal = subprocess.Popen(
            ["ros2", "run", "openarm_ik_ros", "openarm_portal", "--port", str(portal_port)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
            env=environment,
        )
        portal_deadline = time.monotonic() + 8.0
        while True:
            assert portal.poll() is None, portal.stdout.read()
            try:
                health_status, _ = portal_request(portal_port, "GET", "/api/health")
                if health_status == 200:
                    break
            except OSError:
                pass
            assert time.monotonic() < portal_deadline, "portal health deadline exceeded"
            time.sleep(0.05)
        page_status, page = portal_request(portal_port, "GET", "/")
        assert page_status == 200
        csrf_match = re.search(r'name="portal-csrf" content="([0-9a-f]{64})"', page)
        assert csrf_match
        csrf = csrf_match.group(1)
        portal_state_deadline = time.monotonic() + 8.0
        while True:
            state_status, state_body = portal_request(portal_port, "GET", "/api/state")
            if state_status == 200 and json.loads(state_body)["state_fresh"]:
                break
            assert time.monotonic() < portal_state_deadline, "portal state deadline exceeded"
            time.sleep(0.05)

        scaled_action = ActionClient(
            node, MovePairedTcpScaled, "/openarm_ik/move_paired_tcp_scaled"
        )
        assert scaled_action.wait_for_server(timeout_sec=8.0)
        for invalid_scale in (0.49, 1.01, float("nan"), float("inf"), float("-inf")):
            invalid_future = scaled_action.send_goal_async(scaled_goal(node, invalid_scale))
            wait_for(node, invalid_future.done)
            assert not invalid_future.result().accepted

        move_status, move_body = portal_request(
            portal_port,
            "POST",
            "/api/v3/move",
            {"side": "left", "unit": "m", "x": 0.30, "y": 0.22, "z": 0.30,
             "motion_limit_scale": 0.8},
            csrf,
        )
        assert move_status == 202, move_body
        move_response = json.loads(move_body)
        assert move_response["motion_limit_scale"] == 0.8
        wait_for(
            node,
            lambda: diagnostics
            and fields(diagnostics[-1]).get("last_action") == "move_paired_tcp_scaled"
            and fields(diagnostics[-1]).get("outcome") == "completed"
            and fields(diagnostics[-1]).get("active_owner") == "",
            timeout=30.0,
        )
        scaled_report = fields(diagnostics[-1])
        assert int(scaled_report["result_plan_duration_ns"]) > 0
        assert int(scaled_report["result_left_terminal_feedback_seq"]) >= int(
            scaled_report["result_left_plan_seed_feedback_seq"]
        )
        assert int(scaled_report["result_right_terminal_feedback_seq"]) >= int(
            scaled_report["result_right_plan_seed_feedback_seq"]
        )

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
        assert invalid_frame.returncode == 7, (
            invalid_frame.returncode, invalid_frame.stdout, invalid_frame.stderr
        )
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
        joint_target = 0.8 if states[-1].position[3] < 0.6 else 0.2
        command = subprocess.run(
            ["ros2", "run", "openarm_ik_ros", "openarm_control_cli", "move-joint",
             "openarm_left_joint4", str(joint_target)],
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
            and states and abs(states[-1].position[3] - joint_target) < 5e-4,
        )
        assert int(fields(diagnostics[-1])["left_feedback_seq"]) > before_sequence
        assert abs(states[-1].position[3] - joint_target) < 5e-4

        cancel_target = scaled_goal(
            node, 0.8, left=(0.20, 0.30, 0.30), right=(0.20, -0.30, 0.30)
        )
        cancel_goal_future = scaled_action.send_goal_async(cancel_target)
        wait_for(node, cancel_goal_future.done)
        cancel_handle = cancel_goal_future.result()
        assert cancel_handle.accepted
        wait_for(
            node,
            lambda: diagnostics
            and fields(diagnostics[-1]).get("last_action") == "move_paired_tcp_scaled"
            and fields(diagnostics[-1]).get("executing") == "true"
            and fields(diagnostics[-1]).get("active_owner") != "",
        )
        cancel_future = cancel_handle.cancel_goal_async()
        wait_for(node, cancel_future.done)
        assert cancel_future.result().goals_canceling
        canceled_result_future = cancel_handle.get_result_async()
        wait_for(node, canceled_result_future.done)
        canceled_wrapped_result = canceled_result_future.result()
        assert canceled_wrapped_result.status == GoalStatus.STATUS_CANCELED
        assert canceled_wrapped_result.result.outcome == MovePairedTcpScaled.Result.OUTCOME_CANCELED
        assert not canceled_wrapped_result.result.collision_checked
        scaled_action.destroy()
    finally:
        cleanup_errors = []
        try:
            if portal is not None:
                stop_process_group(portal, "portal", cleanup_errors)
            try:
                if tf_listener is not None:
                    del tf_listener
                node.destroy_node()
                rclpy.shutdown()
            except Exception as error:  # Keep process cleanup unconditional on ROS teardown failures.
                cleanup_errors.append(f"ROS client teardown failed: {error}")
        finally:
            stop_process_group(launch, "headless launch", cleanup_errors)
        if cleanup_errors:
            raise AssertionError("; ".join(cleanup_errors))


if __name__ == "__main__":
    main()
