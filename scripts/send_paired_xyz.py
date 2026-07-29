#!/usr/bin/env python3
"""Publish and acknowledge one paired XYZ transaction: left_x left_y left_z right_x right_y right_z."""
import argparse
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Pose, PoseArray


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("xyz", type=float, nargs=6)
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    rclpy.init()
    node = rclpy.create_node("openarm_ik_paired_xyz_cli")
    publisher = node.create_publisher(PoseArray, "/openarm_ik/paired_xyz", 10)
    acknowledgement = []

    def receive_diagnostic(message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name == "openarm_ik_ros/paired_position_ik":
                acknowledgement.append({item.key: item.value for item in status.values})

    node.create_subscription(DiagnosticArray, "/openarm_ik/diagnostics", receive_diagnostic, 10)
    deadline = time.monotonic() + args.timeout
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if publisher.get_subscription_count() == 0:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError("adapter subscription was not discovered")
    message = PoseArray()
    message.header.frame_id = "world"
    message.header.stamp = node.get_clock().now().to_msg()
    for offset in (0, 3):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = args.xyz[offset:offset + 3]
        message.poses.append(pose)
    publisher.publish(message)
    while not acknowledgement and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not acknowledgement:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError("adapter diagnostic acknowledgement timed out")
    print(" ".join(f"{key}={value}" for key, value in sorted(acknowledgement[-1].items())))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
