#!/usr/bin/env python3
"""Publish one fresh paired XYZ transaction: left_x left_y left_z right_x right_y right_z."""
import argparse
import sys

import rclpy
from geometry_msgs.msg import Pose, PoseArray


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("xyz", type=float, nargs=6)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("openarm_ik_paired_xyz_cli")
    publisher = node.create_publisher(PoseArray, "/openarm_ik/paired_xyz", 10)
    message = PoseArray()
    message.header.frame_id = "world"
    message.header.stamp = node.get_clock().now().to_msg()
    for offset in (0, 3):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = args.xyz[offset:offset + 3]
        message.poses.append(pose)
    for _ in range(5):
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
