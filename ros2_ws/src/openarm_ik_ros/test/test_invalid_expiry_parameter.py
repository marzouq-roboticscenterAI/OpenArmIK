#!/usr/bin/env python3
"""The ROS parameter boundary rejects unsafe expiry conversion before node startup."""
import subprocess


def main():
    for value in ("0", "-1", "9223372036854775807", "60001"):
        result = subprocess.run(
            ["ros2", "run", "openarm_ik_ros", "openarm_ik_ros_node", "--ros-args",
             "-p", f"request_expiry_ms:={value}"],
            text=True,
            capture_output=True,
            timeout=5.0,
            check=False,
        )
        if result.returncode == 0:
            raise AssertionError(f"unsafe expiry {value} unexpectedly started the node")


if __name__ == "__main__":
    main()
