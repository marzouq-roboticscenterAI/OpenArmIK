#!/usr/bin/env python3
"""Validate the frozen bimanual URDF contract and all mesh package paths."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True, type=Path)
    parser.add_argument("--description-root", required=True, type=Path)
    args = parser.parse_args()
    root = ET.parse(args.urdf).getroot()
    world_joint = root.find("joint[@name='openarm_body_world_joint']")
    if world_joint is None or world_joint.attrib.get("type") != "fixed":
        raise SystemExit("world -> openarm_body_link0 fixed TF root is absent")
    if world_joint.find("parent").attrib.get("link") != "world" or \
            world_joint.find("child").attrib.get("link") != "openarm_body_link0":
        raise SystemExit("unexpected static TF root frames")
    joints = {joint.attrib["name"] for joint in root.findall("joint")}
    expected = {
        *(f"openarm_left_joint{index}" for index in range(1, 8)),
        *(f"openarm_right_joint{index}" for index in range(1, 8)),
        "openarm_left_finger_joint1",
        "openarm_right_finger_joint1",
    }
    missing = expected - joints
    if missing:
        raise SystemExit(f"missing generated joints: {sorted(missing)}")
    if "openarm_left_finger_joint2" not in joints or "openarm_right_finger_joint2" not in joints:
        raise SystemExit("generated mimic finger joints are absent")
    meshes = [element.attrib["filename"] for element in root.findall(".//mesh")]
    if not meshes:
        raise SystemExit("generated URDF has no mesh references")
    prefix = "package://openarm_description/"
    for mesh in meshes:
        if not mesh.startswith(prefix):
            raise SystemExit(f"unexpected mesh URI: {mesh}")
        path = args.description_root / mesh[len(prefix):]
        if not path.is_file():
            raise SystemExit(f"unresolved mesh: {mesh} -> {path}")


if __name__ == "__main__":
    main()
