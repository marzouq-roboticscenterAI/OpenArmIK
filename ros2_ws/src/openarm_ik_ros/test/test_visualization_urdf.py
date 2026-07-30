#!/usr/bin/env python3
"""Prove the Stage-A visualization URDF is a narrow canonical-model overlay."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import subprocess
import tempfile
import xml.etree.ElementTree as ET


CANONICAL_SHA256 = "dd4b5d5fa0c050b78922bc95850ea65bb15721cb259030cccf106a953f171c55"
FINGER_JOINTS = {
    "openarm_left_finger_joint1": (
        "openarm_left_link7", "openarm_left_right_finger", "0.0 -0.005 0.1025",
        "0 -1 0", None),
    "openarm_left_finger_joint2": (
        "openarm_left_link7", "openarm_left_left_finger", "0.0 0.005 0.1025",
        "0 1 0", "openarm_left_finger_joint1"),
    "openarm_right_finger_joint1": (
        "openarm_right_link7", "openarm_right_right_finger", "0.0 -0.005 0.1025",
        "0 -1 0", None),
    "openarm_right_finger_joint2": (
        "openarm_right_link7", "openarm_right_left_finger", "0.0 0.005 0.1025",
        "0 1 0", "openarm_right_finger_joint1"),
}
FINGER_LINKS = {
    "openarm_left_left_finger",
    "openarm_left_right_finger",
    "openarm_right_left_finger",
    "openarm_right_right_finger",
}


def serialized(element):
    return ET.tostring(element, encoding="unicode")


def semantic(element):
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        (element.text or "").strip(),
        tuple(semantic(child) for child in element),
    )


def vector(text):
    return tuple(float(value) for value in text.split())


def identity():
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def multiply(left, right):
    return tuple(tuple(sum(left[row][index] * right[index][column]
                           for index in range(4))
                       for column in range(4))
                 for row in range(4))


def translation(xyz):
    matrix = [list(row) for row in identity()]
    for index, value in enumerate(xyz):
        matrix[index][3] = value
    return tuple(tuple(row) for row in matrix)


def rpy_rotation(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0.0),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0.0),
        (-sp, cp * sr, cp * cr, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def axis_rotation(axis, angle):
    x, y, z = axis
    cosine, sine = math.cos(angle), math.sin(angle)
    one_minus = 1.0 - cosine
    return (
        (cosine + x*x*one_minus, x*y*one_minus - z*sine,
         x*z*one_minus + y*sine, 0.0),
        (y*x*one_minus + z*sine, cosine + y*y*one_minus,
         y*z*one_minus - x*sine, 0.0),
        (z*x*one_minus - y*sine, z*y*one_minus + x*sine,
         cosine + z*z*one_minus, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def joint_transform(joint, positions):
    origin = joint.find("origin")
    transform = multiply(
        translation(vector(origin.attrib.get("xyz", "0 0 0"))),
        rpy_rotation(vector(origin.attrib.get("rpy", "0 0 0"))),
    )
    joint_type = joint.attrib["type"]
    position = positions.get(joint.attrib["name"], 0.0)
    if joint_type in ("revolute", "continuous"):
        transform = multiply(transform, axis_rotation(vector(joint.find("axis").attrib["xyz"]),
                                                       position))
    elif joint_type == "prismatic":
        axis = vector(joint.find("axis").attrib["xyz"])
        transform = multiply(transform, translation(tuple(value * position for value in axis)))
    elif joint_type != "fixed":
        raise AssertionError(f"unsupported joint type {joint_type}")
    return transform


def frame_transforms(root, positions):
    children = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        children.setdefault(parent, []).append(joint)
    transforms = {"world": identity()}
    pending = ["world"]
    while pending:
        parent = pending.pop()
        for joint in children.get(parent, []):
            child = joint.find("child").attrib["link"]
            transforms[child] = multiply(transforms[parent], joint_transform(joint, positions))
            pending.append(child)
    return transforms


def compare_kinematics(canonical, visualization):
    arm_joints = [
        joint for joint in canonical.findall("joint")
        if joint.attrib["name"] not in FINGER_JOINTS and joint.attrib["type"] == "revolute"
    ]
    lower = {joint.attrib["name"]: float(joint.find("limit").attrib["lower"])
             for joint in arm_joints}
    upper = {joint.attrib["name"]: float(joint.find("limit").attrib["upper"])
             for joint in arm_joints}
    postures = [{name: 0.0 for name in lower}, lower, upper]
    generator = random.Random(0xA11CE)
    for _ in range(12):
        postures.append({name: generator.uniform(lower[name], upper[name]) for name in lower})
    for posture in postures:
        canonical_frames = frame_transforms(canonical, posture)
        visualization_frames = frame_transforms(visualization, posture)
        assert canonical_frames.keys() == visualization_frames.keys()
        for frame in canonical_frames:
            error = max(abs(canonical_frames[frame][row][column]
                            - visualization_frames[frame][row][column])
                        for row in range(4) for column in range(4))
            assert error < 1e-12, (frame, error)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--canonical", required=True, type=Path)
    parser.add_argument("--visualization", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--description-root", required=True, type=Path)
    parser.add_argument("--generator", required=True, type=Path)
    parser.add_argument("--cmake", required=True, type=Path)
    args = parser.parse_args()

    canonical_bytes = args.canonical.read_bytes()
    visualization_bytes = args.visualization.read_bytes()
    assert hashlib.sha256(canonical_bytes).hexdigest() == CANONICAL_SHA256
    notice = (
        f"Derived from canonical SHA-256\n       {CANONICAL_SHA256}."
    ).encode()
    assert notice in visualization_bytes

    with tempfile.TemporaryDirectory() as directory:
        regenerated = Path(directory) / "visualization.urdf"
        subprocess.run([
            str(args.cmake), f"-DINPUT_URDF={args.canonical}",
            f"-DOUTPUT_URDF={regenerated}", "-P", str(args.generator),
        ], check=True)
        assert regenerated.read_bytes() == visualization_bytes
        assert hashlib.sha256(args.canonical.read_bytes()).hexdigest() == CANONICAL_SHA256

    canonical = ET.fromstring(canonical_bytes)
    visualization = ET.fromstring(visualization_bytes)
    canonical_links = {link.attrib["name"]: link for link in canonical.findall("link")}
    visualization_links = {
        link.attrib["name"]: link for link in visualization.findall("link")
    }
    canonical_joints = {
        joint.attrib["name"]: joint for joint in canonical.findall("joint")
    }
    visualization_joints = {
        joint.attrib["name"]: joint for joint in visualization.findall("joint")
    }
    assert len(canonical_links) == len(visualization_links) == 26
    assert len(canonical_joints) == len(visualization_joints) == 25
    assert canonical_links.keys() == visualization_links.keys()
    assert canonical_joints.keys() == visualization_joints.keys()

    for name, link in canonical_links.items():
        derived = visualization_links[name]
        if name not in FINGER_LINKS:
            assert serialized(link) == serialized(derived), name
            continue
        assert len(link.findall("inertial")) == 1
        assert not derived.findall("inertial")
        canonical_noninertial = [semantic(child) for child in link if child.tag != "inertial"]
        derived_children = [semantic(child) for child in derived]
        assert canonical_noninertial == derived_children, name

    for name, joint in canonical_joints.items():
        derived = visualization_joints[name]
        if name not in FINGER_JOINTS:
            assert serialized(joint) == serialized(derived), name
            continue
        parent, child, xyz, axis, mimic = FINGER_JOINTS[name]
        assert joint.attrib["type"] == "prismatic"
        assert joint.find("parent").attrib == {"link": parent}
        assert joint.find("child").attrib == {"link": child}
        assert joint.find("origin").attrib == {"rpy": "0 0 0", "xyz": xyz}
        assert joint.find("axis").attrib == {"xyz": axis}
        assert joint.find("limit").attrib == {
            "effort": "333", "lower": "0.0", "upper": "0.044", "velocity": "10.0",
        }
        if mimic is None:
            assert joint.find("mimic") is None
        else:
            assert joint.find("mimic").attrib == {"joint": mimic}
        assert derived.attrib["type"] == "fixed"
        assert derived.find("parent").attrib == joint.find("parent").attrib
        assert derived.find("child").attrib == joint.find("child").attrib
        assert derived.find("origin").attrib == joint.find("origin").attrib
        assert [child.tag for child in derived] == ["parent", "child", "origin"]
        assert derived.find("axis") is None
        assert derived.find("limit") is None
        assert derived.find("mimic") is None

    compare_kinematics(canonical, visualization)

    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    assert manifest["schema"] == 1
    assert manifest["upstream"] == {
        "repository": "https://github.com/enactic/openarm_description",
        "commit": "6c7b720f1ba48e8bafa3a3dc752c45f397b42221",
        "license": "Apache-2.0",
    }
    meshes = manifest["meshes"]
    assert len(meshes) == 11
    assert manifest["total_bytes"] == 2498724
    assert manifest["total_triangles"] == 49956
    assert sum(mesh["bytes"] for mesh in meshes) == manifest["total_bytes"]
    assert sum(mesh["triangles"] for mesh in meshes) == manifest["total_triangles"]
    routes = {mesh["route"] for mesh in meshes}
    assert len(routes) == len(meshes)
    collision_sources = {
        mesh.attrib["filename"]
        for link in visualization.findall("link")
        for collision in link.findall("collision")
        for mesh in collision.findall("geometry/mesh")
    }
    assert collision_sources == {mesh["source"] for mesh in meshes}
    for mesh in meshes:
        assert mesh["route"].startswith("/viewer/mesh/")
        source = mesh["source"]
        prefix = "package://openarm_description/"
        assert source.startswith(prefix)
        asset = args.description_root / source[len(prefix):]
        payload = asset.read_bytes()
        assert len(payload) == mesh["bytes"]
        assert hashlib.sha256(payload).hexdigest() == mesh["sha256"]
        assert len(payload) == 84 + 50 * mesh["triangles"]


if __name__ == "__main__":
    main()
