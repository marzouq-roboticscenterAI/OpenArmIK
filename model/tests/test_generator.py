#!/usr/bin/env python3
import pathlib
import subprocess
import sys
import tempfile

generator, root, expected_data, expected_urdf, xacro = map(pathlib.Path, sys.argv[1:6])
pythonpath, ament_prefix = sys.argv[6:8]
with tempfile.TemporaryDirectory() as directory:
    data = pathlib.Path(directory) / "model.inc"
    urdf = pathlib.Path(directory) / "model.urdf"
    subprocess.run([sys.executable, str(generator), str(root), str(data), "--urdf-output", str(urdf),
                    "--xacro", str(xacro), "--pythonpath", pythonpath, "--ament-prefix", ament_prefix], check=True)
    if data.read_bytes() != expected_data.read_bytes():
        raise SystemExit("regenerated model data differs")
    if urdf.read_bytes() != expected_urdf.read_bytes():
        raise SystemExit("regenerated flattened URDF differs")
