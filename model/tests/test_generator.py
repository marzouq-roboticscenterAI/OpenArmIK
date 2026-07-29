#!/usr/bin/env python3
import pathlib
import shutil
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

    clone = pathlib.Path(directory) / "dirty-source"
    subprocess.run(["git", "clone", "--quiet", "--no-hardlinks", str(root), str(clone)], check=True)
    tracked = clone / "assets/robot/openarm_v1.0/config/arm/kinematics.yaml"
    tracked.write_text(tracked.read_text() + "\n# dirty tracked mutation\n")
    command = [sys.executable, str(generator), str(clone), str(data), "--xacro", str(xacro),
               "--pythonpath", pythonpath, "--ament-prefix", ament_prefix]
    if subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
        raise SystemExit("generator accepted a dirty tracked canonical source")
    shutil.rmtree(clone)
    subprocess.run(["git", "clone", "--quiet", "--no-hardlinks", str(root), str(clone)], check=True)
    (clone / "untracked-generation-input.xacro").write_text("dirty")
    if subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
        raise SystemExit("generator accepted a dirty untracked canonical source")
