#!/usr/bin/env python3
import pathlib
import subprocess
import sys
import tempfile

generator, description_root, expected = map(pathlib.Path, sys.argv[1:])
with tempfile.TemporaryDirectory() as directory:
    generated = pathlib.Path(directory) / "model.inc"
    subprocess.run([sys.executable, str(generator), str(description_root), str(generated)], check=True)
    if generated.read_bytes() != expected.read_bytes():
        raise SystemExit("generated model differs from checked-in immutable data")
    if "0.186" not in generated.read_text():
        raise SystemExit("current hand_tcp offset regressed to stale zero-offset example")
