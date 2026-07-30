#!/usr/bin/env python3
"""Prove the visualization executable has no CAN dependency or PF_CAN syscall."""
import argparse
import os
import re
import shutil
import subprocess
import tempfile


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True)
    parser.add_argument("--session-library", required=True)
    args = parser.parse_args()
    undefined = subprocess.run(
        ["nm", "-u", args.session_library], check=True, text=True, capture_output=True
    ).stdout
    forbidden = re.findall(
        r"\b(?:oa_controller_|oa_motion_plan_|oa_manifest_(?!runtime))\w+", undefined
    )
    if forbidden:
        raise SystemExit(f"session bypasses OpenArm::Runtime: {sorted(set(forbidden))}")
    if "oa_runtime_create" not in undefined or "oa_runtime_snapshot_get" not in undefined:
        raise SystemExit(f"session does not consume the runtime facade:\n{undefined}")
    linked = subprocess.run(["ldd", args.executable], check=True, text=True, capture_output=True).stdout
    if re.search(r"(?:openarm_can|socketcan|libcan)", linked, re.IGNORECASE):
        raise SystemExit(f"CAN linkage found:\n{linked}")
    strace = shutil.which("strace")
    if strace is None:
        raise SystemExit("strace is required to verify PF_CAN syscall isolation")
    with tempfile.NamedTemporaryFile() as trace:
        result = subprocess.run(
            [strace, "-f", "-e", "trace=socket", "-o", trace.name,
             "timeout", "1", args.executable],
            env=os.environ.copy(), text=True, capture_output=True, check=False)
        if result.returncode not in (0, 124):
            raise SystemExit(f"node failed during syscall isolation check:\n{result.stderr}")
        trace.seek(0)
        content = trace.read().decode("utf-8", errors="replace")
    if "AF_CAN" in content or "PF_CAN" in content:
        raise SystemExit(f"PF_CAN syscall found:\n{content}")


if __name__ == "__main__":
    main()
