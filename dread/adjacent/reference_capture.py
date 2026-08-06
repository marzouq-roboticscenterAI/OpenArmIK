"""Persist a passive multi-sample CAN capture at the displayed L reference.

DEPRECATED for calibration. This reads DISABLED motors, which report a FROZEN
encoder value (they do not track hand motion), so the captured "reference" does
not match the physical pose. Use the live wizard instead:
``ros2 run m1_can_tools m1_calibrate_live <side>`` (see deploy/agx-orin/CALIBRATION.md).
Kept only for passive bus inventory / telemetry snapshots.


Safety contract: this utility emits only Damiao ``0xCC`` state-refresh requests
to arbitration id ``0x7ff``.  It never enables, disables, zeroes, or sends MIT
setpoints.  The resulting JSON stores every raw sample and both possible
direction-sign offset candidates; it never edits the deployed motor map.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import os
import tempfile
import time
from pathlib import Path

from m1_can_tools.calibrate import (
    ADAPTER_PATH_HINTS,
    BusLocks,
    EXPECTED_IDS,
    MAX_ABS_VEL,
    MAX_TEMP_C,
    SIDES,
    _can_socket_preflight,
    _conflicting_processes,
    _iface_preflight,
    joint_name,
    side_map,
    validate_map_shape,
)
from m1_can_tools.motor_bus import MotorBus, load_map
from m1_can_tools.transport import SocketCanTransport


BASE_REFERENCE_Q = {
    "lift_joint": 0.62865,
    "openarm_left_joint1": 0.0,
    "openarm_left_joint2": 0.0,
    "openarm_left_joint3": -math.pi / 2,
    "openarm_left_joint4": math.pi / 2,
    "openarm_left_joint5": 0.0,
    "openarm_left_joint6": 0.0,
    "openarm_left_joint7": 0.0,
    "openarm_left_finger_joint1": 0.0,
    "openarm_right_joint1": 0.0,
    "openarm_right_joint2": 0.0,
    "openarm_right_joint3": math.pi / 2,
    "openarm_right_joint4": math.pi / 2,
    "openarm_right_joint5": 0.0,
    "openarm_right_joint6": 0.0,
    "openarm_right_joint7": 0.0,
    "openarm_right_finger_joint1": 0.0,
}

_ASD = 0.6  # allsign per-joint nudge (rad); must match calibration_reference._D
ALLSIGN_Q = {
    "lift_joint": 0.62865,
    "openarm_left_joint1": 0.0 + _ASD,
    "openarm_left_joint2": 0.0 - _ASD,
    "openarm_left_joint3": -math.pi / 2 + _ASD,
    "openarm_left_joint4": math.pi / 2 + _ASD,
    "openarm_left_joint5": 0.0 + _ASD,
    "openarm_left_joint6": 0.0 + _ASD,
    "openarm_left_joint7": 0.0 + _ASD,
    "openarm_left_finger_joint1": 0.7,
    "openarm_right_joint1": 0.0 + _ASD,
    "openarm_right_joint2": 0.0 + _ASD,
    "openarm_right_joint3": math.pi / 2 - _ASD,
    "openarm_right_joint4": math.pi / 2 + _ASD,
    "openarm_right_joint5": 0.0 + _ASD,
    "openarm_right_joint6": 0.0 + _ASD,
    "openarm_right_joint7": 0.0 + _ASD,
    "openarm_right_finger_joint1": -0.7,
}

REFERENCE_STAGES = {
    "base": {
        "q": BASE_REFERENCE_Q,
        "description": (
            "upper arms vertical; J3 mirrored +/-pi/2; elbows +pi/2; "
            "forearms horizontal/outward; wrists neutral; grippers closed"
        ),
    },
    "allsign": {
        "q": ALLSIGN_Q,
        "description": (
            "base L-pose with every joint nudged ~0.6 rad (signed to stay "
            "in-limit) and grippers OPEN; the delta vs base resolves each "
            "joint's direction sign and the gripper scale"
        ),
    },
    "left_j2_sign": {
        "q": {
            **BASE_REFERENCE_Q,
            "openarm_left_joint2": -math.pi / 6,
        },
        "description": (
            "base L-pose plus LEFT J2=-pi/6; left upper arm tilted 30deg "
            "toward URDF robot-front; every other joint unchanged"
        ),
    },
}


def _atomic_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "w") as fh:
            json.dump(data, fh, indent=2, sort_keys=True)
            fh.write("\n")
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


def _summary(samples: list[dict], info: dict, q_ref: float) -> dict:
    fields = ("pos", "vel", "torque", "t_mos", "t_rotor")
    avg = {key: sum(float(s[key]) for s in samples) / len(samples)
           for key in fields}
    raw = avg["pos"]
    scale = float(info.get("scale", 1.0))
    return {
        "count": len(samples),
        "mean": avg,
        "position_range": (
            max(float(s["pos"]) for s in samples)
            - min(float(s["pos"]) for s in samples)
        ),
        "candidate_offset_dir_plus": q_ref - scale * raw,
        "candidate_offset_dir_minus": q_ref + scale * raw,
        "map_at_capture": {
            "dir": int(info.get("dir", 1)),
            "scale": scale,
            "offset": float(info.get("offset", 0.0)),
        },
    }


def capture(map_path: Path, output_dir: Path, sample_count: int,
            reference_stage: str = "base") -> Path:
    reference = REFERENCE_STAGES[reference_stage]
    reference_q = reference["q"]
    motor_map = load_map(str(map_path))
    validate_map_shape(motor_map)
    for side, iface in SIDES.items():
        _iface_preflight(iface, ADAPTER_PATH_HINTS[side])
    _can_socket_preflight(SIDES.values())
    conflicts = _conflicting_processes(SIDES.values())
    if conflicts:
        raise RuntimeError("CAN owner conflict:\n  " + "\n  ".join(conflicts))

    stamp = dt.datetime.now(dt.timezone.utc)
    path = output_dir / (
        stamp.strftime("%Y%m%dT%H%M%S.%fZ")
        + f"-{reference_stage}-reference.json")
    session = {
        "schema": "m1-passive-reference-capture-v1",
        "captured_at": stamp.isoformat(),
        "status": "capturing",
        "safety": {
            "motors_enabled": False,
            "map_applied": False,
            "allowed_tx": "Damiao state refresh only: 0xCC to 0x7ff",
            "forbidden_tx": ["enable", "disable", "MIT setpoint", "set-zero"],
        },
        "reference": {
            "name": reference_stage,
            "description": reference["description"],
            "q": reference_q,
        },
        "interfaces": dict(SIDES),
        "source_map": str(map_path),
        "source_map_sha256": hashlib.sha256(map_path.read_bytes()).hexdigest(),
        "requested_samples_per_motor": sample_count,
        "samples": {"left": {}, "right": {}},
        "summary": {"left": {}, "right": {}},
        "validation_errors": [],
    }
    _atomic_json(path, session)  # reserve the timestamp before touching CAN

    buses = {}
    try:
        with BusLocks(SIDES.values()):
            for side, iface in SIDES.items():
                sm = side_map(motor_map, side)
                buses[side] = MotorBus(SocketCanTransport(iface, fd=False), sm)
                session["samples"][side] = {j: [] for j in sm}

            # Interleave sides each round so all motors represent the same still
            # interval. Retry partial rounds; every individual reply is retained.
            max_rounds = max(sample_count * 4, sample_count + 8)
            for _ in range(max_rounds):
                for side in ("right", "left"):
                    got = buses[side].telemetry_all_raw(timeout=0.05)
                    for name, fb in got.items():
                        dst = session["samples"][side].get(name)
                        if dst is not None and len(dst) < sample_count:
                            dst.append({k: fb[k] for k in (
                                "id", "err", "pos", "vel", "torque",
                                "t_mos", "t_rotor")})
                if all(
                    len(values) >= sample_count
                    for by_joint in session["samples"].values()
                    for values in by_joint.values()
                ):
                    break
                time.sleep(0.04)
    finally:
        # Close sockets only. MotorBus.close() would emit disable frames and is
        # intentionally forbidden by this capture's refresh-only contract.
        for bus in buses.values():
            bus.transport.close()

    errors = session["validation_errors"]
    for side in ("right", "left"):
        sm = side_map(motor_map, side)
        ids = set()
        for name, info in sm.items():
            values = session["samples"][side].get(name, [])
            if len(values) < sample_count:
                errors.append(
                    f"{name}: received {len(values)}/{sample_count} samples")
                continue
            ids.add(int(info["id"]))
            errs = [int(v["err"]) for v in values]
            if any(errs):
                errors.append(f"{name}: motor errors {errs}")
            vmax = max(abs(float(v["vel"])) for v in values)
            if vmax > MAX_ABS_VEL:
                errors.append(
                    f"{name}: max |velocity| {vmax:.5f} > {MAX_ABS_VEL}")
            hot = max(max(float(v["t_mos"]), float(v["t_rotor"]))
                      for v in values)
            if hot > MAX_TEMP_C:
                errors.append(
                    f"{name}: temperature {hot:.1f} C > {MAX_TEMP_C} C")
            session["summary"][side][name] = _summary(
                values, info, reference_q[name])
        if ids != EXPECTED_IDS:
            errors.append(
                f"{side}: responding mapped IDs {sorted(ids)}, expected 1..8")

    session["status"] = "valid" if not errors else "invalid"
    session["completed_at"] = dt.datetime.now(dt.timezone.utc).isoformat()
    session["direction_decision"] = {
        "applied": False,
        "reason": (
            "One static pose determines offsets only after a direction sign is "
            "chosen; it cannot measure sign. Preserve both candidates and take "
            "a second known pose before changing the map."
        ),
    }
    _atomic_json(path, session)
    if errors:
        raise RuntimeError(
            f"capture persisted but validation failed at {path}:\n  "
            + "\n  ".join(errors))
    return path


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--motor-map",
        default=str(Path.home() / ".config/m1/motor_map.yaml"))
    p.add_argument(
        "--output-dir",
        default=str(Path.home() / ".config/m1/calibration-sessions"))
    p.add_argument("--samples", type=int, default=8)
    p.add_argument(
        "--reference-stage", choices=tuple(REFERENCE_STAGES), default="base")
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    if args.samples < 3:
        raise SystemExit("--samples must be >= 3")
    path = capture(
        Path(args.motor_map), Path(args.output_dir), args.samples,
        args.reference_stage)
    print(f"PASS: passive reference capture saved to {path}")


if __name__ == "__main__":
    main()
