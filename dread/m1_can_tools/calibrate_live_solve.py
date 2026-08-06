"""Pure calibration math for the live hardstop wizard (no ROS, no CAN).

The live wizard (:mod:`m1_can_tools.calibrate_live`) enables one arm at strictly
zero torque so the operator can hand-move each joint to its mechanical hardstops
while the true encoder position streams. This module turns the captured raw
encoder values into a motor-map transform ``joint = dir * scale * motor + offset``
and is exercised entirely offline by ``test/test_calibrate_live_solve.py``.

Design (why this is foolproof where eyeballing RViz was not):

* **Offset is objective.** A mechanical hardstop is a viewpoint-free physical
  fact. Given the two stop raws and the URDF limits ``(lo, hi)``, the offset that
  lands the stops on the limits is exact (:func:`offset_from_stops`), and the
  measured stop-to-stop span is checked against the URDF span
  (:func:`span_mismatch`) to catch a joint whose URDF limit is a *soft* command
  limit rather than a true stop.
* **Direction is the only convention-dependent bit**, so it is never inferred
  from a camera view: it is seeded from the hardware-proven OpenArm identity and
  then confirmed per joint in the wizard's live verify loop, where a one-key
  flip recomputes the offset EXACTLY from the stored stop raws
  (:func:`recompute_offset`) -- no re-capture, fully reversible.
"""
from __future__ import annotations

import math
from typing import Dict, Tuple

# Mirror of m1_can_tools.calibrate.URDF_LIMITS / GRIPPER_TRAVEL, re-exported so a
# consumer needs only this pure module. Kept here to avoid importing the CAN
# wizard (which pulls socketcan) into unit tests.
URDF_LIMITS: Dict[str, list] = {
    "left": [
        (-3.4907, 1.3963), (-3.3161, 0.17453), (-1.5708, 1.5708),
        (0.0, 2.4435), (-1.5708, 1.5708), (-0.7854, 0.7854),
        (-1.5708, 1.5708), (0.0, 0.7854),
    ],
    "right": [
        (-1.3963, 3.4907), (-0.17453, 3.3161), (-1.5708, 1.5708),
        (0.0, 2.4435), (-1.5708, 1.5708), (-0.7854, 0.7854),
        (-1.5708, 1.5708), (-0.7854, 0.0),
    ],
}
GRIPPER_TRAVEL = {"left": 0.7854, "right": -0.7854}

# Joints whose front hardstop is unsafe to reach by hand (the elbow can drive the
# forearm into the body). They are captured at their ONE safe stop only; the
# offset is pinned to that end and the span self-check is skipped.
SINGLE_STOP_JOINTS = {4}
# The URDF limit that the SAFE (reachable) stop of a single-stop joint sits at.
# J4 URDF range is (0, 2.4435); the safe stop is the elbow fully OPEN/straight
# (the back stop), i.e. the URDF lower limit 0.0. (Confidence: medium -- the
# verify loop lets the operator nudge this end if the straight pose is offset.)
SINGLE_STOP_URDF_END = {4: "lo"}

SPAN_MISMATCH_TOL = 0.20  # rad; measured stop span vs URDF span before a warning
GRIP_MIN, GRIP_MAX = 0.25, 3.0  # plausible gripper motor travel (motor rad)


def offset_from_stops(dir_: int, lo: float, hi: float,
                      r_min: float, r_max: float) -> float:
    """Offset that maps the two mechanical stops onto the URDF limits.

    The joint transform is ``q = dir*raw + offset`` (scale is 1 for the seven
    rotary arm joints). ``q`` is monotonic in ``raw``: for ``dir = +1`` it
    increases with raw, so the URDF ``lo`` sits at the smaller raw (``r_min``);
    for ``dir = -1`` it decreases, so ``lo`` sits at the larger raw (``r_max``).
    """
    if dir_ not in (1, -1):
        raise ValueError(f"dir must be +1 or -1, got {dir_!r}")
    raw_at_lo = r_min if dir_ > 0 else r_max
    return lo - dir_ * raw_at_lo


def span_mismatch(lo: float, hi: float, r_min: float, r_max: float) -> float:
    """|measured stop-to-stop span - URDF span| in rad (0 == perfect)."""
    return abs((r_max - r_min) - (hi - lo))


def solve_arm_joint(dir_: int, lo: float, hi: float,
                    r_min: float, r_max: float) -> dict:
    """Full arm-joint result from a two-stop capture. ``scale`` is fixed at 1."""
    off = offset_from_stops(dir_, lo, hi, r_min, r_max)
    return {
        "dir": int(dir_),
        "scale": 1.0,
        "offset": off,
        "span_measured": r_max - r_min,
        "span_urdf": hi - lo,
        "span_mismatch": span_mismatch(lo, hi, r_min, r_max),
    }


def solve_single_stop_joint(dir_: int, lo: float, hi: float,
                            raw_stop: float, which_end: str) -> dict:
    """Offset for a joint captured at ONE safe hardstop (e.g. J4).

    ``which_end`` is ``"lo"`` or ``"hi"`` -- the URDF limit the safe stop is at.
    No span self-check is possible from a single stop.
    """
    end = lo if which_end == "lo" else hi
    return {
        "dir": int(dir_),
        "scale": 1.0,
        "offset": end - dir_ * raw_stop,
        "single_stop_end": which_end,
    }


def solve_gripper(raw_closed: float, raw_open: float,
                  desired_travel: float) -> dict:
    """Direction + scale + offset for a gripper from a closed/open measurement.

    ``desired_travel`` is the URDF finger-joint angle at the fully-open pose
    (``GRIPPER_TRAVEL[side]``); closed is the URDF zero. Matches the convention
    in ``m1_can_tools.calibrate.build_calibrated_map``.
    """
    delta = raw_open - raw_closed
    if not (GRIP_MIN <= abs(delta) <= GRIP_MAX):
        raise ValueError(
            f"gripper motor travel {abs(delta):.3f} rad is implausible "
            f"(expected {GRIP_MIN}..{GRIP_MAX}); re-capture closed/open")
    dir_ = 1 if desired_travel / delta > 0 else -1
    scale = abs(desired_travel / delta)
    offset = -dir_ * scale * raw_closed  # closed pose maps to URDF zero
    return {"dir": dir_, "scale": scale, "offset": offset}


def recompute_offset(dir_: int, cal_stops: dict) -> float:
    """Recompute an arm joint's offset for a (possibly flipped) direction.

    ``cal_stops`` is the ``calibration.stops`` block stored at capture:
    ``{lo, hi, r_min, r_max}`` for a two-stop joint, or
    ``{lo, hi, raw_stop, end}`` for a single-stop joint. This makes a direction
    flip in the verify loop EXACT (re-derived from the stored raws) rather than
    an approximate negate-about-a-point.
    """
    lo, hi = float(cal_stops["lo"]), float(cal_stops["hi"])
    if "raw_stop" in cal_stops:  # single-stop joint
        return solve_single_stop_joint(
            dir_, lo, hi, float(cal_stops["raw_stop"]),
            cal_stops["end"])["offset"]
    return offset_from_stops(
        dir_, lo, hi, float(cal_stops["r_min"]), float(cal_stops["r_max"]))


def verify_maps_to_urdf(dir_: int, scale: float, offset: float,
                        raw: float, expected_q: float, tol: float = 1e-6) -> bool:
    """True if ``dir*scale*raw + offset`` maps ``raw`` to ``expected_q``."""
    return abs(dir_ * scale * raw + offset - expected_q) <= tol
