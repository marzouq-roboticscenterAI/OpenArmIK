"""Gated unit tests for the live hardstop calibration math (no ROS, no CAN)."""
import math

import pytest

from m1_can_tools.calibrate_live_solve import (
    GRIPPER_TRAVEL,
    URDF_LIMITS,
    offset_from_stops,
    recompute_offset,
    scale_from_stops,
    solve_arm_joint,
    solve_gripper,
    solve_single_stop_joint,
    span_mismatch,
    verify_maps_to_urdf,
)


def test_offset_lands_both_stops_on_urdf_limits_dir_plus():
    lo, hi = -1.5708, 1.5708
    # dir=+1, raw increases with joint angle; motor zero somewhere mid-range.
    r_min, r_max = -1.0, 2.1416  # raw span == urdf span (3.1416), offset 0.5708
    off = offset_from_stops(1, lo, hi, r_min, r_max)
    assert verify_maps_to_urdf(1, 1.0, off, r_min, lo)
    assert verify_maps_to_urdf(1, 1.0, off, r_max, hi)


def test_offset_lands_both_stops_on_urdf_limits_dir_minus():
    lo, hi = -1.5708, 1.5708
    r_min, r_max = -1.0, 2.1416
    off = offset_from_stops(-1, lo, hi, r_min, r_max)
    # dir=-1: lo sits at the LARGER raw, hi at the smaller.
    assert verify_maps_to_urdf(-1, 1.0, off, r_max, lo)
    assert verify_maps_to_urdf(-1, 1.0, off, r_min, hi)


def test_span_mismatch_zero_when_spans_equal_and_positive_otherwise():
    lo, hi = 0.0, 2.4435
    assert span_mismatch(lo, hi, -1.0, 1.4435) == pytest.approx(0.0)
    # A soft command limit inside the mechanical range reads a LARGER raw span.
    assert span_mismatch(lo, hi, -1.5, 1.4435) == pytest.approx(0.5)


def test_solve_arm_joint_reports_spans():
    r = solve_arm_joint(1, -1.5708, 1.5708, -1.0, 2.1416)
    assert r["scale"] == 1.0
    assert r["span_measured"] == pytest.approx(3.1416)
    assert r["span_urdf"] == pytest.approx(3.1416)
    assert r["span_mismatch"] == pytest.approx(0.0, abs=1e-4)


def test_solve_arm_joint_maps_unequal_encoder_span_to_full_urdf_range():
    lo, hi = -1.5708, 1.5708
    r_min, r_max = -0.75, 2.75
    r = solve_arm_joint(1, lo, hi, r_min, r_max)
    assert r["scale"] == pytest.approx((hi - lo) / (r_max - r_min))
    assert verify_maps_to_urdf(1, r["scale"], r["offset"], r_min, lo)
    assert verify_maps_to_urdf(1, r["scale"], r["offset"], r_max, hi)
    assert scale_from_stops(lo, hi, r_min, r_max) == pytest.approx(r["scale"])


def test_flip_recomputes_offset_exactly_from_stored_stops():
    lo, hi = -1.5708, 1.5708
    r_min, r_max = -1.0, 2.1416
    stops = {"lo": lo, "hi": hi, "r_min": r_min, "r_max": r_max}
    off_plus = recompute_offset(1, stops)
    off_minus = recompute_offset(-1, stops)
    # +1 puts lo at r_min; -1 puts lo at r_max -> both are consistent maps.
    assert verify_maps_to_urdf(1, 1.0, off_plus, r_min, lo)
    assert verify_maps_to_urdf(-1, 1.0, off_minus, r_max, lo)
    assert off_plus != pytest.approx(off_minus)


def test_single_stop_joint_pins_safe_end_only():
    lo, hi = 0.0, 2.4435  # J4
    raw_stop = -0.35
    r = solve_single_stop_joint(1, lo, hi, raw_stop, "lo")
    assert verify_maps_to_urdf(1, 1.0, r["offset"], raw_stop, lo)
    # recompute via the stored-stops path agrees.
    off = recompute_offset(1, {"lo": lo, "hi": hi, "raw_stop": raw_stop,
                               "end": "lo"})
    assert off == pytest.approx(r["offset"])


def test_single_stop_joint_preserves_commissioned_scale():
    r = solve_single_stop_joint(1, 0.0, 2.4435, -0.35, "lo", 0.987)
    assert r["scale"] == pytest.approx(0.987)
    assert verify_maps_to_urdf(1, r["scale"], r["offset"], -0.35, 0.0)


def test_gripper_sign_scale_and_zero():
    # Current OpenArm v1.0 URDF: closed=0, open=+0.044 m.
    r = solve_gripper(raw_closed=-1.10, raw_open=-0.01, desired_travel=GRIPPER_TRAVEL["left"])
    assert r["dir"] == 1
    assert 0.0 < r["scale"] < 3.0
    assert verify_maps_to_urdf(r["dir"], r["scale"], r["offset"], -1.10, 0.0)
    assert verify_maps_to_urdf(r["dir"], r["scale"], r["offset"], -0.01,
                               GRIPPER_TRAVEL["left"])


def test_gripper_negative_direction():
    # Both current URDF finger joints open in the positive prismatic direction.
    r = solve_gripper(raw_closed=0.0, raw_open=1.09, desired_travel=GRIPPER_TRAVEL["right"])
    assert r["dir"] == 1
    assert verify_maps_to_urdf(r["dir"], r["scale"], r["offset"], 1.09,
                               GRIPPER_TRAVEL["right"])


def test_gripper_rejects_implausible_travel():
    with pytest.raises(ValueError, match="implausible"):
        solve_gripper(0.0, 0.05, GRIPPER_TRAVEL["left"])  # < GRIP_MIN
    with pytest.raises(ValueError, match="implausible"):
        solve_gripper(0.0, 4.0, GRIPPER_TRAVEL["left"])   # > GRIP_MAX


def test_offset_rejects_bad_dir():
    with pytest.raises(ValueError, match="dir must be"):
        offset_from_stops(0, -1.0, 1.0, -1.0, 1.0)


def test_urdf_tables_are_consistent_shape():
    for side in ("left", "right"):
        assert len(URDF_LIMITS[side]) == 8
        for lo, hi in URDF_LIMITS[side]:
            assert hi > lo
    assert URDF_LIMITS["left"][7] == (0.0, 0.044)
    assert URDF_LIMITS["right"][7] == (0.0, 0.044)
