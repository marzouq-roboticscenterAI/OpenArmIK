#!/usr/bin/env python3
"""Build the ROS motor map from OpenArm-C's saved ``arm_calib.txt``.

The direct controller's established bus contract is can0=robot-right and
can1=robot-left.  The input contains raw mechanical min/max values for motors
1..8, so no actuator movement or re-calibration is needed.
"""

import argparse
import copy
from datetime import datetime, timezone
from pathlib import Path

import yaml


BUS_TO_SIDE = {"can0": "right", "can1": "left"}
# The consolidated template is already side-correct. It supplies each physical
# motor's model/gains; arm_calib.txt supplies the saved mechanical ranges.
BUS_TO_TEMPLATE_SIDE = {"can0": "right", "can1": "left"}


def joint_name(side, motor_id):
    if motor_id == 8:
        return "openarm_%s_finger_joint1" % side
    return "openarm_%s_joint%d" % (side, motor_id)


def read_ranges(path):
    ranges = {}
    for line_no, line in enumerate(path.read_text().splitlines(), 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 4:
            raise ValueError("%s:%d: expected 'canX id min max'" % (path, line_no))
        bus, motor, low, high = fields
        motor = int(motor)
        low, high = float(low), float(high)
        if bus not in BUS_TO_SIDE or motor not in range(1, 9) or low >= high:
            raise ValueError("%s:%d: invalid bus/id/range" % (path, line_no))
        ranges[(bus, motor)] = (low, high)
    expected = {(bus, motor) for bus in BUS_TO_SIDE for motor in range(1, 9)}
    missing = sorted(expected - set(ranges))
    if missing:
        raise ValueError("missing saved ranges: %s" % missing)
    return ranges


def build(template, ranges, captured_at):
    result = {}
    for bus, side in BUS_TO_SIDE.items():
        template_side = BUS_TO_TEMPLATE_SIDE[bus]
        for motor in range(1, 9):
            source = copy.deepcopy(template[joint_name(template_side, motor)])
            destination = template[joint_name(side, motor)]
            low, high = ranges[(bus, motor)]
            lo, hi = map(float, destination["soft_limits"]["pos"])

            # Arm joints preserve their established sign. Gripper URDF limits
            # are mirrored: saved raw-min is closed (q=0), raw-max is open.
            direction = (-1 if side == "right" else 1) if motor == 8 \
                else int(destination["dir"])
            if motor == 8:
                travel = abs(hi - lo)
                scale = travel / (high - low)
                offset = -direction * scale * low
                stops = {"lo": lo, "hi": hi, "raw_stop": low,
                         "raw_open": high, "end": "closed"}
            else:
                scale = (hi - lo) / (high - low)
                expected_at_low = lo if direction > 0 else hi
                offset = expected_at_low - direction * scale * low
                stops = {"lo": lo, "hi": hi,
                         "r_min": low, "r_max": high}

            source["dir"] = direction
            source["scale"] = float(scale)
            source["offset"] = float(offset)
            source["soft_limits"] = copy.deepcopy(destination["soft_limits"])
            source["calibration"] = {
                "source": "saved_arm_calib_file",
                "captured_at": captured_at,
                "firmware_zero_changed": False,
                "stops": stops,
                "import": ("OpenArm-C arm_calib.txt; can0=robot-right, "
                           "can1=robot-left; no live recalibration"),
            }
            result[joint_name(side, motor)] = source
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--template", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    template = yaml.safe_load(args.template.read_text())
    captured = datetime.fromtimestamp(
        args.input.stat().st_mtime, timezone.utc).isoformat()
    result = build(template, read_ranges(args.input), captured)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(result, sort_keys=False))
    print("wrote %s from %s (no hardware accessed)" % (args.output, args.input))


if __name__ == "__main__":
    main()
