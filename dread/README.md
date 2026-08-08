# DREAD calibration adapted for OpenArm v1.0

This directory contains the collected DREAD live hard-stop wizard, now adapted
to this repository's pinned OpenArm v1.0 description. The active entry point is
the repository-root `dread.sh`; do not launch the collected M1/Ranger commands
directly.

## Active OpenArm configuration

- `config/motor_map.openarm_v10.yaml` is the only seed/repository map used by
  `dread.sh`.
- Each arm has DaMiao IDs 1 through 8, with feedback IDs `0x11` through `0x18`.
- J1/J2 use DM8009, J3/J4 use DM4340, and J5/J6/J7/gripper use DM4310. The
  validator accepts the corresponding upstream legacy model aliases.
- Joint names and limits match the installed `openarm_v10_bimanual.urdf`. Both
  fingers are prismatic, closed at 0 m and fully open at 0.044 m.
- On this commissioned pair, robot-left is `can1` and robot-right is `can0`.
  `dread.sh` exports that mapping explicitly because both channels belong to one
  dual-channel DM-USB2FDCAN and cannot be distinguished by USB serial alone.

The older `motor_map.m1robot.yaml`, `motor_map.example.yaml`, and M1 hard-stop
JSON are retained as provenance/reference assets. They describe another robot,
including a rotary gripper, and are not loaded by `dread.sh`.

## What the live wizard does

The wizard first sends only DaMiao `0xCC` status-refresh requests. It refuses to
energize anything unless all eight expected motors on every selected bus reply.
It then primes and continually sends MIT frames with `kp=0`, `kd=0`, and
`tau=0`, enables the selected arm, and keeps it limp/back-drivable while reading
the built-in rotary encoders.

For each two-stop joint, the transform is solved from the measured encoder
endpoints:

```text
q_urdf = direction * scale * q_encoder + offset
scale  = (urdf_max - urdf_min) / (encoder_max - encoder_min)
```

This maps the measured sweep to the full URDF range instead of assuming one
motor radian equals one joint radian. J4 retains its existing scale because only
its safe straight/open stop is captured. The gripper's closed/open motor sweep
is mapped to the full 0 to 0.044 m prismatic range.

Every normal exit, Ctrl-C, SIGTERM, or exception disables all selected motors.
The root launcher also terminates its RViz child on exit.

## Run

Build the workspace first, then follow `docs/CALIBRATION.md`:

```bash
./scripts/build.sh --incremental --jobs 1
bash dread.sh left       # or right / both
```

For a hardware-free rehearsal:

```bash
set +u
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
set -u
PYTHONPATH="$PWD/dread${PYTHONPATH:+:$PYTHONPATH}" \
M1_MOTOR_MAP="$PWD/dread/config/motor_map.openarm_v10.yaml" \
M1_REPO_MAP="$PWD/dread/config/motor_map.openarm_v10.yaml" \
python3 -m m1_can_tools.calibrate_live both --transport sim --yes --dry-run
```

The live calibration tool remains Python because DREAD itself is an imported
third-party tool. The production kinematics, control, collision, runtime, CAN
codec, commissioning interface, and portal backend in this repository remain
C/C++ with public C APIs.

## Verification

```bash
PYTHONPATH="$PWD/dread${PYTHONPATH:+:$PYTHONPATH}" \
python3 -m unittest discover -s dread/test -p 'test_*.py' -v
```

The suite covers the exact full-range fit, unequal encoder/URDF spans,
single-safe-stop behavior, 44 mm grippers, map validation, model aliases, and
atomic calibration math. It never opens a real CAN interface.
