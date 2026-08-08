# OpenArm v1.0 live encoder calibration

This procedure calibrates this repository's current bimanual OpenArm v1.0 URDF
against the DaMiao rotary encoders. It is manual, one selected arm at a time;
`both` performs the two arms sequentially.

## Safety gate

Do not start the live wizard unless all of the following are true:

- the source of any visible spark has been found and repaired;
- every expected motor replies to a read-only scan (IDs 1 through 8 on both
  `can0` and `can1`);
- the arm is supported, the workspace is clear, and the hardware power cut-off
  is in the operator's hand;
- no portal, observer, ros2_control process, or other CAN owner is running.

The wizard is not read-only after its preflight. It energizes motors with zero
position/velocity gains and zero requested torque so the joints remain limp and
the encoders update while the operator moves them. That is deliberately safer
than motor-driven stop discovery, but it is not a safety-rated state.

## Start

From `/home/signalprocessing-dev/OpenArmIK`:

```bash
bash dread.sh left       # robot-left on can1
bash dread.sh right      # robot-right on can0
bash dread.sh both       # left, then right
```

If a CAN interface is down, the script calls
`bash scripts/setup_can_interfaces.sh`, which asks for sudo only to configure
the interfaces. Do not run `dread.sh` itself with sudo.

The script launches the installed current OpenArm v1.0 URDF in RViz. It then
prints the resolved bus, performs a status-only 8/8 preflight, and asks you to
type the selected side before any motor is enabled. A missing reply aborts before
an enable or MIT frame is transmitted.

## Capture J1 through J7

After zero-gain enable, move one named joint at a time, gently by hand:

1. Hold the first mechanical stop without forcing it and press Enter.
2. Hold the opposite stop and press Enter.
3. Confirm that RViz follows the real joint while it moves.

For every two-stop joint, DREAD measures the encoder span as a binary64 value and
fits both `scale` and `offset` so those encoder endpoints map to the complete
URDF range. It does not estimate an angle from appearance.

J4 is intentionally different: capture only the safe fully straight/open stop.
Do not drive the elbow toward the body to seek the other stop. Its previously
commissioned scale is retained and the safe endpoint determines its offset.

## Capture the gripper

Hold the gripper fully closed and press Enter, then fully open and press Enter.
The measured motor sweep is mapped to the current URDF's full prismatic range:

```text
closed = 0.000 m
open   = 0.044 m
```

If it cannot be back-driven, retry. Skipping preserves the prior mapping; it
does not invent a new scale. Only skip when the prior map is already known to be
the OpenArm v1.0 0–44 mm mapping.

## Live verification

After both map copies are written atomically, the prompt accepts:

| command | action |
|---|---|
| `t` | print raw encoder, mapped URDF value, limit, and in-range result |
| `flip 3` | reverse J3 and recompute its offset from stored stops |
| `nudge 0.05 6` | add 0.05 rad to J6's offset |
| `stops 3` | recapture both stops and refit J3 |
| `done` | disable motors, close RViz, and exit |

Judge direction while looking along the joint axis. Verify pitch joints first,
then roll joints. Ctrl-C at any point disables the enabled motors and releases
the CAN bus.

The deployed map is `~/.config/m1/motor_map.yaml`; the repository copy is
`dread/config/motor_map.openarm_v10.yaml`. Before replacing either, the wizard
creates a timestamped backup under
`~/.config/m1/calibration-backups/` and prints the rollback path.

## Hardware-free rehearsal

Run the simulation command in `dread/README.md`. It exercises both arms, all
joints, the 44 mm grippers, solving, and validation without opening SocketCAN or
writing either map.
