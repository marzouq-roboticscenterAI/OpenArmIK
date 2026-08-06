# CLAUDE.md — working notes for this repository

Orientation for an assistant picking this up cold. `codex.md` carries the
detailed per-phase history and the plans for unbuilt work; this file is the
shorter map plus the things that cost real time to learn.

## What this is

A bimanual OpenArm v1.0 control stack. C11 public ABIs with C++17 internals
behind opaque handles, layered `model` → `can` → `transport` → `commission` →
`control` → `runtime`, with a ROS 2 (Lyrical) adapter in
`ros2_ws/src/openarm_ik_ros` and a Boost.Beast web portal.

Two entry points:

- `./run.sh` — virtual stack. Simulated controller, no CAN. Safe.
- `./run-real.sh` — physical arms. Read-only observer; cannot move a motor.
- `./calib.sh` — hand-guided calibration GUI (GTK, C). **Superseded, see below.**
- `./dread.sh` — third-party calibration wizard. **Use this one.**

## Verification: run the real gate, not `colcon test`

```bash
bash scripts/build.sh --tests
```

That is the only command that verifies everything: native CTest for `can`,
`model`, `commission`, `transport`, plus the ROS suites, plus a gate asserting an
exact registered test count.

A bare `colcon test` after a non-testing build prints
`0 tests, 0 errors, 0 failures`. That is a **vacuous pass** — the test targets
were never built. This has produced a false all-clear more than once here.

If you add or remove a ROS test, move the expected count in `scripts/build.sh`
in the same commit or the gate fails.

## The C planner API, as it actually stands

Four planners, all in `control/include/openarm_control.h`, exposed through
`runtime/include/openarm_runtime_motion.h`:

| plan | request struct | what it does |
|---|---|---|
| `OA_PLAN_PAIRED_TCP` | `oa_paired_tcp_move` | independent target per arm, one atomic command |
| `OA_PLAN_CENTROID_TCP` | `oa_centroid_tcp_move` | midpoint between claws moves by a delta; both follow |
| `OA_PLAN_MIRRORED_TCP` | `oa_mirrored_tcp_move` | one lead claw's target, reflected onto the other |
| `OA_PLAN_CONVERGE_TCP` | `oa_converge_tcp_move` | both claws close on a point, **stopping on contact** |

Single-arm motion is `OA_PLAN_PAIRED_TCP` with one side held at its measured
pose, which is what the portal's per-arm buttons do.

**Converge is NOT removed.** A request to fold it into mirror is designed in
`codex.md` but unbuilt. The distinction that matters: converge's geometry really
is a mirror and is duplicated, but its six contact fields make it
force-terminated motion, which mirror has no way to express. Merging means
moving contact termination onto mirror, then converge becomes a wrapper.

## Hard constraints that break builds if ignored

**The runtime ABI is hash-frozen.** `runtime/include/openarm_runtime.h` is
byte-pinned against a frozen copy, and there is an exact exported-symbol
manifest at `runtime/tests/abi_v1/expected_symbols.txt`. Never widen a frozen
struct — `struct_size` changes and the freeze test fails. Additions go in
`openarm_runtime_motion.h`.

Centroid and mirrored are currently `static inline` adapters over the frozen V1
planner and export **zero** symbols. Anything needing the real-time monitor
cannot stay header-only, so it becomes an exported symbol and the manifest count
must move with it, in the same commit.

**Two clearance thresholds, deliberately different.** 25 mm planning gate,
10 mm real-time intervention floor. Collapsing them aborts motions the planner
legitimately accepted. Demo waypoints must clear the **25 mm** one; eight of the
original fourteen presets were rejected because they were tuned against 10 mm.

**The keepout capsules under-cover the real geometry** (tool 84.21 mm of
extent against a 75 mm radius; arm 59.84 against 50) and cannot simply be
raised: 0.060/0.085 makes the robot's own neutral pose fail and segfaults
`test_ros_contract`. The envelope is mis-shaped, not mis-sized; it needs
oriented boxes.

**Do not map every `STOPPED` event to `COMPLETED`.** That was tried and masked
real failures: pick-place reported all seven steps complete while the arms sat
8 cm off target. `complete_on_contact()` is narrow on purpose.

## Physical hardware facts, measured

- **Eight motors per arm**, not seven: `0x01`–`0x07` joints, `0x08` gripper.
  Replies at `send_id + 0x10`.
- Motor IDs are unique **within** a bus only. A stock pair puts the same eight
  IDs on both interfaces, so IDs cannot identify which arm is which.
- `PMAX` 12.5, `VMAX` 45, `TMAX` 54, gear ratio 9.0 (DM8009 on joint 1).
  Position quantises to **3.8148e-4 rad** per code across every type in this
  family, so angles decode without knowing the motor type. Velocity and torque
  do not.
- Refresh-status is `7FF#<id>00CC...`; register query is `7FF#<id>0033<rid>...`.

### Left/right identification: four methods falsified

1. **Motor IDs** — identical on both buses.
2. **Registers** — `DIRECTION` reads +1.0 on all 16 motors; serial bytes match.
3. **Absolute joint angles vs mirrored limits** — got a real arm backwards.
   Motor zeros are not commissioned to URDF zeros, so absolute angles carry no
   side information. Applying the manifest's `q_scale`/`q_offset` does not
   rescue it.
4. **Upstream convention** — `openarm.bimanual.launch.py` declares
   `right_can_interface:=can0`, `left:=can1`. That is a default, not a
   measurement, and it is the reverse of how this rig is wired.

The observer therefore reports a **guess** with `confidence: "low"` and offers
**Swap arms**. The only sound automatic method left is a motion-delta test,
where the unknown zero cancels in a difference.

### A disabled DaMiao motor reports a FROZEN encoder value

This is the single most expensive lesson here. `calib.sh` was built to record
joint ranges from unpowered motors and cannot work: a 180° sweep recorded as
13°. DREAD's README documents the same failure independently.

The working approach — DREAD's — is to **compliant-enable at kp = kd = tau = 0**.
Powered, so the position observer runs and streams true angles; commanding
nothing, so the arm stays limp and back-drivable. Use `./dread.sh`.

## Portal notes

- Real mode is `launch_web_portal.sh --real`, a flag rather than a fork. It
  swaps in `openarm_real.launch.xml`, which **omits** `openarm_ik_ros_node`:
  that and the observer both publish `/joint_states`, and running both renders a
  blend of a real arm and a simulated one.
- Real mode loads the **full** URDF, not the Stage-A visualization one. Stage-A
  rewrites the prismatic finger joints to `fixed`, and a fixed joint cannot move
  whatever is published, so the claws render dead.
- The observer publishes `/joint_states` **even while passive**. Going silent
  makes the robot vanish from RViz, and an idle stack then looks broken.
- The RViz view in the browser is real RViz pixels over MJPEG, and it is
  interactive: pointer events are replayed with `XSendEvent` addressed to the
  render widget. XTEST was tried and is wrong — it drives the shared cursor, so
  it needs RViz topmost, and in normal use the browser is on top.

## Traps that wasted time here

- `pkill -f <pattern>` matches the assistant's own shell. Use a bracket:
  `pgrep -f "openarm_real[_]observer"`.
- `stdout` is block-buffered to a file. A missing banner is not evidence a
  script died; `bash -x` is.
- Sourcing ROS setup files under `set -u` **silently aborts the shell** —
  they reference unbound variables and `|| true` does not catch it. This made
  `calib.sh` exit after its build with no window and no error.
- The build lease is held by a running `run-real.sh`. Check what holds it before
  killing anything; I killed a live user session doing this.
- gs_usb rejects `restart-ms`. `setup_can_interfaces.sh` attempts attribute
  combinations and takes the first accepted.

## Style

Comments explain *why*, especially where a value is tuned, a constraint is
non-obvious, or a simpler approach was tried and failed. Prefer recording a
falsified hypothesis over deleting it — several were re-derived more than once
before they were written down.
