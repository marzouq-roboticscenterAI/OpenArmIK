# DREAD — manual arm-motor calibration, collected

Copies of the code behind the **manual hardstop calibration + live RViz correction**
procedure. Collected 2026-08-05. Nothing here is wired into a FREAKY variant; this is
the working set gathered in one place.

Sourced from `~/Downloads/m1-ros2` (git branch `real-hardware-deployment`), which is
the canonical tree — verified byte-identical to the `~/Desktop/Scare-M1/reference/`
duplicates, so none of these came from a reference copy. One exception, noted below.

## The script

`m1_can_tools/calibrate_live.py` — `m1_calibrate_live`, the whole procedure in one
guided wizard. Both halves you're looking for are in here:

- **Manual range capture.** Compliant-enables the arm at strictly zero torque
  (kp=kd=tau=0), so it goes limp and freely back-drivable while the encoder streams
  *true* live position. You hand-move each joint J1–J7 to both mechanical hardstops
  and hold; it captures the raw at each and solves `offset` + a URDF span check.
  J4 (elbow) captures only its one safe stop. Gripper: fully-closed → fully-open
  gives sign + scale.
- **Live RViz offset/direction correction.** After writing the map it keeps streaming
  and drops you at a `verify>` prompt: `t` (live table), `flip 3`, `nudge 0.05 6`,
  `stops 3`, `done`. Each command rewrites both map files immediately, no restart.
  Flips are exact and reversible because they recompute from the stored hardstops.

Run it in two terminals — see `docs/CALIBRATION.md` for the full procedure:

```bash
ros2 launch m1_bringup live_calibration.launch.py   # terminal 1: RViz
ros2 run m1_can_tools m1_calibrate_live left        # terminal 2: wizard (left|right|both)
```

## Layout

| path | what |
|---|---|
| `m1_can_tools/` | the wizard + its complete import closure (`calibrate_live_solve`, `calibrate`, `motor_bus`, `transport`, `dm_protocol`) |
| `launch/live_calibration.launch.py` | the RViz side (robot_state_publisher + RViz) |
| `config/motor_map.m1robot.yaml` | committed twin of the deployed map; per-joint `dir` / `offset` / `scale` + `calibration:` blocks stamped `source: live_hardstop_wizard` |
| `config/motor_map.example.yaml` | blank template with the whole schema documented inline — **start here when porting**, not from the m1robot map |
| `config/openarm_hardstops.m1robot.json` | per-motor captured spans, read by `calibrate.py` |
| `test/` | `test_calibrate_live_solve.py`, `test_calibrate.py` — pure math, no hardware |
| `docs/CALIBRATION.md` | the operator procedure, incl. Step C direction-confirmation technique |
| `packaging/` | `setup.py` (console_scripts entry points), `setup.cfg`, `package.xml`, `resource/m1_can_tools` |
| `adjacent/` | the other two calibration paths — see below |

`adjacent/` is deliberately separate:

- `calib.c` / `calib.h` + `import_arm_calib.py` — the **automated** path. Motor-drives
  each joint into its stops via stall detection and writes `~/arm_calib.txt`
  (`can0 1 -3.83974 1.04772` …); the importer converts that into a motor map.
  Motorized, not hand-moved. `import_arm_calib.py` exists only in
  `~/Consolidation/VR-Control/tools/`, so that one is not from the m1-ros2 tree.
- `reference_capture.py` / `passive_joint_state.py` — **superseded, do not calibrate
  with these.** Their own docstrings say why: a *disabled* Damiao motor reports a
  **frozen** encoder value, so these read dead data and RViz never matched. The
  near-zero `raw_span` entries in `~/.config/m1/passive_range.json` are that
  signature. Kept for passive bus inventory / telemetry only.

## Porting this to another Damiao arm

The protocol layer transfers as-is — `dm_protocol.LIMITS` already carries
quantization for 15 Damiao models (DM3507, DM4310/`_48V`/P, DM4340/`_48V`/`_V20`,
DM6006, DM8006, DM8009, DM10010/L, DMH3510, DMH6215, DMG6220). An unlisted model
raises `unknown DM model` rather than silently mis-scaling.

Beyond RViz + a URDF, they supply: ROS 2 with `rclpy` + `sensor_msgs`,
`robot_state_publisher` (and **no** `joint_state_publisher` — exactly one publisher
owns `/joint_states`), PyYAML, `iproute2`/`udevadm` on PATH, and **`python-can`**,
which `transport.py` imports lazily so a missing install fails at motor-enable
rather than at startup.

Edits required, in order of how badly they bite:

1. **`URDF_LIMITS` and `GRIPPER_TRAVEL` are defined TWICE** — `calibrate.py:157`/`153`
   and `calibrate_live_solve.py:31`/`43`. The wizard reads *calibrate.py's* copy;
   the unit tests read *solve's*. Edit only the solve copy and you get 21 green
   tests while the wizard calibrates against the wrong limits. Change both.
   Nothing parses a URDF anywhere — these tables are hand-maintained mirrors.
2. **Joint names are hardcoded** — `joint_name()` at `calibrate.py:171` and the
   `/joint_states` name-building at `calibrate_live.py:225`
   (`openarm_{side}_joint{1..7}`, `finger_joint1`→`finger_joint2`, plus `lift_joint`).
   A name mismatch means `robot_state_publisher` silently drops the joint and the
   model never moves — indistinguishable from the frozen-encoder bug this tool
   exists to prevent. Check names first when nothing moves.
3. **`_ARM_CAN_DRIVER = "peak_usb"`** + `ADAPTER_PATH_HINTS = {"right": ".4.4",
   "left": ".4.3"}` (`calibrate.py:52`/`59`). `_iface_preflight` runs in the
   constructor and raises `expected the peak_usb driver` before any motor is
   enabled. Non-PCAN adapters need this relaxed and side resolution rewritten for
   their wiring.
4. **`EXPECTED_MODELS`** (`calibrate.py:147`) is a fixed tuple in motor-id order
   — `DM8009, DM8009, DM4340_V20, DM4340_V20, DM4310P ×4`, `EXPECTED_IDS = 1..8`.
   A different model lineup trips `validate_map_shape`.
5. **`packaging/setup.py` entry points reference two modules not in this folder** —
   `hwconfig_node` (the config web page) and `passive_joint_state`. Delete those
   two `console_scripts` lines, or the built `m1_hwconfig` /
   `m1_passive_joint_state` commands ImportError. `m1_calibrate` and
   `m1_calibrate_live` are the ones that matter.
6. Move `packaging/*` back to the package root before `colcon build` — `setup.py`
   expects `resource/m1_can_tools`, `package.xml`, and `config/` beside it. The
   `web/` glob resolving to nothing is harmless.

Map paths need no edits: `M1_MOTOR_MAP`, `M1_REPO_MAP`, `M1_BACKUP_ROOT` are all
env-overridable. `calibrate.py`'s `--repo-map` / `--hardstops` defaults are
`Path.cwd()`-relative, so run it from the workspace root or pass the flags.

**Rehearse before powering anything.** `--transport sim --yes` runs the entire
wizard against fake motors and `--dry-run` solves + validates without writing maps;
`docs/CALIBRATION.md` has the offline recipe against temp copies. Get a clean sim
run before the arm is live.

Not included, deliberately: our `ranger_air_description` URDF and `m1_bringup`
RViz config. A porter uses their own, and transcribes their own limits into the
tables in step 1.

## State on this machine (not copied — live files)

- `~/.config/m1/motor_map.yaml` — the deployed result of the real 2026-07-10 run.
  Several joints carry a `refit` note about the 2026-07-11 bus-side swap.
- `~/.config/m1/calibration-backups/` — timestamped backups, incl.
  `pre-side-swap-20260710-165851/README-ROLLBACK.md`.
- Rollback: `ros2 run m1_can_tools m1_calibrate --rollback <backup-dir>`.

## Verified

`PYTHONPATH=. pytest test/ -q` → **21 passed**. All non-ROS modules import cleanly
against this folder alone. `calibrate_live.py` additionally needs `rclpy` +
`sensor_msgs` at runtime, which is why it is not covered by that check.
