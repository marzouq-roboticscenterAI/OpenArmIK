# M1 arm calibration — the foolproof live procedure

**Goal:** make RViz match the real arms. This is the *one* supported way to do it.
It is guided, live, and objective — you cannot get a wrong direction sign or a
stale reference out of it, because the whole time the motors stream their **true**
position and every zero comes from a **mechanical hardstop**, not from eyeballing.

> **Why the old way failed (read once).** A *disabled* Damiao motor reports a
> **frozen** encoder value — it does not track motion. So every capture that read
> disabled motors (`m1_calibrate --capture-dropped` and the now-removed passive
> reference-capture executables) was built on dead data, and RViz never matched. This
> procedure **compliant-enables** the arm at **zero torque** (limp, freely
> back-drivable, no snap) so the encoder streams the real position. Directions are
> confirmed live, offsets come from hardstops. That's the whole trick.

You calibrate **one arm at a time**. Budget ~10 min per arm.

---

## Before you start

- **Operator at the robot, hardware e-stop (motor power) in reach.** The wizard
  only ever sends *enable → zero-torque keepalive → disable*, and disables on
  every exit — but keep the e-stop anyway.
- Stop anything that owns an arm CAN bus: `m1_hwconfig`, the ros2_control stack,
  or the direct-CAN portal. The wizard refuses to start if the bus is busy.
- Resolve and configure the two arm buses by persistent adapter identity. Never
  infer left/right from an enumeration-order `canN` name:

  ```bash
  ./deploy/agx-orin/can_roles.sh configure --require arms
  ```
- One-time on the console host so RViz can open a window:
  ```bash
  xhost +local:
  ```

---

## Run it — two terminals

**Terminal 1 — RViz** (robot_state_publisher + RViz; renders the live arm):

```bash
sg docker -c './deploy/agx-orin/run_container.sh ros2 launch m1_bringup live_calibration.launch.py'
```

**Terminal 2 — the wizard** (owns the bus, drives the guided steps):

```bash
# left arm (use  right  or  both  as needed):
sg docker -c './deploy/agx-orin/run_container.sh ros2 run m1_can_tools m1_calibrate_live left'
```

The wizard prints the resolved bus and asks you to **type the side name** to
proceed (so you can't calibrate the wrong arm). Then it compliant-enables — **the
arm goes limp.** RViz should now move exactly as you move the real arm. If RViz
does **not** move when you move the arm, stop: the bus/telemetry is wrong (never
"calibrate" against a frozen model again).

### Step A — hardstops (J1–J7)

For each joint the wizard names it (e.g. *"J2 shoulder pitch (upper arm up/down)"*)
and its URDF range, then asks you to move it to **each mechanical hardstop**, hold,
and press **Enter**:

- Move the joint **gently by hand** until it physically stops. Hold it still
  against the stop, press Enter. Then the **opposite** stop, hold, Enter.
- **J4 (elbow) is special:** only its **safe** stop is captured — straighten the
  elbow fully **open** to its back stop. **Never** push the elbow toward the body
  (front stop). The wizard asks for just the one safe stop.
- If the wizard warns that the measured span ≠ the URDF span, that joint's URDF
  limit may not be a true mechanical stop — note it and check it in Step C.

### Step B — grippers

Fully **close** the gripper, hold, Enter; fully **open**, hold, Enter. The wizard
derives the gripper sign and scale from that travel.

The wizard then **writes both motor-map copies atomically**, makes a timestamped
backup, and prints an exact **rollback** command. (Nothing is written until every
joint is captured; Ctrl-C at any point changes nothing and disables the motors.)

### Step C — live verify (this is where you *confirm* it matches)

The wizard keeps streaming with the **new** calibration. Move each joint and watch
RViz. Type commands at the `verify>` prompt:

| command | effect |
|---|---|
| `t` | print a live table: raw, URDF angle `q`, range, in-range? |
| `flip 3` | flip joint 3's direction (RViz moves the *opposite* way) — exact, reversible, recomputed from the stored hardstops |
| `flip 1 5 7` | flip several at once |
| `nudge 0.05 6` | add 0.05 rad to joint 6's zero (small "off by a bit" fix) |
| `stops 3` | re-capture joint 3's two hardstops and re-solve it |
| `done` | finish (disables motors, releases the bus) |

**How to confirm a direction without the viewpoint trap:** stand so you sight
**along** the joint's rotation axis (or use a gravity-obvious joint — pitch joints
J2/J4/J6 are unambiguous: up is up). Move the real joint; if the RViz joint goes
the opposite way, `flip` it. Do the pitch joints first — they anchor your sense of
the model's orientation before you judge the roll joints (J1/J3/J5/J7).

Every `flip`/`nudge`/`stops` rewrites both map files immediately — no restart.

---

## When it's done

Both maps are updated:
`~/.config/m1/motor_map.yaml` (deployed) and
`ros2_ws/src/m1_can_tools/config/motor_map.m1robot.yaml` (committed). The launch
gate that checks for calibration metadata is satisfied
(`source: live_hardstop_wizard`). Bring up the stack normally
(`hardware.launch.py … use_brain:=true`) and do the closed-loop check in
`LIVE_BRINGUP.md`: commanded fingertip ≈ measured fingertip.

**Rollback anytime:** `ros2 run m1_can_tools m1_calibrate --rollback <backup-dir>`
(the wizard prints the exact path).

---

## Notes / limits

- **Scope:** the seven rotary arm joints (scale = 1, Damiao 1:1 rad) + the gripper
  (measured scale). The **lift** is a separate CANopen servo — calibrate it per the
  `LIVE_BRINGUP.md` "Lift calibration" section, not here.
- **J2 / any span-warned joint:** if the URDF limit isn't a true hardstop, the
  offset is still pinned to the reachable stop; verify that joint in Step C and
  `nudge` if its zero pose looks off.
- **Offline rehearsal** (no hardware, no writes to the real map):
  ```bash
  sg docker -c './deploy/agx-orin/run_container.sh bash -c "
    D=/tmp/m1-home/cal; mkdir -p \$D
    cp ros2_ws/src/m1_can_tools/config/motor_map.m1robot.yaml \$D/deployed.yaml
    cp \$D/deployed.yaml \$D/repo.yaml
    M1_MOTOR_MAP=\$D/deployed.yaml M1_REPO_MAP=\$D/repo.yaml M1_BACKUP_ROOT=\$D/bk \
      ros2 run m1_can_tools m1_calibrate_live left --transport sim --yes"'
  ```
- The math is unit-gated: `test/test_calibrate_live_solve.py` (11/11) inside the
  full `m1_can_tools` pytest suite.
