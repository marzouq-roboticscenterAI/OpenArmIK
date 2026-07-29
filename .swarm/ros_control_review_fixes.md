# ROS/control review fixes

Date: 2026-07-29 (America/Los_Angeles)
Branch: `feat/ros-control-sim`
Merged baseline: `main` through `21ab251` (`be628f9`)
Status: **DONE**

No GUI, CAN, external network, hardware, commissioning, or physical-control
operation was performed.

## Finding resolution

- **I1:** Session callbacks and the worker now contain exceptions. ROS feedback
  and terminal publication check both context validity and goal activity and
  catch action-server exceptions. Shutdown disable-stops first, captures a
  post-stop DISARMED snapshot, records the terminal result even when ROS can no
  longer publish it, and exits within the two-second bound.
- **I2:** The merged native builder installs CAN, model, commission, transport,
  and control in dependency order. The ROS builder now explicitly selects
  `openarm_control_msgs` between description and adapter builds. A clean unified
  build from empty output directories succeeded.
- **I3:** Cancellation always attempts `OA_STOP_DISABLE`, including a reservation
  with no submitted command. The adapter snapshots DISARMED state, remains
  restart-gated, and retains the canceled reservation until the accepted
  callback submits its command, which receives exactly one CANCELED terminal.
  Releasing a validation-failed canceled reservation cannot rearm or relabel the
  disarmed adapter as idle.
- **I4:** Diagnostics use one mutex-protected correlated record containing source,
  owner/UUID, request stamp, outcome, status, command/plan provenance, terminal
  sequences, lifecycle, event, cause, and collision status. Every request,
  rejection, and terminal replaces the complete record. A live legacy command
  completing after a rejected action retains only its legacy identity.
- **I5:** Active command and owner state remain held while the terminal callback
  publishes/records its result. Cancellation and new reservations are rejected
  during this terminalizing interval; ownership is released only afterward.
- **M1:** A CLI result timeout now waits for the cancel response and then the
  terminal result before returning. Unconfirmed cancellation remains exit 5;
  confirmed CANCELED remains exit 6.
- **M2:** Added the ABI-checked
  `oa_manifest_get_openarm_v10_virtual_config()` accessor. ROS joint names and
  limits are now derived from that canonical manifest. Native tests verify both
  arms' names, indices, limits, motor families, affine signs, and zero offsets.
- **M3:** Diagnostics expose capability bits, plan seeds/duration, and terminal
  measured sequences. Action results include plan duration, and completion,
  cancellation, fault, and shutdown preserve active plan seeds, duration, and
  collision provenance.

## Adversarial coverage added

- Reservation-only cancel, delayed accepted callback, exact-one terminal, and
  persistent DISARMED/restart-gated state.
- Completion callback held open while a competing reservation is attempted.
- Throwing terminal callbacks and bounded idempotent close.
- Active shutdown with exactly one aborted result and measured seed/duration/
  terminal provenance.
- Process-level SIGINT at queued, started, settling, and completed action phases;
  each process exits 0 within two seconds with no terminate or missing-goal error.
- Live legacy completion after a rejected action, with full diagnostic identity
  and provenance correlation.

## Final evidence

- Clean unified build: all five native components plus
  `openarm_description`, `openarm_control_msgs`, and `openarm_ik_ros` built and
  installed from empty output directories.
- Native CTest: **14/14 passed** across CAN (1), model (4), commission (2),
  transport (3), and control (4).
- Final authored ROS CTest aggregate: **9/9 passed** in 68.98 seconds, including
  the nine-case production session suite, no-CAN syscall isolation, generated
  URDF, invalid configuration, live graph/action/diagnostic contract, and the
  four-phase SIGINT matrix.
- ASan/UBSan: control **3/3** and production session **9/9** passed with leak and
  undefined-behavior reporting enabled; the final edge/race subset was rerun
  after the last state-transition fix.
- TSan: control **3/3** and production session **9/9** passed with
  `halt_on_error=1`; the final edge/race subset was rerun after the last fix.
- Warnings-as-errors ROS build and `git diff --check` passed.

Not claimed: GUI rendering, CAN/vcan/hardware operation, million-cycle soak, or
exposing simulator fault injection through the ROS API.
