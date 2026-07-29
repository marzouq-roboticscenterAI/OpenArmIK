# Independent ROS/control re-review through `63f5617`

Date: 2026-07-29 (America/Los_Angeles)
Reviewed: merge `be628f9` plus fix `63f5617`, with `b3993ba` and the prior
review used as the regression baseline
Disposition: **FINDINGS**

No GUI, CAN, network service, hardware, commissioning, calibration, or physical
control operation was performed. DDS traffic was confined to isolated ROS
domains. The implementation was not modified; this report is the only worktree
write.

## Critical

None.

## Important

### I1. A callback fault continues publishing apparently live measured state, and a core fault reports a stale lifecycle

`publish_feedback()` catches an exception from an action feedback callback and
calls `fault()` (`virtual_control_session.cpp:644-678`), but it returns `void`.
Consequently `publish_measured()` returns true (`361-372`) and the worker loop
continues. The loop gates only `stopped_requires_restart`, not `fault`
(`239-244`). `fault()` has already disable-stopped the controller and cleared
the active command, but subsequent DISARMED virtual advances and measured-state
callbacks continue. At the ROS boundary those callbacks publish new
`JointState` messages, making a faulted adapter look live.

This was reproduced against the exact fresh RelWithDebInfo build by submitting
a production `VirtualControlSession` command whose feedback callback throws.
The adapter reached `fault`, disable-stop produced lifecycle 2
(`OA_LIFECYCLE_DISARMED`), and the aborted terminal callback ran exactly once,
yet measured-state callback count increased from 2 to 22 over the following
100 ms:

```text
adapter1=fault lifecycle1=2 reason1=feedback_callback_failed
states_before=2 states_after=22 adapter2=fault lifecycle2=2 terminals=1
```

There is a second truthfulness failure in the same fault path. `fault()` only
refreshes `snapshot_` when `oa_controller_stop()` succeeds (`791-796`). If
`oa_controller_advance()` has already latched the native controller into
`OA_LIFECYCLE_FAULT`, stop is rejected by the core, so adapter health and an
active terminal result retain the last pre-fault snapshot (`828-833`). A 35 ms
cycle overrun during an active command caused the native deadline fault and
produced exactly one aborted terminal callback with this stale result:

```text
adapter=fault health_lifecycle=5 cause=5 terminals=1
result_lifecycle=5 result_event=6 result_reason=advance_failed
```

Lifecycle 5 is `OA_LIFECYCLE_EXECUTING`; the underlying controller had entered
lifecycle 7 (`OA_LIFECYCLE_FAULT`). The same overrun before command submission
reported stale lifecycle 4 (`OA_LIFECYCLE_ARMED_IDLE`). Thus adapter diagnostics
and results can disagree with authoritative core state exactly at fault time.
The worker must stop publication on every adapter fault, and the fault
snapshot/result must be refreshed even when disable-stop is rejected because
the core is already faulted.

## Prior finding closure

All eight prior findings are otherwise resolved by `63f5617`:

- **I1, active SIGINT exception:** the exact installed node exited cleanly and
  within two seconds at queued, started, settling, and completed phases. The
  aggregate test found neither `terminate called` nor the old missing-goal
  exception. Goal finalization and user callbacks are now exception-bounded.
- **I2, incomplete build:** a clean archived checkout with only its pinned
  upstream linked in a temporary source tree completed `scripts/build.sh
  --tests`. It built/installed CAN, model, commission, transport, control,
  `openarm_description`, `openarm_control_msgs`, and `openarm_ik_ros`; both the
  generated actions and C++ node/CLI are in the fresh install.
- **I3, reserved/executing cancellation:** the production session tests now
  reproduce cancel-before-submit, cancel/release, and executing cancel.
  Reserved cancel disable-stops to `OA_LIFECYCLE_DISARMED`, retains the owner
  until the accepted command can receive one canceled terminal callback, and
  rejects restart. Executing cancel likewise returns one canceled result with
  a refreshed DISARMED snapshot.
- **I4, legacy diagnostic correlation:** the fresh ROS contract test ran the
  original legacy-command/concurrent-rejected-goal race. Its terminal record
  remained atomically correlated as `deprecated_paired_xyz`, the exact legacy
  owner and request stamp, `outcome=completed`, and matching seed/duration/
  terminal sequences.
- **I5, terminal ownership:** a blocking terminal callback test proved a new
  reservation remains `busy` until the prior terminal callback returns, then
  succeeds.
- **M1, CLI cancel wait:** the compiled CLI now waits up to three seconds for
  the cancel response and five seconds for the original result future before
  shutting ROS down; ordinary completion/rejection paths passed in the ROS
  contract test. A dedicated exact timeout reproduction is recorded below.
- **M2, manifest duplication:** ROS joint names and limits are derived from the
  public standard-manifest configuration accessor, and the session suite checks
  canonical mapping and limit boundaries.
- **M3, provenance:** actions/results and diagnostics now carry capability
  bits, plan seed sequences, duration, terminal measured sequences, outcome,
  cause, lifecycle, event, owner, and request identity. Shutdown/fault results
  preserve the active plan provenance.

The new Important finding above is outside those successful regression tests;
there is no test for a throwing feedback callback or for adapter reporting
after a native controller fault.

## Other verified properties

- The fresh runtime graph has one measured-state authority and the duplicate
  local adapter is rejected. The adapter publishes no TF, target-as-state, or
  finger joints.
- The simultaneous identity/provenance race is covered by the legacy/rejected
  ROS test; reservation/terminal races are covered directly by the session
  suite. Named-frame, invalid-name, missing-transform, and duplicate-authority
  rejection paths passed.
- No adapter or message target links CAN, transport, commission, Python runtime,
  or physical-control libraries. Source and syscall checks found no CAN device,
  SocketCAN, calibration, commissioning, persistence, or physical-backend ROS
  endpoint. `physical_motion_authorized=false` remains explicit.
- Current `main` is three non-overlapping documentation/portal-launcher commits
  beyond the merge base `21ab251`; `git merge-tree` reports no conflicts with
  this branch. Its portal launcher starts the same virtual-only launch and adds
  no control-core semantic overlap.

## Fresh test evidence

- Unified clean build: succeeded from a temporary archive of exact `63f5617`.
- Native tests in the unified build: 14/14 passed (CAN 1, model 4,
  commission 2, transport 3, control/ABI/install consumer 4).
- Authored exact-binary ROS aggregate: 9/9 passed, including the expanded
  9-case session suite, no-CAN syscall isolation, generated URDF, invalid
  expiry, full ROS contract, four-phase active SIGINT, and helper checks.
- CLI terminal-timeout reproduction: a deliberately nonterminating isolated
  action server accepted cancellation; after 45.25 s the exact installed CLI
  waited for the cancel response and canceled result, printed the server's
  terminal reason, and exited with the distinct canceled status 6.
- ASan/UBSan with halt/leak checking: native control 3/3 and production session
  9/9 passed.
- TSan with halt-on-error: native control 3/3 and production session 9/9 passed.
- Exact targeted fault reproductions deterministically exposed I1 above; no
  sanitizer or race-detector report accompanied either logical failure.
- `git diff --check be628f9..63f5617` is clean. The total feature diff still
  contains the two previously noted trailing-space lines in the implementation
  report `.swarm/ros_control_impl.md`; no executable source has a whitespace
  error.

Not claimed: GUI rendering, physical/CAN behavior, exhaustive injected
freeze/drop/skew matrices, or million-cycle soak.
