# Independent ROS/control third review through `43687a9`

Date: 2026-07-29 (America/Los_Angeles)
Reviewed: fault fix `9e427e8`, current-main merge `527d25d`, and report commit
`43687a9`, with the prior two reviews used as the regression baseline
Disposition: **CLEAN**

No GUI, browser, CAN, external service, hardware, commissioning, calibration,
or physical-control operation was performed. DDS was confined to isolated ROS
domains. The implementation was not modified; this report is the only worktree
write.

## Findings

Critical: none.

Important: none.

Minor: none.

## Prior Important convergence check

The callback/fault defect from the second review does **not** persist.

- `publish_feedback()` now returns failure after a rejected or throwing
  callback; `publish_measured()` propagates it and the worker breaks. The loop
  also explicitly gates `AdapterState::fault`, so it cannot resume measured or
  feedback publication on a DISARMED virtual controller.
- Throwing measured-state, feedback, and terminal callbacks are contained by
  `noexcept` wrappers and converted to a fault. Normal fault terminalization
  moves the active/pending command once, invokes at most one terminal callback,
  clears owner and terminalizing state, and permanently rejects reuse with
  `adapter_fault`. A terminal callback that itself fails is not invoked again.
- `fault()` now performs its best-effort disable-stop and then independently
  snapshots the native controller regardless of the stop result. Therefore a
  native fault that rejects stop with `OA_CONTROL_ESTATE` still reports the
  authoritative `OA_LIFECYCLE_FAULT` snapshot and the first native cause in
  both `SessionHealth` and the action result.
- Shutdown and cancellation detect an already-latched fault and do not issue a
  second terminal result or relabel its lifecycle/cause.

The exact fresh session binary independently passed the five focused tests for
throwing terminal, feedback, and measured callbacks plus idle and active 35 ms
cycle overruns. Each callback-fault test observed frozen publication for a
further 100 ms, exactly one terminal attempt where a command existed, empty
ownership, and failed reuse. No exception escaped or terminated the process.

The active overrun returned one aborted result with event `OA_EVENT_FAULTED`,
lifecycle `OA_LIFECYCLE_FAULT`, cause/status `OA_CONTROL_ETIMEOUT`, and retained
plan provenance. The idle overrun exposed the same authoritative lifecycle and
cause in health without publishing another sample.

A separate exact native C-ABI reproduction established the difficult stop
ordering directly:

```text
first=0 fault=5 stop_after_fault=3 snapshot=0 lifecycle=7
```

That is: first advance OK, deadline advance `OA_CONTROL_ETIMEOUT`, subsequent
disable-stop `OA_CONTROL_ESTATE`, successful snapshot, and authoritative
`OA_LIFECYCLE_FAULT`.

## Earlier finding regression check

The five original Important and three Minor findings remain closed:

- Four-phase queued/started/settling/completed SIGINT passed with bounded clean
  exit and no missing-goal exception or process abort.
- Reserved and executing cancellation disable-stop truthfully, terminate once,
  retain authority through terminal publication, and reject restart. The
  completion callback test likewise holds ownership until the callback returns.
- The ROS contract reran the legacy-command/concurrent-rejection provenance
  race; its diagnostic record remained one coherent action/owner/stamp/outcome
  record with matching seed, duration, and terminal sequences.
- The CLI completion/rejection paths passed. Its timeout path still waits for a
  cancel response and terminal result; that code is unchanged from the prior
  exact 45.25 s cancel-confirmation reproduction.
- ROS names and limits still come from the standard-manifest accessor, and the
  canonical mapping/limit boundary test passed.
- Capability, seed, duration, terminal sequence, lifecycle, event, cause,
  owner, UUID/stamp, and outcome provenance remain present in results and
  diagnostics.

## Build, integration, and safety surface

- A clean temporary archive of exact `43687a9`, supplied only the repository's
  pinned upstream, completed `scripts/build.sh --tests` into empty build and
  install directories. It built/installed CAN, model, commission, transport,
  control, `openarm_description`, `openarm_control_msgs`, and `openarm_ik_ros`.
- `main` is an ancestor of HEAD after `527d25d`. Its added dependency/portal
  launchers do not change controller ownership or the virtual-only ROS adapter.
- The ROS node and CLI still have no CAN, transport, commission, calibration,
  persistence, device, or physical-backend endpoint/linkage. The syscall
  isolation test passed, and `physical_motion_authorized=false` remains
  explicit.
- The fault-fix commit and post-merge report diff pass `git diff --check`; no
  executable-source whitespace issue was introduced.

## Fresh test evidence

- Unified native CTest: 14/14 passed.
- Exact installed ROS aggregate: 9/9 passed in 72.52 s.
- Production session suite: 13/13 passed; the focused five fault cases passed
  independently in 1.48 s.
- Fresh ASan/UBSan exact session binary with halt-on-error and leak checking:
  13/13 passed.
- Fresh TSan exact session binary with halt-on-error: 13/13 passed.
- Direct native core fault/stop/snapshot reproduction passed as shown above.

Not claimed: GUI/browser rendering, physical/CAN behavior, randomized executor
campaigns, exhaustive injected fault matrices, or million-cycle soak.
