# Independent ROS/control CLI server-loss review of `99bc85e`

Date: 2026-07-29 (America/Los_Angeles)
Reviewed: `99bc85e` against `0e16183`, with all earlier ROS/control findings
used as regression constraints
Disposition: **FINDINGS**

No GUI, browser, CAN, external service, hardware, commissioning, calibration,
or physical-control operation was performed. DDS was confined to isolated ROS
domains. The implementation was not modified; this report is the only worktree
write.

## Critical

None.

## Important

None.

## Minor

### M1. An already-completed terminal result loses to cancel-response timeout

After the initial terminal wait expires, the CLI creates both a still-pending
`result_future` and a `cancel_future`, but waits only on `cancel_future`
(`openarm_control_cli.cpp:203-211`). If the action reaches a terminal result
while the cancel response is delayed, that result is processed by the executor
and becomes ready, yet a cancel-response timeout immediately returns status 5.
The code does not make the terminal future the final authority until after the
cancel response succeeds (`213-223`). The same ordering can let server/context
loss during the cancel-response wait mask an already-ready result.

This was reproduced against the exact fresh test executable, which compiles the
production source with only its documented shorter timeout constants. The
server accepted and started the goal; at 1.5 s the CLI requested cancellation;
the cancel callback deliberately remained pending while the execute callback
successfully sent a completed result (`command_id=77`). The CLI had received
and spun that result but exited 5 at the 500 ms cancel-response deadline:

```text
CLI_EXIT=5 ELAPSED=2.334 RESULT_SENT=True CANCEL_ENTERED=True
STDERR=terminal result timeout; cancel response timeout
```

The expected outcome is successful terminal handling (exit 0 and command 77),
because action terminal state is final and makes the cancel acknowledgement
irrelevant. A canceled/aborted result in the same ordering must likewise retain
its normal 6/7 exit semantics. The new `cancel_race` test does not cover this
reverse ordering: its cancel callback returns immediately, and only afterward
does its execute callback publish the canceled result.

Before returning any cancel-wait timeout or post-acceptance loss, the CLI should
give `result_future` a final readiness check and route a ready result through
the common terminal-result handling path. Waiting on both futures during this
phase would close the broader race.

## Verified server-loss behavior

The primary server-loss fix works as intended. Independent production-binary
reproductions killed the server only after the indicated accepted phase:

```text
queued:   exit=8 elapsed_after_death=0.762s alive=false
started:  exit=8 elapsed_after_death=0.737s alive=false
settling: exit=8 elapsed_after_death=0.738s alive=false
```

All three emitted only `action server lost after goal acceptance`, did not enter
the 45-second timeout/cancel cascade, and left no CLI process behind.

The 750 ms debounce also resets correctly. After the original accepted server
disappeared, a replacement action server became ready before the grace expired,
remained available for 600 ms, and disappeared again. The CLI was still alive
after that transient recovery and exited 8 about 0.88 s after the second loss,
not 750 ms after the first loss.

A healthy one-second action using the exact production CLI completed in 1.246 s
with exit 0 and `completed command_id=41`; it was not misclassified as server
loss. The focused test also passed its isolated context-shutdown path with
distinct exit 8/message and its ordinary cancel/result ordering with exit 6.

Existing production timeout semantics remain intact: a reachable server that
withheld its result for 45 seconds, then accepted cancellation and returned a
canceled terminal result, produced exit 6 and the exact terminal reason after
45.34 s. Rejection, ordinary completion, and initial terminal-result handling
also passed.

## Race and integration review

- The watcher uses the caller-owned single-threaded executor; client, context,
  node, goal handle, and futures outlive every wait. No detached C++ thread or
  callback reference escapes its scope.
- Each 50 ms wait checks the current future before graph readiness, resets the
  absence timestamp on readiness, and performs a final check of that current
  future before declaring timeout or server loss. M1 is specifically that the
  current future is the cancel response rather than the already-final action
  result during the second phase.
- Context invalidation is caught without an exception escaping `main`, and a
  repeated global shutdown is benign in the tested path.
- The clean unified build still installs all native components,
  `openarm_control_msgs`, and `openarm_ik_ros`; it now registers exactly ten ROS
  tests. No CAN/transport/commission/physical dependency was introduced.
- `git diff --check 0e16183..99bc85e` is clean.

## Fresh test evidence

- Unified native CTest: 14/14 passed.
- Exact installed ROS aggregate: 10/10 passed in 79.02 s.
- New authored CLI lifecycle test: passed independently in 9.18 s and again in
  the aggregate; it covers production queued/started/settling loss, shortened
  healthy completion, ordinary cancel/result ordering, and context shutdown.
- Independent production server-loss, debounce-reset, healthy-slow, and
  45-second timeout/cancel reproductions passed as detailed above.
- Independent reverse cancel/result reproduction deterministically exposed M1.
- The prior 13-case callback/fault session suite and all earlier ROS contract,
  SIGINT, cancellation, authority, provenance, manifest, and isolation tests
  remain green in the aggregate.

Not claimed: GUI/browser rendering, physical/CAN behavior, randomized executor
campaigns, or exhaustive DDS discovery-fault injection.
