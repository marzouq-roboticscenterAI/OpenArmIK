# Independent ROS/control CLI terminal-precedence review of `a26118c`

Date: 2026-07-29 (America/Los_Angeles)
Reviewed: `a26118c` against `99bc85e`, with all earlier ROS/control findings
used as regression constraints
Disposition: **CLEAN**

No Critical, Important, or Minor findings.

No GUI, browser, CAN, external service, hardware, commissioning, calibration,
or physical-control operation was performed. DDS was confined to isolated ROS
domains. The implementation was not modified; this report is the only worktree
write.

## Prior finding disposition

The prior reverse cancel/result race does not persist. The exact reproduction
that previously exited 5 now consumes the successful terminal result while its
cancel response is still pending:

```text
EXIT=0 ELAPSED=1.852 RESULT_SENT=True CANCEL_ENTERED=True
OUT=completed command_id=77
```

The new wait helper watches both the phase-specific future and the action's
terminal-result future. It checks the terminal future first on each iteration
and again before reporting timeout, context shutdown, or server loss. All ready
terminal results use one common reporting path. There is one
`result_future.get()` site, and each successful consumption immediately returns
from `run_goal`, so the terminal future is neither consumed twice nor allowed
to lose to a later phase outcome.

## Independent terminal-ordering matrix

The exact fresh test executable was exercised with terminal results before,
during, immediately before, and immediately after the cancel-response timeout,
and after accepted and rejected cancel responses:

```text
before_accept_response:       exit=0 elapsed=1.805 terminal_count=1 completed command_id=88
during_accept_response:       exit=7 elapsed=1.969 terminal_count=1 during_accept_response
just_before_cancel_timeout:   exit=0 elapsed=2.162 terminal_count=1 completed command_id=88
just_after_cancel_timeout:    exit=5 elapsed=2.244 terminal_count=1 terminal result timeout; cancel response timeout
after_rejected_response:      exit=0 elapsed=1.948 terminal_count=1 completed command_id=88
canceled_after_accept:        exit=6 elapsed=2.234 terminal_count=1 canceled_after_accept
aborted_after_reject:         exit=7 elapsed=1.976 terminal_count=1 aborted_after_reject
MATRIX_OK=True
```

The just-after-deadline exit 5 is the expected hard-timeout result; terminal
results ready at or before the boundary won. Success, canceled, and aborted
states retained exit codes 0, 6, and 7 respectively. Accepted and rejected
cancel responses both allowed the final action state to remain authoritative.
Each server sent exactly one terminal result and each CLI run emitted exactly
one terminal outcome.

The authored lifecycle test independently covers success before a delayed
cancel response, abort during it, success at the timeout boundary, success
after rejection, and cancellation after acceptance. It passed against the
fresh exact binary in 21.98 s.

## Server-loss and race review

Production-binary server loss remained bounded well below the requested two
seconds after server termination, with no lingering CLI process:

```text
queued:   exit=8 signal_to_exit=0.859s alive=false
started:  exit=8 signal_to_exit=0.882s alive=false
settling: exit=8 signal_to_exit=0.880s alive=false
```

Each emitted only `action server lost after goal acceptance`. The earlier
debounce-reset, healthy-slow-server, context-shutdown, and ordinary cancellation
behaviors remain covered by the focused and aggregate tests.

The caller-owned single-threaded executor performs all spins. The client,
context, node, goal handle, and futures remain alive throughout every wait; no
C++ worker thread or callback reference escapes the operation. Readiness tests
use the shared futures without mutating them, and terminal handling has no
second getter or fallthrough path. No executor-lifetime or future-lifetime race
was identified.

## Fresh build and test evidence

- Clean unified native CTest: 14/14 passed.
- Exact installed ROS aggregate: 10/10 passed in 91.86 s.
- Exact focused CLI lifecycle test: 1/1 passed in 21.98 s.
- Independent reverse-race, seven-case terminal matrix, and three-phase
  production server-loss reproductions passed as detailed above.
- `git diff --check 99bc85e..a26118c` is clean.

Two initial clean builds stopped while cloning the unchanged pinned model
dependency because `/tmp` lacked capacity. Re-running the same exact archived
HEAD with `TMPDIR` relocated to a filesystem with sufficient space completed
the unified build and all tests. This was an environment-capacity issue, not a
source or test finding.

Not claimed: GUI/browser rendering, physical/CAN behavior, randomized
nanosecond-boundary scheduling, or exhaustive DDS discovery-fault injection.
