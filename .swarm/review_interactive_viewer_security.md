# Final independent reliability/security re-review: interactive WebGL viewer

Reviewed commit `e01c091` atop `6203c8a` in `/home/signalprocessing-dev/OpenArmIK/build/worktrees/interactive-webgl-viewer`.

Final verdict: **CLEAN.** The remaining partial-body intake starvation finding is closed, and the earlier static-serving starvation and asset TOCTOU fixes remain closed.

## Partial-body intake closure

Intake is now asynchronous on the server's single `io_context`, with a real 500 ms Beast stream deadline (`openarm_portal.cpp:678-762`). At most sixteen sessions are retained; admitting another connection cancels and evicts the oldest incomplete session (`openarm_portal.cpp:839-850`). Successful parsing removes the session before moving its stream/request into the separately bounded static or API executor (`openarm_portal.cpp:852-886`). Shutdown cancels all retained sessions and drains cancellation handlers before destroying the server (`openarm_portal.cpp:888-900`).

The prior sixteen-incomplete-body reproduction now passed:

```
partial_clients 16
stop_http_code 200
stop_elapsed_ms 12
fds_baseline 60
fds_with_partial 76
fds_after_deadline 60
threads_baseline 24
threads_after_deadline 24
```

Evidence: `build/review-security-final/partial-body-closure.txt`. The authenticated stop completed well below its 750 ms test bound. After the 500 ms intake deadline, all sixteen server-side FDs were reclaimed and the thread count remained at baseline.

With sixteen incomplete bodies open during SIGTERM, shutdown completed in 121 ms:

```
partial_clients 16
fds_before_term 76
term_exit_ms 121
```

Evidence: `build/review-security-final/partial-sigterm.txt`. This is comfortably inside the launcher's five-second portal escalation window.

## Lifetime, eviction, and route review

- Every async handler owns its `IntakeSession` through `shared_from_this()`; the session owns its stream/parser/buffer, while `PortalServer` outlives callbacks because shutdown cancels and polls intake completions before returning.
- Intake deque mutation and `io_context::poll()` occur on the server thread only. Static/API workers never access the intake deque, so no container race was introduced.
- Eviction removes the oldest shared pointer, cancels/closes its socket, and leaves the pending handler's self-reference valid until its error completion. Its later `finish_intake()` safely becomes a no-op if it was already evicted.
- On successful intake, the stream and released request move into downstream execution while the handler's self-reference keeps the session alive through the handoff. Static/API tasks capture the server only until their pools are joined.
- Host, Origin, Fetch Metadata, GET-body, parser header/body limits, exact route classification, CSRF, and content-type validation remain before any state-changing API action. Partial or evicted requests are never dispatched.

No use-after-free, double-removal, unbalanced admission counter, or new route/trust-boundary regression was found.

## Earlier finding closure retained

- Static viewer responses remain isolated in the two-worker/one-pending static lane; the checked-in regression confirms eight one-byte-per-second mesh clients do not delay authenticated stop.
- Assets remain exact-size/SHA-256 verified and resident; post-start disk mutation cannot change served bytes, while corrupted bytes fail the next startup closed.
- The properly ROS-sourced checked-in `test_visualization_urdf_parser` regression passed in 6.13 seconds and exercises static backpressure, sixteen incomplete bodies, deadline FD/thread reclamation, prompt SIGTERM, immutable asset serving, corrupt-restart rejection, and the browser oracle. Evidence: `build/review-security-final/checked-in-regression-sourced.log`.

The first bare CTest invocation lacked the ROS message-library runtime path and exited before portal startup; rerunning under the normal sourced ROS/workspace environment passed. No production files were edited. All new probes used loopback only, with no move request, CAN access, physical transmit, or external network.
