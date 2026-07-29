# Independent review: SocketCAN transport

Commit: `5dd2493` versus parent `5dd2493^`  
Verdict: **CHANGES REQUIRED — 2 Critical, 2 Important, 1 Minor**

## Ranked findings

### Critical 1 — The public "capability" is caller-forgeable and bypasses the approved Stage-A physical gate

`oa_transport_capability` contains only caller-selected permission bits and an
expiry (`transport/include/openarm_transport.h:95-101`). `oa_transport_open()`
trusts those values directly, requiring merely a future timestamp for dangerous
permissions (`transport/src/c_api.cpp:143-165`). There is no nonce, verification
epoch, issuer, one-shot consumption/replay protection, interface binding, target
ID binding, or maximum capability lifetime. Any caller that can call the API can
self-authorize `CONTROL` or `COMMISSION` until an arbitrarily distant expiry.

This is not the expiring operator challenge approved in
`.swarm/controller_design_final.md:76-79,108-116`, and it also bypasses the
explicit Stage-A rule that physical SocketCAN may open only to verify disabled
motors and that destructive operations live in a separate commissioning product
(`.swarm/controller_design_final.md:33-36,125-139`). The backend makes no
physical-versus-virtual distinction, so the self-issued record enables motion,
enable, register writes, flash save, set-zero, and clear-error on any existing
classic-CAN interface. Expiry checks in `Transport::send()` are internally
consistent, but expiry alone cannot establish authority.

Dangerous authorization needs to be opaque/issued after the relevant verified
lifecycle transition, bound to a controller verification epoch, unpredictable
nonce, interface and exact target set, short bounded expiry, and replay policy.
Until Stage C is explicitly enabled, a physical backend must reject control and
commissioning regardless of a caller-supplied record; commissioning writes
belong behind the separately challenged commissioning workflow.

### Critical 2 — Raw-frame classification is not fail-closed for motion or commissioning writes

After recognizing a few exact special/system shapes, `classify()` labels every
other eight-byte frame with arbitration ID `1..0x7fe` as motion
(`transport/src/transport.cpp:49-110`). It has no allowlist of commissioned ESC
IDs, no motor-mode/profile correlation, and no distinction among MIT, POS_VEL,
POS_FORCE, feedback IDs, or unrelated standard-CAN traffic. Consequently a
control-authorized handle transmits arbitrary payloads to arbitrary standard IDs.
This is materially weaker than the current codec, whose POS builders bind a
verified profile and send ID and reject non-finite, negative-speed, and
out-of-profile values (`can/src/openarm_can.c:597-652`). MIT payload bits cannot
all be rejected syntactically, but their target ID and commissioned mode/profile
still must be bound rather than treating the whole standard-ID namespace as MIT.

The `0x7ff` path is also only partially strict. Query, refresh/status, and save
padding are checked correctly, and query RIDs are constrained. Register writes,
however, check only that the RID is in a writable set
(`transport/src/transport.cpp:93-103`); illegal control modes/bitrates/IDs,
non-finite float bit patterns, and out-of-range PMAX/VMAX/TMAX or protection
values are passed even though the codec rejects them (`can/src/openarm_can.c:125-159,489-517`).
Special-command recognition runs before the `0x7ff` branch, so
`0x7ff/FF FF FF FF FF FF FF {FB,FC,FD,FE}` is classified as clear/enable/
disable/set-zero instead of unknown. Exact special payloads are otherwise
recognized, but their arbitration IDs are not restricted to commissioned motor
targets.

The transport should accept typed codec output or validate against immutable
per-handle target/mode/profile metadata. Unknown IDs/shapes and semantically
invalid register/POS values must be rejected before the syscall. Set-zero and
flash-save additionally require the disabled, serial-correlated commissioning
state mandated by the final design; a generic `COMMISSION` bit is insufficient.

### Important 1 — `close()` is a wakeup, not a completed-send barrier

The eventfd is level-readable and correctly wakes both blocked directions, but
close does not synchronize with an operation that already passed its `closed_`
check. A sender can load `closed_ == false` at
`transport/src/socketcan_backend.cpp:63-73`, be descheduled, have
`oa_transport_close()` set both closed flags, write eventfd, and return, then
resume and transmit at line 75. That sender can report success after close has
returned because the CAN FD remains open until destruction. The same race lets
the immediate initial-link receive complete successfully across close.

For shutdown to be a safety boundary, close should first publish/signal closure
and then synchronize with both direction locks before returning, while retaining
the eventfd wakeup so it does not wait for the caller's deadline. The tests cover
only a blocked fake receive (`transport/tests/test_transport.cpp:265-279`), not a
blocked or pre-syscall send and not simultaneous send/receive shutdown.

### Important 2 — Link monitoring can miss a safety-significant down transition

Initial link state is queried before the netlink socket is created/subscribed
(`transport/src/socketcan_backend.cpp:435-501`), leaving a window in which a link
transition is neither reflected in `initial_link_pending_` nor received through
netlink. In addition, `drainLinkEvents()` drains every queued link message and
overwrites one `out_up` value (`transport/src/socketcan_backend.cpp:213-274`); a
down/delete followed by up in the same drain produces only a final up event.
That conflicts with the approved rule that any link-down latches a fault, even
if the interface later recovers.

Subscribe before taking the initial snapshot (or re-query after subscription),
and preserve every transition or at least report that a down occurred during the
batch. `NLMSG_ERROR`/`NLMSG_OVERRUN` do fail closed, but no real netlink, filter,
timestamp, error-frame, overflow, or eventfd-send path is exercised here: `vcan0`
was unavailable and the smoke test skipped.

### Minor 1 — Absolute-deadline equality has inconsistent semantics

`Transport::validateDeadline()` accepts `deadline == now`
(`transport/src/transport.cpp:122-130`), while SocketCAN send/wait treat equality
as timed out (`transport/src/socketcan_backend.cpp:67-73,156-164`). The fake send
accepts equality, and the SocketCAN initial-link fast path can return success at
or just after an already-reached deadline because it does not recheck time
(`transport/src/socketcan_backend.cpp:105-111`). Use one `deadline <= now`
policy, including immediate queued events, and test the exact boundary.

## Areas that passed review

- The public header is valid C11, uses fixed-width enum-like/status types and
  opaque ownership, and the C++ wrappers do not unwind across the C boundary.
  Export inspection found all seven public entry points unmangled. Output
  records are written only on success, and the documented destroy-after-join
  lifetime rule avoids use-after-free when followed.
- Interface names are bounded and resolved exactly. The backend validates the
  existing ifindex, `CAN_MTU`, `ARPHRD_CAN`, and read-only flags before binding a
  fully zero-initialized `sockaddr_can`. All ioctls are getters.
- The CAN socket is nonblocking/CLOEXEC; filters exclude EFF/RTR, error filters
  are separate, own-message receive is disabled, realtime software timestamps
  are labeled explicitly, overflow counters/deltas use wrap-safe unsigned
  arithmetic, malformed/truncated classic frames fail closed, and netlink is
  receive-only.
- The send deadline is rechecked immediately before the nonblocking syscall,
  deadline horizons are bounded to 60 seconds, and dangerous sends require a
  deadline no later than their expiry. The single persistent eventfd wakeup is
  sufficient to wake both blocked directions.
- Static source/test inspection found no `SETLINK`, setter ioctl, shell,
  subprocess, sudo, interface creation, bitrate change, or test transmission.
  The only interface test against `lo` performs read-only validation and rejects
  it before bind. The `vcan0` test verifies a virtual sysfs path and contains no
  send, but it skipped on this host because `vcan0` is absent. No physical CAN
  interface was opened and no link or hardware state was mutated during review.

## Fresh verification

- GCC 15.2 Release configure/build and `ctest`: **PASS**, 3/3.
- Debug ASan+UBSan with leak detection and halt-on-error: **PASS**, 3/3.
- GCC ThreadSanitizer build and `ctest`: **PASS**, 3/3. The close/send race above
  is an ordering/contract race not represented by the current tests, so it does
  not produce a TSan data-race report.
- Separately compiled strict C11 consumer: **PASS**.
- Exported-symbol inspection (`nm`/`readelf`): C names present and unmangled.
- `git diff --check 5dd2493^ 5dd2493`: one trailing-space warning in
  `.swarm/socketcan_impl.md:3`; documentation-only and below review severity.

No source files were edited. This report is the only review artifact added.
