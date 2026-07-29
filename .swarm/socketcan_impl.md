# SocketCAN transport implementation

Status: **DONE — transport foundation, hardware-free**  
Branch: `impl/socketcan`

## Scope delivered

- Added standalone `transport/` CMake project with modular C++17 `Transport`,
  abstract `Backend`, and Linux `SocketCanBackend` classes behind
  `openarm_transport.h`, a versioned ISO-C opaque-handle API.
- Public frames are fixed classic-CAN eight-byte records. Opens require an
  exact existing interface name, `ARPHRD_CAN`, and `CAN_MTU`; CAN-FD MTU,
  extended/RTR frames, malformed records, oversized or ambiguous filter values,
  and invalid output ABI records are rejected.
- Opens never transmit or mutate the host link. There is no netlink SETLINK,
  ioctl mutation, subprocess, shell, sudo, motor enable, automatic probing, or
  bitrate cycling. Netlink is receive-only and monitors the bound interface for
  link changes.
- Caller-defined standard-ID receive filters are installed with EFF/RTR
  exclusion. CAN error frames are subscribed separately, parsed, and exposed
  with their Linux error mask. The receive record also carries a monotonic
  dequeue time, an explicitly realtime kernel software timestamp when present,
  cumulative/delta `SO_RXQ_OVFL` diagnostics, and link events.
- Sockets are nonblocking. Absolute `CLOCK_MONOTONIC` send/receive deadlines are
  mandatory and capped by a configurable horizon no greater than 60 seconds.
  Sends recheck the deadline immediately before the syscall. Per-direction
  mutexes serialize concurrent senders and receivers, while an eventfd makes
  idempotent `oa_transport_close()` interrupt either blocked direction without
  waiting for its deadline.
- The frame classifier recognizes exact DaMiao register query, status query,
  register write, save-parameters, enable, disable, set-zero, clear-error, and
  generic motion shapes. Unknown and malformed system/special frames fail
  closed. Known register IDs and the codec's conservative writable RID set are
  enforced at the transport boundary.
- With no capability record, only register/status queries can transmit.
  Disable, enable, motion, writes, zero, clear, and save are rejected before
  reaching the backend. Disable/control/commission permissions require an
  explicit record; every dangerous record needs a future monotonic expiry, and
  expiry is checked again on every send. An I/O deadline may not extend beyond
  that expiry, so a queued frame cannot be transmitted after authority lapses.
  Commission permission does not imply motion permission, and control permission
  does not imply commissioning.

## Test coverage

- Injectable fake backend tests cover all frame classifications, malformed
  query/special rejection, query-only no-write behavior, distinct control and
  commissioning capabilities, expiry, expired/overlong deadlines, event
  diagnostics, per-call output preservation, and close interrupting a blocked
  receive from another thread.
- The public boundary is consumed by a separately compiled C11 executable.
- Exact interface rejection is exercised with the non-CAN loopback device.
- A `vcan0` smoke test verifies through sysfs that the device is virtual, then
  opens, observes the initial link event, and closes without transmitting. It
  safely skipped on this host because `vcan0` is unavailable; no interface was
  created or reconfigured.

## Fresh verification

Strict GCC 15.2 Release build (`-Werror`, conversion/sign/shadow/format checks):

```text
cmake -S transport -B /tmp/openarmik-socketcan-release \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build /tmp/openarmik-socketcan-release --parallel
ctest --test-dir /tmp/openarmik-socketcan-release --output-on-failure
3/3 tests passed
```

Debug ASan + UBSan build with leak detection and halt-on-error:

```text
cmake -S transport -B /tmp/openarmik-socketcan-sanitize \
  -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON \
  -DOA_TRANSPORT_ENABLE_SANITIZERS=ON
cmake --build /tmp/openarmik-socketcan-sanitize --parallel
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1 \
ctest --test-dir /tmp/openarmik-socketcan-sanitize --output-on-failure
3/3 tests passed
```

## Explicit non-claims

No physical CAN interface was opened, no CAN frame was sent to an arm, and no
motor was enabled, calibrated, or moved. The transport does not decide motor
identity, side/joint assignment, encoder sign/offset, PMAX/VMAX/TMAX correctness,
physical limits, collision safety, robot home, or E-stop policy. A higher
controller must still bind this transport to a verified immutable commissioning
manifest and keep physical motion behind the approved Stage-C qualification
gates.
