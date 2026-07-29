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
  mutexes serialize concurrent senders and receivers. Close publishes closure,
  signals both directions through eventfd, and joins both backend and public
  operation locks before returning, so no send/receive can succeed later and no
  descriptor can be reused while an operation is still live.
- The frame classifier recognizes exact DaMiao register query, status query,
  semantically valid register write, save-parameters, enable, disable, set-zero,
  and clear-error shapes. Register writes are reconstructed through the verified
  `openarm_can` codec, rejecting illegal IDs/modes/bitrates, non-finite values,
  and range failures. Every other ordinary frame, including all untyped raw
  motion, is unknown and rejected regardless of authority.
- The installed C ABI is permanently query-only. The private integration layer
  uses unpredictable one-shot tokens stored in a per-transport nonce registry,
  bound to one exact frame, instance, class, and expiry of at most five seconds.
  They cannot cross instances or replay. Physical SocketCAN backends categorically
  refuse issuance; only sysfs-verified virtual/test backends may issue. This keeps
  physical enable, motion, zero, write, clear, and save unavailable in Stage A.
- Netlink subscribes before the first interface-state snapshot. Every matching
  down/delete/up transition is parsed into a bounded FIFO instead of collapsing
  a batch to its final state; overflow or malformed/overrun netlink data fails
  closed.

## Test coverage

- Injectable fake backend tests cover malformed/unknown/motion exploit rejection,
  verified register-write semantics, public query-only behavior, cross-instance
  and wrong-frame token rejection, one-shot replay prevention, physical authority
  refusal, expiry, exact deadline equality, and diagnostic event preservation.
- A simultaneous blocked-send/blocked-receive race proves close wakes and joins
  both operations. Synthetic netlink datagrams prove down followed by up remains
  two ordered transitions and malformed/overrun input fails closed.
- The public boundary is consumed by a separately compiled C11 executable.
- Exact interface rejection is exercised with the non-CAN loopback device.
- A `vcan0` smoke test verifies through sysfs that the device is virtual, then
  opens, observes the initial link event, and closes without transmitting. It
  safely skipped on this host because `vcan0` is unavailable; no interface was
  created or reconfigured.

## Fresh verification

Strict GCC 15.2 Release build (`-Werror`, conversion/sign/shadow/format checks):

```text
cmake -S transport -B /tmp/openarmik-socketcan-release2 \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build /tmp/openarmik-socketcan-release2 --parallel
ctest --test-dir /tmp/openarmik-socketcan-release2 --output-on-failure
3/3 tests passed
```

Debug ASan + UBSan build with leak detection and halt-on-error:

```text
cmake -S transport -B /tmp/openarmik-socketcan-sanitize2 \
  -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON \
  -DOA_TRANSPORT_ENABLE_SANITIZERS=ON
cmake --build /tmp/openarmik-socketcan-sanitize2 --parallel
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1 \
ctest --test-dir /tmp/openarmik-socketcan-sanitize2 --output-on-failure
3/3 tests passed
```

Debug ThreadSanitizer build:

```text
cmake -S transport -B /tmp/openarmik-socketcan-tsan \
  -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON \
  -DOA_TRANSPORT_ENABLE_THREAD_SANITIZER=ON
cmake --build /tmp/openarmik-socketcan-tsan --parallel
TSAN_OPTIONS=halt_on_error=1 \
ctest --test-dir /tmp/openarmik-socketcan-tsan --output-on-failure
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
