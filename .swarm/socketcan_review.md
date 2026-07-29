# Final independent review: SocketCAN transport

Commit: `321366a` through the full `5dd2493..321366a` corrective series  
Verdict: **CLEAN**

No Critical, Important, or Minor findings remain in the scoped transport,
authorization, classifier, SocketCAN/netlink backend, C ABI, packaging, tests,
or documentation.

## Final finding disposition

- **Stage-A physical authority:** fixed. The installed C API is permanently
  query-only and has no authority issuer. Every Linux SocketCAN backend returns
  false from authority issuance regardless of interface name, sysfs placement,
  or driver, so physical CAN, `slcan`, `vcan`, and `vxcan` cannot acquire a
  dangerous token. Only an explicitly injected simulator/test backend can issue.
- **Token forging/replay:** fixed. Private authorities use unpredictable IDs and
  a per-transport two-word nonce, live only in the issuing transport's registry,
  bind one exact frame/class/instance, expire within five seconds, require the
  I/O deadline not exceed expiry, and are erased before the backend syscall.
  Cross-instance, wrong-frame, expired, over-deadline, and replay attempts fail.
- **Frame classification:** fixed. Exact padded `0x7ff` register/status queries
  remain the only public sends. Save/write and motor specials are recognized
  exactly but require unavailable public authority; `0x7ff` cannot masquerade as
  a motor special. Register writes round-trip byte-for-byte through the verified
  codec, rejecting invalid target/RID/type/value/mode/bitrate/nonfinite/range
  inputs. Raw MIT, POS_VEL, POS_FORCE, feedback-like, malformed, and unrelated
  standard frames remain unknown and are rejected before the backend.
- **Shutdown/concurrency/lifetime:** fixed. Close publishes closure, signals the
  persistent eventfd, joins backend send/receive locks, then joins public
  operation locks and clears grants. Blocked send and receive return closed;
  neither can succeed after close returns. Concurrent close is serialized and
  the documented destroy-after-completed-calls ownership rule remains sound.
- **Deadlines/timestamps/diagnostics:** fixed/verified. Equality consistently
  means timeout, horizons are bounded, send rechecks immediately before syscall,
  and timestamp clock labeling, CAN-error masks, overflow total/delta, output
  preservation, malformed-frame rejection, and EFF/RTR filtering remain sound.
- **Netlink transitions/truncation/sender:** fixed. Subscription precedes the
  initial snapshot; matching transitions are queued in order and down/up cannot
  collapse. The bounded parser fails closed on malformed, overrun, and overflow.
  `recvmsg(MSG_TRUNC)` now checks both returned original length and `MSG_TRUNC`,
  validates a full `AF_NETLINK` sender address with kernel PID zero, and rejects
  before parsing on any mismatch.
- **Installed C link contract:** fixed. The required strict-C11 codec object is
  embedded in the static transport archive. Installation exports the versioned
  `OpenArm::openarm_transport` CMake target, header, Threads/C++ link contract,
  and package/version files. A separate clean project finds only that installed
  package, compiles the C11 consumer, links, and runs successfully.
- **Socket/interface safety:** retained. Exact bounded interface names, ifindex
  round-trip, `CAN_MTU`, `ARPHRD_CAN`, getter-only ioctls, initialized
  `sockaddr_can`, nonblocking/CLOEXEC descriptors, receive-only route netlink,
  and absence of shell/sudo/SETLINK/setter ioctl remain verified.

## Fresh verification

- GCC 15.2 Release/Werror configure/build and `ctest`: **PASS**, 4/4, including
  the staged external install-tree C11 consumer.
- Debug ASan+UBSan with leak detection and halt-on-error: **PASS**, 3/3.
- Debug ThreadSanitizer with halt-on-error: **PASS**, 3/3.
- Export inspection: all seven `oa_transport_*` C entry points are present and
  unmangled; the codec symbols needed by classification are contained in the
  installed archive.
- `git diff --check 446c494 321366a`: one trailing-space warning only in
  `.swarm/socketcan_rereview_fix.md:3`; documentation-only and below severity.
- `vcan0` was unavailable, so the smoke test safely skipped. No CAN interface
  was created, reconfigured, or transmitted on; no physical device was opened
  and no hardware or link state was mutated during review.

No source files were edited. This report is the only review artifact changed.
