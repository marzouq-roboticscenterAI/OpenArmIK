# SocketCAN review remediation

Status: **DONE**  
Base review: `.swarm/socketcan_review.md` against `5dd2493`

## Finding resolution

### Critical 1 — forgeable public capability

Resolved by deleting caller-supplied permission/expiry records from the public
C ABI. `oa_transport_open()` is query-only and the public send path has no way
to acquire dangerous authority. The private C++ integration boundary now issues
opaque one-shot `Authority` objects backed by a per-`Transport` registry. A
token carries an unpredictable random ID plus a two-word instance nonce and is
bound in the registry to one exact frame, class, instance, and expiry no more
than five seconds away. It is erased before the syscall, so failure cannot make
it replayable. Cross-instance, wrong-frame, expired, over-expiry-deadline, and
replayed tokens are rejected.

SocketCAN backends return `OA_TRANSPORT_EPERMISSION` from authority issuance
regardless of driver, name, or sysfs location. Only explicit injected simulator/
test backends can issue, preserving the Stage-A physical query-only gate. There is no public
issuer, generic control bit, or commissioning bit.

### Critical 2 — permissive raw-frame classification

Resolved by classifying all ordinary standard frames as unknown; raw MIT,
POS_VEL, POS_FORCE, feedback-like, unrelated, and malformed payloads are rejected
regardless of authority. The system-ID branch now runs before special-command
matching, so `0x7ff/FF...` cannot masquerade as a motor special. Every motor
special is restricted below `0x7ff`.

Register writes are reconstructed using the verified `openarm_can` RID metadata
and `oa_can_make_register_write()`, then compared byte-for-byte. This rejects
read-only/unknown RIDs, illegal modes/bitrates/IDs, non-finite floats, invalid
protection/PVT values, and padding/layout errors. Known destructive shapes can
only run on an injected simulator/test backend with an exact one-shot authority; physical
set-zero/save/write/clear remains unreachable.

### Important 1 — close was not a completed-operation barrier

Resolved at both layers. Close first publishes closure and signals eventfd, then
joins the backend send/receive mutexes. `Transport::close()` subsequently joins
both public operation mutexes and clears unconsumed tokens before returning.
Concurrent close calls serialize. This preserves wakeup speed but guarantees no
operation can return success after close completes and no socket FD is destroyed
while an operation can still use it.

### Important 2 — missed/collapsed link transitions

Resolved by opening/subscribing the route-netlink socket before the initial
read-only interface snapshot. A bounded no-allocation parser preserves every
matching `RTM_NEWLINK`/`RTM_DELLINK` state in datagram order. The backend queues
each actual state transition in a FIFO; a down followed by up cannot collapse
to up. Queue overflow, `NLMSG_ERROR`, `NLMSG_OVERRUN`, truncation, and malformed
alignment fail closed.

### Minor 1 — deadline equality mismatch

Resolved with one rule: `deadline <= now` is timed out. `Transport`, SocketCAN
send/wait/receive, fake backends, and immediate queued link events all use the
same boundary.

## Targeted regressions

- arbitrary raw motion and unknown special/system payloads never reach backend;
- illegal control-mode and NaN PMAX register writes remain unknown;
- opaque token cross-instance, wrong-frame, replay, expiry, and physical-backend
  attacks fail;
- simultaneous blocked send and receive both return closed before close returns;
- synthetic down/delete then up netlink batch returns two ordered transitions;
- netlink overrun and truncation fail closed;
- exact deadline equality times out for send and receive;
- strict C11 public consumer and output-preservation tests remain green.

## Verification

- GCC 15.2 Release/Werror: 3/3 tests passed.
- ASan + UBSan with leak detection/halt-on-error: 3/3 tests passed.
- ThreadSanitizer with halt-on-error: 3/3 tests passed.
- `vcan0` unavailable, so the read-only virtual smoke test skipped; no interface
  was created, configured, or transmitted on.

No physical CAN device was opened and no CAN frame was transmitted during the
implementation or remediation.
