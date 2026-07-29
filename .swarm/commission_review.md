# Final independent review: commissioning/calibration (`f4a3215`)

Verdict: **CLEAN**

The final active-only monotonic-token registry corrects the remaining handle
lifetime issue without reopening any original functional or ABI finding.

## Handle registry verification

- Active sessions alone occupy the registry. Destroy erases the entry and frees
  its owned session under the same mutex used by operations.
- Tokens increase monotonically and are independent of allocator addresses, so
  address reuse cannot create ABA. Destroyed, arbitrary, double-destroyed, and
  cross-type tokens miss the active map without dereference.
- Concurrent use/destroy is serialized. The adversarial four-reader/two-destroy
  test observed only `OA_COMMISSION_OK` before removal and
  `OA_COMMISSION_EINVAL` afterward, with no sanitizer finding.
- The maximum token is never issued. Forced counter exhaustion returns
  `OA_COMMISSION_ENOMEM`, leaves the output handle null, and leaves zero active
  registry entries; the counter cannot wrap to zero or reuse a token.
- The optimized 500,000 create/destroy regression kept the active count at zero,
  left the first token stale, issued a distinct final token, and completed with
  approximately 3,932 KiB maximum RSS for the full test executable. This is
  bounded baseline allocator/process memory rather than per-session retention
  (the rejected tombstone implementation used approximately 39,980 KiB).

## Original findings re-verified

- Exact four- and eight-byte C records are rejected with
  `OA_COMMISSION_EABI` before later fields are read. The original standalone
  8-byte ASan repro returns `OA_COMMISSION_EABI` with no sanitizer finding.
- C outputs preserve surrounding canaries; invalid/stale/cross-type handles,
  allocation failure, and injected C++ exceptions are contained.
- Manual stale, enabled, faulted, or malformed samples latch abort, clear
  accumulated evidence, and permanently exclude review/commit. Stable dwell,
  one-reference known-sign math, and two-reference sign/scale math remain sound.
- The hard-stop session enforces the original qualified envelope through first
  approach/dwell, retreat, reapproach, and second dwell; phase deadlines,
  freshness, E-stop/deadman, enabled state, speed, torque, temperature, contact
  energy, travel, two-contact dwell, and repeatability all fail closed.
- Arm recipes reject simulation mode and require qualification/fixture revisions
  plus all-other-joint posture on every active step. Simulation-only gripper
  patches retain distinct evidence kind, record, and revision.
- Every state-machine failure or explicit abort excludes commit and leaves the
  caller patch unchanged.
- No FE zero, save/flash/register write, motor frame, SocketCAN/netlink,
  transport, shell, Python, ROS, or hardware-command surface exists in
  `commission/`.

## Fresh evidence

- GCC 15 Debug, `-Wall -Wextra -Wpedantic -Werror`, ASan, UBSan, leak detection:
  passed.
- Debug sanitizer `ctest --output-on-failure`: 2/2 passed.
- Optimized Release `ctest --output-on-failure`: 2/2 passed.
- Direct optimized full test executable, including 500,000 registry cycles and
  concurrency/exhaustion tests: passed; approximately 3,932 KiB maximum RSS.
- Exact standalone short-record ASan repro: returned `OA_COMMISSION_EABI` with
  no sanitizer finding.
- `git diff 125ce14..f4a3215 --check`: clean.

No source edits were made; this report is the only review update.
