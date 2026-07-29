# Commission handle-registry correction

Date: 2026-07-29
Baseline: `125ce14`

## Resolution

The tombstoned pointer-wrapper registry was replaced with an active-only map from
monotonically issued `uintptr_t` tokens to owned calibration sessions. Public
opaque handles carry only these token values and are never dereferenced.

- Create allocates one implementation, issues a process-lifetime-unique token,
  and inserts only the active session.
- Destroy holds the same mutex as use, erases the entry, and immediately frees
  the implementation. There is no retired-entry or wrapper quarantine.
- Stale, arbitrary, double-destroyed, and cross-type tokens miss the active map.
  Allocator address reuse is irrelevant because addresses are not handles.
- Tokens are never reused. The counter refuses to issue its maximum value, so it
  cannot wrap to zero; exhaustion returns `OA_COMMISSION_ENOMEM` with a null
  output handle.

## Verification

- A compiled 500,000-cycle create/destroy regression checks active registry size
  throughout and after the loop, verifies the first token remains stale and the
  final token differs, and checks release-build resident-memory growth stays
  below 16 MiB.
- The optimized full C++ suite, including the stress loop, used 4,144 KiB maximum
  RSS on this host versus the rejected implementation's approximately 39,980
  KiB.
- Concurrent readers racing two destroy calls return only `OA_COMMISSION_OK`
  before destroy or `OA_COMMISSION_EINVAL` afterward. A deterministic post-
  destroy phase verifies stale rejection from four threads.
- A counter-exhaustion hook drives the actual issue path to its terminal value
  and verifies fail-closed `OA_COMMISSION_ENOMEM`, null output, and zero active
  entries.
- Fresh GCC 15 Werror ASan/UBSan/leak tests and the strict C11 consumer pass 2/2;
  the separate optimized build passes 2/2.

No transport, CAN frame, motor enable, FE/save/flash, register write, Python,
ROS, shell, or sudo surface was introduced.
