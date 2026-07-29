# OpenArm CAN diagnostics

`openarm_can` is a standalone C11 frame-codec and diagnostics library.  The codec
can construct control-capable MIT and enable frames, so the library as a whole is
not an incapable-of-control safety boundary.  It supplies no physical CAN
transport.  Its expected-ID probe sends only refresh-status frames, and the
in-memory diagnostics transport rejects MIT and enable frames.

All public input/output records begin with `struct_size` and `abi_version`.
Callers initialize both input and output records to `sizeof(record)` and
`OA_CAN_ABI_VERSION`; an undersized or incompatible record is rejected without
being overwritten.  Public status/type/policy fields use fixed-width integer
types rather than C enums.  Values are SI: rad, rad/s, and Nm.  MIT encoding
rejects NaN and infinities.  With
`OA_CAN_RANGE_REJECT`, any protocol-range violation returns `OA_CAN_ERANGE`.
With `OA_CAN_RANGE_SATURATE`, the encoded endpoint is returned and the bit in
`oa_can_encode_result.saturated_mask` identifies the affected field (position,
velocity, kp, kd, torque respectively).

The feedback decoder rejects extended/RTR/error IDs, nonstandard IDs, wrong DLC,
wrong arbitration IDs, and a mismatched embedded motor-ID nibble.  It preserves
the complete status/fault nibble.  Known fault nibbles return `OA_CAN_EFAULT`
after filling the decoded feedback record; unknown nibbles return `OA_CAN_EFRAME`.
MOS and rotor/coil temperatures are preserved as the protocol's direct unsigned
byte readings in degrees Celsius; the codec does not invent a safe thermal
threshold.

The typed DaMiao register API knows the exact pinned RID set, each RID's wire
type, and the read/write classification published by the upstream CLI.  Typed
queries and writes reject unknown RIDs, type mismatches, non-finite floats, known
register-range violations, invalid IDs, and writes to read-only registers.
Response decoding correlates arbitration ID, echoed target ID, opcode, RID, wire
type, and DLC before exposing a value.  Integer and IEEE-754 binary32 payloads
are encoded explicitly little-endian.  The older untyped query builder remains
available for ABI compatibility.

`oa_can_mit_profile_from_registers` only constructs a complete profile from
correlated PMAX, VMAX, and TMAX query results for one motor.  The profile-aware
MIT and feedback functions bind that profile to the queried send/receive IDs and
use those queried spans instead of a motor-family guess.  Profile-aware MIT,
POS_VEL, and POS_FORCE encoders always reject values
outside the verified mapping spans; they have no saturation mode.  POS_VEL's
second float is a non-negative maximum travel speed, not a signed velocity
target.  POS_FORCE uses the documented non-negative speed limit and `[0,1]`
per-unit current limit, not amperes.

Set-zero (`0xFE`), clear-error (`0xFB`), register-write (`0x55`), and flash-save
(`0xAA`) builders only construct frames.  They do not transmit them and do not
make those operations safe.  Set-zero saves the current encoder pose as zero; it
does not find robot home.  Flash-save and set-zero belong only in an isolated,
explicit commissioning lifecycle after disable and readback.  The in-memory
diagnostics transport continues to reject all of these state-changing frames.

The commissioned manifest contains local expected-ID, decode-range, joint-map,
and serial metadata.  `oa_can_probe_expected` verifies only that post-request,
pre-deadline feedback arrived at each expected arbitration ID, carried the
expected embedded ID nibble, and reported disabled status.  It does not verify
motor family, serial, physical joint assignment, firmware, configuration,
direction, gearing, limits, timeout, or control mode.  The caller-owned transport
must timestamp receives with a monotonic clock and honor the supplied deadline;
the probe also enforces a caller-selected receive-count bound.  Success requires
the transport to report `OA_CAN_ETIMEOUT` before `max_receive_frames` successful
receives.  Consuming exactly the budget is deliberately inconclusive and returns
`OA_CAN_ETIMEOUT`, even if its final frame completed the expected mask, because
another duplicate, enabled, or fault frame may remain unseen.  Set the budget
greater than the expected reply count and allow margin for unrelated traffic.

`oa_can_linux_list_interfaces` is Linux-only.  It issues a read-only rtnetlink
`RTM_GETLINK` dump addressed to the kernel and parses CAN link attributes; it
never calls `RTM_SETLINK`, uses no shell/system command, and never opens or binds
a CAN socket.  Parser tests use synthetic, deliberately unaligned and malformed
datagrams.  The test suite never invokes the live enumerator, so no host interface
is inspected during tests.

The codec/fake-transport surface is strict ISO C11.  CMake disables C language
extensions for both library and tests.  Rtnetlink inspection is an explicitly
Linux-UAPI feature: it uses Linux UAPI headers (including `linux/if.h`) and libc
socket calls, compiles only on Linux, and returns `OA_CAN_EUNSUPPORTED` elsewhere.
CMake avoids the Unix math-library dependency under MSVC; the verified
configuration for this revision is Linux/GCC plus the forced non-Linux stub
build recorded in the change handoff.

Build it independently:

```sh
cmake -S can -B build-can -DCMAKE_BUILD_TYPE=Release
cmake --build build-can --parallel
ctest --test-dir build-can --output-on-failure

cmake -S can -B build-can-sanitize -DCMAKE_BUILD_TYPE=Debug -DOA_CAN_ENABLE_SANITIZERS=ON
cmake --build build-can-sanitize --parallel
ctest --test-dir build-can-sanitize --output-on-failure
```

Installation exports the versioned `OpenArmCan` CMake package and its
`OpenArm::Can` target.
