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
