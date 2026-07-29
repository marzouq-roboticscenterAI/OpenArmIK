# OpenArm CAN diagnostics

`openarm_can` is a standalone C11 protocol and diagnostics library.  It is not a
motor-control or commissioning library.  It can construct MIT, enable, disable,
refresh-status, and register-query frames, but exposes no transport that sends
control frames.  The generic probe sends only refresh-status frames and is
designed to run while torque is disabled.

All public input/output records begin with `struct_size` and `abi_version`.
Callers set both to `sizeof(record)` and `OA_CAN_ABI_VERSION`.  Values are SI:
rad, rad/s, and Nm.  MIT encoding rejects NaN and infinities.  With
`OA_CAN_RANGE_REJECT`, any protocol-range violation returns `OA_CAN_ERANGE`.
With `OA_CAN_RANGE_SATURATE`, the encoded endpoint is returned and the bit in
`oa_can_encode_result.saturated_mask` identifies the affected field (position,
velocity, kp, kd, torque respectively).

The feedback decoder rejects extended/RTR/error IDs, nonstandard IDs, wrong DLC,
wrong arbitration IDs, and a mismatched embedded motor-ID nibble.  It preserves
the complete status/fault nibble.  Known fault nibbles return `OA_CAN_EFAULT`
after filling the decoded feedback record; unknown nibbles return `OA_CAN_EFRAME`.

The commissioned manifest describes expected IDs, model family, identity label,
and explicit motor-to-joint scale/offset terms.  It is a verification contract,
not a discovery or commissioning mechanism.  `oa_can_probe_expected` emits only
refresh-status requests, receives through a caller-owned transport, and reports
missing, duplicate, fault, malformed, and unexpected feedback.

`oa_can_linux_list_interfaces` is Linux-only.  It issues a read-only rtnetlink
`RTM_GETLINK` dump and parses CAN link attributes; it never calls `RTM_SETLINK`,
uses no shell/system command, and never opens or binds a CAN socket.  The test
suite does not invoke it, so no host interface is inspected during tests.

Build it independently:

```sh
cmake -S can -B build-can -DOA_CAN_ENABLE_SANITIZERS=ON
cmake --build build-can --parallel
ctest --test-dir build-can --output-on-failure
```
