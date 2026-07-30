# Physical runtime query boundary fix

Date: 2026-07-29 (America/Los_Angeles)
Branch: `fix/disable-physical-query`

## Result

`openarm_runtime` no longer links CAN-codec or transport code and the legacy
`OA_RUNTIME_BACKEND_SOCKETCAN_QUERY` backend cannot open a SocketCAN socket,
send a frame, wait for a response, or start discovery work. Its inventory-query
entry point clears a non-null output pointer then returns
`OA_RUNTIME_EUNSUPPORTED`, even for malformed/hostile query options.

The physical backend exposes only bounded read-only local CAN-link metadata
from `/sys/class/net`: at most `OA_RUNTIME_MAX_INTERFACES` rows, only
`ARPHRD_CAN` links from at most 256 inspected entries, with bitrate and CAN-FD
fields deliberately unknown. Those rows are explicitly metadata, not motor
evidence.

The physical query capability is clear. Physical configuration preview always
initializes an invalid, non-armable result with
`validation_status == OA_RUNTIME_EUNSUPPORTED` and returns that status;
configuration apply and physical calibration/motion paths remain unsupported.
Virtual inventory and virtual control behavior are unchanged.

The legacy physical test now proves capability absence, no FD/thread growth at
creation or after 1,000 unsupported queries, a two-second query/destroy bound,
eight concurrent query/apply callers racing destroy, cleared outputs, and an
unarmable preview. The resumed virtual-plan test also refreshes feedback on
each bounded retry and permits only transient `ESTALE`/`EBUSY` during the
documented scheduler handoff, eliminating its direct-run timing flake.

## Verification

All commands below used a single build job where compilation occurred.

- Release runtime CTest: `2/2` passed in `17.92 s` from
  `/var/tmp/openarmik-disable-query-release/build/runtime`.
- Focused Release repeat: `ctest --repeat until-fail:3` passed all three
  repetitions (`53.77 s`) from `/var/tmp/openarmik-runtime-query-quick`.
- RelWithDebInfo ASan+UBSan+leak CTest: `2/2` passed in `18.11 s`.
- RelWithDebInfo TSan CTest: `2/2` passed in `18.17 s`.
- Reconfigured/rebuilt installed consumers passed: all-header strict C11,
  all-header C++17, runtime-only C11, and runtime-only C++17.
- `cppcheck --enable=warning,performance,portability` on all runtime sources,
  `bash -n scripts/build_native.sh`, and `git diff --check` passed.
- Installed runtime export/declaration parity is `50/50`; the archive has no
  test-hook exports and no undefined `oa_can_*` or `oa_transport_*` symbols.
  The runtime-only installed executable has no Python, CAN, or transport
  dynamic dependency.

No CAN socket, vCAN interface, hardware, transmission, or GUI was used.

## Scope note

`c444405` is not an ancestor of this feature branch and is a portal-side main
commit; this work did not merge or modify main. The native helper itself still
uses unconstrained `cmake --build --parallel`, so it was not used as evidence
after the host-memory warning; current-code runtime, sanitizer, and installed
consumer checks above were invoked explicitly with `--parallel 1`.
