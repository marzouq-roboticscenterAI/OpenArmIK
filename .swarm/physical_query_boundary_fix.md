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

## Follow-up: resumed-plan epoch handoff

Independent review found that the resumed-plan retry could occasionally receive
an unexpected status. The exact path is `OA_RUNTIME_EINVAL` with control
facility and lower code `OA_CONTROL_EINVAL`: `oa_runtime_plan_joint()` first
checked the requested sequence through one controller snapshot, released the
controller, and then called the lower planner. If the virtual worker advanced
in that gap, the lower planner's own sequence check returned `EINVAL` rather
than a stale-observation status. The same two-call gap existed for paired TCP
planning.

Both planning paths now hold `controller_mutex` across their runtime sequence
check and lower plan construction. An external snapshot that changes before
the planner acquires that epoch returns `OA_RUNTIME_ESTALE`; it cannot leak a
lower `EINVAL` after the runtime has accepted the sequence. The test retains
failure-only diagnostics that print the public status plus last-error status,
facility, and lower code if a future unexpected handoff result occurs.

Follow-up verification used fresh single-job Release build
`/var/tmp/openarmik-runtime-plan-handoff`: five sequential repeated runtime
CTest passes completed without the handoff failure. The current release build
also passed 2/2 in `17.92 s`; ASan+UBSan+leak passed 2/2 in `18.11 s`; and
TSan passed 2/2 in `18.18 s`. The installed C11/C++17 and runtime-only
C11/C++17 consumers, cppcheck, declaration/export `50/50` parity, and the
no-CAN/no-transport symbol audit passed again.
