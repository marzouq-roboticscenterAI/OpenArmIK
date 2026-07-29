# Independent re-review: SocketCAN transport

Commit: `446c494` (corrective commit over `5dd2493`)

Verdict: **CHANGES REQUIRED — 2 Important, 1 Minor**

The corrective commit resolves all five original findings: the public ABI is
query-only; private authorities are random, instance-bound, exact-frame,
five-second, one-shot grants; unknown/MIT/POS/raw frames fail closed; register
writes round-trip through the checked codec; close joins both I/O directions;
netlink batches preserve down/up order; and deadline equality consistently times
out. Targeted token-forging, replay, cross-instance, wrong-frame, expiry,
physical-backend-fake, classifier, close/send/receive, and synthetic netlink
tests pass.

## Remaining findings

### Important 1 — “Virtual” sysfs location does not prove that a CAN interface cannot reach hardware

`SocketCanBackend::permitsAuthorityIssuance()` trusts
`isVirtualInterface()`, which returns true for any CAN netdev whose sysfs path
contains `/devices/virtual/net/` (`transport/src/socketcan_backend.cpp:219-221,483-493`).
That is broader than `vcan`/`vxcan`. Linux `slcan` is a serial-line CAN driver
(`modinfo slcan` describes it as “serial line CAN interface”); its dynamically
created netdev can live under the virtual-net sysfs hierarchy while transmitting
through a physical serial CAN adapter. The private integration API could
therefore issue enable/write/zero/save authority on a physical bus, contrary to
the Stage-A categorical refusal claim.

Authority issuance must allowlist an attested non-hardware kind (normally exact
rtnetlink `IFLA_INFO_KIND == "vcan"`, with `vxcan` only if deliberately accepted),
not infer safety from sysfs placement. The current test uses only
`FakeBackend(false)` and does not exercise SocketCAN interface-kind detection.

### Important 2 — The installed static C API archive has unresolved codec dependencies

The fix makes `openarm_transport` call `oa_can_register_info_for_id()` and
`oa_can_make_register_write()`, linking `openarm_can` privately in the build tree
(`transport/CMakeLists.txt:62-78`). Installation still emits only
`libopenarm_transport.a` and `openarm_transport.h`; it neither bundles nor
installs/exports `libopenarm_can.a` and dependency metadata. A fresh install
followed by compiling the provided strict C11 consumer and linking it against the
installed transport archive fails with undefined references to both codec
symbols. The built-tree C11 test passes because CMake silently supplies the
private static dependency.

Install a self-contained combined archive/shared library, or install/export the
codec target and a CMake/pkg-config dependency contract, then add an
install-tree consumer test.

### Minor 1 — Netlink socket truncation is not reliably detected

`drainLinkEvents()` receives into 8192 bytes with plain `recv(...,
MSG_DONTWAIT)` (`transport/src/socketcan_backend.cpp:311-330`). The parser
rejects a buffer cut inside a message, but plain `recv` does not report the
original datagram size. If an oversized netlink datagram is truncated exactly at
a valid message boundary, parsing succeeds and later transitions are silently
lost despite the report's claim that truncation fails closed. Use `recvmsg()` and
check `MSG_TRUNC`, or request `MSG_TRUNC` and reject a returned length larger than
the buffer. The synthetic test truncates a parser buffer mid-message and does
not cover socket-level truncation.

## Fresh verification

- GCC 15.2 Release/Werror build and `ctest`: **PASS**, 3/3.
- Debug ASan+UBSan with leak detection/halt-on-error: **PASS**, 3/3.
- Debug ThreadSanitizer with halt-on-error: **PASS**, 3/3.
- TSan authority/classifier/close/netlink test executable repeated 100 times:
  **PASS**.
- `vcan0` was unavailable, so the read-only smoke test skipped. No CAN interface
  was created, configured, opened, or transmitted on; no hardware/link mutation
  occurred.
- Install-tree strict C11 consumer link: **FAIL**, confirming Important 2.
- `git diff --check 5dd2493 446c494`: only trailing whitespace in review/fix
  Markdown status lines; below review severity.

No source files were edited. This report is the only re-review artifact changed.
