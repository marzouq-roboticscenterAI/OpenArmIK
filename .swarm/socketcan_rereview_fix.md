# SocketCAN second re-review remediation

Status: **DONE**  
Corrective commit base: `446c494`

## Remaining findings resolved

### Important 1 — sysfs virtual path could include hardware-backed SLCAN

All Linux `SocketCanBackend` instances now return false from authority issuance,
without inspecting driver kind, interface name, or sysfs path. This includes
`vcan`, `vxcan`, `slcan`, physical PCI/USB CAN, and future CAN drivers. Only an
explicitly injected simulator/test `Backend` can issue a private exact-frame
token. The regression directly asserts the SocketCAN authority policy is false;
the existing issuance tests prove only a deliberately injected fake can issue.

### Important 2 — installed static archive had unresolved codec dependencies

The verified `openarm_can.c` codec is compiled as a strict C11 object and embedded
inside `libopenarm_transport.a`; the transport no longer has an uninstalled
`libopenarm_can.a` dependency. Installation exports
`OpenArm::openarm_transport`, its Threads/C++ link contract, public header,
version file, and package configuration.

A CTest install-tree regression performs a clean staged install, configures a
separate project through `find_package(OpenArmTransport CONFIG)`, compiles the
strict C11 consumer, links only the installed exported target, and runs it. This
test passes in Release.

### Minor 1 — socket-level netlink truncation could look parseable

Route-netlink now uses `recvmsg(MSG_TRUNC | MSG_DONTWAIT)`, validates the kernel
sender address, and rejects either `MSG_TRUNC` or a returned original datagram
length larger than the receive buffer before parsing. Zero/short datagrams and
short sender addresses fail closed; the parser continues to reject partial
headers/messages. Regression cases cover a full-length datagram marked truncated
and an original length exceeding capacity, including the exact-valid-message
boundary case that plain `recv()` could previously miss.

## Verification

- GCC 15.2 Release/Werror: 4/4 passed, including external installed C11 consumer.
- ASan + UBSan with leak detection/halt-on-error: 3/3 passed.
- ThreadSanitizer with halt-on-error: 3/3 passed.
- `git diff --check`: clean.
- `vcan0` unavailable; smoke test skipped without interface creation, mutation,
  or traffic.

No physical CAN interface was opened and no CAN frame was transmitted.
