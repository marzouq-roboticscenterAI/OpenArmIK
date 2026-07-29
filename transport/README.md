# OpenArm SocketCAN transport

This directory provides a C++17 SocketCAN transport behind the versioned ISO-C
API in `include/openarm_transport.h`. It is a transport boundary, not a motor
controller or calibration implementation.

Opening a handle does not alter the interface, execute shell commands, request
privileges, enable a motor, or transmit a frame. Only strict DaMiao
register/status queries may be sent. The public C API has no
authority-issuing entry point: disable, motion, enable, and commissioning frames
are always rejected on a caller-opened handle.

The private controller integration boundary can issue one-shot, unpredictable,
transport-instance-bound, exact-frame authorities only for an explicitly
injected simulator/test backend. Every Linux SocketCAN backend, including
`vcan`, `vxcan`, and `slcan`, categorically refuses issuance. Authorities
expire within five seconds, cannot be replayed or moved between transports, and
never authorize untyped motion. Register writes must first round-trip through
the verified `openarm_can` codec; unknown IDs, payloads, modes, and non-finite or
out-of-range values fail closed.

The backend accepts only existing Linux classic-CAN interfaces with an exact
name and `CAN_MTU`. It installs caller-selected standard-ID filters, error-frame
reporting, software kernel timestamps, and receive-overflow diagnostics. Send
and receive use bounded absolute monotonic deadlines. Netlink is subscribed
before the initial link snapshot and queues every transition, including a down
followed immediately by recovery. `oa_transport_close()` signals blocked I/O
and then joins both operation directions before it returns, preventing a late
send or file-descriptor reuse race. Destroy follows completed close.

The library never infers motor identity, joint assignment, encoder sign/offset,
safe limits, or robot home. Those facts belong to a separately verified
commissioning manifest and higher controller lifecycle.

Build and test:

```sh
cmake -S can -B build/can -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX="$PWD/build/native-prefix"
cmake --build build/can --parallel
cmake --install build/can
cmake -S transport -B build/transport -DBUILD_TESTING=ON \
  -DCMAKE_PREFIX_PATH="$PWD/build/native-prefix"
cmake --build build/transport --parallel
ctest --test-dir build/transport --output-on-failure
```

The installed package exports `OpenArm::Transport`, retains
`OpenArm::openarm_transport`, and discovers its `OpenArm::Can` dependency.

The `vcan0` smoke test opens and closes only a sysfs-verified virtual interface.
It skips when such an interface is unavailable and never sends a CAN frame or
enables authority.
