# OpenArm SocketCAN transport

This directory provides a C++17 SocketCAN transport behind the versioned ISO-C
API in `include/openarm_transport.h`. It is a transport boundary, not a motor
controller or calibration implementation.

Opening a handle does not alter the interface, execute shell commands, request
privileges, enable a motor, or transmit a frame. With no capability record, only
strict DaMiao register/status queries may be sent. Disable, motion, enable, and
commissioning frames are rejected. Dangerous permissions require an explicit
future `CLOCK_MONOTONIC` expiry and are checked again on every send.

The backend accepts only existing Linux classic-CAN interfaces with an exact
name and `CAN_MTU`. It installs caller-selected standard-ID filters, error-frame
reporting, software kernel timestamps, and receive-overflow diagnostics. Send
and receive use bounded absolute monotonic deadlines. `oa_transport_close()` is
thread-safe and interrupts blocked I/O; destroy is performed after users have
joined their I/O threads.

The library never infers motor identity, joint assignment, encoder sign/offset,
safe limits, or robot home. Those facts belong to a separately verified
commissioning manifest and higher controller lifecycle.

Build and test:

```sh
cmake -S transport -B build/transport -DBUILD_TESTING=ON
cmake --build build/transport --parallel
ctest --test-dir build/transport --output-on-failure
```

The `vcan0` smoke test opens and closes only a sysfs-verified virtual interface.
It skips when such an interface is unavailable and never sends a CAN frame.
