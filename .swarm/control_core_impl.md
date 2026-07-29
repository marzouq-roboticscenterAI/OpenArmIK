# Stage-A controller core implementation report

Date: 2026-07-29
Branch: `impl/control-core`

## Delivered

- New standalone `control/` CMake project building C++17
  `libopenarm_control` behind `include/openarm_control.h`, an ISO-C V1 ABI.
- One opaque `oa_controller` owner and opaque immutable manifest/plan handles.
  Every public record starts with fixed-width `struct_size, abi_version`; all C
  entry points contain C++ exceptions and validate records before output writes.
- The previously omitted `oa_controller_get_arm_challenge` operation is in the
  frozen V1 surface. Arming and fault reset require fresh, expiring nonces tied
  to the current verification epoch.
- A compiled manifest builder that copies into immutable storage and rejects
  wrong/missing identity, versions, watchdog, motor profiles, PMAX/VMAX/TMAX,
  gearbox metadata, joint names/order, URDF limits, duplicate bus IDs/serials,
  duplicate buses, non-unit affine mappings, and non-finite/dynamic-limit data.
  `oa_manifest_load` remains an explicit unsupported reservation because Stage A
  selected the approved compiled C config-builder alternative; normal runtime
  has no manifest write/edit path.
- Modular OOP core: `Manifest`, `Controller`, `ArmRuntime`,
  `DamiaoMotorSimulator`, `FakeTransport`, `MotionPlan`, and the kinematics
  adapter. No Python, ROS, shell, privilege escalation, or hardware CAN write is
  present.
- Encoder snapshots are invalid until a complete fresh generation exists. The
  only state mapping is `q=a*qout+b`, `dq=a*dqout`, `tau=tauout/a`; arm joints
  require `abs(a)==1`, and the validated 9:1/10:1/40:1 actuator gearing is never
  applied again.
- Fail-closed lifecycle, producer/feedback/plan expiry watchdogs, latched faults,
  global two-arm disable on either-arm failure, fixed-capacity event ring, and
  deterministic freeze/drop/fault injection.
- Synchronized seventh-order smooth trajectories with conservative analytic
  velocity/acceleration/jerk duration bounds. Individual-joint plans command the
  full seven-joint arm and hold the other six measured start coordinates.
- Measured FK provides every URDF joint origin/axis plus hand TCP XYZ. Paired TCP
  position IK starts only from the two current measured encoder-derived q
  vectors, binds both feedback sequences and all revisions, and executes as one
  bimanual plan.
- Completion is never inferred from the reference or elapsed time. It requires
  three fresh measured q/dq dwell cycles; TCP completion additionally requires
  measured FK residual. A frozen encoder advances command references but never
  completes.
- Collision checking defaults to `RejectAll`. Only an explicitly selected
  virtual-only unchecked policy can plan/execute, and its report retains
  `collision_checked == 0`. The physical backend returns `OA_EUNSUPPORTED`
  before verification and sends no CAN traffic.

## Fresh verification

Release/Werror:

```text
cmake -S control -B /tmp/openarmik-control-build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/openarmik-control-build --parallel
ctest --test-dir /tmp/openarmik-control-build --output-on-failure
100% tests passed, 0 tests failed out of 2
```

ASan/UBSan:

```text
cmake -S control -B /tmp/openarmik-control-asan -DCMAKE_BUILD_TYPE=Debug \
  -DOA_CONTROL_SANITIZERS=ON
cmake --build /tmp/openarmik-control-asan --parallel
ctest --test-dir /tmp/openarmik-control-asan --output-on-failure
100% tests passed, 0 tests failed out of 2
```

The C++ suite covers valid/wrong manifests, physical rejection, collision
rejection, ABI canary preservation, invalid numeric/limit requests, exact
position/velocity/effort affine consistency, FK, measured single-joint
convergence, frozen encoder non-completion and expiry, stale feedback latching,
fault reset/reverify, paired measured TCP convergence, and an injected fault on
one bus while a paired command is executing that disables both arms. A separate
source is compiled as strict C11 and links/calls the C++ library.

## Hardware boundary

This is only Stage A. It does not claim physical detection, calibration, CAN
timing, collision safety, or movement. SocketCAN writes, motor register changes,
saved-zero commands, automatic/manual calibration, and commissioning belong to
the separately gated later stages described in `controller_design_final.md`.
