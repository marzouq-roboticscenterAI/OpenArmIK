# OpenArm Stage-A controller core

`libopenarm_control` is a C++17 controller behind the versioned ISO-C header
`include/openarm_control.h`. It has no Python, ROS, shell, `sudo`, or hardware
write path. The Stage-A backend is a deterministic DaMiao encoder simulator.
The physical backend is deliberately present only as a fail-closed gate and
returns `OA_EUNSUPPORTED` before verification or motion.

The manifest is built with `oa_manifest_create` from fixed-width C records and
copied into immutable validated storage. Validation rejects duplicate buses,
IDs and serials, missing identity/version/watchdog data, incorrect motor family
or PMAX/VMAX/TMAX/gear profiles, non-unit arm mappings, and limits outside the
pinned URDF. Text/digest loading is reserved and returns
`OA_EUNSUPPORTED`; normal runtime never edits a manifest.

State is encoder-derived. J1--J7 use the output-shaft relationship
`q=a*qout+b`, `dq=a*dqout`, `tau=tauout/a`; the integrated DaMiao gearing is
validated but is never applied a second time. Every plan binds the complete
fresh measured sequence. Joint and TCP completion require new measured q/dq;
TCP completion additionally uses FK of the measured joints.

Motion uses a synchronized seventh-order smooth time law. Single-joint plans
hold the other six measured starting coordinates. Paired TCP plans run bounded
position IK for both arms using only measured q as the initial seed. The model
has no collision engine, so the default `OA_COLLISION_REJECT_ALL` blocks every
plan. Tests may opt into `OA_COLLISION_VIRTUAL_UNCHECKED`; reports continue to
state `collision_checked == 0`.

Build and test without Python:

```sh
cmake -S control -B control/build -DCMAKE_BUILD_TYPE=Release
cmake --build control/build --parallel
ctest --test-dir control/build --output-on-failure
```

Sanitizer verification:

```sh
cmake -S control -B control/build-sanitize -DCMAKE_BUILD_TYPE=Debug \
  -DOA_CONTROL_SANITIZERS=ON
cmake --build control/build-sanitize --parallel
ctest --test-dir control/build-sanitize --output-on-failure
```
