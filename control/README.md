# OpenArm Stage-A controller core

`libopenarm_control` is a C++17 controller behind the versioned ISO-C header
`include/openarm_control.h`. It has no Python, ROS, shell, `sudo`, or hardware
write path. The Stage-A backend is a deterministic DaMiao encoder simulator
with an independent bounded-velocity/acceleration plant. Commands update only
the plant reference; the lagging plant emits quantized eight-byte DaMiao
feedback frames, and only decoded frames update measured state.
The physical backend is deliberately present only as a fail-closed gate and
returns `OA_CONTROL_EUNSUPPORTED` before verification or motion.

Public controller results use the ABI-neutral `oa_control_status` typedef and
`OA_CONTROL_*` constants. The original V1 `oa_status` and `OA_*` source names
remain available to control-only translation units. A translation unit
combining model and control must define
`OPENARM_DISABLE_LEGACY_GENERIC_STATUS` before either header and use the
module-prefixed API; otherwise both include orders fail with a diagnostic
instead of silently assigning generic names different meanings. This changes
no function symbol, calling convention, status representation, numeric value,
or V1 record layout.

Installed CMake consumers use `find_package(openarm_control CONFIG REQUIRED)`
and link `openarm_control::openarm_control`. The package discovers and links
its installed `openarm_model::openarm_model` dependency.

The manifest is built with `oa_manifest_create` from fixed-width C records and
copied into immutable validated storage. Validation rejects duplicate buses,
IDs and serials, missing identity/version/watchdog data, incorrect motor family
or PMAX/VMAX/TMAX/gear profiles, non-unit arm mappings, and limits outside the
pinned URDF. Text/digest loading is reserved and returns
`OA_EUNSUPPORTED`; normal runtime never edits a manifest.

State is encoder-derived and published only from one complete coherent feedback
generation. Missing members invalidate the generation immediately; cross-bus
skew and partial paired command cycles latch a two-arm fault. J1--J7 use the output-shaft relationship
`q=a*qout+b`, `dq=a*dqout`, `tau=tauout/a`; the integrated DaMiao gearing is
validated but is never applied a second time. Every plan binds the complete
fresh measured sequence. Joint and TCP completion require new measured q/dq;
TCP completion additionally uses FK of the measured joints.

Motion uses a synchronized seventh-order smooth time law. Plans bind their
originating controller, verification epoch, collision scene, measured sequence,
and measured start pose. Single-joint plans hold the other six measured starting
coordinates. Paired TCP plans run bounded position IK for both arms using only
measured q as the initial seed, then solve 17 Cartesian waypoints seeded from
their predecessor. Intermediate residual, bounds, protocol span, branch jump,
and singularity-policy violations reject the complete paired plan. The model
has no collision engine, so the default `OA_COLLISION_REJECT_ALL` blocks every
plan. Tests may opt into `OA_COLLISION_VIRTUAL_UNCHECKED`; reports continue to
state `collision_checked == 0`.

Opaque handles are validated through typed registries without dereferencing
caller memory. Controller operations are internally serialized. Any destroy can
overlap an operation already in progress: it removes the handle from lookup,
synchronizes with pins, then frees the registry slot; already-pinned immutable
work safely retains shared state. Handles are monotonic, never-dereferenced
token values and are not reused. Later calls return `OA_EINVAL`; token-space
exhaustion fails closed with `OA_ENOMEM`.

V1 input records are append-compatible. The original `8bc839e` prefixes for
controller options, paired TCP moves, and simulator faults remain accepted.
Missing virtual-scene, branch/singularity, and fault-detail tails receive the
documented defaults exercised by the frozen-header compatibility executable.

While armed, `advance` must be called at the configured positive cycle. Missed
cycles latch timeout, equal timestamps generate neither feedback nor dwell, and
completion requires three full cycle intervals of measured in-tolerance state.
The execution request's controlled and disable stop policies produce distinct
simulator stop states while physical execution remains hard-gated. Only
complete, fresh, coherent, fault-free producer/cycle watchdog faults materialize
a measured zero-velocity enabled hold for controlled-stop requests. Missing or
skewed feedback, partial sends, and motor faults take priority and instead
materialize a zero-velocity two-arm disable fallback.

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

ThreadSanitizer is available with `-DOA_CONTROL_TSAN=ON`, mutually exclusive
with `OA_CONTROL_SANITIZERS`.
