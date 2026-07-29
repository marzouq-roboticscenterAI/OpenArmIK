# OpenArm runtime facade

`openarm_runtime` is the stable ISO-C orchestration boundary over the CAN,
model, commissioning, transport, and control modules. It provides deterministic
virtual state, calibration, and explicitly unchecked virtual motion. Its real
SocketCAN backend is read-only: it may enumerate local CAN links and issue only
typed register queries through `openarm_transport`. Physical configuration,
calibration actuation, enable, zero, save, motion, and collision-authorized
motion are always unsupported.

Runtime V1 uses SI units. Model joints and output-shaft feedback are radians,
radians/second, and Newton-metre estimates. XYZ is metres in
`openarm_body_link0`. Transforms are row-major parent-to-child matrices for
column vectors. IK orientation is free. All runtime deadlines and measurements
use `OA_RUNTIME_CLOCK_MONOTONIC`; foreign clock IDs are rejected.

The virtual inventory contains exactly two interfaces and fourteen motors.
Physical query evidence never assigns a side or joint: it reports
`unresolved_assignment`, presence, ambiguity, timestamps, and the values that
were actually correlated. Interface absence yields an immutable empty result.

## Manifest format

Manifest persistence is opt-in through `OA_RUNTIME_PERSISTENCE_AUTHORIZED` and
accepts only absolute, non-traversing, non-symlink paths. The bounded UTF-8
format is line-oriented and non-executable:

```text
OPENARM_RUNTIME_MANIFEST|1
summary|state|backend|manifest_revision|model_revision|inventory_revision|fingerprint
motor|side|joint|...fixed numeric and bounded text fields...
...exactly fourteen motor records in side/joint order...
sha256|lowercase SHA-256 of every preceding byte
```

Fields and record order are fixed; unknown, duplicate, missing, non-finite,
overlong, or malformed data is rejected. Writes use a same-directory exclusive
temporary file, file `fsync`, atomic rename, and directory `fsync`. Loading
verifies the digest before publishing an immutable typed handle.
