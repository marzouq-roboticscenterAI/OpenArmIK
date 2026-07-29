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

Capability bits describe facade operations only. The runtime does not advertise
standalone FK or IK: measured virtual coordinates and motion planning are the
operations it actually exposes. `oa_runtime_get_model_identity` binds every
coordinate to the exact side-specific model ID, model-data/source/flattened-URDF
digests, named TCP frame and revision. Coordinate requests additionally carry a
combined identity digest covering frame, units, orientation, collision policy,
scene revision, and both arm/TCP artifacts.

The facade clock is initialized from and remains comparable to the host steady
clock on every backend. The virtual controller uses a private exact-cadence
counter with the same initial epoch; this prevents host scheduling jitter from
violating the controller's hard cycle contract while facade expiry continues
to advance during planning. Only one unexecuted plan may own that quiescence
authority. A second plan returns `OA_RUNTIME_EBUSY`.

The virtual inventory contains exactly two interfaces and fourteen motors.
Physical query evidence never assigns a side or joint: it reports
`unresolved_assignment`, presence, ambiguity, timestamps, and the values that
were actually correlated. Interface absence yields an immutable empty result.

## Manifest format

Manifest persistence requires an opaque directory-scoped authority created
under the process's operating-system credentials. The authority contains a
caller-supplied, nonzero 256-bit HMAC key and bounded key ID; save/load accept
only a single non-traversing file name relative to the already-open,
non-symlink directory. Plain absolute-path load and preview remain available,
but their returned summaries explicitly report `authenticated == 0`.

The bounded UTF-8 format is line-oriented and non-executable:

```text
OPENARM_RUNTIME_MANIFEST|1
summary|state|backend|manifest_revision|model_revision|inventory_revision|fingerprint
motor|side|joint|...fixed numeric and bounded text fields...
...exactly fourteen motor records in side/joint order...
sha256|lowercase SHA-256 of every preceding byte
hmac-sha256|bounded-key-id|HMAC-SHA-256 of the complete sha256-terminated payload
```

Fields and record order are fixed; unknown, duplicate, missing, non-finite,
overlong, or malformed data is rejected. The unkeyed digest is deterministic
corruption detection; it is never described as authentication. Authenticated
load verifies the HMAC in constant time before publishing an accepted immutable
handle. Saves reject revision rollback and same-revision equivocation, retain
the prior authenticated revision as `<name>.previous`, use a same-directory
exclusive temporary file, file `fsync`, atomic rename, reload/HMAC verification,
and directory `fsync`. Pre-commit failures leave the target unchanged. A final
directory-sync failure triggers an atomic rollback to the prior artifact (or
removal when no prior artifact existed) and a second directory sync before
returning `OA_RUNTIME_EIO`. The distinct `OA_RUNTIME_EDURABILITY` result is
reserved for the narrower case where that post-commit rollback cannot itself be
confirmed, so target state and durability must then be treated as unknown.

Controller events expose the exact lower aggregate sequence separately. Since
the lower event does not contain atomic per-arm sequences or a measurement
timestamp, those fields remain zero and their validity flags are clear rather
than fabricating evidence.
