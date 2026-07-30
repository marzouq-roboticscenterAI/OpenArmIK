# OpenArm runtime facade

Include `openarm_runtime_units.h` for the header-only binary64 paired-TCP unit
adapter. It preserves the frozen Runtime V1 header and archive symbol contract,
converts targets once to metres, and leaves tolerances and reports in metres.

Runtime ABI V1 freezes at the current reviewed header in
`tests/abi_v1/openarm_runtime.h`. Earlier Runtime feature headers before
`987f512` were unpublished pre-release drafts, not the V1 compatibility
baseline. The frozen record layouts, signatures (including
`oa_runtime_manifest_save`), and symbols are now the compatibility contract.
Runtime V1 also freezes the Commission 0.1.0 records exposed by its six public
calibration operations; the package requires that exact dependency and installs
the frozen Commission header beside the Runtime snapshot. A breaking Commission
record change therefore requires Runtime V2 names or an explicit V1 compatibility
adapter.
Future breaking changes require V2 names, a new ABI version, and an explicit
compatibility path; the obsolete pre-release persistence signature must not be
restored.

`openarm_runtime` is the stable ISO-C orchestration boundary over the model,
commissioning, and control modules. It provides deterministic virtual state,
calibration, and explicitly unchecked virtual motion. The legacy
`OA_RUNTIME_BACKEND_SOCKETCAN_QUERY` name remains for ABI compatibility, but
the runtime has no CAN-codec or transport dependency and cannot open SocketCAN
or transmit frames. It may enumerate local CAN link metadata read-only through
sysfs. Physical motor query, evidence, configuration preview/apply, calibration
actuation, enable, zero, save, motion, and collision-authorized motion are
always unsupported.

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
scene revision, and both arm/TCP artifacts. Joint requests bind the same model,
side TCP, coordinate digest, collision policy, and scene contract as paired
requests; plan reports expose the verified binding.

The facade clock is initialized from and remains comparable to the host steady
clock on every backend. The virtual controller uses a private exact-cadence
counter with the same initial epoch; this prevents host scheduling jitter from
violating the controller's hard cycle contract while facade expiry continues
to advance during planning. Feedback timestamps are captured in the facade
domain when each sequence is produced. A planning pause therefore makes old
feedback stale by real elapsed time, while resumed feedback receives a fresh
facade timestamp. Controller events are drained into the same facade domain.
Only one unexecuted plan may own that quiescence authority. A second plan
returns `OA_RUNTIME_EBUSY`.

The virtual inventory contains exactly two interfaces and fourteen motors.
`oa_runtime_inventory_query()` remains successful for that virtual backend.
On the legacy physical backend it clears `out_inventory` and immediately
returns `OA_RUNTIME_EUNSUPPORTED`, regardless of query-option contents. It does
not open a CAN socket, transmit, wait, or start work. Physical
`oa_runtime_configuration_preview_physical()` similarly returns an initialized
invalid, non-armable preview whose validation status is
`OA_RUNTIME_EUNSUPPORTED`; local link metadata is never promoted into motor,
fingerprint, mapping, or configuration evidence. The corresponding capability
bits are clear. Metadata enumeration inspects at most 256 local link entries
and returns at most `OA_RUNTIME_MAX_INTERFACES` CAN rows.

## Manifest format

Local authenticated files are not durable rollback authority. A static HMAC key
proves origin and integrity, but a directory snapshot can replay the file and
every authenticated sidecar together. The V2 persistence API therefore requires
a caller-owned `(revision, content_sha256)` checkpoint at authority open and at
every load, save, or recovery operation. The checkpoint must be durably stored
outside the manifest directory's replay domain and bound by the caller to the
application, key ID, canonical directory, and logical file name.

`oa_runtime_persistence_authority_open_v2` pins the directory by walking each
absolute path component from `/` with no symlink following. Revision zero with
an empty digest is accepted only to provision an absent stream. V2 load and
recovery require a nonzero checkpoint; a V2-authenticated file handle reports
both `authenticated == 1` and `checkpoint_authorized == 1` and may create a
virtual runtime. Plain load/preview report `authenticated == 0`. The legacy V1
HMAC load reports authentication but `checkpoint_authorized == 0`; it is a
replayable defense-in-depth preview artifact and cannot create an armable
runtime.

The bounded UTF-8 format is line-oriented and non-executable:

```text
OPENARM_RUNTIME_MANIFEST|1
summary|state|backend|manifest_revision|model_revision|inventory_revision|fingerprint
motor|side|joint|...fixed numeric and bounded text fields...
...exactly fourteen motor records in side/joint order...
sha256|lowercase SHA-256 of every preceding byte
hmac-sha256|bounded-key-id|HMAC-SHA-256 of the complete sha256-terminated payload
```

V2 writes end with `hmac-sha256-v2` and authenticate a domain-separated input
that includes the key ID and logical slot name. Copying or hard-linking an
authentic artifact under another public name therefore fails authentication.
Replaying older authentic bytes under the original name is rejected only
relative to the caller's external checkpoint.

Fields and record order are fixed; unknown, duplicate, missing, non-finite,
overlong, or malformed data is rejected. The unkeyed digest is deterministic
corruption detection; it is never described as authentication. Authenticated
load verifies the HMAC in constant time before publishing an accepted immutable
handle. V2 save is an exact-current compare-and-swap: under a fresh independently
opened transaction lock FD, the current revision and digest must equal the
supplied checkpoint. A numerically newer proposal cannot overwrite an
intervening commit. On success it retains the prior authenticated revision as
`<name>.previous`, writes and syncs an exclusive same-directory temporary,
installs the prior copy and syncs the directory, atomically renames current,
reloads and authenticates it, and performs the final directory sync. The output
checkpoint is populated only after that verified durable commit.

The caller's safe commit sequence is: call V2 save with its current external
checkpoint; on `OK`, durably compare-and-swap the external checkpoint to the
returned tuple; only then publish that tuple for future opens. A crash between
the filesystem commit and external CAS may expose a newer valid file, which a
later checked load can observe above the old floor. It never makes a file below
the external floor acceptable.

`OA_RUNTIME_EIO` means no namespace commit occurred, or the old state was
restored and the rollback directory sync succeeded. `OA_RUNTIME_EDURABILITY`
means a post-rename rollback could not be confirmed; no checkpoint is returned
and that authority is poisoned. All ordinary operations then return
`EDURABILITY` until explicit `oa_runtime_manifest_recover_v2` reconciles current
and `.previous` using the unchanged external checkpoint, or the authority is
destroyed and reopened. Recovery promotes `.previous` through a synced,
independent temporary inode so current and previous do not remain hard-link
aliases. Under the transaction lock, checked operations also remove and sync
reserved-prefix temporaries abandoned by a killed cooperating writer. Ordinary
load never falls back to `.previous`.

The lock coordinates cooperating library users only. Directory writers that
ignore it can deny service, and holders of the HMAC key can sign arbitrary
content. Durability requires a local filesystem/storage stack with documented
same-directory rename/hard-link atomicity and meaningful file and directory
`fsync`; NFS/SMB/FUSE-like filesystems are not qualified by this contract.

OpenArm handles and C++ runtime state are process-local. After the library has
been used, a forked child must call only async-signal-safe functions and `exec`;
it must not call OpenArm APIs or destroy inherited handles. A creator-PID guard
returns `OA_RUNTIME_ESTATE` before registry locking in ordinary accidental child
calls, but it is diagnostic rather than a general post-fork safety guarantee.
Independently exec'd processes open their own authorities and coordinate through
fresh per-transaction open-file-description locks.

Controller events expose the exact lower aggregate sequence separately. Since
the lower event does not contain atomic per-arm sequences or a measurement
timestamp, those fields remain zero and their validity flags are clear rather
than fabricating evidence.
