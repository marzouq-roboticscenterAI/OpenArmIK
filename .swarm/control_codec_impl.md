# DaMiao C codec implementation

Status: **DONE — codec-only, hardware-free**  
Branch: `impl/control-codec`

## Scope delivered

- Preserved `OA_CAN_ABI_VERSION == 2`, every pre-existing public record, and all
  old function signatures. Existing motor-family MIT/feedback behavior and its
  explicit saturation-result API remain source/ABI compatible.
- Added fixed-width, versioned C records and constants for the exact pinned
  DaMiao RID set, register wire types, operations, decoded values, queried MIT
  scaling profiles, profile-aware MIT, POS_VEL, and POS_FORCE commands.
- Added strict typed query, write, and response codecs. They correlate standard
  arbitration ID, echoed target send ID, operation (`0x33`/`0x55`), RID, DLC,
  and declared RID type. Unknown/gap RIDs, type mismatches, non-finite floats,
  invalid IDs, invalid flags/DLC, read-only writes, and documented range errors
  are rejected before output records are changed.
- Added golden frame builders for save-current-position-as-zero (`0xFE`), clear
  error (`0xFB`), and save RAM parameters to flash (`0xAA`). These functions
  construct frames only. They do not transmit or claim that an operation is safe.
- Added `oa_can_mit_profile_from_registers`. It accepts only correlated PMAX,
  VMAX, and TMAX query results for the same target/receive IDs, checks positive
  finite IEEE-754 binary32 values and raw/value consistency, then binds those
  IDs into a complete profile.
- Added profile-aware MIT encoding and feedback decoding. They use the queried
  mapping spans, reject incomplete/mismatched profiles, reject all command range
  violations without saturation, and preserve/validate the full status nibble,
  embedded motor ID, MOS temperature byte, and rotor/coil temperature byte.
- Added strict POS_VEL (`ESC_ID + 0x100`, two LE float32 values) and POS_FORCE
  (`ESC_ID + 0x300`, LE float32 position, speed x100, per-unit current x10000)
  encoders. The speed is non-negative and profile-bounded; POS_FORCE additionally
  enforces its 100 rad/s wire bound and `[0,1]` per-unit current range. Values are
  rejected, never silently clipped.
- Kept the in-memory diagnostics transport incapable of motion/configuration:
  tests prove it rejects profile-aware motion, register writes, set-zero,
  clear-error, and flash-save frames in addition to legacy MIT/enable frames.

## Evidence decisions

- RID values and U32/F32 typing follow the pinned `dm_motor_constants.hpp`,
  `parse_motor_param_data`, and current OpenArm register CLI metadata.
- Direction RID 55 remains read-only/F32 because pinned read and write tools
  contradict one another. No write is exposed until installed firmware resolves
  that contradiction.
- CAN bitrate writes conservatively accept only the metadata-documented codes
  0..9. The CLI's extra 10/11 labels are not treated as qualification evidence.
- PMAX/VMAX/TMAX are protocol mapping spans, not physical joint safety limits.
  The codec never labels them as continuous/peak/structural/collision limits.
- Temperature bytes are returned exactly as protocol degrees-C bytes. No
  unverified safe thermal threshold was invented.

## Verification

Strict Release build and tests:

```text
cmake -S can -B /tmp/openarmik-codec-build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build /tmp/openarmik-codec-build --parallel
ctest --test-dir /tmp/openarmik-codec-build --output-on-failure
1/1 Test #1: openarm_can_tests ... Passed
100% tests passed, 0 tests failed
```

ASan + UBSan build and tests:

```text
cmake -S can -B /tmp/openarmik-codec-sanitize \
  -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON \
  -DOA_CAN_ENABLE_SANITIZERS=ON
cmake --build /tmp/openarmik-codec-sanitize --parallel
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1 \
ctest --test-dir /tmp/openarmik-codec-sanitize --output-on-failure
1/1 Test #1: openarm_can_tests ... Passed
100% tests passed, 0 tests failed
```

Coverage includes command/register golden vectors, U32 and F32 responses,
dynamic MIT encode/decode quantization, status/temperature/embedded-ID checks,
all status nibbles, malformed IDs/flags/DLC/opcodes/RIDs/types, non-finite and
range failures, incomplete/wrong-motor profiles, undersized/wrong-version
records, output canaries, legacy randomized round trips, and fake-transport
motion rejection.

## Explicit non-claims

No SocketCAN motion transport was added, no interface was opened or configured,
no CAN frame was transmitted to hardware, and no physical arm was calibrated or
moved. Register writes, set-zero, clear-error, and flash-save remain dangerous
commissioning primitives that the controller/commissioning lifecycle must gate.
This work does not solve arm side, joint assignment, URDF sign/zero, physical
limits, collision checking, watchdog qualification, or E-stop integration.
