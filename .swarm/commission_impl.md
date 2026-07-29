# Commissioning/calibration implementation

Date: 2026-07-29
Branch: `impl/commission`

## Delivered scope

- Added standalone `commission/` CMake product. The runtime is C++17 with modular
  `ManualCalibrationSession` and `RecipeCalibrationSession` objects behind the
  versioned ISO-C `openarm_commission.h` ABI.
- The product has no Python, ROS, shell, sudo, SocketCAN, physical transport, or
  motor-frame dependency. It only consumes caller-owned encoder/interlock
  records and returns reports or bounded abstract next-action requests.
- Manual known-pose calibration requires fresh, strictly sequenced,
  torque-disabled, fault-free output-encoder samples. It enforces dwell,
  position spread, and velocity stability. One reference requires a known sign;
  otherwise two distinct known references determine and validate the sign.
  Candidate mappings are exactly `q_model = a*q_output + b`, with `a` restricted
  to `-1` or `+1`; no motor gearing is reapplied.
- Manual candidates pass through explicit candidate/review/commit states. Commit
  constructs a complete caller-visible revision patch locally and copies it only
  after every check succeeds. Abort discards the candidate.
- Added the supervised recipe states `PRECHECK`, `WAIT`, `APPROACH`,
  `CONTACT_DWELL`, `RETREAT`, `REAPPROACH`, `REPEATABILITY`, `CANDIDATE`,
  `REVIEW`, `COMMIT`, and latched `ABORT`.
- Recipe actions carry expiry, direction, target, maximum travel, speed, torque,
  and temperature. The session enforces fresh feedback, explicit E-stop and
  deadman state, prior directional travel, low measured velocity plus measured
  torque evidence, approach/retreat deadlines, travel/speed/torque/temperature/
  contact-energy ceilings, a retreat/reapproach cycle, repeatability, and
  explicit review.
- Arm-joint recipes default to `OA_COMMISSION_EUNSUPPORTED`; creation requires an
  explicit nonzero hardware-qualification revision and record. Gripper recipes
  may run simulation-only but still pass all sample, interlock, ceiling,
  repeatability, review, and revision gates.
- There is deliberately no frame type, byte payload, transport callback,
  enable/zero/save/register operation, or flash API. A source scan for the
  DaMiao zero opcode/payload and transport/system-call surfaces returned no
  matches. Every failure latches abort, returns only `ABORT_DISABLE`, and makes
  commit return `OA_COMMISSION_ESTATE` without touching the caller's patch.

## Verification evidence

Fresh strict build and sanitizer tests:

```text
cmake -S commission -B /tmp/openarmik-commission-build \
  -DCMAKE_BUILD_TYPE=Debug -DOA_COMMISSION_ENABLE_SANITIZERS=ON
cmake --build /tmp/openarmik-commission-build --clean-first --parallel
ctest --test-dir /tmp/openarmik-commission-build --output-on-failure
100% tests passed, 0 tests failed out of 2
```

The C++ suite covers manual one/two-reference calibration, unknown-sign and
freshness/interlock/stability failures, the complete gripper simulation recipe,
unqualified arm-recipe rejection, time/direction/speed/torque/energy/temperature/
contact/repeatability/review failures, false-contact rejection before minimum
travel, commit immutability after failure, explicit abort from every nonterminal
state, and injected drive fault from every nonterminal state. The second test is
a separately compiled C11 consumer of the public ABI.

Additional checks passed:

- GCC 15 `-Wall -Wextra -Wpedantic -Werror`.
- AddressSanitizer and UndefinedBehaviorSanitizer.
- `cppcheck --enable=warning,performance,portability --error-exitcode=1`.
- Clean install to `/tmp/openarmik-commission-install`, including exported
  `OpenArm::Commission` CMake package target.
- `git diff --check`.

## Safety boundary

This increment cannot move hardware and does not claim hardware calibration.
The returned action records are requests for a separately reviewed controller;
they are not authorization to energize a drive. Simulation does not qualify an
arm-joint recipe or physical installation. Physical qualification records,
fixtures, installed E-stop/deadman behavior, safe limits, and supervised staged
acceptance remain mandatory external gates.

## Blockers

None for this transport-free calibration increment.
