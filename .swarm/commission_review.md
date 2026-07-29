# Independent review: commissioning/calibration (`ef02cef`)

Verdict: **CHANGES REQUIRED**

## Ranked findings

### 1. Critical — short C ABI records can cause out-of-bounds reads

`oa_commission_recipe_create()` correctly rejects a short record in
`valid_recipe()`, but its error-classification path then reads `recipe_kind`,
`hardware_qualified`, `qualification_revision`, and `qualification_record`
without first establishing that those fields exist
(`commission/src/c_api.cpp:231-238`). An independently compiled ASan repro using
an exactly 8-byte `{struct_size, abi_version}` allocation reports a heap-buffer-
overflow at `c_api.cpp:234`. A short top-level recipe-step input has a related
path: validation fails, then abort-action construction reads `input.now_ns`
(`calibration_session.cpp:263-268,389-392,371-373`). This violates the versioned
C record boundary and means malformed/older callers are not contained.

### 2. Critical — the hard-stop retry is not bounded by the established safe envelope, and contact dwell can outlive its deadline

The first approach is bounded from its initial position, but reapproach resets
`phase_start_q_` after retreat and grants another full
`maximum_approach_travel_rad` (`calibration_session.cpp:508-512,525-529,362-369`).
For the test recipe, after an initial stop near `0.111` and retreat to `0.061`, a
fresh enabled sample at `0.250` is accepted in `REAPPROACH`, although the original
approach corridor ended at `0.200`. A missed second contact can therefore drive
past both the first contact and the originally qualified envelope.

Additionally, `CONTACT_DWELL` never enforces the approach deadline
(`calibration_session.cpp:458-485`). The returned action retains a deadline, but
the state machine accepts contact after it: a repro with an approach deadline of
`211 ns` accepted a dwell sample at `301 ns` and transitioned to `RETREAT`.
Action metadata is advisory unless the session enforces it, so the claimed
bounded transition is not fail-closed.

### 3. Critical — an interlock failure does not invalidate a manual disabled/stable dwell

Manual sample validation errors are returned without clearing the accumulator or
latching `OA_MANUAL_ABORTED` (`calibration_session.cpp:104-142`). Repro: accept a
disabled sample at `t=100`, reject an enabled-drive sample at `t=105`, then accept
a disabled sample at `t=110`; with two samples and a 10 ns dwell, the session
immediately produces a candidate, enters review, and commits successfully. Thus
the measured dwell was not continuously torque-disabled, and a session can
commit after a physical interlock failure. This contradicts the required fresh-
disabled dwell and no-commit-after-failure behavior.

### 4. Important — repeatability uses only one second-contact sample

`minimum_contact_samples` and `contact_dwell_ns` are applied only to the first
contact. In `REAPPROACH`, one low-velocity/high-torque sample becomes
`second_stop_q_` and immediately advances to `REPEATABILITY`
(`calibration_session.cpp:538-543`). A transient or noisy torque sample can
therefore be treated as the repeated stop and produce a committable mapping. The
design requires consecutive contact evidence, retreat/reapproach, and a genuine
repeatability check; the second observation needs the same bounded dwell/evidence
quality or an explicitly qualified equivalent.

### 5. Important — a hot WAIT sample can authorize approach before the ceiling is checked

Temperature is checked in `PRECHECK` and active motion, but not in `WAIT` before
issuing `OA_RECIPE_ACTION_APPROACH` (`calibration_session.cpp:410-425`). A repro
entered `WAIT` at 25 C, then supplied a fresh 100 C disabled sample with operator
ready; the call returned success and an APPROACH action despite a 60 C ceiling.
The next call would abort, but the unsafe action has already crossed the ABI.
Active approach/retreat states also do not require `drive_enabled != 0`, allowing
an unexpected disable/re-enable cycle to continue without renewed readiness.

### 6. Important — arm qualification is not fully preserved/bound by the recipe API

The arm recipe contains no other-joint posture values or feedback, so it cannot
enforce the mandatory fixture posture before issuing actions; `fixture_record`
is only an unchecked text label. The API also accepts the contradictory
combination `OA_RECIPE_ARM_JOINT + hardware_qualified=1 + simulation_only=1`.
Commit then selects `fixture_record` instead of `qualification_record`
(`calibration_session.cpp:622-625`), so the emitted patch no longer carries the
record that admitted the arm recipe. More generally, a simulation-only gripper
patch has no flag distinguishing it from hardware calibration. This weakens the
"simulation never qualifies hardware" and installed-recipe binding at the
handoff boundary.

### 7. Important — opaque-handle lifetime and advertised ABI robustness are untested/uncontained

Every non-null opaque pointer is dereferenced or deleted directly; stale,
double-destroyed, or arbitrary handles are not detected. The design acceptance
explicitly calls for invalid-handle and lifetime coverage, but the tests cover
only null abort handles. Canary coverage is limited to failed commit output
immutability; there are no guarded short-record allocations, surrounding
canaries, invalid/stale handles, or allocation/exception injection tests. The
exception guards do contain constructor allocation failures and internal C++
exceptions for normal valid handles, but they cannot contain faults caused by
invalid pointers.

## Verified clean aspects

- Manual calibration math is output-shaft affine mapping only:
  `q_model = a*q_output + b`, with `a` exactly `+1/-1`; one reference requires a
  known sign and two references infer/check sign and unit scale. No gear ratio is
  reapplied.
- Recipe faults and explicit aborts that reach the OOP state machine latch
  `OA_RECIPE_ABORT`; recipe commit then returns `OA_COMMISSION_ESTATE` without
  modifying the caller's patch.
- Arm recipes without the required nonzero qualification fields are rejected.
- Source inspection found no motor frame, `FE` zero, save/flash/register write,
  SocketCAN, netlink, shell, or transport path in `commission/`.
- Output commits are assembled locally and copied only on success.

## Verification evidence

- Fresh GCC 15 Debug build with `-Wall -Wextra -Wpedantic -Werror`, ASan, and
  UBSan: build succeeded; both C++ tests and the separately compiled strict C11
  consumer passed (`2/2`).
- `cppcheck --enable=warning,performance,portability --error-exitcode=1`: clean.
- `git diff ef02cef^ ef02cef --check`: clean.
- Independent ASan short-record repro: deterministic heap-buffer-overflow at
  `commission/src/c_api.cpp:234`.
- Independent state-machine repros confirmed findings 2, 3, and 5.

No source edits were made; this report is the only repository change from the
review.
