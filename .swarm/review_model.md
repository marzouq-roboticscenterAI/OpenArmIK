# Independent review: model core `5bbdcf0`

## Verdict

**CHANGES_REQUIRED.** The checked-in bimanual constants and the FK/geometric-Jacobian implementation agree with the current canonical `openarm_description@6c7b720f1ba48e8bafa3a3dc752c45f397b42221`, including the 0.186 m `link7 -> hand -> hand_tcp` chain, side-specific J2 frames, effective J1/J2 limits, and mirrored J7 axis. However, the position IK is nondeterministic and can return `OA_OK` with an out-of-bounds joint. The generator also mixes a stale example URDF with current xacro/config fragments and does not hash or parse the actual arm sources it claims to authenticate.

No ROS, CAN, or hardware interface was accessed. The implementation worktree was not modified.

## Critical

### 1. The DLS matrix is built from uninitialized Jacobian rows

File: `model/src/openarm_model.c:128`

The loop assigns `j[r][i]` and immediately evaluates `j[c][i]` for every `c`. While `r == 0`, rows 1 and 2 of the automatic `j` array have not been initialized. Thus `A = J W^-1 J^T` depends on stack contents. ASan/UBSan do not diagnose uninitialized reads, which is why the supplied sanitizer test remains green.

This is observably nondeterministic: three identical calls for target `[10,10,10]`, zero seed/options, and seven iterations returned errors `16.3696663546`, `16.3677919823`, and `16.3677758133` with three different joint vectors. A temporary review-only build that first populated all of `j` made repeated results bit-for-bit stable. Fill the complete 3x7 matrix before forming any cross-row product and add a repeated-call determinism regression test (or run MemorySanitizer/Valgrind in CI).

### 2. The bounded solver can leave the feasible box and still report success

Files: `model/src/openarm_model.c:136-142`

The step-to-bound calculation ignores a negative `bound`. If roundoff has placed a joint infinitesimally beyond a lower/upper face, an outward step produces a negative ratio, is ignored, and can move the joint grossly farther outside. Candidates are never projected or explicitly revalidated, and final success checks only Cartesian residual. The active mask is also persistent and is only updated in the `alpha < 1e-10` path.

To isolate this independently of Critical 1, a temporary review-only build changed only the Jacobian initialization order. In 3,000 deterministic random reachable left-arm cases, 14 returned joint vectors outside the box; one returned `OA_OK`, residual `1.7553420866e-9`, but J4 was `-0.0694980951` rad although its canonical lower limit is `0`. Its target was `[0.2708042689, 0.3246735719, 0.3571402434]` from seed `[-0.0437097440, -1.4722387129, -1.5562786197, 0.2972001284, 0.3836473821, -0.3155313167, 1.5662590524]`.

Every accepted iterate must be explicitly feasible, joints at/outside a face with an outward step must enter the active set, and `OA_OK` must recheck every effective bound including the configured margin.

## Important

### 3. Generation does not derive from or authenticate the claimed canonical xacro

Files: `model/tools/generate_model.py:2-6,15-16,33-65`; `model/README.md:3-5`; `model/tests/test_generator.py:7-14`

The generator parses arm origins, axes, and limits from `urdf/example/v1.urdf`, whose `hand_tcp` is the known stale zero-offset, then hard-codes `0.1025 + 0.0835` after substring checks in two newer files. It does not flatten the pinned entry xacro, parse the current `hand_joint`/`hand_tcp_joint` chain, validate the example against current arm xacro/YAML, or verify that the supplied checkout is actually commit `6c7b720f`; `PIN` is only emitted as text.

The `source_sha256` omits all sources that define the arm constants: `openarm_arm.xacro`, `openarm_macro.xacro`, `kinematics.yaml`, `kinematics_offset.yaml`, and `joint_limits.yaml`. As a proof, changing canonical arm `kinematics.yaml` J1 z from `0.0625` to `9.0625` in a temporary copy produced a byte-identical generated include and identical advertised hashes. The test merely reproduces the same extraction and therefore cannot detect this.

Generate/parse the exact flattened bimanual xacro, archive/hash that artifact and the full transitive input set, verify the Git commit, and golden-assert every side-specific name/origin/axis/limit and both fixed TCP joints. Do not label manually spliced data as an exact named URDF frame.

### 4. Posture is not a strict secondary task and line search is not always merit decreasing

File: `model/src/openarm_model.c:109-113,133-139`

The line-search merit combines Cartesian error and posture cost, so a posture improvement can reject a primary-task improvement. The damped projector in line 134 is not an exact null-space projector, allowing posture motion to alter the primary task. Further, line 138 accepts a candidate unconditionally once `alpha < 1e-5`, even if its merit did not decrease, contradicting `model/README.md:11`.

This does not silently return an approximate Cartesian result as success, but it can turn a reachable target into budget exhaustion and does not implement the task-priority contract required by the design reports. Use a hierarchical/constrained acceptance rule: primary error first, posture only within its null space/tolerance, and never accept a non-decreasing infeasible fallback.

### 5. Failure diagnostics and extreme finite inputs can contain contradictory/non-finite results

File: `model/src/openarm_model.c:118-123,138,141-142`

After zeroing `out`, non-finite or invalid options return `OA_ENONFINITE`/`OA_EINVAL` without assigning `out->status`, leaving it as `OA_OK`; ABI/size failures return before even initializing the result. Independently observed examples were return `OA_ENONFINITE` with `diag.status == OA_OK` for a NaN target, and return `OA_EINVAL` with `diag.status == OA_OK` for zero iterations or zero posture weight.

The candidate and final `oa_fk` return values are ignored. Large but finite inputs can overflow internal squares/divisions: a `DBL_MAX` finite target or minimum positive posture weight returned `OA_ENONFINITE` with NaNs in every reported `q` and infinity in the error. Validate numerically safe ranges, propagate every FK/linear-solver failure, and define/produce a finite initialized diagnostic result for every return path.

### 6. The advertised pure C11 strict build fails

Files: `model/CMakeLists.txt:9-12`; `model/src/openarm_model.c:84,97,129-132`

`cc -std=c11 -Wall -Wextra -Wpedantic -Werror` rejects the calls passing non-const multidimensional arrays to the const-qualified helper parameters: GCC reports “invalid use of pointers to arrays with different qualifiers in ISO C before C23.” The CMake build passed only because C extensions remain enabled and GCC 15's default language mode is newer; `c_std_11` does not force `-std=c11` when the compiler default already satisfies it.

Set `C_EXTENSIONS OFF`, test an explicit strict C11 mode, and make the helper signatures/calls valid in C11.

### 7. Apache-2.0 redistribution obligations are not packaged

Files: `model/src/generated/oa_model_data.inc:1-2`; `model/README.md:15`

The generated derivative data carries provenance text, but commit `5bbdcf0` contains no `LICENSE`/`LICENSE.txt`; the README tells recipients to “see the upstream `LICENSE.txt`,” which is not in the committed tree. The reviewed design explicitly requires shipping the license and preserving attribution. Add the Apache-2.0 license text and applicable Enactic copyright/attribution (and mark modifications as appropriate) before distribution. No upstream NOTICE file was found.

## Minor

### 8. The ABI marker does not make all public result layout fixed-width or evolvable

File: `model/include/openarm_model.h:16,24-33,49-73`

On this x86_64 build, `oa_ik_options` is 224 bytes and `oa_ik_diagnostics` is 248 bytes, and both C and C++ consumers compile. However, `oa_status` has implementation-defined enum representation, `collision_checked` is an `int`, and the output structure has no `abi_version`/`struct_size`. The input size check requires the library's entire current structure, rather than documenting a required prefix. Use fixed-width public fields and an explicit result size/version policy if this is intended as a stable binary ABI.

### 9. Several status/diagnostic semantics are dead or misleading

Files: `model/include/openarm_model.h:24-33,63-73`; `model/src/openarm_model.c:115-142`

`OA_EBOUNDS` is never returned because out-of-range seeds are silently clamped. `OA_ENOCONVERGENCE` is effectively converted to `OA_EBUDGET` on loop exhaustion. `min_singular_value` remains zero on any immediate-success call, even when the pose is nonsingular, and it is undocumented whether this is the full, translational, weighted, or active-set Jacobian. Define each status/field precisely and test every reachable outcome.

### 10. The generator test has a machine-specific default dependency

File: `model/CMakeLists.txt:7,29-34`

The default description root is an absolute developer path outside the committed tree. A clean clone elsewhere configures but its determinism test cannot run. Accept an explicit required path, use a pinned vendored/submodule/lockfile location, or skip the regeneration test with a clear status when the canonical source is unavailable.

## Verification evidence

- Canonical checkout: exact Git HEAD `6c7b720f1ba48e8bafa3a3dc752c45f397b42221`, clean before the temporary copied-source mutation test.
- Independently derived current bimanual constants match the include: left/right bases `(0, +/-0.031, 0.698)` with RPY `-/+1.5708`; J2 RPY `-/+1.57079632679`; left/right J7 axes `(0,-1,0)` / `(0,1,0)`; left J1/J2 limits `[-3.490659,1.396263]` / `[-3.316125326795,0.174532673205]`; right J1/J2 `[-1.396263,3.490659]` / `[-0.174532673205,3.316125326795]`; current hand and TCP translations sum to exactly 0.186 m.
- Independent NumPy reference over 200 random legal poses per side: maximum full-transform error `4.44e-16`, world-axis error `3.33e-16`, linear Jacobian error `4.44e-16`, angular Jacobian error `3.33e-16`.
- Independent central differences over 100 random legal poses per side and every column: maximum linear error `2.25e-9`, angular error `3.05e-9`.
- Supplied clean strict-warning Release build and CTest: 2/2 passed. Separate explicit strict ISO C11 compilation failed as described above.
- Clean ASan+UBSan Debug build and CTest: 2/2 passed; no sanitizer report.
- Gcov: 100% of 105 source lines executed, 100% of 240 branches executed, but only 81.25% of branches taken at least once. This did not expose the semantic defects above.
- Supplied nearby-seed IK random tests passed. Independent 1,000-case-per-side nearby-seed tests passed; remote-seed tests converged only 592/1000 left and 581/1000 right, with failures correctly non-success but affected by Critical 1. Reachable boundary/singular/remote-seed cases produced budget/stagnation outcomes; unreachable targets remained non-success. No approximate residual was labeled success except the explicitly documented out-of-bounds success, whose Cartesian residual met tolerance.
