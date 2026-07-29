# Final independent review: model core through `0422d4b`

## Verdict

**CLEAN.** No Critical, Important, or Minor findings remain in the reviewed model-core scope. All findings from the reviews of `5bbdcf0` and `6eeccbb` are fixed and independently reproduced below.

No ROS, CAN, or hardware interface was accessed. The implementation worktree was not modified.

## ABI and failure-output evidence

- `OA_MODEL_ABI_VERSION` is now 2. The new `oa_ik_position_v2` takes explicit output version and capacity and checks both before the first output write (`model/src/openarm_model.c:295-309`).
- The published ABI-v1 `oa_ik_position` symbol remains callable but returns `OA_EINVAL` without dereferencing or writing any argument (`model/src/openarm_model.c:286-293`).
- The repository's separately compiled exact ABI-v1 248-byte diagnostics canary passed under Release and ASan/UBSan.
- An independent guarded ABI-v1 call preserved the entire old result plus 32-byte canary.
- Independent ABI-v2 guards tested output versions 0, 1, and 3 and capacities 0, 248, and 255: all returned `OA_EINVAL` and preserved every byte. A valid 256-byte result did not touch its trailing canary, including when options ABI/version or size was invalid.
- Current public sizes on x86_64 are options 224 bytes, ABI-v1 diagnostics 248 bytes, and ABI-v2 diagnostics 256 bytes. Public status/flag fields are fixed-width.
- Valid ABI-v2 failures initialize finite diagnostics with matching status. Unsafe finite ranges and NaN/Inf inputs remain fail-closed.

## Canonical generation and provenance evidence

- The generator requires exact HEAD `6c7b720f1ba48e8bafa3a3dc752c45f397b42221`, rejects a dirty checkout before flattening, and compares the complete 59-file v1/parallel-gripper source set to a known canonical hash (`model/tools/generate_model.py:140-150`).
- Clean regeneration passed byte-for-byte for both the generated C include and archived flattened URDF.
- Supplied tests independently clone and reject dirty tracked and untracked inputs. An additional direct test against the prior modified J1 source copy was rejected.
- Independently recomputed identities match the embedded provenance:

  - canonical source set: `3f48ffec1598bebca34f90419521d5e320787746b66bf54937c3faeb7c6cb5fc`;
  - flattened URDF: `dd4b5d5fa0c050b78922bc95850ea65bb15721cb259030cccf106a953f171c55`;
  - xacro 2.1.1 implementation: `a22ff2294dd9ee54cd3c89801b62e4edd05220d560d8177d27d5b6c35c862b24`;
  - generator 3.0.0 source: `d6f773fbef6d4f5388e0ed1838935719e86019a43e281d73394ccf4b85f689ed`;
  - bound generation/model-data manifest: `9844be1eff37a801b4c48372bc35d8e96c4b872bb9d45c77a9787c4b50774354`.
- The archived current xacro contains `link7 -> hand` at 0.1025 m and `hand -> hand_tcp` at 0.0835 m, totaling 0.186 m. Side-specific base/J2 transforms, effective J1/J2 limits, and mirrored J7 axes remain canonical.
- Apache-2.0 license text, Enactic attribution, modified-data notice, and upstream notices are packaged.

## Kinematics and IK evidence

- FK multiplication/direction, body-frame world axes, and all 6x7 geometric-Jacobian rows matched the archived canonical URDF reference over 600 random poses.
- Independent central differences over 100 additional random legal poses per side and all columns had maximum linear error `1.81e-9` and angular error `2.85e-9`.
- The DLS translational matrix is completely initialized before any Gram product; 50 repeated unreachable solves remained byte-for-byte deterministic.
- Candidate steps are projected into the effective box, active outward faces are recomputed, and `OA_OK` revalidates both tolerance and feasibility.
- A fresh independent 20,000-case remote-seed sweep produced:

  - left: 8,787 successes and 1,213 explicit failures;
  - right: 8,592 successes and 1,408 explicit failures;
  - zero non-finite diagnostics, bound escapes, status mismatches, or false successes.
- The exact former J4 escape case remained feasible and returned explicit bound stagnation rather than false success.
- Primary squared position error is the sole line-search acceptance merit. Posture remains secondary; approximate Cartesian results are never promoted to success.
- Exact limit, midpoint, singular, unreachable, non-finite, invalid-option, line-search, active-bound, and iteration-budget paths remain finite, deterministic, and correctly classified.

## Build, sanitizer, and coverage evidence

- Clean strict Release build used explicit ISO C11 (`-std=c11`, extensions off), `-Wall -Wextra -Wpedantic -Werror`, and additional conversion, sign, shadow, format, undef, and common-symbol warnings.
- Configured Release CTest: 4/4 passed (core/status/bounds, ABI-v1 canary, 600-case URDF reference, canonical generator/dirty-source tests).
- ASan+UBSan Debug build with the complete configured suite: 4/4 passed with no sanitizer report.
- Coverage suite: 3/3 passed; `openarm_model.c` reached 99.65% of 288 lines, 99.35% of 308 branches executed, 79.87% of branches taken at least once, and 97.78% of calls executed.
- Default builds no longer depend on a developer-specific source path; optional regeneration is disabled with an explicit configuration message when its pinned inputs are absent.
