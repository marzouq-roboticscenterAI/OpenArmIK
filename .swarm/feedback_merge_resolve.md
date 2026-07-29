# Simulator feedback / ABI merge resolution

Status: **DONE**

## Resolution

The paused simulator-feedback cherry-pick series was completed on top of the
public ABI namespace work. Conflict resolution retained the delayed/coherent
feedback, lifecycle-retirement, and event-overflow semantics and adversarial
tests while translating the pre-namespace `oa_status` / `OA_*` status spellings
to `oa_control_status` / `OA_CONTROL_*`. The strict public-header matrix,
installed-package consumers, current C11 ABI consumer, and frozen original V1
ABI consumer remain present.

Resulting commits, in requested order:

- `597df342fc6875e0fee65dee83ccb9a115391348` — delay coherent simulator feedback
- `8bcf6968d8f25481a4be6b2995ac2d4eacf7c4a3` — retire feedback on stop transitions
- `c4c2a6c3bc378511a374072910d3bef929c6eee3` — terminate on event overflow
- `3bc2184a2332ac38d86c4086e0fddb6b8fe16d17` — record clean simulator feedback review

## Fresh verification

- Release, `/tmp/openarmik-merge-release.UsLzl7`: build passed with the exhaustive
  warnings-as-errors C11/C++17 header matrix; CTest **4/4 passed**, including
  control, current C11 ABI, frozen original V1 ABI, and installed C11/C++17
  consumers.
- ASan/UBSan RelWithDebInfo, `/tmp/openarmik-merge-asan.Pb7nvB`: CTest **3/3
  passed** with leak detection and halt-on-error enabled; no sanitizer report.
- TSan RelWithDebInfo, `/tmp/openarmik-merge-tsan.vKULbt`: CTest **3/3 passed**
  with halt-on-error enabled; no race report.
- Both Release ABI executables passed again directly, and the installed consumer
  passed again directly via CTest **1/1**.
- `git diff --check 29657f7..HEAD` and the working-tree `git diff --check` passed.

No GUI, CAN, network, or hardware command was run. No residual merge marker,
known test failure, or unresolved risk remains.
