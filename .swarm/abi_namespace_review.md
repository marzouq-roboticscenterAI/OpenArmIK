# Independent ABI namespace review

Date: 2026-07-29 (America/Los_Angeles)

Commit reviewed: `fd01eaeabb91b960b83e8eb3107d7f38435e5665`
against parent `514e3a8b6ba21a687d18bd761d17c9412a50804c`

Verdict: **FINDINGS**

No Critical findings. Two Important findings and one Minor finding remain.

## Important findings

### I1. The default legacy surface remains ambiguous and changes with include order

Both headers use the shared `OPENARM_LEGACY_GENERIC_STATUS_DEFINED` guard
(`model/include/openarm_model.h:43-55`,
`control/include/openarm_control.h:52-73`). This makes the first header silently
own all generic names rather than actually giving those names one stable meaning:

- model then control: `oa_status` is signed `int32_t`, `OA_EINVAL` expands to
  `OA_MODEL_EINVAL`, `OA_ENONFINITE` exists, and the control legacy name
  `OA_EABI` does not exist;
- control then model: `oa_status` is unsigned `uint32_t`, `OA_EINVAL` expands to
  `OA_CONTROL_EINVAL`, `OA_EABI` exists, and the model legacy name
  `OA_ENONFINITE` does not exist.

Thus the pair now compiles by suppressing the second header's aliases, but the
visible typedef, macro ownership, signed arithmetic/conversion behavior, and
available legacy constants depend solely on include order. The two shared
values happen to be numerically equal today, which does not make the typedef or
the rest of the surface unambiguous. This directly contradicts the supplied
verification requirement at `.swarm/abi_collision_verify.md:137-140` to avoid
include-order-dependent selection because it can silently choose the wrong
module's status surface.

Exact evidence:

```text
# model -> control, from cc -E -dM
#define OA_EINVAL OA_MODEL_EINVAL
#define OA_OK OA_MODEL_OK
#define OA_ENONFINITE OA_MODEL_ENONFINITE

# control -> model, from cc -E -dM
#define OA_EINVAL OA_CONTROL_EINVAL
#define OA_OK OA_CONTROL_OK
#define OA_EABI OA_CONTROL_EABI
```

Strict C11 assertions also confirmed `(oa_status)-1 < 0` model-first and
`(oa_status)-1 > 0` control-first. The committed matrix only checks that the
translation units compile; it has no semantic assertions for legacy mode, so
it cannot catch this defect.

The compatibility-preserving design in the supplied verifier is preferable:
make module-prefixed names canonical, retain single-header legacy source
compatibility, and require a documented opt-out for a combined consumer. Do
not try to make the ambiguous legacy namespace appear composable by first-win
suppression.

### I2. The new installed-consumer test does not link an installed library, and control has no installed CMake export

The exact acceptance requirement says to "link a consumer against installed
packages only" (`.swarm/ros_integration_critic.md:126`). The added consumer
only finds an include directory and builds constant-only executables:

- `control/tests/install_consumer/CMakeLists.txt:4-24` has no `find_package`,
  `find_library`, or `target_link_libraries`;
- `control/tests/install_consumer/consumer.c:6-9` and its C++ counterpart call
  no API function, so neither library is needed by the linker;
- `control/CMakeLists.txt:53-57` installs `libopenarm_control.a` and the header
  but defines no `EXPORT`, config file, or imported target for control.

Consequently `openarm_public_headers_install_consumer` passing proves that two
copied headers compile, not that a C or C++ consumer can discover and link the
installed model/control packages with the public signatures and C linkage.
The installed prefix contains a valid `openarm_model` config/export, but no
control config/export; an explicit `openarm_control` package lookup failed.

A manual independent C11 and C++17 test did compile, link, and run when given
the installed archive paths explicitly (`-lopenarm_control -lopenarm_model
-lpthread -lm`), so this is an install/export and regression-test gap rather
than evidence of a broken ELF ABI.

## Minor finding

### M1. The advertised canonical-only macro cannot be applied project-wide

The public documentation tells combined consumers to define
`OPENARM_DISABLE_LEGACY_GENERIC_STATUS`, but the model implementation still
uses `oa_status` and `OA_*` throughout (for example
`model/src/openarm_model.c:79-116`), and the control implementation does the
same (for example `control/src/c_api.cpp:45-53`). A fresh build configured with
the macro in `CMAKE_C_FLAGS` and `CMAKE_CXX_FLAGS` fails immediately with
`unknown type name 'oa_status'` and undeclared `OA_EINVAL`/`OA_OK`.

The source-local defines in the committed consumers work, so the documented
narrow usage is possible. However, a public feature switch of this kind is
commonly supplied as a target or project compile definition; the libraries
should use their canonical module-prefixed status names internally so their
own build is not coupled to deprecated aliases. No committed test builds the
implementation in canonical-only mode.

## Checks that passed

- `git diff --check 514e3a8..fd01eae`: passed.
- Fresh GCC 15.2 strict syntax matrix: 100/100 passed. This covered each of the
  five intended public headers standalone plus all 20 ordered distinct pairs,
  in C11 and C++17, with legacy aliases enabled and disabled, using
  `-pedantic-errors -Wall -Wextra -Werror -fsyntax-only`.
- Fresh control Release configure/build: passed; CTest 4/4 passed, including
  the committed installed-header test.
- Fresh model Release configure/build: passed; CTest 3/3 passed, including the
  ABI-v1 canary and randomized/reference tests.
- Fresh control `RelWithDebInfo` with `OA_CONTROL_SANITIZERS=ON`: built cleanly;
  ASan/UBSan CTest 3/3 passed with leak detection and halt-on-error enabled.
- A fresh parent build and fresh target build produced byte-identical archives:
  model SHA-256
  `32e57096989b4481df85f4713451d1dc62e55e63ab505657f67ee05cf01c8aab`;
  control SHA-256
  `3ab99892c8044806172853443a67f03a4e4c977792caf498e95339a9d0c2dc97`.
- Sorted global defined-symbol lists were identical (model 16 lines, control
  139 lines), and parent-header C11 model/control consumers linked and ran
  against the target libraries.
- Independently installed canonical C11 and C++17 consumers in both model/control
  include orders compiled, explicitly linked both installed archives, and ran.
- Canonical status storage/value checks passed: model remains signed 32-bit,
  control remains unsigned 32-bit; `oa_ik_diagnostics.status` remains at offset
  8 and `oa_event.cause` at offset 40.
- No GUI, CAN interface, network, device, or hardware action was used.

## Commands and exact results

```sh
# Fresh Release
cmake -S control -B /tmp/openarmik-abi-review-control-release \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/openarmik-abi-review-control-release --parallel 4
ctest --test-dir /tmp/openarmik-abi-review-control-release --output-on-failure
# 4/4 passed

cmake -S model -B /tmp/openarmik-abi-review-model-release \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/openarmik-abi-review-model-release --parallel 4
ctest --test-dir /tmp/openarmik-abi-review-model-release --output-on-failure
# 3/3 passed

# Fresh sanitizer build
cmake -S control -B /tmp/openarmik-abi-review-control-sanitize \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DOA_CONTROL_SANITIZERS=ON
cmake --build /tmp/openarmik-abi-review-control-sanitize --parallel 4
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
ctest --test-dir /tmp/openarmik-abi-review-control-sanitize --output-on-failure
# 3/3 passed

# Binary comparison after independently building parent and target
sha256sum parent/libopenarm_model.a target/libopenarm_model.a
# both 32e57096989b4481df85f4713451d1dc62e55e63ab505657f67ee05cf01c8aab
sha256sum parent/libopenarm_control.a target/libopenarm_control.a
# both 3ab99892c8044806172853443a67f03a4e4c977792caf498e95339a9d0c2dc97
nm -g --defined-only parent/archive | sort
nm -g --defined-only target/archive | sort
diff -u parent.nm target.nm
# no differences

# Canonical-only project-wide probe
cmake -S model -B /tmp/openarmik-abi-review-model-canonical-global \
  -DOA_MODEL_BUILD_TESTS=OFF \
  '-DCMAKE_C_FLAGS=-DOPENARM_DISABLE_LEGACY_GENERIC_STATUS=1'
cmake --build /tmp/openarmik-abi-review-model-canonical-global --parallel 4
# failed: unknown type name 'oa_status'; OA_EINVAL/OA_OK undeclared
```

Compiler portability note: GCC 15.2 was the only C/C++ compiler installed;
Clang was unavailable. ISO C11/C++17 language checks were therefore repeated
with strict GCC diagnostics, but a second compiler could not be independently
exercised on this host.
