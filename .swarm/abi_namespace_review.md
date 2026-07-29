# Independent ABI namespace re-review

Date: 2026-07-29 (America/Los_Angeles)

Series reviewed: `514e3a8b6ba21a687d18bd761d17c9412a50804c..5059a3a139d4f8e6384949b735ffc520c6a4e7d3`

Verdict: **CLEAN**

No Critical, Important, or Minor findings remain. The three findings from the
review of `fd01eae` are resolved, and the broader canonical-name rewrite did
not change the rebuilt model or control archives.

## Prior finding resolution

### I1 — include-order-dependent legacy aliases: resolved

The shared first-header-wins guard is gone. Each header now exposes generic
aliases only for a single-module legacy translation unit. A combined consumer
must define `OPENARM_DISABLE_LEGACY_GENERIC_STATUS` before both model and
control headers and use `oa_model_status`/`OA_MODEL_*` plus
`oa_control_status`/`OA_CONTROL_*`.

Direct strict probes established deterministic behavior in both C11 and
C++17:

- model then control and control then model pass in canonical mode;
- both orders fail with the intended diagnostic in legacy mode;
- defining the opt-out after the first header fails in both orders;
- defining it for the first header and then removing it before the second also
  fails in both orders;
- model-only legacy remains signed `int32_t` with the original values;
- control-only legacy remains unsigned `uint32_t` with the original values.

The committed configuration checks cover the first eight negative cases, and
the new legacy object targets assert single-module signedness and representative
values. Independent probing extended this to 96/96 strict positive header
cases and 12/12 intended rejection cases. No include order silently selects a
different generic status type or constant set.

### I2 — installed consumer/export gap: resolved

Control now installs a relocatable `openarm_controlConfig.cmake`, version file,
and `openarm_control::openarm_control` imported target. Its exported interface
uses only `_IMPORT_PREFIX`, links `openarm_model::openarm_model` and
`Threads::Threads`, and the config discovers both dependencies.

The committed external C11 and C++17 consumers now use `find_package` for the
installed packages, link both imported targets, call one model API and one
control API, and run in opposite include orders. A fresh clean install test
passed. An additional independent test moved the entire installed prefix,
verified that no build/source path was embedded in either CMake export,
configured both consumers from the relocated prefix, and observed verbose link
commands containing both installed archives. Both executables then passed.
Installed model/control headers were byte-identical to their source copies.

### M1 — canonical-only project build: resolved

Model, control, tests, and the active ROS model consumer use module-prefixed
status names internally. A fresh project-wide build with
`OPENARM_DISABLE_LEGACY_GENERIC_STATUS=1` in both C and C++ compiler flags built
and passed 4/4 tests. Its model and control archives were byte-identical to the
ordinary Release build. The ROS-independent paired-transaction source also
compiled strictly as C++17 under the global definition.

## ABI and compatibility evidence

- Fresh base and target Release model archives were byte-for-byte identical:
  `32e57096989b4481df85f4713451d1dc62e55e63ab505657f67ee05cf01c8aab`.
- Fresh base and target Release control archives were byte-for-byte identical:
  `3ab99892c8044806172853443a67f03a4e4c977792caf498e95339a9d0c2dc97`.
- Sorted `nm -g --defined-only` output had no differences: 16 model lines and
  139 control lines.
- C11 consumers compiled against the base model/control headers linked and ran
  against the target archives.
- The model ABI-v1 canary and frozen original control-V1 consumer passed in
  Release; the frozen control consumer also passed under ASan/UBSan.
- Canonical types preserve the exact underlying representations and status
  values. The checked layout offsets remain 8 for
  `oa_ik_diagnostics.status` and 40 for `oa_event.cause`; no record field,
  function name, calling convention, or exported symbol changed.
- The target Release and global-canonical builds also produced identical
  archives, independently confirming that disabling aliases does not affect
  implementation code generation.

## Fresh commands and results

```sh
cmake -S control -B /tmp/openarmik-abi-rereview-control-release \
  -DCMAKE_BUILD_TYPE=Release -DOA_CONTROL_BUILD_TESTS=ON
cmake --build /tmp/openarmik-abi-rereview-control-release --parallel 4
ctest --test-dir /tmp/openarmik-abi-rereview-control-release --output-on-failure
# 4/4 passed, including installed C/C++ compile/link/run

cmake -S model -B /tmp/openarmik-abi-rereview-model-release \
  -DCMAKE_BUILD_TYPE=Release -DOA_MODEL_BUILD_TESTS=ON
cmake --build /tmp/openarmik-abi-rereview-model-release --parallel 4
ctest --test-dir /tmp/openarmik-abi-rereview-model-release --output-on-failure
# 3/3 passed

cmake -S control -B /tmp/openarmik-abi-rereview-canonical-global \
  -DCMAKE_BUILD_TYPE=Release \
  '-DCMAKE_C_FLAGS=-DOPENARM_DISABLE_LEGACY_GENERIC_STATUS=1' \
  '-DCMAKE_CXX_FLAGS=-DOPENARM_DISABLE_LEGACY_GENERIC_STATUS=1'
cmake --build /tmp/openarmik-abi-rereview-canonical-global --parallel 4
ctest --test-dir /tmp/openarmik-abi-rereview-canonical-global --output-on-failure
# 4/4 passed

cmake -S control -B /tmp/openarmik-abi-rereview-control-sanitize \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DOA_CONTROL_SANITIZERS=ON
cmake --build /tmp/openarmik-abi-rereview-control-sanitize --parallel 4
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
ctest --test-dir /tmp/openarmik-abi-rereview-control-sanitize --output-on-failure
# 3/3 passed

# Independent strict header matrix and adversarial behavior probes used:
# -std=c11 / -std=c++17 -pedantic-errors -Wall -Wextra -Werror -fsyntax-only
# 96/96 positive cases passed; 12/12 ambiguous/late-opt-out cases were rejected

# Fresh base archive/symbol comparison
git archive 514e3a8 | tar -x -C /tmp/openarmik-abi-rereview-base-T2yZRH
cmake -S /tmp/openarmik-abi-rereview-base-T2yZRH/model \
  -B /tmp/openarmik-abi-rereview-base-T2yZRH/build-model \
  -DCMAKE_BUILD_TYPE=Release -DOA_MODEL_BUILD_TESTS=OFF
cmake -S /tmp/openarmik-abi-rereview-base-T2yZRH/control \
  -B /tmp/openarmik-abi-rereview-base-T2yZRH/build-control \
  -DCMAKE_BUILD_TYPE=Release -DOA_CONTROL_BUILD_TESTS=OFF
# both builds passed; sha256sum matched the target hashes above
# sorted nm -g --defined-only output had no differences

# The clean install was moved from its original prefix before configuring:
cmake -S control/tests/install_consumer \
  -B /tmp/openarmik-abi-rereview-relocated-consumer \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/tmp/openarmik-abi-rereview-relocated
cmake --build /tmp/openarmik-abi-rereview-relocated-consumer --verbose
# both installed C/C++ consumers linked both relocated archives and passed

git diff --check 514e3a8..5059a3a
# passed
```

GCC/G++ 15.2 was the only C/C++ toolchain available on this host; Clang was
not installed. All checks were hardware-free. No GUI, CAN interface, network,
device, or hardware path was used.
