# Public ABI namespace fix evidence

Date: 2026-07-29 (America/Los_Angeles)

Status: DONE

## Change

- Model public declarations now use signed `oa_model_status` and
  `OA_MODEL_*`; control public declarations now use unsigned
  `oa_control_status` and `OA_CONTROL_*`.
- Model-only and control-only sources retain their original generic names.
  Combining those headers without defining
  `OPENARM_DISABLE_LEGACY_GENERIC_STATUS` before both now fails deterministically
  in either order; canonical combined consumers compile in either order.
- Control installs a relocatable `openarm_control` CMake config and
  `openarm_control::openarm_control` target with model and Threads dependencies.
  Implementations and active project consumers use canonical names internally.
- No function name, integer representation, status value, record order, size,
  alignment, or field offset changed. Compile assertions retain the status
  field offsets (model diagnostics 8; control event 40); the verified x86-64
  sizes remain 256 and 48 bytes respectively.

## Verification

- Fresh Release configure/build: passed. Eight adversarial C11/C++17 probes
  verified that ambiguous and late-opt-out model/control combinations fail in
  both orders. Eighty strict positive translation units covered every ordered
  header pair in canonical mode, all non-conflicting legacy pairs, both complete
  five-header orders, and both single-module legacy surfaces.
- Release CTest: 4/4 passed, including the C11 ABI consumer, frozen original
  control V1 consumer, and external C11/C++17 consumers that discover installed
  packages, link both installed archives, call both APIs, and run in both orders.
- Fresh ASan/UBSan RelWithDebInfo build: passed; sanitizer CTest 3/3 passed.
- A global canonical-only Release build and CTest passed 4/4. The ROS-independent
  transaction implementation also compiled strictly with the global opt-out.
- Model C11 tests passed 3,200 randomized bounded IK cases.
- Model ABI-v1 canary compiled against its published declarations and passed;
  the frozen controller V1 binary consumer passed in Release and ASan/UBSan.
- Release model/control archives are byte-identical to the pre-review build;
  their SHA-256 values remain `32e57096989b4481df85f4713451d1dc62e55e63ab505657f67ee05cf01c8aab`
  and `3ab99892c8044806172853443a67f03a4e4c977792caf498e95339a9d0c2dc97`.
- `git diff --check`: passed. No Python, ROS GUI, CAN/network, or hardware path
  was executed.

## Commands

```sh
cmake -S control -B /tmp/openarmik-abi-fix2-release \
  -DCMAKE_BUILD_TYPE=Release -DOA_CONTROL_BUILD_TESTS=ON
cmake --build /tmp/openarmik-abi-fix2-release --parallel 4
ctest --test-dir /tmp/openarmik-abi-fix2-release --output-on-failure
# 4/4 passed

cmake -S control -B /tmp/openarmik-abi-fix2-sanitize \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DOA_CONTROL_SANITIZERS=ON
cmake --build /tmp/openarmik-abi-fix2-sanitize --parallel 4
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
ctest --test-dir /tmp/openarmik-abi-fix2-sanitize --output-on-failure
# 3/3 passed

cmake -S control -B /tmp/openarmik-abi-fix2-canonical-global \
  -DCMAKE_BUILD_TYPE=Release \
  '-DCMAKE_C_FLAGS=-DOPENARM_DISABLE_LEGACY_GENERIC_STATUS=1' \
  '-DCMAKE_CXX_FLAGS=-DOPENARM_DISABLE_LEGACY_GENERIC_STATUS=1'
cmake --build /tmp/openarmik-abi-fix2-canonical-global --parallel 4
ctest --test-dir /tmp/openarmik-abi-fix2-canonical-global --output-on-failure
# 4/4 passed
```

## Compatibility note

Generic status names are intentionally a single-module source-compatibility
surface. New combined consumers must define
`OPENARM_DISABLE_LEGACY_GENERIC_STATUS` before either header and use
module-prefixed names; ambiguous legacy combinations are rejected rather than
assigned an include-order-dependent meaning.
