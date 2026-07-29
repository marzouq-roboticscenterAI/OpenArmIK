# Public ABI namespace fix evidence

Date: 2026-07-29 (America/Los_Angeles)

Status: DONE

## Change

- Model public declarations now use signed `oa_model_status` and
  `OA_MODEL_*`; control public declarations now use unsigned
  `oa_control_status` and `OA_CONTROL_*`.
- `OPENARM_DISABLE_LEGACY_GENERIC_STATUS` disables all generic aliases. Without
  it, the first model/control header supplies the original `oa_status` and
  `OA_*` names through one shared guard, so both headers compile in either
  order while single-header sources remain compatible.
- No function name, integer representation, status value, record order, size,
  alignment, or field offset changed. Compile assertions retain the status
  field offsets (model diagnostics 8; control event 40); the verified x86-64
  sizes remain 256 and 48 bytes respectively.

## Verification

- Fresh Release configure/build: passed. The build compiled 80 strict matrix
  translation units: all 20 ordered pairs of the five public headers, C11 and
  C++17, with legacy aliases enabled and disabled.
- Release CTest: 4/4 passed, including the C11 ABI consumer, frozen original
  control V1 consumer, and external C11/C++17 consumers built from independently
  installed model/control headers in both orders.
- Fresh ASan/UBSan RelWithDebInfo build: passed; sanitizer CTest 3/3 passed.
- Model legacy C11 tests: passed 3,200 randomized bounded IK cases.
- Model ABI-v1 canary compiled against its published declarations and passed;
  the frozen controller V1 binary consumer passed in Release and ASan/UBSan.
- `git diff --check`: passed. No Python, ROS GUI, CAN/network, or hardware path
  was executed.

## Compatibility note

Generic status names are intentionally a single-module source-compatibility
surface: in a multi-module translation unit they belong to the first included
model/control header. New combined consumers should define
`OPENARM_DISABLE_LEGACY_GENERIC_STATUS` and use module-prefixed names.
