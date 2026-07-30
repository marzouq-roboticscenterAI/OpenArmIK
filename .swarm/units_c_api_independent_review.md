# Independent review: binary64 coordinate-unit C APIs

## Verdict

**CLEAN — no findings remain.**

Reviewed commit `1f2d543f3bd5cf5614227fca2b5ff9550cdbcdf5` against parent
`2b401b34be32fb89015a73f18180eeaba3e2f7a6`, limited to the Model,
Control, Runtime, and installed native consumer changes. No source was edited.
All generated review artifacts are under `build/review-units-api`.
A targeted follow-up at `3b9ef8b7bbebe90dde2c0e5333dffcdaebf6742e`
independently confirmed the sole Low's closure and found no regression.

## Follow-up closure

The sole Low portability finding is closed. The public strict-C header now
rejects implementations unless `double` has binary radix, 53 mantissa digits,
binary64 maximum exponent 1024, binary64 minimum exponent -1021, eight-byte
storage, and the frozen vector layout/alignment. The C++17 consumer independently
asserts the corresponding `std::numeric_limits<double>` properties, including
`is_iec559`.

Fresh follow-up evidence used
`build/review-units-api/final-confirm-3b9ef8b`:

- strict C11 compilation of `openarm_units.c` passed with `-pedantic-errors
  -Wall -Wextra -Werror -Wconversion -Wdouble-promotion -Wfloat-conversion`;
- strict C++17 compilation of the unit consumer passed with the same warnings;
- the targeted C11 and C++17 Model unit tests passed **2/2**, sequentially.

The follow-up changes only the portability assertions and their C++17 test
mirrors; public declarations, record layouts, and implementation source are
unchanged. The fresh Model archive has the same public symbol set as the prior
review archive. Control/Model API headers other than `openarm_units.h`, both
Runtime V1 headers, and the Runtime 50-symbol manifest are byte-unchanged from
`1f2d543`. Current and frozen Runtime headers still share SHA-256
`e4b2ed9c3e57bd353e805a09270478341681e7aebbe96062e4bb1e39725fd9e5`.

## Static audit evidence

- Unit conversion uses `double` inputs, scale constants, factor, temporaries,
  and output fields throughout; no `float` storage or cast exists. All six
  ordered unit pairs use the correctly rounded binary64 ratios for metres,
  centimetres, and exact 0.0254-metre inches.
- `oa_vec3d_convert` validates both pointers and both unit IDs before output,
  rejects NaN/Inf before arithmetic, rejects non-finite multiplied results as
  overflow, commits through one final struct assignment, and therefore keeps
  output unchanged on every error. Exact self-aliasing is safe.
- Public native XYZ ingress inventory is complete: Model
  `oa_ik_position_v2`, Control `oa_controller_plan_paired_tcp`, and Runtime
  `oa_runtime_plan_paired_tcp_body`. Each now has a unit-aware entry or adapter;
  no other public native XYZ input was found. FK/kinematics/reports are outputs.
- Model converts one target vector to metres and delegates once to V2. Unit
  NaN/Inf/overflow maps to `OA_MODEL_ENONFINITE`; invalid pointers/unit IDs map
  to `OA_MODEL_EINVAL`; conversion failure does not write diagnostics.
- Control converts left and right vectors once, maps all conversion failures to
  its established `OA_CONTROL_EINVAL`, copies every non-coordinate request
  field without changing metre tolerances, and delegates to the existing
  metre-native planner. The output plan pointer is untouched on local failure.
- Runtime's adapter is genuinely header-only. It validates full size/version
  and both reserved fields, converts each vector once, fixes the delegated
  `units_id` to `OA_RUNTIME_UNITS_SI_V1`, copies every identity/deadline/policy
  field, leaves tolerance/report units in metres, and calls the frozen Runtime
  V1 planner. Invalid unit/non-finite input returns `OA_RUNTIME_EINVAL` without
  calling the frozen symbol or writing the plan output.
- Existing Model/Control record layouts and signatures are unchanged; additions
  are new fixed-width typedefs, records, and entry points. `oa_length_unit`,
  `oa_units_status`, control status, and runtime status are `uint32_t`; model
  status remains `int32_t`. `oa_vec3d` is three contiguous, double-aligned
  `double` values, with asserted offsets 0/8/16 and size 24 on the gate host.
- Model installs `openarm_units.h` and exports the implementation through its
  existing target. Control's public dependency on Model supplies the new header
  and symbol in build and split-install use. Runtime installs
  `openarm_runtime_units.h`; `OpenArm::Runtime` already exports Model publicly,
  so installed adapter consumers receive the header path and link dependency.
- Runtime current and frozen `openarm_runtime.h` blobs and
  `expected_symbols.txt` are byte-identical to the parent commit. Current,
  frozen, and installed headers all hash to
  `e4b2ed9c3e57bd353e805a09270478341681e7aebbe96062e4bb1e39725fd9e5`.
  Both build-tree and installed Runtime archives expose exactly the unchanged
  50-symbol manifest.

## Original fresh sequential validation at `1f2d543`

All configurations and builds used Release mode and `--parallel 1`; CTest used
`-j1` with `OPENARM_BUILD_JOBS=1` where nested installed-consumer builds occur.

- Model: **5/5 passed** — model, units C11, units C++17, original ABI canary,
  and Python reference.
- Control: **4/4 passed** — controller suite, C11 ABI consumer, original-V1 ABI
  consumer, and installed public-header C11/C++17 consumer. The build also
  compiled the full strict C11/C++17 public-header ordering matrix.
- Runtime: **12/12 passed** — runtime suite, ordinary C11 consumer, units C11
  and C++17 consumers, header-only adapter test, four frozen/current ABI
  consumers, frozen-header policy, 50-symbol archive manifest, and installed
  split-prefix consumers (including installed unit C11/C++17).
- Independent strict compiles of `openarm_units.c` as C11 and the C++ consumer
  as C++17 passed with `-pedantic-errors -Wall -Wextra -Werror -Wconversion
  -Wdouble-promotion -Wfloat-conversion`.
- `git diff --check 2b401b3 1f2d543` passed. The pre-existing modified
  `transport/tests/test_transport.cpp` and all pre-existing untracked files were
  preserved. No portal, ROS, GUI, network, or hardware test was run.

CTest logs are retained at:

- `build/review-units-api/model/Testing/Temporary/LastTest.log`
- `build/review-units-api/control/Testing/Temporary/LastTest.log`
- `build/review-units-api/runtime/Testing/Temporary/LastTest.log`
