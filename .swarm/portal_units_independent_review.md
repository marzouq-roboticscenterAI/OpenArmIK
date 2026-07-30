# Independent review: portal coordinate units

## Verdict

**CLEAN — no Critical or Important findings.**

Targeted re-review of current main `31cab6a` (`Send selected portal coordinate
units`), following the independent review of `b8cb496`. The prior Important
browser/v2 unit-boundary finding is closed. No source code was edited; the
pre-existing transport edit and unrelated untracked files were preserved.

## Prior Important closure

- `portal_page.cpp:66` now reads only the canonical `targetsM[side]`, derives
  the outgoing coordinates with the binary64 `unitsPerMetre[unit]` factor, and
  posts `{side, unit, x, y, z}` to `/api/v2/move`. With the default selection it
  sends centimetres; after the toggle it sends inches.
- Submission does not read or parse the rounded DOM text fields. `move()` does
  not call `parseDecimal`, `Number`, `parseFloat`, `Math.fround`, or any typed
  float32 storage. The canonical metre array is not modified during payload
  construction, so repeated display toggles and submission preserve it.
- The prior hard-coded `unit:'m'` payload is absent. The focused page test now
  requires selected-unit scaling and the shorthand selected `unit` property,
  and explicitly verifies that `move()` does not invoke `parseDecimal`.
- JavaScript numeric storage and arithmetic remain binary64. `JSON.stringify`
  serializes those numeric values; the strict v2 parser reads them into
  `double` members of `oa_vec3d`. The route then calls
  `normalise_move_to_metres`, whose sole conversion is
  `oa_vec3d_convert(input_unit, OA_LENGTH_UNIT_METRES)`, before constructing
  the metre `MoveRequest` used by the guard and ROS action.
- The unit test independently models the browser/server boundary for both
  `cm` (`100.0`) and `in` (`1.0 / 0.0254`): canonical metres are scaled to the
  selected unit, serialized with binary64 round-trip precision, strictly
  parsed, and normalized through the C API. Both results are within one ULP of
  the canonical metre value.

## Page regression audit

The centimetre default, inches labels/rendering, strict decimal validation,
canonical target arrays, dirty-field polling protection, metre-tagged state,
virtual-only/collision/E-stop wording, and truthful no-portal-grid statement
are unchanged from the prior audit. The two-file fix changes only the
unit-display factor spelling, payload construction/message, and focused tests.
`git diff --check 31cab6a^ 31cab6a` passed.

## Fresh focused verification

All new artifacts are under
`build/review-portal-units/followup-31cab6a`, including repo-local `TMPDIR` and
`ROS_LOG_DIR`. A fresh Release CMake configuration compiled
`portal_core.cpp`, `portal_page.cpp`, and `test_portal_core.cpp` sequentially
with `--parallel 1`.

- Focused portal inventory: exactly **24 cases in 12 suites**.
- Focused `test_portal_core`: **PASS, 24/24**.
- No full 13-test CTest rerun was requested or performed.
- No GUI, browser, network, or hardware was used.

Retained evidence:

- `build/review-portal-units/followup-31cab6a/configure.txt`
- `build/review-portal-units/followup-31cab6a/build.txt`
- `build/review-portal-units/followup-31cab6a/portal-gtest-list.txt`
- `build/review-portal-units/followup-31cab6a/portal-gtest.txt`
- `build/review-portal-units/followup-31cab6a/portal-gtest.xml`
