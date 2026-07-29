# Independent public-header / ABI collision verification

Date: 2026-07-29 (America/Los_Angeles)

Tree: `main` at `19a92de923f7`

Host/compiler: x86_64, GCC/G++ 15.2.0

Disposition: **DONE_WITH_CONCERNS**

## Result

The hypothesis is confirmed. `openarm_model.h` and `openarm_control.h` cannot
coexist in one strict C11 or C++17 translation unit, in either include order.
This is a compile-time public namespace/type collision, not an ELF function
symbol collision:

| Public name | Model declaration | Control declaration | Effect |
|---|---|---|---|
| `oa_status` | `typedef int32_t oa_status;` (`model/include/openarm_model.h:25`) | `typedef uint32_t oa_status;` (`control/include/openarm_control.h:19`) | Hard incompatible typedef declaration (signed versus unsigned). |
| `OA_OK` | `((oa_status)0)` (line 34) | `UINT32_C(0)` (line 33) | Macro redefinition; same value but different replacement list/type. |
| `OA_EINVAL` | `((oa_status)1)` (line 35) | `UINT32_C(1)` (line 34) | Macro redefinition; same value but different replacement list/type. |

The functions themselves have distinct names, so there is no duplicate model /
control linker symbol. The status typedef and constants are source-level names;
their collision prevents an adapter from merely including both APIs.

## Exact reproduction

Each test was sent to the compiler on standard input, so no temporary source or
object file was created:

```bash
printf '#include "openarm_model.h"\n#include "openarm_control.h"\nint main(void) { return 0; }\n' |
  cc -std=c11 -pedantic-errors -Wall -Wextra -Werror -fsyntax-only -x c - \
    -Imodel/include -Icontrol/include

printf '#include "openarm_control.h"\n#include "openarm_model.h"\nint main(void) { return 0; }\n' |
  cc -std=c11 -pedantic-errors -Wall -Wextra -Werror -fsyntax-only -x c - \
    -Imodel/include -Icontrol/include

# The same two sources were also compiled with:
c++ -std=c++17 -pedantic-errors -Wall -Wextra -Werror -fsyntax-only -x c++ - \
  -Imodel/include -Icontrol/include
```

All four commands exited 1. In both C11 orders GCC reports `conflicting types
for 'oa_status'` plus redefinitions of `OA_OK` and `OA_EINVAL`. In both C++17
orders it reports the corresponding `conflicting declaration` plus both macro
redefinitions. The result is not warning-policy-only: the incompatible typedef
is a hard language diagnostic.

## Full public-header collision audit

Audited intended public headers:

- `can/include/openarm_can.h`
- `model/include/openarm_model.h`
- `transport/include/openarm_transport.h`
- `commission/include/openarm_commission.h`
- `control/include/openarm_control.h`

Strict standalone compile result: **5/5 pass** in C11 and **5/5 pass** in C++17.

Every ordered pair (20 per language) was then compiled using the same
`-pedantic-errors -Wall -Wextra -Werror -fsyntax-only` policy. Results:

| Language | Pass | Fail | Failed ordered pairs |
|---|---:|---:|---|
| C11 | 18 | 2 | model -> control; control -> model |
| C++17 | 18 | 2 | model -> control; control -> model |

An exhaustive lexical cross-header scan of public `OA_*` and `oa_*` identifiers
found exactly three names present in more than one header: `oa_status`, `OA_OK`,
and `OA_EINVAL`. Thus there is no additional public typedef, tag, function,
macro, or enum-constant collision among these five headers today. Include guards
are also distinct.

The only installed workspace copy is
`ros2_ws/install/include/openarm_model.h`; it is byte-identical to the source
header (both SHA-256
`1d09e17ce711b871498dcd5676c4984b141f91a383691da64047ae5b3077723d`).
Model, transport, commission, and control have header install rules. CAN has no
install/export rule, so it has no installed header/package despite having an
intended public include directory.

## ABI consistency observations

1. The colliding status types have equal width but different signedness. Do not
   resolve this by changing either underlying type: model callers and the model
   diagnostics record currently use `int32_t`, while control V1 uses
   `uint32_t`. A spelling-only namespace migration can preserve each ABI
   exactly.
2. Model's two versioned IK records put `abi_version` before `struct_size`
   (`openarm_model.h:58-59,72-73`). Every versioned record in CAN, transport,
   commission, and control puts `struct_size` first. Model's ordering is also in
   its checked ABI-v1 canary, so it is a cross-module convention inconsistency
   but must not be reordered in an existing ABI.
3. Control deliberately remains `OA_CONTROL_ABI_V1`. Its frozen original-V1
   header confirms `uint32_t oa_status` and the generic `OA_OK` spelling. Three
   grown records expose frozen prefix sizes; changing the control status names
   or record layouts in place would unnecessarily break V1 source/ABI.
4. C function linkage does not encode typedef names. Keeping model status
   canonical storage as `int32_t` and leaving all function names/signatures and
   record layouts otherwise unchanged permits a binary-neutral namespace fix.

## Smallest compatibility-preserving migration

Leave `openarm_control.h` V1 unchanged and migrate the model header only:

1. Add canonical `typedef int32_t oa_model_status;`.
2. Declare model functions and the diagnostics `status` member with
   `oa_model_status` (same exact underlying type/layout as today).
3. Add canonical constants `OA_MODEL_OK`, `OA_MODEL_EINVAL`,
   `OA_MODEL_ENONFINITE`, `OA_MODEL_EBOUNDS`,
   `OA_MODEL_ENOCONVERGENCE`, `OA_MODEL_ESTAGNATED_AT_BOUNDS`,
   `OA_MODEL_ESINGULAR`, and `OA_MODEL_EBUDGET` with the existing values and
   casts.
4. By default, preserve existing model source using deprecated aliases:
   `typedef oa_model_status oa_status;` and `#define OA_OK OA_MODEL_OK`, etc.
   Put **all** generic model aliases behind one documented switch such as
   `OPENARM_MODEL_NO_LEGACY_STATUS_NAMES`.
5. The combined adapter defines that switch before including the model header,
   then includes model plus control in either order. It uses `oa_model_status` /
   `OA_MODEL_*` for model results and the unchanged V1 `oa_status` / `OA_*` for
   control results.

This is the smallest migration because it adds no new exported function and
changes no integer representation, calling convention, struct size/alignment,
field offset, status value, or existing control V1 spelling. Existing
model-only source remains valid by default; only a consumer that needs header
coexistence opts out of the legacy aliases. A small compile-only C11/C++17
regression target should cover all five headers in forward and reverse order
with the opt-out enabled, plus legacy model-only and frozen control-V1
consumers.

Avoid an include-order-dependent workaround (`#ifdef OPENARM_CONTROL_H`) and
avoid preprocessor `#undef` surgery in the adapter: both make the visible API
depend on include order and can silently select the wrong module's status
constant.

## Scope / side effects

No GUI, CAN interface, network, device, or runtime library call was made. No
build/configure/install was run. Compiler checks were syntax-only over standard
input. The only workspace write from this verifier is this requested report.
