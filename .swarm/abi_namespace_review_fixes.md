# ABI namespace review fixes

Date: 2026-07-29 (America/Los_Angeles)

Status: DONE

- I1: replaced first-header-wins aliases with deterministic rejection of
  combined legacy model/control headers in both orders. The opt-out must precede
  both headers. C11/C++17 adversarial checks cover default and late opt-out in
  both orders; single-module compatibility has signedness/value assertions.
- I2: added the installed `openarm_control::openarm_control` CMake target,
  package config/version files, and transitive installed model/Threads
  dependencies. External installed C11 and C++17 consumers include opposite
  orders, find both packages, link both archives, call both APIs, and run.
- M1: migrated model, control, and the current ROS model consumers to canonical
  status names. A global `OPENARM_DISABLE_LEGACY_GENERIC_STATUS=1` build and all
  four Release tests pass; the ROS-independent transaction source also compiles
  strictly under that definition.

Fresh Release, ASan/UBSan, ABI-v1, strict header matrix, installed-package, and
binary identity checks all pass. Exact command evidence is recorded in
`.swarm/abi_namespace_fix.md` and the task return.
