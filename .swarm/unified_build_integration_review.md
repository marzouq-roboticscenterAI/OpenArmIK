# Final unified-build integration review

Reviewed tree: `main` at `aeffc2b3625b07d41daa0aeff8c36b74dd9eecc9`

Date: 2026-07-29 (America/Los_Angeles)

Verdict: **FINDINGS**

## Finding

### MEDIUM — The two-prefix regression passes while its installed all-header consumers still link prefix A

The production component dependency fix is integrated correctly, but the
installed C11/C++17 consumer build has the same unrefreshed package-cache
problem that was fixed for transport and control.

Exact reproduction:

```bash
./tests/test_native_prefix_reuse.sh \
  '/tmp/openarmik final integration prefix reuse'
```

The script exited 0, reported all 14 native tests passing under both prefixes,
built both installed all-header executables in both rounds, and printed:

```text
Native prefix-reuse regression passed: .../prefix-a -> .../prefix-b
```

After the prefix-B round, however, the reused installed-consumer cache showed a
new general prefix but five stale package-specific directories:

```text
CMAKE_PREFIX_PATH=.../prefix-b
OpenArmCan_DIR=.../prefix-a/lib/cmake/OpenArmCan
OpenArmTransport_DIR=.../prefix-a/lib/cmake/OpenArmTransport
openarm_commission_DIR=.../prefix-a/lib/cmake/openarm_commission
openarm_control_DIR=.../prefix-a/lib/cmake/openarm_control
openarm_model_DIR=.../prefix-a/lib/cmake/openarm_model
```

Both supposed prefix-B consumers compile and link entirely against prefix A:

```text
openarm_installed_c11 flags:   -isystem ".../prefix-a/include"
openarm_installed_cxx17 flags: -isystem ".../prefix-a/include"

openarm_installed_c11 link:
  .../prefix-a/lib/libopenarm_can.a
  .../prefix-a/lib/libopenarm_commission.a
  .../prefix-a/lib/libopenarm_transport.a
  .../prefix-a/lib/libopenarm_control.a
  .../prefix-a/lib/libopenarm_model.a

openarm_installed_cxx17 link: the same five prefix-A archives
```

Running both executables returned 0, but that only revalidated prefix A. A
prefix-B installed-header/export/link regression can therefore be missed while
the permanent A-to-B test passes.

`scripts/build_native.sh:222-226` reconfigures the existing
`installed_native_consumer` directory with only `CMAKE_PREFIX_PATH`; it neither
clears nor overrides `OpenArmCan_DIR`, `OpenArmTransport_DIR`,
`openarm_commission_DIR`, `openarm_control_DIR`, and `openarm_model_DIR`.
`tests/test_native_prefix_reuse.sh:68-75` checks only transport/control verbose
output, not the installed-consumer cache, include flags, or link lines.

Fix by recreating the installed-consumer build directory when the prefix
changes, or explicitly setting and asserting all five package directories.
Extend the regression to require prefix B and reject prefix A in both installed
consumer compile/link commands before executing them.

This is a validation-integrity defect, not evidence that the installed
prefix-B production libraries themselves link stale dependencies. Transport
and control caches and executable link lines correctly select prefix B.

## Passing integrated evidence

### Fresh unified Release build

The following fresh space-containing build exited 0:

```bash
./scripts/build.sh --tests \
  --output-root '/tmp/openarmik final integration review' \
  --build-type Release
```

Native CTest registration and results were:

- CAN: 1/1 passed;
- model: 4/4 passed, including generator determinism;
- commission: 2/2 passed;
- transport: 3/3 passed, with no vcan test registered;
- control: 4/4 passed, including the C11 ABI, frozen original-v1 ABI, and
  installed public-header consumer; and
- total: 14/14 passed.

Both ROS packages built and installed. Fresh CTest enumeration listed exactly
the expected eight `openarm_ik_ros` tests.

A same-prefix `--incremental --tests` rerun also exited 0, repeated all 14
native passes, rebuilt and ran the all-header consumers, and again registered
exactly eight ROS tests.

### Actual installed five-library consumers

In the fresh unified build, not the defective cross-prefix reuse case, both
strict external executables were ELF binaries, included all five installed
headers in opposite orders with canonical status names, linked every installed
library, called public functions from CAN/model/commission/transport/control,
and returned 0:

```text
openarm_installed_c11   exit 0
openarm_installed_cxx17 exit 0
```

Their exact link lines used only
`/tmp/openarmik final integration review/install/lib` archives. Their CMake
cache discovered `OpenArmCan`, `OpenArmTransport`, `openarm_model`,
`openarm_commission`, and `openarm_control` from that same prefix.

### ABI, export, and feedback merge resolution

Inspection confirmed the merge preserved:

- module-prefixed `oa_model_status`/`OA_MODEL_*` and
  `oa_control_status`/`OA_CONTROL_*` APIs with guarded legacy aliases;
- build-tree `OpenArm::Control` and legacy
  `openarm_control::openarm_control` aliases;
- installed `openarm_control` config/version/target export plus its
  `OpenArm::Control` adapter;
- installed model and `Threads` dependencies in the control package;
- the separate `openarm_control_test` archive with test-hook compilation;
- the strict public-header matrix and installed-header CTest;
- C11 and original-v1 ABI consumers; and
- delayed-feedback provenance/atomicity, lifecycle retirement, and event
  overflow termination tests wired into `openarm_control_tests`.

The compiled `openarm_control_tests` suite containing those feedback paths
passed in the fresh build and in both prefix-reuse rounds.

### Production archive boundaries

Independent `nm -gC --defined-only` checks on the fresh installed Release
prefix found:

- no `oa_control_test_*` symbols in `libopenarm_control.a`;
- no `openarm::commission::test::*` symbols in
  `libopenarm_commission.a`; and
- no embedded `oa_can_*` implementation symbols in
  `libopenarm_transport.a`.

The non-installed control and commission test archives contain their expected
hook symbols, proving the hooks were isolated rather than lost.

### Production two-prefix linkage

After the A-to-B run, transport and control caches selected prefix B;
transport tests linked prefix-B CAN; and control tests/ABI consumers linked
prefix-B model. Their active compile flags and link commands contained no
prefix-A dependency. No duplicate transport codec or nested control model build
was present.

## Safety and workspace notes

- `git diff --check` and Bash syntax checks passed.
- No GUI, CAN interface/socket, network, hardware, sudo, or system install was
  used.
- Existing untracked portal/web reports and other unrelated untracked `.swarm`
  files were observed and left untouched.
- The only repository write from this review is this report.

## Disposition

**FINDINGS.** Fresh and same-prefix unified integration, ABI/header coexistence,
control export, feedback tests, Release archive isolation, and ROS registration
are clean. The cross-prefix regression is not yet a valid installed-SDK proof
because both all-header consumers silently remain bound to prefix A.
