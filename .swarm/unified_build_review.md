# Independent unified-build re-review

Reviewed series: `514e3a8..0c213b680f886e626760f61707bc2b42dc8034c4`

Updated commits: `16906a0`, `0c213b6`

Date: 2026-07-29 (America/Los_Angeles)

Verdict: **FINDINGS**

## Finding

### MEDIUM — Reusing a native build root with a new install prefix silently keeps old dependency packages

`scripts/build_native.sh:114-127` passes the selected
`CMAKE_PREFIX_PATH` on every component configure, but does not clear or
override CMake's cached package-specific directory variables. Transport uses
`find_package(OpenArmCan)` (`transport/CMakeLists.txt:18-22`) and control uses
`find_package(openarm_model)` (`control/CMakeLists.txt:21-28`), so an existing
`OpenArmCan_DIR` or `openarm_model_DIR` wins over the newly selected prefix.

This was reproduced without modifying source:

```bash
# First build installed to prefix A.
./scripts/build.sh --tests \
  --output-root '/tmp/openarmik updated review fresh' \
  --build-type Release

# Reuse its native build root but request prefix B.
./scripts/build_native.sh \
  --build-root '/tmp/openarmik updated review fresh/native_build' \
  --install-prefix '/tmp/openarmik updated review alternate install' \
  --build-type Release
```

The second command exited 0 and installed outputs into prefix B, but its caches
and generated build rules still used prefix A:

```text
transport/CMakeCache.txt:
  CMAKE_INSTALL_PREFIX=/tmp/openarmik updated review alternate install
  CMAKE_PREFIX_PATH=/tmp/openarmik updated review alternate install
  OpenArmCan_DIR=/tmp/openarmik updated review fresh/install/lib/cmake/OpenArmCan

control/CMakeCache.txt:
  CMAKE_INSTALL_PREFIX=/tmp/openarmik updated review alternate install
  CMAKE_PREFIX_PATH=/tmp/openarmik updated review alternate install
  openarm_model_DIR=/tmp/openarmik updated review fresh/install/lib/cmake/openarm_model

transport/CMakeFiles/openarm_transport.dir/flags.make:
  -isystem "/tmp/openarmik updated review fresh/install/include"
transport/CMakeFiles/openarm_transport_tests.dir/link.txt:
  "/tmp/openarmik updated review fresh/install/lib/libopenarm_can.a"
control/CMakeFiles/openarm_control_tests.dir/link.txt:
  "/tmp/openarmik updated review fresh/install/lib/libopenarm_model.a"
```

Thus prefix B can contain libraries compiled and tested against a different
SDK prefix even though the command succeeds. This is a stale-artifact and
deterministic-linkage failure in the new public native build entry point.
Explicitly set `OpenArmCan_DIR` and `openarm_model_DIR` from
`install_prefix`, unset the cached variables before configuration, or reject a
build-root/install-prefix pairing that differs from the one recorded in the
cache.

The ordinary top-level `build.sh --incremental` path keeps build and install
under one fixed output root and passed; the failure requires the independently
documented `build_native.sh` paths to be reused with a changed prefix.

## Prior findings rechecked

- **CAN exports — resolved.** A clean prefix contains versioned `OpenArmCan`
  and compatibility `openarm_can` packages. Both expose `OpenArm::Can`.
- **Duplicate/rebuilt dependencies — resolved for a clean prefix.** Transport
  finds the installed `OpenArmCan`; control finds the installed model. There is
  no `openarm_transport_codec` object, no model sub-build below control, and
  the installed transport archive defines no `oa_can_*` symbol.
- **Production test hooks — resolved.** Clean Release control and commission
  archives expose no test hooks. Non-installed test variants contain the
  expected hook symbols, and all component tests pass against those variants.
- **Colon/semicolon paths — resolved.** Both entry points reject `:` and `;`
  before creating output. Paths containing spaces build, install, discover,
  link, and run successfully.
- **Installed branch-owned targets — resolved for CAN/model/commission/
  transport.** A strict relocated external C11 consumer found both CAN package
  names plus `openarm_model`, `openarm_commission`, and `OpenArmTransport`;
  asserted `OpenArm::Can`, `OpenArm::Model`, `OpenArm::Commission`,
  `OpenArm::Transport`, and the compatibility transport target; linked the
  relocated archives; and ran successfully. The installed transport target
  carries `$<LINK_ONLY:OpenArm::Can>` and its config declares the CAN
  dependency.

## Integration dependency, not a branch finding

The model/control generic-status collision and the installed
`OpenArm::Control` export remain owned by the separately reviewed ABI branch,
as directed. They are not re-flagged here. This branch prepares strict installed
C11 and C++17 all-five-header consumers and correctly defers them until the
control config and collision-free headers are present:

```text
Installed all-header consumers deferred until control export/status integration
```

The clean branch-owned package graph does not block that integration. The
combined integration must rerun `./scripts/build.sh --tests` and must actually
configure, build, and execute both strict all-header consumers; a deferral in
the combined tree would be a failure.

## Fresh verification evidence

- `git diff --check 514e3a8..0c213b6` and `bash -n` passed.
- Clean Release `--tests` build at
  `/tmp/openarmik updated review fresh` passed with a space-containing path:
  CAN 1/1, model 4/4, commission 2/2, transport 3/3, and control 3/3; both ROS
  packages built; all eight current ROS tests were freshly registered.
- A top-level `--incremental --tests` rerun passed the same 13 native tests and
  eight ROS registrations.
- A clean Debug tests-off build installed all products; every native component
  and ROS enumerated zero tests, and the production archive-boundary checks
  passed.
- Five safe ROS tests were executed and passed: paired transaction, generated
  URDF, and the three close-helper argument tests. DDS-launching tests were not
  run, consistent with the no-network constraint.
- Standalone transport registered four tests, including vcan smoke; only its
  three hardware-free tests were run and passed. The unified profile registered
  and ran the same three, never the vcan test.
- Relocating the native `include` and `lib` trees to a new path containing
  spaces preserved package discovery and external linkage.
- `:` and `;` output/build/install paths returned 2 and created no directory.
  A cleanup child symlinked outside its output root returned 2 and preserved
  the external sentinel.
- No GUI, CAN interface, socket, network, hardware, sudo, or install mutation
  outside temporary prefixes was used.

## Disposition

**FINDINGS.** All earlier branch-owned packaging and Release-boundary findings
are closed, and the ABI-owned work is properly isolated as an integration
dependency. The native entry point still needs to eliminate or reject cached
cross-prefix dependency reuse before its explicit build/install paths are
deterministic.
