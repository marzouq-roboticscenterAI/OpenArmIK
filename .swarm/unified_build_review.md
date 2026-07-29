# Independent unified-build third review

Reviewed series: `514e3a8..59a492d5f60857a3b2ee2fee8a5f41441a476968`

Updated commits: `16906a0`, `0c213b6`, `59a492d`

Date: 2026-07-29 (America/Los_Angeles)

Verdict: **CLEAN_WITH_INTEGRATION_DEPENDENCY**

## Result

No branch-scoped build, packaging, cleanup, test-selection, or installed-target
finding remains. The stale native dependency-prefix finding is fixed and all
earlier build findings remain closed.

The collision-free model/control status headers and installed
`OpenArm::Control` export are now merged separately on `main` but are not in
this branch checkout. They remain an integration dependency, not a finding
against this branch.

## Two-prefix stale-cache verification

The permanent regression was run independently with a space-containing root:

```bash
./tests/test_native_prefix_reuse.sh \
  '/tmp/openarmik third review prefix reuse'
```

It exited 0. The same five native build directories first built, installed,
and passed all 13 tests against prefix A, then changed to prefix B and again
passed all 13 tests.

After the A-to-B reuse, the active caches contained exactly the requested
prefix B:

```text
transport/CMakeCache.txt:
  CMAKE_INSTALL_PREFIX=/tmp/openarmik third review prefix reuse/prefix-b
  CMAKE_PREFIX_PATH=/tmp/openarmik third review prefix reuse/prefix-b
  OpenArmCan_DIR=/tmp/openarmik third review prefix reuse/prefix-b/lib/cmake/OpenArmCan

control/CMakeCache.txt:
  CMAKE_INSTALL_PREFIX=/tmp/openarmik third review prefix reuse/prefix-b
  CMAKE_PREFIX_PATH=/tmp/openarmik third review prefix reuse/prefix-b
  openarm_model_DIR=/tmp/openarmik third review prefix reuse/prefix-b/lib/cmake/openarm_model
```

Independent artifact inspection confirmed:

- transport compile flags and its current `.o.d` dependency select
  `prefix-b/include/openarm_can.h`;
- control compile flags and its current `.o.d` dependency select
  `prefix-b/include/openarm_model.h`;
- `openarm_transport_tests` links
  `prefix-b/lib/libopenarm_can.a`;
- `openarm_control_tests`, the C11 ABI consumer, and the original-v1 consumer
  link `prefix-b/lib/libopenarm_model.a`; and
- no active cache, compile flag, object dependency, or executable link command
  selects prefix A.

The implementation now passes explicit `OpenArmCan_DIR` and
`openarm_model_DIR` values on every relevant configure and asserts the selected
install prefix, general prefix path, and package-specific directories in each
resulting `CMakeCache.txt`. This directly closes the reproduced failure mode.

## Prior findings rechecked

- **CAN exports:** clean prefixes provide both `OpenArmCan` and compatibility
  `openarm_can`; both expose `OpenArm::Can`.
- **Dependency graph:** transport finds and links installed `OpenArm::Can`;
  control finds and links the installed model. There is no transport codec copy
  and no model sub-build below control.
- **Archive boundaries:** installed Release control and commission archives
  expose no test-hook symbols. Their non-installed test variants contain the
  expected hooks. Installed transport defines no `oa_can_*` implementation
  symbols.
- **Installed targets:** a strict external C11 consumer found both CAN package
  names, `openarm_model`, `openarm_commission`, and `OpenArmTransport`; asserted
  `OpenArm::Can`, `OpenArm::Model`, `OpenArm::Commission`,
  `OpenArm::Transport`, and the compatibility transport target; linked and ran
  from the clean prefix containing spaces. Transport's installed export carries
  its `$<LINK_ONLY:OpenArm::Can>` dependency.
- **Path handling:** spaces succeed. `:` and `;` output/build/install paths
  return 2 before directory creation.
- **Cleanup:** a child symlink escaping the output root is rejected and its
  external sentinel remains intact.

## Focused build and test evidence

- `git diff --check 514e3a8..59a492d` and Bash syntax checks passed.
- A fresh top-level Release `--tests` build at
  `/tmp/openarmik third review clean` passed: CAN 1/1, model 4/4, commission
  2/2, transport 3/3, and control 3/3; both ROS packages built; all eight
  current ROS tests were freshly registered.
- A top-level `--incremental --tests` rerun passed the same 13 native tests and
  eight ROS registrations.
- A fresh Debug native tests-off build installed all five products and
  enumerated zero tests for every component.
- The unified profile did not register or run the vcan smoke test.
- No GUI, CAN interface/socket, network, hardware, or sudo action was used.

## Integration dependency

This branch correctly prints:

```text
Installed all-header consumers deferred until control export/status integration
```

That deferral is expected only in this branch checkout. After integration with
the already reviewed and merged ABI/control-export commits, the combined tree
must rerun `./scripts/build.sh --tests`; both strict installed C11 and C++17
all-five-header consumers must configure, build, and execute. A deferral in the
combined tree would be a failure.

## Disposition

**CLEAN_WITH_INTEGRATION_DEPENDENCY.** The branch is sound within its build and
packaging scope. Final integration is gated only on consuming the separately
merged ABI/control-export work and running the prepared all-header consumers.
