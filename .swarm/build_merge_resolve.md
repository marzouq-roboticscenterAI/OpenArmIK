# Unified build merge resolution

Status: **DONE**

## Integrated commits

- `ddbe200` — unified native and ROS build path (already applied at handoff)
- `04923ef` — semantic resolution of `0c213b6`
- `8600a47` — cherry-pick of `59a492d`
- `9c86ea0` — cherry-pick of `80f1d13`
- `5bed508` — post-merge control export/test-isolation integration fixes

The `control/CMakeLists.txt` conflict retained the canonical/public-header and
v1 ABI tests, legacy `openarm_control::openarm_control` export, package config,
install rules, and feedback/control tests. It also adopted installed model
dependency targets and a separately compiled `openarm_control_test` archive
with `OPENARM_CONTROL_ENABLE_TEST_HOOKS`, leaving the installed production
archive hook-free. `OpenArm::Control` is available in both build and installed
package contexts while the legacy target remains supported.

The retained installed-header CTest now reuses the model package prefix found
during control configuration instead of assuming the removed nested model
build. The repository-wide installed consumers initialize the CAN v2 record
header before exercising the API.

## Verification

Fresh unified Release build:

```text
scripts/build.sh --tests \
  --output-root /tmp/openarmik-unified-final.ZO4r5S \
  --build-type Release
```

Result: exit 0. Native CTest registrations and passes were CAN 1/1, model 4/4,
commission 2/2, transport 3/3, and control 4/4 (14 total). The installed
all-header C11 and C++17 executables linked CAN, model, commission, transport,
and control and ran successfully. Both ROS packages built and CTest freshly
reported exactly 8 `openarm_ik_ros` registrations.

Fresh two-prefix reuse regression:

```text
tests/test_native_prefix_reuse.sh /tmp/openarmik-prefix-final.zldpib
```

Result: exit 0 for prefix A followed by prefix B. All 14 native CTests and both
installed all-header consumers passed in both rounds. The regression's verbose
rebuild proved the transport and control tests linked prefix B dependencies and
contained no prefix A link/include paths.

Additional audits against prefix B:

- `libopenarm_control.a` defines no `oa_control_test_*` symbols.
- `libopenarm_commission.a` defines no `openarm::commission::test::*` symbols.
- `libopenarm_transport.a` defines no embedded `oa_can_*` implementation symbols.
- Prefix B and the transport/control dependency caches contain no prefix A path.
- `git diff --check` and Bash syntax checks passed.

## Residual risk

The upstream `openarm_description` build emits only its existing CMake
deprecation and unused `OPENARM_IK_ROS_COVERAGE` warnings. No GUI, CAN,
network, or hardware action was performed.
