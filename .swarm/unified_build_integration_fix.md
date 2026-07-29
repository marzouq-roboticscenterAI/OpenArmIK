# Unified-build installed-consumer prefix fix

Date: 2026-07-29 (America/Los_Angeles)

Status: **DONE**

Finding source: `.swarm/unified_build_integration_review.md`

Implementation: `8f3a99915069531e1e96c14798f7d4cf59b5467b`

## Resolution

`scripts/build_native.sh` now supplies and asserts the current install prefix
for all five package-specific cache variables used by the repository-wide
installed consumer:

- `OpenArmCan_DIR`
- `openarm_model_DIR`
- `openarm_commission_DIR`
- `OpenArmTransport_DIR`
- `openarm_control_DIR`

The control installed-header CTest likewise pins both its freshly installed
control package and the current model package, eliminating the second stale
subordinate cache found during the full cache audit.

`tests/test_native_prefix_reuse.sh` now checks every active top-level and
subordinate CMake cache after the prefix-B round. It also cleanly rebuilds both
repository-wide C11/C++17 consumers, rejects prefix A in each `flags.make` and
`link.txt`, requires the prefix-B include directory and all five prefix-B
archives, then executes both resulting binaries.

## Verification

Fresh space-containing two-prefix Release regression:

```text
tests/test_native_prefix_reuse.sh \
  '/tmp/openarmik prefix refresh.ADDjhY'
```

Result: exit 0. Both rounds passed all 14 native CTests and both installed
all-header consumers. After A -> B reuse:

- all eight discovered `CMakeCache.txt` files excluded prefix A;
- the repository-wide consumer's general prefix and five package directories
  selected prefix B;
- the control nested consumer selected its local control export and prefix-B
  model;
- the transport nested consumer selected its local transport export and
  prefix-B CAN;
- both C11 and C++17 command files used the prefix-B include directory and all
  five prefix-B archives, with no prefix-A path; and
- both rebuilt binaries returned 0.

Fresh space-containing unified Release build:

```text
scripts/build.sh --tests \
  --output-root '/tmp/openarmik prefix unified.K0ouAm' \
  --build-type Release
```

Result: exit 0. Native results were CAN 1/1, model 4/4, commission 2/2,
transport 3/3, and control 4/4 (14/14 total). Both installed all-header
consumers passed, both ROS packages built, and exactly eight
`openarm_ik_ros` tests were registered.

Independent Release archive inspection found no `oa_control_test_*` symbols in
installed control, no `openarm::commission::test::*` symbols in installed
commission, and no embedded `oa_can_*` definitions in installed transport.
`git diff --check` and Bash syntax checks passed.

## Residual risk

None identified for the finding. The upstream description package retains its
pre-existing non-fatal CMake deprecation and unused coverage-variable warnings.
No GUI, CAN, network, hardware, sudo, or system install action was performed.
