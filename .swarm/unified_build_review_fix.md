# Unified build review resolution

Date: 2026-07-29 (America/Los_Angeles)

Branch: `feat/unified-native-build`

Reviewed revision: `16906a02aa555775aec7255f4d720e2ee092f6ed`

Disposition: **DONE** within build/packaging ownership

## Resolved findings

### Installed CMake graph

CAN now installs versioned `OpenArmCan` and compatibility `openarm_can` CMake
packages. Both expose the imported `OpenArm::Can` target. Model retains
`openarm_model::openarm_model` and adds `OpenArm::Model`; transport retains
`OpenArm::openarm_transport` and adds `OpenArm::Transport`.

Transport uses `find_package(OpenArmCan CONFIG REQUIRED)` and links
`OpenArm::Can`; it no longer compiles `can/src/openarm_can.c` into its archive.
Its installed config declares the CAN dependency. The isolated installed
transport C11 consumer now requires and links `OpenArm::Transport` and asserts
that both new targets exist.

Control uses `find_package(openarm_model CONFIG REQUIRED)` and links the
installed `OpenArm::Model` target. It no longer adds the model source directory
or creates a second model build.

### Production archives

Commission and control tests link non-installed test variants compiled with
their respective hook definitions. Production targets compile without those
definitions. The unified builder inspects installed archives with `nm` and
fails if a production control/commission hook or a transport-defined CAN symbol
appears.

No public ABI or controller/commission runtime semantics changed. The test
variants use the same sources and build options plus the test-hook definition.

### Prefix validation

`build.sh` and `build_native.sh` reject `:` and `;` in output/build/install
paths before creating a directory. These characters delimit environment or
CMake prefix lists. Paths containing spaces remain supported.

### Integrated header consumers

Strict external C11 and C++17 consumers were added under
`tests/installed_native_consumer`. They include all five installed headers in
opposite orders with `OPENARM_DISABLE_LEGACY_GENERIC_STATUS`, use the module
status types, discover all five packages, link all five `OpenArm::*` targets,
and run only hardware-free calls.

The unified `--tests` path activates them when the installed control config and
the two collision-free headers are present. On this branch it reports the
explicit deferral because the ABI namespace and control export are owned by a
separate integration branch. After that branch is rebased, these consumers
become a required build/run gate automatically rather than needing an
orchestration edit.

## Verification evidence

Clean Release, tests enabled, output path containing spaces:

```bash
./scripts/build.sh --tests \
  --output-root '/tmp/openarmik unified final review' \
  --build-type Release
```

Result: exit 0.

- CAN 1/1, model 4/4, commission 2/2, transport 3/3, and control 3/3
  CTests passed: 13 native drivers total.
- Model generator determinism and the installed transport consumer passed.
- Both ROS packages rebuilt and installed; exactly eight current
  `openarm_ik_ros` tests were registered without executing DDS tests.
- `openarm_model_DIR` in the control cache resolves to the selected install
  prefix. There is no model sub-build under the control build directory.
- `OpenArmTransportTargets.cmake` carries
  `$<LINK_ONLY:OpenArm::Can>`. The installed consumer found and linked
  `OpenArm::Transport` and `OpenArm::Can` from the spaced prefix.
- `find_package` discovery succeeded for both `OpenArmCan` and `openarm_can`.
- Installed control and commission archives contain no test hook symbols.
  Their non-installed test archives contain all expected hook symbols.
- The installed transport archive defines no `oa_can_*` symbols.

Incremental tests-enabled rebuild:

```bash
./scripts/build.sh --incremental --tests \
  --output-root '/tmp/openarmik unified final review' \
  --build-type Release
```

Result: exit 0; the same 13 native tests and eight ROS registrations passed.

Clean tests-off build:

```bash
./scripts/build.sh \
  --output-root '/tmp/openarmik unified final tests off' \
  --build-type Release
```

Result: exit 0; all five native components and both ROS packages installed.
CTest enumeration reported zero for every native component and ROS package,
and installed archive boundary checks passed.

Standalone transport configuration against the installed CAN prefix retains
four registered tests, including the opt-in-capable vcan smoke test. The
unified hardware-free profile registers only its three non-vcan tests.

Colon handling:

```text
./scripts/build.sh --output-root '/tmp/openarmik:final-reject'
exit 2; no output directory created
Output roots containing : or ; are unsupported: /tmp/openarmik:final-reject
```

Final static checks:

```bash
bash -n scripts/build.sh scripts/build_native.sh scripts/test_ros_coverage.sh
git diff --check
```

Both passed.

## Integration dependency and risk

The current branch intentionally does not duplicate the public status-header or
control-package export edits under separate ownership. Therefore the strict
five-library consumers are present but deferred on this branch. The combined
integration must re-run `./scripts/build.sh --tests`; success requires those two
external consumers to configure, build, and execute. No other known build-scope
finding remains.
