# Independent unified-build review

Reviewed commit: `16906a02aa555775aec7255f4d720e2ee092f6ed`

Parent: `514e3a8`

Date: 2026-07-29 (America/Los_Angeles)

Verdict: **FINDINGS**

## Findings

### HIGH — The installed five-library SDK cannot be consumed as one header set

The install places both `openarm_model.h` and `openarm_control.h` in the same
include directory, but they still define the same generic API names with
incompatible types. `model/include/openarm_model.h:25,34-35` defines signed
`oa_status`, `OA_OK`, and `OA_EINVAL`; `control/include/openarm_control.h:19,33-34`
defines the same names using unsigned values.

Exact strict installed-header checks failed in both include orders:

```text
cc -std=c11 -Wall -Wextra -Wpedantic -Werror \
  -I"/tmp/openarmik reviewer fresh/install/include" \
  -include openarm_can.h -include openarm_model.h \
  -include openarm_commission.h -include openarm_transport.h \
  -include openarm_control.h -x c -c /dev/null

openarm_control.h:19:18: error: conflicting types for 'oa_status'
openarm_model.h:25:17: note: previous declaration ... {aka 'int'}
openarm_control.h:33:9: error: 'OA_OK' redefined [-Werror]
```

Reversing all five headers fails with the reciprocal conflict. Each header
alone compiles cleanly under the same strict flags, so this specifically breaks
the required integrated external-consumer surface. This is also the exact ABI
collision Stage 0 of `.swarm/ros_design_synthesis.md` says must be corrected
before the unified build is accepted.

### HIGH — CAN and control are installed without CMake packages or dependency exports

The fresh prefix contains all five archives and headers, but only model,
commission, and transport install config packages. `can/CMakeLists.txt:36-40`
and `control/CMakeLists.txt:53-57` install bare archives/headers without an
export set, config file, version file, namespace target, or dependency metadata.

Exact discovery from the clean installed prefix:

```text
openarm_model      found
openarm_commission found
openarm_can        not found
openarm_control    not found
```

The available packages are also inconsistent: `openarm_model::openarm_model`,
`OpenArm::Commission`, and `OpenArm::openarm_transport`; there is no consistent
`OpenArm::Can`, `OpenArm::Model`, `OpenArm::Transport`,
`OpenArm::Commission`, `OpenArm::Control` graph. A strict control consumer only
worked by manually specifying undocumented transitive link details:

```text
c++ consumer.o -L<prefix>/lib -lopenarm_control -lopenarm_model -lm
```

Thus the script is a sequential builder/installer, not the exported native
build graph required by the synthesis and its external-consumer acceptance
test.

### MEDIUM — The claimed dependency order does not create dependency linkage

`scripts/build_native.sh:124-151` builds components sequentially, but control
does not discover the just-installed model: `control/CMakeLists.txt:21-36`
adds and compiles `../model` again. Transport likewise does not link the
just-built CAN target/package: `transport/CMakeLists.txt:67-95` compiles
`../can/src/openarm_can.c` into its own archive.

Fresh-build evidence showed a second model compile under
`native_build/control/openarm_model`, and both installed archives define the
same codec symbols; for example `nm -g --defined-only` reports
`oa_can_decode_feedback`, `oa_can_encode_mit`, and the fake-transport symbols in
both `libopenarm_can.a` and `libopenarm_transport.a`. This preserves the
previous divergent duplicate-code boundary and means building CAN before
transport and model before control does not validate package dependency
discovery or prevent implementation drift.

### MEDIUM — Installed Release archives still expose test-injection controls

The clean Release install contains production-reachable test hooks:

```text
libopenarm_control.a:
  oa_control_test_active_controller_count
  oa_control_test_active_manifest_count
  oa_control_test_active_plan_count
  oa_control_test_fail_controller_create_after

libopenarm_commission.a (nm -gC):
  openarm::commission::test::active_handle_count()
  openarm::commission::test::fail_next_allocation()
  openarm::commission::test::throw_next_exception()
  openarm::commission::test::exhaust_handle_tokens()
```

The unified entry point unconditionally installs those archives. This fails the
design's Release-install requirement that injection/registry hooks be compiled
only into test objects.

### LOW — A valid output path containing `:` builds native code and then fails ROS prefix discovery

`--output-root` accepts arbitrary absolute paths, but `scripts/build.sh:130`
serializes the path into the colon-delimited `CMAKE_PREFIX_PATH` environment
variable. A clean invocation using `/tmp/openarmik:colon-review` built and
installed all native libraries, then exited 1 while configuring
`openarm_ik_ros` because it could not find the already-installed
`openarm_descriptionConfig.cmake`. Either reject `:` during argument
validation or use explicit CMake/colcon prefix mechanisms that do not split the
selected path. Ordinary paths containing spaces did work.

## Passing evidence

- `git diff --check` and `bash -n` passed.
- A clean Release build at `/tmp/openarmik reviewer fresh` (path includes
  spaces) passed: CAN 1/1, model 4/4, commission 2/2, transport 3/3, and control
  3/3 native tests; both ROS packages built; exactly eight current ROS tests
  were registered.
- Five hardware/network/GUI-free ROS tests were executed separately and passed:
  paired transaction, generated URDF, and the three close-helper argument
  tests. The DDS-launching ROS tests were intentionally not run.
- A repeated `--incremental --tests` build passed the same 13 native tests and
  eight-test registration check.
- A clean Debug tests-off build passed, recorded Debug for all five native
  components and ROS, and registered zero tests. Repeating that clean build
  removed planted stale files from both native-build and install directories.
- Strict installed single-component C11 consumers for model, commission, and
  control built and ran; the installed transport CMake-package consumer built
  and ran from a path containing spaces.
- Help, missing-value, unknown-option, relative native-root, and invalid
  build-type behavior returned the expected 0/2 statuses.
- Cleanup rejected `/`, `/tmp`, `/home`, the repository root, and a symlinked
  install child escaping the output root; the external sentinel was preserved.
- The scripts contain no `sudo`, interface mutation, or vcan invocation. The
  unified transport profile registered only its three hardware-free tests.
- Sanitizer controls are not exposed by these entry points; all component
  sanitizer options are explicitly forced off, so no unified sanitizer claim
  was tested.

## Disposition

**FINDINGS.** The orchestration itself is reproducible for ordinary paths and
its cleanup/test-selection behavior held up, but the result is not a usable
unified installed SDK until the two high-severity header/export failures are
fixed. The duplicate dependency builds and installed test hooks should be
closed at the same packaging boundary.
