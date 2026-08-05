# Fresh final sweep

- Target: `53bfd809a7bd81c753fcdd354c1412339b9d2f1a`
- Date: 2026-07-29 (America/Los_Angeles)
- Verdict: **FINDINGS**
- Critical: **none found**
- Important: **3**

This was a fresh, read-only source/history/caller/artifact audit. The prior
combined verification was treated as evidence, not as a conclusion. No GUI,
ROS process, network access, build, CAN interface, or transmit path was run.
The pre-existing modified `transport/tests/test_transport.cpp` and existing
untracked/ignored files were not changed.

## Important 1: the declared Runtime ABI V1 is not binary/source compatible

`runtime/include/openarm_runtime.h:14` still declares
`OA_RUNTIME_ABI_VERSION == 1`, but public V1 records and an existing V1 function
were changed incompatibly after their introduction:

- `6c89618` introduced `oa_runtime_manifest_save(manifest, absolute_path,
  persistence_authority)` and the original V1 record layouts.
- `3a02c5b` changed the same symbol to
  `oa_runtime_manifest_save(manifest, authority, file_name)` and inserted fields
  into existing output records.
- `4790241` enlarged the existing joint request and plan-report records.
- `df8803c` inserted `checkpoint_authorized` into the existing
  `oa_runtime_manifest_summary` layout (`openarm_runtime.h:311-325`).

This is not merely a header-history concern. The current generic validator at
`runtime/src/runtime_internal.hpp:204-208` requires
`record->struct_size >= sizeof(current T)`. Consequently, a consumer compiled
against an earlier Runtime ABI V1 header passes a smaller, still-V1 record and
is rejected with `OA_RUNTIME_EABI`; the implementation does not copy/fill the
recognized prefix as the Control compatibility code does. The changed
`oa_runtime_manifest_save` symbol cannot service the original V1 calling
contract at all. There is no frozen original-runtime-header ABI consumer/canary
comparable to Control's `v1_original_abi_consumer`.

Impact: the advertised stable ISO-C ABI breaks already-compiled and
source-compatible V1 consumers without a major/package/ABI-version change.
Current-header C11 tests only prove that the newest header and newest archive
agree with one another.

Required fix: either restore the original V1 layouts/signature and add
size-aware prefix normalization/output for all extended records, or publish a
new ABI/major version and retain compatibility symbols for V1. Add a consumer
built from the frozen original Runtime V1 header and execute it against the
current installed library.

## Important 2: launchers can silently run stale products that bypass Runtime

The source build now audits the freshly built session archive at
`scripts/build.sh:206-219`, requiring `oa_runtime_create` and rejecting direct
`oa_controller_*`, `oa_motion_plan_*`, and `oa_manifest_*` references. That
audit is only reached when a build occurs.

The default web-launch `auto` decision at
`scripts/launch_web_portal.sh:141-151,192-197` rebuilds only when
`install/setup.bash` or the portal executable is absent. It does not compare a
source revision/content manifest, validate the installed session authority, or
even compare timestamps. `scripts/launch_rviz.sh:29-30,102-112` likewise
blindly sources the default ignored install tree. (`run.sh` is safer because it
explicitly supplies `--build`, but the public launchers remain affected.)

The current workspace demonstrates the defect without launching anything:

- `ros2_ws/build/openarm_ik_ros/libopenarm_virtual_control_session.a` is dated
  15:56/17:50, before the 19:11 Runtime-session migration source, and `nm -u`
  shows direct `oa_controller_*`, `oa_motion_plan_*`, and `oa_manifest_*`
  references with no `oa_runtime_create`.
- `ros2_ws/install/lib/libopenarm_runtime.a` is dated 17:50, before the 18:25
  physical-query shutdown, and `nm -u` still shows `oa_can_*` and
  `oa_transport_*` references.
- Both required installed executables/setup files exist, so an ordinary
  `scripts/launch_web_portal.sh` invocation takes neither rebuild branch and
  launches that stale stack. The standalone RViz launcher does the same.

No CAN transmission was performed in this audit, and the observed stale ROS
node is virtual-only. Nevertheless, the shipped default launch path can violate
the product's sole-Runtime-authority contract and can evade later fail-closed
fixes merely because ignored products exist.

Required fix: install an authenticated/source-content build manifest and make
`auto` rebuild when it does not exactly match the current sources and pinned
inputs. In `--no-build` mode, fail closed on a missing/mismatched manifest.
Before every launch, independently enforce the session undefined-symbol audit
and the no-CAN Runtime audit (or an equivalent signed artifact identity), not
only after builds.

## Important 3: “pinned” build inputs are selected by path but not authenticated

Both top-level builders call `upstream/openarm_description` pinned, but validate
only the existence of `package.xml` (`scripts/build.sh:105-109` and
`scripts/build_native.sh:126-130`). The checkout is then passed directly to
model testing and colcon (`scripts/build.sh:184-188`). Neither builder verifies:

- the canonical origin;
- exact HEAD `6c7b720f1ba48e8bafa3a3dc752c45f397b42221`;
- detached state; or
- a clean worktree/submodule state.

The resource-control test's claimed “pinned fixture” creates only an empty
`package.xml` (`tests/test_build_resource_controls.sh:293-304`) and therefore
does not test pin identity. The model generator's source hash is useful, but it
runs only in the `--tests` profile; ordinary Release builds and the ROS
`openarm_description` package have no such gate.

The checkout present during this audit did happen to be clean, canonical, and
at the expected commit. That does not close the fail-open build behavior: a
locally modified or substituted checkout with a plausible `package.xml` is
accepted as the supposedly pinned production input.

Required fix: perform a pre-mutation validation of origin, exact commit, clean
tracked/untracked/submodule state, and expected content digest (preferably a
checked-in source manifest), and cover rejection of wrong-HEAD, dirty, and
non-Git fixtures in the lightweight resource-control test.

## Additional documentation defect

`scripts/launch_web_portal.sh:30-32` still says motion waits for an installed
controller-reported “verified collision scene.” The actual source deliberately
requires and reports `collision_checked=false`; portal eligibility uses a
separate 17-sample nominal virtual guard. This text was already identified in
the earlier whole-branch review and remains inaccurate. It should use the same
truthful sampled-guard/non-certification language as the page and package
README.

## Areas found clean in current source

- The current Runtime source has no CAN/Transport dependency; physical motor
  inventory query clears its output and returns `OA_RUNTIME_EUNSUPPORTED`, and
  physical configuration apply remains unsupported.
- Current ROS source owns state/planning/execution through
  `OpenArm::Runtime`; the portal submits the paired action and has no separate
  control authority.
- Virtual joint motion, paired measured-feedback motion, virtual inventory,
  kinematics, and virtual calibration facade entry points exist. Physical
  motion/configuration/calibration capabilities remain absent.
- Portal UI/package documentation clearly labels the sampled nominal guard,
  free orientation, unchecked collision status, simulation-only calibration,
  and non-safety-rated software stop, aside from the launcher-help defect above.
- The new build supervisor bounds job propagation, serializes mutation roots,
  forwards HUP/INT/TERM to its owned process group, closes lock descriptors in
  children, and has focused lightweight regression coverage. No new production
  background process or Python portal/controller implementation was found.

