# Final fresh-sweep hardening implementation

- Branch: `fix/final-fresh-sweep`
- Base: `53bfd809a7bd81c753fcdd354c1412339b9d2f1a`
- Date: 2026-07-29
- Safety: no GUI, browser, physical interface, CAN transmission, or network
  operation was run. All builds were sequential with one-job limits.

## Implemented

The public builders now authenticate the local `openarm_description` checkout
offline before acquiring mutation locks or changing output. Production constants
hardcode commit `6c7b720f1ba48e8bafa3a3dc752c45f397b42221`, canonical HTTPS origin, and tree
`a05f0116710a4948e9f769237696fbf43701762f`. Validation requires a detached Git
worktree, exact actual tracked bytes, clean index/worktree, no untracked or ignored
additions, and the expected no-submodule policy. Linked Git worktrees are accepted;
symlinked roots are rejected.

Both launchers use one shared freshness/authority implementation. A direct default
launch always requests one bounded incremental build and propagates `--jobs` when
specified. `run.sh` builds once and hands off through `--no-build`. The no-build
path requires an atomic stamp matching dirty tracked production source bytes, the
pinned description commit/tree, build profile, and hashes of every gated artifact.
It then repeats live symbol gates. The ROS session archive must consume Runtime and
must not reference direct Control/plan/manifest APIs; the installed Runtime archive
must not reference CAN, Transport, or socket send/receive APIs. Required setup,
node, portal, close helper, launch, and RViz files are bound into the stamp. A build
removes only the exact old stamp under the existing output lock and publishes a new
one by atomic rename only after successful build, unchanged inputs, pin validation,
and live authority gates.

Runtime ABI V1 now freezes the actual current header, not an obsolete feature draft.
The installed package includes the frozen header and layout constants. Strict C11
and C++17 canaries compile only against it, link to the current archive, run output
validation and the current `oa_runtime_manifest_save` signature, and assert sizes
for all 21 public records plus offsets affected by prior draft edits. A checked hash
policy requires byte identity between current and frozen V1 headers. Breaking future
changes require V2 names, ABI versioning, and compatibility; pre-`987f512` feature
headers are documented as unpublished pre-release drafts.

Portal launcher help now states that its sampled nominal virtual prefilter is not a
verified scene or physical collision certification, and that the controller reports
`collision_checked=false`.

## Verification

- `tests/test_description_pin.sh`: PASS.
- `tests/test_launch_integrity.sh`: PASS with real compiled archives and `nm` gates.
- `tests/test_build_resource_controls.sh`: PASS with sequential real CMake/CTest and
  command shims.
- Sequential native build/tests: PASS for CAN 1/1, model 4/4, commission 2/2,
  transport 3/3, control 4/4, Runtime 6/6, and all four installed native consumers.
- Runtime V1 C11, C++17, header policy, and installed consumer: PASS.
- Real top-level one-job build: all three ROS packages built. A deliberately exposed
  isolated-prefix gate mismatch failed closed and left no stamp; after correcting
  the artifact path, the incremental rerun passed, published the stamp, and passed
  live stamp/symbol validation.
- `run.sh --help`, RViz help, shell syntax, and `git diff --check`: PASS.

## Independent-review follow-up

The follow-up closes the two Runtime ABI and five launch review findings. Runtime
V1 now freezes the exact Commission 0.1.0 header it exposes, asserts numeric
size/alignment/offset contracts for all transitive Commission records, exports
the frozen dependency include order, and enforces exact package resolution.
Frozen and current strict C11/C++17 objects retain correctly typed references to
all 50 public Runtime functions. Both build-tree and installed archives must
exactly match the checked 50-symbol manifest.

Launch stamp V2 binds the canonical output root, complete deterministic install
file/symlink manifest, source entries including untracked/ignored inputs in the
explicit consumed roots, effective build/test/coverage environment, canonical
tool executables and hashes, and ROS prefix/setup identity. Shared leases on the
same output/native/install lock keys are acquired after building and held through
the application lifetime; a no-build launch waits for an active builder and
revalidates under the lease. Sourced ROS package identity must resolve to the
audited isolated prefix. Browser children explicitly close GUI and all shared
build-lock descriptors. `run.sh` now honors last-option build-mode semantics, so
an explicit final `--no-build` invokes no builder.

Follow-up verification after those review changes:

- Standalone Runtime configure/build and CTest: 9/9 PASS. This covers strict
  current and frozen C11/C++17 full-reference consumers, byte-identity header
  policy for Runtime and Commission, the exact 50-symbol archive manifest, and
  all four installed consumers.
- Focused description-pin, launch-integrity, and build-resource suites: PASS.
  The launch suite includes full-install tamper, copied-output-root, dirty and
  ignored source, toolchain, shared-lease wait/contention, and raced-mutation
  fail-closed cases.
- Final incremental top-level build with all job controls set to one: PASS; all
  three ROS packages completed and launch stamp V2 was published. The isolated
  package-prefix check resolved `openarm_ik_ros` to that audited install root.
- Final shell syntax and `git diff --check`: PASS.

## Independent re-review follow-up

The final re-review path issue is closed by capturing and validating Runtime's
absolute frozen V1 include directory immediately after `PACKAGE_INIT`, before
any nested dependency lookup can change `PACKAGE_PREFIX_DIR` on CMake 3.16-3.29.
The installed regression uses split Runtime/dependency prefixes, statically
checks config ordering, validates the resolved frozen include path, and retains
the negative exact Commission 0.1.1 request against installed 0.1.0.

Incremental top-level builds now preserve native and ROS compilation caches but
recreate the single launcher-facing native/ROS install prefix from empty under
the existing exclusive output/native/install locks. Cleanup accepts only four
explicit known output children and must prove the selected tree disappeared;
the stamp is invalidated first, and any failure leaves no publishable stamp.

Source symlinks must resolve to regular files inside the repository and bind
their target mode and bytes. The complete install manifest includes directory
metadata and rejects broken, escaping, or special-file entries; internal links
bind link text and resolved target metadata/content, while physical directory
contents remain covered by the tree manifest. The web portal no longer accepts
an executable override and always executes the stamped binary under the exact
audited package prefix.

Toolchain fingerprinting safely tokenizes command chains without evaluation and
binds the content of every resolved compiler/wrapper token, C and C++ options,
archiver, ranlib, linker, nm, strip, objcopy, generator backend, and the canonical
toolchain-file bytes. Only the aggregate fingerprint is written to the stamp.
Focused regressions mutate fixed-path CC/CXX/linker/backend wrappers and the
toolchain file, exercise internal/broken/escaping links, and prove a removed
tracked launch file is absent after an incremental install refresh.

Re-review verification was fully sequential and hardware-free:

- Runtime reconfigure/build with one-job limits and CTest: 9/9 PASS, including
  split-prefix config ordering/resolution and the Commission version mismatch.
- Description pin, launch integrity, and resource controls: PASS. New cases
  cover CC/CXX/linker/backend and toolchain-file mutation, symlink target and
  special-file rejection, tracked launch deletion, stale-path absence, and
  fresh-install stamp validation.
- The single final real incremental top-level build ran with every job control
  set to one: PASS; three ROS packages finished in 1.80 seconds after the install
  prefix was recreated. Live V2 stamp/authority validation and exact isolated
  `openarm_ik_ros` package-prefix resolution both passed.
- Final shell syntax and `git diff --check`: PASS.
