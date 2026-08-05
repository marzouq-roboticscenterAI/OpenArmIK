# Independent OOM/build design review

Date: 2026-07-29  
Scope: adversarial review of `.swarm/oom_build_audit.md`, `run.sh`,
`scripts/build.sh`, `scripts/build_native.sh`, `scripts/launch_web_portal.sh`, and
the directly relevant RViz launcher and recursive build helpers. No project
source or pre-existing dirty file was changed.

## Verdict

The audit found a real and important resource-control defect, but it overstates
what the available evidence proves and its item 4 is a correctness regression.
The host and command probes prove that the scripts *permit* excessive build
parallelism; they do not prove which process the kernel killed in either reported
OOM. The smallest robust fix is:

1. impose one validated build-job budget, default `2`, on native, nested-test,
   and ROS builds;
2. make colcon package execution sequential so the budget is global rather than
   per package;
3. make the existing forced launch build actually incremental; and
4. **retain** `run.sh`'s `--build`, so a source edit cannot silently launch a
   stale installed binary.

Lock and signal changes are useful hardening but are not needed to establish the
primary OOM ceiling. They should not be allowed to enlarge or delay that patch.

## Confirmed findings

### 1. Bare native `--parallel` is unbounded on this tree

Confirmed. The current generator is Unix Makefiles and the non-compiling probe

```text
cmake --build ros2_ws/native_build/can --parallel --target help --verbose
```

executes `gmake -f Makefile -j help`. An argument-less GNU Make `-j` has no job
slot limit. This occurs at `scripts/build_native.sh:165` and `:275`. It also
occurs in `tests/test_native_prefix_reuse.sh` and in the control and transport
installed-consumer CMake scripts. The latter scripts run below `ctest`, so
limiting only the top-level native command is not a complete resource contract.

The practical fan-out is smaller than “all 32 CPUs” when a component has fewer
ready recipes, but absence of a ceiling is still a bug.

### 2. Colcon and its per-package builds have independent parallelism

Confirmed. On this installation, colcon's parallel executor admits 32 packages.
`colcon_cmake` independently returns `-j32 -l32` when `MAKEFLAGS` has no jobs or
load limit. A direct probe of `_get_make_arguments` returned:

```text
{}                         -> ['-j32', '-l32']
{'MAKEFLAGS':'-j2 -l2'}    -> []
```

Therefore `--parallel-workers` alone cannot cap compiler jobs. For the current
three-package graph, `openarm_description` and `openarm_control_msgs` can be
scheduled together, then `openarm_ik_ros` follows its dependencies. The audit's
“64 make job slots” is a scheduler upper bound, not evidence of 64 simultaneous
compiler processes: the description package is mostly install/configuration,
the current message package has 23 object files, and `openarm_ik_ros` has eight.
That correction does not make `-j32` safe on a 14.4-GiB host.

### 3. The launcher's “incremental” native build is currently clean

Confirmed. `build.sh --incremental` preserves the top-level directories, but
`build_native.sh` removes each component build directory before configuring it.
Every normal `run.sh` therefore recompiles all native targets. This is a much
more credible repeated-exposure mechanism than `--cmake-clean-cache`.

The full directory removal was deliberately introduced as a CMake 3.16-compatible
replacement for `cmake --fresh`. Reuse must consequently be an explicit mode;
blindly deleting line 139 and changing the standalone script's default would
discard its established clean-refresh behavior. The audit's proposed
`--reuse-build-trees` interface is the safer design. The existing
`tests/test_native_prefix_reuse.sh` must exercise that new mode, particularly on
its second prefix, rather than merely proving the old clean path.

### 4. Colcon cache cleaning is unnecessary in incremental mode

Confirmed with a qualification. `--cmake-clean-cache` deletes only
`CMakeCache.txt` and forces configure; it does **not** by itself delete object
files or force a full recompile. Omitting it in incremental mode is correct:
colcon reconfigures when its recorded CMake arguments change, and the generated
build system re-runs CMake when CMake inputs change. Clean mode already deletes
the entire ROS build directory, so the option is redundant there but harmless.

### 5. Locking is incomplete, but the proposed scope needs precision

Confirmed that two `scripts/build.sh` processes can mutate the same canonical
output root concurrently. A lock must be acquired after safely creating the
output root and before *any* cleanup. Canonicalizing the root before deriving the
lock identity is necessary.

A per-output-root lock prevents cache/install corruption; it is not a global OOM
lock, because builds using different output roots can still overlap. It also
does not coordinate a direct `build_native.sh` invocation unless both entry
points deliberately share a lock identity. Adding a lock only to `build.sh`
must not be described as serializing every build entry point.

The portal and standalone RViz launchers do use different per-user lock files,
so they can run together. A shared GUI lock would prevent duplicated ROS/RViz
stacks, but this is runtime hardening rather than the primary build fix. If it is
done, both launchers must use the portal launcher's validated private runtime
directory rules; changing only the filename leaves `launch_rviz.sh`'s weaker
runtime-path handling intact.

### 6. Normal RViz/ROS cleanup is sound; abrupt-parent cleanup is bounded

Confirmed. The launchers retain exact child PIDs, put each owned service in its
own session/process group, terminate groups in a bounded order, and reap their
direct children. `wait -n` makes an owned child exit tear down its siblings.

Adding `HUP` to both the installed traps and the trap reset inside `shutdown` is
a small valid improvement. It cannot solve `SIGKILL` or OOM-killed-parent
orphans. Because the `setsid` children inherit the launcher lock descriptor, a
surviving orphan will normally continue to block a duplicate using that same
lock; that prevents multiplication but does not reap the orphan. A real
parent-death guarantee would require a supervisor/PDEATHSIG wrapper and is not
part of the smallest OOM fix.

Firefox is intentionally not an owned service. `firefox URL` may forward to an
existing instance, and killing “Firefox” on shutdown risks destroying unrelated
user tabs. Do not add broad `pkill`, and do not make browser termination part of
acceptance. Reducing forced recompilation is the appropriate way to avoid
combining an existing browser's fixed memory with an unnecessarily clean build.

## Refuted or narrowed claims

1. **“Direct, confirmed explanation” is too strong.** The unsafe limits and host
   memory are confirmed; actual OOM causality and victim selection were not
   captured from the host kernel log/process history. Treat this as the leading,
   code-confirmed mechanism, not a post-mortem proof.
2. **Remove `--build` from `run.sh`: refuted.** Portal `auto` mode checks only
   whether setup/binary artifacts exist. Once they exist, it has no source
   freshness test. Removing `--build` would make ordinary development launches
   silently run stale code. Keep the forced *incremental* build; users already
   have `./run.sh --no-build` when they knowingly want no build.
3. **`--cmake-clean-cache` causes the repeated full ROS compile: narrowed.** It
   forces configure, not clean compilation. It should still be omitted for a
   true incremental path.
4. **One per-output-root lock solves memory concurrency: refuted as stated.** It
   protects a tree, not host memory, and does not automatically cover standalone
   `build_native.sh`. The job ceiling is the OOM guarantee.
5. **`MAKEFLAGS` alone is a portable universal budget: narrowed.** It is correct
   for the current Unix Makefiles generator and suppresses colcon's injected
   `-j32`. Also exporting `CMAKE_BUILD_PARALLEL_LEVEL=$jobs` is the small,
   generator-neutral way to cover nested bare `cmake --build --parallel` calls.

## Exact implementation design

### Required OOM/incremental patch

1. In both build entry points, read `OPENARM_BUILD_JOBS` with default `2` and
   reject anything not matching `^[1-9][0-9]*$` with exit status 2 before cleanup
   or configuration. `build.sh` passes the resolved value explicitly to
   `build_native.sh` (a `--jobs N` argument is clearer than reparsing ambient
   state). `build_native.sh` also supports its standalone default/environment.
2. Export both `OPENARM_BUILD_JOBS=$jobs` and
   `CMAKE_BUILD_PARALLEL_LEVEL=$jobs` in `build_native.sh` before any build or
   CTest, and use `cmake --build ... --parallel "$jobs"` for its two direct
   builds. This export bounds recursive CMake consumer builds in the normal
   path. Also make the two installed-consumer CMake helpers read and validate
   `OPENARM_BUILD_JOBS` (default `2`) and pass an explicit `--parallel N`, so a
   developer running their CTest directly cannot recover bare `-j`. Update the
   standalone prefix-reuse shell test's three direct build calls to use the same
   validated budget; do not leave executable bare `--parallel` paths.
3. Invoke colcon with `--executor sequential` and a command-scoped
   `MAKEFLAGS=-j$jobs`. Also provide command-scoped
   `CMAKE_BUILD_PARALLEL_LEVEL=$jobs`; this covers a future non-Make CMake
   generator while `MAKEFLAGS` prevents the current colcon plugin from adding
   `-j32 -l32`. Deliberately replace, rather than append, a caller's job flags.
   A load-average limit is optional and should not be confused with the memory
   ceiling; `-j$jobs` is the actual bound.
4. Add `--reuse-build-trees` to `build_native.sh`. In that mode, skip component
   and installed-consumer directory removal but still run `cmake -S ... -B ...`
   and all existing cache assertions. `build.sh` passes it only for
   `--incremental`; clean/default builds retain the present clean behavior.
5. Do not pass `--cmake-clean-cache` on the incremental colcon path. It may be
   retained conditionally for the clean path, although clean mode has already
   removed the cache.
6. Keep `run.sh`'s argument ordering and `--build`. A later user `--no-build`
   must continue to override it.

### Small, separable hardening

1. Add a nonblocking canonical-output-root lock to `build.sh`, acquired before
   cleanup. The error must identify the root already being built. Either scope
   the claim/test to `build.sh`, or design an explicitly shared lock protocol
   with standalone `build_native.sh`; do not imply coordination that is absent.
2. Use one validated per-user GUI lock in both launchers if simultaneous portal
   and RViz modes are unsupported.
3. Trap `HUP` in both GUI launchers and clear `HUP` in `shutdown` with the other
   traps. Preserve exact-PID/process-group shutdown. Leave Firefox unmanaged.

## Verification acceptance criteria

All required criteria below must pass on a clean temporary output root without
touching the user's existing `ros2_ws` products.

### Static and argument validation

- `bash -n` passes for every changed shell script.
- `OPENARM_BUILD_JOBS=0`, `-1`, `1x`, and an empty explicit `--jobs` value fail
  with status 2 before any output-tree deletion. Values `1` and `2` are accepted.
- Repository executable build/test paths contain no argument-less
  `cmake --build ... --parallel`; native, prefix-reuse, and installed-consumer
  helper calls all show an explicit validated number.
- `run.sh` still resolves ordinary invocation to forced incremental build and a
  later `--no-build` still wins.

### Resource ceiling

- A clean `OPENARM_BUILD_JOBS=2 scripts/build.sh --tests --output-root <tmp>`
  completes successfully.
- Native verbose command evidence contains `-j2`, never bare `-j` or `-j32`.
- Colcon command/event logs show the sequential executor, no overlapping package
  build intervals, and no injected `-j32/-l32`. Generated Make invocations run
  with an effective two-job MAKEFLAGS/jobserver.
- During the build, sampled process-tree evidence never shows more than two
  compiler processes owned by this build. Recursive installed-consumer tests are
  included in this observation.
- `OPENARM_BUILD_JOBS=1` repeats the same proof with at most one compiler.

### Clean and incremental correctness

- A default clean build removes marker files placed under that temporary root's
  `native_build`, `build`, `install`, and `log` children before rebuilding.
- An immediate `--incremental` rebuild retains all six native component build
  directories and their CMake caches, does not recompile or relink unchanged
  native/ROS targets, and succeeds without `--cmake-clean-cache`.
- In an isolated disposable worktree, changing one native source causes its
  target and required downstream links to update; changing a relevant
  `CMakeLists.txt` causes automatic reconfiguration; unchanged independent
  components remain untouched.
- `tests/test_native_prefix_reuse.sh <empty-tmp>` runs its second build with
  `--reuse-build-trees` and passes every cache/link assertion from prefix A to
  prefix B. This is the regression proof that reuse does not retain the old
  dependency prefix.
- Switching build type and tests flags across an incremental build updates cache
  values and test registration as requested. A clean build remains the supported
  remedy for removed/renamed install artifacts, as with conventional CMake
  incremental installs.

### Locks, only if included

- Two `build.sh` processes using the same canonical output root cannot pass the
  lock concurrently, including when one path uses `..`/symlink aliases; the loser
  exits clearly before cleanup. Builds using different roots are not represented
  as globally serialized.
- The test separately establishes the documented behavior of a direct
  `build_native.sh` collision; it may not infer this from the `build.sh` test.
- Starting either GUI mode while the other owns the shared GUI lock exits before
  launching ROS/RViz. A launcher killed by `HUP` exits and all exact recorded
  child PIDs/process groups disappear within the existing bounded shutdown
  window. `INT` and `TERM` behavior remain passing.
- Closing the OpenArm launcher does not kill a pre-existing Firefox instance or
  unrelated tabs. No broad name-based process kill is introduced.

## Final recommendation

Ship the required six-point patch first. It removes the unbounded/nested job
fan-out without sacrificing source freshness and turns repeated `run.sh` builds
into cheap no-op incrementals. Add same-tree build locking and shared GUI/HUP
hardening only with their narrower claims and explicit lifecycle tests. Do not
remove `--build`, do not kill Firefox, and do not attempt a speculative process
supervisor as part of this OOM correction.
