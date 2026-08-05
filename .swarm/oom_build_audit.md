# OOM/build/launch audit

Date: 2026-07-29  
Scope: `run.sh`, every repository build/test shell entry point, both GUI launchers,
the ROS launch description, and CMake test helpers that recursively build. This
was a read-only code/runtime audit; no project source was changed. The pre-existing
dirty file `transport/tests/test_transport.cpp` was not touched.

## Executive finding

The reported OOMs have a direct, confirmed build-script explanation. This host
exposes 32 CPUs but only 15,107,516 kB (~14.4 GiB) RAM and 4 GiB swap. Native
builds use GNU Make with a bare `-j` (unlimited job slots), and ROS builds use
colcon's 32-package parallel executor while each CMake package also receives
`-j32 -l32`. The launch shortcut then repeats that exposure on every normal
`./run.sh`, even when nothing changed, because its "incremental" native build
deletes and recreates all six native component build trees.

## Evidence gathered

- `nproc` -> `32`.
- `/proc/meminfo` -> `MemTotal: 15107516 kB`, `SwapTotal: 4194300 kB`.
- Every current `ros2_ws/native_build/*/CMakeCache.txt` and
  `ros2_ws/build/*/CMakeCache.txt` reports `CMAKE_GENERATOR=Unix Makefiles`.
- No `MAKEFLAGS`, `CMAKE_BUILD_PARALLEL_LEVEL`, or colcon defaults override was
  present in the audit environment.
- Non-compiling probe:
  `cmake --build ros2_ws/native_build/can --parallel --target help --verbose`
  printed `gmake -f Makefile -j help`. GNU Make's argument-less `-j` has no job
  limit.
- `colcon build --help` on this machine reports default executor `parallel`,
  `--parallel-workers` default `32`.
- Installed colcon code
  `/usr/lib/python3/dist-packages/colcon_cmake/task/cmake/build.py:281-316`
  chooses the CPU count and returns `-j32 -l32` when `MAKEFLAGS` does not already
  contain a job/load limit. The parallel executor independently admits 32 package
  jobs (`colcon_parallel_executor/executor/parallel.py:51-63,143-151`).
- The process snapshot available to this audit ran in an isolated PID namespace,
  so it cannot prove whether host-session Firefox/RViz/ROS processes were still
  alive. Lingering-process conclusions below are therefore based on explicit
  ownership, signal, and wait paths in the launchers, not on a misleading empty
  `ps` sample.

## Prioritized confirmed risks

### P0 — native build parallelism is unbounded

`scripts/build_native.sh:135-169` deletes, configures, builds, tests, and installs
each component. Line 165 invokes:

```text
cmake --build "$component_build" --parallel
```

On the repository's current Unix Makefiles generator this is a bare `gmake -j`,
not "use a modest CPU default." A component with many translation units may run
all available recipes concurrently, with no memory-aware cap. The installed
consumer build repeats the same issue at line 275. This is the clearest direct
OOM trigger.

The same bare `--parallel` appears in lower-frequency paths:

- `tests/test_native_prefix_reuse.sh:103-118` (three clean-first rebuilds),
- `control/tests/test_install_consumer.cmake:25-27`, and
- `transport/tests/test_install_consumer.cmake:22-24`.

The latter two consumers are small, but they violate the same global resource
contract and can run inside a test build.

### P0 — ROS package parallelism is multiplied by per-package `-j32`

`scripts/build.sh:147-159` calls `colcon build` without `--executor` or
`--parallel-workers`, and without a bounded `MAKEFLAGS`. On this installation:

1. colcon can schedule up to 32 packages concurrently; and
2. every ready CMake package gets its own `-j32 -l32` build.

This workspace selects three packages. Dependency ordering limits the actual
package fan-out, but `openarm_description` and `openarm_control_msgs` can be ready
together, so the scripts permit up to 64 make job slots at that stage; a later
single package still permits 32 compiler/linker jobs. CPU-count parallelism is
unsafe on a 32-core / 14.4-GiB machine.

### P1 — `./run.sh` rebuilds on every launch, and native "incremental" is full

- `run.sh:12` always prepends `--build` (a later user-supplied `--no-build` can
  override it, but the normal command always builds).
- `scripts/launch_web_portal.sh:143-148` maps that to
  `scripts/build.sh --incremental`.
- `scripts/build.sh:113-118` correctly preserves top-level directories in
  incremental mode, but `scripts/build_native.sh:135-140` unconditionally runs
  `cmake -E remove_directory "$component_build"` for every one of `can`, `model`,
  `commission`, `transport`, `control`, and `runtime`. All six are configured and
  recompiled from scratch on every `./run.sh`.
- `scripts/build.sh:153` also always supplies `--cmake-clean-cache`; installed
  colcon removes each ROS `CMakeCache.txt` and forces configuration even in the
  launcher's incremental path.

Thus a simple GUI restart repeats the two P0 exposures. This also makes a prior,
still-open Firefox session consume RAM during the next forced build.

### P1 — builds and the two GUI modes do not share a global lock

The portal lock is acquired before its build (`launch_web_portal.sh:129-150`), so
two portal launches are serialized. However:

- `scripts/build.sh` itself has no per-output-root lock, so direct builds can run
  concurrently with each other or with a portal-triggered build against the same
  trees.
- `scripts/launch_rviz.sh:5-10` locks `openarmik-rviz-$UID.lock`, while
  `scripts/launch_web_portal.sh:129-134` locks `openarmik-portal-$UID.lock`.
  Both GUI modes may therefore run together, duplicating RViz and the ROS core
  processes. The README's single-instance claim only holds within one mode.

Concurrent direct builds multiply memory load and can also race over build/cache
files. Concurrent GUI modes add a substantial fixed RAM load before any separate
build is started.

### P2 — normal cleanup is strong; abrupt-parent cleanup is incomplete

Positive evidence:

- `launch_web_portal.sh:279-305` owns exact portal, RViz, and ROS launcher PIDs,
  starts each in a separate session/process group, terminates the entire group,
  escalates to KILL on bounded timeouts, and `wait`s each child.
- `launch_rviz.sh:121-149` follows the same bounded RViz-then-core shutdown.
- `wait -n` in each launcher causes any tracked child exit to tear down the rest.

Remaining gaps:

- Only `INT`, `TERM`, and `EXIT` are trapped
  (`launch_web_portal.sh:308-310`, `launch_rviz.sh:152-154`). There is no explicit
  `HUP` trap. `SIGKILL`/OOM death is inherently untrappable. Because children are
  deliberately detached with `setsid`, they are not coupled to the launcher's
  terminal session and can survive abrupt launcher death.
- The opened browser has no stored PID and is not part of `shutdown`
  (`launch_web_portal.sh:361-368`). Keeping a user's Firefox open may be intended,
  but the launcher does not reclaim that memory before a future forced build.

These are amplification/recovery risks, not the primary first-run build OOM
cause. The per-mode lock descriptor is inherited by ordinary child processes,
which may prevent a same-mode duplicate while an orphan remains, but it neither
reaps the orphan nor blocks the other GUI mode/direct builds.

## GUI startup sequencing

The portal sequencing is mostly sound and is not an OOM cause:

- Core ROS and RViz start concurrently (`launch_web_portal.sh:312-317`).
- RViz process identity is established before portal start (`319-338`).
- The portal health endpoint returns success only when the exact launcher-owned
  RViz process has exactly one mapped top-level window
  (`openarm_portal.cpp:563-575`, `rviz_capture.cpp:119-136`).
- The launcher polls that endpoint for up to 20 seconds and only then opens the
  browser (`launch_web_portal.sh:340-368`).

The health gate does not require fresh joint state, diagnostics, or action-server
readiness. The browser can therefore open before the ROS application is fully
ready, but `openarm_portal.cpp:155-186,189-204` reports state as unavailable and
the page disables motion until state is fresh. This is a fail-closed UX race, not
an unsafe or memory-amplifying race.

The standalone RViz launcher starts ROS then RViz immediately
(`launch_rviz.sh:158-162`) without a readiness probe. ROS discovery tolerates
this, and either process exiting triggers cleanup. No redundant RViz is launched
inside the Python launch because both shell launchers pass `rviz:=false`.

## Smallest safe fix design

Apply these changes as one resource-control fix, in order:

1. **One explicit job budget.** Add a validated positive setting such as
   `OPENARM_BUILD_JOBS`, with a conservative default of `2` for this project/host.
   Pass it as `cmake --build ... --parallel "$jobs"` everywhere, including the
   prefix-reuse and CMake consumer helpers.
2. **Prevent nested colcon oversubscription.** Run the three-package build with
   `--executor sequential` and a locally scoped `MAKEFLAGS=-j$jobs -l$jobs`.
   `--parallel-workers` alone is insufficient because it controls packages, not
   the `-j32` inner make invocations. Sequential packages plus bounded MAKEFLAGS
   provides a real global compile ceiling. Preserve a caller's unrelated
   MAKEFLAGS only if its job/load limits can be validated; otherwise an explicit
   project budget should win for reliability.
3. **Make launch truly incremental.** Remove the unconditional component
   `remove_directory` calls when `build.sh --incremental` is requested, and omit
   `--cmake-clean-cache` in that mode. The smallest interface-preserving approach
   is a `--reuse-build-trees` flag passed from `build.sh` to `build_native.sh`;
   keep today's clean behavior as the default for explicit clean builds.
4. **Stop rebuilding on an ordinary run.** Remove hard-coded `--build` from
   `run.sh`; the portal launcher's existing `auto` mode already builds when the
   install setup or portal binary is absent. Users retain explicit `./run.sh
   --build` when desired.
5. **Serialize heavy work.** Add a per-output-root build lock inside `build.sh`,
   held across cleanup/configure/build/install. Use one shared GUI-runtime lock
   for both RViz-only and portal modes, or explicitly reject the other mode before
   startup.
6. **Harden abnormal shutdown without changing normal behavior.** Trap `HUP` in
   both launchers. A complete SIGKILL/OOM orphan solution needs a real supervisor
   or parent-death-signal wrapper; do not replace the precise process-group logic
   with broad `pkill`. Browser ownership should remain opt-in: defaulting
   `run.sh` to no forced rebuild is safer than killing an existing user Firefox.

The essential OOM fix is items 1-4. Items 5-6 prevent concurrency and orphaned
GUI processes from recreating memory pressure around that bounded build.
