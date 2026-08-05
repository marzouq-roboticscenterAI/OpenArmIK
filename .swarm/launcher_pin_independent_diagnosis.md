# Independent launcher freshness and description-pin diagnosis

- Audited checkout: `main` at `53bfd809a7bd81c753fcdd354c1412339b9d2f1a`
- Date: 2026-07-29 (America/Los_Angeles)
- Scope: read-only source, Git, launcher, and existing-artifact inspection; no
  build, GUI, ROS process, CAN operation, or network access
- Verdict: **confirmed Important/high finding; not demonstrated Critical**

The default ignored install is demonstrably older than two fail-closed source
changes, and both direct public launchers accept it.  The description checkout
present now is the intended clean canonical checkout, but that is accidental
from the builders' point of view: both production builders accept any directory
containing `package.xml` as the supposedly pinned input.

## 1. Existing default products are stale in the relevant ways

The timestamps are useful ordering evidence but are not the authority.  The
symbol graphs give the decisive result.

| Object | Existing time | Newer source | Decisive observation |
|---|---:|---:|---|
| `ros2_ws/build/openarm_ik_ros/libopenarm_virtual_control_session.a` | 15:56:14 | Runtime-session sources 19:11:30 | `nm -u` has `oa_controller_*`, `oa_motion_plan_*`, and `oa_manifest_*`; it has no `oa_runtime_create` |
| installed `openarm_ik_ros_node` | 17:50:55 | node/session sources 19:11:30 | 45 `oa_controller_` symbol records, including `oa_controller_create`; zero `oa_runtime_`, `oa_can_`, and `oa_transport_` records |
| `ros2_ws/install/lib/libopenarm_runtime.a` | 17:50:51 | physical-query shutdown sources 18:25:35 | `nm -A -u` attributes `oa_can_*` and `oa_transport_*` references to `inventory.cpp.o` |

In contrast, current `virtual_control_session.cpp` creates and owns the Runtime
facade (`oa_runtime_manifest_create_virtual`, `oa_runtime_create`, capability,
inventory, arm, plan, execute, heartbeat, event, stop, and destroy calls).
Current Runtime source contains no `oa_can_` or `oa_transport_` reference.  The
post-build audits in `scripts/build.sh:206-219` and
`scripts/lib/build_native_body.sh:116-145` encode those same intended
properties, but only execute if a build is actually performed.

After sourcing the default install, `ldd` resolves the workspace's ROS message
DSOs and reports no dynamic native OpenArm control/runtime library.  Thus the
native authority code is statically carried by the node.  `strings` likewise
shows the installed node's controller implementation and controller-facing
text, but no Runtime-facade identity.  `ldd` and `strings` are corroboration;
they cannot replace the `nm` audits for this static link graph.

Important nuance: this exact installed node is the old virtual direct-controller
node.  It contains no CAN/Transport symbols, so the evidence does **not** show
that launching this particular node would open CAN.  The separately installed
Runtime archive is CAN-capable and stale, however.  Because the launchers admit
arbitrary intermediate install states without a gate, a node rebuilt after the
Runtime migration but against that pre-shutdown Runtime could statically carry
the forbidden CAN path.  The demonstrated result is therefore:

1. the current default direct launch runs pre-migration controller authority;
2. the same acceptance defect can admit a stale CAN-capable Runtime stack; and
3. no CAN access or transmission was demonstrated or attempted here.

## 2. Launcher trace

### `run.sh`

`run.sh:12` executes `launch_web_portal.sh --build`, so this wrapper asks for an
incremental build.  A successful build reaches the current symbol gates.  This
wrapper is safer, but it does not make the two public scripts safe when invoked
directly, and it currently has no independent post-build launch gate.

### `scripts/launch_web_portal.sh`

- `build_mode` defaults to `auto` (`:9`).
- The first auto branch builds only if `install/setup.bash` is unreadable
  (`:141-151`).
- The second auto branch builds only if the resolved portal executable is not
  executable (`:192-197`).
- Existing setup, package, portal, helper, RViz config, and launch file are
  enough to proceed; there is no source identity, installed identity, timestamp,
  or symbol comparison.
- The script then runs `ros2 launch openarm_ik_ros openarm_ik_rviz.launch.py`
  (`:313`), whose launch description unconditionally selects the installed
  `openarm_ik_ros_node` (`launch/openarm_ik_rviz.launch.py:27-31`).
- `--no-build` means only "do not invoke the builder".  It does not mean "prove
  the existing product corresponds to the current source".

Both setup and portal exist in the default tree, so direct default `auto` takes
neither build branch and selects the stale node described above.

### `scripts/launch_rviz.sh`

This script has no build/freshness mode at all.  It unconditionally sources
`/opt/ros/lyrical/setup.bash` and `ros2_ws/install/setup.bash` at `:29-30`, then
launches the same installed node at `:102-105` or `:173-174`.  The helper check
at `:107-112` checks existence only.  It therefore has the same stale-product
defect even though it shares the GUI-instance lock with the portal launcher.

The GUI lock serializes the two GUIs, while `build.sh` separately serializes and
bounds mutation through `openarm_run_with_locks` and a default job limit of two.
Neither lock currently establishes artifact freshness.  There is also a narrow
build-versus-launch race after a builder releases its output locks; if strict
runtime immutability is required, builders need an exclusive output lease and a
running launcher a shared lease (or builders must refuse while a GUI lease is
held).

## 3. Safe minimal launcher design

The smallest safe correction is one shared helper used before either launcher
sources the workspace overlay.  Do not maintain two implementations.

Suggested interface:

```text
openarm_ensure_current_launch_tree ROOT OUTPUT_ROOT MODE
openarm_assert_launch_authority ROOT OUTPUT_ROOT
```

Recommended initial policy:

1. In `auto` and `always`, call the existing
   `scripts/build.sh --incremental --output-root ...` every time.  This is the
   simplest freshness policy and inherits the existing nonblocking output locks,
   bounded native parallelism, and sequential colcon build.  A lock collision or
   build failure must abort launch.
2. In `never`, never build, but require an exact current source/build stamp and
   all artifact gates below.  A missing legacy stamp is a mismatch, not a reason
   to trust the tree.
3. After either route, independently audit the objects that will be used:
   - require the session archive; reject undefined direct
     `oa_controller_*`, `oa_motion_plan_*`, and `oa_manifest_*`; require
     undefined `oa_runtime_create`;
   - require the actual installed node to carry the Runtime-facade identity.
     Merely rejecting controller symbols in the final node is wrong because a
     correct statically linked Runtime legitimately contains its internal
     Control implementation;
   - reject `oa_can_*` or `oa_transport_*` references in the installed Runtime
     archive;
   - require the installed setup, node, portal/helper as appropriate, launch
     file, and configuration resources.
4. Hash the exact gated node, Runtime archive, and session archive into the
   stamp and recheck those hashes before launch.  This binds the gate result to
   the files subsequently selected.  The session archive is currently only in
   `build/`; either require the local build tree, install an audit copy/record,
   or add an explicit session-authority ELF marker to the installed node.
5. Invalidate the stamp inside the output lock before any build mutation.  On
   success, recompute the source fingerprint, require that it did not change
   during the build, run all gates, and atomically rename a completed stamp into
   the install.  A failed or interrupted build must leave no valid stamp.

Always running the incremental builder in `auto` avoids making the first fix
depend on a newly invented fingerprint.  A later optimization may skip the
build only when a versioned content fingerprint, build profile, tool identities,
artifact hashes, and gates all match.  A Git commit alone is insufficient in a
dirty developer checkout, and mtimes are not an identity.  A useful fingerprint
should hash path, file type/mode, and actual bytes of tracked and non-ignored
untracked production inputs under the native, ROS, CMake, and build-script
roots; it should also bind the description identity, build type/coverage mode,
ROS prefix/version, compiler, CMake, and colcon identities.  Output roots,
`.git`, and `.swarm` are not inputs.

An unsigned local stamp is an integrity/freshness record, not cryptographic
authentication against an attacker who can rewrite both artifacts and stamp.
That stronger threat model would require a trusted signature/key or immutable
distribution.  For the present accidental-staleness and partial-build threat,
atomic stamping plus live symbol/hash checks is sufficient.

Incremental build tools still ultimately decide which objects to recompile.  If
the stamp is intended to prove byte-for-byte derivation under adversarial clock
skew rather than merely record a completed current build, a fingerprint change
must trigger a clean build or the fingerprint must participate in compiler
dependencies.  The architecture-specific symbol gates should remain mandatory
regardless, because they directly enforce the safety properties at issue.

## 4. Description source pin

### Records and current state

`UPSTREAM_SOURCES.md:15` records:

- canonical origin `https://github.com/enactic/openarm_description.git`;
- commit `6c7b720f1ba48e8bafa3a3dc752c45f397b42221`;
- an intentionally detached, clean checkout;
- no submodules and no Git LFS paths at this snapshot.

The current checkout matches that record:

```text
HEAD       6c7b720f1ba48e8bafa3a3dc752c45f397b42221
tree       a05f0116710a4948e9f769237696fbf43701762f
branch     (empty; detached)
origin     https://github.com/enactic/openarm_description.git
status     empty, including untracked files
shallow    false
submodule  no .gitmodules, no gitlinks, no status output
```

The worktree `package.xml` blob equals `HEAD:package.xml`
(`5078ed786f2733339b516f46016d6422e8d714a6`).  There are 173 tracked files,
no symlinks or gitlinks, and no ignored worktree additions.  The existing model
generator also records pin `6c7b720...` and expected hash
`3f48ffec1598...`, but that hash covers only the v1.0 robot and parallel-link
source prefixes used by model generation, not every directory recursively
installed by the description package.

The parent OpenArmIK checkout is `main` at the requested `53bfd80`; its only
pre-existing tracked modification is `transport/tests/test_transport.cpp`, and
the existing `.swarm` reports are untracked.  The parent repository has no
configured `origin`.  These facts do not alter the nested description identity.

### The builders are identity-fail-open

`scripts/build.sh:105-109` and `scripts/build_native.sh:126-130` check only that
`upstream/openarm_description/package.xml` exists.  They do not invoke the
generator's Git/content validation in an ordinary Release build.  Generator
validation is registered only when model tests are enabled
(`model/CMakeLists.txt:61,79-86`).  The top-level builder then gives the unchecked
directory directly to both model configuration and colcon
(`scripts/build.sh:184-188`).

Thus "fail-open" here means fail-open with respect to source identity: a
non-Git directory, wrong commit, wrong origin, modified checkout, or checkout
with extra installable files passes the precondition if it has `package.xml`.
Missing `package.xml` does correctly fail before mutation.

The lightweight test confirms the weakness rather than covering the promised
pin.  Its so-called pinned fixture is created with only:

```text
mkdir -p .../upstream/openarm_description
touch .../upstream/openarm_description/package.xml
```

The subsequent assertions prove fixed path selection, not commit/provenance.

### Exact pin policy

Use one offline validator, sourced by both production builders and called before
any output mutation.  Store the URL, commit, and (preferably) full tracked-tree
content digest in one machine-readable checked-in record rather than duplicating
constants parsed from prose.

For this input the validator should require:

1. `git -C DIR rev-parse --is-inside-work-tree` succeeds.  Do not require
   `DIR/.git` to be a directory.
2. `HEAD` exactly equals `6c7b720f...` and the commit/tree objects resolve.
3. the local `origin` fetch URL exactly equals the declared HTTPS URL.  If SSH
   is to be supported, add it as an explicit allowlisted spelling; do not use a
   loose substring normalization.  This is a local check and performs no fetch.
4. index and tracked worktree content are clean (`git diff --quiet` and
   `git diff --cached --quiet`, with submodule differences not ignored).
5. the actual tracked bytes match a checked-in full-tree content digest.  This
   closes `assume-unchanged`/`skip-worktree` and index-stat-cache blind spots
   that a status-only check can have.
6. reject all untracked **and ignored** additions below CMake-consumed roots.
   This is not merely cosmetic: upstream `CMakeLists.txt:32-35` recursively
   installs `assets`, `launch`, `rviz`, and `scripts`, so an ignored/untracked
   file under those paths changes the production install.  The current checkout
   has none.  An alternative is to build a tracked-files-only staged tree.
7. because the checked-in manifest explicitly promises a detached checkout,
   either require detached state or change that promise.  Detached state is
   operational hardening, not source authentication: exact HEAD plus content is
   sufficient for identity.  Enforcing detached state is the least surprising
   policy for this repository.

### Offline, worktree, and submodule behavior

- Validation must be entirely offline: inspect local HEAD, tree/content, config,
  index, and worktree.  Do not fetch or resolve `origin/main`.  A missing object
  fails with an instruction to run `fetch_upstreams.sh` when online.
- A linked Git worktree has a `.git` **file**, so use `git -C` plumbing.  It is
  valid if it is at the exact pin and satisfies the same clean/content policy;
  it can also be detached.  Current `fetch_upstreams.sh:13` is not linked-
  worktree-safe because it tests `-d DIR/.git` and would try to clone into it.
- The current pinned commit has no `.gitmodules` and zero gitlinks, so the exact
  policy can simply expect none.  If a future pin introduces submodules, the
  superproject gitlink commits, each canonical origin, initialization, recursive
  exact HEAD, and clean/untracked policy must all be validated.  Reject `-`,
  `+`, or `U` prefixes from recursive submodule status; do not auto-update during
  an offline build validation.
- Shallow versus full-history state is not needed to authenticate the build
  bytes once the exact commit/tree exists.  It may be enforced separately to
  preserve the archival promise in `UPSTREAM_SOURCES.md`, but should not be
  confused with the production content gate.

`fetch_upstreams.sh` does check origin and HEAD after a network fetch/checkout,
but it does not establish build-time cleanliness/content identity, is not
called by either builder, is not offline, and is not future-submodule-safe.

## 5. Focused tests required

All of these can remain lightweight and hardware/GUI/network-free.

### Pin validator

- clean local repo at expected HEAD/origin: accept;
- non-Git fixture with a plausible `package.xml`: reject before any fake CMake,
  colcon, cleanup, or lock callback is observed;
- wrong HEAD and missing commit object: reject;
- wrong/missing/multiple unexpected origin URL: reject;
- unstaged tracked, staged tracked, untracked installable, and ignored
  installable changes: reject;
- altered content hidden with index flags: content digest rejects;
- linked worktree at the same pin: accept (and detached-policy case is explicit);
- uninitialized, wrong-commit, dirty, and wrong-origin submodules for a generic
  fixture: reject; current no-submodule pin: accept.

The miniature build test should create a real local Git fixture and source the
same machine-readable pin record/helper.  A zero-byte `package.xml` is no longer
a valid "pinned" fixture.

### Launcher/stamp/artifact gate

- `auto` with the existing legacy no-stamp tree invokes exactly one bounded
  incremental build before any overlay source/ROS command;
- `never` with missing or mismatched stamp fails and never invokes ROS/RViz;
- successful current stamp plus current symbol fixtures passes;
- session direct-Control references, missing `oa_runtime_create`, installed
  node lacking Runtime identity, and Runtime CAN/Transport references each fail
  independently;
- changed source, description identity, build profile/tool identity, or artifact
  bytes invalidates the stamp;
- failed/interrupted build leaves no valid stamp; source mutation during build
  prevents stamp publication;
- build-lock collision propagates failure and never launches;
- both web and standalone RViz scripts call the same helper, including the
  `rviz:=false` fast `exec` path;
- paths containing spaces and custom absolute output roots remain supported.

Use tiny synthetic archives compiled with the relevant undefined symbols, or a
controlled `nm` shim, rather than a full ROS build.

## 6. Exact false collision-help claim

`scripts/launch_web_portal.sh --help` currently says:

> Portal motion remains disabled unless the installed controller reports a
> verified collision scene containing both arms and the central support pole.

That is false.  The portal requires `collision_checked=false` and applies its
own sampled nominal capsule/central-keepout guard.  Its page and package README
correctly say this is not physical collision certification.  The help should
say, for example:

> Portal motion uses a limited sampled nominal virtual guard and central
> keepout; controller collision checking remains unavailable
> (`collision_checked=false`). This is not physical collision certification.

This documentation defect is separate from, and lower severity than, the two
fail-open build/launch defects.
