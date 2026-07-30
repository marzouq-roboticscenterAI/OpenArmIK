# Independent bounded-build final review

Date: 2026-07-29  
Reviewed: `131e5b1` on `fix/bounded-build-memory-resume`  
Verdict: **CLEAN** — no Critical or Important findings remain.

## Final targeted verification

### Lock-directory security

- A supplied XDG runtime base is accepted only when it is a non-symlink
  directory owned by the effective UID, writable/searchable by that user, and
  has no group/other write bits. The normal Ubuntu base observed here is
  `/run/user/1000`, user-owned at mode 700.
- Otherwise the helper accepts `/tmp` only after verifying root ownership,
  directory type, non-symlink status, usability, and the sticky bit.
- The fixed child is created with one atomic `mkdir -m 700` operation. Whether
  newly created or pre-existing, it is then independently checked as a
  non-symlink directory owned by the effective UID at exact mode 700 before any
  lock file is opened.
- All `mkdir` and `stat` statuses are captured and checked explicitly. The code
  no longer depends on `errexit` inside a function invoked through a conditional
  command substitution, so the former masked-`chmod` path is gone.
- If another UID wins the `/tmp` creation race, postvalidation rejects its
  object. If the victim wins, `/tmp`'s sticky bit prevents another UID from
  replacing the victim-owned directory, and mode 700 prevents entry or planting
  lock files. Thus there is no remaining cross-UID window between validation
  and `exec > lock_file` on the supported fallback.
- A pre-existing attacker lock file or symlink cannot be planted by another UID
  inside a validated victim-owned mode-700 directory. Same-UID deliberate
  tampering has the victim's own filesystem authority and is outside this
  cooperative build-serialization boundary.
- The regression covers owner mismatch, permissive modes, insecure/symlinked
  XDG bases, a deterministic mkdir-time symlink race, preservation of a victim
  sentinel, and failure before the mutation callback.

### Supervisor and build semantics

- The waiting parent retains all canonical resource locks. The private callback
  closes every lock FD, disables monitor mode, and starts cleanup/build/test
  work in one owned process group; callback and grandchild FD/PGID checks pass.
- Recursive requests contend rather than reenter. There is no public internal
  body option, environment lock record, or boolean ownership bypass.
- HUP/INT/TERM are trapped before callback creation, including the pending-signal
  window. Signals are forwarded to the exact callback PGID, the leader is
  reaped, stubborn groups receive bounded KILL escalation, locks remain held
  until the group is empty, conventional statuses are returned, and immediate
  reacquisition succeeds.
- Top-level builds own output, native-build, and install-prefix resources while
  directly composing the source-only native body. Public native builds always
  lock build root and install prefix. Canonical aliases/shared prefixes contend;
  independent siblings proceed.
- Native and top-level public scripts use only the repository's pinned
  `upstream/openarm_description`. The copied mini-repository fixture proves a
  decoy environment path is ignored and missing pinned input fails before
  mutation.
- Top-level validation now checks the xacro executable and Python package
  directory, as the native public wrapper does, before locking, cleanup, or
  other mutation.
- The resource test operates only below its `mktemp` root, uses a clearly
  bounded shim section for argument inspection, and also runs a real one-job
  CMake compile and real CTest.
- The unmanaged browser closes GUI FD 9. Existing exact GUI process-group
  cleanup, source-fresh `run.sh`, incremental reuse, explicit job bounds,
  sequential colcon, and merged runtime-authority checks remain intact.

## Fresh checks

- `bash -n` passed for all changed shell files.
- `tests/test_build_resource_controls.sh` passed in full at `131e5b1`, including
  real CMake/CTest, security-directory cases, FD noninheritance, recursive
  contention, signal/escalation, pinned fixtures, canonical locks, incremental
  reuse, and browser-FD closure.
- No heavy project build, GUI, middleware launch, or physical transmission was
  run.

