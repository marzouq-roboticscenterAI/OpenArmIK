# Persistence redesign B: adversarial architecture review

Date: 2026-07-29 (America/Los_Angeles)
Reviewed commit: `8c92f078dd5205eddbf835992f3f50cc36794650`
Disposition: **RECOMMENDATION**

## Executive result

The two reproduced failures are real, but they have different remedies.

1. The inherited-`flock` race is an implementation bug for cooperative writers.
   The lock is currently applied to the authority's long-lived directory file
   descriptor (`persistence.cpp:205-216, 419-423, 460-465`). `flock` ownership is
   attached to the open file description; `fork()` and `dup()` retain that same
   owner. The smallest correct lock change is to perform a fresh `openat(dirfd,
   ".", O_RDONLY|O_DIRECTORY|O_CLOEXEC)` for every transaction, lock that fresh
   open file description, and close it to unlock. A `dup`, `F_DUPFD`, or OFD lock
   placed on the inherited long-lived descriptor does **not** fix the problem.

2. The reopen/deletion replay is not fixable from authenticated files in the
   same rollback domain. `refresh_revision_floor` takes the maximum of the files
   that still exist and process-local memory (`persistence.cpp:218-256`). After a
   new authority starts with an empty memory floor, deleting revision 2 and
   replaying revision 1, whether by hard link, ordinary copy, filesystem snapshot,
   or whole-directory restore, makes revision 1 the locally self-consistent
   maximum. HMAC proves who produced the bytes; it does not prove that they are
   the newest bytes. Rejecting the `.previous` suffix (`persistence.cpp:428-434`)
   or rejecting multi-link inodes only blocks particular names/representations.

Accordingly, the existing API can honestly provide authenticated integrity,
same-authority monotonic memory, crash-conscious replacement, and cooperative
writer serialization. It cannot honestly provide durable rollback resistance
across authority/process restart. Either narrow that claim, or add a checked API
whose `(revision, digest)` floor is owned and durably stored by the caller outside
the manifest directory's rollback domain.

## Evidence in the current implementation

- The authority's accepted floor and per-name map are ordinary process memory
  (`runtime_internal.hpp:166-178`). They disappear when the authority/process is
  destroyed.
- Directory reconstruction authenticates every safe-named file currently visible
  and takes its maximum revision/digest (`persistence.cpp:218-256`). It has no
  external observation of a deleted maximum.
- Normal load only rejects a literal filename ending in `.previous`
  (`persistence.cpp:428-434`). The reviewed focused test deleted current revision
  2, hard-linked authenticated revision-1 `.previous` as `rollback.oarm`, reopened
  the authority, and both authenticated load and runtime creation succeeded.
  Copying the bytes rather than linking them has the same security result.
- The authority mutex only coordinates threads using the same in-process object.
  The OS lock uses its long-lived `directory_fd` (`persistence.cpp:205-216`). Linux
  `flock(2)` specifies that duplicated/fork-inherited descriptors refer to the same
  lock and that independently opened descriptions conflict. This exactly explains
  the focused result in which two forked children inherited one authority and both
  conflicting revision-2 saves returned `OK`.
- Traditional `F_SETLKW` record locks are process-owned, so separate authority
  objects in two threads of one process would not exclude each other. Linux
  `F_OFD_SETLKW` is suitable only if every contender first obtains a different
  open file description. Fresh-FD `flock` is already sufficient on the declared
  Linux/local-filesystem boundary; switching lock APIs is not itself a remedy.
- Locks are advisory. A process that deletes or replaces files without taking the
  protocol lock can still race every pathname operation.
- Path validation first `lstat`s each absolute component and later calls `open`
  (`persistence.cpp:66-91, 382-390`); an ancestor can be swapped between those
  operations. `O_NOFOLLOW` on the final `open` does not freeze earlier components.
  Once created, authority-relative operations correctly retain a directory inode,
  use `openat`, and reject target symlinks, but a hostile writer of that directory
  can still replay, delete, copy, hard-link, or race names.
- Save fsyncs the new file, makes/replaces `.previous`, fsyncs the directory,
  renames current, verifies it, and fsyncs the directory
  (`persistence.cpp:514-585`). This is a reasonable local crash protocol. It does
  not implement restart recovery: normal authenticated load rejects `.previous`,
  and a corrupt current prevents save because save first authenticates current.

## Honest threat model

| Actor/failure | Defended now or after the lock fix | Not defended |
|---|---|---|
| Random corruption, truncation, malformed input | Strict parser, SHA-256, HMAC, bounded reads | Media that returns old but valid authenticated sectors |
| File writer without the HMAC key | Cannot forge a new accepted manifest | Can replay an old signed file, delete files, deny service, and race advisory-lock users |
| Cooperative processes using the API and the same directory/key | Fresh-per-transaction lock plus exact target revision/digest checks can serialize conflicts | A writer that bypasses the lock |
| Holder of the HMAC key | Nothing should claim protection from arbitrary newly signed content | Can sign any revision/content; an independent policy/anchor is required to constrain it |
| Filesystem snapshot/backup restore or administrator/root | External trusted floor can detect a restored older tuple | Local current/floor/hash-chain files restored together cannot detect rollback |
| Crash/power loss | Only the documented fsync/rename guarantees of a qualified local filesystem/storage stack | Universal power-loss behavior across unqualified NFS/SMB/FUSE/device caches |
| `fork()` after library use | `CLOEXEC` plus immediate `exec` is a sound process model | General child use of inherited C++ registry/authority handles |

The security boundary must explicitly require that the manifest directory is not
writable by untrusted principals if availability and race freedom matter. HMAC
still prevents forgery by a directory-only attacker, but it does not prevent that
attacker's replay or denial of service. Root, an authorized signer, or an attacker
that also controls the external checkpoint is outside any local rollback claim.

## Proposed remedy and attempts to disprove it

### A. Cooperative transaction serialization

Use a newly opened description for each load/save/recovery transaction:

```text
lock_fd = openat(authority.directory_fd, ".",
                 O_RDONLY|O_DIRECTORY|O_CLOEXEC)
flock(lock_fd, LOCK_EX)        # retry EINTR or fail without touching files
... complete scan/check/write/rename/fsync ...
close(lock_fd)                 # releases this transaction's lock
```

This survives the reported fork topology because each child transaction performs
a new open after the fork. It also excludes separate authorities in the same
process. `dup(authority.directory_fd)` is specifically wrong: it preserves the
same open file description. `F_OFD_SETLKW` is equivalent only with the same fresh
open rule. Traditional POSIX `F_SETLKW` is wrong for same-process independent
authority objects because those locks are process-associated.

This remedy is intentionally only a cooperating-writer guarantee. Advisory locks
cannot protect against direct file mutation. Qualify the supported filesystem set;
do not silently generalize local Linux lock/durability behavior to arbitrary
network filesystems.

### B. Fork handling

Fresh lock FDs fix the demonstrated lock-owner collision, but do not make general
inherited-handle use safe. `Registry::pin` and every authority contain
`std::mutex`; if a multithreaded parent forks while one was locked, the child can
deadlock before a per-authority PID check is reached. A copied unlocked mutex is
also not a portable post-fork C++ synchronization contract.

The smallest honest policy is: after the process has used this library, a forked
child may call only async-signal-safe functions and `exec`; no OpenArm handle or
other OpenArm API is valid in the child. Document authorities as process-local.
A creator PID check may return `OA_RUNTIME_ESTATE` in ordinary accidental cases,
but it is diagnostic, not a proof of fork safety. If detection is desired, use a
process-generation/PID guard before entering any registry mutex, not a field that
can only be read after `Registry::pin`.

Supporting arbitrary inherited handles would require a library-wide at-fork
design for every registry/object/worker mutex, not a persistence-only patch. It is
not the smallest safe remedy and should not be promised in V1.

### C. Durable rollback resistance

No signed local sidecar, journal, maximum file, `.previous`, hash chain, inode-link
rule, or directory scan fixes rollback if it can be restored/deleted with current.
The smallest sound additive interface accepts a caller-owned checkpoint and
returns the newly observed/committed checkpoint only after success:

```text
checkpoint := { ABI size/version, revision, exact 32-byte content digest }

load_authenticated_checked(authority, name, trusted_floor,
                           out_manifest, out_observed_checkpoint)

save_checked(manifest, authority, name,
             trusted_floor, expected_current,
             out_committed_checkpoint)
```

Required semantics:

- A nonzero floor must include an exact digest. Candidate revision below the floor,
  or equal revision with a different digest, returns `OA_RUNTIME_ESTALE`.
- A candidate above the floor may be accepted; the caller must durably advance its
  external checkpoint after the API returns `OK`.
- Checked save must compare the exact current `(revision,digest)` with
  `expected_current` under the transaction lock. This is a CAS, preventing a
  revision-4 writer based on revision 1 from silently overwriting intervening
  revision 2/3 merely because 4 is numerically larger.
- The checkpoint scope is the logical slot `(application domain, key ID,
  file_name)`, not merely a directory maximum. The caller must bind/store that
  scope with the tuple. Otherwise checkpoints from independent artifacts collide
  or can be misapplied.
- `out_*checkpoint` is valid only on `OK`. On `EIO` it remains unchanged. On
  `EDURABILITY`, the caller must not advance it and must run explicit checked
  reconciliation. A crash after filesystem commit but before external checkpoint
  update is safe: the old external floor accepts a strictly newer authenticated
  current, after which the caller may advance the anchor.
- Storing this checkpoint in the same directory, under the same snapshot/restore
  authority, gives no rollback security. Appropriate anchors include a caller
  database with independent anti-rollback controls, a remote append-only service,
  or suitable TPM/secure monotonic state.

Keep the existing functions for ABI compatibility, but describe them as
authentication plus best-effort local stale detection, not restart-persistent
rollback protection. If a checked API is not being added now, delete/narrow the
README claim that authenticated artifacts establish a monotonic floor across
newly opened authorities.

### D. Path, link, and directory defenses

These are worthwhile hardening but must not be used as the rollback proof:

- Open the authority directory by walking components from `/` with `openat` and
  `O_DIRECTORY|O_NOFOLLOW`, or use Linux `openat2` with an appropriate
  `RESOLVE_NO_SYMLINKS` policy. The current check-then-open component walk has a
  race.
- Continue all artifact operations relative to the retained directory FD.
- Reject non-regular files and symlinks. Optionally require `st_nlink == 1` at
  stable read/target boundaries to catch accidental aliases, while documenting
  that an ordinary byte copy bypasses this check and an uncooperative directory
  writer can race it.
- Document or enforce owner/mode expectations for the authority directory. A
  group/world-writable directory permits replay and denial of service even when
  the HMAC key remains secret.
- Reserve all internal temporary/lock/recovery names. Check operation errors and
  compare opened inode metadata where it closes a real TOCTOU window.

### E. Current/`.previous` recovery

`.previous` is an availability artifact, not a monotonic oracle. Add an explicit
read-only recovery/reconcile operation (or document a caller procedure) that,
under the same transaction lock, authenticates current and previous and reports
which was selected. It must apply the caller's external floor before returning an
authenticated/armable handle.

- With no external floor, selecting the highest valid current/previous is useful
  crash recovery but is not rollback-resistant.
- If the trusted floor is revision 2 and only revision-1 previous survives, fail
  closed with `ESTALE`; availability cannot override rollback protection.
- Do not make a literal `.previous` suffix a security boundary. Normal load may
  continue to reject it to prevent accidental use, while an explicit recovery API
  treats it as a candidate subject to the trusted floor.
- A corrupt current currently wedges ordinary save because save authenticates it
  before replacement. Recovery must define whether repair is read-only, an atomic
  promotion, or an explicitly authorized overwrite; it must not silently turn
  corruption into rollback.

## Exact verification matrix

All concurrency cases need a start barrier, per-process result pipe, bounded
timeout, exact final revision/digest checks, and at least 100 iterations under
Release plus a smaller sanitizer run. Tests must compare digest, not revision
alone.

### Locking and conflicts

| ID | Topology/setup | Operation | Required result |
|---|---|---|---|
| L1 | One authority, two threads, current `(1,A)` | Concurrent saves `(2,B)` and `(2,C)` | Exactly one `OK`, one `ESTALE`; current equals winner; previous `(1,A)` |
| L2 | Two authorities in one process/two threads | Same as L1 | Same result; catches incorrect traditional process locks |
| L3 | Two independently exec'd processes | Same as L1 | Same result; no hang/lost update |
| L4 | One authority created before single-threaded fork | If inherited handles are deliberately supported, children run same as L1 | Same result with a fresh lock FD in each child; otherwise contract test/documentation says all child OpenArm calls are invalid |
| L5 | Parent forks while another thread is inside registry/authority | Child immediately execs a helper | Exec succeeds; no child OpenArm call is made. If a global fork guard is added, an accidental child call returns `ESTATE` before registry locking |
| L6 | Two writers propose `(3,D)` and `(4,E)` from expected `(1,A)` after current became `(2,B)` | `save_checked` | Both `ESTALE` until they reload exact current; proves CAS rather than numeric-only ordering |
| L7 | Lock acquisition interrupted by a caught signal | Load/save | Retry safely or return a documented error before any mutation; never run unlocked |
| L8 | Independent logical filenames in one directory | Advance only slot A | Slot B behavior matches the documented per-slot scope; no undocumented directory-wide digest collision |

### Rollback and checkpoint behavior

| ID | Setup/mutation | Legacy expected result | Checked expected result |
|---|---|---|---|
| R1 | Save 1 then 2; reopen with current 2 still present; load copied/linked 1 under another name | May reject due local maximum | Floor `(2,B)` rejects `ESTALE` |
| R2 | Save 1 then 2; destroy; delete current; hard-link previous as current/other name | Must not be claimed rollback-safe; authenticated replay may load | Floor `(2,B)` rejects `ESTALE` |
| R3 | R2 using byte copy with link count 1 | Same as R2 | Same as R2; disproves hard-link-only defense |
| R4 | Restore whole directory snapshot containing only revision 1 | Authenticates revision 1 | External floor `(2,B)` rejects `ESTALE` |
| R5 | Candidate revision 2 with different valid HMAC/digest C | Local scan may detect only if B remains | Floor `(2,B)` rejects `ESTALE` |
| R6 | External floor 1; current valid 2 | Authenticated load succeeds | Checked load succeeds and returns observed `(2,B)`; caller advances anchor only afterward |
| R7 | External floor 3; current/previous at 2/1 | Legacy may load 2 | Checked load/recovery returns `ESTALE`, no handle |
| R8 | Checkpoint stored beside manifests, then directory snapshot restored | Appears self-consistent | Test documents this is **not** a trusted deployment; no security claim |
| R9 | Correct directory but floor bound to another filename/key/domain | N/A | Checked API rejects scope mismatch (`EINVAL`/`EPERMISSION` as specified) |

### Namespace/type hardening

| ID | Mutation | Required result |
|---|---|---|
| P1 | Symlink in every possible ancestor component before create | Authority creation `EPERMISSION` |
| P2 | Repeatedly swap an ancestor between directory and symlink during create | Never opens outside the intended component walk; only success on the real directory or clean failure |
| P3 | Target is symlink, directory, FIFO, socket, or device | Load/save reject; no blocking on FIFO and no mutation |
| P4 | Current or candidate has `st_nlink > 1` | If single-link policy is adopted, reject deterministically; test explicitly notes copies are still possible |
| P5 | Directory is writable by an untrusted UID/process that ignores the lock | Replay/delete/race is detected or fails closed where possible, but test and docs classify denial/replay as outside cooperative guarantees |
| P6 | Directory pathname is renamed/replaced after authority creation | Existing authority stays on retained inode; a new authority opens only the newly validated directory and relies on its own external checkpoint |

### Crash, fsync, and recovery

Use subprocess killpoints after each numbered operation, remount/reopen in a new
process when the test environment permits, and enumerate current, previous, and
internal orphan names. A mere mocked `fsync` return test is not a power-loss test.

| ID | Kill/failure point during update old `(1,A)` to new `(2,B)` | Required restart invariant |
|---|---|---|
| C1 | During temp write, before temp fsync | Old current remains acceptable; partial temp is never selected |
| C2 | After temp fsync, before backup link/rename | Old current remains acceptable; orphan temp may be cleaned explicitly |
| C3 | After backup link or rename, before backup directory fsync | Current is old; recovery ignores internal backup names and authenticates candidates |
| C4 | After backup directory fsync, before current rename | Durable old current and recoverable old previous |
| C5 | After current rename, before verify/final directory fsync | Recovery accepts only a fully authenticated current/previous satisfying floor; otherwise fails closed. Do not assume more than the qualified filesystem promises |
| C6 | After verification, before final directory fsync | Same as C5 |
| C7 | After successful final directory fsync/`OK` | New current `(2,B)` is the committed result; previous `(1,A)`; returned checkpoint `(2,B)` |
| C8 | Final directory fsync reports failure; rollback rename succeeds and rollback fsync succeeds | Return `EIO`; old current `(1,A)` durable; caller checkpoint unchanged |
| C9 | Final sync failure plus rollback rename or rollback sync failure | Return `EDURABILITY`; checkpoint unchanged; checked reconciliation required before use |
| C10 | Corrupt/missing current with valid previous equal to trusted floor | Explicit recovery may return previous and identify its source; normal load behavior remains documented |
| C11 | Corrupt/missing current with previous below trusted floor | `ESTALE`/fail closed; no authenticated handle and no automatic promotion |
| C12 | Initial save with no old current, killed before/after final dir fsync | Before confirmed commit: absent or valid new are the only candidates; after `OK`: valid new is durable; there is no fabricated previous |

Run C1-C12 on each filesystem actually claimed (at minimum the CI local
filesystem); treat NFS, SMB, overlay/FUSE, and removable-device write caches as
unsupported until separately qualified. Add fault injection for write, close,
link, rename, open/verify, each distinct fsync, rollback rename/unlink, and rollback
fsync. The current single countdown cannot directly force both the commit-sync and
rollback-sync failures needed to cover `EDURABILITY` deterministically.

## Final recommendation

Ship the fresh-per-transaction open-description lock as an internal compatible
fix. Make post-fork OpenArm use unsupported (optionally diagnosed by a pre-registry
process guard). Harden directory opening component-by-component and optionally
reject stable multi-link artifacts as defense in depth.

Most importantly, remove the unconditional restart-persistent monotonic/rollback
claim from the existing API. If that property is required, add checked load/save
and explicit recovery APIs with caller-owned, independently persisted
`(revision,digest)` checkpoints and exact-current CAS. There is no honest
local-files-only patch that preserves the present claim against deletion,
hard-link/copy replay, snapshot restore, or an authorized local rollback actor.
