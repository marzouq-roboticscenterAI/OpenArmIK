# Persistence redesign A: freshness must come from outside the replayable directory

Reviewed: `8c92f078dd5205eddbf835992f3f50cc36794650` and
`.swarm/runtime_facade_review.md`.

Disposition: **RECOMMENDATION**. Keep persistence only after replacing the
current implicit anti-rollback claim with an explicit caller-supplied trusted
checkpoint and fixing transaction lock ownership. If the product cannot supply
that checkpoint from storage outside the directory attack domain, remove
authenticated ARMABLE-file acceptance and the manifest-persistence capability;
HMAC-only files may remain as authenticated, replayable drafts.

## Evidence and impossibility boundary

The review's two reproductions follow directly from the implementation:

* `refresh_revision_floor()` derives freshness from authenticated directory
  entries and process-local memory (`persistence.cpp:218-256`). Removing the
  newest entry and reopening erases both sources. The `.previous` suffix test
  (`:428-434`) is a name policy, not a freshness proof; a hard link gives the
  same authenticated inode another accepted name.
* `DirectoryTransaction` calls `flock()` on the authority's long-lived directory
  fd (`:205-216`). After `fork()`, parent and children refer to one open file
  description, hence one `flock` owner. Their copied C++ mutexes do not
  synchronize. Conflicting child transactions can therefore overlap.

There is a simple indistinguishability proof for the first defect. Consider:

1. World A legitimately stopped after authenticated state `(revision=1,
   digest=D1)`.
2. World B committed `(2,D2)`, then an attacker lacking the HMAC key deleted it
   and restored the previously observed authentic bytes `(1,D1)`.

At a fresh open, the directory bytes and static external HMAC key are identical
in A and B. No algorithm using only those inputs can accept A and reject B.
Authenticated hash chains, an authenticated high-water-mark file, extra copies,
`.previous`, file names, inode numbers, timestamps, and scan maxima are all
replayable with the directory. HMAC proves origin and integrity, not freshness.
Availability is also impossible: a writer able to unlink directory entries can
always force a fail-closed error.

Rollback detection becomes possible only with non-replayable state outside this
attack domain: for example a caller-owned trusted `(revision,digest)` checkpoint,
a TPM/secure-element monotonic record, a remote compare-and-swap service, or
read-only administrator deployment state. An HMAC key is insufficient unless
its provider also stores changing checkpoint state. A checkpoint stored beside
the manifest, even if MACed, is insufficient.

## Accurate threat contract

The supported contract should say all of the following explicitly:

* The key and caller checkpoint are trusted. The directory may replay, remove,
  rename, hard-link, or corrupt files, but cannot read the key or modify the
  checkpoint. SHA-256 collision resistance is assumed.
* Given trusted floor `(R,D)`, authenticated load accepts candidate `(r,d)` only
  when `r > R`, or when `r == R && d == D`. It rejects `r < R` and same-revision
  equivocation. Thus the guarantee is relative to the supplied floor, not to
  history the caller failed to record.
* A newly observed `(r,d)` above the floor must be committed to trusted external
  storage before the application treats it as its checkpoint for a later open.
  An ARMABLE file load requires a nonzero trusted checkpoint. Revision zero is
  allowed only for provisioning an absent stream; it is not an anti-rollback
  assertion.
* The checkpoint is scoped by logical stream identity: application namespace,
  key ID, canonical directory/slot, and file name. It must not be reused for an
  unrelated stream.
* Advisory locks serialize cooperating library users. An untrusted process may
  ignore them and cause denial of service. A process that knows the HMAC key can
  create arbitrary authentic revisions and is a writer authority, not an
  attacker this scheme can contain. Same-UID `ptrace`/memory-reading attacks are
  likewise outside the key-confidentiality model.
* Crash durability assumes a local filesystem that documents atomic same-
  directory rename/hard-link and meaningful file/directory `fsync`. NFS-like or
  otherwise weaker filesystems are unsupported unless separately qualified.
* A pre-fork authority may be used after a quiescent, single-threaded `fork`.
  Forking while any runtime call is active, or using inherited C++ registry/
  mutex state after fork of a multithreaded process, is unsupported; POSIX in
  general permits only async-signal-safe calls in that child before `exec`.

Without these qualifications, persistence must not be described as unconditionally
rollback-safe, cross-process safe, tamper-proof, or durable.

## Small stable C API

Use one ABI-versioned checkpoint record and require it at authority open and at
every authenticated operation. New names avoid silently changing the semantics
of the published V1 functions.

```c
typedef struct oa_runtime_persistence_checkpoint {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t revision;
    char content_sha256[OA_RUNTIME_DIGEST_CAPACITY];
} oa_runtime_persistence_checkpoint;

oa_runtime_status oa_runtime_persistence_authority_open_v2(
    const char *absolute_directory,
    const uint8_t key[OA_RUNTIME_PERSISTENCE_KEY_BYTES],
    const char *key_id,
    const oa_runtime_persistence_checkpoint *trusted_floor,
    oa_runtime_persistence_authority **out_authority);

oa_runtime_status oa_runtime_manifest_load_authenticated_v2(
    const oa_runtime_persistence_authority *authority,
    const char *file_name,
    const oa_runtime_persistence_checkpoint *trusted_floor,
    oa_runtime_manifest **out_manifest,
    oa_runtime_persistence_checkpoint *out_observed);

oa_runtime_status oa_runtime_manifest_save_v2(
    const oa_runtime_manifest *manifest,
    const oa_runtime_persistence_authority *authority,
    const char *file_name,
    const oa_runtime_persistence_checkpoint *trusted_floor,
    oa_runtime_persistence_checkpoint *out_committed);

oa_runtime_status oa_runtime_manifest_recover_v2(
    const oa_runtime_persistence_authority *authority,
    const char *file_name,
    const oa_runtime_persistence_checkpoint *trusted_floor,
    oa_runtime_manifest **out_manifest,
    oa_runtime_persistence_checkpoint *out_observed);
```

`revision==0` requires an empty digest. A nonzero revision requires exactly 64
lowercase hexadecimal digits. Outputs use the normal size/version convention
and are cleared on failure. `load` is read-only and never interprets a public
name ending in `.previous`; `recover` is the sole operation allowed to inspect
the internal prior copy and may reinstall it only if it satisfies the supplied
floor. Plain preview/load remain explicitly unauthenticated and may not produce
an ARMABLE runtime.

The effective floor for each call is the maximum of the open-time floor, the
call-time floor, and successful observations/commits in that authority. Equal
revisions with unequal digests are an immediate `OA_RUNTIME_ESTALE`. Do not scan
the directory to invent a floor. A scan can aid diagnostics but is neither
trusted state nor transaction correctness.

The V1 authenticated load/save/open symbols should be removed in an ABI-major
change, or retained only under names/documentation that say “authentication,
no replay protection.” They must not keep enabling ARMABLE runtime creation
while accepting an omitted/zero checkpoint.

## Implementation shape

1. Bind each HMAC to its logical slot, not merely to manifest bytes. Use domain-
   separated input such as
   `"OPENARM-RUNTIME-SLOT-V2\0" || key_id || "\0" || file_name || "\0" || payload`.
   Verify `.previous` using its base slot name. This makes relinking bytes under
   `rollback.oarm` fail authentication, although the external floor remains
   necessary against replay under the original name.
2. Open an absolute directory component-by-component from `/` with
   `openat(O_DIRECTORY|O_NOFOLLOW|O_CLOEXEC)` (or qualified `openat2` resolve
   flags), rejecting `.` and `..`. The current pre-`lstat` path walk followed by
   one `open()` is raceable at intermediate components. Thereafter use only the
   pinned directory fd and relative `*at` calls.
3. At the start of **every transaction**, obtain a new open file description:

   ```c
   txfd = openat(authority_dirfd, ".",
                 O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
   while (flock(txfd, LOCK_EX) < 0 && errno == EINTR) {}
   ```

   Hold `txfd` through validation, write, verification, and final directory
   sync; close it to unlock. Never `dup()` the authority fd for locking because
   that preserves the open file description. An independently opened OFD lock
   is also suitable. The transaction lock itself serializes threads, separate
   authorities, processes, and quiescent-fork children, so remove the per-
   authority mutex from this path; copied `std::mutex` state is not a fork
   primitive. The registry limitation after multithreaded fork remains part of
   the contract above.
4. Under the lock, authenticate the canonical current entry from one opened fd,
   apply the effective trusted floor, and compare it with the proposed revision/
   digest. Accept an identical save idempotently only after syncing the file and
   directory; otherwise `OK` would overstate durability.
5. Keep the current same-directory sequence: exclusive temporary file, complete
   write, file `fsync`, close; atomically install/fsync `<name>.previous`; rename
   temporary to current; reopen/HMAC/revision/digest verify; directory `fsync`.
   Retry interruptible I/O where appropriate and clean only transaction-owned
   temporary names.
6. Preserve the existing status distinction, but make it exact:
   * `OK`: new current was reloaded and the final directory sync succeeded;
     `out_committed` is valid.
   * `EIO`: no namespace commit occurred, or the old current was restored and
     that rollback directory sync succeeded. No committed output is returned.
   * `EDURABILITY`: a post-rename failure could not be followed by a confirmed,
     synced rollback. On this result the authority becomes poisoned: every
     further operation returns `EDURABILITY` until destroy/reopen/recover with
     the unchanged external checkpoint. Never guess a new in-memory floor.
7. Zero the key on destruction. Reserve `.previous` and both temporary prefixes
   from all public file-name inputs.

The checkpoint update and manifest save cannot be made atomically by this
library when the checkpoint provider is external. The safe sequence is: save;
on `OK`, durably compare-and-swap the external checkpoint from the supplied
floor to `out_committed`; only then advertise the new checkpoint for future
opens. A crash between these steps may leave a newer valid current above the old
floor, which recovery may adopt; it must never make an older-than-floor state
acceptable.

## Required adversarial tests

All tests use only the installed C API except syscall/crash hooks. Assert the
exact status and that `out_manifest`/checkpoint outputs remain cleared on every
failure.

### Replay, unlink, rename, and hard-link

1. Save revision 1 and record `C1`; retain its exact bytes. Save distinct
   revision 2 and record trusted `C2`; destroy all authorities. Replace current
   with the saved revision-1 bytes, reopen with `C2`, and require `ESTALE`.
2. Repeat by deleting current and hard-linking `.previous` as (a) the canonical
   current name and (b) `rollback.oarm`. Case (a) is stale against `C2`; case
   (b) fails slot authentication. Neither can create an ARMABLE runtime.
3. Delete current while leaving `.previous`; ordinary load fails and never
   falls back. Recovery with `C2` rejects old `.previous`. Recovery with `C1`
   may reinstall exact `(1,D1)`, demonstrating the explicitly relative
   guarantee.
4. Delete current and `.previous`; load/recovery fail closed. Replace either
   with symlinks, directories, devices, FIFOs, overlarge files, truncated files,
   and byte-modified HMACs; none is accepted and no blocking special file is
   opened.
5. Replay an authentic current under a different public name, use intermediate-
   component symlink swaps during authority open, and hard-link an external
   regular file into the directory. Slot binding/path walk must reject or safely
   pin the intended object.
6. Supply `(R,Dwrong)`, a lower floor after the authority has observed a higher
   one, an invalid digest encoding, and a zero revision with a digest. Require
   `ESTALE` or `EINVAL` as specified. Supply trusted `(2,D2)` against a separately
   authenticated same-revision `D2b`; require `ESTALE`.

### Serialization and fork

7. Create the authority and two distinct revision-2 manifest handles before a
   quiescent single-threaded `fork`. Release two children simultaneously through
   a pipe/barrier; both call save through the inherited authority and `C1`.
   Across at least 1,000 iterations exactly one returns `OK` and one `ESTALE`;
   current is the winner and `.previous` is revision 1. This is the regression
   the present suite lacks.
8. Run the same conflict with two threads sharing one authority, two authorities
   in one process, independently opened authorities in separate processes, and
   mixed thread/process contenders. Every run has one winner and no torn file.
9. Kill a child while it holds the transaction lock at each hook. A waiter must
   acquire the lock after fd closure and recover according to the last durable
   crash point. Verify no inherited long-lived `flock` makes contenders appear
   mutually locked.
10. State rather than test support for fork-during-active-call/multithreaded
    fork. A debug hook may fail such use fast, but it must not be advertised as
    supported.

### Crash and durability points

11. With and without an existing current, terminate the writer after each of:
    temporary create, partial/full write, temporary file sync, `.previous`
    link/rename, first directory sync, current rename, reload verification, and
    final directory sync. After real remount/recovery on a qualified filesystem:
    * before the first directory sync, old current remains the only required
      usable state;
    * after durable `.previous` but before current rename, current and prior are
      old;
    * after current rename but before final sync, recovery may see old, new, or
      missing current, but must accept nothing below the trusted floor and has a
      durable prior recovery input;
    * after final sync, new current and old `.previous` are durable.
12. Inject failure for every write/close/file-sync/link/rename/reload/directory-
    sync call. Before current rename require `EIO` and unchanged current. After
    current rename, a successful synced rollback requires `EIO`; failed rollback
    or rollback-sync requires `EDURABILITY`, cleared output, and a poisoned
    authority. Cover both restore-old and remove-new branches.
13. Crash after save `OK` but before external checkpoint CAS. Reopen with the old
    trusted floor: the newer authentic current may be observed and adopted. Then
    commit the new checkpoint and prove every old/current/previous replay is
    rejected. Also test a failed external CAS caused by another writer: the
    loser must reload against the winning checkpoint, not overwrite it.

Ordinary unit fault injection cannot prove power-loss durability. The crash
matrix needs a loopback/throwaway qualified filesystem with forced process death
and remount (or a storage fault framework such as dm-flakey), in addition to the
fast syscall-failure suite.

## Final decision

Persistence is not mathematically blocked. Integrity, relative rollback
detection, cooperative transaction serialization, atomic replacement, and
qualified-filesystem durability can all be stated honestly with the design
above. Absolute freshness from a static HMAC key plus an attacker-writable
filesystem is impossible. Therefore either implement the V2 checkpoint contract
and fork-safe transaction fd, or remove persistence as an authority-bearing
feature; another directory-local “floor” revision would repeat the same defect.
