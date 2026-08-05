# Independent Runtime ABI V1 diagnosis

- Target: `53bfd809a7bd81c753fcdd354c1412339b9d2f1a`
- Scope: public Runtime headers, implementation, history/reflogs, package metadata,
  tests, installed consumers/artifacts, and an isolated old-header canary
- Verdict: **Important, confirmed contract defect; not Critical**

## Bottom line

The mechanical finding is real. An object compiled from the first Runtime V1
header does not interoperate with the current archive even though both sides say
`OA_RUNTIME_ABI_VERSION == 1`, the CMake package remains `1.0.0`, and the changed
function retains the same unmangled C symbol. The current implementation safely
rejects old-size output/input records rather than overwriting them, so this is
not a demonstrated memory-corruption or motion-authorization path and is not
Critical. It is nevertheless an Important compatibility defect in the API that
the repository calls its "stable ISO-C" boundary.

There is meaningful pre-release evidence: no Git tags exist, no `origin` is
configured, all Runtime feature/fix commits were authored in one afternoon, and
the six commits were cherry-picked to `main` in the same second before the
recorded clean acceptance. That evidence does **not** establish a source-only
pre-release policy or prove that no installed artifact was distributed. It also
does not override the explicit stable/V1/1.0.0 claims. The repository's Control
component provides the relevant local precedent: it freezes its first feature
header, accepts original V1 prefixes, and runs a frozen-header consumer even
though those feature/fix commits followed the same batch-integration workflow.
Therefore the defect cannot responsibly be dismissed as a false positive from
the repository evidence alone. It could be reclassified as pre-release only
after an authoritative release record proves that no pre-final Runtime header
or artifact was ever published and the project explicitly documents that
commit/feature APIs are unstable until a named release point.

## Frozen header and change history

The original authored header is at `0f0ea372bddc302ef78837c611ee9c25c9b82fad`;
the byte-identical header on the current ancestry is
`6c89618b9faed650dabfe025c814c7943e93bc4a`. Both use header blob
`9afc63ae76c3571225f45d63ca57b6ce800c9c9c`. `6c89618` is consequently the
last current-ancestry commit containing the original V1 API, and is the correct
frozen-old-header canary input if V1 froze when first advertised.

The incompatible edits are exact:

1. `3a02c5b57447b73e942cf867e3c4055cb14ec994` changed
   `oa_runtime_manifest_save(manifest, absolute_path,
   persistence_authority_uint32)` to
   `oa_runtime_manifest_save(manifest, authority_handle, file_name)`, removed
   `OA_RUNTIME_PERSISTENCE_AUTHORIZED`, and enlarged seven existing records:
   `oa_runtime_capability_report`, `oa_runtime_manifest_summary`,
   `oa_runtime_snapshot`, `oa_runtime_kinematics`,
   `oa_runtime_paired_tcp_move`, `oa_runtime_plan_report`, and
   `oa_runtime_event`. Most new fields were inserted before old fields, changing
   old offsets rather than forming an append-only tail.
2. `4790241ea998e38c69c3633b18cf36f79b8d7ad4` appended identity fields to
   `oa_runtime_joint_move` and inserted `tcp_revision` into the existing plan
   report.
3. `df8803c913f0d5bb2e8dfb4e8094a0f89b2b2c86` inserted
   `checkpoint_authorized` into the existing manifest summary while retaining
   Runtime ABI version 1.
4. Later `bd46687` changes only header comments. No later commit repairs old
   layouts/signature or changes the ABI/package major.

If the intended freeze point was instead the completed feature review, header
blob `daf321e9f0eaf625dda08fbc3188625c001936e9` at `315f787` is the final
pre-acceptance V1 header, and no ABI layout has changed after it. That is the
best argument for a pre-release exception, but no checked-in release policy
names `315f787` (or the subsequent `e0d06c8` acceptance) as the first public
Runtime ABI.

## Concrete x86-64 layout evidence

A strict C11 probe compiled the shared old fields once against `6c89618` and
once against the current header. Representative results are:

| Record | Old size | Current size | Existing old offset change |
|---|---:|---:|---|
| `oa_runtime_capability_report` | 40 | 128 | `collision_checked` 28 -> 32; `capabilities` 32 -> 48 |
| `oa_runtime_manifest_summary` | 176 | 248 | fingerprint 40 -> 52; content digest 105 -> 117 |
| `oa_runtime_snapshot` | 848 | 920 | `arm` 48 -> 120 |
| `oa_runtime_kinematics` | 576 | 920 | `q_model_rad` 32 -> 376 |
| `oa_runtime_joint_move` | 88 | 184 | old prefix offsets retained; tail appended |
| `oa_runtime_paired_tcp_move` | 152 | 248 | branch step 136 -> 232 |
| `oa_runtime_plan_report` | 256 | 344 | scene revision 72 -> 88; targets 80 -> 168 |
| `oa_runtime_event` | 136 | 152 | command ID 104 -> 120 |

The values are platform-specific, but the C field insertion and size changes
are platform-independent ABI breaks. Only the joint request is naturally
append-compatible; the implementation still rejects its old prefix.

## `struct_size`, output behavior, and source compatibility

`runtime/src/runtime_internal.hpp:204-208` accepts an output record only when
`record->struct_size >= sizeof(current T)` and its ABI version is 1. The same
current-size rule is open-coded for request records in `runtime.cpp`. It has two
important consequences:

- an old caller's smaller output record receives `OA_RUNTIME_EABI` before a
  current-sized assignment, so the old buffer is not overrun and remains
  unchanged;
- the library never emits the recognized old prefix and cannot satisfy an old
  V1 caller. No layout-specific compatibility path exists for inserted fields,
  and no input-prefix normalization/defaulting exists for the append-only joint
  request.

This is the reverse of the compatibility behavior needed when replacing a V1
library under a V1 client. The size header lets a provider dispatch by known
layout; it does not by itself make arbitrary field insertion compatible.

Most old source that only names old output fields will recompile with the new
larger types, but old zero-initialized joint/paired motion source then lacks the
new mandatory identity and collision-policy fields and fails semantically.
The persistence source break is direct: the old authorization macro is gone and
the second/third argument types are incompatible in C with warnings and errors
in C++, so an unchanged old call cannot compile against the current header.

## Old call convention, symbol, and canary

Both persistence declarations have three arguments and export the same plain C
symbol `oa_runtime_manifest_save`; C symbol names contain no type information.
On x86-64 SysV the old caller passes the manifest pointer in `RDI`, the absolute
path pointer in `RSI`, and the 32-bit magic value in `EDX`. The current callee
interprets `RSI` as an authority handle and `RDX` as a file-name pointer. Its
typed authority registry rejects the old path token and short-circuits before
using the bogus third pointer, so the observed result is safe failure, not
compatibility. Other ABIs differ in register/stack placement but retain the
same type/meaning mismatch.

An isolated strict-C11 old-header object was linked to the current installed
static archive and its current dependencies. It reported:

```text
old_summary_size=176 create=0x52000000 get_summary=0x52000002 size_after=176 save=0x52000001
canary_exit=0
```

Thus handle creation still succeeds, the old summary is rejected with
`OA_RUNTIME_EABI` and left at size 176, and the old persistence call resolves
but returns `OA_RUNTIME_EINVAL`. `nm` confirms only the unversioned base symbol.

The project installs only a static Runtime archive. An executable already
fully linked to the old archive embeds the old implementation and is unaffected
by replacing files; the relevant binary contract is an old compiled object
relinked to the newer archive (or the equivalent ABI if a shared build is later
produced). The canary tests exactly that contract.

## Package/version and release evidence

- The current header still defines `OA_RUNTIME_ABI_VERSION` as 1.
- `runtime/CMakeLists.txt` has been `project(openarm_runtime VERSION 1.0.0)`
  since introduction and emits `SameMajorVersion` package compatibility.
- `runtime/README.md` called the component the "stable ISO-C" boundary and
  "Runtime V1" in the original commit and still does so.
- `git tag` is empty; `git remote -v` is empty. There is no tag/release ledger
  proving an external 6c-era artifact, but absence of local tag/remote is not
  proof that no header or installed prefix was distributed.
- Reflogs show `6c89618`, `3a02c5b`, `4790241`, `df8803c`, `74e148d`, and the
  clean review commit `315f787` were cherry-picked onto `main` at
  `2026-07-29 17:27:55 -0700`; final installed acceptance followed at
  `e0d06c8`. This supports a batch-under-review interpretation.
- Current installed consumers compile only against the current installed
  header. They prove header/archive parity, not compatibility with an older V1
  header. Runtime has no analogue of Control's
  `openarm_control_v1_original_abi` target.

The safest conclusion is therefore: the ABI failure is proven, external
release of the old header is unproven, and the repository has not documented
the condition required to waive the advertised V1 contract.

## Persisted records are separate

The public records above are caller memory layouts; they are not overlaid on
manifest file bytes. The disk marker `OPENARM_RUNTIME_MANIFEST|1` is a separate
format version. The current parser deliberately accepts the original 17-line
plain SHA-256 record and the newer 18-line authenticated variants, so an old
plain manifest is forward-readable by the current library. The newer legacy
HMAC/V2 authentication lines and checkpoint authorization have their own
security semantics. That persistence compatibility does not repair the C ABI,
and `oa_runtime_manifest_summary` growth does not itself change the file
record. Conversely, an old library requiring exactly 17 lines cannot read a
new 18-line authenticated file; that is a format-direction issue, not evidence
that the in-memory V1 layouts are compatible.

## Smallest correct remedy

Do not merely relax `output_valid` to accept all smaller records: inserted
fields mean a blind prefix copy would put values at the wrong offsets, and old
motion inputs need explicit safe normalization.

If original V1 is supported, the smallest coherent design is:

1. Check in the exact `6c89618` public header (or a minimal exact copy) as an ABI
   fixture. Add compile-time size/alignment/offset assertions for every V1
   record on supported architectures and a strict C11 executable compiled only
   against that fixture, linked and run against the current installed archive.
2. Dispatch every modified record by exact recognized `struct_size`. For old
   outputs, fill a private frozen-layout struct field-by-field and never write
   past the caller size. For old motion inputs, parse the private old layout and
   derive/default the new internal identity policy in a documented fail-closed
   way, as Control already does for original V1 prefixes. Unknown intermediate
   sizes remain `EABI`.
3. Restore the original base symbol/declaration for
   `oa_runtime_manifest_save`. Put the authority-based operation under a new,
   unambiguous symbol (for example `oa_runtime_manifest_save_authenticated`;
   retain the existing checkpoint CAS as `oa_runtime_manifest_save_v2`). A
   public header cannot safely expose two incompatible prototypes for the same
   C symbol. If the original unkeyed save cannot be supported under the current
   security policy, keep the old ABI entry point explicitly deprecated and
   fail it with a documented non-success status, but recognize that this
   preserves link/call safety rather than successful behavioral compatibility.
4. Prefer new `_v2` record/function names for layouts whose old offsets cannot
   remain fixed. Keep `OA_RUNTIME_ABI_VERSION == 1` only on the genuinely
   compatible old surface; otherwise bump package major/ABI to 2 and use a V2
   symbol/type namespace while retaining V1 wrappers.

If authoritative project owners instead confirm that no Runtime API/artifact
before the clean final header was ever released, the smaller remedy is to
document that exact pre-release fact and freeze the final V1 now. Even then,
add a frozen-current-header consumer and layout manifest immediately; otherwise
the next same-version edit will recreate an unambiguous released-ABI defect.

No production source, build tree, hardware, CAN interface, or persisted product
data was modified for this diagnosis. The only repository change is this report;
the canary used `/tmp` and existing installed static archives.
