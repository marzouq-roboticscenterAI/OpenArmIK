# Independent runtime findings verification

Date: 2026-07-29 (America/Los_Angeles)  
Verified HEAD: `e0d06c86d3fe3cba7c2fc78c4c0b4926de384ad9`  
Reference report: `.swarm/final_whole_branch_review.md`

## Disposition

- **A — CONFIRMED (Important):** the physical inventory and physical
  configuration-preview APIs do not implement authoritative discovery or a
  manifest/evidence comparison. The public preview can positively validate a
  virtual inventory.
- **B — MIXED wording, core defect CONFIRMED (Important):** destroy cannot
  cancel an already-pinned physical query, and that query can continue sending
  after destroy returns for a programmed worst case of 224 seconds. The claim
  that `oa_runtime_destroy()` itself blocks for about 224 seconds is
  **DISPROVED**: destroy returns promptly; the query caller/thread remains
  blocked. Runtime object destruction is deferred safely by `shared_ptr`.

There are no runtime/transport changes between the reference report's reviewed
commit `315f787` and current HEAD. No GUI, CAN socket/interface, network service,
hardware, or real transmission was used. Evidence came from source inspection,
an in-memory linker-injected runtime transport, and the existing in-memory
transport `BlockingBackend` test with its SocketCAN-opening test call omitted.

## A. Inventory/fingerprint/preview semantics

### Reproduction

A public-API probe created the built-in virtual manifest/runtime/inventory and
passed that inventory to `oa_runtime_configuration_preview_physical()`. It
returned:

```text
motors=14 unresolved=0 ambiguous=0x0
preview_call=OA_RUNTIME_OK valid=1 validation=OA_RUNTIME_OK
identity=0 mapping=0 limits=0 armable=0
```

Thus an explicitly virtual inventory is accepted as a valid **physical**
preview. `would_be_armable=0` and unsupported physical apply mitigate immediate
motion risk, but do not make the evidence/preview claim truthful.

### Exact implemented semantics

1. `oa_runtime_inventory_query()` pins the runtime at
   `runtime/src/inventory.cpp:216`. For SocketCAN it accepts one interface name
   and up to 16 caller-supplied `(send_id, receive_id)` candidates.
2. If that interface is found, it appends exactly one
   `oa_runtime_motor_evidence` row for every requested candidate, even if no
   response was seen (`inventory.cpp:292-365`). Consequently
   `summary.motor_count` is candidate-result-row count, not observed unique
   motor count (`:367-370`). Duplicate candidates are accepted. A missing
   interface returns `OA_RUNTIME_OK` with an empty inventory.
3. Each candidate is queried for 14 fixed registers. A matching response must
   have the expected CAN receive ID, target send ID, operation, register ID,
   value type and valid value (`can/src/openarm_can.c:520-558`). That is valid
   per-frame correlation, but not proof of a unique physical responder.
4. The receive loop stops at the first matching response for a register
   (`inventory.cpp:351-356`). It therefore cannot count multiple responders
   using the same IDs. `maximum_received_frames` is cumulative across all 14
   registers for a candidate (`received` is declared at `:304`), not a fresh
   bound per register.
5. Every nonempty physical candidate row is unconditionally
   `OA_RUNTIME_EVIDENCE_AMBIGUOUS`, side/joint `UINT32_MAX`, and unresolved
   (`:301-303,360`). Every corresponding summary bit is unconditionally both
   unknown and ambiguous (`:378-381`). This is fail-closed, but it is a
   candidate probe rather than completed motor discovery.
6. `register_mismatch_mask`, `duplicate_count`, and `enabled_observed` are never
   assigned outside zero initialization. `summary.conflict_mask` is likewise
   never assigned. Self-reported configured IDs, bitrate, mode, versions,
   timeout, limits, gearing and direction are stored but never compared with a
   manifest or expected values.
7. Interfaces found by the physical path are labelled
   `OA_RUNTIME_INTERFACE_KIND_UNKNOWN`, not physical
   (`inventory.cpp:104-128`). `InventoryData` stores no source backend or
   provenance.
8. A query covers only one interface, while the built-in/manifest architecture
   uses two buses. There is no inventory-merge API. Nevertheless preview asks
   only for 14 rows and ignores interface count/name, so 14 candidate rows from
   one interface could satisfy its count test.
9. The physical fingerprint input is, per row,
   `QUERY-<interface>-<requested-send-id>-<serial>:<presence-mask>`
   (`inventory.cpp:361-385`). It omits expected/observed receive ID and every
   stored configuration value, conflict/duplicate/mismatch/fault state, and
   confidence. Configuration changes can therefore retain the same digest.
   Excluding query timestamps is not itself a defect for a stable identity
   fingerprint; the reference report overstates that part. The defect is that
   neither the documented domain nor the identity/configuration fields needed
   for safe comparison are bound.
10. `oa_runtime_configuration_preview_physical()` does not examine a single
    manifest motor or evidence row. It checks only
    `motor_count == 14`, `unresolved_assignment == 0`, and
    `ambiguous_mask == 0` (`inventory.cpp:440-458`). It ignores source backend,
    interface kind/count/name, fingerprint, presence, conflicts, all values,
    serials, mappings and limits. It never populates `changed_motor_mask`,
    `mapping_change_mask`, or `limit_change_mask`; invalid input merely sets all
    identity bits. Real physical-query results with any rows can never pass,
    while the virtual inventory passes.

### Severity and smallest safe correction

**Important contract/identity defect, currently safety-mitigated.** Physical
apply is always `OA_RUNTIME_EUNSUPPORTED`, preview always reports
`would_be_armable=0`, physical motion capabilities are absent, and query frames
are typed register reads. The result can still mislead callers and cannot serve
as commissioning authority.

The smallest release-safe change is to make
`oa_runtime_configuration_preview_physical()` initialize an invalid output and
return `OA_RUNTIME_EUNSUPPORTED` until physical association/comparison exists.
Do not infer physical provenance from the current rows or interface name.

If the query remains, describe it as a non-authoritative candidate register
probe. A real V2 evidence contract should distinguish requested candidates,
responses and unique observed motors; carry source backend/interface
provenance; require complete register presence; detect duplicate serials and
IDs; compare every manifest mapping/configuration field; populate exact
identity/mapping/limit/conflict masks; and hash a documented canonical set of
stable identity/configuration/evidence fields (not timestamps).

### Adversarial tests required

- Virtual inventory passed to physical preview must return unsupported/invalid.
- Fourteen synthetic clean-summary rows with one mismatched interface, serial,
  ID, firmware/config field, direction, limit or gear ratio must fail and set
  the exact mask.
- No-response candidates must not increase observed-motor count; duplicate
  candidate IDs, duplicate responder IDs and duplicate serials must conflict.
- Fourteen rows from one bus must not validate a two-bus manifest.
- Missing/malformed/stale/fault/partial registers must fail. Vary each stable
  fingerprint field independently and assert digest change; vary only
  timestamps and assert stability.
- Flood unrelated/stale frames through the receive budget and verify later
  register responses cannot be falsely represented as complete evidence.

## B. Destroy, cancellation, lifetime and deadline composition

### Reproduction

A linker-injected fake exposed one `fakecan` interface. Its send counted frames;
receive waited to the supplied deadline and returned timeout. One candidate and
`per_query_timeout_ns=5,000,000` were queried on a worker thread. Immediately
after the first send, the main thread destroyed the runtime:

```text
destroy_us=3 sends_at_destroy=1 sends_final=14 query_ms=71
query_status=OA_RUNTIME_OK summary_status=OA_RUNTIME_OK motor_count=1
```

No CAN or network API was called. This demonstrates all material lifecycle
facts: destroy did not block, did not close the query's transport, future sends
continued after destroy returned, and the query published an `OK` inventory
against an already-destroyed runtime handle.

The existing injected `BlockingBackend` transport test also passed. It starts
blocked send and receive operations, calls `Transport::close()`, and proves both
return `OA_TRANSPORT_ECLOSED` within one second. The production SocketCAN
backend similarly writes an `eventfd` wake signal and waits for operation locks
(`transport/src/socketcan_backend.cpp:214-220`); `Transport::close()` is a
completed-I/O barrier (`transport/src/transport.cpp:343-353`). The needed lower
mechanism already exists but runtime destroy has no reference to the local
query transport.

### Exact bound and races

- There are at most 16 candidates and exactly 14 register queries per
  candidate. Each iteration computes one deadline of
  `before + per_query_timeout_ns`; send and all receives share it
  (`inventory.cpp:323-337`). Therefore the programmed loop deadline sum on a
  silent, writable bus is `16 * 14 * 1 s = 224 s`, plus interface enumeration,
  open/close, CPU and scheduler overhead. `maximum_received_frames` does **not**
  multiply this bound because receives reuse the same per-register deadline.
- `oa_runtime_destroy()` only pins, sets `closing`, wakes the virtual worker and
  erases the registry handle (`runtime.cpp:555-564`). It neither knows nor
  closes the query-local transport.
- The query pins a `shared_ptr<RuntimeData>` before destroy. Erasing the handle
  cannot cause use-after-free: the object destructor and worker/controller
  cleanup are deferred until the query releases that pin. This is safe lifetime
  management, but not cancellation.
- If destroy wins before query pinning, query returns `EINVAL` and sends
  nothing. If query pins first—even if it has not opened yet—it never checks
  `closing`, can open and transmit after destroy, then insert an inventory and
  update the closed runtime's inventory revision before returning `OK`.
- Eventually the query's RAII owner closes/destroys its transport. Thus there
  is no transport leak; the defect is unbounded composed latency and authority
  continuing beyond runtime teardown.

### Severity and smallest safe correction

**Important lifecycle/authority defect, with limited frame hazard.** Continuing
TX after lifecycle teardown violates caller authority. The transport rejects
motion and the runtime only constructs typed register queries, so this is not a
physical-motion path.

Add a runtime-owned, mutex-protected active-query state before opening the
transport. It should contain a cancellation flag and the current transport
pointer. Destroy must atomically set `closing/cancelled`, call
`oa_transport_close()` on any active transport, and rely on close's I/O barrier
before returning. Query must check cancellation/closing before open, after
registration, before every send, after every send/receive, and before inventory
publication; cancellation returns `OA_RUNTIME_ESTATE` with a null output. Use a
shared active-operation object or equivalent synchronization so destroy never
dereferences a transport concurrently with query cleanup. Reject a second
physical query with `EBUSY` unless multi-query ownership is deliberately
implemented.

An ABI-compatible lifecycle fix does not require waiting 224 seconds: closing
the transport wakes current I/O and makes later sends fail immediately. A V2
query should additionally accept a query-wide deadline/cancellation token so
ordinary callers have a bound much smaller than the sum of 224 independent
deadlines.

### Adversarial tests required

- Inject blocking backends and destroy at: before open, during open handoff,
  during send, during receive, between registers/candidates, and immediately
  before publication. Assert no send starts after destroy returns, query returns
  `ESTATE`, and output remains null.
- Make close wait for an in-flight operation to acknowledge cancellation;
  assert destroy does not return before that completed-I/O barrier.
- With a fake clock, exercise 16 silent candidates and verify exactly 224
  shared send/receive deadlines rather than sleeping; separately verify the
  query-wide deadline truncates the operation.
- Race repeated query/destroy operations under TSAN; cover double destroy,
  second-query `EBUSY`, close idempotence, allocation failure, and transport
  error cleanup.
- Queue continuous malformed/irrelevant frames and maximum receive budgets;
  cancellation must take precedence over frame processing and publication.

## Should physical query be removed until hardware exists?

**Yes for a release capability, unless explicitly compiled/labelled as an
experimental non-authoritative diagnostic.** The smallest honest release gate
is to stop advertising `OA_RUNTIME_CAP_PHYSICAL_REGISTER_QUERY` and return
`OA_RUNTIME_EUNSUPPORTED` for the SocketCAN inventory path until the lifecycle
fix, evidence redesign, and supervised two-bus hardware qualification exist.
The typed codecs, query-only transport and injected tests can remain. Keeping a
public “physical inventory” capability without hardware qualification is not
just an evidence gap: its present API cannot detect unique responders, merge
the two intended buses, or ever produce a validating real inventory.
