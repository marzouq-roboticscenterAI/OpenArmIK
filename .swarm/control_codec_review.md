# Independent review: DaMiao control codecs

Commit: `0c275c1` versus parent `36d2316`  
Verdict: **CLEAN**

No Critical, Important, or Minor correctness findings were found in the scoped
codec, public ABI, documentation, tests, or fake-transport changes.

## Review coverage

- Read `SKILL.md`, `.swarm/hardware_protocol_recon.md`,
  `.swarm/hardware_protocol_verify.md`, `.swarm/controller_design_final.md`, and
  `.swarm/control_codec_impl.md` completely where applicable to the change.
- Compared the implementation with pinned upstream `openarm_can` RID metadata,
  request/response parsing, MIT/POS_VEL/POS_FORCE packing, and commissioning
  command builders.
- Verified MIT bit packing and offset-binary scaling, POS_VEL float32 LE layout,
  POS_FORCE float32/u16 LE layout, register request/write/response layout, RID
  values and U32/F32 classifications, special opcodes, status/fault handling,
  temperature preservation, and standard-ID/DLC/flag rejection.
- Verified dynamic profiles correlate PMAX/VMAX/TMAX operation, RID, wire type,
  target send ID, receive ID, and raw/value consistency; motion and feedback APIs
  additionally bind the resulting profile to the relevant IDs.
- Verified new motion APIs reject non-finite and out-of-profile values and do
  not saturate. The legacy explicit saturation API remains unchanged for ABI and
  source compatibility, including its saturation-result mask.
- Verified no guessed gearbox conversion, electrical-current measurement, safe
  torque/thermal limit, timeout unit, home position, or joint sign is introduced.
  POS_FORCE current is correctly described as per-unit, and PVT values remain
  codec mapping spans rather than robot safety limits.
- Verified all pre-existing public records and function signatures are unchanged,
  `OA_CAN_ABI_VERSION` remains 2, and every pre-existing exported symbol remains
  present. New public enum-like types use fixed-width integers.
- Verified the fake accepts only read-only refresh/query diagnostics plus disable;
  it rejects MIT, POS_VEL, register write, set-zero, clear-error, and flash-save
  frames while never reporting torque enabled.

## Fresh verification

- GCC 15.2 Release configure/build and `ctest`: **PASS**, 1/1.
- Debug ASan+UBSan build and tests with leak detection and halt-on-error:
  **PASS**, 1/1.
- Exported-symbol and header-diff audit: all parent API symbols and record layouts
  retained; additions are append-only at the API surface.
- `git diff --check 0c275c1^ 0c275c1`: one trailing-space warning in
  `.swarm/control_codec_impl.md:3`; documentation-only and below review severity.

No CAN socket, interface mutation, or hardware operation was performed.
