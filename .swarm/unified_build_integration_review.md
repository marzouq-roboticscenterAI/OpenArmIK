# Final unified-build integration review

Reviewed tree: `main` at `25b2735b41a1576b60f5ea4f89ba2ee2bae4fb60`

Date: 2026-07-29 (America/Los_Angeles)

Verdict: **CLEAN**

## Resolved prior finding

The prior MEDIUM installed-consumer cache finding is resolved. A fresh,
space-containing two-prefix regression exited 0:

```bash
./tests/test_native_prefix_reuse.sh \
  '/tmp/openarmik consumer cache final review'
```

The regression built and tested prefix A, reused the native build tree for
prefix B, and reported the expected native results in both rounds: CAN 1/1,
model 4/4, commission 2/2, transport 3/3, and control 4/4 (14/14 total).

After the B round, independent inspection found no prefix-A path in any
`CMakeCache.txt` below the reused native build tree. In particular:

- the installed all-header consumer's `CMAKE_PREFIX_PATH` and all five package
  directories (`OpenArmCan`, `openarm_model`, `openarm_commission`,
  `OpenArmTransport`, and `openarm_control`) resolve to prefix B;
- the transport installed-consumer cache resolves its external OpenArmCan
  dependency to B; and
- the control installed public-header consumer cache resolves its external
  `openarm_model` dependency to B.

The subordinate consumers' own staged package directories are newly generated
inside the B-round build tree. None of their cache, dependency, compile, or
link artifacts contains prefix A.

Both strict installed consumers were rebuilt with verbose commands and run:

```text
openarm_installed_c11   C11    exit 0
openarm_installed_cxx17 C++17  exit 0
```

Their include flags reference only `prefix-b/include`. Each exact link command
references only prefix-B archives and contains all five production libraries:
`libopenarm_can.a`, `libopenarm_model.a`, `libopenarm_commission.a`,
`libopenarm_transport.a`, and `libopenarm_control.a`. Prefix A is absent from
both consumers' cache, flags, dependency files, object metadata, and link
commands.

## Fresh unified-build cross-check

This fresh space-containing Release build also exited 0:

```bash
./scripts/build.sh --tests \
  --output-root '/tmp/openarmik consumer cache unified final' \
  --build-type Release
```

It built, linked, and ran both installed C11/C++17 all-header consumers. Native
CTest results were 14/14 passed with registrations of 1 CAN, 4 model,
2 commission, 3 transport, and 4 control tests.

Fresh enumeration under `build/openarm_ik_ros` registered exactly eight ROS
tests:

1. `test_paired_transaction`
2. `test_generated_urdf`
3. `test_no_can_linkage`
4. `test_ros_contract`
5. `test_invalid_expiry_parameter`
6. `test_close_rviz_window_help`
7. `test_close_rviz_window_invalid_pid`
8. `test_close_rviz_window_invalid_timeout`

## Archive boundaries

Independent demangled, defined-symbol inspection of the fresh installed
Release archives found:

- no `oa_control_test_*` definitions in `libopenarm_control.a`;
- no `openarm::commission::test::*` definitions in
  `libopenarm_commission.a`; and
- no embedded `oa_can_*` definitions in `libopenarm_transport.a`.

The separate non-installed control and commission test archives retain their
expected hook definitions, confirming that test hooks remain isolated from the
production libraries.

## Safety and workspace notes

- `git diff --check` passed after the report update.
- No GUI, CAN interface/socket, network, hardware, sudo, or system install was
  used.
- Existing unrelated untracked `.swarm` files were observed and left untouched.
- The only repository write from this review is this report.

## Disposition

**CLEAN.** The consumer cache fix makes the permanent A-to-B regression a valid
prefix-B SDK proof. Native tests, installed C11/C++17 compilation and linkage,
ROS registration, and production archive isolation all pass on current `main`.
