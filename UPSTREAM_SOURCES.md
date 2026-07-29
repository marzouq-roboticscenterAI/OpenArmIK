# Vendored upstream OpenArm sources

Ingested 2026-07-28 from the ten repositories listed by the canonical OpenArm hub at <https://github.com/enactic/openarm#-repositories>. All origins use the canonical `enactic` organization. These are full, unfiltered, non-shallow Git clones, with clean worktrees detached at the audited commits below.

No repository code, Git hook, setup utility, hardware command, CAN command, motor command, or firmware was executed. The unlicensed `dmBots/motor-firmware` repository and unrelated similarly named projects were not cloned.

## Active source manifest

`Size` is the local clone/worktree size reported by `du -sb` after checkout. `Source ref` records the remote default branch from which the audited commit was selected; every local checkout is detached.

| Local path | Canonical origin | Audited detached commit | Source ref / exact tag | License | Size (bytes) |
|---|---|---|---|---|---:|
| `upstream/openarm` | <https://github.com/enactic/openarm.git> | `990fda921c82ae9d12b00f23e449793a9a313afd` | `origin/main`; no exact tag | Apache-2.0, `LICENSE` | 685,403,092 |
| `upstream/openarm_hardware` | <https://github.com/enactic/openarm_hardware.git> | `12c07510c09b2c10b7dfe48010dae5c05cbe887f` | `origin/main`; no exact tag | CERN-OHL-S-2.0, `LICENSE.txt` | 185,125,151 |
| `upstream/openarm_description` | <https://github.com/enactic/openarm_description.git> | `6c7b720f1ba48e8bafa3a3dc752c45f397b42221` | `origin/main`; no exact tag | Apache-2.0, `LICENSE.txt` | 249,113,169 |
| `upstream/openarm_can` | <https://github.com/enactic/openarm_can.git> | `c32ecd31da267967f0c913c2118c843177d88b91` | `origin/main`; no exact tag | Apache-2.0, `LICENSE.txt` | 764,143 |
| `upstream/openarm_ros2` | <https://github.com/enactic/openarm_ros2.git> | `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` | `origin/main`; no exact tag | Apache-2.0, `LICENSE` | 114,893,857 |
| `upstream/openarm_teleop` | <https://github.com/enactic/openarm_teleop.git> | `eb2d49338bf70ace95282ea724903849397b7811` | `origin/main`; no exact tag | Apache-2.0, `LICENSE.txt` | 257,341 |
| `upstream/openarm_isaac_lab` | <https://github.com/enactic/openarm_isaac_lab.git> | `bad82e23716e6941c2de78ccb978f57c78b37734` | `origin/main`; no exact tag | Apache-2.0, `LICENSE.txt` | 264,478,323 |
| `upstream/openarm_mujoco` | <https://github.com/enactic/openarm_mujoco.git> | `8955afb54e4adfb59a236e2b4d15192b7a02865c` | `origin/master`; exact tag `2.0.1` | Apache-2.0, `LICENSE` | 177,726,349 |
| `upstream/openarm_dataset` | <https://github.com/enactic/openarm_dataset.git> | `2da1062f524a5b240e61e1031f637d765076569d` | `origin/main`; exact tag `0.4.0` | Apache-2.0, `LICENSE.txt` | 94,453,735 |
| `upstream/dora-openarm` | <https://github.com/enactic/dora-openarm.git> | `d988dd24537d670f07b6d6e85e0cdd25f0b05b82` | `origin/main`; no exact tag | Apache-2.0, `LICENSE` | 133,514 |

## Integrity and repository state

The following checks passed for all ten repositories:

- `git rev-parse --is-shallow-repository` returned `false`.
- `git status --porcelain` returned no changes or untracked files.
- `git branch --show-current` returned empty because each checkout is intentionally detached.
- The canonical `origin` URL and checked-out commit match the table.
- `git submodule update --init --recursive` completed. None of the ten audited commits declares a `.gitmodules` entry, so there are zero initialized submodules at this snapshot.
- `git lfs ls-files` returned zero paths for every repository. No Git LFS objects are declared at these audited commits.

Full-history commit counts at ingest were: `openarm` 480, `openarm_hardware` 44, `openarm_description` 49, `openarm_can` 104, `openarm_ros2` 157, `openarm_teleop` 11, `openarm_isaac_lab` 7, `openarm_mujoco` 93, `openarm_dataset` 53, and `dora-openarm` 18. These counts cover every ref fetched by a normal full clone and are recorded as an additional non-shallow-clone check, not as immutable upstream properties.

## Release alternatives retained in history

The active manifest deliberately uses one coherent current audited baseline; it does not mix older package releases into the working snapshot. The full clones retain these release alternatives for comparison:

| Repository | Release alternative | Peeled release commit |
|---|---|---|
| `openarm` | `1.1` (“OpenArm 01: Release No.2”) | `5fc656ab096b49115528e64dd19a0c08f43e9bb3` |
| `openarm_hardware` | `1.0.1` | `0403835afae64949ede85e440e86a283b747b9dd` |
| `openarm_description` | `1.0.4` | `5db5232d4bbf7396222437a568c625176bac1139` |
| `openarm_can` | `1.2.9` | `7549401e65366d89cfed793fa6038490eec94efb` |
| `openarm_ros2` | `0.9.2` | `73ef89838763496b94da30ede38fc92218bea18e` |
| `openarm_mujoco` | `2.0.1` | `8955afb54e4adfb59a236e2b4d15192b7a02865c` (active) |
| `openarm_dataset` | `0.4.0` | `2da1062f524a5b240e61e1031f637d765076569d` (active) |

`openarm_teleop`, `openarm_isaac_lab`, and the audited `dora-openarm` HEAD had no exact release tag observed at ingestion.

## Transfer record

The first concurrent full-clone attempts for `openarm` and `openarm_isaac_lab` terminated before producing a checkout; Git removed their incomplete destination directories, and the execution layer surfaced no terminal error text beyond the missing checkout. After the other original transfers completed, both repositories were retried one at a time as full `--recurse-submodules` clones and succeeded at the commits in the active manifest. There are no final clone failures or incomplete repositories.
