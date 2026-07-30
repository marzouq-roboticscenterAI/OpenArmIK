#!/usr/bin/env bash
set -euo pipefail
root_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$root_dir/scripts/lib/description_pin.sh"
work_root=$(mktemp -d "${TMPDIR:-/tmp}/openarm-pin-test.XXXXXX")
cleanup() { rm -rf -- "$work_root"; }
trap cleanup EXIT

make_fixture() {
  local repository=$1
  mkdir -p "$repository"
  git -C "$repository" init -q
  git -C "$repository" config user.name fixture
  git -C "$repository" config user.email fixture@example.invalid
  printf '<package/>\n' > "$repository/package.xml"
  printf 'source\n' > "$repository/input.txt"
  printf 'ignored.tmp\n' > "$repository/.gitignore"
  git -C "$repository" add .
  git -C "$repository" commit -qm fixture
  git -C "$repository" remote add origin https://example.invalid/description.git
  git -C "$repository" checkout -q --detach
}

validate_fixture() {
  local repository=$1
  openarm_validate_description_repository "$repository" \
    "$(git -C "$repository" rev-parse HEAD)" \
    https://example.invalid/description.git \
    "$(git -C "$repository" rev-parse 'HEAD^{tree}')" 1 none
}

fixture="$work_root/fixture"
make_fixture "$fixture"
validate_fixture "$fixture"

set +e
openarm_validate_description_repository "$fixture" "$(printf '0%.0s' {1..40})" \
  https://example.invalid/description.git "$(git -C "$fixture" rev-parse 'HEAD^{tree}')" \
  1 none >/dev/null 2>&1
[[ $? == 1 ]]
git -C "$fixture" remote set-url origin https://wrong.invalid/repo.git
validate_fixture "$fixture" >/dev/null 2>&1
[[ $? == 1 ]]
git -C "$fixture" remote set-url origin https://example.invalid/description.git
printf 'dirty\n' >> "$fixture/input.txt"
validate_fixture "$fixture" >/dev/null 2>&1
[[ $? == 1 ]]
git -C "$fixture" restore input.txt
printf 'staged\n' >> "$fixture/input.txt"
git -C "$fixture" add input.txt
validate_fixture "$fixture" >/dev/null 2>&1
[[ $? == 1 ]]
git -C "$fixture" reset -q HEAD -- input.txt
git -C "$fixture" restore input.txt
printf 'extra\n' > "$fixture/untracked.txt"
validate_fixture "$fixture" >/dev/null 2>&1
[[ $? == 1 ]]
rm "$fixture/untracked.txt"
printf 'ignored\n' > "$fixture/ignored.tmp"
validate_fixture "$fixture" >/dev/null 2>&1
[[ $? == 1 ]]
rm "$fixture/ignored.tmp"
git -C "$fixture" update-index --assume-unchanged input.txt
printf 'hidden\n' >> "$fixture/input.txt"
validate_fixture "$fixture" >/dev/null 2>&1
[[ $? == 1 ]]
git -C "$fixture" update-index --no-assume-unchanged input.txt
git -C "$fixture" restore input.txt
set -e

linked="$work_root/linked-worktree"
git -C "$fixture" worktree add -q --detach "$linked" HEAD
validate_fixture "$linked"
git -C "$fixture" worktree remove -f "$linked"
ln -s "$fixture" "$work_root/symlink"
if validate_fixture "$work_root/symlink" >/dev/null 2>&1; then
  printf 'Symlinked checkout unexpectedly accepted\n' >&2
  exit 1
fi

submodule_fixture="$work_root/submodule-fixture"
make_fixture "$submodule_fixture"
git -C "$submodule_fixture" switch -q --detach HEAD
printf '[submodule "unexpected"]\n\tpath = unexpected\n\turl = ../unexpected\n' \
  > "$submodule_fixture/.gitmodules"
git -C "$submodule_fixture" add .gitmodules
git -C "$submodule_fixture" -c user.name=fixture -c user.email=fixture@example.invalid \
  commit -qm submodule-policy
if validate_fixture "$submodule_fixture" >/dev/null 2>&1; then
  printf 'Unexpected submodule declaration accepted\n' >&2
  exit 1
fi

printf '%s\n' 'Description pin regression passed'
