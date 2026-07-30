#!/usr/bin/env bash
# Offline identity validation for the production description input. This file
# is sourced by builders and tests; it never fetches or mutates the checkout.

OPENARM_DESCRIPTION_COMMIT=6c7b720f1ba48e8bafa3a3dc752c45f397b42221
OPENARM_DESCRIPTION_ORIGIN=https://github.com/enactic/openarm_description.git
OPENARM_DESCRIPTION_TREE=a05f0116710a4948e9f769237696fbf43701762f

openarm_validate_description_repository() {
  local repository=$1 expected_commit=$2 expected_origin=$3 expected_tree=$4
  local require_detached=${5:-1} submodule_policy=${6:-none}
  local actual_commit actual_tree branch origin_lines origin object_type
  local entry metadata path mode type object actual_object

  [[ ! -L "$repository" ]] || {
    printf 'Pinned description checkout must not be a symlink: %s\n' "$repository" >&2
    return 1
  }
  repository=$(realpath -e -- "$repository" 2>/dev/null) || {
    printf 'Pinned description checkout is missing: %s\n' "$repository" >&2
    return 1
  }
  [[ -d "$repository" ]] &&
    git -C "$repository" rev-parse --is-inside-work-tree >/dev/null 2>&1 || {
      printf 'Pinned description is not a Git worktree: %s\n' "$repository" >&2
      return 1
    }
  actual_commit=$(git -C "$repository" rev-parse --verify HEAD 2>/dev/null) || {
    printf 'Pinned description has no locally available HEAD: %s\n' "$repository" >&2
    return 1
  }
  [[ "$actual_commit" == "$expected_commit" ]] || {
    printf 'Pinned description HEAD mismatch: expected %s, found %s\n' \
      "$expected_commit" "$actual_commit" >&2
    return 1
  }
  git -C "$repository" cat-file -e "$expected_commit^{commit}" 2>/dev/null || {
    printf 'Pinned description commit object is unavailable locally: %s\n' \
      "$expected_commit" >&2
    return 1
  }
  object_type=$(git -C "$repository" cat-file -t "$expected_commit" 2>/dev/null)
  [[ "$object_type" == commit ]] || {
    printf 'Pinned description object is not a commit: %s\n' "$expected_commit" >&2
    return 1
  }
  actual_tree=$(git -C "$repository" rev-parse "$expected_commit^{tree}") || return 1
  [[ "$actual_tree" == "$expected_tree" ]] || {
    printf 'Pinned description tree mismatch: expected %s, found %s\n' \
      "$expected_tree" "$actual_tree" >&2
    return 1
  }

  origin_lines=$(git -C "$repository" remote get-url --all origin 2>/dev/null || true)
  [[ "$origin_lines" == "$expected_origin" ]] || {
    printf 'Pinned description origin mismatch: expected %s, found %s\n' \
      "$expected_origin" "${origin_lines:-none}" >&2
    return 1
  }
  origin=$(git -C "$repository" remote get-url origin 2>/dev/null || true)
  [[ "$origin" == "$expected_origin" ]] || return 1
  if ((require_detached)); then
    branch=$(git -C "$repository" symbolic-ref -q --short HEAD 2>/dev/null || true)
    [[ -z "$branch" ]] || {
      printf 'Pinned description must be detached, found branch: %s\n' "$branch" >&2
      return 1
    }
  fi

  git -C "$repository" diff --quiet --no-ext-diff --ignore-submodules=none -- || {
    printf 'Pinned description has tracked worktree changes\n' >&2
    return 1
  }
  git -C "$repository" diff --cached --quiet --no-ext-diff \
    --ignore-submodules=none -- || {
    printf 'Pinned description has staged changes\n' >&2
    return 1
  }
  [[ -z $(git -C "$repository" ls-files --others --exclude-standard) ]] || {
    printf 'Pinned description has untracked files\n' >&2
    return 1
  }
  [[ -z $(git -C "$repository" ls-files --others --ignored --exclude-standard) ]] || {
    printf 'Pinned description has ignored additions\n' >&2
    return 1
  }

  case "$submodule_policy" in
    none)
      [[ ! -e "$repository/.gitmodules" ]] || {
        printf 'Pinned description unexpectedly declares submodules\n' >&2
        return 1
      }
      [[ -z $(git -C "$repository" ls-files -s | awk '$1 == "160000" {print}') ]] || {
        printf 'Pinned description unexpectedly contains gitlinks\n' >&2
        return 1
      }
      [[ -z $(git -C "$repository" submodule status --recursive 2>/dev/null) ]] || {
        printf 'Pinned description unexpectedly has submodule state\n' >&2
        return 1
      }
      ;;
    *)
      printf 'Unsupported description submodule policy: %s\n' "$submodule_policy" >&2
      return 2
      ;;
  esac

  # Compare every actual tracked byte to the pinned tree. This deliberately
  # does not trust index stat caches, assume-unchanged, or skip-worktree bits.
  while IFS= read -r -d '' entry; do
    metadata=${entry%%$'\t'*}
    path=${entry#*$'\t'}
    read -r mode type object <<<"$metadata"
    [[ "$type" == blob && "$mode" != 120000 ]] || {
      printf 'Pinned description has an unexpected tracked object: %s (%s %s)\n' \
        "$path" "$mode" "$type" >&2
      return 1
    }
    [[ -f "$repository/$path" && ! -L "$repository/$path" ]] || {
      printf 'Pinned description tracked file is missing or symlinked: %s\n' "$path" >&2
      return 1
    }
    actual_object=$(git -C "$repository" hash-object --no-filters -- "$repository/$path") ||
      return 1
    [[ "$actual_object" == "$object" ]] || {
      printf 'Pinned description content mismatch: %s\n' "$path" >&2
      return 1
    }
  done < <(git -C "$repository" ls-tree -rz --full-tree "$expected_commit")

  [[ -f "$repository/package.xml" ]] || {
    printf 'Pinned description is missing package.xml\n' >&2
    return 1
  }
}

openarm_validate_description_pin() {
  openarm_validate_description_repository "$1" \
    "$OPENARM_DESCRIPTION_COMMIT" "$OPENARM_DESCRIPTION_ORIGIN" \
    "$OPENARM_DESCRIPTION_TREE" 1 none
}
