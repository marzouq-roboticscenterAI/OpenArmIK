#!/usr/bin/env bash
# Shared launch freshness, stamping, and authority checks. This file is sourced.

OPENARM_LAUNCH_STAMP_NAME=.openarm-launch-stamp-v2
OPENARM_LEGACY_LAUNCH_STAMP_NAME=.openarm-launch-stamp-v1
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/build_cache_state.sh"

openarm_compute_launch_source_fingerprint() {
  local root_dir=$1 description_dir=$2 build_type=${3:-Release}
  local run_tests=${4:-0} coverage=${OPENARM_IK_ROS_COVERAGE:-0}
  local manifest paths path file_mode file_hash description_commit description_tree
  local tool tool_path tool_real tool_hash tool_version variable value root_real
  local link_text link_real link_mode generator backend toolchain toolchain_real
  local request_digest
  manifest=$(mktemp "${TMPDIR:-/tmp}/openarmik-fingerprint.XXXXXX") || return 1
  paths=$(mktemp "${TMPDIR:-/tmp}/openarmik-fingerprint-paths.XXXXXX") || {
    rm -f -- "$manifest"
    return 1
  }
  git -C "$root_dir" ls-files -z -- can model commission transport control \
    runtime ros2_ws/src scripts tests run.sh > "$paths" || {
      rm -f -- "$manifest" "$paths"
      return 1
    }
  (
    cd "$root_dir"
    find can model commission transport control runtime ros2_ws/src scripts tests \
      \( -type f -o -type l \) -print0
    [[ ! -f run.sh && ! -L run.sh ]] || printf 'run.sh\0'
  ) >> "$paths" || { rm -f -- "$manifest" "$paths"; return 1; }
  root_real=$(realpath -e -- "$root_dir") || { rm -f -- "$manifest" "$paths"; return 1; }
  (
    printf 'OPENARM_LAUNCH_INPUT_V2\0build_type\0%s\0tests\0%s\0coverage\0%s\0' \
      "$build_type" "$run_tests" "$coverage"
    request_digest=$(openarm_build_state_requested_digest launch "$build_type" \
      "$run_tests" "$coverage" '') || exit 1
    printf 'build_request\0%s\0' "$request_digest"
    while IFS= read -r -d '' path; do
      case "$path" in
        */__pycache__/*|*.pyc|*/.pytest_cache/*|*/.coverage|*/coverage_html/*) continue ;;
      esac
      printf 'path\0%s\0' "$path"
      if [[ -L "$root_dir/$path" ]]; then
        link_text=$(readlink -- "$root_dir/$path") || exit 1
        link_real=$(realpath -e -- "$root_dir/$path") || {
          printf 'Broken source symlink: %s\n' "$root_dir/$path" >&2
          exit 1
        }
        case "$link_real" in
          "$root_real"/*) ;;
          *)
            printf 'Source symlink escapes repository: %s -> %s\n' \
              "$root_dir/$path" "$link_real" >&2
            exit 1
            ;;
        esac
        [[ -f "$link_real" ]] || {
          printf 'Source directory/special symlinks are unsupported: %s\n' \
            "$root_dir/$path" >&2
          exit 1
        }
        link_mode=$(stat -c '%a' -- "$link_real") || exit 1
        file_hash=$(sha256sum -- "$link_real") || exit 1
        printf 'symlink\0%s\0%s\0%s\0%s\0' "$link_text" "$link_real" \
          "$link_mode" "${file_hash%% *}"
      elif [[ -f "$root_dir/$path" ]]; then
        file_mode=$(stat -c '%a' -- "$root_dir/$path") || exit 1
        file_hash=$(sha256sum -- "$root_dir/$path") || exit 1
        printf 'file\0%s\0%s\0' "$file_mode" "${file_hash%% *}"
      else
        printf 'missing\0'
      fi
    done < <(sort -zu "$paths")
    description_commit=$(git -C "$description_dir" rev-parse --verify HEAD) || exit 1
    description_tree=$(git -C "$description_dir" rev-parse 'HEAD^{tree}') || exit 1
    printf 'description_commit\0%s\0description_tree\0%s\0' \
      "$description_commit" "$description_tree"
    for tool in cmake cc c++ /usr/bin/python3 colcon /opt/ros/lyrical/bin/xacro; do
      if [[ "$tool" == /* ]]; then
        tool_path=$tool
      else
        tool_path=$(command -v -- "$tool") || exit 1
      fi
      tool_real=$(realpath -e -- "$tool_path") || exit 1
      tool_hash=$(sha256sum -- "$tool_real") || exit 1
      case "$tool" in
        colcon)
          tool_version=$(/usr/bin/python3 -c \
            'import importlib.metadata as m; print(m.version("colcon-core"))') || exit 1
          ;;
        /opt/ros/lyrical/bin/xacro)
          tool_version=$(/usr/bin/python3 -c \
            'import importlib.metadata as m; print(m.version("xacro"))') || exit 1
          ;;
        *) tool_version=$("$tool_real" --version 2>&1 | sed -n '1p') || exit 1 ;;
      esac
      printf 'tool\0%s\0%s\0%s\0%s\0' "$tool" "$tool_real" \
        "$tool_version" "${tool_hash%% *}"
    done
    openarm_build_state_record_command CC "${CC:-cc}" || exit 1
    openarm_build_state_record_command CXX "${CXX:-c++}" || exit 1
    openarm_build_state_record_command AR "${AR:-ar}" || exit 1
    openarm_build_state_record_command RANLIB "${RANLIB:-ranlib}" || exit 1
    openarm_build_state_record_command LD "${LD:-ld}" || exit 1
    openarm_build_state_record_command NM "${NM:-nm}" || exit 1
    openarm_build_state_record_command STRIP "${STRIP:-strip}" || exit 1
    openarm_build_state_record_command OBJCOPY "${OBJCOPY:-objcopy}" || exit 1
    generator=${CMAKE_GENERATOR:-Unix Makefiles}
    if [[ -n ${CMAKE_MAKE_PROGRAM:-} ]]; then
      backend=$CMAKE_MAKE_PROGRAM
    else
      case "$generator" in
        *Ninja*) backend=ninja ;;
        *Makefiles*) backend=make ;;
        *)
          printf 'CMAKE_MAKE_PROGRAM is required for generator: %s\n' \
            "$generator" >&2
          exit 1
          ;;
      esac
    fi
    openarm_build_state_record_command CMAKE_MAKE_PROGRAM "$backend" || exit 1
    printf 'generator\0%s\0' "$generator"
    toolchain=${CMAKE_TOOLCHAIN_FILE:-}
    if [[ -n "$toolchain" ]]; then
      [[ "$toolchain" == /* && -f "$toolchain" ]] || {
        printf 'CMAKE_TOOLCHAIN_FILE must be an existing absolute file: %s\n' \
          "$toolchain" >&2
        exit 1
      }
      toolchain_real=$(realpath -e -- "$toolchain") || exit 1
      file_hash=$(sha256sum -- "$toolchain_real") || exit 1
      printf 'toolchain\0%s\0%s\0' "$toolchain_real" "${file_hash%% *}"
    else
      printf 'toolchain\0none\0'
    fi
    for variable in CFLAGS CXXFLAGS CPPFLAGS LDFLAGS CMAKE_GENERATOR_PLATFORM \
      CMAKE_GENERATOR_TOOLSET CMAKE_GENERATOR_INSTANCE ROS_DISTRO ROS_VERSION \
      ROS_PYTHON_VERSION; do
      value=${!variable-}
      printf 'env\0%s\0%s\0' "$variable" "$value"
    done
    for variable in CMAKE_C_COMPILER_LAUNCHER CMAKE_CXX_COMPILER_LAUNCHER \
      CMAKE_C_LINKER_LAUNCHER CMAKE_CXX_LINKER_LAUNCHER; do
      value=${!variable-}
      openarm_build_state_record_cmake_launcher launch_launcher "$variable" \
        "$value" || exit 1
    done
    file_hash=$(sha256sum -- /opt/ros/lyrical/setup.bash) || exit 1
    printf 'ros_prefix\0/opt/ros/lyrical\0setup\0%s\0' "${file_hash%% *}"
  ) > "$manifest" || { rm -f -- "$manifest" "$paths"; return 1; }
  sha256sum -- "$manifest" | awk '{print $1}'
  rm -f -- "$manifest" "$paths"
}

openarm_compute_install_manifest_digest() {
  local output_root=$1 install_root="$1/install" manifest entries entry relative mode hash
  local install_real install_mode link_text link_real target_mode unsupported
  [[ -d "$install_root" && ! -L "$install_root" ]] || return 1
  install_real=$(realpath -e -- "$install_root") || return 1
  unsupported=$(find "$install_root" -mindepth 1 ! -type f ! -type l ! -type d \
    -print -quit) || return 1
  [[ -z "$unsupported" ]] || {
    printf 'Unsupported special file in install tree: %s\n' "$unsupported" >&2
    return 1
  }
  manifest=$(mktemp "${TMPDIR:-/tmp}/openarmik-install-manifest.XXXXXX") || return 1
  entries=$(mktemp "${TMPDIR:-/tmp}/openarmik-install-entries.XXXXXX") || {
    rm -f -- "$manifest"
    return 1
  }
  find "$install_root" -mindepth 1 \( -type f -o -type l -o -type d \) \
    -print0 | sort -z > "$entries" || {
      rm -f -- "$manifest" "$entries"
      return 1
    }
  install_mode=$(stat -c '%a' -- "$install_root") || {
    rm -f -- "$manifest" "$entries"
    return 1
  }
  printf 'root\0%s\0' "$install_mode" > "$manifest"
  while IFS= read -r -d '' entry; do
    relative=${entry#"$install_root"/}
    mode=$(stat -c '%a' -- "$entry") || {
      rm -f -- "$manifest" "$entries"
      return 1
    }
    if [[ -L "$entry" ]]; then
      link_text=$(readlink -- "$entry") || { rm -f -- "$manifest" "$entries"; return 1; }
      link_real=$(realpath -e -- "$entry") || {
        printf 'Broken install symlink: %s\n' "$entry" >&2
        rm -f -- "$manifest" "$entries"
        return 1
      }
      case "$link_real" in
        "$install_real"|"$install_real"/*) ;;
        *)
          printf 'Install symlink escapes audited root: %s -> %s\n' \
            "$entry" "$link_real" >&2
          rm -f -- "$manifest" "$entries"
          return 1
          ;;
      esac
      target_mode=$(stat -c '%a' -- "$link_real") || {
        rm -f -- "$manifest" "$entries"
        return 1
      }
      if [[ -f "$link_real" ]]; then
        hash=$(sha256sum -- "$link_real") || { rm -f -- "$manifest" "$entries"; return 1; }
        printf 'link_file\0%s\0%s\0%s\0%s\0%s\0' "$relative" "$mode" \
          "$link_text" "$target_mode" "${hash%% *}"
      elif [[ -d "$link_real" ]]; then
        printf 'link_dir\0%s\0%s\0%s\0%s\0' "$relative" "$mode" \
          "$link_text" "$target_mode"
      else
        printf 'Unsupported install symlink target: %s\n' "$entry" >&2
        rm -f -- "$manifest" "$entries"
        return 1
      fi
    elif [[ -d "$entry" ]]; then
      printf 'dir\0%s\0%s\0' "$relative" "$mode"
    else
      hash=$(sha256sum -- "$entry") || { rm -f -- "$manifest" "$entries"; return 1; }
      printf 'file\0%s\0%s\0%s\0' "$relative" "$mode" "${hash%% *}"
    fi
  done < "$entries" >> "$manifest"
  sha256sum -- "$manifest" | awk '{print $1}'
  rm -f -- "$manifest" "$entries"
}

openarm_compute_launch_build_state_digest() {
  local output_root=$1 run_tests=${2:-0} native_state ros_state
  local -a native_components=(can model commission transport control runtime)
  ((run_tests == 0)) || native_components+=(installed_native_consumer)
  native_state=$(openarm_build_state_read_completed "$output_root/native_build" '' \
    "${native_components[@]}") || return 1
  ros_state=$(openarm_build_state_read_completed "$output_root/build" '' \
    openarm_control_msgs openarm_description openarm_ik_ros) || return 1
  printf 'native %s\nros %s\n' "$native_state" "$ros_state" | sha256sum | awk '{print $1}'
}

openarm_launch_artifact_paths() {
  local output_root=$1
  printf '%s\n' \
    "session|$output_root/build/openarm_ik_ros/libopenarm_virtual_control_session.a" \
    "runtime|$output_root/install/lib/libopenarm_runtime.a" \
    "node|$output_root/install/openarm_ik_ros/lib/openarm_ik_ros/openarm_ik_ros_node" \
    "portal|$output_root/install/openarm_ik_ros/lib/openarm_ik_ros/openarm_portal" \
    "close_helper|$output_root/install/openarm_ik_ros/lib/openarm_ik_ros/close_rviz_window" \
    "portal_css|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/web/portal.css" \
    "portal_js|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/web/portal.js" \
    "viewer_js|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/web/viewer.js" \
    "viewer_manifest|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/viewer/manifest.json" \
    "viewer_urdf|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/viewer/stage_a.urdf" \
    "viewer_license|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/viewer/openarm_description-LICENSE.txt" \
    "viewer_body|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/viewer/mesh/body_link0_symp.stl" \
    "viewer_link0|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/viewer/mesh/link0_symp.stl" \
    "viewer_link1|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/viewer/mesh/link1_symp.stl" \
    "viewer_link2|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/viewer/mesh/link2_symp.stl" \
    "viewer_link3|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/viewer/mesh/link3_symp.stl" \
    "viewer_link4|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/viewer/mesh/link4_symp.stl" \
    "viewer_link5|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/viewer/mesh/link5_symp.stl" \
    "viewer_link6|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/viewer/mesh/link6_symp.stl" \
    "viewer_link7|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/viewer/mesh/link7_symp.stl" \
    "viewer_hand|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/viewer/mesh/hand.stl" \
    "viewer_finger|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/viewer/mesh/finger.stl" \
    "setup|$output_root/install/setup.bash" \
    "launch|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/launch/openarm_ik_rviz.launch.py" \
    "rviz|$output_root/install/openarm_ik_ros/share/openarm_ik_ros/rviz/openarm_ik.rviz"
}

openarm_assert_launch_authority() {
  local output_root=$1 record label artifact session_undefined runtime_undefined runtime_api_count
  while IFS= read -r record; do
    label=${record%%|*}
    artifact=${record#*|}
    [[ -f "$artifact" ]] || {
      printf 'Missing required launch %s artifact: %s\n' "$label" "$artifact" >&2
      return 1
    }
  done < <(openarm_launch_artifact_paths "$output_root")
  [[ -x "$output_root/install/openarm_ik_ros/lib/openarm_ik_ros/openarm_ik_ros_node" &&
     -x "$output_root/install/openarm_ik_ros/lib/openarm_ik_ros/openarm_portal" &&
     -x "$output_root/install/openarm_ik_ros/lib/openarm_ik_ros/close_rviz_window" ]] || {
    printf 'Required installed launch executables are not executable\n' >&2
    return 1
  }

  session_undefined=$(nm -u \
    "$output_root/build/openarm_ik_ros/libopenarm_virtual_control_session.a") || {
      printf 'Could not audit the ROS session archive\n' >&2
      return 1
    }
  grep -Eq '(^|[[:space:]])U[[:space:]]+oa_runtime_create([[:space:]]|$)' \
    <<<"$session_undefined" || {
      printf 'Production ROS session does not consume oa_runtime_create\n' >&2
      return 1
    }
  runtime_api_count=$(grep -Eo 'oa_runtime_[A-Za-z0-9_]+' <<<"$session_undefined" |
    sort -u | wc -l)
  ((runtime_api_count >= 2)) || {
    printf 'Production ROS session lacks the expected Runtime API surface\n' >&2
    return 1
  }
  if grep -Eq '(^|[[:space:]])U[[:space:]]+oa_(controller_|motion_plan_|manifest_)' \
      <<<"$session_undefined"; then
    printf 'Production ROS session bypasses OpenArm::Runtime\n' >&2
    return 1
  fi

  runtime_undefined=$(nm -u "$output_root/install/lib/libopenarm_runtime.a") || {
    printf 'Could not audit the installed Runtime archive\n' >&2
    return 1
  }
  if grep -Eq '(^|[[:space:]])U[[:space:]]+(oa_can_|oa_transport_|socket(@|$)|send(to|msg)?(@|$)|recv(from|msg)?(@|$))' \
      <<<"$runtime_undefined"; then
    printf 'Installed Runtime archive reaches forbidden CAN/transport/socket I/O\n' >&2
    return 1
  fi
}

openarm_write_launch_stamp() {
  local root_dir=$1 output_root=$2 description_dir=$3 expected_fingerprint=$4
  local build_type=${5:-Release} run_tests=${6:-0} actual_fingerprint stamp temporary
  local record label artifact hash canonical_output install_manifest build_state
  canonical_output=$(realpath -m -- "$output_root")
  stamp="$output_root/$OPENARM_LAUNCH_STAMP_NAME"
  openarm_validate_description_pin "$description_dir" || return 1
  actual_fingerprint=$(openarm_compute_launch_source_fingerprint \
    "$root_dir" "$description_dir" "$build_type" "$run_tests") || return 1
  [[ "$actual_fingerprint" == "$expected_fingerprint" ]] || {
    printf 'Build inputs changed during the build; refusing to publish launch stamp\n' >&2
    return 1
  }
  openarm_assert_launch_authority "$output_root" || return 1
  install_manifest=$(openarm_compute_install_manifest_digest "$output_root") || return 1
  build_state=$(openarm_compute_launch_build_state_digest "$output_root" \
    "$run_tests") || return 1
  temporary=$(mktemp "$output_root/.openarm-launch-stamp.XXXXXX") || return 1
  {
    printf 'OPENARM_LAUNCH_STAMP_V2\n'
    printf 'output_root %s\n' "$canonical_output"
    printf 'fingerprint %s\n' "$actual_fingerprint"
    printf 'build_type %s\n' "$build_type"
    printf 'run_tests %s\n' "$run_tests"
    printf 'install_manifest %s\n' "$install_manifest"
    printf 'build_state %s\n' "$build_state"
    while IFS= read -r record; do
      label=${record%%|*}
      artifact=${record#*|}
      hash=$(sha256sum -- "$artifact") || exit 1
      printf 'artifact_%s %s\n' "$label" "${hash%% *}"
    done < <(openarm_launch_artifact_paths "$output_root")
  } > "$temporary" || { rm -f -- "$temporary"; return 1; }
  chmod 0644 -- "$temporary"
  mv -fT -- "$temporary" "$stamp"
}

openarm_assert_current_launch_tree() {
  local root_dir=$1 output_root=$2 build_type=${3:-Release}
  local description_dir="$root_dir/upstream/openarm_description"
  local stamp="$output_root/$OPENARM_LAUNCH_STAMP_NAME"
  local fingerprint expected record label artifact hash stamped_hash canonical_output install_manifest build_state
  [[ -f "$stamp" && ! -L "$stamp" ]] || {
    printf 'Missing trustworthy launch stamp: %s\n' "$stamp" >&2
    return 1
  }
  [[ $(sed -n '1p' "$stamp") == OPENARM_LAUNCH_STAMP_V2 ]] || {
    printf 'Invalid launch stamp format: %s\n' "$stamp" >&2
    return 1
  }
  canonical_output=$(realpath -m -- "$output_root")
  [[ $(awk '$1 == "output_root" {$1=""; sub(/^ /, ""); print}' "$stamp") == \
     "$canonical_output" ]] || {
    printf 'Launch stamp belongs to a different output root\n' >&2
    return 1
  }
  openarm_validate_description_pin "$description_dir" || return 1
  fingerprint=$(openarm_compute_launch_source_fingerprint \
    "$root_dir" "$description_dir" "$build_type" 0) || return 1
  expected=$(awk '$1 == "fingerprint" {print $2}' "$stamp")
  [[ "$expected" == "$fingerprint" ]] || {
    printf 'Launch tree source fingerprint is stale\n' >&2
    return 1
  }
  [[ $(awk '$1 == "build_type" {print $2}' "$stamp") == "$build_type" ]] || {
    printf 'Launch tree build profile is stale\n' >&2
    return 1
  }
  [[ $(awk '$1 == "run_tests" {print $2}' "$stamp") == 0 ]] || {
    printf 'Launch tree test/build mode is stale\n' >&2
    return 1
  }
  openarm_assert_launch_authority "$output_root" || return 1
  install_manifest=$(openarm_compute_install_manifest_digest "$output_root") || return 1
  [[ $(awk '$1 == "install_manifest" {print $2}' "$stamp") == "$install_manifest" ]] || {
    printf 'Installed launch closure changed after validation\n' >&2
    return 1
  }
  build_state=$(openarm_compute_launch_build_state_digest "$output_root") || return 1
  [[ $(awk '$1 == "build_state" {print $2}' "$stamp") == "$build_state" ]] || {
    printf 'Build cache/toolchain state changed after validation\n' >&2
    return 1
  }
  while IFS= read -r record; do
    label=${record%%|*}
    artifact=${record#*|}
    hash=$(sha256sum -- "$artifact") || return 1
    hash=${hash%% *}
    stamped_hash=$(awk -v key="artifact_$label" '$1 == key {print $2}' "$stamp")
    [[ "$stamped_hash" == "$hash" ]] || {
      printf 'Launch artifact changed after validation: %s\n' "$artifact" >&2
      return 1
    }
  done < <(openarm_launch_artifact_paths "$output_root")
}

openarm_ensure_current_launch_tree() {
  local root_dir=$1 output_root=$2 mode=$3 jobs=${4:-}
  local -a build_arguments=(--incremental --output-root "$output_root")
  if [[ -n "$jobs" ]]; then
    build_arguments+=(--jobs "$jobs")
  fi
  case "$mode" in
    auto|always)
      "$root_dir/scripts/build.sh" "${build_arguments[@]}" || return
      ;;
    never) ;;
    *) printf 'Invalid launch build mode: %s\n' "$mode" >&2; return 2 ;;
  esac
  openarm_close_shared_lock_fds
  openarm_acquire_shared_locks "$output_root" "$output_root/native_build" \
    "$output_root/install" || return
  openarm_assert_current_launch_tree "$root_dir" "$output_root" Release
}
