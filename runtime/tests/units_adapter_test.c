/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_runtime_units.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static oa_runtime_paired_tcp_move captured;
static unsigned calls;

oa_runtime_status oa_runtime_plan_paired_tcp_body(
    oa_runtime *runtime, const oa_runtime_paired_tcp_move *request,
    oa_runtime_plan **out_plan) {
    if (runtime != (oa_runtime *)(uintptr_t)UINT32_C(0x200) ||
        out_plan == NULL) {
        return OA_RUNTIME_EFAULT;
    }
    captured = *request;
    ++calls;
    return OA_RUNTIME_EBUSY;
}

static void init_request(oa_runtime_paired_tcp_move_with_units *request,
                         oa_length_unit unit) {
    const oa_vec3d left_m = {0.200000000000001, 0.3, 0.85};
    const oa_vec3d right_m = {0.200000000000001, -0.3, 0.85};
    memset(request, 0, sizeof(*request));
    request->struct_size = (uint32_t)sizeof(*request);
    request->abi_version = OA_RUNTIME_ABI_VERSION;
    request->coordinate_unit = unit;
    request->clock_id = OA_RUNTIME_CLOCK_MONOTONIC;
    request->frame_id = OA_RUNTIME_FRAME_OPENARM_BODY_LINK0;
    request->orientation_policy = OA_RUNTIME_ORIENTATION_FREE;
    request->expiry_runtime_monotonic_ns = UINT64_C(123456789);
    request->required_feedback_seq[0] = UINT64_C(41);
    request->required_feedback_seq[1] = UINT64_C(42);
    if (oa_vec3d_convert(&left_m, OA_LENGTH_UNIT_METRES, unit,
                         &request->left_tcp) != OA_UNITS_OK ||
        oa_vec3d_convert(&right_m, OA_LENGTH_UNIT_METRES, unit,
                         &request->right_tcp) != OA_UNITS_OK) {
        request->struct_size = 0U;
    }
    request->velocity_scale = 0.7;
    request->acceleration_scale = 0.6;
    request->jerk_scale = 0.5;
    request->tcp_tolerance_m = 0.001;
    request->collision_scene_revision = UINT64_C(7);
    request->required_model_revision = UINT64_C(8);
    request->required_tcp_revision[0] = UINT64_C(9);
    request->required_tcp_revision[1] = UINT64_C(10);
    request->required_collision_policy = OA_RUNTIME_COLLISION_VIRTUAL_UNCHECKED;
    memcpy(request->required_coordinate_identity_sha256, "binary64-adapter",
           sizeof("binary64-adapter"));
    request->maximum_branch_step_rad = 2.0;
    request->minimum_singular_value = 0.0;
}

static int check_captured(void) {
    return captured.struct_size == sizeof(captured) &&
           captured.abi_version == OA_RUNTIME_ABI_VERSION &&
           captured.clock_id == OA_RUNTIME_CLOCK_MONOTONIC &&
           captured.units_id == OA_RUNTIME_UNITS_SI_V1 &&
           captured.frame_id == OA_RUNTIME_FRAME_OPENARM_BODY_LINK0 &&
           captured.orientation_policy == OA_RUNTIME_ORIENTATION_FREE &&
           captured.left_tcp_m[0] == 0.200000000000001 &&
           captured.left_tcp_m[1] == 0.3 &&
           captured.left_tcp_m[2] == 0.85 &&
           captured.right_tcp_m[0] == 0.200000000000001 &&
           captured.right_tcp_m[1] == -0.3 &&
           captured.right_tcp_m[2] == 0.85 &&
           captured.tcp_tolerance_m == 0.001 &&
           captured.required_model_revision == UINT64_C(8) &&
           strcmp(captured.required_coordinate_identity_sha256,
                  "binary64-adapter") == 0;
}

int main(void) {
    static const oa_length_unit units[] = {
        OA_LENGTH_UNIT_METRES,
        OA_LENGTH_UNIT_CENTIMETRES,
        OA_LENGTH_UNIT_INCHES
    };
    size_t index;
    oa_runtime_plan *out = (oa_runtime_plan *)(uintptr_t)UINT32_C(0x300);

    for (index = 0; index < 3; ++index) {
        oa_runtime_paired_tcp_move_with_units request;
        init_request(&request, units[index]);
        memset(&captured, 0, sizeof(captured));
        if (oa_runtime_plan_paired_tcp_body_with_units(
                (oa_runtime *)(uintptr_t)UINT32_C(0x200), &request, &out) !=
                OA_RUNTIME_EBUSY ||
            calls != index + 1U || !check_captured()) {
            return 1;
        }
    }

    oa_runtime_paired_tcp_move_with_units invalid;
    init_request(&invalid, OA_LENGTH_UNIT_METRES);
    invalid.coordinate_unit = UINT32_C(0);
    if (oa_runtime_plan_paired_tcp_body_with_units(
            (oa_runtime *)(uintptr_t)UINT32_C(0x200), &invalid, &out) !=
            OA_RUNTIME_EINVAL || calls != 3U ||
        out != (oa_runtime_plan *)(uintptr_t)UINT32_C(0x300)) {
        return 1;
    }
    invalid.coordinate_unit = OA_LENGTH_UNIT_METRES;
    invalid.left_tcp.x = INFINITY;
    if (oa_runtime_plan_paired_tcp_body_with_units(
            (oa_runtime *)(uintptr_t)UINT32_C(0x200), &invalid, &out) !=
            OA_RUNTIME_EINVAL || calls != 3U) {
        return 1;
    }
    puts("Runtime unit adapter binary64 equivalence passed");
    return 0;
}
