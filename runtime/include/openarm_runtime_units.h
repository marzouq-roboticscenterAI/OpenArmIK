/* SPDX-License-Identifier: Apache-2.0 */
#ifndef OPENARM_RUNTIME_UNITS_H
#define OPENARM_RUNTIME_UNITS_H

#include <openarm_runtime_motion.h>
#include <openarm_units.h>

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct oa_runtime_paired_tcp_move_with_units {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_length_unit coordinate_unit;
    uint32_t reserved0;
    oa_runtime_clock_id clock_id;
    oa_runtime_frame_id frame_id;
    oa_runtime_orientation_policy orientation_policy;
    uint32_t reserved1;
    uint64_t expiry_runtime_monotonic_ns;
    uint64_t required_feedback_seq[OA_RUNTIME_ARMS];
    oa_vec3d left_tcp;
    oa_vec3d right_tcp;
    double velocity_scale;
    double acceleration_scale;
    double jerk_scale;
    double tcp_tolerance_m;
    uint64_t collision_scene_revision;
    uint64_t required_model_revision;
    uint64_t required_tcp_revision[OA_RUNTIME_ARMS];
    oa_runtime_collision_policy required_collision_policy;
    char required_coordinate_identity_sha256[OA_RUNTIME_DIGEST_CAPACITY];
    double maximum_branch_step_rad;
    double minimum_singular_value;
} oa_runtime_paired_tcp_move_with_units;

typedef struct oa_runtime_centroid_tcp_move_with_units {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_runtime_paired_tcp_move_with_units base;
    oa_vec3d target_centroid;
} oa_runtime_centroid_tcp_move_with_units;

typedef struct oa_runtime_mirrored_tcp_move_with_units {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_runtime_paired_tcp_move_with_units base;
    uint32_t lead_side;
    uint32_t reserved0;
    oa_vec3d lead_tcp;
} oa_runtime_mirrored_tcp_move_with_units;

typedef struct oa_runtime_converge_tcp_move_with_units {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_runtime_paired_tcp_move_with_units base;
    oa_vec3d target;
    double contact_torque_nm[OA_RUNTIME_DOF];
    double contact_torque_fraction;
    uint32_t contact_persistence_cycles;
    uint32_t reserved0;
    double stop_distance_m;
    double minimum_progress_m;
} oa_runtime_converge_tcp_move_with_units;

static inline oa_runtime_status oa_runtime_units_convert_paired_(
    const oa_runtime_paired_tcp_move_with_units *request,
    oa_runtime_paired_tcp_move *converted) {
    oa_vec3d left_m;
    oa_vec3d right_m;
    if (request == NULL || converted == NULL ||
        request->struct_size < sizeof(*request) ||
        request->abi_version != OA_RUNTIME_ABI_VERSION) {
        return OA_RUNTIME_EABI;
    }
    if (request->reserved0 != 0U || request->reserved1 != 0U ||
        oa_vec3d_convert(&request->left_tcp, request->coordinate_unit,
                         OA_LENGTH_UNIT_METRES, &left_m) != OA_UNITS_OK ||
        oa_vec3d_convert(&request->right_tcp, request->coordinate_unit,
                         OA_LENGTH_UNIT_METRES, &right_m) != OA_UNITS_OK) {
        return OA_RUNTIME_EINVAL;
    }
    memset(converted, 0, sizeof(*converted));
    converted->struct_size = (uint32_t)sizeof(*converted);
    converted->abi_version = OA_RUNTIME_ABI_VERSION;
    converted->clock_id = request->clock_id;
    converted->units_id = OA_RUNTIME_UNITS_SI_V1;
    converted->frame_id = request->frame_id;
    converted->orientation_policy = request->orientation_policy;
    converted->expiry_runtime_monotonic_ns = request->expiry_runtime_monotonic_ns;
    memcpy(converted->required_feedback_seq, request->required_feedback_seq,
           sizeof(converted->required_feedback_seq));
    converted->left_tcp_m[0] = left_m.x;
    converted->left_tcp_m[1] = left_m.y;
    converted->left_tcp_m[2] = left_m.z;
    converted->right_tcp_m[0] = right_m.x;
    converted->right_tcp_m[1] = right_m.y;
    converted->right_tcp_m[2] = right_m.z;
    converted->velocity_scale = request->velocity_scale;
    converted->acceleration_scale = request->acceleration_scale;
    converted->jerk_scale = request->jerk_scale;
    converted->tcp_tolerance_m = request->tcp_tolerance_m;
    converted->collision_scene_revision = request->collision_scene_revision;
    converted->required_model_revision = request->required_model_revision;
    memcpy(converted->required_tcp_revision, request->required_tcp_revision,
           sizeof(converted->required_tcp_revision));
    converted->required_collision_policy = request->required_collision_policy;
    memcpy(converted->required_coordinate_identity_sha256,
           request->required_coordinate_identity_sha256,
           sizeof(converted->required_coordinate_identity_sha256));
    converted->maximum_branch_step_rad = request->maximum_branch_step_rad;
    converted->minimum_singular_value = request->minimum_singular_value;
    return OA_RUNTIME_OK;
}

/* This adapter is header-only so Runtime V1's archive and symbol manifest remain
 * unchanged. TCP inputs are converted once; tolerances and reports stay metres. */
static inline oa_runtime_status oa_runtime_plan_paired_tcp_body_with_units(
    oa_runtime *runtime,
    const oa_runtime_paired_tcp_move_with_units *request,
    oa_runtime_plan **out_plan) {
    oa_runtime_paired_tcp_move converted;
    const oa_runtime_status status = oa_runtime_units_convert_paired_(request, &converted);
    if (status != OA_RUNTIME_OK) {
        return status;
    }
    return oa_runtime_plan_paired_tcp_body(runtime, &converted, out_plan);
}

static inline oa_runtime_status oa_runtime_plan_centroid_tcp_body_with_units(
    oa_runtime *runtime, const oa_runtime_centroid_tcp_move_with_units *request,
    oa_runtime_plan **out_plan) {
    oa_runtime_centroid_tcp_move converted;
    oa_vec3d target_m;
    oa_runtime_status status;
    if (request == NULL || request->struct_size < sizeof(*request) ||
        request->abi_version != OA_RUNTIME_MOTION_ABI_VERSION) {
        return OA_RUNTIME_EABI;
    }
    status = oa_runtime_units_convert_paired_(&request->base, &converted.base);
    if (status != OA_RUNTIME_OK) {
        return status;
    }
    if (oa_vec3d_convert(&request->target_centroid, request->base.coordinate_unit,
                         OA_LENGTH_UNIT_METRES, &target_m) != OA_UNITS_OK) {
        return OA_RUNTIME_EINVAL;
    }
    converted.struct_size = (uint32_t)sizeof(converted);
    converted.abi_version = OA_RUNTIME_MOTION_ABI_VERSION;
    converted.target_centroid_m[0] = target_m.x;
    converted.target_centroid_m[1] = target_m.y;
    converted.target_centroid_m[2] = target_m.z;
    return oa_runtime_plan_centroid_tcp_body(runtime, &converted, out_plan);
}

static inline oa_runtime_status oa_runtime_plan_mirrored_tcp_body_with_units(
    oa_runtime *runtime, const oa_runtime_mirrored_tcp_move_with_units *request,
    oa_runtime_plan **out_plan) {
    oa_runtime_mirrored_tcp_move converted;
    oa_vec3d target_m;
    oa_runtime_status status;
    if (request == NULL || request->struct_size < sizeof(*request) ||
        request->abi_version != OA_RUNTIME_MOTION_ABI_VERSION ||
        request->reserved0 != 0U) {
        return OA_RUNTIME_EABI;
    }
    status = oa_runtime_units_convert_paired_(&request->base, &converted.base);
    if (status != OA_RUNTIME_OK) {
        return status;
    }
    if (oa_vec3d_convert(&request->lead_tcp, request->base.coordinate_unit,
                         OA_LENGTH_UNIT_METRES, &target_m) != OA_UNITS_OK) {
        return OA_RUNTIME_EINVAL;
    }
    converted.struct_size = (uint32_t)sizeof(converted);
    converted.abi_version = OA_RUNTIME_MOTION_ABI_VERSION;
    converted.lead_side = request->lead_side;
    converted.reserved0 = 0U;
    converted.lead_tcp_m[0] = target_m.x;
    converted.lead_tcp_m[1] = target_m.y;
    converted.lead_tcp_m[2] = target_m.z;
    return oa_runtime_plan_mirrored_tcp_body(runtime, &converted, out_plan);
}

static inline oa_runtime_status oa_runtime_plan_converge_tcp_body_with_units(
    oa_runtime *runtime, const oa_runtime_converge_tcp_move_with_units *request,
    oa_runtime_plan **out_plan) {
    oa_runtime_converge_tcp_move converted;
    oa_vec3d target_m;
    oa_runtime_status status;
    if (request == NULL || request->struct_size < sizeof(*request) ||
        request->abi_version != OA_RUNTIME_MOTION_ABI_VERSION ||
        request->reserved0 != 0U) {
        return OA_RUNTIME_EABI;
    }
    status = oa_runtime_units_convert_paired_(&request->base, &converted.base);
    if (status != OA_RUNTIME_OK) {
        return status;
    }
    if (oa_vec3d_convert(&request->target, request->base.coordinate_unit,
                         OA_LENGTH_UNIT_METRES, &target_m) != OA_UNITS_OK) {
        return OA_RUNTIME_EINVAL;
    }
    converted.struct_size = (uint32_t)sizeof(converted);
    converted.abi_version = OA_RUNTIME_MOTION_ABI_VERSION;
    converted.target_m[0] = target_m.x;
    converted.target_m[1] = target_m.y;
    converted.target_m[2] = target_m.z;
    memcpy(converted.contact_torque_nm, request->contact_torque_nm,
           sizeof(converted.contact_torque_nm));
    converted.contact_torque_fraction = request->contact_torque_fraction;
    converted.contact_persistence_cycles = request->contact_persistence_cycles;
    converted.reserved0 = 0U;
    converted.stop_distance_m = request->stop_distance_m;
    converted.minimum_progress_m = request->minimum_progress_m;
    return oa_runtime_plan_converge_tcp_body(runtime, &converted, out_plan);
}

#ifdef __cplusplus
}
#endif
#endif
