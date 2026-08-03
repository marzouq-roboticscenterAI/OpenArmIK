/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Additive bimanual motion API.
 *
 * openarm_runtime.h is a frozen V1 surface: its bytes are hash-pinned and its
 * exported symbol set is manifest-checked. Everything here is therefore purely
 * additive and lives in its own header, exactly as openarm_runtime_units.h
 * does. Nothing in V1 changes layout, meaning, or behaviour.
 *
 * Centroid and mirrored motion are header-only adapters: they derive the two
 * claw targets from measured kinematics and delegate to the V1 paired planner,
 * so they add no new exported symbols and inherit every V1 guarantee
 * (all-or-nothing planning, identity binding, freshness, expiry).
 *
 * Converge-until-resistance and the emergency stop need runtime state and are
 * therefore new exported entry points.
 */
#ifndef OPENARM_RUNTIME_MOTION_H
#define OPENARM_RUNTIME_MOTION_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "openarm_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OA_RUNTIME_MOTION_ABI_VERSION UINT32_C(1)

/* Move the midpoint between the two hand_tcp origins to target_centroid_m.
 * The identical body-frame translation is applied to both claws, so their
 * separation and relative geometry are preserved. `base` carries every field a
 * plain paired move needs; its left_tcp_m and right_tcp_m are ignored and
 * overwritten with the derived targets. */
typedef struct oa_runtime_centroid_tcp_move {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_runtime_paired_tcp_move base;
    double target_centroid_m[3];
} oa_runtime_centroid_tcp_move;

/* Command one claw and mirror it onto the other across the body sagittal
 * plane (y is negated; x and z are preserved). */
typedef struct oa_runtime_mirrored_tcp_move {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_runtime_paired_tcp_move base;
    uint32_t lead_side;
    uint32_t reserved0;
    double lead_tcp_m[3];
} oa_runtime_mirrored_tcp_move;

/* Advance both claws along their own rays toward a shared point, halting at
 * the first of measured contact torque, a real-time keepout violation, or the
 * end of the planned prefix. Halting on contact is a success, not a fault. */
typedef struct oa_runtime_converge_tcp_move {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_runtime_paired_tcp_move base;
    double target_m[3];
    /* Per-joint absolute |tau| stop threshold, newton-metres, model joint
     * coordinates. A non-positive entry falls back to the fraction below. */
    double contact_torque_nm[OA_RUNTIME_DOF];
    /* Fallback threshold as a fraction of each motor's protocol tmax.
     * Non-positive selects the library default. */
    double contact_torque_fraction;
    /* Consecutive cycles the threshold must hold before the stop latches.
     * Zero selects the library default. */
    uint32_t contact_persistence_cycles;
    uint32_t reserved0;
    /* Halt this far short of target_m along each approach ray. */
    double stop_distance_m;
    /* Reject unless at least this much validated travel exists. */
    double minimum_progress_m;
} oa_runtime_converge_tcp_move;

typedef uint32_t oa_runtime_stop_cause;
#define OA_RUNTIME_STOP_CAUSE_NONE UINT32_C(0)
#define OA_RUNTIME_STOP_CAUSE_CONTACT UINT32_C(1)
#define OA_RUNTIME_STOP_CAUSE_KEEPOUT UINT32_C(2)
#define OA_RUNTIME_STOP_CAUSE_PLAN_COMPLETE UINT32_C(3)
#define OA_RUNTIME_STOP_CAUSE_ESTOP UINT32_C(4)

typedef struct oa_runtime_contact_report {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_runtime_stop_cause cause;
    uint32_t contact_detected;
    uint32_t contact_side_mask;
    uint32_t contact_joint_mask[OA_RUNTIME_ARMS];
    uint32_t keepout_violation;
    uint32_t keepout_side;
    uint32_t keepout_segment_a;
    uint32_t keepout_segment_b;
    uint64_t stop_feedback_seq[OA_RUNTIME_ARMS];
    uint64_t stop_monotonic_ns;
    double contact_torque_nm[OA_RUNTIME_ARMS][OA_RUNTIME_DOF];
    double threshold_torque_nm[OA_RUNTIME_ARMS][OA_RUNTIME_DOF];
    double stopped_q_rad[OA_RUNTIME_ARMS][OA_RUNTIME_DOF];
    double stopped_tcp_m[OA_RUNTIME_ARMS][3];
    double minimum_clearance_m;
} oa_runtime_contact_report;

/* Virtual mechanical resistance for the simulated backend only. */
typedef struct oa_runtime_sim_contact {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t side;
    uint32_t enabled;
    double center_m[3];
    double radius_m;
    /* Reaction torque per radian of reference overshoot past the held plant.
     * Zero selects a stiff default derived from each motor's tmax; a negative
     * gain is rejected rather than clamped. */
    double reaction_gain_nm_per_rad;
} oa_runtime_sim_contact;

oa_runtime_status oa_runtime_plan_converge_tcp_body(
    oa_runtime *runtime, const oa_runtime_converge_tcp_move *request,
    oa_runtime_plan **out_plan);

/* Outcome of the real-time contact and keepout monitors for the most recent
 * command. Valid once that command has reached a terminal event. */
oa_runtime_status oa_runtime_get_contact_report(oa_runtime *runtime,
                                                oa_runtime_contact_report *out_report);

oa_runtime_status oa_runtime_sim_set_contact(oa_runtime *runtime,
                                             const oa_runtime_sim_contact *contact);

/* Process-wide emergency stop.
 *
 * oa_runtime_estop_assert is lock-free, allocation-free, and never blocks. It
 * takes no runtime handle and no lock, so it remains available while any
 * runtime is mid-cycle, mid-plan, faulted, or holding every other lock in the
 * process, and it is safe to call from a signal handler. Every controller in
 * the process samples the latch at the top of every control cycle, in every
 * lifecycle state, before any other work.
 *
 * This is a software interlock and is not a substitute for a hardwired,
 * safety-rated emergency stop in the power path. */
void oa_runtime_estop_assert(void);
uint32_t oa_runtime_estop_asserted(void);
oa_runtime_status oa_runtime_estop_clear(void);
uint64_t oa_runtime_estop_assert_count(void);

/* ---- header-only adapters over the frozen V1 paired planner ---- */

static inline oa_runtime_status oa_runtime_motion_measured_tcp_(
    oa_runtime *runtime, const oa_runtime_paired_tcp_move *base,
    double out_tcp[OA_RUNTIME_ARMS][3]) {
    uint32_t side;
    for (side = 0; side < OA_RUNTIME_ARMS; ++side) {
        oa_runtime_kinematics kinematics;
        oa_runtime_status status;
        size_t axis;
        memset(&kinematics, 0, sizeof(kinematics));
        kinematics.struct_size = (uint32_t)sizeof(kinematics);
        kinematics.abi_version = OA_RUNTIME_ABI_VERSION;
        status = oa_runtime_get_kinematics(runtime, side,
                                           base->required_feedback_seq[side],
                                           &kinematics);
        if (status != OA_RUNTIME_OK) {
            return status;
        }
        for (axis = 0; axis < 3u; ++axis) {
            out_tcp[side][axis] = kinematics.tcp_xyz_m[axis];
        }
    }
    return OA_RUNTIME_OK;
}

/* Both claws translate by the vector carrying their measured midpoint to the
 * requested centroid. */
static inline oa_runtime_status oa_runtime_plan_centroid_tcp_body(
    oa_runtime *runtime, const oa_runtime_centroid_tcp_move *request,
    oa_runtime_plan **out_plan) {
    double measured[OA_RUNTIME_ARMS][3];
    oa_runtime_paired_tcp_move derived;
    oa_runtime_status status;
    size_t axis;
    if (runtime == NULL || request == NULL || out_plan == NULL ||
        request->abi_version != OA_RUNTIME_MOTION_ABI_VERSION ||
        request->struct_size < (uint32_t)sizeof(*request)) {
        return OA_RUNTIME_EINVAL;
    }
    status = oa_runtime_motion_measured_tcp_(runtime, &request->base, measured);
    if (status != OA_RUNTIME_OK) {
        return status;
    }
    derived = request->base;
    for (axis = 0; axis < 3u; ++axis) {
        const double centroid = 0.5 * (measured[0][axis] + measured[1][axis]);
        const double delta = request->target_centroid_m[axis] - centroid;
        derived.left_tcp_m[axis] = measured[0][axis] + delta;
        derived.right_tcp_m[axis] = measured[1][axis] + delta;
    }
    return oa_runtime_plan_paired_tcp_body(runtime, &derived, out_plan);
}

/* One claw is commanded; the other receives the sagittal mirror of that target. */
static inline oa_runtime_status oa_runtime_plan_mirrored_tcp_body(
    oa_runtime *runtime, const oa_runtime_mirrored_tcp_move *request,
    oa_runtime_plan **out_plan) {
    oa_runtime_paired_tcp_move derived;
    double *lead;
    double *follow;
    if (runtime == NULL || request == NULL || out_plan == NULL ||
        request->abi_version != OA_RUNTIME_MOTION_ABI_VERSION ||
        request->struct_size < (uint32_t)sizeof(*request) ||
        request->reserved0 != 0u || request->lead_side >= OA_RUNTIME_ARMS) {
        return OA_RUNTIME_EINVAL;
    }
    derived = request->base;
    lead = request->lead_side == 0u ? derived.left_tcp_m : derived.right_tcp_m;
    follow = request->lead_side == 0u ? derived.right_tcp_m : derived.left_tcp_m;
    lead[0] = request->lead_tcp_m[0];
    lead[1] = request->lead_tcp_m[1];
    lead[2] = request->lead_tcp_m[2];
    follow[0] = request->lead_tcp_m[0];
    follow[1] = -request->lead_tcp_m[1];
    follow[2] = request->lead_tcp_m[2];
    return oa_runtime_plan_paired_tcp_body(runtime, &derived, out_plan);
}

#ifdef __cplusplus
}
#endif

#endif
