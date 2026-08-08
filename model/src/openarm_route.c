/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_route.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define OA_ROUTE_SAMPLES 17u
#define OA_ROUTE_CANDIDATES 22u
#define OA_ROUTE_BRANCH_DEFAULT 0.35
#define OA_ROUTE_ENCODER_LIMIT_TOLERANCE 3.9e-4
#define OA_ROUTE_CLEARANCE_EPSILON 1.0e-6
#define OA_ROUTE_TCP_EQUIVALENCE_TOLERANCE_M 1.0e-8

typedef struct route_pose {
    double tcp[2][3];
} route_pose;

typedef struct route_node {
    int discovered;
    int predecessor;
    int used_recovery;
    double edge_minimum_clearance_m;
    double q[2][OA_DOF];
} route_node;

/* These are connectivity anchors, not requested destinations.  They are the
 * already audited Cross, Heart, box, and neutral corridors used by the portal.
 * Edges are never trusted because they appear in this table: every attempted
 * edge is independently solved and collision checked from its actual seed. */
static const route_pose route_anchors[OA_ROUTE_CANDIDATES - 1u] = {
    {{{0.30, -0.04, 0.62}, {0.30, 0.04, 0.22}}}, /* Cross over */
    {{{0.30, 0.26, 0.62}, {0.30, -0.26, 0.22}}}, /* Cross split */
    {{{0.30, 0.26, 0.55}, {0.30, -0.26, 0.30}}}, /* Cross half */
    {{{0.30, 0.26, 0.45}, {0.30, -0.26, 0.45}}}, /* Cross open */
    {{{0.34, 0.22, 0.86}, {0.34, -0.22, 0.86}}}, /* Clap open */
    {{{0.34, 0.26, 0.90}, {0.34, -0.26, 0.90}}}, /* Heart lobe */
    {{{0.33, 0.28, 0.86}, {0.33, -0.28, 0.86}}}, /* Heart outer */
    {{{0.32, 0.25, 0.82}, {0.32, -0.25, 0.82}}}, /* Heart shoulder */
    {{{0.31, 0.18, 0.78}, {0.31, -0.18, 0.78}}}, /* Heart lower */
    {{{0.30, 0.22, 0.74}, {0.30, -0.22, 0.74}}}, /* Heart bottom ready */
    {{{0.30, 0.22, 0.40}, {0.30, -0.22, 0.40}}}, /* Mirrored mid */
    {{{0.30, 0.22, 0.30}, {0.30, -0.22, 0.30}}}, /* Forward mid */
    {{{0.15, 0.22, 0.15}, {0.15, -0.22, 0.15}}}, /* Neutral low */
    {{{0.34, 0.24, 0.30}, {0.34, -0.24, 0.30}}}, /* Box approach */
    {{{0.34, 0.15, 0.30}, {0.34, -0.15, 0.30}}}, /* Box grasp */
    {{{0.34, 0.15, 0.48}, {0.34, -0.15, 0.48}}}, /* Box lift */
    {{{0.26, 0.15, 0.50}, {0.26, -0.15, 0.50}}}, /* Box carry */
    {{{0.26, 0.15, 0.32}, {0.26, -0.15, 0.32}}}, /* Box place */
    {{{0.26, 0.24, 0.32}, {0.26, -0.24, 0.32}}}, /* Box release */
    {{{0.30, 0.30, 0.55}, {0.30, -0.30, 0.55}}}, /* Wide high hub */
    {{{0.22, 0.30, 0.35}, {0.22, -0.30, 0.35}}}  /* Wide low hub */
};

static int finite_array(const double *value, size_t count) {
    size_t index;
    for (index = 0u; index < count; ++index) {
        if (!isfinite(value[index])) return 0;
    }
    return 1;
}

static const oa_model *model_for_side(size_t side) {
    return side == 0u ? oa_model_left_v10_bimanual() :
                        oa_model_right_v10_bimanual();
}

static int q_in_bounds(size_t side, const double q[OA_DOF], double tolerance) {
    size_t joint;
    const oa_model *model = model_for_side(side);
    for (joint = 0u; joint < OA_DOF; ++joint) {
        double lower = 0.0;
        double upper = 0.0;
        if (!isfinite(q[joint]) ||
            oa_model_limits(model, joint, &lower, &upper) != OA_MODEL_OK ||
            q[joint] < lower - tolerance || q[joint] > upper + tolerance) {
            return 0;
        }
    }
    return 1;
}

static int solve(size_t side, const double target[3], const double seed[OA_DOF],
                 double maximum_branch_step_rad, double out_q[OA_DOF]) {
    oa_ik_options options;
    oa_ik_diagnostics diagnostics;
    size_t joint;
    memset(&options, 0, sizeof(options));
    options.abi_version = OA_MODEL_ABI_VERSION;
    options.struct_size = (uint32_t)sizeof(options);
    memcpy(options.seed, seed, sizeof(options.seed));
    memcpy(options.posture, seed, sizeof(options.posture));
    for (joint = 0u; joint < OA_DOF; ++joint) options.posture_weight[joint] = 1.0;
    options.position_tolerance_m = 1.0e-6;
    options.max_joint_step_rad = 0.12;
    options.damping_min = 1.0e-5;
    options.damping_max = 0.1;
    options.limit_margin_rad = 1.0e-5;
    options.max_iterations = 500u;
    memset(&diagnostics, 0, sizeof(diagnostics));
    if (oa_ik_position_v2(model_for_side(side), target, &options,
                          OA_IK_DIAGNOSTICS_VERSION,
                          (uint32_t)sizeof(diagnostics), &diagnostics) != OA_MODEL_OK ||
        diagnostics.collision_checked != 0u ||
        !isfinite(diagnostics.position_error_m) ||
        diagnostics.position_error_m > 1.0e-5 ||
        !q_in_bounds(side, diagnostics.q, 0.0)) {
        return 0;
    }
    for (joint = 0u; joint < OA_DOF; ++joint) {
        if (fabs(diagnostics.q[joint] - seed[joint]) > maximum_branch_step_rad) return 0;
    }
    memcpy(out_q, diagnostics.q, sizeof(diagnostics.q));
    return 1;
}

static int evaluate_q(double q[2][OA_DOF], double threshold,
                      double *minimum_clearance_m) {
    oa_fk_result fk[2];
    oa_collision_report report;
    oa_collision_contact_evidence evidence;
    if (oa_fk(model_for_side(0u), q[0], &fk[0]) != OA_MODEL_OK ||
        oa_fk(model_for_side(1u), q[1], &fk[1]) != OA_MODEL_OK) return 0;
    memset(&report, 0, sizeof(report));
    memset(&evidence, 0, sizeof(evidence));
    if (oa_collision_evaluate_scoped_fk_with_threshold(
            &fk[0], &fk[1], threshold, OA_COLLISION_CONTACT_NONE,
            &report, &evidence) != OA_MODEL_OK) return 0;
    *minimum_clearance_m = report.minimum_clearance_m;
    return report.clear != 0u;
}

static int tcp_from_q(double q[2][OA_DOF], double tcp[2][3]) {
    size_t side;
    for (side = 0u; side < 2u; ++side) {
        oa_fk_result fk;
        if (oa_fk(model_for_side(side), q[side], &fk) != OA_MODEL_OK) return 0;
        tcp[side][0] = fk.hand_tcp.m[3];
        tcp[side][1] = fk.hand_tcp.m[7];
        tcp[side][2] = fk.hand_tcp.m[11];
    }
    return 1;
}

static int validate_edge(const oa_route_request *request,
                         double start_q[2][OA_DOF],
                         const route_pose *target, double out_q[2][OA_DOF],
                         double *edge_minimum_clearance_m, int *used_recovery) {
    double start_tcp[2][3];
    double q[2][OA_DOF];
    double previous_clearance = -DBL_MAX;
    double minimum_clearance = DBL_MAX;
    int recovering;
    size_t sample;
    size_t side;
    const int preserved_side =
        (request->flags & OA_ROUTE_PRESERVE_LEFT) != 0u ? 0 :
        ((request->flags & OA_ROUTE_PRESERVE_RIGHT) != 0u ? 1 : -1);
    if (!tcp_from_q(start_q, start_tcp)) return 0;
    if (preserved_side >= 0) {
        size_t axis;
        for (axis = 0u; axis < 3u; ++axis) {
            if (fabs(target->tcp[preserved_side][axis] -
                     start_tcp[preserved_side][axis]) >
                OA_ROUTE_TCP_EQUIVALENCE_TOLERANCE_M) return 0;
        }
    }
    memcpy(q, start_q, sizeof(q));
    recovering = !evaluate_q(q, oa_collision_required_clearance_m(),
                             &previous_clearance);
    if (recovering) {
        double intervention_clearance = -DBL_MAX;
        if ((request->flags & OA_ROUTE_ALLOW_CLEARANCE_RECOVERY) == 0u ||
            !evaluate_q(q, oa_collision_intervention_clearance_m(),
                        &intervention_clearance)) return 0;
        previous_clearance = intervention_clearance;
        *used_recovery = 1;
    }
    minimum_clearance = previous_clearance;
    for (sample = 1u; sample < OA_ROUTE_SAMPLES; ++sample) {
        const double fraction = (double)sample / (double)(OA_ROUTE_SAMPLES - 1u);
        for (side = 0u; side < 2u; ++side) {
            double waypoint[3];
            size_t axis;
            if ((int)side == preserved_side) {
                memcpy(out_q[side], start_q[side], sizeof(out_q[side]));
                continue;
            }
            for (axis = 0u; axis < 3u; ++axis) {
                waypoint[axis] = start_tcp[side][axis] + fraction *
                    (target->tcp[side][axis] - start_tcp[side][axis]);
            }
            if (!solve(side, waypoint, q[side], request->maximum_branch_step_rad,
                       out_q[side])) return 0;
        }
        memcpy(q, out_q, sizeof(q));
        {
            double clearance = -DBL_MAX;
            const int planning_clear = evaluate_q(
                q, oa_collision_required_clearance_m(), &clearance);
            if (recovering) {
                if (!planning_clear) {
                    double intervention_clearance = -DBL_MAX;
                    if (!evaluate_q(q, oa_collision_intervention_clearance_m(),
                                    &intervention_clearance) ||
                        intervention_clearance < previous_clearance -
                            OA_ROUTE_CLEARANCE_EPSILON) return 0;
                    clearance = intervention_clearance;
                } else {
                    recovering = 0;
                }
            } else if (!planning_clear) {
                return 0;
            }
            if (clearance < minimum_clearance) minimum_clearance = clearance;
            previous_clearance = clearance;
        }
    }
    /* Recovery is an escape, not permission to choose another endpoint inside
     * the planning gate. The edge must finish with ordinary 25 mm clearance. */
    if (recovering) return 0;
    memcpy(out_q, q, sizeof(q));
    *edge_minimum_clearance_m = minimum_clearance;
    return 1;
}

oa_route_status oa_route_plan_paired(const oa_route_request *request,
                                     oa_route_result *out) {
    route_pose candidates[OA_ROUTE_CANDIDATES];
    route_node nodes[OA_ROUTE_CANDIDATES];
    int queue[OA_ROUTE_CANDIDATES + 1u];
    int reverse_path[OA_ROUTE_CANDIDATES];
    size_t queue_head = 0u;
    size_t queue_tail = 0u;
    size_t side;
    size_t index;
    int target_node = -1;
    double start_q[2][OA_DOF];
    double start_tcp[2][3];
    double start_minimum = DBL_MAX;
    int preserved_side;
    if (request == NULL || out == NULL ||
        request->abi_version != OA_ROUTE_ABI_VERSION ||
        request->struct_size < (uint32_t)sizeof(*request) ||
        out->abi_version != OA_ROUTE_ABI_VERSION ||
        out->struct_size < (uint32_t)sizeof(*out) || request->reserved0 != 0u ||
        (request->flags & ~(OA_ROUTE_ALLOW_CLEARANCE_RECOVERY |
                            OA_ROUTE_PRESERVE_LEFT |
                            OA_ROUTE_PRESERVE_RIGHT)) != 0u ||
        (request->flags & (OA_ROUTE_PRESERVE_LEFT | OA_ROUTE_PRESERVE_RIGHT)) ==
            (OA_ROUTE_PRESERVE_LEFT | OA_ROUTE_PRESERVE_RIGHT) ||
        !isfinite(request->maximum_branch_step_rad) ||
        request->maximum_branch_step_rad <= 0.0) return OA_ROUTE_EINVAL;
    if (!finite_array(&request->start_q_rad[0][0], 2u * OA_DOF) ||
        !finite_array(&request->target_tcp_m[0][0], 6u)) return OA_ROUTE_ENONFINITE;
    for (side = 0u; side < 2u; ++side) {
        if (!q_in_bounds(side, request->start_q_rad[side],
                         OA_ROUTE_ENCODER_LIMIT_TOLERANCE)) return OA_ROUTE_EINVAL;
    }
    memset(out, 0, sizeof(*out));
    out->abi_version = OA_ROUTE_ABI_VERSION;
    out->struct_size = (uint32_t)sizeof(*out);
    out->minimum_clearance_m = DBL_MAX;
    memset(nodes, 0, sizeof(nodes));
    memcpy(start_q, request->start_q_rad, sizeof(start_q));
    if (!tcp_from_q(start_q, start_tcp)) return OA_ROUTE_EINVAL;
    memcpy(candidates[0].tcp, request->target_tcp_m, sizeof(candidates[0].tcp));
    memcpy(&candidates[1], route_anchors, sizeof(route_anchors));
    preserved_side =
        (request->flags & OA_ROUTE_PRESERVE_LEFT) != 0u ? 0 :
        ((request->flags & OA_ROUTE_PRESERVE_RIGHT) != 0u ? 1 : -1);
    if (preserved_side >= 0) {
        size_t axis;
        for (axis = 0u; axis < 3u; ++axis) {
            if (fabs(request->target_tcp_m[preserved_side][axis] -
                     start_tcp[preserved_side][axis]) >
                OA_ROUTE_TCP_EQUIVALENCE_TOLERANCE_M) return OA_ROUTE_EINVAL;
        }
        for (index = 0u; index < OA_ROUTE_CANDIDATES; ++index) {
            memcpy(candidates[index].tcp[preserved_side], start_tcp[preserved_side],
                   sizeof(candidates[index].tcp[preserved_side]));
        }
    }

    /* -1 in the queue is the measured start, which is not itself a candidate. */
    queue[queue_tail++] = -1;
    while (queue_head < queue_tail && target_node < 0) {
        const int current = queue[queue_head++];
        double (*current_q)[OA_DOF] = current < 0 ? start_q : nodes[current].q;
        for (index = 0u; index < OA_ROUTE_CANDIDATES; ++index) {
            double terminal_q[2][OA_DOF];
            double edge_minimum = DBL_MAX;
            int edge_recovery = 0;
            if (nodes[index].discovered) continue;
            if (!validate_edge(request, current_q, &candidates[index], terminal_q,
                               &edge_minimum, &edge_recovery)) continue;
            nodes[index].discovered = 1;
            nodes[index].predecessor = current;
            nodes[index].used_recovery = edge_recovery ||
                (current >= 0 && nodes[current].used_recovery);
            nodes[index].edge_minimum_clearance_m = edge_minimum;
            memcpy(nodes[index].q, terminal_q, sizeof(nodes[index].q));
            if (index == 0u) {
                target_node = 0;
                break;
            }
            queue[queue_tail++] = (int)index;
        }
    }
    if (target_node < 0) return OA_ROUTE_ENOPATH;
    {
        size_t count = 0u;
        int cursor = target_node;
        while (cursor >= 0) {
            if (count >= OA_ROUTE_MAX_WAYPOINTS) return OA_ROUTE_ECAPACITY;
            reverse_path[count++] = cursor;
            cursor = nodes[cursor].predecessor;
        }
        out->waypoint_count = (uint32_t)count;
        out->used_clearance_recovery = nodes[target_node].used_recovery ? 1u : 0u;
        while (count > 0u) {
            const int node_index = reverse_path[--count];
            const size_t output_index = (size_t)out->waypoint_count - count - 1u;
            memcpy(out->waypoint_tcp_m[output_index], candidates[node_index].tcp,
                   sizeof(out->waypoint_tcp_m[output_index]));
            memcpy(out->waypoint_q_rad[output_index], nodes[node_index].q,
                   sizeof(out->waypoint_q_rad[output_index]));
            if (nodes[node_index].edge_minimum_clearance_m < start_minimum)
                start_minimum = nodes[node_index].edge_minimum_clearance_m;
        }
    }
    out->minimum_clearance_m = start_minimum;
    return OA_ROUTE_OK;
}
