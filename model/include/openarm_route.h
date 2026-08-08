/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Collision-aware Cartesian routing for the OpenArm v1.0 bimanual model.
 *
 * This API is implemented in C and keeps every coordinate and calculation in
 * IEEE-754 binary64.  It plans a sequence of paired TCP endpoints; callers
 * execute the returned endpoints in order.  Every edge is sampled through the
 * public IK/FK model and the same conservative arm/tool-capsule and finite-pole
 * keepout evaluator used by the real-time controller.  The dedicated
 * exact-mesh terminal-contact corridor is intentionally not an ordinary route.
 */
#ifndef OPENARM_ROUTE_H
#define OPENARM_ROUTE_H

#include <stdint.h>

#include "openarm_collision.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OA_ROUTE_ABI_VERSION 1u
#define OA_ROUTE_MAX_WAYPOINTS 24u

typedef int32_t oa_route_status;
#define OA_ROUTE_OK ((oa_route_status)0)
#define OA_ROUTE_EINVAL ((oa_route_status)1)
#define OA_ROUTE_ENONFINITE ((oa_route_status)2)
#define OA_ROUTE_ENOPATH ((oa_route_status)3)
#define OA_ROUTE_ECAPACITY ((oa_route_status)4)

/* Permit a first edge to leave a measured pose that is inside the 25 mm
 * planning gate but still outside the 10 mm intervention floor.  Such an edge
 * is accepted only while its clearance is monotonically non-decreasing, and
 * normal 25 mm enforcement resumes irreversibly as soon as it is recovered. */
#define OA_ROUTE_ALLOW_CLEARANCE_RECOVERY UINT32_C(1)
/* Keep one arm at its exact measured start joint vector throughout every
 * sampled edge and returned waypoint. The target TCP for that side must equal
 * FK(start_q_rad) within the planner's Cartesian tolerance. These flags are
 * mutually exclusive. */
#define OA_ROUTE_PRESERVE_LEFT UINT32_C(2)
#define OA_ROUTE_PRESERVE_RIGHT UINT32_C(4)

typedef struct oa_route_request {
    uint32_t abi_version;
    uint32_t struct_size;
    uint32_t flags;
    uint32_t reserved0;
    /* Model-joint radians, [left/right][J1..J7]. */
    double start_q_rad[2][OA_DOF];
    /* openarm_body_link0 metres, [left/right][x/y/z]. */
    double target_tcp_m[2][3];
    /* Maximum change between adjacent 1/16 Cartesian edge samples. */
    double maximum_branch_step_rad;
} oa_route_request;

typedef struct oa_route_result {
    uint32_t abi_version;
    uint32_t struct_size;
    uint32_t waypoint_count;
    uint32_t used_clearance_recovery;
    double minimum_clearance_m;
    /* Endpoints are ordered for execution and do not repeat the start pose. */
    double waypoint_tcp_m[OA_ROUTE_MAX_WAYPOINTS][2][3];
    double waypoint_q_rad[OA_ROUTE_MAX_WAYPOINTS][2][OA_DOF];
} oa_route_result;

/* Finds an exact route to target_tcp_m.  It never projects or clamps the
 * target.  OA_ROUTE_ENOPATH means no path was proven through the built-in
 * OpenArm v1.0 routing graph; it does not prove that no path exists in the
 * continuous 14-DOF configuration space. */
oa_route_status oa_route_plan_paired(const oa_route_request *request,
                                     oa_route_result *out);

#ifdef __cplusplus
}
#endif

#endif
