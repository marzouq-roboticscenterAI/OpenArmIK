/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Canonical OpenArm v1.0 bimanual keepout geometry.
 *
 * This is the single source of truth for the nominal collision model used by
 * the pre-flight path guard and by the real-time execution monitor. Both must
 * agree exactly; a divergence between "the plan was checked" and "the arm is
 * checked while moving" is a safety defect, not a rounding difference.
 *
 * The model is nominal and conservative, not a certified collision proof. It
 * covers arm-arm capsule pairs and the central body shaft. It does not model
 * the environment, payloads, cabling, or the mounting surface.
 */
#ifndef OPENARM_COLLISION_H
#define OPENARM_COLLISION_H

#include <stdint.h>

#include "openarm_model.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OA_COLLISION_ABI_VERSION 1u

/* Seven joint origins followed by the hand_tcp origin. Consecutive points form
 * the seven capsule segments of one arm. */
#define OA_COLLISION_POINTS 8u
#define OA_COLLISION_SEGMENTS 7u

typedef uint32_t oa_collision_violation;
#define OA_COLLISION_VIOLATION_NONE ((oa_collision_violation)0)
#define OA_COLLISION_VIOLATION_ARM_ARM ((oa_collision_violation)1)
#define OA_COLLISION_VIOLATION_POLE ((oa_collision_violation)2)
#define OA_COLLISION_VIOLATION_NONFINITE ((oa_collision_violation)3)

typedef struct oa_collision_scene {
    uint32_t abi_version;
    uint32_t struct_size;
    /* [side][point][xyz] in openarm_body_link0. Side 0 is left, 1 is right. */
    double point[2][OA_COLLISION_POINTS][3];
} oa_collision_scene;

typedef struct oa_collision_report {
    uint32_t abi_version;
    uint32_t struct_size;
    /* Nonzero only when every checked pair meets the required clearance. */
    uint32_t clear;
    oa_collision_violation violation;
    /* Populated only when violation != NONE. For ARM_ARM, segment_a indexes the
     * left arm and segment_b the right. For POLE, side names the arm and
     * segment_a its segment; segment_b is unused. */
    uint32_t side;
    uint32_t segment_a;
    uint32_t segment_b;
    double minimum_clearance_m;
    double required_clearance_m;
} oa_collision_report;

/* A deliberately narrow, command-scoped exception for intentional contact.
 * NONE is the default for every ordinary command. TERMINAL_CAPS may exclude
 * only a facing left/right distal TCP spherical-cap pair; every other
 * arm-arm pair and every pole check remains enforced. */
typedef uint32_t oa_collision_contact_policy;
#define OA_COLLISION_CONTACT_NONE ((oa_collision_contact_policy)0)
#define OA_COLLISION_CONTACT_TERMINAL_CAPS ((oa_collision_contact_policy)1)

typedef struct oa_collision_contact_evidence {
    uint32_t abi_version;
    uint32_t struct_size;
    uint32_t terminal_pair_active;
    uint32_t reserved0;
    double terminal_pair_clearance_m;
    double terminal_parameter_left;
    double terminal_parameter_right;
    double tcp_separation_m;
    /* Signed exact-triangle gap between the two pinned hand.stl meshes.
     * Positive is separated, zero is tangent, negative is penetration. */
    double claw_hand_gap_m;
    /* Minimum signed exact-triangle gap across the other eight cross-claw
     * hand/finger mesh pairs. */
    double minimum_other_claw_gap_m;
    /* Nonzero when the two hand/rail meshes enter the expanded safety
     * envelope. The physical meshes remain separated; see
     * oa_collision_claw_rail_clearance_m(). */
    uint32_t claw_contact_active;
    uint32_t reserved1;
} oa_collision_contact_evidence;

/* Nominal keepout parameters. Exposed so callers report the same numbers they
 * are gated on rather than restating literals. */
double oa_collision_required_clearance_m(void);
/* Threshold at which the real-time execution monitor halts a moving arm.
 *
 * This is deliberately smaller than the planning clearance. A planner accepts a
 * path only if every nominal waypoint holds oa_collision_required_clearance_m,
 * and it may legitimately accept one that sits exactly on that limit. The
 * measured arm always lags its reference slightly, so a monitor sharing the
 * planning threshold would abort motions the planner deliberately allowed.
 * Intervening at this tighter floor still cannot be reached by tracking error
 * from a validated path, but does catch genuine divergence. */
double oa_collision_intervention_clearance_m(void);
double oa_collision_arm_radius_m(void);
double oa_collision_tool_radius_m(void);
/* Extra separation retained between the two exact hand/rail STL meshes during
 * intentional Heart/Clap convergence. This is a model-to-hardware margin for
 * encoder quantization, calibration error, and tracking error. */
double oa_collision_claw_rail_clearance_m(void);

/* Minimum distance between two capsule centrelines, minus the summed radii. */
double oa_collision_segment_clearance(const double a0[3], const double a1[3],
                                      const double b0[3], const double b1[3],
                                      double radius_a, double radius_b);

/* Clearance between a capsule and a finite upright cylinder centred on the body
 * Z axis. Returns -INFINITY for any non-finite or malformed input so callers
 * fail closed. */
double oa_collision_finite_cylinder_capsule_clearance(
    const double a[3], const double b[3], double cylinder_radius,
    double cylinder_bottom, double cylinder_top, double capsule_radius);

/* Evaluates the full nominal scene. Returns OA_MODEL_EINVAL for an invalid ABI
 * or null argument, OA_MODEL_ENONFINITE when any input coordinate is not
 * finite, and OA_MODEL_OK otherwise. A clear scene is reported by
 * out->clear, not by the status: OA_MODEL_OK with clear == 0 is a violation. */
oa_model_status oa_collision_evaluate(const oa_collision_scene *scene,
                                      oa_collision_report *out);

/* As oa_collision_evaluate, but gates on an explicit clearance. Used by the
 * real-time monitor so planning and monitoring thresholds stay distinct and
 * both are visible in the report. */
oa_model_status oa_collision_evaluate_with_threshold(const oa_collision_scene *scene,
                                                     double required_clearance_m,
                                                     oa_collision_report *out);

/* Evaluates the same nominal scene with an immutable contact policy carried by
 * the active motion plan. The protected minimum in `out` never includes an
 * allowed terminal pair, so intentional contact cannot hide a worsening link
 * or pole clearance. This is nominal TCP-cap contact, not a certified
 * finger-mesh collision proof. */
oa_model_status oa_collision_evaluate_scoped_with_threshold(
    const oa_collision_scene *scene, double required_clearance_m,
    oa_collision_contact_policy contact_policy, oa_collision_report *out,
    oa_collision_contact_evidence *contact_evidence);

/* FK-backed scoped evaluation used by intentional claw-contact commands. In
 * addition to every capsule and pole check above, this evaluates the exact
 * triangles embedded from the pinned hand.stl and finger.stl collision meshes
 * using double-precision FCL. The mutually facing hand/rail pair may enter only
 * its expanded safety envelope; the physical STL meshes must remain separated.
 * All eight other cross-claw mesh pairs retain the requested clearance. */
oa_model_status oa_collision_evaluate_scoped_fk_with_threshold(
    const oa_fk_result *left, const oa_fk_result *right,
    double required_clearance_m, oa_collision_contact_policy contact_policy,
    oa_collision_report *out,
    oa_collision_contact_evidence *contact_evidence);

/* Fills a scene from two forward-kinematics results. */
oa_model_status oa_collision_scene_from_fk(const oa_fk_result *left,
                                           const oa_fk_result *right,
                                           oa_collision_scene *out);

#ifdef __cplusplus
}
#endif
#endif
