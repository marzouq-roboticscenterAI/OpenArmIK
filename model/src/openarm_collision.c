/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_collision.h"

#include <math.h>
#include <stddef.h>

/* Nominal link envelope. These are the audited pre-flight guard constants; the
 * real-time monitor reuses the same geometry so the two gates cannot diverge,
 * but intervenes at a tighter clearance (see kInterventionClearance). */
static const double kArmRadius = 0.050;
static const double kToolRadius = 0.075;
static const double kRequiredClearance = 0.025;
/* 10 mm. Well below the 25 mm planning gate so ordinary tracking lag on a
 * validated path cannot reach it, and well above zero so a genuine divergence
 * is caught before contact. */
static const double kInterventionClearance = 0.010;
/* The canonical body mesh contains a 60 x 60 mm central shaft. Its
 * circumscribed cylinder conservatively covers the square. */
static const double kPoleRadius = 0.04242640687119285;
static const double kPoleBottom = 0.008;
static const double kPoleTop = 0.758;

static double clamp01(const double value) {
    if (value < 0.0) {
        return 0.0;
    }
    if (value > 1.0) {
        return 1.0;
    }
    return value;
}

static double dot3(const double a[3], const double b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void subtract3(const double a[3], const double b[3], double out[3]) {
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

static void add_scaled3(const double a[3], const double b[3], const double scale,
                        double out[3]) {
    out[0] = a[0] + b[0] * scale;
    out[1] = a[1] + b[1] * scale;
    out[2] = a[2] + b[2] * scale;
}

static double norm3(const double a[3]) {
    return sqrt(dot3(a, a));
}

static int finite3(const double a[3]) {
    return isfinite(a[0]) && isfinite(a[1]) && isfinite(a[2]);
}

double oa_collision_required_clearance_m(void) {
    return kRequiredClearance;
}

double oa_collision_intervention_clearance_m(void) {
    return kInterventionClearance;
}

double oa_collision_arm_radius_m(void) {
    return kArmRadius;
}

double oa_collision_tool_radius_m(void) {
    return kToolRadius;
}

double oa_collision_segment_clearance(const double a0[3], const double a1[3],
                                      const double b0[3], const double b1[3],
                                      const double radius_a, const double radius_b) {
    double d1[3];
    double d2[3];
    double r[3];
    double point_a[3];
    double point_b[3];
    double difference[3];
    double s = 0.0;
    double t = 0.0;
    double a;
    double e;
    double f;

    if (a0 == NULL || a1 == NULL || b0 == NULL || b1 == NULL) {
        return -INFINITY;
    }
    if (!finite3(a0) || !finite3(a1) || !finite3(b0) || !finite3(b1) ||
        !isfinite(radius_a) || !isfinite(radius_b) || radius_a < 0.0 || radius_b < 0.0) {
        return -INFINITY;
    }

    subtract3(a1, a0, d1);
    subtract3(b1, b0, d2);
    subtract3(a0, b0, r);
    a = dot3(d1, d1);
    e = dot3(d2, d2);
    f = dot3(d2, r);

    if (a <= 1.0e-18 && e <= 1.0e-18) {
        return norm3(r) - (radius_a + radius_b);
    }
    if (a <= 1.0e-18) {
        t = clamp01(f / e);
    } else {
        const double c = dot3(d1, r);
        if (e <= 1.0e-18) {
            s = clamp01(-c / a);
        } else {
            const double b = dot3(d1, d2);
            const double denominator = a * e - b * b;
            if (denominator > 1.0e-18) {
                s = clamp01((b * f - c * e) / denominator);
            }
            t = (b * s + f) / e;
            if (t < 0.0) {
                t = 0.0;
                s = clamp01(-c / a);
            } else if (t > 1.0) {
                t = 1.0;
                s = clamp01((b - c) / a);
            }
        }
    }
    add_scaled3(a0, d1, s, point_a);
    add_scaled3(b0, d2, t, point_b);
    subtract3(point_a, point_b, difference);
    return norm3(difference) - (radius_a + radius_b);
}

static double point_cylinder_distance_squared(const double point[3], const double radius,
                                              const double bottom, const double top) {
    const double planar = hypot(point[0], point[1]);
    const double radial_gap = planar > radius ? planar - radius : 0.0;
    double axial_gap = 0.0;
    if (point[2] < bottom) {
        axial_gap = bottom - point[2];
    } else if (point[2] > top) {
        axial_gap = point[2] - top;
    }
    return radial_gap * radial_gap + axial_gap * axial_gap;
}

double oa_collision_finite_cylinder_capsule_clearance(
    const double a[3], const double b[3], const double cylinder_radius,
    const double cylinder_bottom, const double cylinder_top, const double capsule_radius) {
    double direction[3];
    double low = 0.0;
    double high = 1.0;
    double minimum;
    double candidate;
    double sample[3];
    size_t iteration;

    if (a == NULL || b == NULL) {
        return -INFINITY;
    }
    if (!finite3(a) || !finite3(b) || !isfinite(cylinder_radius) ||
        !isfinite(cylinder_bottom) || !isfinite(cylinder_top) ||
        !isfinite(capsule_radius) || cylinder_radius < 0.0 ||
        cylinder_bottom > cylinder_top || capsule_radius < 0.0) {
        return -INFINITY;
    }

    subtract3(b, a, direction);
    /* Squared distance to a closed convex set is convex along a segment.
     * Ternary minimization therefore covers the cylindrical side, caps, and rim
     * without axial clipping holes. */
    for (iteration = 0; iteration < 96u; ++iteration) {
        const double first = (2.0 * low + high) / 3.0;
        const double second = (low + 2.0 * high) / 3.0;
        double first_value;
        double second_value;
        add_scaled3(a, direction, first, sample);
        first_value = point_cylinder_distance_squared(sample, cylinder_radius,
                                                      cylinder_bottom, cylinder_top);
        add_scaled3(a, direction, second, sample);
        second_value = point_cylinder_distance_squared(sample, cylinder_radius,
                                                       cylinder_bottom, cylinder_top);
        if (first_value <= second_value) {
            high = second;
        } else {
            low = first;
        }
    }
    /* Evaluate the endpoints through the same parameterization used by the
     * search above. Substituting a and b directly is algebraically identical
     * but differs in the final ULP, which would desynchronize this gate from
     * the pre-flight guard it must agree with. */
    add_scaled3(a, direction, 0.0, sample);
    minimum = point_cylinder_distance_squared(sample, cylinder_radius, cylinder_bottom,
                                              cylinder_top);
    add_scaled3(a, direction, 1.0, sample);
    candidate = point_cylinder_distance_squared(sample, cylinder_radius, cylinder_bottom,
                                                cylinder_top);
    if (candidate < minimum) {
        minimum = candidate;
    }
    add_scaled3(a, direction, (low + high) / 2.0, sample);
    candidate = point_cylinder_distance_squared(sample, cylinder_radius, cylinder_bottom,
                                                cylinder_top);
    if (candidate < minimum) {
        minimum = candidate;
    }
    /* Bias downward so rounding cannot manufacture clearance. */
    minimum = sqrt(minimum) - 1.0e-9;
    if (minimum < 0.0) {
        minimum = 0.0;
    }
    return minimum - capsule_radius;
}

oa_model_status oa_collision_evaluate(const oa_collision_scene *scene,
                                      oa_collision_report *out) {
    return oa_collision_evaluate_with_threshold(scene, kRequiredClearance, out);
}

oa_model_status oa_collision_evaluate_with_threshold(const oa_collision_scene *scene,
                                                     const double required_clearance_m,
                                                     oa_collision_report *out) {
    size_t side;
    size_t left;
    size_t right;
    size_t segment;
    double minimum = INFINITY;

    if (out == NULL) {
        return OA_MODEL_EINVAL;
    }
    out->abi_version = OA_COLLISION_ABI_VERSION;
    out->struct_size = (uint32_t)sizeof(*out);
    out->clear = 0u;
    out->violation = OA_COLLISION_VIOLATION_NONE;
    out->side = 0u;
    out->segment_a = 0u;
    out->segment_b = 0u;
    out->minimum_clearance_m = -INFINITY;
    out->required_clearance_m = required_clearance_m;

    if (!isfinite(required_clearance_m) || required_clearance_m < 0.0) {
        return OA_MODEL_EINVAL;
    }
    if (scene == NULL || scene->abi_version != OA_COLLISION_ABI_VERSION ||
        scene->struct_size < (uint32_t)sizeof(*scene)) {
        return OA_MODEL_EINVAL;
    }
    for (side = 0; side < 2u; ++side) {
        size_t index;
        for (index = 0; index < OA_COLLISION_POINTS; ++index) {
            if (!finite3(scene->point[side][index])) {
                out->violation = OA_COLLISION_VIOLATION_NONFINITE;
                out->side = (uint32_t)side;
                out->segment_a = (uint32_t)index;
                return OA_MODEL_ENONFINITE;
            }
        }
    }

    for (left = 0; left < OA_COLLISION_SEGMENTS; ++left) {
        for (right = 0; right < OA_COLLISION_SEGMENTS; ++right) {
            const double radius_left = left == 6u ? kToolRadius : kArmRadius;
            const double radius_right = right == 6u ? kToolRadius : kArmRadius;
            const double value = oa_collision_segment_clearance(
                scene->point[0][left], scene->point[0][left + 1u],
                scene->point[1][right], scene->point[1][right + 1u], radius_left,
                radius_right);
            if (value < minimum) {
                minimum = value;
            }
            if (isnan(value) || value < required_clearance_m) {
                out->violation = OA_COLLISION_VIOLATION_ARM_ARM;
                out->segment_a = (uint32_t)left;
                out->segment_b = (uint32_t)right;
                out->minimum_clearance_m = minimum;
                return OA_MODEL_OK;
            }
        }
    }

    for (side = 0; side < 2u; ++side) {
        for (segment = 0; segment < OA_COLLISION_SEGMENTS; ++segment) {
            /* Canonical link1 begins at J1 and lies wholly outward along the J1
             * radial axis. Joint1 only rolls its cross-section about that axis,
             * so no link1 vertex lies radially inward of this centreline. Using
             * the generic isotropic radius here would fabricate inward mount
             * volume; the centreline is its conservative shaft-facing envelope. */
            const double radius =
                segment == 0u ? 0.0 : (segment == 6u ? kToolRadius : kArmRadius);
            const double value = oa_collision_finite_cylinder_capsule_clearance(
                scene->point[side][segment], scene->point[side][segment + 1u],
                kPoleRadius, kPoleBottom, kPoleTop, radius);
            if (value < minimum) {
                minimum = value;
            }
            if (isnan(value) || value < required_clearance_m) {
                out->violation = OA_COLLISION_VIOLATION_POLE;
                out->side = (uint32_t)side;
                out->segment_a = (uint32_t)segment;
                out->minimum_clearance_m = minimum;
                return OA_MODEL_OK;
            }
        }
    }

    out->clear = 1u;
    out->minimum_clearance_m = minimum;
    return OA_MODEL_OK;
}

oa_model_status oa_collision_scene_from_fk(const oa_fk_result *left,
                                           const oa_fk_result *right,
                                           oa_collision_scene *out) {
    const oa_fk_result *source[2];
    size_t side;

    if (out == NULL) {
        return OA_MODEL_EINVAL;
    }
    out->abi_version = OA_COLLISION_ABI_VERSION;
    out->struct_size = (uint32_t)sizeof(*out);
    for (side = 0; side < 2u; ++side) {
        size_t index;
        for (index = 0; index < OA_COLLISION_POINTS; ++index) {
            out->point[side][index][0] = 0.0;
            out->point[side][index][1] = 0.0;
            out->point[side][index][2] = 0.0;
        }
    }
    if (left == NULL || right == NULL) {
        return OA_MODEL_EINVAL;
    }
    source[0] = left;
    source[1] = right;
    for (side = 0; side < 2u; ++side) {
        size_t joint;
        for (joint = 0; joint < OA_DOF; ++joint) {
            out->point[side][joint][0] = source[side]->joint_pre[joint].m[3];
            out->point[side][joint][1] = source[side]->joint_pre[joint].m[7];
            out->point[side][joint][2] = source[side]->joint_pre[joint].m[11];
        }
        out->point[side][OA_DOF][0] = source[side]->hand_tcp.m[3];
        out->point[side][OA_DOF][1] = source[side]->hand_tcp.m[7];
        out->point[side][OA_DOF][2] = source[side]->hand_tcp.m[11];
    }
    return OA_MODEL_OK;
}
