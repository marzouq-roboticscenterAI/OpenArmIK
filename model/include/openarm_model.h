/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Dependency-free OpenArm v1.0 bimanual kinematics model.
 * Generated data is derived from enactic/openarm_description@6c7b720f.
 */
#ifndef OPENARM_MODEL_H
#define OPENARM_MODEL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OA_MODEL_ABI_VERSION 1u
#define OA_DOF 7u

/* Rigid transform from the named parent frame to child frame, row-major 4x4.
 * Vectors are column vectors, units are metres/radians, and q is model-joint
 * coordinates (never motor coordinates). */
typedef struct oa_transform { double m[16]; } oa_transform;

typedef int32_t oa_status;
/* OA_OK: tolerance and effective bounds (including margin) satisfied.
 * OA_EINVAL: invalid ABI/options or numerically unsafe finite input.
 * OA_ENONFINITE: NaN/Inf input or non-finite internal arithmetic.
 * OA_EBOUNDS: requested limit margin leaves no feasible box.
 * OA_ENOCONVERGENCE: no strictly primary-error-decreasing feasible step.
 * OA_ESTAGNATED_AT_BOUNDS: active bounds prevent a decreasing step.
 * OA_ESINGULAR: requested damping cannot solve the rank-deficient system.
 * OA_EBUDGET: max_iterations exhausted without tolerance satisfaction. */
#define OA_OK                   ((oa_status)0)
#define OA_EINVAL               ((oa_status)1)
#define OA_ENONFINITE           ((oa_status)2)
#define OA_EBOUNDS              ((oa_status)3)
#define OA_ENOCONVERGENCE       ((oa_status)4)
#define OA_ESTAGNATED_AT_BOUNDS ((oa_status)5)
#define OA_ESINGULAR            ((oa_status)6)
#define OA_EBUDGET              ((oa_status)7)

typedef struct oa_model oa_model;

typedef struct oa_fk_result {
    oa_transform base_in_body;
    oa_transform joint_pre[OA_DOF]; /* body -> URDF joint frame, before q */
    oa_transform link_post[OA_DOF]; /* body -> child link, after q */
    double joint_axis_body[OA_DOF][3];
    oa_transform hand_tcp;           /* body -> exact named URDF hand_tcp */
} oa_fk_result;

/* Geometric Jacobian is row-major [linear xyz; angular xyz], 6 rows x 7 cols,
 * expressed in openarm_body_link0. */
typedef struct oa_jacobian { double value[6][OA_DOF]; } oa_jacobian;

typedef struct oa_ik_options {
    uint32_t abi_version;
    uint32_t struct_size;
    double seed[OA_DOF];
    double posture[OA_DOF];
    double posture_weight[OA_DOF]; /* strictly positive */
    double position_tolerance_m;
    double max_joint_step_rad;
    double damping_min;
    double damping_max;
    double limit_margin_rad;
    uint32_t max_iterations;
} oa_ik_options;

typedef struct oa_ik_diagnostics {
    uint32_t abi_version;
    uint32_t struct_size;
    oa_status status;
    uint32_t iterations;
    uint32_t active_limit_mask;
    double q[OA_DOF];
    double achieved_position_m[3];
    oa_transform achieved_hand_tcp;
    double position_error_m;
    double min_singular_value; /* weighted translational active-set Jacobian */
    uint32_t collision_checked; /* Always 0: position IK does not check collision. */
} oa_ik_diagnostics;

#define OA_IK_OPTIONS_REQUIRED_SIZE ((uint32_t)sizeof(oa_ik_options))
#define OA_IK_DIAGNOSTICS_SIZE ((uint32_t)sizeof(oa_ik_diagnostics))

const oa_model *oa_model_left_v10_bimanual(void);
const oa_model *oa_model_right_v10_bimanual(void);
const char *oa_model_id(const oa_model *model);
const char *oa_model_provenance(const oa_model *model);
const char *oa_model_data_sha256(const oa_model *model);
const char *oa_model_flattened_urdf_sha256(const oa_model *model);
const char *oa_model_source_sha256(const oa_model *model);
const char *oa_model_joint_name(const oa_model *model, size_t index);
const char *oa_model_tip_frame(const oa_model *model);
oa_status oa_model_limits(const oa_model *model, size_t index, double *lower, double *upper);

oa_status oa_fk(const oa_model *model, const double q[OA_DOF], oa_fk_result *out);
oa_status oa_geometric_jacobian(const oa_model *model, const double q[OA_DOF], oa_jacobian *out);
oa_status oa_ik_position(const oa_model *model, const double target_body_m[3],
                         const oa_ik_options *options, oa_ik_diagnostics *out);

#ifdef __cplusplus
}
#endif
#endif
