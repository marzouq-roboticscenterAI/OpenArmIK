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

#define OA_MODEL_ABI_VERSION 2u
#define OA_IK_DIAGNOSTICS_VERSION 2u
#define OA_DOF 7u

/* Rigid transform from the named parent frame to child frame, row-major 4x4.
 * Vectors are column vectors, units are metres/radians, and q is model-joint
 * coordinates (never motor coordinates). */
typedef struct oa_transform { double m[16]; } oa_transform;

typedef int32_t oa_model_status;
#define OPENARM_MODEL_STATUS_NAMESPACE_PRESENT 1
/* OA_MODEL_OK: tolerance and effective bounds (including margin) satisfied.
 * OA_MODEL_EINVAL: invalid ABI/options or numerically unsafe finite input.
 * OA_MODEL_ENONFINITE: NaN/Inf input or non-finite internal arithmetic.
 * OA_MODEL_EBOUNDS: requested limit margin leaves no feasible box.
 * OA_MODEL_ENOCONVERGENCE: no strictly primary-error-decreasing feasible step.
 * OA_MODEL_ESTAGNATED_AT_BOUNDS: active bounds prevent a decreasing step.
 * OA_MODEL_ESINGULAR: requested damping cannot solve the rank-deficient system.
 * OA_MODEL_EBUDGET: max_iterations exhausted without tolerance satisfaction. */
#define OA_MODEL_OK                   ((oa_model_status)0)
#define OA_MODEL_EINVAL               ((oa_model_status)1)
#define OA_MODEL_ENONFINITE           ((oa_model_status)2)
#define OA_MODEL_EBOUNDS              ((oa_model_status)3)
#define OA_MODEL_ENOCONVERGENCE       ((oa_model_status)4)
#define OA_MODEL_ESTAGNATED_AT_BOUNDS ((oa_model_status)5)
#define OA_MODEL_ESINGULAR            ((oa_model_status)6)
#define OA_MODEL_EBUDGET              ((oa_model_status)7)

#if defined(OPENARM_CONTROL_LEGACY_GENERIC_STATUS_ACTIVE)
#error "model and control legacy status names cannot coexist; define OPENARM_DISABLE_LEGACY_GENERIC_STATUS before both headers"
#endif
#if !defined(OPENARM_DISABLE_LEGACY_GENERIC_STATUS)
#if defined(OPENARM_CONTROL_STATUS_NAMESPACE_PRESENT)
#error "define OPENARM_DISABLE_LEGACY_GENERIC_STATUS before combining model and control headers"
#endif
#define OPENARM_MODEL_LEGACY_GENERIC_STATUS_ACTIVE 1
typedef oa_model_status oa_status;
#define OA_OK                   OA_MODEL_OK
#define OA_EINVAL               OA_MODEL_EINVAL
#define OA_ENONFINITE           OA_MODEL_ENONFINITE
#define OA_EBOUNDS              OA_MODEL_EBOUNDS
#define OA_ENOCONVERGENCE       OA_MODEL_ENOCONVERGENCE
#define OA_ESTAGNATED_AT_BOUNDS OA_MODEL_ESTAGNATED_AT_BOUNDS
#define OA_ESINGULAR            OA_MODEL_ESINGULAR
#define OA_EBUDGET              OA_MODEL_EBUDGET
#endif

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
    oa_model_status status;
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
oa_model_status oa_model_limits(const oa_model *model, size_t index, double *lower,
                                double *upper);

oa_model_status oa_fk(const oa_model *model, const double q[OA_DOF], oa_fk_result *out);
oa_model_status oa_geometric_jacobian(const oa_model *model, const double q[OA_DOF],
                                      oa_jacobian *out);
/* ABI-v1 compatibility symbol. It always returns OA_MODEL_EINVAL and never writes
 * through out. This preserves safety for callers with the old 248-byte result. */
oa_model_status oa_ik_position(const oa_model *model, const double target_body_m[3],
                               const oa_ik_options *options, void *out);

/* output_version and output_size are validated before the first output write. */
oa_model_status oa_ik_position_v2(const oa_model *model, const double target_body_m[3],
                                  const oa_ik_options *options, uint32_t output_version,
                                  uint32_t output_size, oa_ik_diagnostics *out);

#ifdef __cplusplus
}
#endif
#endif
