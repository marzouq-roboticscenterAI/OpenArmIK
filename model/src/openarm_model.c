/* SPDX-License-Identifier: Apache-2.0 */
#include "oa_model_internal.h"

#include <math.h>
#include <string.h>

#include "generated/oa_model_data.inc"

#define OA_NUMERIC_POSITION_MAX 100.0
#define OA_NUMERIC_POSTURE_MAX 100.0
#define OA_WEIGHT_MIN 1e-9
#define OA_WEIGHT_MAX 1e9
#define OA_ALPHA_MIN 1e-12

static int finite_n(const double *values, size_t count) {
    size_t i;
    for (i = 0; i < count; ++i) {
        if (!isfinite(values[i])) return 0;
    }
    return 1;
}

static void identity(oa_transform *transform) {
    memset(transform, 0, sizeof(*transform));
    transform->m[0] = 1.0;
    transform->m[5] = 1.0;
    transform->m[10] = 1.0;
    transform->m[15] = 1.0;
}

static void multiply(oa_transform *result, const oa_transform *left, const oa_transform *right) {
    oa_transform product;
    size_t row, column, k;
    for (row = 0; row < 4; ++row) {
        for (column = 0; column < 4; ++column) {
            product.m[row * 4 + column] = 0.0;
            for (k = 0; k < 4; ++k) {
                product.m[row * 4 + column] += left->m[row * 4 + k] * right->m[k * 4 + column];
            }
        }
    }
    *result = product;
}

static void apply_direction(const oa_transform *transform, const double vector[3], double result[3]) {
    size_t row;
    for (row = 0; row < 3; ++row) {
        result[row] = transform->m[row * 4] * vector[0]
                    + transform->m[row * 4 + 1] * vector[1]
                    + transform->m[row * 4 + 2] * vector[2];
    }
}

static void axis_rotation(oa_transform *transform, const double axis[3], double angle) {
    const double c = cos(angle), s = sin(angle), d = 1.0 - c;
    const double x = axis[0], y = axis[1], z = axis[2];
    identity(transform);
    transform->m[0] = c + x*x*d;
    transform->m[1] = x*y*d - z*s;
    transform->m[2] = x*z*d + y*s;
    transform->m[4] = y*x*d + z*s;
    transform->m[5] = c + y*y*d;
    transform->m[6] = y*z*d - x*s;
    transform->m[8] = z*x*d - y*s;
    transform->m[9] = z*y*d + x*s;
    transform->m[10] = c + z*z*d;
}

static void position(const oa_transform *transform, double result[3]) {
    result[0] = transform->m[3];
    result[1] = transform->m[7];
    result[2] = transform->m[11];
}

static double norm_squared3(const double vector[3]) {
    return vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2];
}

static void diagnostics_init(oa_ik_diagnostics *out, oa_model_status status) {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->abi_version = OA_MODEL_ABI_VERSION;
    out->struct_size = OA_IK_DIAGNOSTICS_SIZE;
    out->status = status;
    identity(&out->achieved_hand_tcp);
}

static oa_model_status diagnostics_fail(oa_ik_diagnostics *out, oa_model_status status) {
    if (out) out->status = status;
    return status;
}

const oa_model *oa_model_left_v10_bimanual(void) { return &oa_left; }
const oa_model *oa_model_right_v10_bimanual(void) { return &oa_right; }
const char *oa_model_id(const oa_model *model) { return model ? model->id : NULL; }
const char *oa_model_provenance(const oa_model *model) { return model ? model->provenance : NULL; }
const char *oa_model_data_sha256(const oa_model *model) { return model ? model->data_sha256 : NULL; }
const char *oa_model_flattened_urdf_sha256(const oa_model *model) { return model ? model->flattened_urdf_sha256 : NULL; }
const char *oa_model_source_sha256(const oa_model *model) { return model ? model->source_sha256 : NULL; }
const char *oa_model_joint_name(const oa_model *model, size_t index) {
    return (model && index < OA_DOF) ? model->joint_name[index] : NULL;
}
const char *oa_model_tip_frame(const oa_model *model) { return model ? model->tip_frame : NULL; }

oa_model_status oa_model_limits(const oa_model *model, size_t index, double *lower, double *upper) {
    if (!model || !lower || !upper || index >= OA_DOF) return OA_MODEL_EINVAL;
    *lower = model->lower[index];
    *upper = model->upper[index];
    return OA_MODEL_OK;
}

oa_model_status oa_fk(const oa_model *model, const double q[OA_DOF], oa_fk_result *out) {
    oa_transform current, rotation;
    size_t i;
    if (!model || !q || !out) return OA_MODEL_EINVAL;
    if (!finite_n(q, OA_DOF)) return OA_MODEL_ENONFINITE;
    current = model->base_in_body;
    out->base_in_body = current;
    for (i = 0; i < OA_DOF; ++i) {
        multiply(&out->joint_pre[i], &current, &model->origin[i]);
        apply_direction(&out->joint_pre[i], model->axis[i], out->joint_axis_body[i]);
        axis_rotation(&rotation, model->axis[i], q[i]);
        multiply(&current, &out->joint_pre[i], &rotation);
        out->link_post[i] = current;
    }
    multiply(&out->hand_tcp, &current, &model->tcp_in_link7);
    return finite_n(out->hand_tcp.m, 16) ? OA_MODEL_OK : OA_MODEL_ENONFINITE;
}

oa_model_status oa_geometric_jacobian(const oa_model *model, const double q[OA_DOF], oa_jacobian *out) {
    oa_fk_result fk;
    double tip[3], joint[3], delta[3];
    size_t i;
    oa_model_status status;
    if (!out) return OA_MODEL_EINVAL;
    status = oa_fk(model, q, &fk);
    if (status != OA_MODEL_OK) return status;
    position(&fk.hand_tcp, tip);
    for (i = 0; i < OA_DOF; ++i) {
        position(&fk.joint_pre[i], joint);
        delta[0] = tip[0] - joint[0];
        delta[1] = tip[1] - joint[1];
        delta[2] = tip[2] - joint[2];
        out->value[0][i] = fk.joint_axis_body[i][1]*delta[2] - fk.joint_axis_body[i][2]*delta[1];
        out->value[1][i] = fk.joint_axis_body[i][2]*delta[0] - fk.joint_axis_body[i][0]*delta[2];
        out->value[2][i] = fk.joint_axis_body[i][0]*delta[1] - fk.joint_axis_body[i][1]*delta[0];
        out->value[3][i] = fk.joint_axis_body[i][0];
        out->value[4][i] = fk.joint_axis_body[i][1];
        out->value[5][i] = fk.joint_axis_body[i][2];
    }
    return finite_n(&out->value[0][0], 6 * OA_DOF) ? OA_MODEL_OK : OA_MODEL_ENONFINITE;
}

static int solve3(double input[3][3], const double rhs[3], double result[3]) {
    double augmented[3][4], temporary, pivot;
    size_t row, column, k, selected;
    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) augmented[row][column] = input[row][column];
        augmented[row][3] = rhs[row];
    }
    for (k = 0; k < 3; ++k) {
        selected = k;
        for (row = k + 1; row < 3; ++row) {
            if (fabs(augmented[row][k]) > fabs(augmented[selected][k])) selected = row;
        }
        if (fabs(augmented[selected][k]) < 1e-15) return 0;
        if (selected != k) {
            for (column = k; column < 4; ++column) {
                temporary = augmented[k][column];
                augmented[k][column] = augmented[selected][column];
                augmented[selected][column] = temporary;
            }
        }
        pivot = augmented[k][k];
        for (column = k; column < 4; ++column) augmented[k][column] /= pivot;
        for (row = 0; row < 3; ++row) {
            if (row != k) {
                temporary = augmented[row][k];
                for (column = k; column < 4; ++column) augmented[row][column] -= temporary * augmented[k][column];
            }
        }
    }
    for (row = 0; row < 3; ++row) result[row] = augmented[row][3];
    return finite_n(result, 3);
}

static double min_eigenvalue3(double input[3][3]) {
    double matrix[3][3], maximum, angle, c, s, app, aqq, apq, akp, akq;
    size_t iteration, p, q, k;
    memcpy(matrix, input, sizeof(matrix));
    for (iteration = 0; iteration < 24; ++iteration) {
        p = 0; q = 1; maximum = fabs(matrix[0][1]);
        if (fabs(matrix[0][2]) > maximum) { p = 0; q = 2; maximum = fabs(matrix[0][2]); }
        if (fabs(matrix[1][2]) > maximum) { p = 1; q = 2; maximum = fabs(matrix[1][2]); }
        if (maximum < 1e-15) break;
        angle = 0.5 * atan2(2.0 * matrix[p][q], matrix[q][q] - matrix[p][p]);
        c = cos(angle); s = sin(angle); app = matrix[p][p]; aqq = matrix[q][q]; apq = matrix[p][q];
        matrix[p][p] = c*c*app - 2.0*c*s*apq + s*s*aqq;
        matrix[q][q] = s*s*app + 2.0*c*s*apq + c*c*aqq;
        matrix[p][q] = 0.0; matrix[q][p] = 0.0;
        for (k = 0; k < 3; ++k) {
            if (k != p && k != q) {
                akp = matrix[k][p]; akq = matrix[k][q];
                matrix[k][p] = c*akp - s*akq; matrix[p][k] = matrix[k][p];
                matrix[k][q] = s*akp + c*akq; matrix[q][k] = matrix[k][q];
            }
        }
    }
    return fmax(0.0, fmin(matrix[0][0], fmin(matrix[1][1], matrix[2][2])));
}

static void translational_matrix(const oa_jacobian *jacobian, double matrix[3][OA_DOF]) {
    size_t row, i;
    for (row = 0; row < 3; ++row) {
        for (i = 0; i < OA_DOF; ++i) matrix[row][i] = jacobian->value[row][i];
    }
}

static double singular_value(double jacobian[3][OA_DOF], const double weights[OA_DOF], uint32_t active) {
    double gram[3][3] = {{0.0}};
    size_t row, column, i;
    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) {
            for (i = 0; i < OA_DOF; ++i) {
                if (!(active & (UINT32_C(1) << i))) gram[row][column] += jacobian[row][i] * jacobian[column][i] / weights[i];
            }
        }
    }
    return sqrt(min_eigenvalue3(gram));
}

static oa_model_status compute_step(double jacobian[3][OA_DOF], const double error[3], const double q[OA_DOF],
                              const oa_ik_options *options, uint32_t active, double step[OA_DOF], double *sigma) {
    double system[3][3] = {{0.0}}, inverse[OA_DOF][3] = {{0.0}};
    double rhs[3], solution[3], lambda, posture_gain, task, secondary;
    size_t row, column, i, k;
    *sigma = singular_value(jacobian, options->posture_weight, active);
    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) {
            for (i = 0; i < OA_DOF; ++i) {
                if (!(active & (UINT32_C(1) << i))) system[row][column] += jacobian[row][i] * jacobian[column][i] / options->posture_weight[i];
            }
        }
    }
    lambda = options->damping_min;
    if (*sigma < 0.03) lambda += (options->damping_max - options->damping_min) * (0.03 - *sigma) / 0.03;
    if (lambda > options->damping_max) lambda = options->damping_max;
    for (row = 0; row < 3; ++row) system[row][row] += lambda * lambda;
    for (i = 0; i < OA_DOF; ++i) {
        step[i] = 0.0;
        if (!(active & (UINT32_C(1) << i))) {
            for (row = 0; row < 3; ++row) rhs[row] = jacobian[row][i] / options->posture_weight[i];
            if (!solve3(system, rhs, solution)) return OA_MODEL_ESINGULAR;
            for (row = 0; row < 3; ++row) inverse[i][row] = solution[row];
        }
    }
    posture_gain = 0.1 * sqrt(norm_squared3(error));
    for (i = 0; i < OA_DOF; ++i) {
        if (!(active & (UINT32_C(1) << i))) {
            task = 0.0;
            for (row = 0; row < 3; ++row) task += inverse[i][row] * error[row];
            secondary = posture_gain * (options->posture[i] - q[i]);
            for (k = 0; k < OA_DOF; ++k) {
                if (!(active & (UINT32_C(1) << k))) {
                    for (row = 0; row < 3; ++row) {
                        secondary -= inverse[i][row] * jacobian[row][k] * posture_gain * (options->posture[k] - q[k]);
                    }
                }
            }
            step[i] = task + secondary;
        }
    }
    return finite_n(step, OA_DOF) ? OA_MODEL_OK : OA_MODEL_ENONFINITE;
}

static int feasible(const oa_model *model, const oa_ik_options *options, const double q[OA_DOF]) {
    size_t i;
    for (i = 0; i < OA_DOF; ++i) {
        const double lower = model->lower[i] + options->limit_margin_rad;
        const double upper = model->upper[i] - options->limit_margin_rad;
        if (!isfinite(q[i]) || q[i] < lower || q[i] > upper) return 0;
    }
    return 1;
}

oa_model_status oa_ik_position(const oa_model *model, const double target[3], const oa_ik_options *options,
                         void *out) {
    (void)model;
    (void)target;
    (void)options;
    (void)out;
    return OA_MODEL_EINVAL;
}

oa_model_status oa_ik_position_v2(const oa_model *model, const double target[3], const oa_ik_options *options,
                            uint32_t output_version, uint32_t output_size, oa_ik_diagnostics *out) {
    double q[OA_DOF] = {0.0}, error[3], tip[3], jacobian[3][OA_DOF], step[OA_DOF], candidate[OA_DOF];
    double old_error, candidate_error, sigma = 0.0, alpha, maximum_step, available, ratio, lower, upper;
    oa_fk_result fk;
    oa_jacobian geometric;
    oa_model_status status = OA_MODEL_ENOCONVERGENCE;
    uint32_t iteration, active = 0, next_active, pass;
    size_t row, i;
    int accepted;

    if (!out || output_version != OA_IK_DIAGNOSTICS_VERSION || output_size < OA_IK_DIAGNOSTICS_SIZE) return OA_MODEL_EINVAL;
    diagnostics_init(out, OA_MODEL_EINVAL);
    if (!model || !target || !options) return diagnostics_fail(out, OA_MODEL_EINVAL);
    if (options->abi_version != OA_MODEL_ABI_VERSION || options->struct_size < OA_IK_OPTIONS_REQUIRED_SIZE) return diagnostics_fail(out, OA_MODEL_EINVAL);
    if (!finite_n(target, 3) || !finite_n(options->seed, OA_DOF) || !finite_n(options->posture, OA_DOF)
        || !finite_n(options->posture_weight, OA_DOF) || !isfinite(options->position_tolerance_m)
        || !isfinite(options->max_joint_step_rad) || !isfinite(options->damping_min)
        || !isfinite(options->damping_max) || !isfinite(options->limit_margin_rad)) return diagnostics_fail(out, OA_MODEL_ENONFINITE);
    for (row = 0; row < 3; ++row) if (fabs(target[row]) > OA_NUMERIC_POSITION_MAX) return diagnostics_fail(out, OA_MODEL_EINVAL);
    if (options->max_iterations == 0 || options->position_tolerance_m < 1e-12 || options->position_tolerance_m > 1.0
        || options->max_joint_step_rad < 1e-12 || options->max_joint_step_rad > 3.141592653589793
        || options->damping_min < 0.0 || options->damping_max < options->damping_min || options->damping_max > 1.0
        || options->limit_margin_rad < 0.0) return diagnostics_fail(out, OA_MODEL_EINVAL);
    for (i = 0; i < OA_DOF; ++i) {
        if (fabs(options->seed[i]) > OA_NUMERIC_POSTURE_MAX || fabs(options->posture[i]) > OA_NUMERIC_POSTURE_MAX
            || options->posture_weight[i] < OA_WEIGHT_MIN || options->posture_weight[i] > OA_WEIGHT_MAX) return diagnostics_fail(out, OA_MODEL_EINVAL);
        lower = model->lower[i] + options->limit_margin_rad;
        upper = model->upper[i] - options->limit_margin_rad;
        if (lower > upper) return diagnostics_fail(out, OA_MODEL_EBOUNDS);
        q[i] = fmin(upper, fmax(lower, options->seed[i]));
    }

    for (iteration = 0; iteration < options->max_iterations; ++iteration) {
        status = oa_fk(model, q, &fk);
        if (status != OA_MODEL_OK) break;
        position(&fk.hand_tcp, tip);
        for (row = 0; row < 3; ++row) error[row] = target[row] - tip[row];
        old_error = norm_squared3(error);
        status = oa_geometric_jacobian(model, q, &geometric);
        if (status != OA_MODEL_OK) break;
        translational_matrix(&geometric, jacobian);
        sigma = singular_value(jacobian, options->posture_weight, 0);
        if (old_error <= options->position_tolerance_m * options->position_tolerance_m) {
            status = OA_MODEL_OK;
            break;
        }

        active = 0;
        for (pass = 0; pass <= OA_DOF; ++pass) {
            status = compute_step(jacobian, error, q, options, active, step, &sigma);
            if (status != OA_MODEL_OK) break;
            next_active = active;
            for (i = 0; i < OA_DOF; ++i) {
                lower = model->lower[i] + options->limit_margin_rad;
                upper = model->upper[i] - options->limit_margin_rad;
                if ((q[i] <= lower && step[i] < 0.0) || (q[i] >= upper && step[i] > 0.0)) next_active |= UINT32_C(1) << i;
            }
            if (next_active == active) break;
            active = next_active;
        }
        if (status != OA_MODEL_OK) break;
        if (active == UINT32_C(0x7f)) { status = OA_MODEL_ESTAGNATED_AT_BOUNDS; break; }

        maximum_step = 0.0;
        for (i = 0; i < OA_DOF; ++i) maximum_step = fmax(maximum_step, fabs(step[i]));
        if (maximum_step < 1e-15) { status = active ? OA_MODEL_ESTAGNATED_AT_BOUNDS : OA_MODEL_ENOCONVERGENCE; break; }
        alpha = fmin(1.0, options->max_joint_step_rad / maximum_step);
        for (i = 0; i < OA_DOF; ++i) {
            lower = model->lower[i] + options->limit_margin_rad;
            upper = model->upper[i] - options->limit_margin_rad;
            if (step[i] > 0.0) available = upper - q[i];
            else if (step[i] < 0.0) available = q[i] - lower;
            else continue;
            if (available < 0.0) { status = OA_MODEL_EBOUNDS; break; }
            ratio = available / fabs(step[i]);
            alpha = fmin(alpha, ratio);
        }
        if (status != OA_MODEL_OK) break;
        if (alpha < OA_ALPHA_MIN) { status = active ? OA_MODEL_ESTAGNATED_AT_BOUNDS : OA_MODEL_ENOCONVERGENCE; break; }

        accepted = 0;
        while (alpha >= OA_ALPHA_MIN) {
            for (i = 0; i < OA_DOF; ++i) {
                lower = model->lower[i] + options->limit_margin_rad;
                upper = model->upper[i] - options->limit_margin_rad;
                candidate[i] = fmin(upper, fmax(lower, q[i] + alpha * step[i]));
            }
            status = oa_fk(model, candidate, &fk);
            if (status != OA_MODEL_OK) break;
            position(&fk.hand_tcp, tip);
            for (row = 0; row < 3; ++row) error[row] = target[row] - tip[row];
            candidate_error = norm_squared3(error);
            if (isfinite(candidate_error) && candidate_error < old_error) {
                memcpy(q, candidate, sizeof(q));
                accepted = 1;
                break;
            }
            alpha *= 0.5;
        }
        if (status != OA_MODEL_OK) break;
        if (!accepted) { status = active ? OA_MODEL_ESTAGNATED_AT_BOUNDS : OA_MODEL_ENOCONVERGENCE; break; }
    }

    if (iteration == options->max_iterations) status = OA_MODEL_EBUDGET;
    if (oa_fk(model, q, &fk) == OA_MODEL_OK) {
        out->achieved_hand_tcp = fk.hand_tcp;
        position(&fk.hand_tcp, out->achieved_position_m);
        for (row = 0; row < 3; ++row) error[row] = target[row] - out->achieved_position_m[row];
        out->position_error_m = sqrt(norm_squared3(error));
    } else {
        status = OA_MODEL_ENONFINITE;
    }
    memcpy(out->q, q, sizeof(q));
    out->iterations = iteration;
    out->active_limit_mask = active;
    out->min_singular_value = isfinite(sigma) ? sigma : 0.0;
    if (out->position_error_m <= options->position_tolerance_m && feasible(model, options, q)) status = OA_MODEL_OK;
    else if (status == OA_MODEL_OK) status = OA_MODEL_EBOUNDS;
    out->status = status;
    return status;
}

oa_model_status oa_ik_position_with_units(const oa_model *model,
                                          const oa_vec3d *target_body,
                                          oa_length_unit target_unit,
                                          const oa_ik_options *options,
                                          uint32_t output_version,
                                          uint32_t output_size,
                                          oa_ik_diagnostics *out) {
    oa_vec3d target_m;
    double target_array_m[3];
    const oa_units_status conversion =
        oa_vec3d_convert(target_body, target_unit, OA_LENGTH_UNIT_METRES,
                         &target_m);
    if (conversion == OA_UNITS_ENONFINITE || conversion == OA_UNITS_EOVERFLOW) {
        return OA_MODEL_ENONFINITE;
    }
    if (conversion != OA_UNITS_OK) {
        return OA_MODEL_EINVAL;
    }
    target_array_m[0] = target_m.x;
    target_array_m[1] = target_m.y;
    target_array_m[2] = target_m.z;
    return oa_ik_position_v2(model, target_array_m, options, output_version,
                             output_size, out);
}
