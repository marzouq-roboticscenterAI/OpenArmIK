/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_model.h"
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t random_state = UINT32_C(0x31415926);
static int failures;
#define CHECK(condition) do { if (!(condition)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); ++failures; } } while (0)

static double random_unit(void) {
    random_state = random_state * UINT32_C(1664525) + UINT32_C(1013904223);
    return (double)(random_state >> 8) / 16777216.0;
}

static void options_default(oa_ik_options *options, const double seed[OA_DOF]) {
    size_t i;
    memset(options, 0, sizeof(*options));
    options->abi_version = OA_MODEL_ABI_VERSION;
    options->struct_size = OA_IK_OPTIONS_REQUIRED_SIZE;
    options->position_tolerance_m = 1e-5;
    options->max_joint_step_rad = 0.15;
    options->damping_min = 1e-4;
    options->damping_max = 0.3;
    options->max_iterations = 300;
    for (i = 0; i < OA_DOF; ++i) {
        options->seed[i] = seed[i];
        options->posture[i] = seed[i];
        options->posture_weight[i] = 1.0;
    }
}

static oa_model_status solve(const oa_model *model, const double target[3], const oa_ik_options *options,
                       oa_ik_diagnostics *diagnostics) {
    return oa_ik_position_v2(model, target, options, OA_IK_DIAGNOSTICS_VERSION,
                             OA_IK_DIAGNOSTICS_SIZE, diagnostics);
}

static void pose_position(const oa_model *model, const double q[OA_DOF], double target[3]) {
    oa_fk_result fk;
    CHECK(oa_fk(model, q, &fk) == OA_MODEL_OK);
    target[0] = fk.hand_tcp.m[3]; target[1] = fk.hand_tcp.m[7]; target[2] = fk.hand_tcp.m[11];
}

static void check_diagnostics(const oa_ik_diagnostics *diagnostics) {
    size_t i;
    CHECK(diagnostics->abi_version == OA_MODEL_ABI_VERSION);
    CHECK(diagnostics->struct_size == OA_IK_DIAGNOSTICS_SIZE);
    CHECK(diagnostics->collision_checked == UINT32_C(0));
    CHECK(isfinite(diagnostics->position_error_m));
    CHECK(isfinite(diagnostics->min_singular_value));
    for (i = 0; i < OA_DOF; ++i) CHECK(isfinite(diagnostics->q[i]));
    for (i = 0; i < 16; ++i) CHECK(isfinite(diagnostics->achieved_hand_tcp.m[i]));
}

static void test_unit_aware_ik(const oa_model *model) {
    static const oa_length_unit units[] = {
        OA_LENGTH_UNIT_METRES,
        OA_LENGTH_UNIT_CENTIMETRES,
        OA_LENGTH_UNIT_INCHES
    };
    double seed[OA_DOF] = {0.0};
    double target_m[3];
    oa_ik_options options;
    size_t index;

    pose_position(model, seed, target_m);
    options_default(&options, seed);
    for (index = 0; index < 3; ++index) {
        const oa_vec3d metres = {target_m[0], target_m[1], target_m[2]};
        oa_vec3d input;
        oa_vec3d converted_m;
        double direct_target_m[3];
        oa_ik_diagnostics direct;
        oa_ik_diagnostics unit_aware;
        oa_model_status direct_status;
        oa_model_status unit_status;

        CHECK(oa_vec3d_convert(&metres, OA_LENGTH_UNIT_METRES, units[index],
                               &input) == OA_UNITS_OK);
        CHECK(oa_vec3d_convert(&input, units[index], OA_LENGTH_UNIT_METRES,
                               &converted_m) == OA_UNITS_OK);
        direct_target_m[0] = converted_m.x;
        direct_target_m[1] = converted_m.y;
        direct_target_m[2] = converted_m.z;
        direct_status = solve(model, direct_target_m, &options, &direct);
        unit_status = oa_ik_position_with_units(
            model, &input, units[index], &options,
            OA_IK_DIAGNOSTICS_VERSION, OA_IK_DIAGNOSTICS_SIZE, &unit_aware);
        CHECK(unit_status == direct_status);
        CHECK(memcmp(&unit_aware, &direct, sizeof(direct)) == 0);
    }

    {
        const oa_vec3d invalid_target = {target_m[0], target_m[1], target_m[2]};
        oa_ik_diagnostics guarded;
        oa_ik_diagnostics expected;
        memset(&guarded, 0xa5, sizeof(guarded));
        expected = guarded;
        CHECK(oa_ik_position_with_units(
                  model, &invalid_target, UINT32_MAX, &options,
                  OA_IK_DIAGNOSTICS_VERSION, OA_IK_DIAGNOSTICS_SIZE,
                  &guarded) == OA_MODEL_EINVAL);
        CHECK(memcmp(&guarded, &expected, sizeof(guarded)) == 0);
    }
}

static void test_metadata(const oa_model *model) {
    double lower, upper;
    size_t i;
    CHECK(oa_model_id(model) != NULL);
    CHECK(strstr(oa_model_provenance(model), "6c7b720f") != NULL);
    CHECK(strlen(oa_model_data_sha256(model)) == 64);
    CHECK(strlen(oa_model_flattened_urdf_sha256(model)) == 64);
    CHECK(strlen(oa_model_source_sha256(model)) == 64);
    CHECK(strstr(oa_model_tip_frame(model), "hand_tcp") != NULL);
    for (i = 0; i < OA_DOF; ++i) {
        CHECK(oa_model_joint_name(model, i) != NULL);
        CHECK(oa_model_limits(model, i, &lower, &upper) == OA_MODEL_OK);
        CHECK(lower < upper);
    }
    CHECK(oa_model_limits(NULL, 0, &lower, &upper) == OA_MODEL_EINVAL);
    CHECK(oa_model_limits(model, OA_DOF, &lower, &upper) == OA_MODEL_EINVAL);
}

static void test_fk_jacobian(const oa_model *model) {
    double q[OA_DOF] = {0.0}, plus[OA_DOF], minus[OA_DOF], lower, upper, positive, negative;
    oa_fk_result fk;
    oa_jacobian jacobian;
    size_t sample, i, row;
    const double h = 1e-7;
    CHECK(oa_fk(model, q, &fk) == OA_MODEL_OK);
    CHECK(oa_geometric_jacobian(model, q, &jacobian) == OA_MODEL_OK);
    for (sample = 0; sample < 80; ++sample) {
        for (i = 0; i < OA_DOF; ++i) {
            oa_model_limits(model, i, &lower, &upper);
            q[i] = lower + (upper - lower) * random_unit();
        }
        CHECK(oa_fk(model, q, &fk) == OA_MODEL_OK);
        CHECK(oa_geometric_jacobian(model, q, &jacobian) == OA_MODEL_OK);
        for (i = 0; i < OA_DOF; ++i) {
            memcpy(plus, q, sizeof(q)); memcpy(minus, q, sizeof(q)); plus[i] += h; minus[i] -= h;
            CHECK(oa_fk(model, plus, &fk) == OA_MODEL_OK); positive = fk.hand_tcp.m[3];
            CHECK(oa_fk(model, minus, &fk) == OA_MODEL_OK); negative = fk.hand_tcp.m[3];
            CHECK(fabs((positive - negative) / (2.0*h) - jacobian.value[0][i]) < 3e-8);
            for (row = 3; row < 6; ++row) CHECK(isfinite(jacobian.value[row][i]));
        }
    }
    q[0] = NAN;
    CHECK(oa_fk(model, q, &fk) == OA_MODEL_ENONFINITE);
    CHECK(oa_geometric_jacobian(model, q, &jacobian) == OA_MODEL_ENONFINITE);
}

static void test_status_and_determinism(const oa_model *model) {
    double seed[OA_DOF] = {0.0}, target[3], lower, upper;
    oa_ik_options options;
    oa_ik_diagnostics first, repeated, diagnostics;
    oa_model_status status;
    size_t i;

    pose_position(model, seed, target);
    options_default(&options, seed);
    CHECK(solve(model, target, &options, &diagnostics) == OA_MODEL_OK);
    CHECK(diagnostics.status == OA_MODEL_OK);
    CHECK(diagnostics.position_error_m <= options.position_tolerance_m);
    CHECK(diagnostics.min_singular_value >= 0.0);
    check_diagnostics(&diagnostics);

    target[0] = 10.0; target[1] = 10.0; target[2] = 10.0;
    options.max_iterations = 7;
    status = solve(model, target, &options, &first);
    CHECK(status != OA_MODEL_OK);
    for (i = 0; i < 20; ++i) {
        CHECK(solve(model, target, &options, &repeated) == status);
        CHECK(memcmp(&first, &repeated, sizeof(first)) == 0);
    }

    options_default(&options, seed); target[0] = NAN;
    CHECK(solve(model, target, &options, &diagnostics) == OA_MODEL_ENONFINITE);
    CHECK(diagnostics.status == OA_MODEL_ENONFINITE); check_diagnostics(&diagnostics);
    target[0] = DBL_MAX;
    CHECK(solve(model, target, &options, &diagnostics) == OA_MODEL_EINVAL);
    CHECK(diagnostics.status == OA_MODEL_EINVAL); check_diagnostics(&diagnostics);
    target[0] = 0.0; options.max_iterations = 0;
    CHECK(solve(model, target, &options, &diagnostics) == OA_MODEL_EINVAL);
    CHECK(diagnostics.status == OA_MODEL_EINVAL); check_diagnostics(&diagnostics);
    options_default(&options, seed); options.posture_weight[0] = DBL_MIN;
    CHECK(solve(model, target, &options, &diagnostics) == OA_MODEL_EINVAL);
    check_diagnostics(&diagnostics);
    options_default(&options, seed); oa_model_limits(model, 3, &lower, &upper); options.limit_margin_rad = upper - lower;
    CHECK(solve(model, target, &options, &diagnostics) == OA_MODEL_EBOUNDS);
    CHECK(diagnostics.status == OA_MODEL_EBOUNDS); check_diagnostics(&diagnostics);
    CHECK(solve(NULL, target, &options, &diagnostics) == OA_MODEL_EINVAL);
    CHECK(solve(model, target, NULL, &diagnostics) == OA_MODEL_EINVAL);
    CHECK(solve(model, target, &options, NULL) == OA_MODEL_EINVAL);

    {
        unsigned char guarded[sizeof(oa_ik_diagnostics) + 16];
        unsigned char expected[sizeof(guarded)];
        memset(guarded, 0xa5, sizeof(guarded)); memcpy(expected, guarded, sizeof(guarded));
        CHECK(oa_ik_position_v2(model, target, &options, 1, OA_IK_DIAGNOSTICS_SIZE,
                                (oa_ik_diagnostics *)guarded) == OA_MODEL_EINVAL);
        CHECK(memcmp(guarded, expected, sizeof(guarded)) == 0);
        CHECK(oa_ik_position_v2(model, target, &options, OA_IK_DIAGNOSTICS_VERSION, 248,
                                (oa_ik_diagnostics *)guarded) == OA_MODEL_EINVAL);
        CHECK(memcmp(guarded, expected, sizeof(guarded)) == 0);
    }
}

static void test_randomized_bounds(const oa_model *model) {
    double source[OA_DOF], seed[OA_DOF], target[3], lower, upper;
    oa_ik_options options;
    oa_ik_diagnostics diagnostics;
    size_t sample, i;
    for (sample = 0; sample < 1600; ++sample) {
        for (i = 0; i < OA_DOF; ++i) {
            oa_model_limits(model, i, &lower, &upper);
            source[i] = lower + (upper - lower) * random_unit();
            seed[i] = lower + (upper - lower) * random_unit();
        }
        pose_position(model, source, target);
        options_default(&options, seed);
        options.limit_margin_rad = 1e-7;
        options.max_iterations = 12;
        (void)solve(model, target, &options, &diagnostics);
        check_diagnostics(&diagnostics);
        for (i = 0; i < OA_DOF; ++i) {
            oa_model_limits(model, i, &lower, &upper);
            CHECK(diagnostics.q[i] >= lower + options.limit_margin_rad);
            CHECK(diagnostics.q[i] <= upper - options.limit_margin_rad);
        }
        if (diagnostics.status == OA_MODEL_OK) CHECK(diagnostics.position_error_m <= options.position_tolerance_m);
    }
}

static void test_every_status(void) {
    const oa_model *model = oa_model_left_v10_bimanual();
    double zero[OA_DOF] = {0.0}, target[3];
    const double no_convergence_seed[OA_DOF] = {-2.3351194144138097,-1.5559889452125173,-1.4120086108555794,1.8931030000010134,-1.518981090880394,-0.39199043284988405,0.28491159285116208};
    const double no_convergence_posture[OA_DOF] = {-1.6860622698258161,-0.8556190834751467,-0.40992014035272595,1.3590249664780498,0.43745407859802254,-0.12192869596838951,1.058507942577362};
    const double bounds_seed[OA_DOF] = {-1.8932441807999609,-1.4609833918140274,-1.2423921659874917,0.77917249844849112,0.65108351654195795,-0.18957560553503039,0.78824013324642195};
    const double bounds_posture[OA_DOF] = {-0.1548328306578397,-2.5623457771073919,-1.3491721494398119,0.31719799238651991,0.26039730414676665,-0.37236037016558649,1.353086496085167};
    oa_ik_options options;
    oa_ik_diagnostics diagnostics;
    size_t i;

    pose_position(model, zero, target);
    options_default(&options, zero);
    CHECK(solve(model, target, &options, &diagnostics) == OA_MODEL_OK);
    target[0] = NAN;
    CHECK(solve(model, target, &options, &diagnostics) == OA_MODEL_ENONFINITE);
    target[0] = 0.0; options.max_iterations = 0;
    CHECK(solve(model, target, &options, &diagnostics) == OA_MODEL_EINVAL);
    options_default(&options, zero); options.limit_margin_rad = 2.0;
    CHECK(solve(model, target, &options, &diagnostics) == OA_MODEL_EBOUNDS);

    pose_position(model, zero, target); target[0] += 0.01;
    options_default(&options, zero); options.damping_min = 0.0; options.damping_max = 0.0;
    CHECK(solve(model, target, &options, &diagnostics) == OA_MODEL_ESINGULAR);
    options_default(&options, zero); options.max_iterations = 1;
    CHECK(solve(model, target, &options, &diagnostics) == OA_MODEL_EBUDGET);

    target[0] = -0.52984821796417236; target[1] = 0.96169185638427734; target[2] = 1.7217741012573242;
    options_default(&options, no_convergence_seed); options.position_tolerance_m = 1e-10; options.max_iterations = 100;
    for (i = 0; i < OA_DOF; ++i) options.posture[i] = no_convergence_posture[i];
    CHECK(solve(model, target, &options, &diagnostics) == OA_MODEL_ENOCONVERGENCE);

    target[0] = -0.64545190334320068; target[1] = -0.33169794082641602; target[2] = 1.9664829969406128;
    options_default(&options, bounds_seed); options.position_tolerance_m = 1e-10; options.max_iterations = 100;
    for (i = 0; i < OA_DOF; ++i) options.posture[i] = bounds_posture[i];
    CHECK(solve(model, target, &options, &diagnostics) == OA_MODEL_ESTAGNATED_AT_BOUNDS);
}

static void test_review_bounds_regression(void) {
    const oa_model *model = oa_model_left_v10_bimanual();
    const double seed[OA_DOF] = {-0.0437097440,-1.4722387129,-1.5562786197,0.2972001284,0.3836473821,-0.3155313167,1.5662590524};
    const double target[3] = {0.2708042689,0.3246735719,0.3571402434};
    oa_ik_options options;
    oa_ik_diagnostics diagnostics;
    double lower, upper;
    size_t i;
    options_default(&options, seed);
    (void)solve(model, target, &options, &diagnostics);
    for (i = 0; i < OA_DOF; ++i) {
        oa_model_limits(model, i, &lower, &upper);
        CHECK(diagnostics.q[i] >= lower);
        CHECK(diagnostics.q[i] <= upper);
    }
    if (diagnostics.status == OA_MODEL_OK) CHECK(diagnostics.position_error_m <= options.position_tolerance_m);
}

int main(void) {
    const oa_model *models[2] = {oa_model_left_v10_bimanual(), oa_model_right_v10_bimanual()};
    size_t i;
    for (i = 0; i < 2; ++i) {
        test_metadata(models[i]);
        test_fk_jacobian(models[i]);
        test_status_and_determinism(models[i]);
        test_unit_aware_ik(models[i]);
        test_randomized_bounds(models[i]);
    }
    test_every_status();
    test_review_bounds_regression();
    if (failures) {
        fprintf(stderr, "%d failures\n", failures);
        return 1;
    }
    puts("oa model tests passed (3200 randomized bounded IK cases)");
    return 0;
}
