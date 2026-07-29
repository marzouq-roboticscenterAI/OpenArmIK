/* Compiled against the exact published ABI-v1 declarations. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define OLD_DOF 7u
typedef int32_t old_status;
typedef struct old_model old_model;
typedef struct old_transform { double m[16]; } old_transform;
typedef struct old_options {
    uint32_t abi_version;
    uint32_t struct_size;
    double seed[OLD_DOF];
    double posture[OLD_DOF];
    double posture_weight[OLD_DOF];
    double position_tolerance_m;
    double max_joint_step_rad;
    double damping_min;
    double damping_max;
    double limit_margin_rad;
    uint32_t max_iterations;
} old_options;
typedef struct old_diagnostics {
    old_status status;
    uint32_t iterations;
    uint32_t active_limit_mask;
    double q[OLD_DOF];
    double achieved_position_m[3];
    old_transform achieved_hand_tcp;
    double position_error_m;
    double min_singular_value;
    int collision_checked;
} old_diagnostics;
typedef struct guarded_result { old_diagnostics result; unsigned char canary[16]; } guarded_result;

extern const old_model *oa_model_left_v10_bimanual(void);
extern old_status oa_ik_position(const old_model *, const double[3], const old_options *, old_diagnostics *);

_Static_assert(sizeof(old_diagnostics) == 248, "published ABI-v1 result size changed");

int main(void) {
    old_options options;
    guarded_result guarded, expected;
    double target[3] = {0.0, 0.0, 0.0};
    memset(&options, 0, sizeof(options));
    options.abi_version = 1;
    options.struct_size = (uint32_t)sizeof(options);
    memset(&guarded, 0xa5, sizeof(guarded));
    expected = guarded;
    if (oa_ik_position(oa_model_left_v10_bimanual(), target, &options, &guarded.result) != 1) return 1;
    if (memcmp(&guarded, &expected, sizeof(guarded)) != 0) {
        fputs("ABI-v1 result or canary was overwritten\n", stderr);
        return 2;
    }
    return 0;
}
