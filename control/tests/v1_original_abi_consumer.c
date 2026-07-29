/* Compiled only against the frozen 8bc839e V1 header subset. */
#include "openarm_control.h"

#include <stdio.h>
#include <string.h>

#define INIT(record) do { (record).struct_size = sizeof(record); \
                          (record).abi_version = OA_CONTROL_ABI_V1; } while (0)

static void build_manifest(oa_manifest_config *config) {
    static const double lower[2][7] = {
        {-3.490659, -3.3161253267948965, -1.570796, 0.0,
         -1.570796, -0.785398, -1.570796},
        {-1.396263, -0.17453267320510335, -1.570796, 0.0,
         -1.570796, -0.785398, -1.570796}};
    static const double upper[2][7] = {
        {1.396263, 0.17453267320510335, 1.570796, 2.443461,
         1.570796, 0.785398, 1.570796},
        {3.490659, 3.3161253267948965, 1.570796, 2.443461,
         1.570796, 0.785398, 1.570796}};
    uint32_t side;
    memset(config, 0, sizeof(*config));
    INIT(*config);
    config->manifest_revision = 41U;
    config->model_revision = 7U;
    for (side = 0U; side < 2U; ++side) {
        uint32_t joint;
        INIT(config->arm[side]);
        (void)snprintf(config->arm[side].bus_name,
                       sizeof(config->arm[side].bus_name), "vcan%u", side);
        for (joint = 0U; joint < 7U; ++joint) {
            oa_motor_config *motor = &config->arm[side].motor[joint];
            INIT(*motor);
            motor->motor_type = joint < 2U ? OA_MOTOR_DM8009 :
                                (joint < 4U ? OA_MOTOR_DM4340 : OA_MOTOR_DM4310);
            motor->joint_index = joint;
            motor->send_id = joint + 1U;
            motor->receive_id = joint + 0x11U;
            motor->embedded_motor_id = joint + 1U;
            motor->control_mode = 1U;
            motor->bitrate = 1000000U;
            motor->timeout_ticks = 1000U;
            motor->hardware_version = 1U;
            motor->software_version = 1U;
            motor->firmware_subversion = 1U;
            motor->q_scale = ((side + joint) & 1U) == 0U ? 1.0 : -1.0;
            motor->q_offset_rad = 0.125;
            motor->lower_rad = lower[side][joint];
            motor->upper_rad = upper[side][joint];
            motor->max_velocity_rad_s = 1.0;
            motor->max_acceleration_rad_s2 = 2.0;
            motor->max_jerk_rad_s3 = 10.0;
            motor->pmax_rad = 12.5;
            if (motor->motor_type == OA_MOTOR_DM8009) {
                motor->vmax_rad_s = 45.0; motor->tmax_nm = 54.0; motor->gear_ratio = 9.0;
            } else if (motor->motor_type == OA_MOTOR_DM4340) {
                motor->vmax_rad_s = 10.0; motor->tmax_nm = 28.0; motor->gear_ratio = 40.0;
            } else {
                motor->vmax_rad_s = 30.0; motor->tmax_nm = 10.0; motor->gear_ratio = 10.0;
            }
            motor->direction = motor->q_scale > 0.0 ? 1 : -1;
            (void)snprintf(motor->serial, sizeof(motor->serial),
                           "OLD-%u-%u", side, joint);
            (void)snprintf(motor->joint_name, sizeof(motor->joint_name),
                           "openarm_%s_joint%u", side == 0U ? "left" : "right",
                           joint + 1U);
        }
    }
}

int main(void) {
    oa_manifest_config config;
    oa_manifest *manifest = NULL;
    oa_controller *controller = NULL;
    oa_motion_plan *plan = NULL;
    oa_controller_options options = {0};
    oa_verify_report verify = {0};
    oa_arm_challenge challenge = {0};
    oa_snapshot snapshot = {0};
    oa_paired_tcp_move move = {0};
    oa_sim_fault fault = {0};
    build_manifest(&config);
    if (oa_manifest_create(&config, &manifest) != OA_OK) return 1;
    INIT(options);
    options.backend = OA_BACKEND_VIRTUAL;
    options.collision_policy = OA_COLLISION_VIRTUAL_UNCHECKED;
    options.cycle_ns = 10000000U;
    options.feedback_timeout_ns = 50000000U;
    options.max_cross_bus_skew_ns = 1000000U;
    if (oa_controller_create(manifest, &options, &controller) != OA_OK) return 2;
    INIT(verify);
    if (oa_controller_open_and_verify(controller, &verify) != OA_OK) return 3;
    INIT(challenge);
    if (oa_controller_get_arm_challenge(controller, &challenge) != OA_OK) return 4;
    if (oa_controller_arm(controller, &challenge) != OA_OK) return 5;
    INIT(snapshot);
    if (oa_controller_snapshot(controller, &snapshot) != OA_OK) return 6;
    INIT(move);
    move.expiry_ns = UINT64_C(60000000000);
    move.required_feedback_seq[0] = snapshot.arm[0].feedback_seq;
    move.required_feedback_seq[1] = snapshot.arm[1].feedback_seq;
    move.left_tcp_m[0] = 0.20; move.left_tcp_m[1] = 0.30; move.left_tcp_m[2] = 0.85;
    move.right_tcp_m[0] = 0.20; move.right_tcp_m[1] = -0.30; move.right_tcp_m[2] = 0.85;
    move.velocity_scale = 0.5; move.acceleration_scale = 0.5; move.jerk_scale = 0.5;
    move.tcp_tol_m = 1.0e-3;
    move.collision_scene_revision = 1U;
    if (oa_controller_plan_paired_tcp(controller, &move, &plan) != OA_OK) return 7;
    INIT(fault);
    fault.side = OA_LEFT;
    fault.fault_mask = 1U;
    if (oa_controller_sim_set_fault(controller, &fault) != OA_OK) return 8;
    oa_motion_plan_destroy(plan);
    oa_controller_destroy(controller);
    oa_manifest_destroy(manifest);
    return 0;
}
