/* Frozen public-record subset from openarmik commit 8bc839e. */
#ifndef OPENARM_CONTROL_V1_ORIGINAL_H
#define OPENARM_CONTROL_V1_ORIGINAL_H
#include <stdint.h>

#define OA_CONTROL_ABI_V1 UINT32_C(1)
#define OA_LEFT UINT32_C(0)
#define OA_RIGHT UINT32_C(1)
#define OA_OK UINT32_C(0)
#define OA_BACKEND_VIRTUAL UINT32_C(1)
#define OA_COLLISION_VIRTUAL_UNCHECKED UINT32_C(1)
#define OA_MOTOR_DM8009 UINT32_C(1)
#define OA_MOTOR_DM4340 UINT32_C(2)
#define OA_MOTOR_DM4310 UINT32_C(3)

typedef struct oa_manifest oa_manifest;
typedef struct oa_controller oa_controller;
typedef struct oa_motion_plan oa_motion_plan;
typedef uint32_t oa_status;
typedef uint32_t oa_side;

typedef struct oa_motor_config {
    uint32_t struct_size, abi_version, motor_type, joint_index;
    uint32_t send_id, receive_id, embedded_motor_id, control_mode;
    uint32_t bitrate, timeout_ticks, hardware_version, software_version;
    uint32_t firmware_subversion;
    double q_scale, q_offset_rad, lower_rad, upper_rad;
    double max_velocity_rad_s, max_acceleration_rad_s2, max_jerk_rad_s3;
    double pmax_rad, vmax_rad_s, tmax_nm, gear_ratio;
    int32_t direction;
    char serial[32];
    char joint_name[48];
} oa_motor_config;

typedef struct oa_arm_config {
    uint32_t struct_size, abi_version;
    char bus_name[16];
    oa_motor_config motor[7];
} oa_arm_config;

typedef struct oa_manifest_config {
    uint32_t struct_size, abi_version;
    uint64_t manifest_revision, model_revision;
    oa_arm_config arm[2];
} oa_manifest_config;

typedef struct oa_arm_snapshot {
    uint32_t struct_size, abi_version;
    uint64_t feedback_seq, t_ns;
    uint32_t expected_mask, fresh_mask, fault_mask;
    double q[7], dq[7], tau[7];
    double raw_q[7], raw_dq[7], raw_tau[7];
    uint8_t status[7], mos_c[7], coil_c[7];
} oa_arm_snapshot;

typedef struct oa_snapshot {
    uint32_t struct_size, abi_version;
    oa_arm_snapshot arm[2];
    uint64_t manifest_revision, model_revision, max_cross_bus_skew_ns;
    uint32_t lifecycle;
} oa_snapshot;

typedef struct oa_controller_options {
    uint32_t struct_size, abi_version, backend, collision_policy;
    uint64_t cycle_ns, feedback_timeout_ns, max_cross_bus_skew_ns;
} oa_controller_options;

typedef struct oa_verify_report {
    uint32_t struct_size, abi_version;
    uint64_t verify_epoch;
    uint32_t verified_mask, failure_mask;
} oa_verify_report;

typedef struct oa_arm_challenge {
    uint32_t struct_size, abi_version;
    uint64_t verify_epoch, nonce, expiry_ns;
} oa_arm_challenge;

typedef struct oa_paired_tcp_move {
    uint32_t struct_size, abi_version;
    uint64_t expiry_ns, required_feedback_seq[2];
    double left_tcp_m[3], right_tcp_m[3];
    double velocity_scale, acceleration_scale, jerk_scale, tcp_tol_m;
    uint64_t collision_scene_revision;
} oa_paired_tcp_move;

typedef struct oa_sim_fault {
    uint32_t struct_size, abi_version;
    oa_side side;
    uint32_t freeze_mask, drop_mask, fault_mask;
} oa_sim_fault;

oa_status oa_manifest_create(const oa_manifest_config *, oa_manifest **);
void oa_manifest_destroy(oa_manifest *);
oa_status oa_controller_create(const oa_manifest *, const oa_controller_options *,
                               oa_controller **);
oa_status oa_controller_open_and_verify(oa_controller *, oa_verify_report *);
oa_status oa_controller_snapshot(oa_controller *, oa_snapshot *);
oa_status oa_controller_get_arm_challenge(oa_controller *, oa_arm_challenge *);
oa_status oa_controller_arm(oa_controller *, const oa_arm_challenge *);
oa_status oa_controller_plan_paired_tcp(oa_controller *, const oa_paired_tcp_move *,
                                        oa_motion_plan **);
oa_status oa_controller_sim_set_fault(oa_controller *, const oa_sim_fault *);
void oa_motion_plan_destroy(oa_motion_plan *);
void oa_controller_destroy(oa_controller *);
#endif
