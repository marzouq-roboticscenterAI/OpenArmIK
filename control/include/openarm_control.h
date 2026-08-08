/* SPDX-License-Identifier: Apache-2.0 */
#ifndef OPENARM_CONTROL_H
#define OPENARM_CONTROL_H

#include <stddef.h>
#include <stdint.h>

#include <openarm_units.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OA_CONTROL_ABI_V1 UINT32_C(1)
#define OA_CONTROL_DOF UINT32_C(7)
#define OA_CONTROL_ARMS UINT32_C(2)

typedef struct oa_manifest oa_manifest;
typedef struct oa_controller oa_controller;
typedef struct oa_motion_plan oa_motion_plan;
typedef uint32_t oa_control_status;
#define OPENARM_CONTROL_STATUS_NAMESPACE_PRESENT 1
typedef uint32_t oa_side;

/* Opaque handles are validated without dereferencing caller memory. Calls on one
 * controller are internally serialized. Every destroy operation may overlap a
 * call already in progress: it removes the handle from its typed registry and
 * synchronizes with pins before freeing its registry slot. Already-pinned
 * immutable work retains shared state safely. Handles are monotonic,
 * never-dereferenced token values and are not reused. Calls begun afterward
 * return OA_CONTROL_EINVAL; token-space exhaustion fails closed with
 * OA_CONTROL_ENOMEM. */

#define OA_LEFT UINT32_C(0)
#define OA_RIGHT UINT32_C(1)

#define OA_CONTROL_OK UINT32_C(0)
#define OA_CONTROL_EINVAL UINT32_C(1)
#define OA_CONTROL_EABI UINT32_C(2)
#define OA_CONTROL_ESTATE UINT32_C(3)
#define OA_CONTROL_ESTALE UINT32_C(4)
#define OA_CONTROL_ETIMEOUT UINT32_C(5)
#define OA_CONTROL_ECAN UINT32_C(6)
#define OA_CONTROL_EFAULT UINT32_C(7)
#define OA_CONTROL_EESTOP UINT32_C(8)
#define OA_CONTROL_ELIMIT UINT32_C(9)
#define OA_CONTROL_EIDENTITY UINT32_C(10)
#define OA_CONTROL_EUNREACHABLE UINT32_C(11)
#define OA_CONTROL_ECOLLISION UINT32_C(12)
#define OA_CONTROL_EBUSY UINT32_C(13)
#define OA_CONTROL_EIO UINT32_C(14)
#define OA_CONTROL_ENOMEM UINT32_C(15)
#define OA_CONTROL_EUNSUPPORTED UINT32_C(16)

#if defined(OPENARM_MODEL_LEGACY_GENERIC_STATUS_ACTIVE)
#error "model and control legacy status names cannot coexist; define OPENARM_DISABLE_LEGACY_GENERIC_STATUS before both headers"
#endif
#if !defined(OPENARM_DISABLE_LEGACY_GENERIC_STATUS)
#if defined(OPENARM_MODEL_STATUS_NAMESPACE_PRESENT)
#error "define OPENARM_DISABLE_LEGACY_GENERIC_STATUS before combining model and control headers"
#endif
#define OPENARM_CONTROL_LEGACY_GENERIC_STATUS_ACTIVE 1
typedef oa_control_status oa_status;
#define OA_OK           OA_CONTROL_OK
#define OA_EINVAL       OA_CONTROL_EINVAL
#define OA_EABI         OA_CONTROL_EABI
#define OA_ESTATE       OA_CONTROL_ESTATE
#define OA_ESTALE       OA_CONTROL_ESTALE
#define OA_ETIMEOUT     OA_CONTROL_ETIMEOUT
#define OA_ECAN         OA_CONTROL_ECAN
#define OA_EFAULT       OA_CONTROL_EFAULT
#define OA_EESTOP       OA_CONTROL_EESTOP
#define OA_ELIMIT       OA_CONTROL_ELIMIT
#define OA_EIDENTITY    OA_CONTROL_EIDENTITY
#define OA_EUNREACHABLE OA_CONTROL_EUNREACHABLE
#define OA_ECOLLISION   OA_CONTROL_ECOLLISION
#define OA_EBUSY        OA_CONTROL_EBUSY
#define OA_EIO          OA_CONTROL_EIO
#define OA_ENOMEM       OA_CONTROL_ENOMEM
#define OA_EUNSUPPORTED OA_CONTROL_EUNSUPPORTED
#endif

#define OA_BACKEND_VIRTUAL UINT32_C(1)
#define OA_BACKEND_PHYSICAL UINT32_C(2)

#define OA_COLLISION_REJECT_ALL UINT32_C(0)
#define OA_COLLISION_VIRTUAL_UNCHECKED UINT32_C(1)

#define OA_LIFECYCLE_CLOSED UINT32_C(0)
#define OA_LIFECYCLE_VERIFYING UINT32_C(1)
#define OA_LIFECYCLE_DISARMED UINT32_C(2)
#define OA_LIFECYCLE_ARMING UINT32_C(3)
#define OA_LIFECYCLE_ARMED_IDLE UINT32_C(4)
#define OA_LIFECYCLE_EXECUTING UINT32_C(5)
#define OA_LIFECYCLE_STOPPING UINT32_C(6)
#define OA_LIFECYCLE_FAULT UINT32_C(7)
#define OA_LIFECYCLE_ESTOP UINT32_C(8)

#define OA_EVENT_VERIFIED UINT32_C(1)
#define OA_EVENT_ARMED UINT32_C(2)
#define OA_EVENT_STARTED UINT32_C(3)
#define OA_EVENT_COMPLETED UINT32_C(4)
#define OA_EVENT_STOPPED UINT32_C(5)
#define OA_EVENT_FAULTED UINT32_C(6)
#define OA_EVENT_DISARMED UINT32_C(7)
#define OA_EVENT_QUEUED UINT32_C(8)
#define OA_EVENT_SETTLING UINT32_C(9)
#define OA_EVENT_ABORTED UINT32_C(10)
#define OA_EVENT_ESTOP UINT32_C(11)

#define OA_PLAN_JOINT UINT32_C(1)
#define OA_PLAN_PAIRED_TCP UINT32_C(2)
/* Both claws translate by the vector that carries their midpoint to a target. */
#define OA_PLAN_CENTROID_TCP UINT32_C(3)
/* One claw is commanded; the other mirrors it across the body sagittal plane. */
#define OA_PLAN_MIRRORED_TCP UINT32_C(4)
/* Both claws advance along the ray toward a shared point until measured joint
 * torque reports contact, the real-time keepout monitor intervenes, or the
 * planned prefix is exhausted. */
#define OA_PLAN_CONVERGE_TCP UINT32_C(5)

#define OA_STOP_DISABLE UINT32_C(1)
#define OA_STOP_CONTROLLED UINT32_C(2)

#define OA_MOTOR_DM8009 UINT32_C(1)
#define OA_MOTOR_DM4340 UINT32_C(2)
#define OA_MOTOR_DM4310 UINT32_C(3)

typedef struct oa_motor_config {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t motor_type;
    uint32_t joint_index;
    uint32_t send_id;
    uint32_t receive_id;
    uint32_t embedded_motor_id;
    uint32_t control_mode;
    uint32_t bitrate;
    uint32_t timeout_ticks;
    uint32_t hardware_version;
    uint32_t software_version;
    uint32_t firmware_subversion;
    double q_scale;
    double q_offset_rad;
    double lower_rad;
    double upper_rad;
    double max_velocity_rad_s;
    double max_acceleration_rad_s2;
    double max_jerk_rad_s3;
    double pmax_rad;
    double vmax_rad_s;
    double tmax_nm;
    double gear_ratio;
    int32_t direction;
    char serial[32];
    char joint_name[48];
} oa_motor_config;

typedef struct oa_arm_config {
    uint32_t struct_size;
    uint32_t abi_version;
    char bus_name[16];
    oa_motor_config motor[7];
} oa_arm_config;

typedef struct oa_manifest_config {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t manifest_revision;
    uint64_t model_revision;
    oa_arm_config arm[2];
} oa_manifest_config;

typedef struct oa_arm_snapshot {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t feedback_seq;
    uint64_t t_ns;
    uint32_t expected_mask;
    uint32_t fresh_mask;
    uint32_t fault_mask;
    double q[7];
    double dq[7];
    double tau[7];
    double raw_q[7];
    double raw_dq[7];
    double raw_tau[7];
    uint8_t status[7];
    uint8_t mos_c[7];
    uint8_t coil_c[7];
} oa_arm_snapshot;

typedef struct oa_snapshot {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_arm_snapshot arm[2];
    uint64_t manifest_revision;
    uint64_t model_revision;
    uint64_t max_cross_bus_skew_ns;
    uint32_t lifecycle;
} oa_snapshot;

typedef struct oa_joint_move {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t expiry_ns;
    uint64_t required_feedback_seq;
    oa_side side;
    uint32_t joint;
    double target_rad;
    double velocity_scale;
    double acceleration_scale;
    double jerk_scale;
    double position_tol_rad;
    double velocity_tol_rad_s;
} oa_joint_move;

typedef struct oa_paired_tcp_move {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t expiry_ns;
    uint64_t required_feedback_seq[2];
    double left_tcp_m[3];
    double right_tcp_m[3];
    double velocity_scale;
    double acceleration_scale;
    double jerk_scale;
    double tcp_tol_m;
    uint64_t collision_scene_revision;
    double max_branch_step_rad;
    double min_singular_value;
} oa_paired_tcp_move;

typedef struct oa_paired_tcp_move_with_units {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_length_unit coordinate_unit;
    uint32_t reserved0;
    uint64_t expiry_ns;
    uint64_t required_feedback_seq[2];
    oa_vec3d left_tcp;
    oa_vec3d right_tcp;
    double velocity_scale;
    double acceleration_scale;
    double jerk_scale;
    double tcp_tol_m;
    uint64_t collision_scene_revision;
    double max_branch_step_rad;
    double min_singular_value;
} oa_paired_tcp_move_with_units;

/* Bimanual translation of the midpoint between the two hand_tcp origins.
 * The measured midpoint is carried to target_centroid_m and the identical body
 * frame delta is applied to both claws, so their relative pose is preserved. */
typedef struct oa_centroid_tcp_move {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t expiry_ns;
    uint64_t required_feedback_seq[2];
    double target_centroid_m[3];
    double velocity_scale;
    double acceleration_scale;
    double jerk_scale;
    double tcp_tol_m;
    uint64_t collision_scene_revision;
    double max_branch_step_rad;
    double min_singular_value;
} oa_centroid_tcp_move;

/* One claw is given a target; the other is commanded to the sagittal mirror of
 * that target (y is negated, x and z are preserved). Both arms move together. */
typedef struct oa_mirrored_tcp_move {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t expiry_ns;
    uint64_t required_feedback_seq[2];
    oa_side lead_side;
    uint32_t reserved0;
    double lead_tcp_m[3];
    double velocity_scale;
    double acceleration_scale;
    double jerk_scale;
    double tcp_tol_m;
    uint64_t collision_scene_revision;
    double max_branch_step_rad;
    double min_singular_value;
} oa_mirrored_tcp_move;

/* Both claws converge on a single body-frame point. Execution halts at the
 * first of: measured contact torque, a real-time keepout violation, or the end
 * of the planned prefix. Halting on contact is a successful outcome, not a
 * fault. */
typedef struct oa_converge_tcp_move {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t expiry_ns;
    uint64_t required_feedback_seq[2];
    double target_m[3];
    /* Per-joint absolute |tau| stop threshold in newton-metres, in model joint
     * coordinates. A non-positive entry falls back to
     * contact_torque_fraction * the motor's protocol tmax. */
    double contact_torque_nm[7];
    /* Fallback threshold as a fraction of each motor's tmax. Non-positive
     * selects the built-in default returned by
     * oa_control_default_contact_torque_fraction(). */
    double contact_torque_fraction;
    /* Contact must persist for this many consecutive cycles before the stop
     * latches. Zero selects the built-in default. Rejects single-cycle noise. */
    uint32_t contact_persistence_cycles;
    uint32_t reserved0;
    /* Stop this far short of target_m along the approach ray. */
    double stop_distance_m;
    /* Reject the request unless at least this much validated travel exists. */
    double minimum_progress_m;
    double velocity_scale;
    double acceleration_scale;
    double jerk_scale;
    double tcp_tol_m;
    uint64_t collision_scene_revision;
    double max_branch_step_rad;
    double min_singular_value;
} oa_converge_tcp_move;

/* Unit-aware binary64 ingress records. Only the XYZ member is converted once
 * to canonical metres. Fields whose names end in `_m` remain metres. */
typedef struct oa_centroid_tcp_move_with_units {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_length_unit coordinate_unit;
    uint32_t reserved0;
    uint64_t expiry_ns;
    uint64_t required_feedback_seq[2];
    oa_vec3d target_centroid;
    double velocity_scale;
    double acceleration_scale;
    double jerk_scale;
    double tcp_tol_m;
    uint64_t collision_scene_revision;
    double max_branch_step_rad;
    double min_singular_value;
} oa_centroid_tcp_move_with_units;

typedef struct oa_mirrored_tcp_move_with_units {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_length_unit coordinate_unit;
    uint32_t reserved0;
    uint64_t expiry_ns;
    uint64_t required_feedback_seq[2];
    oa_side lead_side;
    uint32_t reserved1;
    oa_vec3d lead_tcp;
    double velocity_scale;
    double acceleration_scale;
    double jerk_scale;
    double tcp_tol_m;
    uint64_t collision_scene_revision;
    double max_branch_step_rad;
    double min_singular_value;
} oa_mirrored_tcp_move_with_units;

typedef struct oa_converge_tcp_move_with_units {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_length_unit coordinate_unit;
    uint32_t reserved0;
    uint64_t expiry_ns;
    uint64_t required_feedback_seq[2];
    oa_vec3d target;
    double contact_torque_nm[7];
    double contact_torque_fraction;
    uint32_t contact_persistence_cycles;
    uint32_t reserved1;
    double stop_distance_m;
    double minimum_progress_m;
    double velocity_scale;
    double acceleration_scale;
    double jerk_scale;
    double tcp_tol_m;
    uint64_t collision_scene_revision;
    double max_branch_step_rad;
    double min_singular_value;
} oa_converge_tcp_move_with_units;

typedef uint32_t oa_stop_cause;
#define OA_STOP_CAUSE_NONE UINT32_C(0)
#define OA_STOP_CAUSE_CONTACT UINT32_C(1)
#define OA_STOP_CAUSE_KEEPOUT UINT32_C(2)
#define OA_STOP_CAUSE_PLAN_COMPLETE UINT32_C(3)
#define OA_STOP_CAUSE_ESTOP UINT32_C(4)

/* Outcome of the real-time monitors for the most recent command. */
typedef struct oa_contact_report {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_stop_cause cause;
    uint32_t contact_detected;
    /* Bit 0 left, bit 1 right. */
    uint32_t contact_side_mask;
    /* Bit n set when joint n exceeded its threshold. Indexed per side. */
    uint32_t contact_joint_mask[2];
    uint32_t keepout_violation;
    uint32_t keepout_side;
    uint32_t keepout_segment_a;
    uint32_t keepout_segment_b;
    uint64_t stop_feedback_seq[2];
    uint64_t stop_monotonic_ns;
    /* Measured joint torque and pose at the latching cycle. */
    double contact_torque_nm[2][7];
    double threshold_torque_nm[2][7];
    double stopped_q_rad[2][7];
    double stopped_tcp_m[2][3];
    double minimum_clearance_m;
} oa_contact_report;

double oa_control_default_contact_torque_fraction(void);
uint32_t oa_control_default_contact_persistence_cycles(void);

typedef struct oa_execute_request {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t start_ns;
    uint64_t expiry_ns;
    uint64_t producer_deadline_ns;
    uint32_t stop_kind;
} oa_execute_request;

typedef struct oa_event {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t kind;
    uint32_t lifecycle;
    uint64_t t_ns;
    uint64_t command_id;
    uint64_t feedback_seq;
    oa_control_status cause;
} oa_event;

typedef struct oa_controller_options {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t backend;
    uint32_t collision_policy;
    uint64_t cycle_ns;
    uint64_t feedback_timeout_ns;
    uint64_t max_cross_bus_skew_ns;
    uint64_t collision_scene_revision;
} oa_controller_options;

typedef struct oa_verify_report {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t verify_epoch;
    uint32_t verified_mask;
    uint32_t failure_mask;
} oa_verify_report;

typedef struct oa_arm_challenge {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t verify_epoch;
    uint64_t nonce;
    uint64_t expiry_ns;
} oa_arm_challenge;

typedef struct oa_reset_request {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t verify_epoch;
    uint64_t nonce;
} oa_reset_request;

typedef struct oa_arm_kinematics {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t feedback_seq;
    double q[7];
    double joint_xyz_m[7][3];
    double joint_axis_body[7][3];
    double tcp_transform[16];
    double tcp_xyz_m[3];
} oa_arm_kinematics;

typedef struct oa_motion_plan_report {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t kind;
    uint32_t collision_checked;
    uint64_t seed_feedback_seq[2];
    uint64_t duration_ns;
    uint64_t manifest_revision;
    uint64_t model_revision;
    uint64_t collision_scene_revision;
    double target_q[2][7];
    double achieved_tcp_m[2][3];
    double tcp_residual_m[2];
} oa_motion_plan_report;

typedef struct oa_sim_fault {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_side side;
    uint32_t freeze_mask;
    uint32_t drop_mask;
    uint32_t fault_mask;
    uint32_t fault_status;
    uint32_t command_fail_mask;
    /* Capture-to-publication latency for immutable quantized feedback generations. */
    uint64_t feedback_delay_ns;
} oa_sim_fault;

/* Virtual mechanical resistance for the simulated backend only.
 *
 * The obstacle is a body-frame sphere. Once the arm's measured hand_tcp
 * penetrates it the plant is held in place while the trajectory reference keeps
 * advancing, which is what a stiff position loop does against a hard stop. The
 * reported torque is the resulting servo effort: it grows with the reference
 * overshoot the motor is failing to close and saturates at the protocol tmax.
 *
 * Torque is deliberately not derived from penetration depth. Holding the plant
 * at the obstacle surface keeps penetration near zero, so a penetration-based
 * reaction would stay negligible no matter how hard the arm pushed.
 *
 * This models an object being grasped or an unexpected obstruction. It is not
 * an environment collision model. */
typedef struct oa_sim_contact {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_side side;
    uint32_t enabled;
    double center_m[3];
    double radius_m;
    /* Reaction torque per radian of reference overshoot past the held plant.
     * Zero selects a stiff default derived from each motor's tmax; a negative
     * gain is rejected rather than clamped. */
    double reaction_gain_nm_per_rad;
} oa_sim_contact;

typedef struct oa_sim_state {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_side side;
    double q[7];
    double dq[7];
} oa_sim_state;

/* Original V1 prefix sizes. Later V1 fields are optional append-only tails and
 * receive documented defaults when an older caller supplies exactly a prefix. */
#define OA_CONTROLLER_OPTIONS_V1_PREFIX_SIZE \
    ((uint32_t)offsetof(oa_controller_options, collision_scene_revision))
#define OA_PAIRED_TCP_MOVE_V1_PREFIX_SIZE \
    ((uint32_t)offsetof(oa_paired_tcp_move, max_branch_step_rad))
#define OA_SIM_FAULT_V1_PREFIX_SIZE \
    ((uint32_t)offsetof(oa_sim_fault, fault_status))

/* Process-wide emergency stop latch.
 *
 * A real E-stop is a facility-wide device, not a property of one controller
 * handle, and it must be honoured even when every lock in the process is held.
 * This latch is therefore a lock-free atomic owned by the library rather than
 * by any controller: oa_estop_assert never blocks, never allocates, and is safe
 * to call from any thread and from a signal handler.
 *
 * Every controller samples the latch at the very top of every control cycle, in
 * every lifecycle state, before any command, plan, or feedback processing. Once
 * latched, motion is refused and the arms are stopped; the latch survives faults
 * and command completion and clears only through an explicit oa_estop_clear.
 *
 * This is a software interlock. It is not a substitute for a hardwired,
 * safety-rated emergency stop in the power path. */
void oa_estop_assert(void);
uint32_t oa_estop_asserted(void);
/* Clears the latch. Returns OA_CONTROL_OK when the latch was released. */
oa_control_status oa_estop_clear(void);
/* Monotonically increasing count of assertions, for evidence and testing. */
uint64_t oa_estop_assert_count(void);

oa_control_status oa_manifest_create(const oa_manifest_config *config, oa_manifest **out);
oa_control_status oa_manifest_get_openarm_v10_virtual_config(oa_manifest_config *out);
oa_control_status oa_manifest_create_openarm_v10_virtual(oa_manifest **out);
/* Stage A uses the compiled config builder. Text/digest loading is reserved. */
oa_control_status oa_manifest_load(const char *path, const char *sha256_path,
                                   oa_manifest **out);
void oa_manifest_destroy(oa_manifest *manifest);

oa_control_status oa_controller_create(const oa_manifest *manifest,
                                       const oa_controller_options *options,
                                       oa_controller **out);
oa_control_status oa_controller_open_and_verify(oa_controller *controller,
                                                oa_verify_report *out);
oa_control_status oa_controller_snapshot(oa_controller *controller, oa_snapshot *out);
oa_control_status oa_controller_get_kinematics(oa_controller *controller, oa_side side,
                                               uint64_t required_feedback_seq,
                                               oa_arm_kinematics *out);
oa_control_status oa_controller_get_arm_challenge(oa_controller *controller,
                                                  oa_arm_challenge *out);
oa_control_status oa_controller_arm(oa_controller *controller,
                                    const oa_arm_challenge *challenge);
oa_control_status oa_controller_plan_joint(oa_controller *controller,
                                           const oa_joint_move *request,
                                           oa_motion_plan **out);
oa_control_status oa_controller_plan_paired_tcp(oa_controller *controller,
                                                const oa_paired_tcp_move *request,
                                                oa_motion_plan **out);
/* TCP coordinates are converted once from coordinate_unit to metres. tcp_tol_m
 * and every report value remain metres. */
oa_control_status oa_controller_plan_paired_tcp_with_units(
    oa_controller *controller,
    const oa_paired_tcp_move_with_units *request,
    oa_motion_plan **out);
/* Bimanual planners. Each derives both claw targets and then plans a single
 * all-or-nothing paired motion: if either arm cannot be solved the whole
 * request is rejected and no motion is produced. */
oa_control_status oa_controller_plan_centroid_tcp(oa_controller *controller,
                                                  const oa_centroid_tcp_move *request,
                                                  oa_motion_plan **out);
oa_control_status oa_controller_plan_centroid_tcp_with_units(
    oa_controller *controller, const oa_centroid_tcp_move_with_units *request,
    oa_motion_plan **out);
oa_control_status oa_controller_plan_mirrored_tcp(oa_controller *controller,
                                                  const oa_mirrored_tcp_move *request,
                                                  oa_motion_plan **out);
oa_control_status oa_controller_plan_mirrored_tcp_with_units(
    oa_controller *controller, const oa_mirrored_tcp_move_with_units *request,
    oa_motion_plan **out);
oa_control_status oa_controller_plan_converge_tcp(oa_controller *controller,
                                                  const oa_converge_tcp_move *request,
                                                  oa_motion_plan **out);
oa_control_status oa_controller_plan_converge_tcp_with_units(
    oa_controller *controller, const oa_converge_tcp_move_with_units *request,
    oa_motion_plan **out);
oa_control_status oa_motion_plan_get_report(const oa_motion_plan *plan,
                                            oa_motion_plan_report *out);
/* Real-time monitor outcome for the most recently executed command. Valid once
 * the command reaches a terminal event. */
oa_control_status oa_controller_get_contact_report(oa_controller *controller,
                                                   oa_contact_report *out);
oa_control_status oa_controller_sim_set_contact(oa_controller *controller,
                                                const oa_sim_contact *contact);
oa_control_status oa_controller_execute(oa_controller *controller,
                                        const oa_motion_plan *plan,
                                        const oa_execute_request *request,
                                        uint64_t *out_command_id);
oa_control_status oa_controller_advance(oa_controller *controller,
                                        uint64_t monotonic_ns);
oa_control_status oa_controller_sim_set_fault(oa_controller *controller,
                                              const oa_sim_fault *fault);
oa_control_status oa_controller_sim_set_state(oa_controller *controller,
                                              const oa_sim_state *state);
oa_control_status oa_controller_heartbeat(oa_controller *controller,
                                          uint64_t command_id,
                                          uint64_t producer_deadline_ns);
oa_control_status oa_controller_set_interlock(oa_controller *controller,
                                              uint32_t estop_active,
                                              uint32_t deadman_active);
oa_control_status oa_controller_set_collision_scene_revision(oa_controller *controller,
                                                              uint64_t revision);
oa_control_status oa_controller_stop(oa_controller *controller, uint32_t stop_kind);
oa_control_status oa_controller_disarm(oa_controller *controller, uint64_t deadline_ns);
oa_control_status oa_controller_reset_fault(oa_controller *controller,
                                            const oa_reset_request *request);
oa_control_status oa_controller_poll_event(oa_controller *controller,
                                           uint64_t deadline_ns, oa_event *out);
void oa_motion_plan_destroy(oa_motion_plan *plan);
void oa_controller_destroy(oa_controller *controller);

#ifdef __cplusplus
}
#endif
#endif
