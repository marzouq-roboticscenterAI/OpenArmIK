/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Public C11 client ABI for the production run-real.sh controller.
 *
 * This library never opens SocketCAN and never invokes sudo. The separately
 * launched ROS controller remains the sole physical authority, so every call
 * inherits its inventory, calibration, watchdog, collision, E-stop, and
 * confirmed-disable behavior.
 */
#ifndef OPENARM_REAL_H
#define OPENARM_REAL_H

#include <stdint.h>

#include <openarm_units.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OA_REAL_ABI_VERSION UINT32_C(1)
#define OA_REAL_ARMS UINT32_C(2)
#define OA_REAL_JOINTS UINT32_C(7)
#define OA_REAL_REASON_CAPACITY UINT32_C(256)

typedef int32_t oa_real_status;
#define OA_REAL_OK ((oa_real_status)0)
#define OA_REAL_EINVAL ((oa_real_status)1)
#define OA_REAL_EABI ((oa_real_status)2)
#define OA_REAL_EUNAVAILABLE ((oa_real_status)3)
#define OA_REAL_ETIMEOUT ((oa_real_status)4)
#define OA_REAL_EREJECTED ((oa_real_status)5)
#define OA_REAL_EABORTED ((oa_real_status)6)
#define OA_REAL_ECANCELED ((oa_real_status)7)
#define OA_REAL_ESTALE ((oa_real_status)8)
#define OA_REAL_EINTERNAL ((oa_real_status)9)

typedef uint32_t oa_real_side;
#define OA_REAL_SIDE_LEFT UINT32_C(0)
#define OA_REAL_SIDE_RIGHT UINT32_C(1)

typedef uint32_t oa_real_gripper_mask;
#define OA_REAL_GRIPPER_LEFT UINT32_C(1)
#define OA_REAL_GRIPPER_RIGHT UINT32_C(2)
#define OA_REAL_GRIPPER_BOTH UINT32_C(3)

typedef struct oa_real_client oa_real_client;

typedef struct oa_real_result {
    uint32_t abi_version;
    uint32_t struct_size;
    oa_real_status status;
    uint32_t outcome;
    uint64_t command_id;
    uint32_t cause;
    uint32_t collision_checked;
    double measured_progress;
    char reason[OA_REAL_REASON_CAPACITY];
} oa_real_result;

typedef struct oa_real_snapshot {
    uint32_t abi_version;
    uint32_t struct_size;
    uint32_t controller_available;
    uint32_t connected;
    uint32_t armed;
    uint32_t estop_asserted;
    uint32_t busy;
    uint32_t encoder_state_valid;
    uint64_t ros_stamp_ns;
    uint64_t receipt_steady_ns;
    /* Robot-left is index 0, robot-right is index 1. */
    double joint_position_rad[OA_REAL_ARMS][OA_REAL_JOINTS];
    double joint_velocity_rad_s[OA_REAL_ARMS][OA_REAL_JOINTS];
    double joint_torque_nm[OA_REAL_ARMS][OA_REAL_JOINTS];
    double tcp_m[OA_REAL_ARMS][3];
    double gripper_opening_m[OA_REAL_ARMS];
    double gripper_velocity_m_s[OA_REAL_ARMS];
    double gripper_motor_torque_nm[OA_REAL_ARMS];
    /* Bit 0 is robot-left and bit 1 is robot-right. Only selected arms are
     * encoder-derived; an inactive side is a fixed planning placeholder. */
    uint32_t active_side_mask;
} oa_real_snapshot;

void oa_real_result_init(oa_real_result *out);
void oa_real_snapshot_init(oa_real_snapshot *out);
const char *oa_real_status_string(oa_real_status status);

/* Creates an unprivileged ROS client. It does not contact CAN or enable a
 * motor. Destroying a client does not disconnect the shared controller. */
oa_real_status oa_real_client_create(oa_real_client **out);
void oa_real_client_destroy(oa_real_client *client);

/* Waits for the production controller's status, services, and action servers.
 * Timeout values are finite positive milliseconds. */
oa_real_status oa_real_client_wait_ready(
    oa_real_client *client, uint32_t timeout_ms, oa_real_result *out);

/* Reads fresh state for every arm selected by active_side_mask and computes
 * both TCPs with the pinned binary64 C FK model. An inactive arm is a fixed
 * planning placeholder, not encoder feedback. Passive status is still
 * returned when disconnected; encoder_state_valid will be zero. */
oa_real_status oa_real_client_read(
    oa_real_client *client, uint32_t timeout_ms, oa_real_snapshot *out);

/* Connect performs the production 16-motor inventory, calibration validation,
 * volatile watchdog/gripper-mode configuration, encoder-seeded hold, and gain
 * ramp before it reports success. CAN interfaces must already be up. */
oa_real_status oa_real_client_connect(
    oa_real_client *client, uint32_t timeout_ms, oa_real_result *out);
oa_real_status oa_real_client_disconnect(
    oa_real_client *client, uint32_t timeout_ms, oa_real_result *out);
oa_real_status oa_real_client_stop(
    oa_real_client *client, uint32_t timeout_ms, oa_real_result *out);
oa_real_status oa_real_client_estop(
    oa_real_client *client, uint32_t timeout_ms, oa_real_result *out);
oa_real_status oa_real_client_estop_clear(
    oa_real_client *client, uint32_t timeout_ms, oa_real_result *out);
oa_real_status oa_real_client_neutral(
    oa_real_client *client, uint32_t timeout_ms, oa_real_result *out);

/* Joints are numbered 1 through 7 at this boundary. target_rad is an absolute
 * calibrated model angle derived from the same encoders used by RViz. */
oa_real_status oa_real_client_move_joint(
    oa_real_client *client, oa_real_side side, uint32_t joint,
    double target_rad, uint32_t timeout_ms, oa_real_result *out);

/* All coordinates and calculations are IEEE-754 binary64. The explicit unit
 * is converted exactly once to metres before the ROS metric boundary.
 * motion_limit_scale is finite in [0.5, 1.0]; the physical controller may
 * impose a lower safety cap. Single-arm motion preserves every measured joint
 * of the unselected arm throughout all dynamically checked route legs. */
oa_real_status oa_real_client_move_tcp(
    oa_real_client *client, oa_real_side side, const oa_vec3d *target,
    oa_length_unit unit, double motion_limit_scale, uint32_t timeout_ms,
    oa_real_result *out);
oa_real_status oa_real_client_move_paired_tcp(
    oa_real_client *client, const oa_vec3d *left_target,
    const oa_vec3d *right_target, oa_length_unit unit,
    double motion_limit_scale, uint32_t timeout_ms, oa_real_result *out);

/* Opening is 0.0 m closed through 0.044 m open. The production server enforces
 * speed <= 0.011 m/s and motor torque in [0.05, 1.5] Nm. */
oa_real_status oa_real_client_move_gripper(
    oa_real_client *client, oa_real_gripper_mask side_mask,
    double target_opening_m, double maximum_opening_speed_m_s,
    double maximum_motor_torque_nm, uint32_t stop_on_contact,
    uint32_t timeout_ms, oa_real_result *out);

#ifdef __cplusplus
}
#endif
#endif
