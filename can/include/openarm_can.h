/* SPDX-License-Identifier: Apache-2.0 */
/*
 * OpenArm CAN diagnostics API.
 * Protocol facts are independently implemented from DaMiao documentation and
 * OpenArm CAN evidence at c32ecd31da267967f0c913c2118c843177d88b91.
 */
#ifndef OPENARM_CAN_H
#define OPENARM_CAN_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OA_CAN_ABI_VERSION UINT32_C(2)
#define OA_CAN_MAX_MOTORS 8u
#define OA_CAN_MAX_INTERFACES 32u
#define OA_CAN_SFF_MASK 0x000007ffu
#define OA_CAN_EFF_FLAG 0x80000000u
#define OA_CAN_RTR_FLAG 0x40000000u
#define OA_CAN_ERR_FLAG 0x20000000u

typedef uint32_t oa_can_status;
#define OA_CAN_OK UINT32_C(0)
#define OA_CAN_EINVAL UINT32_C(1)
#define OA_CAN_ERANGE UINT32_C(2)
#define OA_CAN_EFRAME UINT32_C(3)
#define OA_CAN_EID UINT32_C(4)
#define OA_CAN_EFAULT UINT32_C(5)
#define OA_CAN_ESTATE UINT32_C(6)
#define OA_CAN_ETIMEOUT UINT32_C(7)
#define OA_CAN_EIO UINT32_C(8)
#define OA_CAN_ENOMEM UINT32_C(9)
#define OA_CAN_EUNSUPPORTED UINT32_C(10)

typedef uint32_t oa_can_motor_type;
#define OA_CAN_MOTOR_DM8009 UINT32_C(1)
#define OA_CAN_MOTOR_DM4340 UINT32_C(2)
#define OA_CAN_MOTOR_DM4310 UINT32_C(3)

typedef uint32_t oa_can_range_policy;
#define OA_CAN_RANGE_REJECT UINT32_C(0)
#define OA_CAN_RANGE_SATURATE UINT32_C(1)

typedef uint8_t oa_can_feedback_status;
#define OA_CAN_FEEDBACK_DISABLED UINT8_C(0)
#define OA_CAN_FEEDBACK_ENABLED UINT8_C(1)
#define OA_CAN_FEEDBACK_OVER_VOLTAGE UINT8_C(8)
#define OA_CAN_FEEDBACK_UNDER_VOLTAGE UINT8_C(9)
#define OA_CAN_FEEDBACK_OVER_CURRENT UINT8_C(10)
#define OA_CAN_FEEDBACK_MOS_OVER_TEMPERATURE UINT8_C(11)
#define OA_CAN_FEEDBACK_ROTOR_OVER_TEMPERATURE UINT8_C(12)
#define OA_CAN_FEEDBACK_LOST_COMMUNICATIONS UINT8_C(13)
#define OA_CAN_FEEDBACK_OVERLOAD UINT8_C(14)

typedef uint32_t oa_can_fake_lifecycle;
#define OA_CAN_FAKE_CREATED UINT32_C(0)
#define OA_CAN_FAKE_DISABLED UINT32_C(1)
#define OA_CAN_FAKE_PROBING UINT32_C(2)
#define OA_CAN_FAKE_PROBED UINT32_C(3)
#define OA_CAN_FAKE_FAULT UINT32_C(4)

typedef struct oa_can_limits {
    uint32_t struct_size;
    uint32_t abi_version;
    double position_min_rad;
    double position_max_rad;
    double velocity_min_rad_s;
    double velocity_max_rad_s;
    double torque_min_nm;
    double torque_max_nm;
    double kp_min;
    double kp_max;
    double kd_min;
    double kd_max;
} oa_can_limits;

typedef struct oa_can_frame {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t can_id;
    uint8_t dlc;
    uint8_t data[8];
} oa_can_frame;

typedef struct oa_can_received_frame {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t received_monotonic_ns;
    oa_can_frame frame;
} oa_can_received_frame;

typedef struct oa_can_mit_command {
    uint32_t struct_size;
    uint32_t abi_version;
    uint16_t send_id;
    oa_can_motor_type motor_type;
    double position_rad;
    double velocity_rad_s;
    double kp;
    double kd;
    double torque_nm;
} oa_can_mit_command;

typedef struct oa_can_encode_result {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t saturated_mask;
} oa_can_encode_result;

typedef struct oa_can_feedback {
    uint32_t struct_size;
    uint32_t abi_version;
    uint16_t receive_id;
    uint8_t motor_id;
    uint8_t status_nibble;
    double position_rad;
    double velocity_rad_s;
    double torque_nm;
    uint8_t mos_temperature_c;
    uint8_t rotor_temperature_c;
} oa_can_feedback;

typedef struct oa_can_joint_mapping {
    uint32_t struct_size;
    uint32_t abi_version;
    double position_scale;
    double position_offset_rad;
    double velocity_scale;
    double torque_scale;
} oa_can_joint_mapping;

typedef struct oa_can_motor_manifest {
    uint32_t struct_size;
    uint32_t abi_version;
    uint8_t joint_index;
    uint8_t expected_motor_id;
    uint16_t send_id;
    uint16_t receive_id;
    /* Local decode range only; oa_can_probe_expected does not verify motor identity. */
    oa_can_motor_type decode_motor_type;
    oa_can_joint_mapping mapping;
    char joint_name[32];
    /* Local commissioning metadata; not read or verified by this module. */
    char commissioned_serial[32];
} oa_can_motor_manifest;

typedef struct oa_can_arm_manifest {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t motor_count;
    oa_can_motor_manifest motors[OA_CAN_MAX_MOTORS];
} oa_can_arm_manifest;

typedef struct oa_can_probe_report {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t expected_mask;
    uint32_t fresh_mask;
    uint32_t fault_mask;
    uint32_t duplicate_mask;
    uint32_t enabled_mask;
    uint32_t unexpected_frames;
    uint32_t invalid_frames;
    uint32_t stale_frames;
    uint32_t received_frames;
    uint32_t refresh_frames_sent;
    uint32_t receive_limit_reached;
    uint64_t deadline_monotonic_ns;
} oa_can_probe_report;

typedef struct oa_can_probe_options {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t timeout_ns;
    /*
     * Maximum successfully received frames before the result is inconclusive.
     * Success requires the transport to return OA_CAN_ETIMEOUT before this many
     * frames are consumed. Reaching the exact cap returns OA_CAN_ETIMEOUT even
     * when all expected IDs were seen, because an unseen duplicate/fault may
     * remain. Choose a value greater than the expected reply count.
     */
    uint32_t max_receive_frames;
} oa_can_probe_options;

typedef struct oa_can_transport {
    uint32_t struct_size;
    uint32_t abi_version;
    void *context;
    oa_can_status (*now)(void *context, uint64_t *out_monotonic_ns);
    oa_can_status (*send)(void *context, const oa_can_frame *frame,
                          uint64_t *out_sent_monotonic_ns);
    oa_can_status (*receive)(void *context, uint64_t deadline_monotonic_ns,
                             oa_can_received_frame *out_frame);
} oa_can_transport;

typedef struct oa_can_linux_interface {
    uint32_t struct_size;
    uint32_t abi_version;
    char name[16];
    uint32_t ifindex;
    uint32_t flags;
    uint32_t mtu;
    uint32_t bitrate;
    uint32_t data_bitrate;
    uint8_t link_up;
    uint8_t fd_enabled;
} oa_can_linux_interface;

typedef struct oa_can_fake oa_can_fake;

oa_can_status oa_can_motor_limits(oa_can_motor_type motor_type, oa_can_limits *out_limits);
oa_can_status oa_can_encode_mit(const oa_can_mit_command *command,
                                oa_can_range_policy policy,
                                oa_can_frame *out_frame,
                                oa_can_encode_result *out_result);
oa_can_status oa_can_decode_feedback(const oa_can_frame *frame,
                                     uint16_t expected_receive_id,
                                     uint8_t expected_motor_id,
                                     oa_can_motor_type motor_type,
                                     oa_can_feedback *out_feedback);
oa_can_status oa_can_make_enable(uint16_t send_id, oa_can_frame *out_frame);
oa_can_status oa_can_make_disable(uint16_t send_id, oa_can_frame *out_frame);
oa_can_status oa_can_make_refresh_status(uint16_t send_id, oa_can_frame *out_frame);
oa_can_status oa_can_make_register_query(uint16_t send_id, uint8_t register_id,
                                         oa_can_frame *out_frame);
oa_can_status oa_can_validate_manifest(const oa_can_arm_manifest *manifest);
/*
 * Verifies fresh, disabled feedback at expected arbitration/embedded IDs only.
 * It does not verify model, serial, joint assignment, firmware, or registers.
 */
oa_can_status oa_can_probe_expected(const oa_can_transport *transport,
                                    const oa_can_arm_manifest *manifest,
                                    const oa_can_probe_options *options,
                                    oa_can_probe_report *out_report);

/*
 * Linux-UAPI-only and read-only: no SETLINK, ioctl write, shell command, or CAN
 * socket. Caller initializes every output slot's size/version before the call.
 */
oa_can_status oa_can_linux_list_interfaces(oa_can_linux_interface *interfaces,
                                           size_t capacity, size_t *out_count);

oa_can_status oa_can_fake_create(size_t queue_capacity, oa_can_fake **out_fake);
void oa_can_fake_destroy(oa_can_fake *fake);
/* The fake diagnostics transport rejects enable and MIT/control frames. */
oa_can_status oa_can_fake_transport(oa_can_fake *fake, oa_can_transport *out_transport);
oa_can_status oa_can_fake_set_time(oa_can_fake *fake, uint64_t monotonic_ns);
oa_can_status oa_can_fake_enqueue_feedback(oa_can_fake *fake, const oa_can_frame *frame,
                                           uint64_t received_monotonic_ns);
oa_can_fake_lifecycle oa_can_fake_get_lifecycle(const oa_can_fake *fake);
size_t oa_can_fake_sent_count(const oa_can_fake *fake);
size_t oa_can_fake_rejected_control_count(const oa_can_fake *fake);
int oa_can_fake_torque_enabled(const oa_can_fake *fake);

#ifdef __cplusplus
}
#endif

#endif
