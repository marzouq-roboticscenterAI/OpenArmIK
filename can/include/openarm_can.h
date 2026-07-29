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

#define OA_CAN_ABI_VERSION 1u
#define OA_CAN_MAX_MOTORS 8u
#define OA_CAN_MAX_INTERFACES 32u
#define OA_CAN_SFF_MASK 0x000007ffu
#define OA_CAN_EFF_FLAG 0x80000000u
#define OA_CAN_RTR_FLAG 0x40000000u
#define OA_CAN_ERR_FLAG 0x20000000u

typedef enum oa_can_status {
    OA_CAN_OK = 0,
    OA_CAN_EINVAL,
    OA_CAN_ERANGE,
    OA_CAN_EFRAME,
    OA_CAN_EID,
    OA_CAN_EFAULT,
    OA_CAN_ESTATE,
    OA_CAN_ETIMEOUT,
    OA_CAN_EIO,
    OA_CAN_ENOMEM,
    OA_CAN_EUNSUPPORTED
} oa_can_status;

typedef enum oa_can_motor_type {
    OA_CAN_MOTOR_DM8009 = 1,
    OA_CAN_MOTOR_DM4340 = 2,
    OA_CAN_MOTOR_DM4310 = 3
} oa_can_motor_type;

typedef enum oa_can_range_policy {
    OA_CAN_RANGE_REJECT = 0,
    OA_CAN_RANGE_SATURATE = 1
} oa_can_range_policy;

typedef enum oa_can_feedback_status {
    OA_CAN_FEEDBACK_DISABLED = 0,
    OA_CAN_FEEDBACK_ENABLED = 1,
    OA_CAN_FEEDBACK_OVER_VOLTAGE = 8,
    OA_CAN_FEEDBACK_UNDER_VOLTAGE = 9,
    OA_CAN_FEEDBACK_OVER_CURRENT = 10,
    OA_CAN_FEEDBACK_MOS_OVER_TEMPERATURE = 11,
    OA_CAN_FEEDBACK_ROTOR_OVER_TEMPERATURE = 12,
    OA_CAN_FEEDBACK_LOST_COMMUNICATIONS = 13,
    OA_CAN_FEEDBACK_OVERLOAD = 14
} oa_can_feedback_status;

typedef enum oa_can_fake_lifecycle {
    OA_CAN_FAKE_CREATED = 0,
    OA_CAN_FAKE_DISABLED,
    OA_CAN_FAKE_PROBING,
    OA_CAN_FAKE_PROBED,
    OA_CAN_FAKE_FAULT
} oa_can_fake_lifecycle;

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
    oa_can_motor_type motor_type;
    oa_can_joint_mapping mapping;
    char joint_name[32];
    char serial[32];
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
    uint32_t unexpected_frames;
    uint32_t invalid_frames;
    uint32_t refresh_frames_sent;
} oa_can_probe_report;

typedef struct oa_can_transport {
    uint32_t struct_size;
    uint32_t abi_version;
    void *context;
    oa_can_status (*send)(void *context, const oa_can_frame *frame);
    oa_can_status (*receive)(void *context, oa_can_frame *frame);
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
oa_can_status oa_can_probe_expected(const oa_can_transport *transport,
                                    const oa_can_arm_manifest *manifest,
                                    oa_can_probe_report *out_report);

/* Linux-only and read-only: no SETLINK, ioctl write, shell command, or CAN socket. */
oa_can_status oa_can_linux_list_interfaces(oa_can_linux_interface *interfaces,
                                           size_t capacity, size_t *out_count);

oa_can_status oa_can_fake_create(size_t queue_capacity, oa_can_fake **out_fake);
void oa_can_fake_destroy(oa_can_fake *fake);
oa_can_status oa_can_fake_transport(oa_can_fake *fake, oa_can_transport *out_transport);
oa_can_status oa_can_fake_enqueue_feedback(oa_can_fake *fake, const oa_can_frame *frame);
oa_can_fake_lifecycle oa_can_fake_get_lifecycle(const oa_can_fake *fake);
size_t oa_can_fake_sent_count(const oa_can_fake *fake);
int oa_can_fake_torque_enabled(const oa_can_fake *fake);

#ifdef __cplusplus
}
#endif

#endif
