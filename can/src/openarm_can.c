/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_can.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

struct oa_can_fake {
    oa_can_frame *queue;
    size_t capacity;
    size_t head;
    size_t count;
    size_t sent_count;
    oa_can_fake_lifecycle lifecycle;
};

enum {
    OA_FIELD_POSITION = 1u << 0,
    OA_FIELD_VELOCITY = 1u << 1,
    OA_FIELD_KP = 1u << 2,
    OA_FIELD_KD = 1u << 3,
    OA_FIELD_TORQUE = 1u << 4
};

static int oa_valid_version(uint32_t struct_size, size_t required, uint32_t abi_version) {
    return struct_size >= required && abi_version == OA_CAN_ABI_VERSION;
}

static int oa_valid_standard_id(uint16_t id) {
    return id > 0u && id <= OA_CAN_SFF_MASK;
}

static int oa_valid_frame(const oa_can_frame *frame) {
    return frame != NULL && oa_valid_version(frame->struct_size, sizeof(*frame), frame->abi_version);
}

static int oa_valid_type(oa_can_motor_type motor_type) {
    return motor_type == OA_CAN_MOTOR_DM8009 || motor_type == OA_CAN_MOTOR_DM4340 ||
           motor_type == OA_CAN_MOTOR_DM4310;
}

static void oa_init_frame(oa_can_frame *frame, uint16_t id) {
    (void)memset(frame, 0, sizeof(*frame));
    frame->struct_size = (uint32_t)sizeof(*frame);
    frame->abi_version = OA_CAN_ABI_VERSION;
    frame->can_id = id;
    frame->dlc = 8u;
}

static double oa_from_uint(uint32_t value, double lower, double upper, unsigned int bits) {
    const double maximum = (double)((1u << bits) - 1u);
    return ((double)value / maximum) * (upper - lower) + lower;
}

static uint32_t oa_to_uint(double value, double lower, double upper, unsigned int bits) {
    const double maximum = (double)((1u << bits) - 1u);
    return (uint32_t)((value - lower) * maximum / (upper - lower));
}

static oa_can_status oa_limit_value(double input, double lower, double upper,
                                    oa_can_range_policy policy, uint32_t bit,
                                    uint32_t *saturated_mask, double *out_value) {
    if (!isfinite(input)) {
        return OA_CAN_EINVAL;
    }
    if (input < lower || input > upper) {
        if (policy != OA_CAN_RANGE_SATURATE) {
            return OA_CAN_ERANGE;
        }
        *saturated_mask |= bit;
        *out_value = input < lower ? lower : upper;
        return OA_CAN_OK;
    }
    *out_value = input;
    return OA_CAN_OK;
}

oa_can_status oa_can_motor_limits(oa_can_motor_type motor_type, oa_can_limits *out_limits) {
    if (out_limits == NULL || !oa_valid_type(motor_type)) {
        return OA_CAN_EINVAL;
    }
    (void)memset(out_limits, 0, sizeof(*out_limits));
    out_limits->struct_size = (uint32_t)sizeof(*out_limits);
    out_limits->abi_version = OA_CAN_ABI_VERSION;
    out_limits->position_min_rad = -12.5;
    out_limits->position_max_rad = 12.5;
    out_limits->kp_min = 0.0;
    out_limits->kp_max = 500.0;
    out_limits->kd_min = 0.0;
    out_limits->kd_max = 5.0;
    if (motor_type == OA_CAN_MOTOR_DM8009) {
        out_limits->velocity_min_rad_s = -45.0;
        out_limits->velocity_max_rad_s = 45.0;
        out_limits->torque_min_nm = -54.0;
        out_limits->torque_max_nm = 54.0;
    } else if (motor_type == OA_CAN_MOTOR_DM4340) {
        out_limits->velocity_min_rad_s = -10.0;
        out_limits->velocity_max_rad_s = 10.0;
        out_limits->torque_min_nm = -28.0;
        out_limits->torque_max_nm = 28.0;
    } else {
        out_limits->velocity_min_rad_s = -30.0;
        out_limits->velocity_max_rad_s = 30.0;
        out_limits->torque_min_nm = -10.0;
        out_limits->torque_max_nm = 10.0;
    }
    return OA_CAN_OK;
}

oa_can_status oa_can_encode_mit(const oa_can_mit_command *command,
                                oa_can_range_policy policy,
                                oa_can_frame *out_frame,
                                oa_can_encode_result *out_result) {
    oa_can_limits limits;
    double position;
    double velocity;
    double kp;
    double kd;
    double torque;
    uint32_t mask = 0u;
    uint32_t position_raw;
    uint32_t velocity_raw;
    uint32_t kp_raw;
    uint32_t kd_raw;
    uint32_t torque_raw;
    oa_can_status status;
    if (command == NULL || out_frame == NULL || out_result == NULL ||
        !oa_valid_version(command->struct_size, sizeof(*command), command->abi_version) ||
        !oa_valid_standard_id(command->send_id) ||
        (policy != OA_CAN_RANGE_REJECT && policy != OA_CAN_RANGE_SATURATE)) {
        return OA_CAN_EINVAL;
    }
    status = oa_can_motor_limits(command->motor_type, &limits);
    if (status != OA_CAN_OK) {
        return status;
    }
    status = oa_limit_value(command->position_rad, limits.position_min_rad, limits.position_max_rad,
                            policy, OA_FIELD_POSITION, &mask, &position);
    if (status != OA_CAN_OK) return status;
    status = oa_limit_value(command->velocity_rad_s, limits.velocity_min_rad_s, limits.velocity_max_rad_s,
                            policy, OA_FIELD_VELOCITY, &mask, &velocity);
    if (status != OA_CAN_OK) return status;
    status = oa_limit_value(command->kp, limits.kp_min, limits.kp_max, policy, OA_FIELD_KP, &mask, &kp);
    if (status != OA_CAN_OK) return status;
    status = oa_limit_value(command->kd, limits.kd_min, limits.kd_max, policy, OA_FIELD_KD, &mask, &kd);
    if (status != OA_CAN_OK) return status;
    status = oa_limit_value(command->torque_nm, limits.torque_min_nm, limits.torque_max_nm,
                            policy, OA_FIELD_TORQUE, &mask, &torque);
    if (status != OA_CAN_OK) return status;
    position_raw = oa_to_uint(position, limits.position_min_rad, limits.position_max_rad, 16u);
    velocity_raw = oa_to_uint(velocity, limits.velocity_min_rad_s, limits.velocity_max_rad_s, 12u);
    kp_raw = oa_to_uint(kp, limits.kp_min, limits.kp_max, 12u);
    kd_raw = oa_to_uint(kd, limits.kd_min, limits.kd_max, 12u);
    torque_raw = oa_to_uint(torque, limits.torque_min_nm, limits.torque_max_nm, 12u);
    oa_init_frame(out_frame, command->send_id);
    out_frame->data[0] = (uint8_t)(position_raw >> 8u);
    out_frame->data[1] = (uint8_t)position_raw;
    out_frame->data[2] = (uint8_t)(velocity_raw >> 4u);
    out_frame->data[3] = (uint8_t)((velocity_raw << 4u) | (kp_raw >> 8u));
    out_frame->data[4] = (uint8_t)kp_raw;
    out_frame->data[5] = (uint8_t)(kd_raw >> 4u);
    out_frame->data[6] = (uint8_t)((kd_raw << 4u) | (torque_raw >> 8u));
    out_frame->data[7] = (uint8_t)torque_raw;
    (void)memset(out_result, 0, sizeof(*out_result));
    out_result->struct_size = (uint32_t)sizeof(*out_result);
    out_result->abi_version = OA_CAN_ABI_VERSION;
    out_result->saturated_mask = mask;
    return OA_CAN_OK;
}

static int oa_known_feedback_status(uint8_t status) {
    return status == OA_CAN_FEEDBACK_DISABLED || status == OA_CAN_FEEDBACK_ENABLED ||
           (status >= OA_CAN_FEEDBACK_OVER_VOLTAGE && status <= OA_CAN_FEEDBACK_OVERLOAD);
}

oa_can_status oa_can_decode_feedback(const oa_can_frame *frame,
                                     uint16_t expected_receive_id,
                                     uint8_t expected_motor_id,
                                     oa_can_motor_type motor_type,
                                     oa_can_feedback *out_feedback) {
    oa_can_limits limits;
    uint8_t motor_id;
    uint8_t status_nibble;
    uint32_t position_raw;
    uint32_t velocity_raw;
    uint32_t torque_raw;
    oa_can_status status;
    if (!oa_valid_frame(frame) || out_feedback == NULL || !oa_valid_standard_id(expected_receive_id) ||
        expected_motor_id > 15u || !oa_valid_type(motor_type)) {
        return OA_CAN_EINVAL;
    }
    if ((frame->can_id & (OA_CAN_EFF_FLAG | OA_CAN_RTR_FLAG | OA_CAN_ERR_FLAG)) != 0u ||
        frame->can_id > OA_CAN_SFF_MASK || frame->can_id != expected_receive_id || frame->dlc != 8u) {
        return frame->can_id != expected_receive_id ? OA_CAN_EID : OA_CAN_EFRAME;
    }
    motor_id = (uint8_t)(frame->data[0] & 0x0fu);
    status_nibble = (uint8_t)(frame->data[0] >> 4u);
    if (motor_id != expected_motor_id) {
        return OA_CAN_EID;
    }
    if (!oa_known_feedback_status(status_nibble)) {
        return OA_CAN_EFRAME;
    }
    status = oa_can_motor_limits(motor_type, &limits);
    if (status != OA_CAN_OK) return status;
    position_raw = ((uint32_t)frame->data[1] << 8u) | frame->data[2];
    velocity_raw = ((uint32_t)frame->data[3] << 4u) | (frame->data[4] >> 4u);
    torque_raw = ((uint32_t)(frame->data[4] & 0x0fu) << 8u) | frame->data[5];
    (void)memset(out_feedback, 0, sizeof(*out_feedback));
    out_feedback->struct_size = (uint32_t)sizeof(*out_feedback);
    out_feedback->abi_version = OA_CAN_ABI_VERSION;
    out_feedback->receive_id = expected_receive_id;
    out_feedback->motor_id = motor_id;
    out_feedback->status_nibble = status_nibble;
    out_feedback->position_rad = oa_from_uint(position_raw, limits.position_min_rad, limits.position_max_rad, 16u);
    out_feedback->velocity_rad_s = oa_from_uint(velocity_raw, limits.velocity_min_rad_s, limits.velocity_max_rad_s, 12u);
    out_feedback->torque_nm = oa_from_uint(torque_raw, limits.torque_min_nm, limits.torque_max_nm, 12u);
    out_feedback->mos_temperature_c = frame->data[6];
    out_feedback->rotor_temperature_c = frame->data[7];
    return status_nibble == OA_CAN_FEEDBACK_ENABLED || status_nibble == OA_CAN_FEEDBACK_DISABLED ?
           OA_CAN_OK : OA_CAN_EFAULT;
}

static oa_can_status oa_make_special(uint16_t send_id, uint8_t last_byte, oa_can_frame *out_frame) {
    if (!oa_valid_standard_id(send_id) || out_frame == NULL) return OA_CAN_EINVAL;
    oa_init_frame(out_frame, send_id);
    (void)memset(out_frame->data, 0xff, sizeof(out_frame->data));
    out_frame->data[7] = last_byte;
    return OA_CAN_OK;
}

oa_can_status oa_can_make_enable(uint16_t send_id, oa_can_frame *out_frame) {
    return oa_make_special(send_id, 0xfcu, out_frame);
}

oa_can_status oa_can_make_disable(uint16_t send_id, oa_can_frame *out_frame) {
    return oa_make_special(send_id, 0xfdu, out_frame);
}

oa_can_status oa_can_make_refresh_status(uint16_t send_id, oa_can_frame *out_frame) {
    if (!oa_valid_standard_id(send_id) || out_frame == NULL) return OA_CAN_EINVAL;
    oa_init_frame(out_frame, 0x7ffu);
    out_frame->data[0] = (uint8_t)send_id;
    out_frame->data[1] = (uint8_t)(send_id >> 8u);
    out_frame->data[2] = 0xccu;
    return OA_CAN_OK;
}

oa_can_status oa_can_make_register_query(uint16_t send_id, uint8_t register_id, oa_can_frame *out_frame) {
    if (!oa_valid_standard_id(send_id) || out_frame == NULL) return OA_CAN_EINVAL;
    oa_init_frame(out_frame, 0x7ffu);
    out_frame->data[0] = (uint8_t)send_id;
    out_frame->data[1] = (uint8_t)(send_id >> 8u);
    out_frame->data[2] = 0x33u;
    out_frame->data[3] = register_id;
    return OA_CAN_OK;
}

oa_can_status oa_can_validate_manifest(const oa_can_arm_manifest *manifest) {
    uint32_t i;
    uint32_t j;
    if (manifest == NULL || !oa_valid_version(manifest->struct_size, sizeof(*manifest), manifest->abi_version) ||
        manifest->motor_count == 0u || manifest->motor_count > OA_CAN_MAX_MOTORS) return OA_CAN_EINVAL;
    for (i = 0u; i < manifest->motor_count; ++i) {
        const oa_can_motor_manifest *motor = &manifest->motors[i];
        const oa_can_joint_mapping *mapping = &motor->mapping;
        if (!oa_valid_version(motor->struct_size, sizeof(*motor), motor->abi_version) ||
            !oa_valid_version(mapping->struct_size, sizeof(*mapping), mapping->abi_version) ||
            motor->joint_index >= OA_CAN_MAX_MOTORS || motor->expected_motor_id > 15u ||
            !oa_valid_standard_id(motor->send_id) || !oa_valid_standard_id(motor->receive_id) ||
            !oa_valid_type(motor->motor_type) || !isfinite(mapping->position_scale) ||
            !isfinite(mapping->position_offset_rad) || !isfinite(mapping->velocity_scale) ||
            !isfinite(mapping->torque_scale) || mapping->position_scale == 0.0 ||
            mapping->velocity_scale == 0.0 || mapping->torque_scale == 0.0 || motor->joint_name[0] == '\0') {
            return OA_CAN_EINVAL;
        }
        for (j = 0u; j < i; ++j) {
            const oa_can_motor_manifest *prior = &manifest->motors[j];
            if (prior->joint_index == motor->joint_index || prior->send_id == motor->send_id ||
                prior->receive_id == motor->receive_id || prior->expected_motor_id == motor->expected_motor_id) {
                return OA_CAN_EINVAL;
            }
        }
    }
    return OA_CAN_OK;
}

oa_can_status oa_can_probe_expected(const oa_can_transport *transport,
                                    const oa_can_arm_manifest *manifest,
                                    oa_can_probe_report *out_report) {
    uint32_t i;
    oa_can_status result;
    if (transport == NULL || out_report == NULL ||
        !oa_valid_version(transport->struct_size, sizeof(*transport), transport->abi_version) ||
        transport->send == NULL || transport->receive == NULL) return OA_CAN_EINVAL;
    result = oa_can_validate_manifest(manifest);
    if (result != OA_CAN_OK) return result;
    (void)memset(out_report, 0, sizeof(*out_report));
    out_report->struct_size = (uint32_t)sizeof(*out_report);
    out_report->abi_version = OA_CAN_ABI_VERSION;
    for (i = 0u; i < manifest->motor_count; ++i) {
        oa_can_frame refresh;
        result = oa_can_make_refresh_status(manifest->motors[i].send_id, &refresh);
        if (result != OA_CAN_OK) return result;
        result = transport->send(transport->context, &refresh);
        if (result != OA_CAN_OK) return result;
        out_report->expected_mask |= 1u << i;
        ++out_report->refresh_frames_sent;
    }
    for (;;) {
        oa_can_frame frame;
        uint32_t matching = manifest->motor_count;
        oa_can_feedback feedback;
        result = transport->receive(transport->context, &frame);
        if (result == OA_CAN_ETIMEOUT) break;
        if (result != OA_CAN_OK) return result;
        for (i = 0u; i < manifest->motor_count; ++i) {
            if (frame.can_id == manifest->motors[i].receive_id) {
                matching = i;
                break;
            }
        }
        if (matching == manifest->motor_count) {
            ++out_report->unexpected_frames;
            continue;
        }
        result = oa_can_decode_feedback(&frame, manifest->motors[matching].receive_id,
                                        manifest->motors[matching].expected_motor_id,
                                        manifest->motors[matching].motor_type, &feedback);
        if (result == OA_CAN_EFRAME || result == OA_CAN_EID || result == OA_CAN_EINVAL) {
            ++out_report->invalid_frames;
            continue;
        }
        if ((out_report->fresh_mask & (1u << matching)) != 0u) {
            out_report->duplicate_mask |= 1u << matching;
        }
        out_report->fresh_mask |= 1u << matching;
        if (result == OA_CAN_EFAULT) out_report->fault_mask |= 1u << matching;
    }
    if (out_report->fresh_mask != out_report->expected_mask) return OA_CAN_ETIMEOUT;
    if (out_report->fault_mask != 0u || out_report->duplicate_mask != 0u) return OA_CAN_EFAULT;
    return OA_CAN_OK;
}

static oa_can_status oa_fake_send(void *context, const oa_can_frame *frame) {
    oa_can_fake *fake = (oa_can_fake *)context;
    if (fake == NULL || !oa_valid_frame(frame)) return OA_CAN_EINVAL;
    ++fake->sent_count;
    if (frame->can_id == 0x7ffu && frame->dlc == 8u && frame->data[2] == 0xccu) {
        fake->lifecycle = OA_CAN_FAKE_PROBING;
    }
    return OA_CAN_OK;
}

static oa_can_status oa_fake_receive(void *context, oa_can_frame *frame) {
    oa_can_fake *fake = (oa_can_fake *)context;
    if (fake == NULL || frame == NULL) return OA_CAN_EINVAL;
    if (fake->count == 0u) {
        if (fake->lifecycle == OA_CAN_FAKE_PROBING) fake->lifecycle = OA_CAN_FAKE_PROBED;
        return OA_CAN_ETIMEOUT;
    }
    *frame = fake->queue[fake->head];
    fake->head = (fake->head + 1u) % fake->capacity;
    --fake->count;
    return OA_CAN_OK;
}

oa_can_status oa_can_fake_create(size_t queue_capacity, oa_can_fake **out_fake) {
    oa_can_fake *fake;
    if (out_fake == NULL || queue_capacity == 0u || queue_capacity > SIZE_MAX / sizeof(*fake->queue)) return OA_CAN_EINVAL;
    fake = (oa_can_fake *)calloc(1u, sizeof(*fake));
    if (fake == NULL) return OA_CAN_ENOMEM;
    fake->queue = (oa_can_frame *)calloc(queue_capacity, sizeof(*fake->queue));
    if (fake->queue == NULL) {
        free(fake);
        return OA_CAN_ENOMEM;
    }
    fake->capacity = queue_capacity;
    fake->lifecycle = OA_CAN_FAKE_DISABLED;
    *out_fake = fake;
    return OA_CAN_OK;
}

void oa_can_fake_destroy(oa_can_fake *fake) {
    if (fake != NULL) {
        free(fake->queue);
        free(fake);
    }
}

oa_can_status oa_can_fake_transport(oa_can_fake *fake, oa_can_transport *out_transport) {
    if (fake == NULL || out_transport == NULL) return OA_CAN_EINVAL;
    (void)memset(out_transport, 0, sizeof(*out_transport));
    out_transport->struct_size = (uint32_t)sizeof(*out_transport);
    out_transport->abi_version = OA_CAN_ABI_VERSION;
    out_transport->context = fake;
    out_transport->send = oa_fake_send;
    out_transport->receive = oa_fake_receive;
    return OA_CAN_OK;
}

oa_can_status oa_can_fake_enqueue_feedback(oa_can_fake *fake, const oa_can_frame *frame) {
    size_t index;
    if (fake == NULL || !oa_valid_frame(frame)) return OA_CAN_EINVAL;
    if (fake->count == fake->capacity) return OA_CAN_EIO;
    index = (fake->head + fake->count) % fake->capacity;
    fake->queue[index] = *frame;
    ++fake->count;
    return OA_CAN_OK;
}

oa_can_fake_lifecycle oa_can_fake_get_lifecycle(const oa_can_fake *fake) {
    return fake == NULL ? OA_CAN_FAKE_FAULT : fake->lifecycle;
}

size_t oa_can_fake_sent_count(const oa_can_fake *fake) { return fake == NULL ? 0u : fake->sent_count; }
int oa_can_fake_torque_enabled(const oa_can_fake *fake) { (void)fake; return 0; }
