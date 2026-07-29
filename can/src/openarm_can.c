/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_can.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct oa_can_header {
    uint32_t struct_size;
    uint32_t abi_version;
} oa_can_header;

struct oa_can_fake {
    oa_can_received_frame *queue;
    size_t capacity;
    size_t head;
    size_t count;
    size_t sent_count;
    size_t rejected_control_count;
    uint64_t now_ns;
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
    return (size_t)struct_size >= required && abi_version == OA_CAN_ABI_VERSION;
}

static int oa_valid_output(const void *output, size_t required) {
    oa_can_header header;
    if (output == NULL) return 0;
    (void)memcpy(&header, output, sizeof(header));
    return oa_valid_version(header.struct_size, required, header.abi_version);
}

static void oa_clear_output(void *output, size_t size) {
    oa_can_header header;
    (void)memset(output, 0, size);
    header.struct_size = (uint32_t)size;
    header.abi_version = OA_CAN_ABI_VERSION;
    (void)memcpy(output, &header, sizeof(header));
}

static void oa_seed_output(void *output, size_t size) {
    oa_can_header header;
    header.struct_size = (uint32_t)size;
    header.abi_version = OA_CAN_ABI_VERSION;
    (void)memcpy(output, &header, sizeof(header));
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

static int oa_binary32_available(void) {
    return sizeof(float) == 4u && FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
           FLT_MAX_EXP == 128;
}

static uint32_t oa_read_u32_le(const uint8_t *bytes) {
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8u) |
           ((uint32_t)bytes[2] << 16u) | ((uint32_t)bytes[3] << 24u);
}

static void oa_write_u32_le(uint32_t value, uint8_t *bytes) {
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8u);
    bytes[2] = (uint8_t)(value >> 16u);
    bytes[3] = (uint8_t)(value >> 24u);
}

static int oa_double_to_f32_bits(double value, uint32_t *out_bits) {
    float converted;
    if (out_bits == NULL || !oa_binary32_available() || !isfinite(value) ||
        value < -(double)FLT_MAX || value > (double)FLT_MAX) return 0;
    converted = (float)value;
    if (!isfinite((double)converted)) return 0;
    (void)memcpy(out_bits, &converted, sizeof(*out_bits));
    return 1;
}

static int oa_clean_standard_frame(const oa_can_frame *frame, uint8_t expected_dlc) {
    return oa_valid_frame(frame) &&
           (frame->can_id & (OA_CAN_EFF_FLAG | OA_CAN_RTR_FLAG | OA_CAN_ERR_FLAG)) == 0u &&
           frame->can_id <= OA_CAN_SFF_MASK && frame->dlc == expected_dlc;
}

static int oa_register_metadata(oa_can_register_id register_id,
                                oa_can_register_value_type *value_type,
                                uint32_t *writable) {
    int known = register_id <= OA_CAN_RID_SUB_VER ||
                (register_id >= OA_CAN_RID_U_OFF && register_id <= OA_CAN_RID_DIRECTION) ||
                (register_id >= OA_CAN_RID_MOTOR_POSITION &&
                 register_id <= OA_CAN_RID_OUTPUT_POSITION);
    if (!known) return 0;
    if ((register_id >= OA_CAN_RID_MST_ID && register_id <= OA_CAN_RID_CTRL_MODE) ||
        (register_id >= OA_CAN_RID_HW_VER && register_id <= OA_CAN_RID_NPP) ||
        register_id == OA_CAN_RID_CAN_BR || register_id == OA_CAN_RID_SUB_VER) {
        *value_type = OA_CAN_REGISTER_U32;
    } else {
        *value_type = OA_CAN_REGISTER_F32;
    }
    *writable = register_id <= OA_CAN_RID_CTRL_MODE ||
                (register_id >= OA_CAN_RID_PMAX && register_id <= OA_CAN_RID_CAN_BR);
    return 1;
}

static oa_can_status oa_validate_register_u32(oa_can_register_id register_id,
                                               uint32_t value) {
    if ((register_id == OA_CAN_RID_MST_ID || register_id == OA_CAN_RID_ESC_ID) &&
        value > OA_CAN_SFF_MASK) return OA_CAN_ERANGE;
    if (register_id == OA_CAN_RID_CTRL_MODE && (value < 1u || value > 4u)) {
        return OA_CAN_ERANGE;
    }
    if (register_id == OA_CAN_RID_CAN_BR && value > 9u) return OA_CAN_ERANGE;
    return OA_CAN_OK;
}

static oa_can_status oa_validate_register_f32(oa_can_register_id register_id,
                                               float value) {
    double v = (double)value;
    if (!isfinite(v)) return OA_CAN_EINVAL;
    if ((register_id == OA_CAN_RID_UV_VALUE && v <= 10.0) ||
        (register_id == OA_CAN_RID_KT_VALUE && v < 0.0) ||
        (register_id == OA_CAN_RID_OT_VALUE && (v < 80.0 || v >= 200.0)) ||
        (register_id == OA_CAN_RID_OC_VALUE && (v <= 0.0 || v >= 1.0)) ||
        (register_id == OA_CAN_RID_ACC && v <= 0.0) ||
        (register_id == OA_CAN_RID_DEC && v >= 0.0) ||
        (register_id == OA_CAN_RID_MAX_SPD && v <= 0.0) ||
        ((register_id == OA_CAN_RID_PMAX || register_id == OA_CAN_RID_VMAX ||
          register_id == OA_CAN_RID_TMAX) && v <= 0.0) ||
        (register_id == OA_CAN_RID_I_BW && (v < 100.0 || v > 10000.0)) ||
        ((register_id >= OA_CAN_RID_KP_ASR && register_id <= OA_CAN_RID_KI_APR) &&
         v < 0.0) ||
        (register_id == OA_CAN_RID_GREF && (v <= 0.0 || v > 1.0)) ||
        (register_id == OA_CAN_RID_DETA && (v < 1.0 || v > 30.0)) ||
        (register_id == OA_CAN_RID_V_BW && (v <= 0.0 || v >= 500.0)) ||
        (register_id == OA_CAN_RID_IQ_C1 && (v < 100.0 || v > 10000.0)) ||
        (register_id == OA_CAN_RID_VL_C1 && (v <= 0.0 || v > 10000.0))) {
        return OA_CAN_ERANGE;
    }
    return OA_CAN_OK;
}

static int oa_valid_profile(const oa_can_mit_profile *profile) {
    return profile != NULL &&
           oa_valid_version(profile->struct_size, sizeof(*profile), profile->abi_version) &&
           oa_valid_standard_id(profile->target_send_id) &&
           oa_valid_standard_id(profile->receive_id) &&
           profile->verified_mask == OA_CAN_PROFILE_ALL_VERIFIED &&
           isfinite(profile->pmax_rad) && profile->pmax_rad > 0.0 &&
           profile->pmax_rad <= (double)FLT_MAX && isfinite(profile->vmax_rad_s) &&
           profile->vmax_rad_s > 0.0 && profile->vmax_rad_s <= (double)FLT_MAX &&
           isfinite(profile->tmax_nm) && profile->tmax_nm > 0.0 &&
           profile->tmax_nm <= (double)FLT_MAX;
}

static void oa_fill_frame(oa_can_frame *frame, uint16_t id) {
    oa_clear_output(frame, sizeof(*frame));
    frame->can_id = id;
    frame->dlc = 8u;
}

static double oa_from_uint(uint32_t value, double lower, double upper, unsigned int bits) {
    const double maximum = (double)((UINT32_C(1) << bits) - UINT32_C(1));
    return ((double)value / maximum) * (upper - lower) + lower;
}

static uint32_t oa_to_uint(double value, double lower, double upper, unsigned int bits) {
    const double maximum = (double)((UINT32_C(1) << bits) - UINT32_C(1));
    return (uint32_t)((value - lower) * maximum / (upper - lower));
}

static oa_can_status oa_limit_value(double input, double lower, double upper,
                                    oa_can_range_policy policy, uint32_t bit,
                                    uint32_t *saturated_mask, double *out_value) {
    if (!isfinite(input)) return OA_CAN_EINVAL;
    if (input < lower || input > upper) {
        if (policy != OA_CAN_RANGE_SATURATE) return OA_CAN_ERANGE;
        *saturated_mask |= bit;
        *out_value = input < lower ? lower : upper;
        return OA_CAN_OK;
    }
    *out_value = input;
    return OA_CAN_OK;
}

oa_can_status oa_can_motor_limits(oa_can_motor_type motor_type, oa_can_limits *out_limits) {
    if (!oa_valid_type(motor_type) || !oa_valid_output(out_limits, sizeof(*out_limits))) {
        return OA_CAN_EINVAL;
    }
    oa_clear_output(out_limits, sizeof(*out_limits));
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

static oa_can_status oa_encode_mit_values(uint16_t send_id, double position_input,
                                          double velocity_input, double kp_input,
                                          double kd_input, double torque_input,
                                          double pmax, double vmax, double tmax,
                                          oa_can_range_policy policy,
                                          oa_can_frame *out_frame,
                                          oa_can_encode_result *out_result) {
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
    if (!oa_valid_standard_id(send_id) || !oa_valid_output(out_frame, sizeof(*out_frame)) ||
        !oa_valid_output(out_result, sizeof(*out_result)) ||
        !isfinite(pmax) || pmax <= 0.0 || !isfinite(vmax) || vmax <= 0.0 ||
        !isfinite(tmax) || tmax <= 0.0 ||
        (policy != OA_CAN_RANGE_REJECT && policy != OA_CAN_RANGE_SATURATE)) {
        return OA_CAN_EINVAL;
    }
    status = oa_limit_value(position_input, -pmax, pmax,
                            policy, OA_FIELD_POSITION, &mask, &position);
    if (status != OA_CAN_OK) return status;
    status = oa_limit_value(velocity_input, -vmax, vmax,
                            policy, OA_FIELD_VELOCITY, &mask, &velocity);
    if (status != OA_CAN_OK) return status;
    status = oa_limit_value(kp_input, 0.0, 500.0, policy, OA_FIELD_KP, &mask, &kp);
    if (status != OA_CAN_OK) return status;
    status = oa_limit_value(kd_input, 0.0, 5.0, policy, OA_FIELD_KD, &mask, &kd);
    if (status != OA_CAN_OK) return status;
    status = oa_limit_value(torque_input, -tmax, tmax,
                            policy, OA_FIELD_TORQUE, &mask, &torque);
    if (status != OA_CAN_OK) return status;
    position_raw = oa_to_uint(position, -pmax, pmax, 16u);
    velocity_raw = oa_to_uint(velocity, -vmax, vmax, 12u);
    kp_raw = oa_to_uint(kp, 0.0, 500.0, 12u);
    kd_raw = oa_to_uint(kd, 0.0, 5.0, 12u);
    torque_raw = oa_to_uint(torque, -tmax, tmax, 12u);
    oa_fill_frame(out_frame, send_id);
    out_frame->data[0] = (uint8_t)(position_raw >> 8u);
    out_frame->data[1] = (uint8_t)position_raw;
    out_frame->data[2] = (uint8_t)(velocity_raw >> 4u);
    out_frame->data[3] = (uint8_t)((velocity_raw << 4u) | (kp_raw >> 8u));
    out_frame->data[4] = (uint8_t)kp_raw;
    out_frame->data[5] = (uint8_t)(kd_raw >> 4u);
    out_frame->data[6] = (uint8_t)((kd_raw << 4u) | (torque_raw >> 8u));
    out_frame->data[7] = (uint8_t)torque_raw;
    oa_clear_output(out_result, sizeof(*out_result));
    out_result->saturated_mask = mask;
    return OA_CAN_OK;
}

oa_can_status oa_can_encode_mit(const oa_can_mit_command *command,
                                oa_can_range_policy policy,
                                oa_can_frame *out_frame,
                                oa_can_encode_result *out_result) {
    oa_can_limits limits;
    oa_can_status status;
    if (command == NULL ||
        !oa_valid_version(command->struct_size, sizeof(*command), command->abi_version) ||
        !oa_valid_type(command->motor_type)) return OA_CAN_EINVAL;
    oa_seed_output(&limits, sizeof(limits));
    status = oa_can_motor_limits(command->motor_type, &limits);
    if (status != OA_CAN_OK) return status;
    return oa_encode_mit_values(command->send_id, command->position_rad,
                                command->velocity_rad_s, command->kp, command->kd,
                                command->torque_nm, limits.position_max_rad,
                                limits.velocity_max_rad_s, limits.torque_max_nm,
                                policy, out_frame, out_result);
}

oa_can_status oa_can_encode_mit_profile(const oa_can_mit_profile_command *command,
                                        const oa_can_mit_profile *profile,
                                        oa_can_frame *out_frame) {
    oa_can_encode_result result;
    if (command == NULL ||
        !oa_valid_version(command->struct_size, sizeof(*command), command->abi_version) ||
        command->reserved != 0u || !oa_valid_profile(profile) ||
        command->send_id != profile->target_send_id ||
        !oa_valid_output(out_frame, sizeof(*out_frame))) return OA_CAN_EINVAL;
    oa_seed_output(&result, sizeof(result));
    return oa_encode_mit_values(command->send_id, command->position_rad,
                                command->velocity_rad_s, command->kp, command->kd,
                                command->torque_nm, profile->pmax_rad,
                                profile->vmax_rad_s, profile->tmax_nm,
                                OA_CAN_RANGE_REJECT, out_frame, &result);
}

static int oa_known_feedback_status(uint8_t status) {
    return status == OA_CAN_FEEDBACK_DISABLED || status == OA_CAN_FEEDBACK_ENABLED ||
           (status >= OA_CAN_FEEDBACK_OVER_VOLTAGE && status <= OA_CAN_FEEDBACK_OVERLOAD);
}

static oa_can_status oa_decode_feedback_ranges(const oa_can_frame *frame,
                                                uint16_t expected_receive_id,
                                                uint8_t expected_motor_id,
                                                double pmax, double vmax, double tmax,
                                                oa_can_feedback *out_feedback) {
    uint8_t motor_id;
    uint8_t status_nibble;
    uint32_t position_raw;
    uint32_t velocity_raw;
    uint32_t torque_raw;
    if (!oa_valid_output(out_feedback, sizeof(*out_feedback)) ||
        !oa_valid_standard_id(expected_receive_id) || expected_motor_id > 15u ||
        !isfinite(pmax) || pmax <= 0.0 || !isfinite(vmax) || vmax <= 0.0 ||
        !isfinite(tmax) || tmax <= 0.0) return OA_CAN_EINVAL;
    if (!oa_clean_standard_frame(frame, 8u)) return OA_CAN_EFRAME;
    if (frame->can_id != expected_receive_id) return OA_CAN_EID;
    motor_id = (uint8_t)(frame->data[0] & 0x0fu);
    status_nibble = (uint8_t)(frame->data[0] >> 4u);
    if (motor_id != expected_motor_id) return OA_CAN_EID;
    if (!oa_known_feedback_status(status_nibble)) return OA_CAN_EFRAME;
    position_raw = ((uint32_t)frame->data[1] << 8u) | frame->data[2];
    velocity_raw = ((uint32_t)frame->data[3] << 4u) | (frame->data[4] >> 4u);
    torque_raw = ((uint32_t)(frame->data[4] & 0x0fu) << 8u) | frame->data[5];
    oa_clear_output(out_feedback, sizeof(*out_feedback));
    out_feedback->receive_id = expected_receive_id;
    out_feedback->motor_id = motor_id;
    out_feedback->status_nibble = status_nibble;
    out_feedback->position_rad = oa_from_uint(position_raw, -pmax, pmax, 16u);
    out_feedback->velocity_rad_s = oa_from_uint(velocity_raw, -vmax, vmax, 12u);
    out_feedback->torque_nm = oa_from_uint(torque_raw, -tmax, tmax, 12u);
    out_feedback->mos_temperature_c = frame->data[6];
    out_feedback->rotor_temperature_c = frame->data[7];
    return (status_nibble == OA_CAN_FEEDBACK_ENABLED || status_nibble == OA_CAN_FEEDBACK_DISABLED) ?
           OA_CAN_OK : OA_CAN_EFAULT;
}

oa_can_status oa_can_decode_feedback(const oa_can_frame *frame,
                                     uint16_t expected_receive_id,
                                     uint8_t expected_motor_id,
                                     oa_can_motor_type motor_type,
                                     oa_can_feedback *out_feedback) {
    oa_can_limits limits;
    if (!oa_valid_type(motor_type)) return OA_CAN_EINVAL;
    oa_seed_output(&limits, sizeof(limits));
    if (oa_can_motor_limits(motor_type, &limits) != OA_CAN_OK) return OA_CAN_EINVAL;
    return oa_decode_feedback_ranges(frame, expected_receive_id, expected_motor_id,
                                     limits.position_max_rad, limits.velocity_max_rad_s,
                                     limits.torque_max_nm, out_feedback);
}

oa_can_status oa_can_decode_feedback_profile(const oa_can_frame *frame,
                                              uint16_t expected_receive_id,
                                              uint8_t expected_motor_id,
                                              const oa_can_mit_profile *profile,
                                              oa_can_feedback *out_feedback) {
    if (!oa_valid_profile(profile)) return OA_CAN_EINVAL;
    if (expected_receive_id != profile->receive_id ||
        expected_motor_id != (uint8_t)(profile->target_send_id & 0x0fu)) return OA_CAN_EID;
    return oa_decode_feedback_ranges(frame, expected_receive_id, expected_motor_id,
                                     profile->pmax_rad, profile->vmax_rad_s,
                                     profile->tmax_nm, out_feedback);
}

static oa_can_status oa_make_special(uint16_t send_id, uint8_t last_byte, oa_can_frame *out_frame) {
    if (!oa_valid_standard_id(send_id) || !oa_valid_output(out_frame, sizeof(*out_frame))) return OA_CAN_EINVAL;
    oa_fill_frame(out_frame, send_id);
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

oa_can_status oa_can_make_set_zero(uint16_t send_id, oa_can_frame *out_frame) {
    return oa_make_special(send_id, 0xfeu, out_frame);
}

oa_can_status oa_can_make_clear_error(uint16_t send_id, oa_can_frame *out_frame) {
    return oa_make_special(send_id, 0xfbu, out_frame);
}

oa_can_status oa_can_make_save_parameters(uint16_t send_id, oa_can_frame *out_frame) {
    if (!oa_valid_standard_id(send_id) || !oa_valid_output(out_frame, sizeof(*out_frame))) {
        return OA_CAN_EINVAL;
    }
    oa_fill_frame(out_frame, 0x7ffu);
    out_frame->data[0] = (uint8_t)send_id;
    out_frame->data[1] = (uint8_t)(send_id >> 8u);
    out_frame->data[2] = 0xaau;
    return OA_CAN_OK;
}

oa_can_status oa_can_make_refresh_status(uint16_t send_id, oa_can_frame *out_frame) {
    if (!oa_valid_standard_id(send_id) || !oa_valid_output(out_frame, sizeof(*out_frame))) return OA_CAN_EINVAL;
    oa_fill_frame(out_frame, 0x7ffu);
    out_frame->data[0] = (uint8_t)send_id;
    out_frame->data[1] = (uint8_t)(send_id >> 8u);
    out_frame->data[2] = 0xccu;
    return OA_CAN_OK;
}

oa_can_status oa_can_make_register_query(uint16_t send_id, uint8_t register_id, oa_can_frame *out_frame) {
    if (!oa_valid_standard_id(send_id) || !oa_valid_output(out_frame, sizeof(*out_frame))) return OA_CAN_EINVAL;
    oa_fill_frame(out_frame, 0x7ffu);
    out_frame->data[0] = (uint8_t)send_id;
    out_frame->data[1] = (uint8_t)(send_id >> 8u);
    out_frame->data[2] = 0x33u;
    out_frame->data[3] = register_id;
    return OA_CAN_OK;
}

oa_can_status oa_can_register_info_for_id(oa_can_register_id register_id,
                                           oa_can_register_info *out_info) {
    oa_can_register_value_type value_type;
    uint32_t writable;
    if (!oa_valid_output(out_info, sizeof(*out_info)) ||
        !oa_register_metadata(register_id, &value_type, &writable)) return OA_CAN_EINVAL;
    oa_clear_output(out_info, sizeof(*out_info));
    out_info->register_id = register_id;
    out_info->value_type = value_type;
    out_info->writable = writable;
    return OA_CAN_OK;
}

static oa_can_status oa_validate_register_request(const oa_can_register_request *request) {
    oa_can_register_value_type value_type;
    uint32_t writable;
    if (request == NULL ||
        !oa_valid_version(request->struct_size, sizeof(*request), request->abi_version) ||
        !oa_valid_standard_id(request->send_id) ||
        !oa_valid_standard_id(request->receive_id) ||
        !oa_register_metadata(request->register_id, &value_type, &writable) ||
        request->value_type != value_type) return OA_CAN_EINVAL;
    (void)writable;
    return OA_CAN_OK;
}

oa_can_status oa_can_make_register_query_typed(const oa_can_register_request *request,
                                                oa_can_frame *out_frame) {
    if (oa_validate_register_request(request) != OA_CAN_OK ||
        !oa_valid_output(out_frame, sizeof(*out_frame))) return OA_CAN_EINVAL;
    oa_fill_frame(out_frame, 0x7ffu);
    out_frame->data[0] = (uint8_t)request->send_id;
    out_frame->data[1] = (uint8_t)(request->send_id >> 8u);
    out_frame->data[2] = (uint8_t)OA_CAN_REGISTER_QUERY;
    out_frame->data[3] = (uint8_t)request->register_id;
    return OA_CAN_OK;
}

oa_can_status oa_can_make_register_write(const oa_can_register_write *write,
                                         oa_can_frame *out_frame) {
    oa_can_register_value_type value_type;
    uint32_t writable;
    uint32_t raw_value;
    oa_can_status status;
    if (write == NULL ||
        !oa_valid_version(write->struct_size, sizeof(*write), write->abi_version) ||
        !oa_valid_standard_id(write->send_id) || write->reserved != 0u ||
        !oa_valid_output(out_frame, sizeof(*out_frame)) || !oa_binary32_available() ||
        !oa_register_metadata(write->register_id, &value_type, &writable) || writable == 0u ||
        write->value_type != value_type) return OA_CAN_EINVAL;
    if (value_type == OA_CAN_REGISTER_U32) {
        if (write->value_f32 != 0.0f) return OA_CAN_EINVAL;
        status = oa_validate_register_u32(write->register_id, write->value_u32);
        raw_value = write->value_u32;
    } else {
        if (write->value_u32 != 0u) return OA_CAN_EINVAL;
        status = oa_validate_register_f32(write->register_id, write->value_f32);
        (void)memcpy(&raw_value, &write->value_f32, sizeof(raw_value));
    }
    if (status != OA_CAN_OK) return status;
    oa_fill_frame(out_frame, 0x7ffu);
    out_frame->data[0] = (uint8_t)write->send_id;
    out_frame->data[1] = (uint8_t)(write->send_id >> 8u);
    out_frame->data[2] = (uint8_t)OA_CAN_REGISTER_WRITE;
    out_frame->data[3] = (uint8_t)write->register_id;
    oa_write_u32_le(raw_value, &out_frame->data[4]);
    return OA_CAN_OK;
}

oa_can_status oa_can_decode_register_response(const oa_can_frame *frame,
                                               const oa_can_register_request *expected,
                                               oa_can_register_operation expected_operation,
                                               oa_can_register_value *out_value) {
    uint16_t target_send_id;
    uint32_t raw_value;
    uint32_t value_u32 = 0u;
    float value_f32 = 0.0f;
    oa_can_status status;
    if (oa_validate_register_request(expected) != OA_CAN_OK ||
        !oa_valid_output(out_value, sizeof(*out_value)) || !oa_binary32_available() ||
        (expected_operation != OA_CAN_REGISTER_QUERY &&
         expected_operation != OA_CAN_REGISTER_WRITE)) return OA_CAN_EINVAL;
    if (!oa_clean_standard_frame(frame, 8u)) return OA_CAN_EFRAME;
    if (frame->can_id != expected->receive_id) return OA_CAN_EID;
    target_send_id = (uint16_t)((uint16_t)frame->data[0] |
                                ((uint16_t)frame->data[1] << 8u));
    if (target_send_id != expected->send_id) return OA_CAN_EID;
    if ((oa_can_register_operation)frame->data[2] != expected_operation ||
        (oa_can_register_id)frame->data[3] != expected->register_id) return OA_CAN_EFRAME;
    raw_value = oa_read_u32_le(&frame->data[4]);
    if (expected->value_type == OA_CAN_REGISTER_U32) {
        status = oa_validate_register_u32(expected->register_id, raw_value);
        value_u32 = raw_value;
    } else {
        (void)memcpy(&value_f32, &raw_value, sizeof(value_f32));
        status = oa_validate_register_f32(expected->register_id, value_f32);
    }
    if (status != OA_CAN_OK) return status;
    oa_clear_output(out_value, sizeof(*out_value));
    out_value->receive_id = expected->receive_id;
    out_value->target_send_id = expected->send_id;
    out_value->register_id = expected->register_id;
    out_value->value_type = expected->value_type;
    out_value->operation = expected_operation;
    out_value->raw_value = raw_value;
    out_value->value_u32 = value_u32;
    out_value->value_f32 = value_f32;
    return OA_CAN_OK;
}

static int oa_valid_profile_register(const oa_can_register_value *value,
                                     oa_can_register_id expected_register) {
    uint32_t raw_value;
    if (value == NULL ||
        !oa_valid_version(value->struct_size, sizeof(*value), value->abi_version) ||
        !oa_valid_standard_id(value->receive_id) ||
        !oa_valid_standard_id(value->target_send_id) ||
        value->register_id != expected_register ||
        value->value_type != OA_CAN_REGISTER_F32 ||
        value->operation != OA_CAN_REGISTER_QUERY || value->value_u32 != 0u ||
        oa_validate_register_f32(expected_register, value->value_f32) != OA_CAN_OK) return 0;
    (void)memcpy(&raw_value, &value->value_f32, sizeof(raw_value));
    return raw_value == value->raw_value;
}

oa_can_status oa_can_mit_profile_from_registers(const oa_can_register_value *pmax,
                                                const oa_can_register_value *vmax,
                                                const oa_can_register_value *tmax,
                                                oa_can_mit_profile *out_profile) {
    if (!oa_valid_output(out_profile, sizeof(*out_profile)) ||
        !oa_valid_profile_register(pmax, OA_CAN_RID_PMAX) ||
        !oa_valid_profile_register(vmax, OA_CAN_RID_VMAX) ||
        !oa_valid_profile_register(tmax, OA_CAN_RID_TMAX) ||
        pmax->receive_id != vmax->receive_id || pmax->receive_id != tmax->receive_id ||
        pmax->target_send_id != vmax->target_send_id ||
        pmax->target_send_id != tmax->target_send_id) return OA_CAN_EINVAL;
    oa_clear_output(out_profile, sizeof(*out_profile));
    out_profile->target_send_id = pmax->target_send_id;
    out_profile->receive_id = pmax->receive_id;
    out_profile->pmax_rad = (double)pmax->value_f32;
    out_profile->vmax_rad_s = (double)vmax->value_f32;
    out_profile->tmax_nm = (double)tmax->value_f32;
    out_profile->verified_mask = OA_CAN_PROFILE_ALL_VERIFIED;
    return OA_CAN_OK;
}

oa_can_status oa_can_encode_pos_vel(const oa_can_pos_vel_command *command,
                                    const oa_can_mit_profile *profile,
                                    oa_can_frame *out_frame) {
    uint32_t position_bits;
    uint32_t velocity_bits;
    if (command == NULL ||
        !oa_valid_version(command->struct_size, sizeof(*command), command->abi_version) ||
        command->reserved != 0u || !oa_valid_profile(profile) ||
        command->send_id != profile->target_send_id ||
        !oa_valid_standard_id(command->send_id) || command->send_id > 0x6ffu ||
        !oa_valid_output(out_frame, sizeof(*out_frame))) return OA_CAN_EINVAL;
    if (!isfinite(command->position_rad) ||
        !isfinite(command->max_velocity_rad_s)) return OA_CAN_EINVAL;
    if (command->position_rad < -profile->pmax_rad ||
        command->position_rad > profile->pmax_rad ||
        command->max_velocity_rad_s < 0.0 ||
        command->max_velocity_rad_s > profile->vmax_rad_s) return OA_CAN_ERANGE;
    if (!oa_double_to_f32_bits(command->position_rad, &position_bits) ||
        !oa_double_to_f32_bits(command->max_velocity_rad_s, &velocity_bits)) return OA_CAN_EINVAL;
    oa_fill_frame(out_frame, (uint16_t)(command->send_id + 0x100u));
    oa_write_u32_le(position_bits, &out_frame->data[0]);
    oa_write_u32_le(velocity_bits, &out_frame->data[4]);
    return OA_CAN_OK;
}

oa_can_status oa_can_encode_pos_force(const oa_can_pos_force_command *command,
                                      const oa_can_mit_profile *profile,
                                      oa_can_frame *out_frame) {
    uint32_t position_bits;
    uint16_t velocity_scaled;
    uint16_t current_scaled;
    if (command == NULL ||
        !oa_valid_version(command->struct_size, sizeof(*command), command->abi_version) ||
        command->reserved != 0u || !oa_valid_profile(profile) ||
        command->send_id != profile->target_send_id ||
        !oa_valid_standard_id(command->send_id) || command->send_id > 0x4ffu ||
        !oa_valid_output(out_frame, sizeof(*out_frame))) return OA_CAN_EINVAL;
    if (!isfinite(command->position_rad) || !isfinite(command->max_velocity_rad_s) ||
        !isfinite(command->current_limit_per_unit)) return OA_CAN_EINVAL;
    if (command->position_rad < -profile->pmax_rad ||
        command->position_rad > profile->pmax_rad ||
        command->max_velocity_rad_s < 0.0 ||
        command->max_velocity_rad_s > profile->vmax_rad_s ||
        command->max_velocity_rad_s > 100.0 ||
        command->current_limit_per_unit < 0.0 ||
        command->current_limit_per_unit > 1.0) return OA_CAN_ERANGE;
    if (!oa_double_to_f32_bits(command->position_rad, &position_bits)) return OA_CAN_EINVAL;
    velocity_scaled = (uint16_t)(command->max_velocity_rad_s * 100.0);
    current_scaled = (uint16_t)(command->current_limit_per_unit * 10000.0);
    oa_fill_frame(out_frame, (uint16_t)(command->send_id + 0x300u));
    oa_write_u32_le(position_bits, &out_frame->data[0]);
    out_frame->data[4] = (uint8_t)velocity_scaled;
    out_frame->data[5] = (uint8_t)(velocity_scaled >> 8u);
    out_frame->data[6] = (uint8_t)current_scaled;
    out_frame->data[7] = (uint8_t)(current_scaled >> 8u);
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
            !oa_valid_type(motor->decode_motor_type) || !isfinite(mapping->position_scale) ||
            !isfinite(mapping->position_offset_rad) || !isfinite(mapping->velocity_scale) ||
            !isfinite(mapping->torque_scale) || mapping->position_scale == 0.0 ||
            mapping->velocity_scale == 0.0 || mapping->torque_scale == 0.0 || motor->joint_name[0] == '\0' ||
            memchr(motor->joint_name, '\0', sizeof(motor->joint_name)) == NULL ||
            memchr(motor->commissioned_serial, '\0', sizeof(motor->commissioned_serial)) == NULL) {
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
                                    const oa_can_probe_options *options,
                                    oa_can_probe_report *out_report) {
    uint64_t request_epochs[OA_CAN_MAX_MOTORS];
    uint64_t now_ns;
    uint32_t i;
    oa_can_status result;
    if (transport == NULL || options == NULL ||
        !oa_valid_version(transport->struct_size, sizeof(*transport), transport->abi_version) ||
        !oa_valid_version(options->struct_size, sizeof(*options), options->abi_version) ||
        !oa_valid_output(out_report, sizeof(*out_report)) || transport->now == NULL ||
        transport->send == NULL || transport->receive == NULL || options->timeout_ns == 0u ||
        options->max_receive_frames == 0u) return OA_CAN_EINVAL;
    result = oa_can_validate_manifest(manifest);
    if (result != OA_CAN_OK) return result;
    oa_clear_output(out_report, sizeof(*out_report));
    for (i = 0u; i < manifest->motor_count; ++i) {
        oa_can_frame refresh;
        oa_seed_output(&refresh, sizeof(refresh));
        result = oa_can_make_refresh_status(manifest->motors[i].send_id, &refresh);
        if (result != OA_CAN_OK) return result;
        result = transport->send(transport->context, &refresh, &request_epochs[i]);
        if (result != OA_CAN_OK) return result;
        out_report->expected_mask |= UINT32_C(1) << i;
        ++out_report->refresh_frames_sent;
    }
    result = transport->now(transport->context, &now_ns);
    if (result != OA_CAN_OK || UINT64_MAX - now_ns < options->timeout_ns) return OA_CAN_EIO;
    for (i = 0u; i < manifest->motor_count; ++i) {
        if (request_epochs[i] > now_ns) return OA_CAN_EIO;
    }
    out_report->deadline_monotonic_ns = now_ns + options->timeout_ns;
    for (i = 0u; i < options->max_receive_frames; ++i) {
        oa_can_received_frame received;
        oa_can_feedback feedback;
        uint32_t matching = manifest->motor_count;
        uint32_t motor_index;
        oa_seed_output(&received, sizeof(received));
        oa_seed_output(&received.frame, sizeof(received.frame));
        result = transport->receive(transport->context, out_report->deadline_monotonic_ns, &received);
        if (result == OA_CAN_ETIMEOUT) break;
        if (result != OA_CAN_OK) return result;
        ++out_report->received_frames;
        if (!oa_valid_frame(&received.frame) ||
            received.received_monotonic_ns > out_report->deadline_monotonic_ns) {
            ++out_report->invalid_frames;
            continue;
        }
        for (motor_index = 0u; motor_index < manifest->motor_count; ++motor_index) {
            if (received.frame.can_id == manifest->motors[motor_index].receive_id) {
                matching = motor_index;
                break;
            }
        }
        if (matching == manifest->motor_count) {
            ++out_report->unexpected_frames;
            continue;
        }
        if (received.received_monotonic_ns <= request_epochs[matching]) {
            ++out_report->stale_frames;
            continue;
        }
        oa_seed_output(&feedback, sizeof(feedback));
        result = oa_can_decode_feedback(&received.frame, manifest->motors[matching].receive_id,
                                        manifest->motors[matching].expected_motor_id,
                                        manifest->motors[matching].decode_motor_type, &feedback);
        if (result == OA_CAN_EFRAME || result == OA_CAN_EID || result == OA_CAN_EINVAL) {
            ++out_report->invalid_frames;
            continue;
        }
        if (feedback.status_nibble == OA_CAN_FEEDBACK_ENABLED) {
            out_report->enabled_mask |= UINT32_C(1) << matching;
            continue;
        }
        if (result == OA_CAN_EFAULT) {
            out_report->fault_mask |= UINT32_C(1) << matching;
            continue;
        }
        if ((out_report->fresh_mask & (UINT32_C(1) << matching)) != 0u) {
            out_report->duplicate_mask |= UINT32_C(1) << matching;
        }
        out_report->fresh_mask |= UINT32_C(1) << matching;
    }
    if (i == options->max_receive_frames) out_report->receive_limit_reached = 1u;
    if (out_report->enabled_mask != 0u) return OA_CAN_ESTATE;
    if (out_report->fault_mask != 0u || out_report->duplicate_mask != 0u) return OA_CAN_EFAULT;
    if (out_report->receive_limit_reached != 0u) return OA_CAN_ETIMEOUT;
    if (out_report->fresh_mask != out_report->expected_mask || out_report->stale_frames != 0u) return OA_CAN_ETIMEOUT;
    return OA_CAN_OK;
}

static int oa_frame_is_special(const oa_can_frame *frame, uint8_t last_byte) {
    size_t i;
    if (frame->dlc != 8u || frame->can_id > OA_CAN_SFF_MASK) return 0;
    for (i = 0u; i < 7u; ++i) {
        if (frame->data[i] != 0xffu) return 0;
    }
    return frame->data[7] == last_byte;
}

static int oa_frame_is_diagnostic(const oa_can_frame *frame) {
    uint16_t send_id;
    size_t first_zero;
    size_t i;
    if (frame->can_id != 0x7ffu || frame->dlc != 8u) return 0;
    send_id = (uint16_t)((uint16_t)frame->data[0] | ((uint16_t)frame->data[1] << 8u));
    if (!oa_valid_standard_id(send_id)) return 0;
    if (frame->data[2] == 0xccu) first_zero = 3u;
    else if (frame->data[2] == 0x33u) first_zero = 4u;
    else return 0;
    for (i = first_zero; i < sizeof(frame->data); ++i) {
        if (frame->data[i] != 0u) return 0;
    }
    return 1;
}

static oa_can_status oa_fake_now(void *context, uint64_t *out_monotonic_ns) {
    oa_can_fake *fake = (oa_can_fake *)context;
    if (fake == NULL || out_monotonic_ns == NULL) return OA_CAN_EINVAL;
    *out_monotonic_ns = fake->now_ns;
    return OA_CAN_OK;
}

static oa_can_status oa_fake_send(void *context, const oa_can_frame *frame,
                                  uint64_t *out_sent_monotonic_ns) {
    oa_can_fake *fake = (oa_can_fake *)context;
    if (fake == NULL || !oa_valid_frame(frame) || out_sent_monotonic_ns == NULL) return OA_CAN_EINVAL;
    if (fake->now_ns == UINT64_MAX) return OA_CAN_EIO;
    ++fake->now_ns;
    ++fake->sent_count;
    *out_sent_monotonic_ns = fake->now_ns;
    if (oa_frame_is_diagnostic(frame)) {
        if (frame->data[2] == 0xccu && fake->lifecycle != OA_CAN_FAKE_FAULT) {
            fake->lifecycle = OA_CAN_FAKE_PROBING;
        }
        return OA_CAN_OK;
    }
    if (oa_frame_is_special(frame, 0xfdu)) {
        if (fake->lifecycle != OA_CAN_FAKE_FAULT) fake->lifecycle = OA_CAN_FAKE_DISABLED;
        return OA_CAN_OK;
    }
    ++fake->rejected_control_count;
    fake->lifecycle = OA_CAN_FAKE_FAULT;
    return OA_CAN_ESTATE;
}

static oa_can_status oa_fake_receive(void *context, uint64_t deadline_monotonic_ns,
                                     oa_can_received_frame *out_frame) {
    oa_can_fake *fake = (oa_can_fake *)context;
    const oa_can_received_frame *queued;
    if (fake == NULL || !oa_valid_output(out_frame, sizeof(*out_frame)) ||
        !oa_valid_output(&out_frame->frame, sizeof(out_frame->frame))) return OA_CAN_EINVAL;
    if (fake->count == 0u) {
        if (fake->lifecycle == OA_CAN_FAKE_PROBING) fake->lifecycle = OA_CAN_FAKE_PROBED;
        if (fake->now_ns < deadline_monotonic_ns) fake->now_ns = deadline_monotonic_ns;
        return OA_CAN_ETIMEOUT;
    }
    queued = &fake->queue[fake->head];
    if (queued->received_monotonic_ns > deadline_monotonic_ns) {
        if (fake->now_ns < deadline_monotonic_ns) fake->now_ns = deadline_monotonic_ns;
        return OA_CAN_ETIMEOUT;
    }
    *out_frame = *queued;
    fake->head = (fake->head + 1u) % fake->capacity;
    --fake->count;
    if (fake->count == 0u && fake->lifecycle == OA_CAN_FAKE_PROBING) {
        fake->lifecycle = OA_CAN_FAKE_PROBED;
    }
    if (fake->now_ns < out_frame->received_monotonic_ns) fake->now_ns = out_frame->received_monotonic_ns;
    return OA_CAN_OK;
}

oa_can_status oa_can_fake_create(size_t queue_capacity, oa_can_fake **out_fake) {
    oa_can_fake *fake;
    if (out_fake == NULL || queue_capacity == 0u || queue_capacity > SIZE_MAX / sizeof(*fake->queue)) {
        return OA_CAN_EINVAL;
    }
    fake = (oa_can_fake *)calloc(1u, sizeof(*fake));
    if (fake == NULL) return OA_CAN_ENOMEM;
    fake->queue = (oa_can_received_frame *)calloc(queue_capacity, sizeof(*fake->queue));
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
    if (fake == NULL || !oa_valid_output(out_transport, sizeof(*out_transport))) return OA_CAN_EINVAL;
    oa_clear_output(out_transport, sizeof(*out_transport));
    out_transport->context = fake;
    out_transport->now = oa_fake_now;
    out_transport->send = oa_fake_send;
    out_transport->receive = oa_fake_receive;
    return OA_CAN_OK;
}

oa_can_status oa_can_fake_set_time(oa_can_fake *fake, uint64_t monotonic_ns) {
    if (fake == NULL || monotonic_ns < fake->now_ns) return OA_CAN_EINVAL;
    fake->now_ns = monotonic_ns;
    return OA_CAN_OK;
}

oa_can_status oa_can_fake_enqueue_feedback(oa_can_fake *fake, const oa_can_frame *frame,
                                           uint64_t received_monotonic_ns) {
    size_t index;
    oa_can_received_frame *queued;
    if (fake == NULL || !oa_valid_frame(frame)) return OA_CAN_EINVAL;
    if (fake->count == fake->capacity) return OA_CAN_EIO;
    index = (fake->head + fake->count) % fake->capacity;
    queued = &fake->queue[index];
    oa_clear_output(queued, sizeof(*queued));
    queued->received_monotonic_ns = received_monotonic_ns;
    queued->frame = *frame;
    ++fake->count;
    return OA_CAN_OK;
}

oa_can_fake_lifecycle oa_can_fake_get_lifecycle(const oa_can_fake *fake) {
    return fake == NULL ? OA_CAN_FAKE_FAULT : fake->lifecycle;
}

size_t oa_can_fake_sent_count(const oa_can_fake *fake) { return fake == NULL ? 0u : fake->sent_count; }
size_t oa_can_fake_rejected_control_count(const oa_can_fake *fake) {
    return fake == NULL ? 0u : fake->rejected_control_count;
}
int oa_can_fake_torque_enabled(const oa_can_fake *fake) { (void)fake; return 0; }
