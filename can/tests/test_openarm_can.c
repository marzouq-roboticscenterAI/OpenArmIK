/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_can.h"
#include "openarm_can_linux_internal.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#ifdef __linux__
#include <linux/can/netlink.h>
#include <linux/if.h>
#include <linux/if_link.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#endif

static int failures;

#define CHECK(condition) do { if (!(condition)) { \
    (void)fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); ++failures; } } while (0)

#define INITIALIZE(record) do { \
    (void)memset(&(record), 0, sizeof(record)); \
    (record).struct_size = (uint32_t)sizeof(record); \
    (record).abi_version = OA_CAN_ABI_VERSION; \
} while (0)

static oa_can_mit_command command_for(oa_can_motor_type type) {
    oa_can_mit_command command;
    INITIALIZE(command);
    command.send_id = 1u;
    command.motor_type = type;
    return command;
}

static oa_can_frame feedback_frame(uint16_t receive_id, uint8_t motor_id, uint8_t status,
                                   uint32_t position, uint32_t velocity, uint32_t torque) {
    oa_can_frame frame;
    INITIALIZE(frame);
    frame.can_id = receive_id;
    frame.dlc = 8u;
    frame.data[0] = (uint8_t)((uint8_t)(status << 4u) | motor_id);
    frame.data[1] = (uint8_t)(position >> 8u);
    frame.data[2] = (uint8_t)position;
    frame.data[3] = (uint8_t)(velocity >> 4u);
    frame.data[4] = (uint8_t)((velocity << 4u) | (torque >> 8u));
    frame.data[5] = (uint8_t)torque;
    frame.data[6] = 42u;
    frame.data[7] = 51u;
    return frame;
}

static uint32_t raw_float(float value) {
    uint32_t raw;
    (void)memcpy(&raw, &value, sizeof(raw));
    return raw;
}

static oa_can_frame register_frame(uint16_t receive_id, uint16_t send_id,
                                   oa_can_register_operation operation,
                                   oa_can_register_id register_id, uint32_t raw) {
    oa_can_frame frame;
    INITIALIZE(frame);
    frame.can_id = receive_id;
    frame.dlc = 8u;
    frame.data[0] = (uint8_t)send_id;
    frame.data[1] = (uint8_t)(send_id >> 8u);
    frame.data[2] = (uint8_t)operation;
    frame.data[3] = (uint8_t)register_id;
    frame.data[4] = (uint8_t)raw;
    frame.data[5] = (uint8_t)(raw >> 8u);
    frame.data[6] = (uint8_t)(raw >> 16u);
    frame.data[7] = (uint8_t)(raw >> 24u);
    return frame;
}

static oa_can_register_request register_request(oa_can_register_id register_id,
                                                oa_can_register_value_type value_type) {
    oa_can_register_request request;
    INITIALIZE(request);
    request.send_id = 0x123u;
    request.receive_id = 0x321u;
    request.register_id = register_id;
    request.value_type = value_type;
    return request;
}

static oa_can_mit_profile dynamic_profile(void) {
    oa_can_mit_profile profile;
    INITIALIZE(profile);
    profile.target_send_id = 1u;
    profile.receive_id = 0x11u;
    profile.pmax_rad = 20.0;
    profile.vmax_rad_s = 20.0;
    profile.tmax_nm = 30.0;
    profile.verified_mask = OA_CAN_PROFILE_ALL_VERIFIED;
    return profile;
}

static double lsb(double lower, double upper, unsigned int bits) {
    return (upper - lower) / (double)((UINT32_C(1) << bits) - UINT32_C(1));
}

static void test_golden_and_special_frames(void) {
    oa_can_mit_command command = command_for(OA_CAN_MOTOR_DM4310);
    oa_can_frame frame;
    oa_can_encode_result encoded;
    const uint8_t expected_midpoint[8] = {0x7fu, 0xffu, 0x7fu, 0xf7u, 0xffu, 0x7fu, 0xf7u, 0xffu};
    INITIALIZE(frame);
    INITIALIZE(encoded);
    command.position_rad = 0.0;
    command.velocity_rad_s = 0.0;
    command.kp = 250.0;
    command.kd = 2.5;
    command.torque_nm = 0.0;
    CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_REJECT, &frame, &encoded) == OA_CAN_OK);
    CHECK(frame.can_id == 1u && frame.dlc == 8u);
    CHECK(memcmp(frame.data, expected_midpoint, sizeof(expected_midpoint)) == 0);
    CHECK(encoded.saturated_mask == 0u);
    command.position_rad = -12.5;
    command.velocity_rad_s = -30.0;
    command.kp = 0.0;
    command.kd = 0.0;
    command.torque_nm = -10.0;
    CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_REJECT, &frame, &encoded) == OA_CAN_OK);
    CHECK(frame.data[0] == 0u && frame.data[1] == 0u && frame.data[2] == 0u && frame.data[7] == 0u);
    command.position_rad = 12.5;
    command.velocity_rad_s = 30.0;
    command.kp = 500.0;
    command.kd = 5.0;
    command.torque_nm = 10.0;
    CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_REJECT, &frame, &encoded) == OA_CAN_OK);
    CHECK(frame.data[0] == 0xffu && frame.data[1] == 0xffu && frame.data[2] == 0xffu &&
          frame.data[3] == 0xffu && frame.data[4] == 0xffu && frame.data[5] == 0xffu &&
          frame.data[6] == 0xffu && frame.data[7] == 0xffu);
    CHECK(oa_can_make_enable(7u, &frame) == OA_CAN_OK);
    CHECK(frame.can_id == 7u && frame.data[7] == 0xfcu && frame.data[0] == 0xffu);
    CHECK(oa_can_make_disable(7u, &frame) == OA_CAN_OK);
    CHECK(frame.can_id == 7u && frame.data[7] == 0xfdu && frame.data[0] == 0xffu);
    CHECK(oa_can_make_refresh_status(7u, &frame) == OA_CAN_OK);
    CHECK(frame.can_id == 0x7ffu && frame.data[0] == 7u && frame.data[1] == 0u && frame.data[2] == 0xccu);
    CHECK(oa_can_make_register_query(0x123u, 35u, &frame) == OA_CAN_OK);
    CHECK(frame.can_id == 0x7ffu && frame.data[0] == 0x23u && frame.data[1] == 1u &&
          frame.data[2] == 0x33u && frame.data[3] == 35u);
}

static void test_all_motor_golden_vectors(void) {
    struct golden_case {
        oa_can_motor_type type;
        double position;
        double velocity;
        double kp;
        double kd;
        double torque;
        uint8_t bytes[8];
    } cases[] = {
        {OA_CAN_MOTOR_DM8009, 1.25, 4.5, 100.0, 1.25, -5.4,
         {0x8cu, 0xccu, 0x8cu, 0xc3u, 0x33u, 0x3fu, 0xf7u, 0x32u}},
        {OA_CAN_MOTOR_DM4340, -2.5, 2.5, 125.0, 2.5, 7.0,
         {0x66u, 0x66u, 0x9fu, 0xf3u, 0xffu, 0x7fu, 0xf9u, 0xffu}},
        {OA_CAN_MOTOR_DM4310, 6.25, -7.5, 375.0, 3.75, -2.5,
         {0xbfu, 0xffu, 0x5fu, 0xfbu, 0xffu, 0xbfu, 0xf5u, 0xffu}}
    };
    size_t case_index;
    for (case_index = 0u; case_index < sizeof(cases) / sizeof(cases[0]); ++case_index) {
        oa_can_mit_command command = command_for(cases[case_index].type);
        oa_can_frame frame;
        oa_can_encode_result encoded;
        INITIALIZE(frame);
        INITIALIZE(encoded);
        command.position_rad = cases[case_index].position;
        command.velocity_rad_s = cases[case_index].velocity;
        command.kp = cases[case_index].kp;
        command.kd = cases[case_index].kd;
        command.torque_nm = cases[case_index].torque;
        CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_REJECT, &frame, &encoded) == OA_CAN_OK);
        CHECK(memcmp(frame.data, cases[case_index].bytes, sizeof(frame.data)) == 0);
    }
}

static void test_round_trip_properties(void) {
    const oa_can_motor_type types[] = {OA_CAN_MOTOR_DM8009, OA_CAN_MOTOR_DM4340, OA_CAN_MOTOR_DM4310};
    uint32_t seed = UINT32_C(0x5a17);
    size_t type_index;
    for (type_index = 0u; type_index < sizeof(types) / sizeof(types[0]); ++type_index) {
        oa_can_limits limits;
        size_t sample;
        INITIALIZE(limits);
        CHECK(oa_can_motor_limits(types[type_index], &limits) == OA_CAN_OK);
        for (sample = 0u; sample < 1500u; ++sample) {
            oa_can_mit_command command = command_for(types[type_index]);
            oa_can_frame frame;
            oa_can_frame feedback_from_command;
            oa_can_encode_result encoded;
            oa_can_feedback decoded;
            double u1;
            double u2;
            double u3;
            INITIALIZE(frame);
            INITIALIZE(encoded);
            INITIALIZE(decoded);
            seed = seed * UINT32_C(1103515245) + UINT32_C(12345);
            u1 = (double)(seed & UINT32_C(0xffff)) / 65535.0;
            seed = seed * UINT32_C(1103515245) + UINT32_C(12345);
            u2 = (double)(seed & UINT32_C(0xffff)) / 65535.0;
            seed = seed * UINT32_C(1103515245) + UINT32_C(12345);
            u3 = (double)(seed & UINT32_C(0xffff)) / 65535.0;
            command.position_rad = limits.position_min_rad + u1 * (limits.position_max_rad - limits.position_min_rad);
            command.velocity_rad_s = limits.velocity_min_rad_s + u2 * (limits.velocity_max_rad_s - limits.velocity_min_rad_s);
            command.torque_nm = limits.torque_min_nm + u3 * (limits.torque_max_nm - limits.torque_min_nm);
            command.kp = 400.0 * u1;
            command.kd = 4.0 * u2;
            CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_REJECT, &frame, &encoded) == OA_CAN_OK);
            feedback_from_command = feedback_frame(0x11u, 1u, 1u,
                ((uint32_t)frame.data[0] << 8u) | frame.data[1],
                ((uint32_t)frame.data[2] << 4u) | (frame.data[3] >> 4u),
                ((uint32_t)(frame.data[6] & 0x0fu) << 8u) | frame.data[7]);
            CHECK(oa_can_decode_feedback(&feedback_from_command, 0x11u, 1u,
                                         types[type_index], &decoded) == OA_CAN_OK);
            CHECK(fabs(decoded.position_rad - command.position_rad) <=
                  lsb(limits.position_min_rad, limits.position_max_rad, 16u));
            CHECK(fabs(decoded.velocity_rad_s - command.velocity_rad_s) <=
                  lsb(limits.velocity_min_rad_s, limits.velocity_max_rad_s, 12u));
            CHECK(fabs(decoded.torque_nm - command.torque_nm) <=
                  lsb(limits.torque_min_nm, limits.torque_max_nm, 12u));
        }
    }
}

static void test_size_version_and_malformed_inputs(void) {
    oa_can_mit_command command = command_for(OA_CAN_MOTOR_DM8009);
    oa_can_frame frame;
    oa_can_encode_result encoded;
    oa_can_feedback feedback;
    struct guarded_limits {
        oa_can_limits value;
        uint64_t canary;
    } guarded;
    struct guarded_frame {
        oa_can_frame value;
        uint64_t canary;
    } guarded_frame;
    unsigned char snapshot[sizeof(guarded) - sizeof(uint32_t) * 2u];
    unsigned char frame_snapshot[sizeof(guarded_frame) - sizeof(uint32_t) * 2u];
    unsigned int byte_value;
    INITIALIZE(frame);
    INITIALIZE(encoded);
    INITIALIZE(feedback);
    command.position_rad = 13.0;
    command.velocity_rad_s = 46.0;
    command.kp = 501.0;
    command.kd = 6.0;
    command.torque_nm = 55.0;
    CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_REJECT, &frame, &encoded) == OA_CAN_ERANGE);
    CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_SATURATE, &frame, &encoded) == OA_CAN_OK);
    CHECK(encoded.saturated_mask == 31u);
    command.position_rad = NAN;
    CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_SATURATE, &frame, &encoded) == OA_CAN_EINVAL);
    command.position_rad = INFINITY;
    CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_REJECT, &frame, &encoded) == OA_CAN_EINVAL);
    command.position_rad = 0.0;
    command.abi_version = OA_CAN_ABI_VERSION + 1u;
    CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_REJECT, &frame, &encoded) == OA_CAN_EINVAL);
    command.abi_version = OA_CAN_ABI_VERSION;
    (void)memset(&guarded, 0xa5, sizeof(guarded));
    guarded.value.struct_size = (uint32_t)(sizeof(uint32_t) * 2u);
    guarded.value.abi_version = OA_CAN_ABI_VERSION;
    (void)memcpy(snapshot, (const unsigned char *)&guarded + sizeof(uint32_t) * 2u, sizeof(snapshot));
    CHECK(oa_can_motor_limits(OA_CAN_MOTOR_DM4310, &guarded.value) == OA_CAN_EINVAL);
    CHECK(memcmp(snapshot, (const unsigned char *)&guarded + sizeof(uint32_t) * 2u, sizeof(snapshot)) == 0);
    guarded.value.struct_size = (uint32_t)sizeof(guarded.value);
    guarded.value.abi_version = OA_CAN_ABI_VERSION + 1u;
    CHECK(oa_can_motor_limits(OA_CAN_MOTOR_DM4310, &guarded.value) == OA_CAN_EINVAL);
    (void)memset(&guarded_frame, 0x5a, sizeof(guarded_frame));
    guarded_frame.value.struct_size = (uint32_t)(sizeof(uint32_t) * 2u);
    guarded_frame.value.abi_version = OA_CAN_ABI_VERSION;
    (void)memcpy(frame_snapshot, (const unsigned char *)&guarded_frame + sizeof(uint32_t) * 2u,
                 sizeof(frame_snapshot));
    CHECK(oa_can_make_enable(1u, &guarded_frame.value) == OA_CAN_EINVAL);
    CHECK(memcmp(frame_snapshot, (const unsigned char *)&guarded_frame + sizeof(uint32_t) * 2u,
                 sizeof(frame_snapshot)) == 0);
    frame = feedback_frame(0x11u, 1u, 1u, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_decode_feedback(&frame, 0x11u, 1u, OA_CAN_MOTOR_DM4310, &feedback) == OA_CAN_OK);
    frame.dlc = 7u;
    CHECK(oa_can_decode_feedback(&frame, 0x11u, 1u, OA_CAN_MOTOR_DM4310, &feedback) == OA_CAN_EFRAME);
    frame.dlc = 8u;
    frame.can_id = 0x11u | OA_CAN_EFF_FLAG;
    CHECK(oa_can_decode_feedback(&frame, 0x11u, 1u, OA_CAN_MOTOR_DM4310, &feedback) == OA_CAN_EFRAME);
    frame.can_id = 0x11u;
    frame.data[0] = 2u;
    CHECK(oa_can_decode_feedback(&frame, 0x11u, 1u, OA_CAN_MOTOR_DM4310, &feedback) == OA_CAN_EID);
    for (byte_value = 0u; byte_value < 256u; ++byte_value) {
        frame = feedback_frame(0x11u, (uint8_t)(byte_value & 0x0fu), (uint8_t)(byte_value >> 4u),
                               (uint32_t)byte_value * UINT32_C(257),
                               ((uint32_t)byte_value * UINT32_C(17)) & UINT32_C(0xfff),
                               ((uint32_t)byte_value * UINT32_C(29)) & UINT32_C(0xfff));
        (void)oa_can_decode_feedback(&frame, 0x11u, 1u, OA_CAN_MOTOR_DM4310, &feedback);
    }
}

static void test_status_nibbles(void) {
    oa_can_feedback feedback;
    uint8_t status_nibble;
    INITIALIZE(feedback);
    for (status_nibble = 0u; status_nibble < 16u; ++status_nibble) {
        oa_can_frame frame = feedback_frame(0x11u, 1u, status_nibble, 0x8000u, 0x800u, 0x800u);
        oa_can_status result = oa_can_decode_feedback(&frame, 0x11u, 1u, OA_CAN_MOTOR_DM4310, &feedback);
        if (status_nibble == 0u || status_nibble == 1u) CHECK(result == OA_CAN_OK);
        else if (status_nibble >= 8u && status_nibble <= 14u) {
            CHECK(result == OA_CAN_EFAULT);
            CHECK(feedback.status_nibble == status_nibble && feedback.mos_temperature_c == 42u &&
                  feedback.rotor_temperature_c == 51u);
        } else CHECK(result == OA_CAN_EFRAME);
    }
}

static void test_public_integer_widths(void) {
    CHECK(sizeof(oa_can_status) == sizeof(uint32_t));
    CHECK(sizeof(oa_can_motor_type) == sizeof(uint32_t));
    CHECK(sizeof(oa_can_range_policy) == sizeof(uint32_t));
    CHECK(sizeof(oa_can_fake_lifecycle) == sizeof(uint32_t));
    CHECK(sizeof(oa_can_feedback_status) == sizeof(uint8_t));
    CHECK(sizeof(oa_can_register_value_type) == sizeof(uint32_t));
    CHECK(sizeof(oa_can_register_operation) == sizeof(uint32_t));
    CHECK(sizeof(oa_can_register_id) == sizeof(uint32_t));
}

static void test_register_codecs_and_dynamic_profile(void) {
    oa_can_register_info info;
    oa_can_register_request request = register_request(OA_CAN_RID_PMAX, OA_CAN_REGISTER_F32);
    oa_can_register_write write;
    oa_can_register_value pmax;
    oa_can_register_value vmax;
    oa_can_register_value tmax;
    oa_can_register_value register_value;
    oa_can_mit_profile profile;
    oa_can_mit_profile_command mit;
    oa_can_pos_vel_command pos_vel;
    oa_can_pos_force_command pos_force;
    oa_can_frame frame;
    oa_can_feedback feedback;
    const uint8_t mit_golden[8] = {0xbfu, 0xffu, 0xbfu, 0xf7u,
                                   0xffu, 0x7fu, 0xf3u, 0xffu};
    const uint8_t pos_vel_golden[8] = {0x00u, 0x00u, 0xc0u, 0x3fu,
                                       0x00u, 0x00u, 0x10u, 0x40u};
    const uint8_t pos_force_golden[8] = {0x00u, 0x00u, 0xc0u, 0xbfu,
                                         0xe1u, 0x00u, 0x88u, 0x13u};
    INITIALIZE(info);
    INITIALIZE(frame);
    CHECK(oa_can_register_info_for_id(OA_CAN_RID_SERIAL_NUMBER, &info) == OA_CAN_OK);
    CHECK(info.value_type == OA_CAN_REGISTER_U32 && info.writable == 0u);
    CHECK(oa_can_register_info_for_id(OA_CAN_RID_PMAX, &info) == OA_CAN_OK);
    CHECK(info.value_type == OA_CAN_REGISTER_F32 && info.writable == 1u);
    CHECK(oa_can_register_info_for_id(49u, &info) == OA_CAN_EINVAL);

    CHECK(oa_can_make_register_query_typed(&request, &frame) == OA_CAN_OK);
    CHECK(frame.can_id == 0x7ffu && frame.dlc == 8u && frame.data[0] == 0x23u &&
          frame.data[1] == 0x01u && frame.data[2] == 0x33u && frame.data[3] == 21u);
    INITIALIZE(write);
    write.send_id = 0x123u;
    write.register_id = OA_CAN_RID_PMAX;
    write.value_type = OA_CAN_REGISTER_F32;
    write.value_f32 = 12.5f;
    CHECK(oa_can_make_register_write(&write, &frame) == OA_CAN_OK);
    CHECK(frame.can_id == 0x7ffu && frame.data[0] == 0x23u && frame.data[1] == 0x01u &&
          frame.data[2] == 0x55u && frame.data[3] == 21u && frame.data[4] == 0x00u &&
          frame.data[5] == 0x00u && frame.data[6] == 0x48u && frame.data[7] == 0x41u);
    write.register_id = OA_CAN_RID_CTRL_MODE;
    write.value_type = OA_CAN_REGISTER_U32;
    write.value_f32 = 0.0f;
    write.value_u32 = 1u;
    CHECK(oa_can_make_register_write(&write, &frame) == OA_CAN_OK);
    CHECK(frame.data[3] == 10u && frame.data[4] == 1u && frame.data[5] == 0u &&
          frame.data[6] == 0u && frame.data[7] == 0u);
    request.register_id = OA_CAN_RID_CTRL_MODE;
    request.value_type = OA_CAN_REGISTER_U32;
    frame = register_frame(request.receive_id, request.send_id, OA_CAN_REGISTER_WRITE,
                           OA_CAN_RID_CTRL_MODE, 1u);
    INITIALIZE(register_value);
    CHECK(oa_can_decode_register_response(&frame, &request, OA_CAN_REGISTER_WRITE,
                                          &register_value) == OA_CAN_OK);
    CHECK(register_value.operation == OA_CAN_REGISTER_WRITE &&
          register_value.value_type == OA_CAN_REGISTER_U32 &&
          register_value.value_u32 == 1u && register_value.raw_value == 1u);
    request.register_id = OA_CAN_RID_SERIAL_NUMBER;
    frame = register_frame(request.receive_id, request.send_id, OA_CAN_REGISTER_QUERY,
                           OA_CAN_RID_SERIAL_NUMBER, UINT32_C(0x12345678));
    CHECK(oa_can_decode_register_response(&frame, &request, OA_CAN_REGISTER_QUERY,
                                          &register_value) == OA_CAN_OK);
    CHECK(register_value.value_u32 == UINT32_C(0x12345678));
    CHECK(oa_can_make_set_zero(0x123u, &frame) == OA_CAN_OK);
    CHECK(frame.can_id == 0x123u && frame.data[7] == 0xfeu);
    CHECK(oa_can_make_clear_error(0x123u, &frame) == OA_CAN_OK);
    CHECK(frame.can_id == 0x123u && frame.data[7] == 0xfbu);
    CHECK(oa_can_make_save_parameters(0x123u, &frame) == OA_CAN_OK);
    CHECK(frame.can_id == 0x7ffu && frame.data[0] == 0x23u && frame.data[1] == 0x01u &&
          frame.data[2] == 0xaau && frame.data[3] == 0u && frame.data[7] == 0u);

    INITIALIZE(pmax);
    request.send_id = 5u;
    request.receive_id = 0x15u;
    request.register_id = OA_CAN_RID_PMAX;
    request.value_type = OA_CAN_REGISTER_F32;
    frame = register_frame(request.receive_id, request.send_id, OA_CAN_REGISTER_QUERY,
                           OA_CAN_RID_PMAX, raw_float(20.0f));
    CHECK(oa_can_decode_register_response(&frame, &request, OA_CAN_REGISTER_QUERY, &pmax) ==
          OA_CAN_OK);
    CHECK(pmax.receive_id == request.receive_id && pmax.target_send_id == request.send_id &&
          pmax.register_id == OA_CAN_RID_PMAX && pmax.value_type == OA_CAN_REGISTER_F32 &&
          pmax.operation == OA_CAN_REGISTER_QUERY && pmax.value_f32 == 20.0f);
    request.register_id = OA_CAN_RID_VMAX;
    INITIALIZE(vmax);
    frame = register_frame(request.receive_id, request.send_id, OA_CAN_REGISTER_QUERY,
                           OA_CAN_RID_VMAX, raw_float(20.0f));
    CHECK(oa_can_decode_register_response(&frame, &request, OA_CAN_REGISTER_QUERY, &vmax) ==
          OA_CAN_OK);
    request.register_id = OA_CAN_RID_TMAX;
    INITIALIZE(tmax);
    frame = register_frame(request.receive_id, request.send_id, OA_CAN_REGISTER_QUERY,
                           OA_CAN_RID_TMAX, raw_float(30.0f));
    CHECK(oa_can_decode_register_response(&frame, &request, OA_CAN_REGISTER_QUERY, &tmax) ==
          OA_CAN_OK);
    INITIALIZE(profile);
    CHECK(oa_can_mit_profile_from_registers(&pmax, &vmax, &tmax, &profile) == OA_CAN_OK);
    CHECK(profile.target_send_id == 5u && profile.receive_id == 0x15u &&
          profile.pmax_rad == 20.0 && profile.vmax_rad_s == 20.0 &&
          profile.tmax_nm == 30.0 && profile.verified_mask == OA_CAN_PROFILE_ALL_VERIFIED);

    INITIALIZE(mit);
    mit.send_id = 5u;
    mit.position_rad = 10.0;
    mit.velocity_rad_s = 10.0;
    mit.kp = 250.0;
    mit.kd = 2.5;
    mit.torque_nm = -15.0;
    INITIALIZE(frame);
    CHECK(oa_can_encode_mit_profile(&mit, &profile, &frame) == OA_CAN_OK);
    CHECK(frame.can_id == 5u && memcmp(frame.data, mit_golden, sizeof(mit_golden)) == 0);
    frame = feedback_frame(0x15u, 5u, OA_CAN_FEEDBACK_ENABLED,
                           0xbfffu, 0xbffu, 0x3ffu);
    INITIALIZE(feedback);
    CHECK(oa_can_decode_feedback_profile(&frame, 0x15u, 5u, &profile, &feedback) == OA_CAN_OK);
    CHECK(feedback.motor_id == 5u && feedback.status_nibble == OA_CAN_FEEDBACK_ENABLED &&
          feedback.mos_temperature_c == 42u && feedback.rotor_temperature_c == 51u);
    CHECK(fabs(feedback.position_rad - 10.0) <= lsb(-20.0, 20.0, 16u));
    CHECK(fabs(feedback.velocity_rad_s - 10.0) <= lsb(-20.0, 20.0, 12u));
    CHECK(fabs(feedback.torque_nm + 15.0) <= lsb(-30.0, 30.0, 12u));

    INITIALIZE(pos_vel);
    pos_vel.send_id = 5u;
    pos_vel.position_rad = 1.5;
    pos_vel.max_velocity_rad_s = 2.25;
    CHECK(oa_can_encode_pos_vel(&pos_vel, &profile, &frame) == OA_CAN_OK);
    CHECK(frame.can_id == 0x105u && memcmp(frame.data, pos_vel_golden, sizeof(pos_vel_golden)) == 0);
    INITIALIZE(pos_force);
    pos_force.send_id = 5u;
    pos_force.position_rad = -1.5;
    pos_force.max_velocity_rad_s = 2.25;
    pos_force.current_limit_per_unit = 0.5;
    CHECK(oa_can_encode_pos_force(&pos_force, &profile, &frame) == OA_CAN_OK);
    CHECK(frame.can_id == 0x305u &&
          memcmp(frame.data, pos_force_golden, sizeof(pos_force_golden)) == 0);
}

static void test_new_codec_malformed_and_canaries(void) {
    oa_can_register_request request = register_request(OA_CAN_RID_PMAX, OA_CAN_REGISTER_F32);
    oa_can_register_write write;
    oa_can_mit_profile profile = dynamic_profile();
    oa_can_mit_profile_command mit;
    oa_can_pos_vel_command pos_vel;
    oa_can_pos_force_command pos_force;
    oa_can_feedback feedback;
    oa_can_frame frame;
    struct guarded_frame {
        oa_can_frame value;
        uint64_t canary;
    } guarded_frame;
    struct guarded_value {
        oa_can_register_value value;
        uint64_t canary;
    } guarded_value;
    struct guarded_profile {
        oa_can_mit_profile value;
        uint64_t canary;
    } guarded_profile;
    unsigned char frame_snapshot[sizeof(guarded_frame)];
    unsigned char value_snapshot[sizeof(guarded_value)];
    unsigned char profile_snapshot[sizeof(guarded_profile)];

    (void)memset(&guarded_frame, 0xa5, sizeof(guarded_frame));
    guarded_frame.value.struct_size = (uint32_t)sizeof(guarded_frame.value);
    guarded_frame.value.abi_version = OA_CAN_ABI_VERSION;
    (void)memcpy(frame_snapshot, &guarded_frame, sizeof(frame_snapshot));
    request.value_type = OA_CAN_REGISTER_U32;
    CHECK(oa_can_make_register_query_typed(&request, &guarded_frame.value) == OA_CAN_EINVAL);
    CHECK(memcmp(frame_snapshot, &guarded_frame, sizeof(frame_snapshot)) == 0);
    request.value_type = OA_CAN_REGISTER_F32;
    request.register_id = 49u;
    CHECK(oa_can_make_register_query_typed(&request, &guarded_frame.value) == OA_CAN_EINVAL);
    request.register_id = OA_CAN_RID_PMAX;
    request.abi_version = OA_CAN_ABI_VERSION + 1u;
    CHECK(oa_can_make_register_query_typed(&request, &guarded_frame.value) == OA_CAN_EINVAL);
    request.abi_version = OA_CAN_ABI_VERSION;

    INITIALIZE(write);
    write.send_id = 1u;
    write.register_id = OA_CAN_RID_SERIAL_NUMBER;
    write.value_type = OA_CAN_REGISTER_U32;
    write.value_u32 = 1u;
    CHECK(oa_can_make_register_write(&write, &guarded_frame.value) == OA_CAN_EINVAL);
    write.register_id = OA_CAN_RID_CTRL_MODE;
    write.value_u32 = 5u;
    CHECK(oa_can_make_register_write(&write, &guarded_frame.value) == OA_CAN_ERANGE);
    write.register_id = OA_CAN_RID_PMAX;
    write.value_type = OA_CAN_REGISTER_F32;
    write.value_u32 = 0u;
    write.value_f32 = NAN;
    CHECK(oa_can_make_register_write(&write, &guarded_frame.value) == OA_CAN_EINVAL);
    write.value_f32 = -1.0f;
    CHECK(oa_can_make_register_write(&write, &guarded_frame.value) == OA_CAN_ERANGE);

    frame = register_frame(request.receive_id, request.send_id, OA_CAN_REGISTER_QUERY,
                           OA_CAN_RID_PMAX, raw_float(20.0f));
    (void)memset(&guarded_value, 0x5a, sizeof(guarded_value));
    guarded_value.value.struct_size = (uint32_t)sizeof(guarded_value.value);
    guarded_value.value.abi_version = OA_CAN_ABI_VERSION;
    (void)memcpy(value_snapshot, &guarded_value, sizeof(value_snapshot));
    frame.dlc = 7u;
    CHECK(oa_can_decode_register_response(&frame, &request, OA_CAN_REGISTER_QUERY,
                                          &guarded_value.value) == OA_CAN_EFRAME);
    CHECK(memcmp(value_snapshot, &guarded_value, sizeof(value_snapshot)) == 0);
    frame.dlc = 8u;
    frame.can_id |= OA_CAN_EFF_FLAG;
    CHECK(oa_can_decode_register_response(&frame, &request, OA_CAN_REGISTER_QUERY,
                                          &guarded_value.value) == OA_CAN_EFRAME);
    frame.can_id = request.receive_id + 1u;
    CHECK(oa_can_decode_register_response(&frame, &request, OA_CAN_REGISTER_QUERY,
                                          &guarded_value.value) == OA_CAN_EID);
    frame = register_frame(request.receive_id, request.send_id + 1u, OA_CAN_REGISTER_QUERY,
                           OA_CAN_RID_PMAX, raw_float(20.0f));
    CHECK(oa_can_decode_register_response(&frame, &request, OA_CAN_REGISTER_QUERY,
                                          &guarded_value.value) == OA_CAN_EID);
    frame = register_frame(request.receive_id, request.send_id, OA_CAN_REGISTER_WRITE,
                           OA_CAN_RID_PMAX, raw_float(20.0f));
    CHECK(oa_can_decode_register_response(&frame, &request, OA_CAN_REGISTER_QUERY,
                                          &guarded_value.value) == OA_CAN_EFRAME);
    frame = register_frame(request.receive_id, request.send_id, OA_CAN_REGISTER_QUERY,
                           OA_CAN_RID_VMAX, raw_float(20.0f));
    CHECK(oa_can_decode_register_response(&frame, &request, OA_CAN_REGISTER_QUERY,
                                          &guarded_value.value) == OA_CAN_EFRAME);
    frame = register_frame(request.receive_id, request.send_id, OA_CAN_REGISTER_QUERY,
                           OA_CAN_RID_PMAX, raw_float(NAN));
    CHECK(oa_can_decode_register_response(&frame, &request, OA_CAN_REGISTER_QUERY,
                                          &guarded_value.value) == OA_CAN_EINVAL);

    (void)memset(&guarded_profile, 0xc3, sizeof(guarded_profile));
    guarded_profile.value.struct_size = (uint32_t)sizeof(guarded_profile.value);
    guarded_profile.value.abi_version = OA_CAN_ABI_VERSION;
    (void)memcpy(profile_snapshot, &guarded_profile, sizeof(profile_snapshot));
    INITIALIZE(guarded_value.value);
    guarded_value.value.receive_id = request.receive_id;
    guarded_value.value.target_send_id = request.send_id;
    guarded_value.value.register_id = OA_CAN_RID_PMAX;
    guarded_value.value.value_type = OA_CAN_REGISTER_F32;
    guarded_value.value.operation = OA_CAN_REGISTER_QUERY;
    guarded_value.value.raw_value = raw_float(20.0f);
    guarded_value.value.value_f32 = 20.0f;
    CHECK(oa_can_mit_profile_from_registers(&guarded_value.value, &guarded_value.value,
                                            &guarded_value.value, &guarded_profile.value) ==
          OA_CAN_EINVAL);
    CHECK(memcmp(profile_snapshot, &guarded_profile, sizeof(profile_snapshot)) == 0);

    INITIALIZE(mit);
    mit.send_id = 1u;
    profile.verified_mask = OA_CAN_PROFILE_PMAX_VERIFIED;
    CHECK(oa_can_encode_mit_profile(&mit, &profile, &guarded_frame.value) == OA_CAN_EINVAL);
    profile = dynamic_profile();
    mit.position_rad = 21.0;
    CHECK(oa_can_encode_mit_profile(&mit, &profile, &guarded_frame.value) == OA_CAN_ERANGE);
    mit.position_rad = NAN;
    CHECK(oa_can_encode_mit_profile(&mit, &profile, &guarded_frame.value) == OA_CAN_EINVAL);
    frame = feedback_frame(0x11u, 2u, OA_CAN_FEEDBACK_DISABLED, 0x8000u, 0x800u, 0x800u);
    INITIALIZE(feedback);
    CHECK(oa_can_decode_feedback_profile(&frame, 0x11u, 1u, &profile, &feedback) == OA_CAN_EID);
    frame.data[0] = (uint8_t)((7u << 4u) | 1u);
    CHECK(oa_can_decode_feedback_profile(&frame, 0x11u, 1u, &profile, &feedback) == OA_CAN_EFRAME);

    INITIALIZE(pos_vel);
    pos_vel.send_id = 1u;
    pos_vel.position_rad = 21.0;
    pos_vel.max_velocity_rad_s = 1.0;
    CHECK(oa_can_encode_pos_vel(&pos_vel, &profile, &guarded_frame.value) == OA_CAN_ERANGE);
    pos_vel.position_rad = NAN;
    CHECK(oa_can_encode_pos_vel(&pos_vel, &profile, &guarded_frame.value) == OA_CAN_EINVAL);
    pos_vel.position_rad = 0.0;
    pos_vel.send_id = 0x700u;
    CHECK(oa_can_encode_pos_vel(&pos_vel, &profile, &guarded_frame.value) == OA_CAN_EINVAL);
    INITIALIZE(pos_force);
    pos_force.send_id = 1u;
    pos_force.max_velocity_rad_s = 1.0;
    pos_force.current_limit_per_unit = 1.01;
    CHECK(oa_can_encode_pos_force(&pos_force, &profile, &guarded_frame.value) == OA_CAN_ERANGE);
    pos_force.current_limit_per_unit = INFINITY;
    CHECK(oa_can_encode_pos_force(&pos_force, &profile, &guarded_frame.value) == OA_CAN_EINVAL);
    pos_force.current_limit_per_unit = 0.5;
    pos_force.send_id = 0x500u;
    CHECK(oa_can_encode_pos_force(&pos_force, &profile, &guarded_frame.value) == OA_CAN_EINVAL);
}

static oa_can_arm_manifest manifest_two(void) {
    oa_can_arm_manifest manifest;
    size_t motor_index;
    INITIALIZE(manifest);
    manifest.motor_count = 2u;
    for (motor_index = 0u; motor_index < 2u; ++motor_index) {
        oa_can_motor_manifest *motor = &manifest.motors[motor_index];
        INITIALIZE(*motor);
        motor->joint_index = (uint8_t)motor_index;
        motor->expected_motor_id = (uint8_t)(motor_index + 1u);
        motor->send_id = (uint16_t)(motor_index + 1u);
        motor->receive_id = (uint16_t)(0x11u + motor_index);
        motor->decode_motor_type = OA_CAN_MOTOR_DM4310;
        INITIALIZE(motor->mapping);
        motor->mapping.position_scale = 1.0;
        motor->mapping.velocity_scale = 1.0;
        motor->mapping.torque_scale = 1.0;
        (void)snprintf(motor->joint_name, sizeof(motor->joint_name), "joint%u",
                       (unsigned int)(motor_index + 1u));
    }
    return manifest;
}

static oa_can_probe_options probe_options(uint32_t max_receive_frames) {
    oa_can_probe_options options;
    INITIALIZE(options);
    options.timeout_ns = 100u;
    options.max_receive_frames = max_receive_frames;
    return options;
}

static oa_can_fake *create_fake(oa_can_transport *transport, size_t capacity) {
    oa_can_fake *fake = NULL;
    INITIALIZE(*transport);
    CHECK(oa_can_fake_create(capacity, &fake) == OA_CAN_OK);
    CHECK(oa_can_fake_set_time(fake, 100u) == OA_CAN_OK);
    CHECK(oa_can_fake_transport(fake, transport) == OA_CAN_OK);
    return fake;
}

static void test_probe_fresh_disabled_only(void) {
    oa_can_arm_manifest manifest = manifest_two();
    oa_can_transport transport;
    oa_can_probe_options options = probe_options(16u);
    oa_can_probe_report report;
    oa_can_frame frame;
    oa_can_fake *fake;
    INITIALIZE(report);
    CHECK(oa_can_validate_manifest(&manifest) == OA_CAN_OK);
    fake = create_fake(&transport, 2u);
    report.struct_size = (uint32_t)(sizeof(uint32_t) * 2u);
    CHECK(oa_can_probe_expected(&transport, &manifest, &options, &report) == OA_CAN_EINVAL);
    CHECK(oa_can_fake_sent_count(fake) == 0u);
    oa_can_fake_destroy(fake);
    INITIALIZE(report);
    fake = create_fake(&transport, 8u);
    frame = feedback_frame(0x11u, 1u, OA_CAN_FEEDBACK_DISABLED, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame, 110u) == OA_CAN_OK);
    frame = feedback_frame(0x12u, 2u, OA_CAN_FEEDBACK_DISABLED, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame, 111u) == OA_CAN_OK);
    CHECK(oa_can_probe_expected(&transport, &manifest, &options, &report) == OA_CAN_OK);
    CHECK(report.expected_mask == 3u && report.fresh_mask == 3u && report.enabled_mask == 0u);
    CHECK(oa_can_fake_sent_count(fake) == 2u && oa_can_fake_get_lifecycle(fake) == OA_CAN_FAKE_PROBED);
    CHECK(oa_can_fake_torque_enabled(fake) == 0);
    oa_can_fake_destroy(fake);
}

static void test_probe_rejects_stale_enabled_fault_and_busy(void) {
    oa_can_arm_manifest manifest = manifest_two();
    oa_can_transport transport;
    oa_can_probe_options options = probe_options(16u);
    oa_can_probe_report report;
    oa_can_frame frame;
    oa_can_fake *fake;
    size_t index;
    INITIALIZE(report);
    fake = create_fake(&transport, 8u);
    frame = feedback_frame(0x11u, 1u, OA_CAN_FEEDBACK_DISABLED, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame, 50u) == OA_CAN_OK);
    CHECK(oa_can_probe_expected(&transport, &manifest, &options, &report) == OA_CAN_ETIMEOUT);
    CHECK(report.stale_frames == 1u && report.fresh_mask == 0u);
    oa_can_fake_destroy(fake);

    INITIALIZE(report);
    fake = create_fake(&transport, 8u);
    frame = feedback_frame(0x11u, 1u, OA_CAN_FEEDBACK_ENABLED, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame, 110u) == OA_CAN_OK);
    frame = feedback_frame(0x12u, 2u, OA_CAN_FEEDBACK_DISABLED, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame, 111u) == OA_CAN_OK);
    CHECK(oa_can_probe_expected(&transport, &manifest, &options, &report) == OA_CAN_ESTATE);
    CHECK(report.enabled_mask == 1u && report.fresh_mask == 2u);
    oa_can_fake_destroy(fake);

    INITIALIZE(report);
    fake = create_fake(&transport, 8u);
    frame = feedback_frame(0x11u, 1u, OA_CAN_FEEDBACK_OVER_VOLTAGE, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame, 110u) == OA_CAN_OK);
    frame = feedback_frame(0x12u, 2u, OA_CAN_FEEDBACK_DISABLED, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame, 111u) == OA_CAN_OK);
    CHECK(oa_can_probe_expected(&transport, &manifest, &options, &report) == OA_CAN_EFAULT);
    CHECK(report.fault_mask == 1u && report.fresh_mask == 2u);
    oa_can_fake_destroy(fake);

    options = probe_options(4u);
    INITIALIZE(report);
    fake = create_fake(&transport, 8u);
    for (index = 0u; index < 8u; ++index) {
        frame = feedback_frame((uint16_t)(0x40u + index), 9u, OA_CAN_FEEDBACK_DISABLED,
                               0x8000u, 0x800u, 0x800u);
        CHECK(oa_can_fake_enqueue_feedback(fake, &frame, (uint64_t)(110u + index)) == OA_CAN_OK);
    }
    CHECK(oa_can_probe_expected(&transport, &manifest, &options, &report) == OA_CAN_ETIMEOUT);
    CHECK(report.received_frames == 4u && report.unexpected_frames == 4u && report.receive_limit_reached == 1u);
    oa_can_fake_destroy(fake);

    options = probe_options(16u);
    INITIALIZE(report);
    fake = create_fake(&transport, 2u);
    frame = feedback_frame(0x11u, 1u, OA_CAN_FEEDBACK_DISABLED, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame, 500u) == OA_CAN_OK);
    CHECK(oa_can_probe_expected(&transport, &manifest, &options, &report) == OA_CAN_ETIMEOUT);
    CHECK(report.received_frames == 0u && report.deadline_monotonic_ns == 202u);
    oa_can_fake_destroy(fake);
}

static void test_probe_receive_budget_boundary(void) {
    oa_can_arm_manifest manifest = manifest_two();
    oa_can_transport transport;
    oa_can_probe_options options = probe_options(2u);
    oa_can_probe_report report;
    oa_can_frame frame;
    oa_can_fake *fake;
    INITIALIZE(report);
    fake = create_fake(&transport, 4u);
    frame = feedback_frame(0x11u, 1u, OA_CAN_FEEDBACK_DISABLED, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame, 110u) == OA_CAN_OK);
    frame = feedback_frame(0x12u, 2u, OA_CAN_FEEDBACK_DISABLED, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame, 111u) == OA_CAN_OK);
    CHECK(oa_can_probe_expected(&transport, &manifest, &options, &report) == OA_CAN_ETIMEOUT);
    CHECK(report.fresh_mask == 3u && report.received_frames == 2u && report.receive_limit_reached == 1u);
    oa_can_fake_destroy(fake);

    options = probe_options(3u);
    INITIALIZE(report);
    fake = create_fake(&transport, 4u);
    frame = feedback_frame(0x11u, 1u, OA_CAN_FEEDBACK_DISABLED, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame, 110u) == OA_CAN_OK);
    frame = feedback_frame(0x12u, 2u, OA_CAN_FEEDBACK_DISABLED, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame, 111u) == OA_CAN_OK);
    CHECK(oa_can_probe_expected(&transport, &manifest, &options, &report) == OA_CAN_OK);
    CHECK(report.fresh_mask == 3u && report.received_frames == 2u && report.receive_limit_reached == 0u);
    oa_can_fake_destroy(fake);
}

static void test_fake_rejects_control_frames(void) {
    oa_can_transport transport;
    oa_can_fake *fake = create_fake(&transport, 2u);
    oa_can_frame frame;
    oa_can_encode_result encoded;
    oa_can_mit_command command = command_for(OA_CAN_MOTOR_DM4310);
    oa_can_mit_profile profile = dynamic_profile();
    oa_can_pos_vel_command pos_vel;
    oa_can_register_write write;
    uint64_t sent_ns;
    INITIALIZE(frame);
    CHECK(oa_can_make_enable(1u, &frame) == OA_CAN_OK);
    CHECK(transport.send(transport.context, &frame, &sent_ns) == OA_CAN_ESTATE);
    CHECK(oa_can_fake_get_lifecycle(fake) == OA_CAN_FAKE_FAULT);
    CHECK(oa_can_fake_rejected_control_count(fake) == 1u);
    INITIALIZE(frame);
    INITIALIZE(encoded);
    CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_REJECT, &frame, &encoded) == OA_CAN_OK);
    CHECK(transport.send(transport.context, &frame, &sent_ns) == OA_CAN_ESTATE);
    CHECK(oa_can_fake_rejected_control_count(fake) == 2u && oa_can_fake_torque_enabled(fake) == 0);
    INITIALIZE(frame);
    CHECK(oa_can_make_disable(1u, &frame) == OA_CAN_OK);
    CHECK(transport.send(transport.context, &frame, &sent_ns) == OA_CAN_OK);
    CHECK(oa_can_fake_get_lifecycle(fake) == OA_CAN_FAKE_FAULT);
    INITIALIZE(frame);
    frame.can_id = 0x7ffu;
    frame.dlc = 8u;
    frame.data[0] = 1u;
    frame.data[2] = 0xccu;
    frame.data[7] = 1u;
    CHECK(transport.send(transport.context, &frame, &sent_ns) == OA_CAN_ESTATE);
    CHECK(oa_can_fake_rejected_control_count(fake) == 3u);
    INITIALIZE(pos_vel);
    pos_vel.send_id = 1u;
    pos_vel.max_velocity_rad_s = 1.0;
    CHECK(oa_can_encode_pos_vel(&pos_vel, &profile, &frame) == OA_CAN_OK);
    CHECK(transport.send(transport.context, &frame, &sent_ns) == OA_CAN_ESTATE);
    INITIALIZE(write);
    write.send_id = 1u;
    write.register_id = OA_CAN_RID_CTRL_MODE;
    write.value_type = OA_CAN_REGISTER_U32;
    write.value_u32 = 1u;
    CHECK(oa_can_make_register_write(&write, &frame) == OA_CAN_OK);
    CHECK(transport.send(transport.context, &frame, &sent_ns) == OA_CAN_ESTATE);
    CHECK(oa_can_make_set_zero(1u, &frame) == OA_CAN_OK);
    CHECK(transport.send(transport.context, &frame, &sent_ns) == OA_CAN_ESTATE);
    CHECK(oa_can_make_clear_error(1u, &frame) == OA_CAN_OK);
    CHECK(transport.send(transport.context, &frame, &sent_ns) == OA_CAN_ESTATE);
    CHECK(oa_can_make_save_parameters(1u, &frame) == OA_CAN_OK);
    CHECK(transport.send(transport.context, &frame, &sent_ns) == OA_CAN_ESTATE);
    CHECK(oa_can_fake_rejected_control_count(fake) == 8u &&
          oa_can_fake_torque_enabled(fake) == 0);
    oa_can_fake_destroy(fake);
}

#ifdef __linux__
static size_t align_netlink(size_t value) {
    return (value + 3u) & ~(size_t)3u;
}

static size_t append_attribute(unsigned char *buffer, size_t capacity, size_t offset,
                               uint16_t type, const void *payload, size_t payload_length) {
    struct rtattr attribute;
    size_t length = sizeof(attribute) + payload_length;
    size_t aligned_length = align_netlink(length);
    if (offset > capacity || aligned_length > capacity - offset || length > UINT16_MAX) return SIZE_MAX;
    (void)memset(buffer + offset, 0, aligned_length);
    attribute.rta_len = (uint16_t)length;
    attribute.rta_type = type;
    (void)memcpy(buffer + offset, &attribute, sizeof(attribute));
    (void)memcpy(buffer + offset + sizeof(attribute), payload, payload_length);
    return offset + aligned_length;
}

static size_t build_link_datagram(unsigned char *buffer, size_t capacity, uint32_t sequence,
                                  int terminate_name) {
    unsigned char can_data[128];
    unsigned char link_info[256];
    struct can_bittiming timing;
    struct nlmsghdr header;
    struct ifinfomsg info;
    uint32_t mtu = 72u;
    const char kind[] = "can";
    const char name[] = "vcan0";
    size_t can_length = 0u;
    size_t link_length = 0u;
    size_t offset = sizeof(header) + align_netlink(sizeof(info));
    size_t message_length;
    size_t done_offset;
    (void)memset(buffer, 0, capacity);
    (void)memset(&timing, 0, sizeof(timing));
    timing.bitrate = 1000000u;
    can_length = append_attribute(can_data, sizeof(can_data), can_length, IFLA_CAN_BITTIMING,
                                  &timing, sizeof(timing));
    link_length = append_attribute(link_info, sizeof(link_info), link_length, IFLA_INFO_KIND,
                                   kind, sizeof(kind));
    link_length = append_attribute(link_info, sizeof(link_info), link_length,
                                   (uint16_t)(IFLA_INFO_DATA | NLA_F_NESTED),
                                   can_data, can_length);
    (void)memset(&header, 0, sizeof(header));
    header.nlmsg_type = RTM_NEWLINK;
    header.nlmsg_seq = sequence;
    (void)memcpy(buffer, &header, sizeof(header));
    (void)memset(&info, 0, sizeof(info));
    info.ifi_index = 7;
    info.ifi_flags = IFF_RUNNING;
    (void)memcpy(buffer + sizeof(header), &info, sizeof(info));
    offset = append_attribute(buffer, capacity, offset, IFLA_IFNAME, name,
                              terminate_name != 0 ? sizeof(name) : sizeof(name) - 1u);
    offset = append_attribute(buffer, capacity, offset, IFLA_MTU, &mtu, sizeof(mtu));
    offset = append_attribute(buffer, capacity, offset, (uint16_t)(IFLA_LINKINFO | NLA_F_NESTED),
                              link_info, link_length);
    message_length = offset;
    header.nlmsg_len = (uint32_t)message_length;
    (void)memcpy(buffer, &header, sizeof(header));
    done_offset = align_netlink(message_length);
    (void)memset(&header, 0, sizeof(header));
    header.nlmsg_len = (uint32_t)sizeof(header);
    header.nlmsg_type = NLMSG_DONE;
    header.nlmsg_seq = sequence;
    (void)memcpy(buffer + done_offset, &header, sizeof(header));
    return done_offset + sizeof(header);
}

static void test_synthetic_netlink_parser(void) {
    unsigned char storage[1024];
    unsigned char *unaligned = storage + 1u;
    oa_can_linux_interface interface_item;
    size_t count = 0u;
    size_t length;
    int done;
    uint32_t sequence = UINT32_C(0x12345678);
    INITIALIZE(interface_item);
    length = build_link_datagram(unaligned, sizeof(storage) - 1u, sequence, 1);
    CHECK(oa_can_linux_parse_datagram(unaligned, length, sequence, 0u, 0,
                                          &interface_item, 1u, &count, &done) == OA_CAN_OK);
    CHECK(done == 1 && count == 1u && strcmp(interface_item.name, "vcan0") == 0);
    CHECK(interface_item.ifindex == 7u && interface_item.mtu == 72u &&
          interface_item.bitrate == 1000000u && interface_item.link_up == 1u);
    count = 0u;
    INITIALIZE(interface_item);
    CHECK(oa_can_linux_parse_datagram(unaligned, length, sequence + 1u, 0u, 0,
                                          &interface_item, 1u, &count, &done) == OA_CAN_EFRAME);
    CHECK(oa_can_linux_parse_datagram(unaligned, length, sequence, 99u, 0,
                                          &interface_item, 1u, &count, &done) == OA_CAN_EFRAME);
    CHECK(oa_can_linux_parse_datagram(unaligned, length, sequence, 0u, 1,
                                          &interface_item, 1u, &count, &done) == OA_CAN_EFRAME);
    CHECK(oa_can_linux_parse_datagram(unaligned, 0u, sequence, 0u, 0,
                                          &interface_item, 1u, &count, &done) == OA_CAN_EFRAME);
    length = build_link_datagram(unaligned, sizeof(storage) - 1u, sequence, 0);
    count = 0u;
    INITIALIZE(interface_item);
    CHECK(oa_can_linux_parse_datagram(unaligned, length, sequence, 0u, 0,
                                          &interface_item, 1u, &count, &done) == OA_CAN_EFRAME);
    length = build_link_datagram(unaligned, sizeof(storage) - 1u, sequence, 1);
    count = 0u;
    INITIALIZE(interface_item);
    interface_item.struct_size = (uint32_t)(sizeof(uint32_t) * 2u);
    CHECK(oa_can_linux_parse_datagram(unaligned, length, sequence, 0u, 0,
                                          &interface_item, 1u, &count, &done) == OA_CAN_EINVAL);
    {
        struct nlmsghdr short_header;
        (void)memset(&short_header, 0, sizeof(short_header));
        short_header.nlmsg_len = (uint32_t)(sizeof(short_header) + 1u);
        short_header.nlmsg_type = RTM_NEWLINK;
        short_header.nlmsg_seq = sequence;
        (void)memcpy(unaligned, &short_header, sizeof(short_header));
        unaligned[sizeof(short_header)] = 0u;
        count = 0u;
        INITIALIZE(interface_item);
        CHECK(oa_can_linux_parse_datagram(unaligned, sizeof(short_header) + 1u, sequence, 0u, 0,
                                              &interface_item, 1u, &count, &done) == OA_CAN_EFRAME);
    }
}
#else
static void test_synthetic_netlink_parser(void) {
    size_t count = 99u;
    int done = 0;
    CHECK(oa_can_linux_list_interfaces(NULL, 0u, &count) == OA_CAN_EUNSUPPORTED);
    CHECK(count == 0u);
    CHECK(oa_can_linux_parse_datagram(NULL, 0u, 0u, 0u, 0, NULL, 0u, &count, &done) ==
          OA_CAN_EUNSUPPORTED);
}
#endif

int main(void) {
    test_golden_and_special_frames();
    test_all_motor_golden_vectors();
    test_round_trip_properties();
    test_size_version_and_malformed_inputs();
    test_status_nibbles();
    test_public_integer_widths();
    test_register_codecs_and_dynamic_profile();
    test_new_codec_malformed_and_canaries();
    test_probe_fresh_disabled_only();
    test_probe_rejects_stale_enabled_fault_and_busy();
    test_probe_receive_budget_boundary();
    test_fake_rejects_control_frames();
    test_synthetic_netlink_parser();
    if (failures != 0) {
        (void)fprintf(stderr, "%d test assertion(s) failed\n", failures);
        return 1;
    }
    (void)puts("openarm_can tests passed");
    return 0;
}
