/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_can.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures;

#define CHECK(condition) do { if (!(condition)) { \
    (void)fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); ++failures; } } while (0)

static oa_can_mit_command command_for(oa_can_motor_type type) {
    oa_can_mit_command command;
    (void)memset(&command, 0, sizeof(command));
    command.struct_size = (uint32_t)sizeof(command);
    command.abi_version = OA_CAN_ABI_VERSION;
    command.send_id = 1u;
    command.motor_type = type;
    return command;
}

static oa_can_frame feedback_frame(uint16_t receive_id, uint8_t motor_id, uint8_t status,
                                   uint32_t position, uint32_t velocity, uint32_t torque) {
    oa_can_frame frame;
    (void)memset(&frame, 0, sizeof(frame));
    frame.struct_size = (uint32_t)sizeof(frame);
    frame.abi_version = OA_CAN_ABI_VERSION;
    frame.can_id = receive_id;
    frame.dlc = 8u;
    frame.data[0] = (uint8_t)((status << 4u) | motor_id);
    frame.data[1] = (uint8_t)(position >> 8u);
    frame.data[2] = (uint8_t)position;
    frame.data[3] = (uint8_t)(velocity >> 4u);
    frame.data[4] = (uint8_t)((velocity << 4u) | (torque >> 8u));
    frame.data[5] = (uint8_t)torque;
    frame.data[6] = 42u;
    frame.data[7] = 51u;
    return frame;
}

static double lsb(double lower, double upper, unsigned int bits) {
    return (upper - lower) / (double)((1u << bits) - 1u);
}

static void test_golden_and_special_frames(void) {
    oa_can_mit_command command = command_for(OA_CAN_MOTOR_DM4310);
    oa_can_frame frame;
    oa_can_encode_result encoded;
    const uint8_t expected_midpoint[8] = {0x7fu, 0xffu, 0x7fu, 0xf7u, 0xffu, 0x7fu, 0xf7u, 0xffu};
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
    size_t i;
    for (i = 0u; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        oa_can_mit_command command = command_for(cases[i].type);
        oa_can_frame frame;
        oa_can_encode_result encoded;
        command.position_rad = cases[i].position;
        command.velocity_rad_s = cases[i].velocity;
        command.kp = cases[i].kp;
        command.kd = cases[i].kd;
        command.torque_nm = cases[i].torque;
        CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_REJECT, &frame, &encoded) == OA_CAN_OK);
        CHECK(memcmp(frame.data, cases[i].bytes, sizeof(frame.data)) == 0);
    }
}

static void test_round_trip_properties(void) {
    const oa_can_motor_type types[] = {OA_CAN_MOTOR_DM8009, OA_CAN_MOTOR_DM4340, OA_CAN_MOTOR_DM4310};
    unsigned int seed = 0x5a17u;
    size_t t;
    for (t = 0u; t < sizeof(types) / sizeof(types[0]); ++t) {
        oa_can_limits limits;
        size_t i;
        CHECK(oa_can_motor_limits(types[t], &limits) == OA_CAN_OK);
        for (i = 0u; i < 1500u; ++i) {
            oa_can_mit_command command = command_for(types[t]);
            oa_can_frame frame;
            oa_can_frame feedback_frame_from_command;
            oa_can_encode_result encoded;
            oa_can_feedback decoded;
            const double u1 = (double)((seed = seed * 1103515245u + 12345u) & 0xffffu) / 65535.0;
            const double u2 = (double)((seed = seed * 1103515245u + 12345u) & 0xffffu) / 65535.0;
            const double u3 = (double)((seed = seed * 1103515245u + 12345u) & 0xffffu) / 65535.0;
            command.position_rad = limits.position_min_rad + u1 * (limits.position_max_rad - limits.position_min_rad);
            command.velocity_rad_s = limits.velocity_min_rad_s + u2 * (limits.velocity_max_rad_s - limits.velocity_min_rad_s);
            command.torque_nm = limits.torque_min_nm + u3 * (limits.torque_max_nm - limits.torque_min_nm);
            command.kp = 400.0 * u1;
            command.kd = 4.0 * u2;
            CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_REJECT, &frame, &encoded) == OA_CAN_OK);
            feedback_frame_from_command = feedback_frame(0x11u, 1u, 1u,
                ((uint32_t)frame.data[0] << 8u) | frame.data[1],
                ((uint32_t)frame.data[2] << 4u) | (frame.data[3] >> 4u),
                ((uint32_t)(frame.data[6] & 0x0fu) << 8u) | frame.data[7]);
            CHECK(oa_can_decode_feedback(&feedback_frame_from_command, 0x11u, 1u, types[t], &decoded) == OA_CAN_OK);
            CHECK(fabs(decoded.position_rad - command.position_rad) <= lsb(limits.position_min_rad, limits.position_max_rad, 16u));
            CHECK(fabs(decoded.velocity_rad_s - command.velocity_rad_s) <= lsb(limits.velocity_min_rad_s, limits.velocity_max_rad_s, 12u));
            CHECK(fabs(decoded.torque_nm - command.torque_nm) <= lsb(limits.torque_min_nm, limits.torque_max_nm, 12u));
        }
    }
}

static void test_range_and_malformed_inputs(void) {
    oa_can_mit_command command = command_for(OA_CAN_MOTOR_DM8009);
    oa_can_frame frame;
    oa_can_encode_result encoded;
    oa_can_feedback feedback;
    unsigned int byte;
    command.position_rad = 13.0;
    command.velocity_rad_s = 46.0;
    command.kp = 501.0;
    command.kd = 6.0;
    command.torque_nm = 55.0;
    CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_REJECT, &frame, &encoded) == OA_CAN_ERANGE);
    CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_SATURATE, &frame, &encoded) == OA_CAN_OK);
    CHECK(encoded.saturated_mask == 31u);
    CHECK(frame.data[0] == 0xffu && frame.data[7] == 0xffu);
    command.position_rad = NAN;
    CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_SATURATE, &frame, &encoded) == OA_CAN_EINVAL);
    command.position_rad = INFINITY;
    CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_REJECT, &frame, &encoded) == OA_CAN_EINVAL);
    command.position_rad = 0.0;
    command.abi_version = OA_CAN_ABI_VERSION + 1u;
    CHECK(oa_can_encode_mit(&command, OA_CAN_RANGE_REJECT, &frame, &encoded) == OA_CAN_EINVAL);
    command.abi_version = OA_CAN_ABI_VERSION;
    frame = feedback_frame(0x11u, 1u, 1u, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_decode_feedback(&frame, 0x11u, 1u, OA_CAN_MOTOR_DM4310, &feedback) == OA_CAN_OK);
    frame.dlc = 7u;
    CHECK(oa_can_decode_feedback(&frame, 0x11u, 1u, OA_CAN_MOTOR_DM4310, &feedback) == OA_CAN_EFRAME);
    frame.dlc = 8u;
    frame.can_id = 0x11u | OA_CAN_EFF_FLAG;
    CHECK(oa_can_decode_feedback(&frame, 0x11u, 1u, OA_CAN_MOTOR_DM4310, &feedback) == OA_CAN_EID);
    frame.can_id = 0x11u;
    frame.data[0] = 2u;
    CHECK(oa_can_decode_feedback(&frame, 0x11u, 1u, OA_CAN_MOTOR_DM4310, &feedback) == OA_CAN_EID);
    for (byte = 0u; byte < 256u; ++byte) {
        frame = feedback_frame(0x11u, (uint8_t)(byte & 0x0fu), (uint8_t)(byte >> 4u), byte * 257u,
                               (byte * 17u) & 0xfffu, (byte * 29u) & 0xfffu);
        (void)oa_can_decode_feedback(&frame, 0x11u, 1u, OA_CAN_MOTOR_DM4310, &feedback);
    }
}

static void test_status_nibbles(void) {
    oa_can_feedback feedback;
    uint8_t status;
    for (status = 0u; status < 16u; ++status) {
        oa_can_frame frame = feedback_frame(0x11u, 1u, status, 0x8000u, 0x800u, 0x800u);
        oa_can_status result = oa_can_decode_feedback(&frame, 0x11u, 1u, OA_CAN_MOTOR_DM4310, &feedback);
        if (status == 0u || status == 1u) CHECK(result == OA_CAN_OK);
        else if (status >= 8u && status <= 14u) {
            CHECK(result == OA_CAN_EFAULT);
            CHECK(feedback.status_nibble == status && feedback.mos_temperature_c == 42u && feedback.rotor_temperature_c == 51u);
        } else CHECK(result == OA_CAN_EFRAME);
    }
}

static oa_can_arm_manifest manifest_two(void) {
    oa_can_arm_manifest manifest;
    size_t i;
    (void)memset(&manifest, 0, sizeof(manifest));
    manifest.struct_size = (uint32_t)sizeof(manifest);
    manifest.abi_version = OA_CAN_ABI_VERSION;
    manifest.motor_count = 2u;
    for (i = 0u; i < 2u; ++i) {
        oa_can_motor_manifest *motor = &manifest.motors[i];
        motor->struct_size = (uint32_t)sizeof(*motor);
        motor->abi_version = OA_CAN_ABI_VERSION;
        motor->joint_index = (uint8_t)i;
        motor->expected_motor_id = (uint8_t)(i + 1u);
        motor->send_id = (uint16_t)(i + 1u);
        motor->receive_id = (uint16_t)(0x11u + i);
        motor->motor_type = OA_CAN_MOTOR_DM4310;
        motor->mapping.struct_size = (uint32_t)sizeof(motor->mapping);
        motor->mapping.abi_version = OA_CAN_ABI_VERSION;
        motor->mapping.position_scale = 1.0;
        motor->mapping.velocity_scale = 1.0;
        motor->mapping.torque_scale = 1.0;
        (void)snprintf(motor->joint_name, sizeof(motor->joint_name), "joint%u", (unsigned int)(i + 1u));
    }
    return manifest;
}

static void test_manifest_probe_and_lifecycle(void) {
    oa_can_arm_manifest manifest = manifest_two();
    oa_can_fake *fake = NULL;
    oa_can_transport transport;
    oa_can_probe_report report;
    oa_can_frame frame;
    CHECK(oa_can_validate_manifest(&manifest) == OA_CAN_OK);
    manifest.motors[1].receive_id = manifest.motors[0].receive_id;
    CHECK(oa_can_validate_manifest(&manifest) == OA_CAN_EINVAL);
    manifest = manifest_two();
    CHECK(oa_can_fake_create(8u, &fake) == OA_CAN_OK);
    CHECK(oa_can_fake_get_lifecycle(fake) == OA_CAN_FAKE_DISABLED);
    CHECK(oa_can_fake_torque_enabled(fake) == 0);
    CHECK(oa_can_fake_transport(fake, &transport) == OA_CAN_OK);
    frame = feedback_frame(0x11u, 1u, 1u, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame) == OA_CAN_OK);
    frame = feedback_frame(0x12u, 2u, 1u, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame) == OA_CAN_OK);
    CHECK(oa_can_probe_expected(&transport, &manifest, &report) == OA_CAN_OK);
    CHECK(report.expected_mask == 3u && report.fresh_mask == 3u && report.refresh_frames_sent == 2u);
    CHECK(oa_can_fake_sent_count(fake) == 2u && oa_can_fake_get_lifecycle(fake) == OA_CAN_FAKE_PROBED);
    oa_can_fake_destroy(fake);
    fake = NULL;
    CHECK(oa_can_fake_create(8u, &fake) == OA_CAN_OK);
    CHECK(oa_can_fake_transport(fake, &transport) == OA_CAN_OK);
    CHECK(oa_can_probe_expected(&transport, &manifest, &report) == OA_CAN_ETIMEOUT);
    CHECK(report.expected_mask == 3u && report.fresh_mask == 0u);
    oa_can_fake_destroy(fake);
    fake = NULL;
    CHECK(oa_can_fake_create(8u, &fake) == OA_CAN_OK);
    CHECK(oa_can_fake_transport(fake, &transport) == OA_CAN_OK);
    frame = feedback_frame(0x11u, 1u, 8u, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame) == OA_CAN_OK);
    frame = feedback_frame(0x11u, 1u, 1u, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame) == OA_CAN_OK);
    frame = feedback_frame(0x12u, 2u, 1u, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame) == OA_CAN_OK);
    CHECK(oa_can_probe_expected(&transport, &manifest, &report) == OA_CAN_EFAULT);
    CHECK(report.fault_mask == 1u && report.duplicate_mask == 1u);
    oa_can_fake_destroy(fake);
    fake = NULL;
    CHECK(oa_can_fake_create(8u, &fake) == OA_CAN_OK);
    CHECK(oa_can_fake_transport(fake, &transport) == OA_CAN_OK);
    frame = feedback_frame(0x11u, 9u, 1u, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame) == OA_CAN_OK);
    frame = feedback_frame(0x40u, 9u, 1u, 0x8000u, 0x800u, 0x800u);
    CHECK(oa_can_fake_enqueue_feedback(fake, &frame) == OA_CAN_OK);
    CHECK(oa_can_probe_expected(&transport, &manifest, &report) == OA_CAN_ETIMEOUT);
    CHECK(report.invalid_frames == 1u && report.unexpected_frames == 1u);
    oa_can_fake_destroy(fake);
}

int main(void) {
    test_golden_and_special_frames();
    test_all_motor_golden_vectors();
    test_round_trip_properties();
    test_range_and_malformed_inputs();
    test_status_nibbles();
    test_manifest_probe_and_lifecycle();
    if (failures != 0) {
        (void)fprintf(stderr, "%d test assertion(s) failed\n", failures);
        return 1;
    }
    (void)puts("openarm_can tests passed");
    return 0;
}
