/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_control.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace {

template <typename T>
void init(T &record) noexcept {
    record = {};
    record.struct_size = sizeof(record);
    record.abi_version = OA_CONTROL_ABI_V1;
}

oa_manifest_config standard_config() noexcept {
    static constexpr std::array<double, 7> left_lower{
        -3.490659, -3.3161253267948965, -1.570796, 0.0,
        -1.570796, -0.785398, -1.570796};
    static constexpr std::array<double, 7> left_upper{
        1.396263, 0.17453267320510335, 1.570796, 2.443461,
        1.570796, 0.785398, 1.570796};
    static constexpr std::array<double, 7> right_lower{
        -1.396263, -0.17453267320510335, -1.570796, 0.0,
        -1.570796, -0.785398, -1.570796};
    static constexpr std::array<double, 7> right_upper{
        3.490659, 3.3161253267948965, 1.570796, 2.443461,
        1.570796, 0.785398, 1.570796};

    oa_manifest_config config{};
    init(config);
    config.manifest_revision = 41U;
    config.model_revision = 7U;
    for (std::size_t side = 0; side < 2U; ++side) {
        init(config.arm[side]);
        std::snprintf(config.arm[side].bus_name, sizeof(config.arm[side].bus_name),
                      "virtual_arm_%zu", side);
        for (std::size_t joint = 0; joint < 7U; ++joint) {
            auto &motor = config.arm[side].motor[joint];
            init(motor);
            motor.motor_type = joint < 2U ? OA_MOTOR_DM8009
                                          : (joint < 4U ? OA_MOTOR_DM4340 : OA_MOTOR_DM4310);
            motor.joint_index = static_cast<std::uint32_t>(joint);
            motor.send_id = static_cast<std::uint32_t>(joint + 1U);
            motor.receive_id = static_cast<std::uint32_t>(joint + 0x11U);
            motor.embedded_motor_id = static_cast<std::uint32_t>(joint + 1U);
            motor.control_mode = 1U;
            motor.bitrate = 1000000U;
            motor.timeout_ticks = 1000U;
            motor.hardware_version = 1U;
            motor.software_version = 1U;
            motor.firmware_subversion = 1U;
            motor.q_scale = ((side + joint) & 1U) == 0U ? 1.0 : -1.0;
            motor.q_offset_rad = 0.125;
            motor.lower_rad = side == 0U ? left_lower[joint] : right_lower[joint];
            motor.upper_rad = side == 0U ? left_upper[joint] : right_upper[joint];
            motor.max_velocity_rad_s = 1.0;
            motor.max_acceleration_rad_s2 = 2.0;
            motor.max_jerk_rad_s3 = 10.0;
            motor.pmax_rad = 12.5;
            if (motor.motor_type == OA_MOTOR_DM8009) {
                motor.vmax_rad_s = 45.0;
                motor.tmax_nm = 54.0;
                motor.gear_ratio = 9.0;
            } else if (motor.motor_type == OA_MOTOR_DM4340) {
                motor.vmax_rad_s = 10.0;
                motor.tmax_nm = 28.0;
                motor.gear_ratio = 40.0;
            } else {
                motor.vmax_rad_s = 30.0;
                motor.tmax_nm = 10.0;
                motor.gear_ratio = 10.0;
            }
            motor.direction = motor.q_scale > 0.0 ? 1 : -1;
            std::snprintf(motor.serial, sizeof(motor.serial), "VIRTUAL-%zu-%zu", side, joint);
            std::snprintf(motor.joint_name, sizeof(motor.joint_name),
                          "openarm_%s_joint%zu", side == 0U ? "left" : "right", joint + 1U);
        }
    }
    return config;
}

}

extern "C" oa_control_status oa_manifest_create_openarm_v10_virtual(oa_manifest **out) {
    const auto config = standard_config();
    return oa_manifest_create(&config, out);
}
