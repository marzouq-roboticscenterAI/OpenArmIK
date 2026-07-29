/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_control.h"
#include "control_core.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <thread>
#include <vector>

#ifdef __linux__
#include <unistd.h>
#endif

extern "C" void oa_control_test_fail_controller_create_after(std::int32_t checkpoints);
extern "C" std::size_t oa_control_test_active_controller_count(void);
extern "C" std::size_t oa_control_test_active_manifest_count(void);
extern "C" std::size_t oa_control_test_active_plan_count(void);

namespace {

[[noreturn]] void fail(const char *expression, const char *file, int line) {
    std::fprintf(stderr, "%s:%d: check failed: %s\n", file, line, expression);
    std::exit(1);
}

#define CHECK(expression) ((expression) ? static_cast<void>(0) : fail(#expression, __FILE__, __LINE__))

template <typename T>
void init(T &record) {
    record = {};
    record.struct_size = sizeof(record);
    record.abi_version = OA_CONTROL_ABI_V1;
}

#ifndef OA_CONTROL_TEST_SANITIZED
std::uint64_t resident_bytes() {
#ifdef __linux__
    std::FILE *const stream = std::fopen("/proc/self/statm", "r");
    if (stream == nullptr) {
        return 0U;
    }
    unsigned long total_pages = 0UL;
    unsigned long resident_pages = 0UL;
    const int fields = std::fscanf(stream, "%lu %lu", &total_pages, &resident_pages);
    std::fclose(stream);
    if (fields != 2) {
        return 0U;
    }
    const long page_bytes = sysconf(_SC_PAGESIZE);
    return page_bytes > 0
               ? static_cast<std::uint64_t>(resident_pages) *
                     static_cast<std::uint64_t>(page_bytes)
               : 0U;
#else
    return 0U;
#endif
}
#endif

oa_manifest_config valid_config() {
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
                      "vcan%zu", side);
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
            std::snprintf(motor.serial, sizeof(motor.serial), "SIM-%zu-%zu", side, joint);
            std::snprintf(motor.joint_name, sizeof(motor.joint_name),
                          "openarm_%s_joint%zu", side == 0U ? "left" : "right", joint + 1U);
        }
    }
    return config;
}

oa_controller_options virtual_options(std::uint32_t collision_policy =
                                          OA_COLLISION_VIRTUAL_UNCHECKED) {
    oa_controller_options options{};
    init(options);
    options.backend = OA_BACKEND_VIRTUAL;
    options.collision_policy = collision_policy;
    options.cycle_ns = 10000000U;
    options.feedback_timeout_ns = 50000000U;
    options.max_cross_bus_skew_ns = 1000000U;
    options.collision_scene_revision = 1U;
    return options;
}

struct Fixture {
    oa_manifest *manifest{};
    oa_controller *controller{};
    oa_snapshot state{};

    explicit Fixture(const std::uint32_t collision_policy = OA_COLLISION_VIRTUAL_UNCHECKED,
                     const bool arm_now = true) {
        auto config = valid_config();
        CHECK(oa_manifest_create(&config, &manifest) == OA_CONTROL_OK);
        auto options = virtual_options(collision_policy);
        CHECK(oa_controller_create(manifest, &options, &controller) == OA_CONTROL_OK);
        oa_verify_report verify{};
        init(verify);
        CHECK(oa_controller_open_and_verify(controller, &verify) == OA_CONTROL_OK);
        CHECK(verify.verified_mask == 3U);
        init(state);
        CHECK(oa_controller_snapshot(controller, &state) == OA_CONTROL_OK);
        if (arm_now) {
            oa_arm_challenge challenge{};
            init(challenge);
            CHECK(oa_controller_get_arm_challenge(controller, &challenge) == OA_CONTROL_OK);
            CHECK(oa_controller_arm(controller, &challenge) == OA_CONTROL_OK);
            init(state);
            CHECK(oa_controller_snapshot(controller, &state) == OA_CONTROL_OK);
        }
    }

    ~Fixture() {
        oa_controller_destroy(controller);
        oa_manifest_destroy(manifest);
    }
};

oa_motion_plan *joint_plan(Fixture &fixture, const std::uint32_t side,
                           const std::uint32_t joint, const double target,
                           const std::uint64_t expiry = 5000000000ULL) {
    oa_joint_move move{};
    init(move);
    move.expiry_ns = expiry;
    move.required_feedback_seq = fixture.state.arm[side].feedback_seq;
    move.side = side;
    move.joint = joint;
    move.target_rad = target;
    move.velocity_scale = 0.5;
    move.acceleration_scale = 0.5;
    move.jerk_scale = 0.5;
    move.position_tol_rad = 5.0e-4;
    move.velocity_tol_rad_s = 2.0e-2;
    oa_motion_plan *plan = nullptr;
    CHECK(oa_controller_plan_joint(fixture.controller, &move, &plan) == OA_CONTROL_OK);
    return plan;
}

oa_motion_plan_report plan_report(oa_motion_plan *plan) {
    oa_motion_plan_report report{};
    init(report);
    CHECK(oa_motion_plan_get_report(plan, &report) == OA_CONTROL_OK);
    return report;
}

oa_paired_tcp_move paired_move(const Fixture &fixture) {
    oa_paired_tcp_move move{};
    init(move);
    move.expiry_ns = 60000000000ULL;
    move.required_feedback_seq[0] = fixture.state.arm[0].feedback_seq;
    move.required_feedback_seq[1] = fixture.state.arm[1].feedback_seq;
    move.left_tcp_m[0] = 0.20;
    move.left_tcp_m[1] = 0.30;
    move.left_tcp_m[2] = 0.85;
    move.right_tcp_m[0] = 0.20;
    move.right_tcp_m[1] = -0.30;
    move.right_tcp_m[2] = 0.85;
    move.velocity_scale = 0.5;
    move.acceleration_scale = 0.5;
    move.jerk_scale = 0.5;
    move.tcp_tol_m = 1.0e-3;
    move.max_branch_step_rad = 2.0;
    move.min_singular_value = 0.0;
    move.collision_scene_revision = 1U;
    return move;
}

std::uint64_t execute(Fixture &fixture, oa_motion_plan *plan,
                      const oa_motion_plan_report &report,
                      const std::uint64_t extra_ns = 1000000000ULL) {
    oa_execute_request request{};
    init(request);
    request.start_ns = 0U;
    request.expiry_ns = report.duration_ns + extra_ns;
    request.producer_deadline_ns = request.expiry_ns + extra_ns;
    request.stop_kind = OA_STOP_DISABLE;
    std::uint64_t command = 0U;
    CHECK(oa_controller_execute(fixture.controller, plan, &request, &command) == OA_CONTROL_OK);
    CHECK(command != 0U);
    return command;
}

bool has_event(oa_controller *controller, const std::uint32_t wanted,
               const std::uint64_t command_id) {
    for (;;) {
        oa_event event{};
        init(event);
        const oa_control_status status = oa_controller_poll_event(controller, 0U, &event);
        if (status == OA_CONTROL_ETIMEOUT) {
            return false;
        }
        CHECK(status == OA_CONTROL_OK);
        if (event.kind == wanted && event.command_id == command_id) {
            return true;
        }
    }
}

bool take_event(oa_controller *controller, const std::uint32_t wanted,
                const std::uint64_t command_id, oa_event &matched) {
    for (;;) {
        oa_event event{};
        init(event);
        const oa_control_status status = oa_controller_poll_event(controller, 0U, &event);
        if (status == OA_CONTROL_ETIMEOUT) {
            return false;
        }
        CHECK(status == OA_CONTROL_OK);
        if (event.kind == wanted && event.command_id == command_id) {
            matched = event;
            return true;
        }
    }
}

void check_arm_payload_equal(const oa_arm_snapshot &actual,
                             const oa_arm_snapshot &expected) {
    CHECK(actual.feedback_seq == expected.feedback_seq);
    CHECK(actual.t_ns == expected.t_ns);
    CHECK(actual.fresh_mask == expected.fresh_mask);
    CHECK(actual.fault_mask == expected.fault_mask);
    for (std::size_t joint = 0U; joint < 7U; ++joint) {
        CHECK(actual.q[joint] == expected.q[joint]);
        CHECK(actual.dq[joint] == expected.dq[joint]);
        CHECK(actual.tau[joint] == expected.tau[joint]);
        CHECK(actual.raw_q[joint] == expected.raw_q[joint]);
        CHECK(actual.raw_dq[joint] == expected.raw_dq[joint]);
        CHECK(actual.raw_tau[joint] == expected.raw_tau[joint]);
        CHECK(actual.status[joint] == expected.status[joint]);
        CHECK(actual.mos_c[joint] == expected.mos_c[joint]);
        CHECK(actual.coil_c[joint] == expected.coil_c[joint]);
    }
}

oa_execute_request request_for(const oa_motion_plan_report &report,
                               std::uint64_t start_ns = 0U);

void set_feedback_delay(Fixture &fixture, const std::uint64_t delay_ns) {
    for (std::uint32_t side = OA_LEFT; side <= OA_RIGHT; ++side) {
        oa_sim_fault fault{};
        init(fault);
        fault.side = side;
        fault.feedback_delay_ns = delay_ns;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
    }
}

oa_motion_plan *start_delayed_motion(Fixture &fixture, const std::uint32_t stop_kind,
                                     const std::uint64_t producer_deadline_ns,
                                     std::uint64_t &command_id) {
    oa_motion_plan *plan = joint_plan(fixture, OA_LEFT, 0U, 0.3);
    const auto report = plan_report(plan);
    set_feedback_delay(fixture, 20000000U);
    auto request = request_for(report);
    request.producer_deadline_ns = producer_deadline_ns;
    request.stop_kind = stop_kind;
    CHECK(oa_controller_execute(fixture.controller, plan, &request, &command_id) == OA_CONTROL_OK);
    CHECK(oa_controller_advance(fixture.controller, 10000000U) == OA_CONTROL_OK);
    CHECK(oa_controller_advance(fixture.controller, 20000000U) == OA_CONTROL_OK);
    return plan;
}

oa_snapshot controller_snapshot(Fixture &fixture) {
    oa_snapshot state{};
    init(state);
    CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_OK);
    return state;
}

void check_stop_payload(const oa_snapshot &state, const std::uint32_t lifecycle,
                        const bool enabled, const std::int32_t fault_side = -1,
                        const std::int32_t fault_joint = -1) {
    CHECK(state.lifecycle == lifecycle);
    for (std::size_t side = 0U; side < 2U; ++side) {
        CHECK(state.arm[side].fresh_mask == 0x7fU);
        for (std::size_t joint = 0U; joint < 7U; ++joint) {
            const bool faulted = static_cast<std::int32_t>(side) == fault_side &&
                                 static_cast<std::int32_t>(joint) == fault_joint;
            CHECK(state.arm[side].status[joint] ==
                  (faulted ? 8U : (enabled ? 1U : 0U)));
            CHECK(std::abs(state.arm[side].dq[joint]) <= 1.2e-2);
        }
    }
}

void reset_and_reverify(Fixture &fixture) {
    for (;;) {
        oa_event event{};
        init(event);
        if (oa_controller_poll_event(fixture.controller, 0U, &event) == OA_CONTROL_ETIMEOUT) {
            break;
        }
    }
    oa_arm_challenge challenge{};
    init(challenge);
    CHECK(oa_controller_get_arm_challenge(fixture.controller, &challenge) == OA_CONTROL_OK);
    oa_reset_request reset{};
    init(reset);
    reset.verify_epoch = challenge.verify_epoch;
    reset.nonce = challenge.nonce;
    CHECK(oa_controller_reset_fault(fixture.controller, &reset) == OA_CONTROL_OK);
    oa_snapshot unavailable{};
    init(unavailable);
    CHECK(oa_controller_snapshot(fixture.controller, &unavailable) == OA_CONTROL_ESTATE);
    oa_verify_report verify{};
    init(verify);
    CHECK(oa_controller_open_and_verify(fixture.controller, &verify) == OA_CONTROL_OK);
    check_stop_payload(controller_snapshot(fixture), OA_LIFECYCLE_DISARMED, false);
}

void test_manifest_validation() {
    auto config = valid_config();
    oa_manifest *manifest = nullptr;
    CHECK(oa_manifest_create(&config, &manifest) == OA_CONTROL_OK);
    oa_manifest_destroy(manifest);

    config.arm[0].motor[0].gear_ratio = 10.0;
    CHECK(oa_manifest_create(&config, &manifest) == OA_CONTROL_EINVAL);
    config = valid_config();
    std::strcpy(config.arm[1].bus_name, config.arm[0].bus_name);
    CHECK(oa_manifest_create(&config, &manifest) == OA_CONTROL_EINVAL);
    config = valid_config();
    config.arm[0].motor[2].q_scale = 40.0;
    CHECK(oa_manifest_create(&config, &manifest) == OA_CONTROL_EINVAL);
    config = valid_config();
    config.arm[1].motor[4].serial[0] = '\0';
    CHECK(oa_manifest_create(&config, &manifest) == OA_CONTROL_EINVAL);
    config = valid_config();
    config.arm[0].motor[0].pmax_rad = 11.0;
    CHECK(oa_manifest_create(&config, &manifest) == OA_CONTROL_EINVAL);
    config = valid_config();
    config.arm[0].motor[0].q_offset_rad = 12.5;
    CHECK(oa_manifest_create(&config, &manifest) == OA_CONTROL_EINVAL);
}

void test_abi_and_physical_gate() {
    auto config = valid_config();
    config.abi_version = 99U;
    oa_manifest *manifest = nullptr;
    CHECK(oa_manifest_create(&config, &manifest) == OA_CONTROL_EABI);

    config = valid_config();
    CHECK(oa_manifest_create(&config, &manifest) == OA_CONTROL_OK);
    auto options = virtual_options();
    options.backend = OA_BACKEND_PHYSICAL;
    options.collision_policy = OA_COLLISION_REJECT_ALL;
    oa_controller *controller = nullptr;
    CHECK(oa_controller_create(manifest, &options, &controller) == OA_CONTROL_OK);
    oa_verify_report report{};
    init(report);
    CHECK(oa_controller_open_and_verify(controller, &report) == OA_CONTROL_EUNSUPPORTED);
    CHECK(report.failure_mask == 3U);
    oa_controller_destroy(controller);
    oa_manifest_destroy(manifest);

    std::array<unsigned char, sizeof(oa_snapshot)> canary{};
    canary.fill(0xa5U);
    auto *short_output = reinterpret_cast<oa_snapshot *>(canary.data());
    short_output->struct_size = 1U;
    short_output->abi_version = OA_CONTROL_ABI_V1;
    const auto before = canary;
    CHECK(oa_controller_snapshot(nullptr, short_output) == OA_CONTROL_EINVAL);
    CHECK(canary == before);

    struct ExtendedSnapshot {
        oa_snapshot value;
        std::array<unsigned char, 32> trailing;
    } extended{};
    Fixture fixture;
    init(extended.value);
    extended.trailing.fill(0x5aU);
    CHECK(oa_controller_snapshot(fixture.controller, &extended.value) == OA_CONTROL_OK);
    CHECK(std::all_of(extended.trailing.begin(), extended.trailing.end(),
                      [](const unsigned char value) { return value == 0x5aU; }));
}

void test_mapping_kinematics_and_joint_convergence() {
    Fixture fixture;
    for (std::size_t side = 0; side < 2U; ++side) {
        CHECK(fixture.state.arm[side].fresh_mask == 0x7fU);
        for (std::size_t joint = 0; joint < 7U; ++joint) {
            CHECK(std::abs(fixture.state.arm[side].q[joint]) < 3.0e-4);
            const double scale = ((side + joint) & 1U) == 0U ? 1.0 : -1.0;
            CHECK(std::abs(fixture.state.arm[side].raw_q[joint] + 0.125 / scale) < 3.0e-4);
        }
        oa_arm_kinematics kinematics{};
        init(kinematics);
        CHECK(oa_controller_get_kinematics(
                  fixture.controller, static_cast<std::uint32_t>(side),
                  fixture.state.arm[side].feedback_seq, &kinematics) == OA_CONTROL_OK);
        CHECK(std::isfinite(kinematics.tcp_xyz_m[0]));
        CHECK(std::isfinite(kinematics.joint_xyz_m[6][2]));
    }

    oa_motion_plan *plan = joint_plan(fixture, OA_LEFT, 1U, -0.1);
    const auto report = plan_report(plan);
    CHECK(report.kind == OA_PLAN_JOINT);
    for (std::size_t joint = 0; joint < 7U; ++joint) {
        if (joint != 1U) {
            CHECK(report.target_q[0][joint] == fixture.state.arm[0].q[joint]);
        }
    }
    const auto command = execute(fixture, plan, report);
    bool lag_observed = false;
    for (std::uint64_t now = 10000000U; now <= report.duration_ns + 900000000U;
        now += 10000000U) {
        CHECK(oa_controller_advance(fixture.controller, now) == OA_CONTROL_OK);
        if (now < report.duration_ns) {
            oa_snapshot measured{};
            init(measured);
            CHECK(oa_controller_snapshot(fixture.controller, &measured) == OA_CONTROL_OK);
            for (std::size_t side = 0; side < 2U; ++side) {
                for (std::size_t joint = 0; joint < 7U; ++joint) {
                    const double scale = ((side + joint) & 1U) == 0U ? 1.0 : -1.0;
                    CHECK(std::abs(measured.arm[side].q[joint] -
                                   (scale * measured.arm[side].raw_q[joint] + 0.125)) <
                          1.0e-12);
                    CHECK(std::abs(measured.arm[side].dq[joint] -
                                   scale * measured.arm[side].raw_dq[joint]) < 1.0e-12);
                    CHECK(std::abs(measured.arm[side].tau[joint] -
                                   measured.arm[side].raw_tau[joint] / scale) < 1.0e-12);
                }
            }
        } else if (!lag_observed) {
            oa_snapshot measured{};
            init(measured);
            CHECK(oa_controller_snapshot(fixture.controller, &measured) == OA_CONTROL_OK);
            lag_observed = std::abs(measured.arm[0].q[1] - report.target_q[0][1]) >
                           5.0e-4;
        }
    }
    CHECK(lag_observed);
    CHECK(has_event(fixture.controller, OA_EVENT_COMPLETED, command));
    init(fixture.state);
    CHECK(oa_controller_snapshot(fixture.controller, &fixture.state) == OA_CONTROL_OK);
    CHECK(std::abs(fixture.state.arm[0].q[1] + 0.1) < 5.0e-4);
    CHECK(std::abs(fixture.state.arm[0].raw_q[1] - 0.225) < 5.0e-4);
    oa_motion_plan_destroy(plan);
}

void test_frozen_encoder_never_completes() {
    Fixture fixture;
    oa_motion_plan *plan = joint_plan(fixture, OA_LEFT, 0U, 0.2);
    const auto report = plan_report(plan);
    oa_sim_fault fault{};
    init(fault);
    fault.side = OA_LEFT;
    fault.freeze_mask = 1U;
    CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
    const auto command = execute(fixture, plan, report, 50000000U);
    for (std::uint64_t now = 10000000U; now <= report.duration_ns + 50000000U;
         now += 10000000U) {
        CHECK(oa_controller_advance(fixture.controller, now) == OA_CONTROL_OK);
    }
    CHECK(!has_event(fixture.controller, OA_EVENT_COMPLETED, command));
    CHECK(oa_controller_advance(fixture.controller, report.duration_ns + 60000000U) ==
          OA_CONTROL_ETIMEOUT);
    oa_snapshot state{};
    init(state);
    CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_OK);
    CHECK(state.lifecycle == OA_LIFECYCLE_FAULT);
    CHECK(std::abs(state.arm[0].q[0]) < 3.0e-4);
    oa_motion_plan_destroy(plan);
}

void test_stale_feedback_and_paired_fault_stop() {
    {
        Fixture fixture;
        oa_sim_fault fault{};
        init(fault);
        fault.side = OA_RIGHT;
        fault.drop_mask = 1U;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 10000000U) == OA_CONTROL_ESTALE);
        oa_snapshot state{};
        init(state);
        CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_OK);
        CHECK(state.lifecycle == OA_LIFECYCLE_FAULT);
        oa_arm_challenge challenge{};
        init(challenge);
        CHECK(oa_controller_get_arm_challenge(fixture.controller, &challenge) == OA_CONTROL_OK);
        oa_reset_request reset{};
        init(reset);
        reset.verify_epoch = challenge.verify_epoch;
        reset.nonce = challenge.nonce;
        CHECK(oa_controller_reset_fault(fixture.controller, &reset) == OA_CONTROL_OK);
        CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_ESTATE);
        oa_verify_report reverify{};
        init(reverify);
        CHECK(oa_controller_open_and_verify(fixture.controller, &reverify) == OA_CONTROL_OK);
        init(state);
        CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_OK);
        CHECK(state.lifecycle == OA_LIFECYCLE_DISARMED);
        CHECK(state.arm[1].fresh_mask == 0x7fU);
    }
    {
        Fixture fixture;
        auto move = paired_move(fixture);
        oa_motion_plan *plan = nullptr;
        CHECK(oa_controller_plan_paired_tcp(fixture.controller, &move, &plan) == OA_CONTROL_OK);
        const auto report = plan_report(plan);
        (void)execute(fixture, plan, report);
        oa_sim_fault fault{};
        init(fault);
        fault.side = OA_RIGHT;
        fault.fault_mask = 1U << 3U;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 10000000U) == OA_CONTROL_EFAULT);
        oa_snapshot state{};
        init(state);
        CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_OK);
        CHECK(state.lifecycle == OA_LIFECYCLE_FAULT);
        CHECK(state.arm[1].status[3] == 8U);
        CHECK((state.arm[1].fault_mask & (1U << 3U)) != 0U);
        CHECK(has_event(fixture.controller, OA_EVENT_FAULTED, 1U));
        oa_motion_plan_destroy(plan);
    }
    {
        Fixture fixture;
        auto move = paired_move(fixture);
        oa_motion_plan *plan = nullptr;
        CHECK(oa_controller_plan_paired_tcp(fixture.controller, &move, &plan) == OA_CONTROL_OK);
        const auto report = plan_report(plan);
        auto request = request_for(report);
        request.producer_deadline_ns = 50000000U;
        std::uint64_t command = 0U;
        CHECK(oa_controller_execute(fixture.controller, plan, &request, &command) == OA_CONTROL_OK);
        for (std::uint64_t now = 10000000U; now <= 40000000U; now += 10000000U) {
            CHECK(oa_controller_advance(fixture.controller, now) == OA_CONTROL_OK);
        }
        CHECK(oa_controller_heartbeat(fixture.controller, command, 500000000U) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 50000000U) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 60000000U) == OA_CONTROL_OK);
        CHECK(oa_controller_stop(fixture.controller, OA_STOP_DISABLE) == OA_CONTROL_OK);
        CHECK(has_event(fixture.controller, OA_EVENT_ABORTED, command));
        oa_motion_plan_destroy(plan);
    }
}

void test_collision_rejection_and_limits() {
    Fixture fixture(OA_COLLISION_REJECT_ALL);
    oa_joint_move move{};
    init(move);
    move.expiry_ns = 1000000000U;
    move.required_feedback_seq = fixture.state.arm[0].feedback_seq;
    move.side = OA_LEFT;
    move.joint = 0U;
    move.target_rad = 0.1;
    move.velocity_scale = 1.0;
    move.acceleration_scale = 1.0;
    move.jerk_scale = 1.0;
    move.position_tol_rad = 1.0e-3;
    move.velocity_tol_rad_s = 1.0e-3;
    oa_motion_plan *plan = nullptr;
    CHECK(oa_controller_plan_joint(fixture.controller, &move, &plan) == OA_CONTROL_ECOLLISION);

    Fixture allowed;
    move.required_feedback_seq = allowed.state.arm[0].feedback_seq;
    move.target_rad = 2.0;
    CHECK(oa_controller_plan_joint(allowed.controller, &move, &plan) == OA_CONTROL_ELIMIT);
    move.target_rad = std::numeric_limits<double>::quiet_NaN();
    CHECK(oa_controller_plan_joint(allowed.controller, &move, &plan) == OA_CONTROL_EINVAL);
}

void test_paired_tcp_measured_convergence() {
    Fixture fixture;
    auto move = paired_move(fixture);
    oa_motion_plan *plan = nullptr;
    CHECK(oa_controller_plan_paired_tcp(fixture.controller, &move, &plan) == OA_CONTROL_OK);
    const auto report = plan_report(plan);
    CHECK(report.kind == OA_PLAN_PAIRED_TCP);
    CHECK(report.collision_checked == 0U);
    CHECK(report.tcp_residual_m[0] < 1.0e-5);
    CHECK(report.tcp_residual_m[1] < 1.0e-5);
    const auto command = execute(fixture, plan, report, 5000000000ULL);
    for (std::uint64_t now = 10000000U; now <= report.duration_ns + 4500000000ULL;
         now += 10000000U) {
        CHECK(oa_controller_advance(fixture.controller, now) == OA_CONTROL_OK);
    }
    CHECK(has_event(fixture.controller, OA_EVENT_COMPLETED, command));
    init(fixture.state);
    CHECK(oa_controller_snapshot(fixture.controller, &fixture.state) == OA_CONTROL_OK);
    for (std::size_t side = 0; side < 2U; ++side) {
        oa_arm_kinematics kinematics{};
        init(kinematics);
        CHECK(oa_controller_get_kinematics(
                  fixture.controller, static_cast<std::uint32_t>(side),
                  fixture.state.arm[side].feedback_seq, &kinematics) == OA_CONTROL_OK);
        const double *target = side == 0U ? move.left_tcp_m : move.right_tcp_m;
        double squared = 0.0;
        for (std::size_t axis = 0; axis < 3U; ++axis) {
            const double error = kinematics.tcp_xyz_m[axis] - target[axis];
            squared += error * error;
        }
        CHECK(std::sqrt(squared) < move.tcp_tol_m);
    }
    oa_motion_plan_destroy(plan);
}

void test_faults_gate_arming_and_idle_motion() {
    for (std::uint32_t status_code = 8U; status_code <= 14U; ++status_code) {
        Fixture fixture(OA_COLLISION_VIRTUAL_UNCHECKED, false);
        oa_sim_fault fault{};
        init(fault);
        fault.side = OA_LEFT;
        fault.fault_mask = 1U;
        fault.fault_status = status_code;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
        oa_arm_challenge challenge{};
        init(challenge);
        CHECK(oa_controller_get_arm_challenge(fixture.controller, &challenge) == OA_CONTROL_EFAULT);
        oa_snapshot state{};
        init(state);
        CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_OK);
        CHECK(state.lifecycle == OA_LIFECYCLE_FAULT);
        CHECK(state.arm[0].status[0] == status_code);
    }
    {
        Fixture fixture(OA_COLLISION_VIRTUAL_UNCHECKED, false);
        oa_arm_challenge challenge{};
        init(challenge);
        CHECK(oa_controller_get_arm_challenge(fixture.controller, &challenge) == OA_CONTROL_OK);
        oa_sim_fault fault{};
        init(fault);
        fault.side = OA_RIGHT;
        fault.fault_mask = 2U;
        fault.fault_status = 10U;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
        CHECK(oa_controller_arm(fixture.controller, &challenge) == OA_CONTROL_EFAULT);
    }
    {
        Fixture fixture;
        oa_sim_fault fault{};
        init(fault);
        fault.side = OA_LEFT;
        fault.fault_mask = 4U;
        fault.fault_status = 14U;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
        oa_motion_plan *plan = nullptr;
        oa_joint_move move{};
        init(move);
        move.expiry_ns = 1000000000U;
        move.required_feedback_seq = fixture.state.arm[0].feedback_seq;
        move.side = OA_LEFT;
        move.joint = 0U;
        move.target_rad = 0.1;
        move.velocity_scale = 1.0;
        move.acceleration_scale = 1.0;
        move.jerk_scale = 1.0;
        move.position_tol_rad = 5.0e-4;
        move.velocity_tol_rad_s = 2.0e-2;
        CHECK(oa_controller_plan_joint(fixture.controller, &move, &plan) == OA_CONTROL_EFAULT);
    }
}

oa_execute_request request_for(const oa_motion_plan_report &report,
                               const std::uint64_t start_ns) {
    oa_execute_request request{};
    init(request);
    request.start_ns = start_ns;
    request.expiry_ns = start_ns + report.duration_ns + 1000000000ULL;
    request.producer_deadline_ns = request.expiry_ns;
    request.stop_kind = OA_STOP_DISABLE;
    return request;
}

void test_plan_ownership_and_start_drift() {
    Fixture first;
    Fixture second;
    oa_motion_plan *plan = joint_plan(first, OA_LEFT, 0U, 0.1);
    const auto report = plan_report(plan);
    auto request = request_for(report);
    std::uint64_t command = 0U;
    CHECK(oa_controller_execute(second.controller, plan, &request, &command) == OA_CONTROL_EIDENTITY);

    oa_sim_state moved{};
    init(moved);
    moved.side = OA_LEFT;
    std::copy(std::begin(first.state.arm[0].q), std::end(first.state.arm[0].q), moved.q);
    moved.q[0] += 0.05;
    CHECK(oa_controller_sim_set_state(first.controller, &moved) == OA_CONTROL_OK);
    CHECK(oa_controller_execute(first.controller, plan, &request, &command) == OA_CONTROL_ESTALE);
    oa_motion_plan_destroy(plan);

    Fixture future;
    plan = joint_plan(future, OA_LEFT, 0U, 0.1);
    const auto future_report = plan_report(plan);
    request = request_for(future_report, 100000000U);
    CHECK(oa_controller_execute(future.controller, plan, &request, &command) == OA_CONTROL_OK);
    for (std::uint64_t now = 10000000U; now <= 90000000U; now += 10000000U) {
        CHECK(oa_controller_advance(future.controller, now) == OA_CONTROL_OK);
    }
    init(moved);
    moved.side = OA_LEFT;
    std::copy(std::begin(future.state.arm[0].q), std::end(future.state.arm[0].q), moved.q);
    moved.q[0] += 0.05;
    CHECK(oa_controller_sim_set_state(future.controller, &moved) == OA_CONTROL_OK);
    CHECK(oa_controller_advance(future.controller, 100000000U) == OA_CONTROL_ESTALE);
    oa_motion_plan_destroy(plan);
}

void test_coherent_feedback_skew_and_partial_send() {
    {
        Fixture fixture;
        const oa_arm_snapshot before = fixture.state.arm[1];
        oa_sim_fault fault{};
        init(fault);
        fault.side = OA_RIGHT;
        fault.drop_mask = 1U;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 10000000U) == OA_CONTROL_ESTALE);
        oa_snapshot state{};
        init(state);
        CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_OK);
        CHECK(state.arm[1].fresh_mask == 0x7fU);
        CHECK(state.arm[1].feedback_seq == before.feedback_seq + 1U);
        CHECK(state.arm[1].t_ns == 10000000U);
        for (std::size_t joint = 0U; joint < 7U; ++joint) {
            CHECK(state.arm[1].q[joint] == before.q[joint]);
            CHECK(state.arm[1].raw_q[joint] == before.raw_q[joint]);
        }
    }
    {
        Fixture fixture;
        oa_sim_fault fault{};
        init(fault);
        fault.side = OA_RIGHT;
        fault.feedback_delay_ns = 2000000U;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 10000000U) == OA_CONTROL_ECAN);
    }
    {
        Fixture fixture;
        oa_sim_fault fault{};
        init(fault);
        fault.side = OA_LEFT;
        fault.feedback_delay_ns = UINT64_MAX;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 10000000U) == OA_CONTROL_ECAN);
        oa_snapshot state{};
        init(state);
        CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_OK);
        CHECK(state.lifecycle == OA_LIFECYCLE_FAULT);
    }
    {
        Fixture fixture;
        auto move = paired_move(fixture);
        oa_motion_plan *plan = nullptr;
        CHECK(oa_controller_plan_paired_tcp(fixture.controller, &move, &plan) == OA_CONTROL_OK);
        const auto report = plan_report(plan);
        (void)execute(fixture, plan, report);
        oa_sim_fault fault{};
        init(fault);
        fault.side = OA_LEFT;
        fault.command_fail_mask = 1U;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 10000000U) == OA_CONTROL_ECAN);
        oa_snapshot state{};
        init(state);
        CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_OK);
        CHECK(state.lifecycle == OA_LIFECYCLE_FAULT);
        oa_motion_plan_destroy(plan);
    }
}

void test_delayed_feedback_public_abi_oracle() {
    Fixture immediate;
    Fixture delayed;
    for (std::uint32_t side = OA_LEFT; side <= OA_RIGHT; ++side) {
        oa_sim_fault fault{};
        init(fault);
        fault.side = side;
        fault.feedback_delay_ns = 20000000U;
        CHECK(oa_controller_sim_set_fault(delayed.controller, &fault) == OA_CONTROL_OK);
    }

    oa_motion_plan *immediate_plan = joint_plan(immediate, OA_LEFT, 0U, 0.3);
    oa_motion_plan *delayed_plan = joint_plan(delayed, OA_LEFT, 0U, 0.3);
    const auto immediate_report = plan_report(immediate_plan);
    const auto delayed_report = plan_report(delayed_plan);
    CHECK(immediate_report.duration_ns == delayed_report.duration_ns);
    const std::uint64_t immediate_command =
        execute(immediate, immediate_plan, immediate_report, 3000000000ULL);
    const std::uint64_t delayed_command =
        execute(delayed, delayed_plan, delayed_report, 3000000000ULL);

    std::vector<oa_snapshot> immediate_history;
    bool differs_from_current = false;
    std::uint64_t immediate_completed_ns = 0U;
    std::uint64_t delayed_completed_ns = 0U;
    oa_event immediate_completed{};
    oa_event delayed_completed{};
    oa_snapshot immediate_completion_state{};
    oa_snapshot delayed_completion_state{};
    for (std::uint64_t cycle = 1U; delayed_completed_ns == 0U && cycle < 400U; ++cycle) {
        const std::uint64_t now = cycle * 10000000U;
        CHECK(oa_controller_advance(immediate.controller, now) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(delayed.controller, now) == OA_CONTROL_OK);
        oa_snapshot immediate_state{};
        oa_snapshot delayed_state{};
        init(immediate_state);
        init(delayed_state);
        CHECK(oa_controller_snapshot(immediate.controller, &immediate_state) == OA_CONTROL_OK);
        CHECK(oa_controller_snapshot(delayed.controller, &delayed_state) == OA_CONTROL_OK);
        immediate_history.push_back(immediate_state);
        if (cycle <= 2U) {
            CHECK(delayed_state.arm[0].feedback_seq == delayed.state.arm[0].feedback_seq);
            CHECK(delayed_state.arm[0].t_ns == 0U);
        } else if (immediate_completed_ns == 0U) {
            const oa_snapshot &historical = immediate_history[cycle - 3U];
            for (std::size_t side = 0U; side < 2U; ++side) {
                check_arm_payload_equal(delayed_state.arm[side], historical.arm[side]);
            }
            differs_from_current = differs_from_current ||
                                   delayed_state.arm[0].q[0] != immediate_state.arm[0].q[0] ||
                                   delayed_state.arm[0].dq[0] != immediate_state.arm[0].dq[0];
        }
        if (immediate_completed_ns == 0U &&
            take_event(immediate.controller, OA_EVENT_COMPLETED, immediate_command,
                       immediate_completed)) {
            immediate_completed_ns = now;
            immediate_completion_state = immediate_state;
        }
        if (take_event(delayed.controller, OA_EVENT_COMPLETED, delayed_command,
                       delayed_completed)) {
            delayed_completed_ns = now;
            delayed_completion_state = delayed_state;
        }
    }
    CHECK(differs_from_current);
    CHECK(immediate_completed_ns != 0U);
    CHECK(delayed_completed_ns == immediate_completed_ns + 20000000U);
    CHECK(immediate_completed.feedback_seq ==
          std::min(immediate_completion_state.arm[0].feedback_seq,
                   immediate_completion_state.arm[1].feedback_seq));
    CHECK(delayed_completed.feedback_seq ==
          std::min(delayed_completion_state.arm[0].feedback_seq,
                   delayed_completion_state.arm[1].feedback_seq));
    oa_motion_plan_destroy(immediate_plan);
    oa_motion_plan_destroy(delayed_plan);
}

void test_feedback_generation_atomicity_and_bounds() {
    const auto config = valid_config();
    openarm::control::ArmRuntime partial(config.arm[0]);
    const openarm::control::JointVector zero{};
    partial.force_state(zero, zero, 0U);
    const oa_arm_snapshot before = partial.snapshot(0U, 100000000U);
    openarm::control::JointVector target{};
    target.fill(0.5);
    partial.set_injection(0U, 1U, 0U, 0U, 0U, 0U);
    CHECK(partial.command_and_step(target, zero, 10000000U, 0.01));
    const oa_arm_snapshot after = partial.snapshot(10000000U, 100000000U);
    CHECK(after.feedback_seq == before.feedback_seq);
    CHECK(after.fresh_mask == 0x7eU);
    CHECK(after.t_ns == before.t_ns);
    for (std::size_t joint = 0U; joint < 7U; ++joint) {
        CHECK(after.q[joint] == before.q[joint]);
        CHECK(after.dq[joint] == before.dq[joint]);
        CHECK(after.raw_q[joint] == before.raw_q[joint]);
        CHECK(after.raw_dq[joint] == before.raw_dq[joint]);
        CHECK(after.status[joint] == before.status[joint]);
    }

    openarm::control::ArmRuntime immediate(config.arm[0]);
    openarm::control::ArmRuntime nongrid(config.arm[0]);
    immediate.force_state(zero, zero, 0U);
    nongrid.force_state(zero, zero, 0U);
    immediate.set_injection(0U, 0U, 0U, 0U, 0U, 0U);
    nongrid.set_injection(0U, 0U, 0U, 0U, 0U, 15000000U);
    CHECK(immediate.command_and_step(target, zero, 10000000U, 0.01));
    CHECK(nongrid.command_and_step(target, zero, 10000000U, 0.01));
    CHECK(nongrid.command_and_step(target, zero, 20000000U, 0.01));
    CHECK(nongrid.snapshot(20000000U, 100000000U).feedback_seq == 1U);
    CHECK(nongrid.command_and_step(target, zero, 30000000U, 0.01));
    const oa_arm_snapshot delayed_first = nongrid.snapshot(30000000U, 100000000U);
    const oa_arm_snapshot immediate_first = immediate.snapshot(10000000U, 100000000U);
    check_arm_payload_equal(delayed_first, immediate_first);
    CHECK(delayed_first.t_ns == 10000000U);

    openarm::control::ArmRuntime mutation(config.arm[0]);
    mutation.force_state(zero, zero, 0U);
    mutation.set_injection(0U, 0U, 0U, 0U, 0U, 20000000U);
    CHECK(mutation.command_and_step(target, zero, 10000000U, 0.01));
    mutation.set_injection(0U, 0U, 0U, 0U, 0U, 100000000U);
    CHECK(mutation.command_and_step(target, zero, 20000000U, 0.01));
    CHECK(mutation.command_and_step(target, zero, 30000000U, 0.01));
    const oa_arm_snapshot retained_ready_time =
        mutation.snapshot(30000000U, 100000000U);
    check_arm_payload_equal(retained_ready_time, immediate_first);

    openarm::control::ArmRuntime delayed_partial(config.arm[0]);
    delayed_partial.force_state(zero, zero, 0U);
    const oa_arm_snapshot delayed_partial_before =
        delayed_partial.snapshot(0U, 100000000U);
    delayed_partial.set_injection(0U, 1U, 0U, 0U, 0U, 20000000U);
    CHECK(delayed_partial.command_and_step(target, zero, 10000000U, 0.01));
    CHECK(delayed_partial.command_and_step(target, zero, 20000000U, 0.01));
    CHECK(delayed_partial.command_and_step(target, zero, 30000000U, 0.01));
    const oa_arm_snapshot delayed_partial_after =
        delayed_partial.snapshot(30000000U, 100000000U);
    CHECK(delayed_partial_after.feedback_seq == delayed_partial_before.feedback_seq);
    CHECK(delayed_partial_after.fresh_mask == 0x7eU);
    CHECK(delayed_partial_after.t_ns == delayed_partial_before.t_ns);
    for (std::size_t joint = 0U; joint < 7U; ++joint) {
        CHECK(delayed_partial_after.q[joint] == delayed_partial_before.q[joint]);
        CHECK(delayed_partial_after.raw_q[joint] == delayed_partial_before.raw_q[joint]);
    }

    openarm::control::ArmRuntime overflow(config.arm[0]);
    overflow.force_state(zero, zero, 0U);
    overflow.set_injection(0U, 0U, 0U, 0U, 0U, UINT64_MAX);
    CHECK(!overflow.command_and_step(target, zero, 1U, 1.0e-9));
    const oa_arm_snapshot overflow_state = overflow.snapshot(1U, 100000000U);
    CHECK(overflow_state.feedback_seq == 1U);
    CHECK(overflow_state.t_ns == 0U);

    openarm::control::ArmRuntime bounded(config.arm[0]);
    bounded.force_state(zero, zero, 0U);
    bounded.set_injection(0U, 0U, 0U, 0U, 0U, 1000000000U);
    for (std::uint64_t now = 1U; now <= 64U; ++now) {
        CHECK(bounded.command_and_step(target, zero, now, 1.0e-9));
    }
    CHECK(!bounded.command_and_step(target, zero, 65U, 1.0e-9));
    CHECK(bounded.snapshot(65U, 100000000U).feedback_seq == 1U);

    openarm::control::ArmRuntime freshness(config.arm[0]);
    freshness.force_state(zero, zero, 0U);
    freshness.set_injection(0U, 0U, 0U, 0U, 0U, 1000000000U);
    for (std::uint64_t now = 10000000U; now <= 50000000U; now += 10000000U) {
        CHECK(freshness.command_and_step(target, zero, now, 0.01));
    }
    CHECK(freshness.complete_fresh(50000000U, 50000000U));
    const oa_arm_snapshot freshness_boundary =
        freshness.snapshot(50000000U, 50000000U);
    CHECK(!freshness.complete_fresh(50000001U, 50000000U));
    const oa_arm_snapshot stale = freshness.snapshot(50000001U, 50000000U);
    CHECK(stale.feedback_seq == freshness_boundary.feedback_seq);
    CHECK(stale.t_ns == 0U);
    for (std::size_t joint = 0U; joint < 7U; ++joint) {
        CHECK(stale.q[joint] == freshness_boundary.q[joint]);
        CHECK(stale.raw_q[joint] == freshness_boundary.raw_q[joint]);
    }
}

void test_completion_requires_distinct_delivered_generations() {
    Fixture fixture;
    oa_motion_plan *plan = joint_plan(fixture, OA_LEFT, 0U, 0.1);
    const auto report = plan_report(plan);
    const std::uint64_t command = execute(fixture, plan, report, 3000000000ULL);
    std::uint64_t now = 0U;
    bool at_goal = false;
    while (!at_goal && now < report.duration_ns + 1500000000ULL) {
        now += 10000000U;
        CHECK(oa_controller_advance(fixture.controller, now) == OA_CONTROL_OK);
        oa_snapshot state{};
        init(state);
        CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_OK);
        at_goal = now >= report.duration_ns &&
                  std::abs(state.arm[0].q[0] - report.target_q[0][0]) <= 5.0e-4 &&
                  std::abs(state.arm[0].dq[0]) <= 2.0e-2;
        CHECK(!has_event(fixture.controller, OA_EVENT_COMPLETED, command));
    }
    CHECK(at_goal);
    for (std::uint32_t side = OA_LEFT; side <= OA_RIGHT; ++side) {
        oa_sim_fault fault{};
        init(fault);
        fault.side = side;
        fault.feedback_delay_ns = 40000000U;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
    }
    for (std::size_t cycle = 1U; cycle <= 6U; ++cycle) {
        CHECK(oa_controller_advance(fixture.controller, now + cycle * 10000000U) == OA_CONTROL_OK);
        CHECK(!has_event(fixture.controller, OA_EVENT_COMPLETED, command));
    }
    CHECK(oa_controller_advance(fixture.controller, now + 70000000U) == OA_CONTROL_OK);
    CHECK(has_event(fixture.controller, OA_EVENT_COMPLETED, command));
    oa_motion_plan_destroy(plan);
}

void test_delayed_feedback_is_retired_by_lifecycle_transitions() {
    for (const std::uint32_t stop_kind : {OA_STOP_DISABLE, OA_STOP_CONTROLLED}) {
        Fixture fixture;
        std::uint64_t command = 0U;
        oa_motion_plan *plan = start_delayed_motion(
            fixture, stop_kind, UINT64_C(5000000000), command);
        const oa_snapshot before = controller_snapshot(fixture);
        check_stop_payload(before, OA_LIFECYCLE_EXECUTING, true);
        CHECK(oa_controller_stop(fixture.controller, stop_kind) == OA_CONTROL_OK);
        const oa_snapshot stopped = controller_snapshot(fixture);
        const bool enabled = stop_kind == OA_STOP_CONTROLLED;
        check_stop_payload(stopped, enabled ? OA_LIFECYCLE_ARMED_IDLE
                                            : OA_LIFECYCLE_DISARMED,
                           enabled);
        for (std::size_t side = 0U; side < 2U; ++side) {
            CHECK(stopped.arm[side].feedback_seq == before.arm[side].feedback_seq + 1U);
            CHECK(stopped.arm[side].t_ns == 20000000U);
            for (std::size_t joint = 0U; joint < 7U; ++joint) {
                CHECK(stopped.arm[side].q[joint] == before.arm[side].q[joint]);
                CHECK(stopped.arm[side].raw_q[joint] == before.arm[side].raw_q[joint]);
            }
        }
        oa_event stopped_event{};
        CHECK(take_event(fixture.controller, OA_EVENT_STOPPED, command, stopped_event));
        CHECK(stopped_event.feedback_seq ==
              std::min(stopped.arm[0].feedback_seq, stopped.arm[1].feedback_seq));
        CHECK(oa_controller_advance(fixture.controller, 30000000U) == OA_CONTROL_OK);
        const oa_snapshot next = controller_snapshot(fixture);
        for (std::size_t side = 0U; side < 2U; ++side) {
            check_arm_payload_equal(next.arm[side], stopped.arm[side]);
        }
        CHECK(oa_controller_advance(fixture.controller, 40000000U) == OA_CONTROL_OK);
        const oa_snapshot second = controller_snapshot(fixture);
        for (std::size_t side = 0U; side < 2U; ++side) {
            check_arm_payload_equal(second.arm[side], stopped.arm[side]);
        }
        CHECK(oa_controller_advance(fixture.controller, 50000000U) == OA_CONTROL_OK);
        const oa_snapshot post_stop_feedback = controller_snapshot(fixture);
        check_stop_payload(post_stop_feedback,
                           enabled ? OA_LIFECYCLE_ARMED_IDLE
                                   : OA_LIFECYCLE_DISARMED,
                           enabled);
        CHECK(post_stop_feedback.arm[0].t_ns == 30000000U);
        CHECK(post_stop_feedback.arm[1].t_ns == 30000000U);
        oa_motion_plan_destroy(plan);
    }

    {
        Fixture fixture;
        std::uint64_t command = 0U;
        oa_motion_plan *plan = start_delayed_motion(
            fixture, OA_STOP_CONTROLLED, UINT64_C(5000000000), command);
        const oa_snapshot before = controller_snapshot(fixture);
        CHECK(oa_controller_disarm(fixture.controller, UINT64_MAX) == OA_CONTROL_OK);
        const oa_snapshot disarmed = controller_snapshot(fixture);
        check_stop_payload(disarmed, OA_LIFECYCLE_DISARMED, false);
        for (std::size_t side = 0U; side < 2U; ++side) {
            CHECK(disarmed.arm[side].feedback_seq == before.arm[side].feedback_seq + 1U);
            CHECK(disarmed.arm[side].t_ns == 20000000U);
        }
        CHECK(oa_controller_advance(fixture.controller, 30000000U) == OA_CONTROL_OK);
        const oa_snapshot next = controller_snapshot(fixture);
        for (std::size_t side = 0U; side < 2U; ++side) {
            check_arm_payload_equal(next.arm[side], disarmed.arm[side]);
        }
        oa_motion_plan_destroy(plan);
    }

    {
        Fixture fixture;
        std::uint64_t command = 0U;
        oa_motion_plan *plan = start_delayed_motion(
            fixture, OA_STOP_CONTROLLED, UINT64_C(5000000000), command);
        const oa_snapshot before = controller_snapshot(fixture);
        CHECK(oa_controller_set_interlock(fixture.controller, 1U, 0U) == OA_CONTROL_EESTOP);
        const oa_snapshot estop = controller_snapshot(fixture);
        check_stop_payload(estop, OA_LIFECYCLE_ESTOP, false);
        for (std::size_t side = 0U; side < 2U; ++side) {
            CHECK(estop.arm[side].feedback_seq == before.arm[side].feedback_seq + 1U);
            CHECK(estop.arm[side].t_ns == 20000000U);
        }
        reset_and_reverify(fixture);
        CHECK(oa_controller_advance(fixture.controller, 30000000U) == OA_CONTROL_OK);
        const oa_snapshot after_close = controller_snapshot(fixture);
        check_stop_payload(after_close, OA_LIFECYCLE_DISARMED, false);
        CHECK(after_close.arm[0].t_ns == 30000000U);
        CHECK(after_close.arm[1].t_ns == 30000000U);
        oa_motion_plan_destroy(plan);
    }

    {
        Fixture fixture;
        std::uint64_t command = 0U;
        oa_motion_plan *plan = start_delayed_motion(
            fixture, OA_STOP_CONTROLLED, 20000000U, command);
        CHECK(oa_controller_advance(fixture.controller, 30000000U) == OA_CONTROL_ESTALE);
        const oa_snapshot watchdog = controller_snapshot(fixture);
        check_stop_payload(watchdog, OA_LIFECYCLE_FAULT, true);
        CHECK(watchdog.arm[0].t_ns == 30000000U);
        CHECK(watchdog.arm[1].t_ns == 30000000U);
        reset_and_reverify(fixture);
        oa_motion_plan_destroy(plan);
    }

    {
        Fixture fixture;
        std::uint64_t command = 0U;
        oa_motion_plan *plan = start_delayed_motion(
            fixture, OA_STOP_DISABLE, UINT64_C(5000000000), command);
        oa_sim_fault fault{};
        init(fault);
        fault.side = OA_LEFT;
        fault.command_fail_mask = 1U;
        fault.feedback_delay_ns = 20000000U;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 30000000U) == OA_CONTROL_ECAN);
        const oa_snapshot bus_fault = controller_snapshot(fixture);
        check_stop_payload(bus_fault, OA_LIFECYCLE_FAULT, false);
        CHECK(bus_fault.arm[0].t_ns == 30000000U);
        CHECK(bus_fault.arm[1].t_ns == 30000000U);
        reset_and_reverify(fixture);
        oa_motion_plan_destroy(plan);
    }

    {
        Fixture fixture;
        std::uint64_t command = 0U;
        oa_motion_plan *plan = start_delayed_motion(
            fixture, OA_STOP_CONTROLLED, UINT64_C(5000000000), command);
        oa_sim_fault fault{};
        init(fault);
        fault.side = OA_RIGHT;
        fault.fault_mask = 1U << 3U;
        fault.feedback_delay_ns = 20000000U;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 30000000U) == OA_CONTROL_EFAULT);
        const oa_snapshot motor_fault = controller_snapshot(fixture);
        check_stop_payload(motor_fault, OA_LIFECYCLE_FAULT, false, 1, 3);
        CHECK((motor_fault.arm[1].fault_mask & (1U << 3U)) != 0U);
        CHECK(motor_fault.arm[0].t_ns == 30000000U);
        CHECK(motor_fault.arm[1].t_ns == 30000000U);
        reset_and_reverify(fixture);
        oa_motion_plan_destroy(plan);
    }

    {
        Fixture fixture;
        std::uint64_t command = 0U;
        oa_motion_plan *plan = start_delayed_motion(
            fixture, OA_STOP_CONTROLLED, UINT64_C(5000000000), command);
        oa_sim_fault fault{};
        init(fault);
        fault.side = OA_LEFT;
        fault.drop_mask = 1U;
        fault.feedback_delay_ns = 20000000U;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 30000000U) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 40000000U) == OA_CONTROL_OK);
        const oa_snapshot before_partial = controller_snapshot(fixture);
        CHECK(oa_controller_advance(fixture.controller, 50000000U) == OA_CONTROL_ESTALE);
        const oa_snapshot partial = controller_snapshot(fixture);
        check_stop_payload(partial, OA_LIFECYCLE_FAULT, false);
        for (std::size_t joint = 0U; joint < 7U; ++joint) {
            CHECK(partial.arm[0].q[joint] == before_partial.arm[0].q[joint]);
            CHECK(partial.arm[0].raw_q[joint] == before_partial.arm[0].raw_q[joint]);
        }
        reset_and_reverify(fixture);
        oa_motion_plan_destroy(plan);
    }

    {
        Fixture fixture;
        std::uint64_t command = 0U;
        oa_motion_plan *plan = start_delayed_motion(
            fixture, OA_STOP_DISABLE, UINT64_C(5000000000), command);
        CHECK(oa_controller_advance(fixture.controller, 40000000U) == OA_CONTROL_ETIMEOUT);
        const oa_snapshot deadline = controller_snapshot(fixture);
        check_stop_payload(deadline, OA_LIFECYCLE_FAULT, false);
        CHECK(deadline.arm[0].t_ns == 40000000U);
        CHECK(deadline.arm[1].t_ns == 40000000U);
        reset_and_reverify(fixture);
        oa_motion_plan_destroy(plan);
    }

    {
        Fixture fixture;
        for (;;) {
            oa_event event{};
            init(event);
            if (oa_controller_poll_event(fixture.controller, 0U, &event) == OA_CONTROL_ETIMEOUT) {
                break;
            }
        }
        for (std::size_t event = 0U; event < 64U; ++event) {
            CHECK(oa_controller_disarm(fixture.controller, UINT64_MAX) == OA_CONTROL_OK);
        }
        set_feedback_delay(fixture, 20000000U);
        CHECK(oa_controller_advance(fixture.controller, 10000000U) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 20000000U) == OA_CONTROL_OK);
        CHECK(oa_controller_set_interlock(fixture.controller, 1U, 0U) == OA_CONTROL_EESTOP);
        const oa_snapshot overflow = controller_snapshot(fixture);
        check_stop_payload(overflow, OA_LIFECYCLE_FAULT, false);
        CHECK(overflow.arm[0].t_ns == 20000000U);
        CHECK(overflow.arm[1].t_ns == 20000000U);
        reset_and_reverify(fixture);
    }

    {
        const auto config = valid_config();
        oa_manifest *manifest = nullptr;
        CHECK(oa_manifest_create(&config, &manifest) == OA_CONTROL_OK);
        auto options = virtual_options();
        oa_controller *controller = nullptr;
        CHECK(oa_controller_create(manifest, &options, &controller) == OA_CONTROL_OK);
        CHECK(oa_controller_set_interlock(controller, 1U, 0U) == OA_CONTROL_EESTOP);
        oa_snapshot state{};
        init(state);
        CHECK(oa_controller_snapshot(controller, &state) == OA_CONTROL_OK);
        CHECK(state.lifecycle == OA_LIFECYCLE_ESTOP);
        CHECK(state.arm[0].feedback_seq == 0U);
        CHECK(state.arm[1].feedback_seq == 0U);
        CHECK(state.arm[0].fresh_mask == 0U);
        CHECK(state.arm[1].fresh_mask == 0U);
        oa_controller_destroy(controller);
        oa_manifest_destroy(manifest);
    }
}

void test_cartesian_path_policies_and_scene_binding() {
    Fixture fixture;
    auto move = paired_move(fixture);
    oa_motion_plan *plan = nullptr;
    move.max_branch_step_rad = 1.0e-9;
    CHECK(oa_controller_plan_paired_tcp(fixture.controller, &move, &plan) ==
          OA_CONTROL_EUNREACHABLE);
    move = paired_move(fixture);
    move.min_singular_value = 100.0;
    CHECK(oa_controller_plan_paired_tcp(fixture.controller, &move, &plan) ==
          OA_CONTROL_EUNREACHABLE);
    move = paired_move(fixture);
    CHECK(oa_controller_plan_paired_tcp(fixture.controller, &move, &plan) == OA_CONTROL_OK);
    const auto report = plan_report(plan);
    CHECK(oa_controller_set_collision_scene_revision(fixture.controller, 2U) == OA_CONTROL_OK);
    auto request = request_for(report);
    std::uint64_t command = 0U;
    CHECK(oa_controller_execute(fixture.controller, plan, &request, &command) == OA_CONTROL_ESTALE);
    oa_motion_plan_destroy(plan);
}

void test_watchdog_estop_event_overflow_and_concurrency() {
    {
        Fixture fixture;
        oa_motion_plan *plan = joint_plan(fixture, OA_LEFT, 0U, 0.1);
        const auto report = plan_report(plan);
        auto request = request_for(report);
        request.producer_deadline_ns = 50000000U;
        std::uint64_t command = 0U;
        CHECK(oa_controller_execute(fixture.controller, plan, &request, &command) == OA_CONTROL_OK);
        for (std::uint64_t now = 10000000U; now <= 50000000U; now += 10000000U) {
            CHECK(oa_controller_advance(fixture.controller, now) == OA_CONTROL_OK);
        }
        CHECK(oa_controller_advance(fixture.controller, 60000000U) == OA_CONTROL_ESTALE);
        oa_motion_plan_destroy(plan);
    }
    {
        Fixture fixture;
        CHECK(oa_controller_set_interlock(fixture.controller, 1U, 0U) == OA_CONTROL_EESTOP);
        oa_snapshot state{};
        init(state);
        CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_OK);
        CHECK(state.lifecycle == OA_LIFECYCLE_ESTOP);
        oa_arm_challenge challenge{};
        init(challenge);
        CHECK(oa_controller_get_arm_challenge(fixture.controller, &challenge) == OA_CONTROL_OK);
        oa_reset_request reset{};
        init(reset);
        reset.verify_epoch = challenge.verify_epoch;
        reset.nonce = challenge.nonce;
        CHECK(oa_controller_reset_fault(fixture.controller, &reset) == OA_CONTROL_OK);
        CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_ESTATE);
    }
    {
        Fixture fixture;
        CHECK(oa_controller_disarm(fixture.controller, UINT64_MAX) == OA_CONTROL_OK);
        for (std::size_t index = 0; index < 70U; ++index) {
            const oa_control_status status = oa_controller_disarm(fixture.controller, UINT64_MAX);
            if (status != OA_CONTROL_OK) {
                CHECK(status == OA_CONTROL_ESTATE);
                break;
            }
        }
        oa_snapshot state{};
        init(state);
        CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_OK);
        CHECK(state.lifecycle == OA_LIFECYCLE_FAULT);
    }
    {
        Fixture fixture;
        for (;;) {
            oa_event event{};
            init(event);
            if (oa_controller_poll_event(fixture.controller, 0U, &event) == OA_CONTROL_ETIMEOUT) {
                break;
            }
        }
        std::atomic<oa_control_status> waited_status{OA_CONTROL_EFAULT};
        std::thread waiter([&]() {
            oa_event event{};
            init(event);
            const auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::milliseconds(500);
            const auto deadline_ns = static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    deadline.time_since_epoch()).count());
            waited_status.store(
                oa_controller_poll_event(fixture.controller, deadline_ns, &event));
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        CHECK(oa_controller_disarm(fixture.controller, UINT64_MAX) == OA_CONTROL_OK);
        waiter.join();
        CHECK(waited_status.load() == OA_CONTROL_OK);
    }
    {
        Fixture fixture;
        std::atomic<bool> run{true};
        std::atomic<bool> failed{false};
        std::thread reader([&]() {
            while (run.load()) {
                oa_snapshot state{};
                init(state);
                const oa_control_status status = oa_controller_snapshot(fixture.controller, &state);
                if (status != OA_CONTROL_OK && status != OA_CONTROL_EINVAL && status != OA_CONTROL_ESTATE) {
                    failed.store(true);
                }
            }
        });
        std::thread event_reader([&]() {
            while (run.load()) {
                oa_event event{};
                init(event);
                const oa_control_status status = oa_controller_poll_event(fixture.controller, 0U, &event);
                if (status != OA_CONTROL_OK && status != OA_CONTROL_ETIMEOUT && status != OA_CONTROL_EINVAL &&
                    status != OA_CONTROL_ESTATE) {
                    failed.store(true);
                }
            }
        });
        for (std::uint64_t index = 1U; index <= 100U; ++index) {
            CHECK(oa_controller_advance(fixture.controller, index * 10000000U) == OA_CONTROL_OK);
        }
        oa_controller_destroy(fixture.controller);
        fixture.controller = nullptr;
        run.store(false);
        reader.join();
        event_reader.join();
        CHECK(!failed.load());
    }
}

void test_invalid_handles_and_transactional_create() {
    auto config = valid_config();
    oa_manifest *manifest = nullptr;
    CHECK(oa_manifest_create(&config, &manifest) == OA_CONTROL_OK);
    auto options = virtual_options();
    std::array<unsigned char, 1> arbitrary{};
    auto *invalid_manifest = reinterpret_cast<oa_manifest *>(arbitrary.data());
    oa_controller *controller = reinterpret_cast<oa_controller *>(UINTPTR_MAX - 16U);
    CHECK(oa_controller_create(invalid_manifest, &options, &controller) == OA_CONTROL_EINVAL);
    CHECK(controller == reinterpret_cast<oa_controller *>(UINTPTR_MAX - 16U));
    oa_manifest_destroy(invalid_manifest);

    CHECK(oa_control_test_active_controller_count() == 0U);
    for (std::int32_t checkpoint = 0; checkpoint < 3; ++checkpoint) {
        controller = reinterpret_cast<oa_controller *>(UINTPTR_MAX - 32U);
        oa_control_test_fail_controller_create_after(checkpoint);
        CHECK(oa_controller_create(manifest, &options, &controller) == OA_CONTROL_ENOMEM);
        CHECK(controller == reinterpret_cast<oa_controller *>(UINTPTR_MAX - 32U));
        CHECK(oa_control_test_active_controller_count() == 0U);
    }
    oa_control_test_fail_controller_create_after(-1);
    controller = nullptr;
    CHECK(oa_controller_create(manifest, &options, &controller) == OA_CONTROL_OK);
    CHECK(oa_controller_create(reinterpret_cast<oa_manifest *>(controller),
                               &options, &controller) == OA_CONTROL_EINVAL);
    oa_controller_destroy(controller);

    oa_manifest *stale_manifest = manifest;
    std::atomic<bool> manifest_run{true};
    std::atomic<bool> manifest_failed{false};
    std::thread manifest_user([&]() {
        while (manifest_run.load()) {
            oa_controller *temporary = nullptr;
            const oa_control_status status =
                oa_controller_create(stale_manifest, &options, &temporary);
            if (status == OA_CONTROL_OK) {
                oa_controller_destroy(temporary);
            } else if (status != OA_CONTROL_EINVAL) {
                manifest_failed.store(true);
            }
        }
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    oa_manifest_destroy(manifest);
    manifest_run.store(false);
    manifest_user.join();
    CHECK(!manifest_failed.load());
    controller = nullptr;
    CHECK(oa_controller_create(stale_manifest, &options, &controller) == OA_CONTROL_EINVAL);

    Fixture fixture;
    oa_motion_plan *plan = joint_plan(fixture, OA_LEFT, 0U, 0.1);
    oa_motion_plan_report report{};
    init(report);
    auto *invalid_plan = reinterpret_cast<oa_motion_plan *>(arbitrary.data());
    CHECK(oa_motion_plan_get_report(invalid_plan, &report) == OA_CONTROL_EINVAL);
    CHECK(oa_motion_plan_get_report(reinterpret_cast<oa_motion_plan *>(fixture.controller),
                                    &report) == OA_CONTROL_EINVAL);
    CHECK(oa_motion_plan_get_report(reinterpret_cast<oa_motion_plan *>(stale_manifest),
                                    &report) == OA_CONTROL_EINVAL);
    oa_motion_plan_destroy(invalid_plan);
    oa_motion_plan *stale_plan = plan;
    std::atomic<bool> plan_run{true};
    std::atomic<bool> plan_failed{false};
    std::thread plan_user([&]() {
        while (plan_run.load()) {
            oa_motion_plan_report concurrent_report{};
            init(concurrent_report);
            const oa_control_status status =
                oa_motion_plan_get_report(stale_plan, &concurrent_report);
            if (status != OA_CONTROL_OK && status != OA_CONTROL_EINVAL) {
                plan_failed.store(true);
            }
        }
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    oa_motion_plan_destroy(plan);
    plan_run.store(false);
    plan_user.join();
    CHECK(!plan_failed.load());
    CHECK(oa_motion_plan_get_report(stale_plan, &report) == OA_CONTROL_EINVAL);
    oa_execute_request request{};
    init(request);
    request.start_ns = 0U;
    request.expiry_ns = 1000000000U;
    request.producer_deadline_ns = 1000000000U;
    request.stop_kind = OA_STOP_DISABLE;
    std::uint64_t command = 0U;
    CHECK(oa_controller_execute(fixture.controller, stale_plan, &request, &command) ==
          OA_CONTROL_EINVAL);
}

void test_registry_storage_is_bounded() {
    auto config = valid_config();
    auto options = virtual_options();
    const auto cycle_handles = [&]() {
        oa_manifest *manifest = nullptr;
        CHECK(oa_manifest_create(&config, &manifest) == OA_CONTROL_OK);
        oa_controller *controller = nullptr;
        CHECK(oa_controller_create(manifest, &options, &controller) == OA_CONTROL_OK);
        oa_controller_destroy(controller);
        oa_manifest_destroy(manifest);
    };

    for (std::size_t iteration = 0U; iteration < 64U; ++iteration) {
        cycle_handles();
    }
    CHECK(oa_control_test_active_manifest_count() == 0U);
    CHECK(oa_control_test_active_controller_count() == 0U);
#ifndef OA_CONTROL_TEST_SANITIZED
    const std::uint64_t rss_before = resident_bytes();
#endif
    for (std::size_t iteration = 0U; iteration < 4096U; ++iteration) {
        cycle_handles();
    }
    CHECK(oa_control_test_active_manifest_count() == 0U);
    CHECK(oa_control_test_active_controller_count() == 0U);

    {
        Fixture fixture;
        for (std::size_t iteration = 0U; iteration < 4096U; ++iteration) {
            oa_motion_plan *const plan = joint_plan(fixture, OA_LEFT, 0U, 0.1);
            oa_motion_plan_destroy(plan);
        }
        CHECK(oa_control_test_active_manifest_count() == 1U);
        CHECK(oa_control_test_active_controller_count() == 1U);
        CHECK(oa_control_test_active_plan_count() == 0U);
    }
    CHECK(oa_control_test_active_manifest_count() == 0U);
    CHECK(oa_control_test_active_controller_count() == 0U);
    CHECK(oa_control_test_active_plan_count() == 0U);

#ifndef OA_CONTROL_TEST_SANITIZED
    const std::uint64_t rss_after = resident_bytes();
    constexpr std::uint64_t kMaximumAllocatorGrowth = UINT64_C(16) * 1024U * 1024U;
    if (rss_before != 0U && rss_after != 0U) {
        CHECK(rss_after <= rss_before + kMaximumAllocatorGrowth);
    }
#endif
}

void check_materialized_fault_stop(Fixture &fixture, const bool enabled_hold,
                                   const std::int32_t fault_side = -1,
                                   const std::int32_t fault_joint = -1) {
    oa_snapshot state{};
    init(state);
    CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_OK);
    CHECK(state.lifecycle == OA_LIFECYCLE_FAULT);
    for (std::size_t side = 0U; side < 2U; ++side) {
        for (std::size_t joint = 0U; joint < 7U; ++joint) {
            const bool faulted = static_cast<std::int32_t>(side) == fault_side &&
                                 static_cast<std::int32_t>(joint) == fault_joint;
            CHECK(state.arm[side].status[joint] ==
                  (faulted ? 8U : (enabled_hold ? 1U : 0U)));
            CHECK(std::abs(state.arm[side].dq[joint]) <= 1.2e-2);
        }
    }
}

void test_fault_stop_policy_by_cause() {
    for (const std::uint32_t stop_kind : {OA_STOP_DISABLE, OA_STOP_CONTROLLED}) {
        Fixture fixture;
        oa_motion_plan *plan = joint_plan(fixture, OA_LEFT, 0U, 0.2);
        const auto report = plan_report(plan);
        auto request = request_for(report);
        request.stop_kind = stop_kind;
        request.producer_deadline_ns = 10000000U;
        std::uint64_t command = 0U;
        CHECK(oa_controller_execute(fixture.controller, plan, &request, &command) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 10000000U) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 20000000U) == OA_CONTROL_ESTALE);
        check_materialized_fault_stop(fixture, stop_kind == OA_STOP_CONTROLLED);
        oa_motion_plan_destroy(plan);
    }
    {
        Fixture fixture;
        oa_motion_plan *plan = joint_plan(fixture, OA_LEFT, 0U, 0.2);
        const auto report = plan_report(plan);
        auto request = request_for(report);
        request.stop_kind = OA_STOP_CONTROLLED;
        request.expiry_ns = report.duration_ns;
        std::uint64_t command = 0U;
        CHECK(oa_controller_execute(fixture.controller, plan, &request, &command) == OA_CONTROL_OK);
        std::uint64_t now = 0U;
        while (now + 10000000U <= request.expiry_ns) {
            now += 10000000U;
            CHECK(oa_controller_advance(fixture.controller, now) == OA_CONTROL_OK);
        }
        CHECK(oa_controller_advance(fixture.controller, now + 10000000U) == OA_CONTROL_ETIMEOUT);
        check_materialized_fault_stop(fixture, true);
        oa_motion_plan_destroy(plan);
    }
    {
        Fixture fixture;
        oa_motion_plan *plan = joint_plan(fixture, OA_LEFT, 0U, 0.2);
        const auto report = plan_report(plan);
        auto request = request_for(report);
        request.stop_kind = OA_STOP_CONTROLLED;
        std::uint64_t command = 0U;
        CHECK(oa_controller_execute(fixture.controller, plan, &request, &command) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 10000000U) == OA_CONTROL_OK);
        oa_sim_fault fault{};
        init(fault);
        fault.side = OA_RIGHT;
        fault.drop_mask = 1U;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 20000000U) == OA_CONTROL_ESTALE);
        check_materialized_fault_stop(fixture, false);
        oa_motion_plan_destroy(plan);
    }
    {
        Fixture fixture;
        oa_motion_plan *plan = joint_plan(fixture, OA_LEFT, 0U, 0.2);
        const auto report = plan_report(plan);
        auto request = request_for(report);
        request.stop_kind = OA_STOP_CONTROLLED;
        std::uint64_t command = 0U;
        CHECK(oa_controller_execute(fixture.controller, plan, &request, &command) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 10000000U) == OA_CONTROL_OK);
        oa_sim_fault fault{};
        init(fault);
        fault.side = OA_LEFT;
        fault.command_fail_mask = 1U;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 20000000U) == OA_CONTROL_ECAN);
        check_materialized_fault_stop(fixture, false);
        oa_motion_plan_destroy(plan);
    }
    {
        Fixture fixture;
        oa_motion_plan *plan = joint_plan(fixture, OA_LEFT, 0U, 0.2);
        const auto report = plan_report(plan);
        auto request = request_for(report);
        request.stop_kind = OA_STOP_CONTROLLED;
        std::uint64_t command = 0U;
        CHECK(oa_controller_execute(fixture.controller, plan, &request, &command) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 10000000U) == OA_CONTROL_OK);
        oa_sim_fault fault{};
        init(fault);
        fault.side = OA_RIGHT;
        fault.feedback_delay_ns = 2000000U;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 20000000U) == OA_CONTROL_ECAN);
        check_materialized_fault_stop(fixture, false);
        oa_motion_plan_destroy(plan);
    }
    {
        Fixture fixture;
        oa_motion_plan *plan = joint_plan(fixture, OA_LEFT, 0U, 0.2);
        const auto report = plan_report(plan);
        auto request = request_for(report);
        request.stop_kind = OA_STOP_CONTROLLED;
        std::uint64_t command = 0U;
        CHECK(oa_controller_execute(fixture.controller, plan, &request, &command) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 10000000U) == OA_CONTROL_OK);
        oa_sim_fault fault{};
        init(fault);
        fault.side = OA_RIGHT;
        fault.fault_mask = 1U << 3U;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 20000000U) == OA_CONTROL_EFAULT);
        check_materialized_fault_stop(fixture, false, 1, 3);
        oa_motion_plan_destroy(plan);
    }
}

void test_cycle_deadline_dwell_and_stop_policy() {
    {
        Fixture fixture;
        oa_motion_plan *plan = joint_plan(fixture, OA_LEFT, 0U, 0.1);
        const auto report = plan_report(plan);
        auto request = request_for(report);
        request.stop_kind = OA_STOP_DISABLE;
        std::uint64_t command = 0U;
        CHECK(oa_controller_execute(fixture.controller, plan, &request, &command) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 20000000U) == OA_CONTROL_ETIMEOUT);
        oa_snapshot state{};
        init(state);
        CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_OK);
        CHECK(state.arm[0].status[0] == 0U);
        oa_motion_plan_destroy(plan);
    }
    {
        Fixture fixture;
        oa_motion_plan *plan = joint_plan(fixture, OA_LEFT, 0U, 0.1);
        const auto report = plan_report(plan);
        auto request = request_for(report);
        request.stop_kind = OA_STOP_CONTROLLED;
        std::uint64_t command = 0U;
        CHECK(oa_controller_execute(fixture.controller, plan, &request, &command) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 20000000U) == OA_CONTROL_ETIMEOUT);
        oa_snapshot state{};
        init(state);
        CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_OK);
        CHECK(state.arm[0].status[0] == 1U);
        oa_motion_plan_destroy(plan);
    }
    {
        Fixture fixture;
        oa_motion_plan *plan = joint_plan(fixture, OA_LEFT, 0U, 0.1);
        const auto report = plan_report(plan);
        auto request = request_for(report);
        request.stop_kind = OA_STOP_CONTROLLED;
        std::uint64_t command = 0U;
        CHECK(oa_controller_execute(fixture.controller, plan, &request, &command) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 60000000U) == OA_CONTROL_ESTALE);
        check_materialized_fault_stop(fixture, false);
        oa_motion_plan_destroy(plan);
    }
    {
        Fixture fixture;
        oa_motion_plan *plan = joint_plan(fixture, OA_LEFT, 0U, 0.1);
        const auto report = plan_report(plan);
        auto request = request_for(report);
        request.stop_kind = OA_STOP_CONTROLLED;
        std::uint64_t command = 0U;
        CHECK(oa_controller_execute(fixture.controller, plan, &request, &command) == OA_CONTROL_OK);
        oa_sim_fault fault{};
        init(fault);
        fault.side = OA_RIGHT;
        fault.fault_mask = 1U << 3U;
        CHECK(oa_controller_sim_set_fault(fixture.controller, &fault) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, 60000000U) == OA_CONTROL_EFAULT);
        check_materialized_fault_stop(fixture, false, 1, 3);
        oa_motion_plan_destroy(plan);
    }
    {
        Fixture fixture;
        oa_motion_plan *plan = joint_plan(fixture, OA_LEFT, 0U, 0.1);
        const auto report = plan_report(plan);
        const auto command = execute(fixture, plan, report, 2000000000U);
        std::uint64_t now = 0U;
        bool measured_goal = false;
        while (!measured_goal && now < report.duration_ns + 1500000000ULL) {
            now += 10000000U;
            CHECK(oa_controller_advance(fixture.controller, now) == OA_CONTROL_OK);
            oa_snapshot state{};
            init(state);
            CHECK(oa_controller_snapshot(fixture.controller, &state) == OA_CONTROL_OK);
            measured_goal = now >= report.duration_ns &&
                            std::abs(state.arm[0].q[0] - report.target_q[0][0]) <= 5.0e-4 &&
                            std::abs(state.arm[0].dq[0]) <= 2.0e-2;
            for (;;) {
                oa_event event{};
                init(event);
                const oa_control_status status =
                    oa_controller_poll_event(fixture.controller, 0U, &event);
                if (status == OA_CONTROL_ETIMEOUT) break;
                CHECK(status == OA_CONTROL_OK);
                CHECK(event.kind != OA_EVENT_COMPLETED);
            }
        }
        CHECK(measured_goal);
        for (std::size_t repeat = 0; repeat < 10U; ++repeat) {
            CHECK(oa_controller_advance(fixture.controller, now) == OA_CONTROL_OK);
        }
        CHECK(!has_event(fixture.controller, OA_EVENT_COMPLETED, command));
        CHECK(oa_controller_advance(fixture.controller, now + 10000000U) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, now + 20000000U) == OA_CONTROL_OK);
        CHECK(oa_controller_advance(fixture.controller, now + 30000000U) == OA_CONTROL_OK);
        CHECK(has_event(fixture.controller, OA_EVENT_COMPLETED, command));
        oa_motion_plan_destroy(plan);
    }
}

}  // namespace

int main() {
    test_manifest_validation();
    test_abi_and_physical_gate();
    test_mapping_kinematics_and_joint_convergence();
    test_frozen_encoder_never_completes();
    test_stale_feedback_and_paired_fault_stop();
    test_collision_rejection_and_limits();
    test_paired_tcp_measured_convergence();
    test_faults_gate_arming_and_idle_motion();
    test_plan_ownership_and_start_drift();
    test_coherent_feedback_skew_and_partial_send();
    test_delayed_feedback_public_abi_oracle();
    test_feedback_generation_atomicity_and_bounds();
    test_completion_requires_distinct_delivered_generations();
    test_delayed_feedback_is_retired_by_lifecycle_transitions();
    test_cartesian_path_policies_and_scene_binding();
    test_watchdog_estop_event_overflow_and_concurrency();
    test_invalid_handles_and_transactional_create();
    test_registry_storage_is_bounded();
    test_fault_stop_policy_by_cause();
    test_cycle_deadline_dwell_and_stop_policy();
    std::puts("openarm_control: all tests passed");
    return 0;
}
