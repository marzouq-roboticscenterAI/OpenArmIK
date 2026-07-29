/* SPDX-License-Identifier: Apache-2.0 */
#ifndef OPENARM_CONTROL_CORE_HPP
#define OPENARM_CONTROL_CORE_HPP

#include "kinematics.hpp"
#include "openarm_control.h"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace openarm::control {

constexpr std::uint32_t kAllJoints = 0x7fU;

class Manifest final {
public:
    explicit Manifest(const oa_manifest_config &config);
    [[nodiscard]] const oa_manifest_config &config() const noexcept { return config_; }

private:
    oa_manifest_config config_{};
};

struct MeasuredMotor {
    double raw_q{};
    double raw_dq{};
    double raw_tau{};
    std::uint8_t status{};
    std::uint8_t mos_c{25U};
    std::uint8_t coil_c{25U};
    std::uint64_t t_ns{};
    bool valid{};
};

struct FeedbackFrame {
    std::array<std::uint8_t, 8> data{};
    std::uint64_t t_ns{};
};

class DamiaoMotorSimulator final {
public:
    explicit DamiaoMotorSimulator(const oa_motor_config &config);
    void set_enabled(bool enabled) noexcept;
    void set_fault(std::uint8_t status) noexcept;
    void command(double q_model, double dq_model) noexcept;
    bool step(double dt_s, std::uint64_t feedback_ns, bool frozen,
              bool dropped) noexcept;
    void force_state(double q_model, double dq_model,
                     std::uint64_t feedback_ns) noexcept;
    [[nodiscard]] const MeasuredMotor &measured() const noexcept { return measured_; }
    [[nodiscard]] double mapped_q() const noexcept;
    [[nodiscard]] double mapped_dq() const noexcept;
    [[nodiscard]] double mapped_tau() const noexcept;

private:
    oa_motor_config config_{};
    MeasuredMotor measured_{};
    FeedbackFrame feedback_frame_{};
    double plant_raw_q_{};
    double plant_raw_dq_{};
    double command_raw_q_{};
    double command_raw_dq_{};
    bool enabled_{};
    std::uint8_t fault_status_{};
};

class FakeTransport final {
public:
    void record_complete_cycle() noexcept { ++complete_cycles_; }
    [[nodiscard]] std::uint64_t complete_cycles() const noexcept { return complete_cycles_; }

private:
    std::uint64_t complete_cycles_{};
};

class ArmRuntime final {
public:
    explicit ArmRuntime(const oa_arm_config &config);
    void set_enabled(bool enabled) noexcept;
    void set_fault_mask(std::uint32_t mask, std::uint8_t status) noexcept;
    void set_injection(std::uint32_t freeze_mask, std::uint32_t drop_mask,
                       std::uint32_t fault_mask, std::uint8_t fault_status,
                       std::uint32_t command_fail_mask,
                       std::uint64_t feedback_delay_ns) noexcept;
    bool command_and_step(const JointVector &q_reference,
                          const JointVector &dq_reference,
                          std::uint64_t now_ns, double dt_s) noexcept;
    void force_state(const JointVector &q, const JointVector &dq,
                     std::uint64_t now_ns) noexcept;
    void materialize_stop(bool enabled_hold, std::uint64_t now_ns) noexcept;
    [[nodiscard]] oa_arm_snapshot snapshot(std::uint64_t now_ns,
                                           std::uint64_t timeout_ns) const noexcept;
    [[nodiscard]] JointVector measured_q() const noexcept;
    [[nodiscard]] JointVector measured_dq() const noexcept;
    [[nodiscard]] std::uint64_t feedback_sequence() const noexcept { return feedback_seq_; }
    [[nodiscard]] bool complete_fresh(std::uint64_t now_ns,
                                      std::uint64_t timeout_ns) const noexcept;
    [[nodiscard]] bool fault_free(std::uint64_t now_ns,
                                  std::uint64_t timeout_ns) const noexcept;
    [[nodiscard]] bool all_disabled(std::uint64_t now_ns,
                                    std::uint64_t timeout_ns) const noexcept;
    [[nodiscard]] std::uint64_t generation_timestamp() const noexcept {
        return generation_timestamp_;
    }
    [[nodiscard]] const FakeTransport &transport() const noexcept { return transport_; }

private:
    std::array<DamiaoMotorSimulator, 7> motor_;
    FakeTransport transport_{};
    std::uint64_t feedback_seq_{};
    std::uint32_t freeze_mask_{};
    std::uint32_t drop_mask_{};
    std::uint32_t fault_mask_{};
    std::uint32_t command_fail_mask_{};
    std::uint64_t feedback_delay_ns_{};
    std::uint32_t generation_mask_{};
    std::uint64_t generation_timestamp_{};
};

class MotionPlan final {
public:
    static constexpr std::size_t kMaxWaypoints = 33U;
    std::uint32_t kind{};
    std::uint32_t active_arm_mask{};
    std::uint64_t manifest_revision{};
    std::uint64_t model_revision{};
    std::uint64_t collision_scene_revision{};
    std::uint64_t controller_instance{};
    std::uint64_t verify_epoch{};
    std::array<std::uint64_t, 2> seed_seq{};
    std::uint64_t expiry_ns{};
    std::uint64_t duration_ns{};
    std::array<JointVector, 2> start_q{};
    std::array<JointVector, 2> target_q{};
    std::array<double, 2> joint_position_tolerance{1.0e-4, 1.0e-4};
    std::array<double, 2> joint_velocity_tolerance{1.0e-3, 1.0e-3};
    std::array<double, 2> tcp_tolerance{};
    std::array<std::array<double, 3>, 2> target_tcp{};
    std::array<std::array<double, 3>, 2> achieved_tcp{};
    std::array<double, 2> tcp_residual{};
    bool collision_checked{};
    std::size_t waypoint_count{};
    std::array<std::array<JointVector, kMaxWaypoints>, 2> waypoint_q{};
    std::array<std::uint64_t, kMaxWaypoints> waypoint_time_ns{};
};

class Controller final {
public:
    Controller(std::shared_ptr<const Manifest> manifest,
               const oa_controller_options &options);
    oa_status open_and_verify(oa_verify_report &out) noexcept;
    oa_status snapshot(oa_snapshot &out) noexcept;
    oa_status kinematics(std::uint32_t side, std::uint64_t required_seq,
                         oa_arm_kinematics &out) noexcept;
    oa_status challenge(oa_arm_challenge &out) noexcept;
    oa_status arm(const oa_arm_challenge &challenge) noexcept;
    oa_status plan_joint(const oa_joint_move &request,
                         std::unique_ptr<MotionPlan> &out) noexcept;
    oa_status plan_paired(const oa_paired_tcp_move &request,
                          std::unique_ptr<MotionPlan> &out) noexcept;
    oa_status execute(const MotionPlan &plan, const oa_execute_request &request,
                      std::uint64_t &command_id) noexcept;
    oa_status advance(std::uint64_t monotonic_ns) noexcept;
    oa_status set_sim_fault(const oa_sim_fault &fault) noexcept;
    oa_status set_sim_state(const oa_sim_state &state) noexcept;
    oa_status heartbeat(std::uint64_t command_id,
                        std::uint64_t producer_deadline_ns) noexcept;
    oa_status set_interlock(bool estop_active, bool deadman_active) noexcept;
    oa_status set_collision_scene_revision(std::uint64_t revision) noexcept;
    oa_status stop(std::uint32_t stop_kind) noexcept;
    oa_status disarm(std::uint64_t deadline_ns) noexcept;
    oa_status reset(const oa_reset_request &request) noexcept;
    oa_status poll_event(oa_event &out) noexcept;
    [[nodiscard]] std::uint32_t lifecycle() const noexcept { return lifecycle_; }

private:
    void publish(std::uint32_t kind, oa_status cause, std::uint64_t command_id) noexcept;
    void materialize_fault_stop(bool enabled_hold) noexcept;
    void latch_fault(oa_status cause,
                     bool controlled_stop_available = false) noexcept;
    [[nodiscard]] bool fresh() const noexcept;
    [[nodiscard]] bool healthy() const noexcept;
    [[nodiscard]] bool disabled() const noexcept;
    [[nodiscard]] bool start_pose_matches(const MotionPlan &plan) const noexcept;
    [[nodiscard]] bool collision_allowed() const noexcept;
    [[nodiscard]] std::uint64_t trajectory_duration(
        const std::array<JointVector, 2> &start,
        const std::array<JointVector, 2> &target,
        double velocity_scale, double acceleration_scale,
        double jerk_scale, std::uint32_t arm_mask) const noexcept;
    [[nodiscard]] bool measured_at_goal() noexcept;

    std::shared_ptr<const Manifest> manifest_;
    oa_controller_options options_{};
    std::array<ArmRuntime, 2> arm_;
    std::uint32_t lifecycle_{OA_LIFECYCLE_CLOSED};
    std::uint64_t now_ns_{};
    std::uint64_t verify_epoch_{};
    std::uint64_t nonce_counter_{0x4f50454e41524dULL};
    std::uint64_t outstanding_nonce_{};
    std::uint64_t outstanding_challenge_expiry_{};
    std::uint64_t next_command_id_{1U};
    std::uint64_t command_id_{};
    std::uint64_t command_start_ns_{};
    std::uint64_t command_expiry_ns_{};
    std::uint64_t producer_deadline_ns_{};
    std::uint64_t settle_start_ns_{};
    std::uint32_t active_stop_kind_{OA_STOP_DISABLE};
    bool command_started_{};
    bool settling_published_{};
    bool deadman_active_{true};
    std::uint64_t instance_id_{};
    std::optional<MotionPlan> executing_{};
    std::array<oa_event, 64> events_{};
    std::size_t event_head_{};
    std::size_t event_count_{};
};

}  // namespace openarm::control
#endif
