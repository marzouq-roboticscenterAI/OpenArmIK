/* SPDX-License-Identifier: Apache-2.0 */
/* The keepout geometry lives in the model library, whose header carries the
 * legacy unprefixed status names. Suppress them for this translation unit
 * before any public header is seen; this file uses the OA_CONTROL_* names
 * throughout and never relies on the legacy aliases. */
#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1

#include "control_core.hpp"
#include "openarm_collision.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace openarm::control {
namespace {

bool finite(const double value) noexcept { return std::isfinite(value); }

bool terminated(const char *text, const std::size_t capacity) noexcept {
    return std::memchr(text, '\0', capacity) != nullptr;
}

std::array<double, 3> codec_spans(const std::uint32_t motor_type) {
    switch (motor_type) {
        case OA_MOTOR_DM8009:
            return {12.5, 45.0, 54.0};
        case OA_MOTOR_DM4340:
            return {12.5, 10.0, 28.0};
        case OA_MOTOR_DM4310:
            return {12.5, 30.0, 10.0};
        default:
            throw std::invalid_argument("unsupported motor profile");
    }
}

double expected_gear(const std::uint32_t motor_type) {
    switch (motor_type) {
        case OA_MOTOR_DM8009:
            return 9.0;
        case OA_MOTOR_DM4340:
            return 40.0;
        case OA_MOTOR_DM4310:
            return 10.0;
        default:
            throw std::invalid_argument("unsupported motor profile");
    }
}

double smoothstep7(const double u) noexcept {
    const double x = std::clamp(u, 0.0, 1.0);
    return x * x * x * x * (35.0 + x * (-84.0 + x * (70.0 - 20.0 * x)));
}

double smoothstep7_derivative(const double u) noexcept {
    const double x = std::clamp(u, 0.0, 1.0);
    const double one_minus = 1.0 - x;
    return 140.0 * x * x * x * one_minus * one_minus * one_minus;
}

bool valid_scale(const double value) noexcept {
    return finite(value) && value > 0.0 && value <= 1.0;
}

std::uint32_t encode_field(const double value, const double span,
                           const std::uint32_t maximum) noexcept {
    const double unit = (std::clamp(value, -span, span) + span) / (2.0 * span);
    return static_cast<std::uint32_t>(std::llround(unit * static_cast<double>(maximum)));
}

double decode_field(const std::uint32_t value, const double span,
                    const std::uint32_t maximum) noexcept {
    return static_cast<double>(value) / static_cast<double>(maximum) * (2.0 * span) - span;
}

std::atomic<std::uint64_t> next_controller_instance{1U};

bool configure_contact_thresholds(
    const Manifest &manifest, const double request_threshold_nm[7],
    const double request_fraction, const std::uint32_t request_persistence_cycles,
    MotionPlan &plan) noexcept {
    plan.contact_monitored = true;
    plan.contact_persistence_cycles =
        request_persistence_cycles == 0U
            ? oa_control_default_contact_persistence_cycles()
            : request_persistence_cycles;
    const double fraction = request_fraction > 0.0
                                ? request_fraction
                                : oa_control_default_contact_torque_fraction();
    for (std::size_t side = 0; side < 2U; ++side) {
        for (std::size_t joint = 0; joint < 7U; ++joint) {
            double threshold = request_threshold_nm[joint];
            if (!std::isfinite(threshold) || threshold <= 0.0) {
                threshold = fraction * manifest.config().arm[side].motor[joint].tmax_nm;
            }
            if (!std::isfinite(threshold) || threshold <= 0.0) {
                return false;
            }
            plan.contact_threshold_nm[side][joint] = threshold;
        }
    }
    return true;
}

}  // namespace

Manifest::Manifest(const oa_manifest_config &config) : config_(config) {
    if (config.struct_size < sizeof(config) || config.abi_version != OA_CONTROL_ABI_V1 ||
        config.manifest_revision == 0U || config.model_revision == 0U) {
        throw std::invalid_argument("invalid manifest record");
    }
    std::unordered_set<std::string> serials;
    for (std::size_t side = 0; side < 2U; ++side) {
        const auto &arm = config.arm[side];
        if (arm.struct_size < sizeof(arm) || arm.abi_version != OA_CONTROL_ABI_V1 ||
            !terminated(arm.bus_name, sizeof(arm.bus_name)) || arm.bus_name[0] == '\0') {
            throw std::invalid_argument("invalid arm record");
        }
        std::unordered_set<std::uint32_t> send_ids;
        std::unordered_set<std::uint32_t> receive_ids;
        for (std::size_t joint = 0; joint < 7U; ++joint) {
            const auto &motor = arm.motor[joint];
            const std::uint32_t expected_type =
                joint < 2U ? OA_MOTOR_DM8009 : (joint < 4U ? OA_MOTOR_DM4340 : OA_MOTOR_DM4310);
            const std::string expected_name = std::string("openarm_") +
                                              (side == 0U ? "left_joint" : "right_joint") +
                                              std::to_string(joint + 1U);
            if (motor.struct_size < sizeof(motor) ||
                motor.abi_version != OA_CONTROL_ABI_V1 || motor.joint_index != joint ||
                motor.motor_type != expected_type || motor.send_id == 0U ||
                motor.send_id > 0x7ffU || motor.receive_id == 0U ||
                motor.receive_id > 0x7ffU || motor.embedded_motor_id > 0x0fU ||
                motor.control_mode != 1U || motor.bitrate == 0U ||
                motor.timeout_ticks == 0U || motor.hardware_version == 0U ||
                motor.software_version == 0U || motor.firmware_subversion == 0U ||
                !terminated(motor.serial, sizeof(motor.serial)) || motor.serial[0] == '\0' ||
                !terminated(motor.joint_name, sizeof(motor.joint_name)) ||
                expected_name != motor.joint_name || !finite(motor.q_scale) ||
                std::abs(std::abs(motor.q_scale) - 1.0) > 1.0e-12 ||
                !finite(motor.q_offset_rad) || !finite(motor.lower_rad) ||
                !finite(motor.upper_rad) || motor.lower_rad >= motor.upper_rad ||
                !finite(motor.max_velocity_rad_s) || motor.max_velocity_rad_s <= 0.0 ||
                !finite(motor.max_acceleration_rad_s2) ||
                motor.max_acceleration_rad_s2 <= 0.0 ||
                !finite(motor.max_jerk_rad_s3) || motor.max_jerk_rad_s3 <= 0.0 ||
                (motor.direction != -1 && motor.direction != 1)) {
                throw std::invalid_argument("invalid motor record");
            }
            const auto spans = codec_spans(motor.motor_type);
            if (std::abs(motor.pmax_rad - spans[0]) > 1.0e-12 ||
                std::abs(motor.vmax_rad_s - spans[1]) > 1.0e-12 ||
                std::abs(motor.tmax_nm - spans[2]) > 1.0e-12 ||
                std::abs(motor.gear_ratio - expected_gear(motor.motor_type)) > 1.0e-12) {
                throw std::invalid_argument("codec or gear profile mismatch");
            }
            double model_lower = 0.0;
            double model_upper = 0.0;
            if (!model_limit(static_cast<std::uint32_t>(side), joint, model_lower, model_upper) ||
                motor.lower_rad < model_lower - 1.0e-12 ||
                motor.upper_rad > model_upper + 1.0e-12 ||
                motor.q_offset_rad < -motor.pmax_rad ||
                motor.q_offset_rad > motor.pmax_rad ||
                std::abs((motor.lower_rad - motor.q_offset_rad) / motor.q_scale) >
                    motor.pmax_rad + 1.0e-12 ||
                std::abs((motor.upper_rad - motor.q_offset_rad) / motor.q_scale) >
                    motor.pmax_rad + 1.0e-12 ||
                !send_ids.insert(motor.send_id).second ||
                !receive_ids.insert(motor.receive_id).second ||
                !serials.insert(motor.serial).second) {
                throw std::invalid_argument("ambiguous or unsafe motor configuration");
            }
        }
    }
    if (std::strcmp(config.arm[0].bus_name, config.arm[1].bus_name) == 0) {
        throw std::invalid_argument("bimanual arms require distinct buses");
    }
}

DamiaoMotorSimulator::DamiaoMotorSimulator(const oa_motor_config &config) : config_(config) {
    plant_raw_q_ = -config_.q_offset_rad / config_.q_scale;
    command_raw_q_ = plant_raw_q_;
}

void DamiaoMotorSimulator::set_enabled(const bool enabled) noexcept {
    enabled_ = enabled;
}

void DamiaoMotorSimulator::set_fault(const std::uint8_t status) noexcept {
    fault_status_ = status;
}

void DamiaoMotorSimulator::command(const double q_model, const double dq_model) noexcept {
    command_raw_q_ = (q_model - config_.q_offset_rad) / config_.q_scale;
    command_raw_dq_ = dq_model / config_.q_scale;
}

void DamiaoMotorSimulator::set_blocked(const bool blocked,
                                       const double reaction_gain_nm_per_rad) noexcept {
    blocked_ = blocked;
    /* A stiff joint reaches full rated torque within a fraction of a radian of
     * unclosed error. The default is derived from the motor's own tmax so every
     * joint saturates at the same relative overshoot. */
    const double fallback = config_.tmax_nm / 0.15;
    reaction_gain_nm_per_rad_ =
        (std::isfinite(reaction_gain_nm_per_rad) && reaction_gain_nm_per_rad > 0.0)
            ? reaction_gain_nm_per_rad
            : fallback;
}

FeedbackFrame DamiaoMotorSimulator::capture(const double dt_s,
                                            const std::uint64_t capture_ns,
                                            const bool frozen) noexcept {
    /* A blocked plant is held by an external obstacle. The reference keeps
     * advancing, so the position error the controller must overcome grows and
     * the reaction torque reported below rises with it. */
    if (!frozen && !blocked_) {
        const double max_velocity = config_.max_velocity_rad_s;
        const double max_acceleration = config_.max_acceleration_rad_s2;
        const double position_error = command_raw_q_ - plant_raw_q_;
        const double desired_velocity =
            std::clamp(command_raw_dq_ + 8.0 * position_error, -max_velocity, max_velocity);
        const double acceleration =
            std::clamp((desired_velocity - plant_raw_dq_) / 0.08,
                       -max_acceleration, max_acceleration);
        const double previous_error = position_error;
        plant_raw_dq_ = std::clamp(plant_raw_dq_ + acceleration * dt_s,
                                   -max_velocity, max_velocity);
        plant_raw_q_ += plant_raw_dq_ * dt_s;
        const double remaining_error = command_raw_q_ - plant_raw_q_;
        if (previous_error * remaining_error < 0.0 &&
            std::abs(command_raw_dq_) < 1.0e-9) {
            plant_raw_q_ = command_raw_q_;
            plant_raw_dq_ = 0.0;
        }
        if (std::abs(remaining_error) < 5.0e-5 &&
            std::abs(command_raw_dq_) < 1.0e-9 && std::abs(plant_raw_dq_) < 1.0e-3) {
            plant_raw_q_ = command_raw_q_;
            plant_raw_dq_ = 0.0;
        }
    }
    const std::uint32_t q_field = encode_field(plant_raw_q_, config_.pmax_rad, 65535U);
    const std::uint32_t dq_field = encode_field(plant_raw_dq_, config_.vmax_rad_s, 4095U);
    /* Free motion in this simulator is frictionless and gravity-free, so the
     * only torque reported is the servo effort spent against an external
     * contact. While blocked the reference keeps advancing and the plant does
     * not, so the unclosed error, and with it the reported torque, grows until
     * it saturates. Encoding is the ordinary DaMiao torque field, so the
     * contact monitor observes it through the same quantized feedback path as
     * real hardware would. */
    double contact_tau_model_nm = 0.0;
    if (blocked_) {
        const double error_model_rad =
            (command_raw_q_ - plant_raw_q_) * config_.q_scale;
        contact_tau_model_nm = reaction_gain_nm_per_rad_ * error_model_rad;
        if (!std::isfinite(contact_tau_model_nm)) {
            contact_tau_model_nm = 0.0;
        }
    }
    const std::uint32_t tau_field =
        encode_field(contact_tau_model_nm * config_.q_scale, config_.tmax_nm, 4095U);
    const std::uint8_t status = fault_status_ != 0U ? fault_status_ : (enabled_ ? 1U : 0U);
    FeedbackFrame frame{};
    frame.data[0] = static_cast<std::uint8_t>(
        (static_cast<std::uint32_t>(status) << 4U) | (config_.embedded_motor_id & 0x0fU));
    frame.data[1] = static_cast<std::uint8_t>((q_field >> 8U) & 0xffU);
    frame.data[2] = static_cast<std::uint8_t>(q_field & 0xffU);
    frame.data[3] = static_cast<std::uint8_t>((dq_field >> 4U) & 0xffU);
    frame.data[4] = static_cast<std::uint8_t>(((dq_field & 0x0fU) << 4U) |
                                             ((tau_field >> 8U) & 0x0fU));
    frame.data[5] = static_cast<std::uint8_t>(tau_field & 0xffU);
    frame.data[6] = 25U;
    frame.data[7] = 25U;
    frame.t_ns = capture_ns;
    return frame;
}

void DamiaoMotorSimulator::publish(const FeedbackFrame &frame) noexcept {
    measured_.status = static_cast<std::uint8_t>(frame.data[0] >> 4U);
    const std::uint32_t decoded_q =
        (static_cast<std::uint32_t>(frame.data[1]) << 8U) | frame.data[2];
    const std::uint32_t decoded_dq =
        (static_cast<std::uint32_t>(frame.data[3]) << 4U) |
        (static_cast<std::uint32_t>(frame.data[4]) >> 4U);
    const std::uint32_t decoded_tau =
        ((static_cast<std::uint32_t>(frame.data[4]) & 0x0fU) << 8U) |
        frame.data[5];
    measured_.raw_q = decode_field(decoded_q, config_.pmax_rad, 65535U);
    measured_.raw_dq = decode_field(decoded_dq, config_.vmax_rad_s, 4095U);
    measured_.raw_tau = decode_field(decoded_tau, config_.tmax_nm, 4095U);
    measured_.mos_c = frame.data[6];
    measured_.coil_c = frame.data[7];
    measured_.t_ns = frame.t_ns;
    measured_.valid = true;
}

void DamiaoMotorSimulator::force_state(const double q_model, const double dq_model,
                                       const std::uint64_t feedback_ns) noexcept {
    plant_raw_q_ = (q_model - config_.q_offset_rad) / config_.q_scale;
    plant_raw_dq_ = dq_model / config_.q_scale;
    command_raw_q_ = plant_raw_q_;
    command_raw_dq_ = plant_raw_dq_;
    publish(capture(0.0, feedback_ns, true));
}

double DamiaoMotorSimulator::mapped_q() const noexcept {
    return config_.q_scale * measured_.raw_q + config_.q_offset_rad;
}

double DamiaoMotorSimulator::mapped_dq() const noexcept {
    return config_.q_scale * measured_.raw_dq;
}

double DamiaoMotorSimulator::mapped_tau() const noexcept {
    return measured_.raw_tau / config_.q_scale;
}

ArmRuntime::ArmRuntime(const oa_arm_config &config)
    : motor_{DamiaoMotorSimulator(config.motor[0]), DamiaoMotorSimulator(config.motor[1]),
             DamiaoMotorSimulator(config.motor[2]), DamiaoMotorSimulator(config.motor[3]),
             DamiaoMotorSimulator(config.motor[4]), DamiaoMotorSimulator(config.motor[5]),
             DamiaoMotorSimulator(config.motor[6])} {}

void ArmRuntime::set_enabled(const bool enabled) noexcept {
    for (auto &motor : motor_) {
        motor.set_enabled(enabled);
    }
}

void ArmRuntime::set_fault_mask(const std::uint32_t mask,
                                const std::uint8_t status) noexcept {
    for (std::size_t joint = 0; joint < motor_.size(); ++joint) {
        motor_[joint].set_fault((mask & (1U << joint)) != 0U ? status : 0U);
    }
}

void ArmRuntime::set_injection(const std::uint32_t freeze_mask,
                               const std::uint32_t drop_mask,
                               const std::uint32_t fault_mask,
                               const std::uint8_t fault_status,
                               const std::uint32_t command_fail_mask,
                               const std::uint64_t feedback_delay_ns) noexcept {
    freeze_mask_ = freeze_mask & kAllJoints;
    drop_mask_ = drop_mask & kAllJoints;
    fault_mask_ = fault_mask & kAllJoints;
    command_fail_mask_ = command_fail_mask & kAllJoints;
    feedback_delay_ns_ = feedback_delay_ns;
    set_fault_mask(fault_mask_, fault_mask_ == 0U ? 0U : fault_status);
}

bool ArmRuntime::command_and_step(const JointVector &q_reference,
                                  const JointVector &dq_reference,
                                  const std::uint64_t now_ns,
                                  const double dt_s) noexcept {
    if (command_fail_mask_ != 0U) {
        generation_mask_ = 0U;
        return false;
    }
    if (feedback_delay_ns_ > std::numeric_limits<std::uint64_t>::max() - now_ns) {
        return false;
    }
    FeedbackGeneration generation{};
    generation.capture_ns = now_ns;
    generation.ready_ns = now_ns + feedback_delay_ns_;
    generation.member_mask = kAllJoints & ~drop_mask_;
    for (std::size_t joint = 0; joint < motor_.size(); ++joint) {
        motor_[joint].command(q_reference[joint], dq_reference[joint]);
        generation.frame[joint] = motor_[joint].capture(
            dt_s, now_ns, (freeze_mask_ & (1U << joint)) != 0U);
    }
    if (!enqueue(std::move(generation)) || !publish_due(now_ns)) {
        return false;
    }
    transport_.record_complete_cycle();
    return true;
}

bool ArmRuntime::enqueue(FeedbackGeneration generation) noexcept {
    if (feedback_queue_count_ == feedback_queue_.size()) {
        return false;
    }
    const std::size_t tail =
        (feedback_queue_head_ + feedback_queue_count_) % feedback_queue_.size();
    feedback_queue_[tail] = std::move(generation);
    ++feedback_queue_count_;
    return true;
}

bool ArmRuntime::publish_due(const std::uint64_t now_ns) noexcept {
    while (feedback_queue_count_ != 0U) {
        const FeedbackGeneration &generation = feedback_queue_[feedback_queue_head_];
        if (generation.ready_ns > now_ns) {
            break;
        }
        const std::uint32_t member_mask = generation.member_mask;
        if (member_mask == kAllJoints) {
            if (feedback_seq_ == std::numeric_limits<std::uint64_t>::max()) {
                return false;
            }
            for (std::size_t joint = 0; joint < motor_.size(); ++joint) {
                motor_[joint].publish(generation.frame[joint]);
            }
            generation_mask_ = kAllJoints;
            generation_timestamp_ = generation.capture_ns;
            ++feedback_seq_;
        } else {
            generation_mask_ = member_mask;
        }
        feedback_queue_head_ = (feedback_queue_head_ + 1U) % feedback_queue_.size();
        --feedback_queue_count_;
        if (member_mask != kAllJoints) {
            break;
        }
    }
    return true;
}

void ArmRuntime::clear_queue() noexcept {
    feedback_queue_head_ = 0U;
    feedback_queue_count_ = 0U;
}

void ArmRuntime::retire_pending_feedback() noexcept {
    clear_queue();
}

void ArmRuntime::force_state(const JointVector &q, const JointVector &dq,
                             const std::uint64_t now_ns) noexcept {
    clear_queue();
    generation_mask_ = kAllJoints;
    generation_timestamp_ = now_ns;
    for (std::size_t joint = 0; joint < motor_.size(); ++joint) {
        motor_[joint].force_state(q[joint], dq[joint], now_ns);
    }
    ++feedback_seq_;
}

void ArmRuntime::materialize_stop(const bool enabled_hold,
                                  const std::uint64_t now_ns) noexcept {
    const JointVector held_q = measured_q();
    set_enabled(enabled_hold);
    force_state(held_q, JointVector{}, now_ns);
}

oa_arm_snapshot ArmRuntime::snapshot(const std::uint64_t now_ns,
                                     const std::uint64_t timeout_ns) const noexcept {
    oa_arm_snapshot out{};
    out.struct_size = sizeof(out);
    out.abi_version = OA_CONTROL_ABI_V1;
    out.feedback_seq = feedback_seq_;
    out.expected_mask = kAllJoints;
    for (std::size_t joint = 0; joint < motor_.size(); ++joint) {
        const auto &measured = motor_[joint].measured();
        const bool in_generation = (generation_mask_ & (1U << joint)) != 0U;
        const bool fresh = in_generation && measured.valid && now_ns >= measured.t_ns &&
                           now_ns - measured.t_ns <= timeout_ns;
        if (fresh) {
            out.fresh_mask |= 1U << joint;
            out.t_ns = std::max(out.t_ns, measured.t_ns);
        }
        if (measured.status >= 8U) {
            out.fault_mask |= 1U << joint;
        }
        out.raw_q[joint] = measured.raw_q;
        out.raw_dq[joint] = measured.raw_dq;
        out.raw_tau[joint] = measured.raw_tau;
        out.q[joint] = motor_[joint].mapped_q();
        out.dq[joint] = motor_[joint].mapped_dq();
        out.tau[joint] = motor_[joint].mapped_tau();
        out.status[joint] = measured.status;
        out.mos_c[joint] = measured.mos_c;
        out.coil_c[joint] = measured.coil_c;
    }
    return out;
}

JointVector ArmRuntime::measured_q() const noexcept {
    JointVector q{};
    for (std::size_t joint = 0; joint < motor_.size(); ++joint) {
        q[joint] = motor_[joint].mapped_q();
    }
    return q;
}

JointVector ArmRuntime::measured_dq() const noexcept {
    JointVector dq{};
    for (std::size_t joint = 0; joint < motor_.size(); ++joint) {
        dq[joint] = motor_[joint].mapped_dq();
    }
    return dq;
}

JointVector ArmRuntime::measured_tau() const noexcept {
    JointVector tau{};
    for (std::size_t joint = 0; joint < motor_.size(); ++joint) {
        tau[joint] = motor_[joint].mapped_tau();
    }
    return tau;
}

void ArmRuntime::set_blocked(const bool blocked,
                             const double reaction_gain_nm_per_rad) noexcept {
    for (auto &motor : motor_) {
        motor.set_blocked(blocked, reaction_gain_nm_per_rad);
    }
}

void ArmRuntime::clear_contact() noexcept {
    for (auto &motor : motor_) {
        motor.set_blocked(false, 0.0);
    }
}

bool ArmRuntime::complete_fresh(const std::uint64_t now_ns,
                                const std::uint64_t timeout_ns) const noexcept {
    const auto state = snapshot(now_ns, timeout_ns);
    return state.fresh_mask == state.expected_mask;
}

bool ArmRuntime::fault_free(const std::uint64_t now_ns,
                            const std::uint64_t timeout_ns) const noexcept {
    return snapshot(now_ns, timeout_ns).fault_mask == 0U;
}

bool ArmRuntime::all_disabled(const std::uint64_t now_ns,
                              const std::uint64_t timeout_ns) const noexcept {
    const auto state = snapshot(now_ns, timeout_ns);
    if (state.fresh_mask != kAllJoints || state.fault_mask != 0U) {
        return false;
    }
    return std::all_of(std::begin(state.status), std::end(state.status),
                       [](const std::uint8_t status) { return status == 0U; });
}

Controller::Controller(std::shared_ptr<const Manifest> manifest,
                       const oa_controller_options &options)
    : manifest_(std::move(manifest)), options_(options),
      arm_{ArmRuntime(manifest_->config().arm[0]), ArmRuntime(manifest_->config().arm[1])},
      instance_id_(next_controller_instance.fetch_add(1U)) {
    if (options_.backend != OA_BACKEND_VIRTUAL && options_.backend != OA_BACKEND_PHYSICAL) {
        throw std::invalid_argument("invalid backend");
    }
    if (options_.collision_policy != OA_COLLISION_REJECT_ALL &&
        options_.collision_policy != OA_COLLISION_VIRTUAL_UNCHECKED) {
        throw std::invalid_argument("invalid collision policy");
    }
    if (options_.cycle_ns == 0U || options_.feedback_timeout_ns < options_.cycle_ns ||
        options_.max_cross_bus_skew_ns == 0U ||
        options_.cycle_ns > std::numeric_limits<std::uint64_t>::max() / 3U) {
        throw std::invalid_argument("invalid timing policy");
    }
    if (options_.backend == OA_BACKEND_PHYSICAL &&
        options_.collision_policy == OA_COLLISION_VIRTUAL_UNCHECKED) {
        throw std::invalid_argument("unchecked collision policy is virtual-only");
    }
    if (options_.collision_policy == OA_COLLISION_VIRTUAL_UNCHECKED &&
        options_.collision_scene_revision == 0U) {
        throw std::invalid_argument("virtual collision scene revision is required");
    }
}

oa_control_status Controller::open_and_verify(oa_verify_report &out) noexcept {
    if (lifecycle_ != OA_LIFECYCLE_CLOSED) {
        return OA_CONTROL_ESTATE;
    }
    lifecycle_ = OA_LIFECYCLE_VERIFYING;
    if (options_.backend == OA_BACKEND_PHYSICAL) {
        lifecycle_ = OA_LIFECYCLE_CLOSED;
        out.verify_epoch = 0U;
        out.verified_mask = 0U;
        out.failure_mask = 0x3U;
        return OA_CONTROL_EUNSUPPORTED;
    }
    const JointVector zero{};
    for (auto &runtime : arm_) {
        const JointVector verified_q = verify_epoch_ == 0U ? zero : runtime.measured_q();
        runtime.set_enabled(false);
        runtime.force_state(verified_q, zero, now_ns_);
    }
    ++verify_epoch_;
    lifecycle_ = OA_LIFECYCLE_DISARMED;
    out.verify_epoch = verify_epoch_;
    out.verified_mask = 0x3U;
    out.failure_mask = 0U;
    return publish(OA_EVENT_VERIFIED, OA_CONTROL_OK, 0U) ? OA_CONTROL_OK : OA_CONTROL_EBUSY;
}

oa_control_status Controller::snapshot(oa_snapshot &out) noexcept {
    if (lifecycle_ == OA_LIFECYCLE_CLOSED || lifecycle_ == OA_LIFECYCLE_VERIFYING) {
        return OA_CONTROL_ESTATE;
    }
    out.arm[0] = arm_[0].snapshot(now_ns_, options_.feedback_timeout_ns);
    out.arm[1] = arm_[1].snapshot(now_ns_, options_.feedback_timeout_ns);
    out.manifest_revision = manifest_->config().manifest_revision;
    out.model_revision = manifest_->config().model_revision;
    out.max_cross_bus_skew_ns = out.arm[0].t_ns > out.arm[1].t_ns
                                    ? out.arm[0].t_ns - out.arm[1].t_ns
                                    : out.arm[1].t_ns - out.arm[0].t_ns;
    out.lifecycle = lifecycle_;
    return OA_CONTROL_OK;
}

oa_control_status Controller::kinematics(const std::uint32_t side,
                                 const std::uint64_t required_seq,
                                 oa_arm_kinematics &out) noexcept {
    if (side > OA_RIGHT || !fresh()) {
        return side > OA_RIGHT ? OA_CONTROL_EINVAL : OA_CONTROL_ESTALE;
    }
    const auto &runtime = arm_[side];
    if (runtime.feedback_sequence() != required_seq) {
        return OA_CONTROL_ESTALE;
    }
    KinematicResult result{};
    const auto q = runtime.measured_q();
    if (!forward(side, q, result)) {
        return OA_CONTROL_EFAULT;
    }
    out.feedback_seq = required_seq;
    std::copy(q.begin(), q.end(), out.q);
    for (std::size_t joint = 0; joint < 7U; ++joint) {
        std::copy(result.joint_xyz[joint].begin(), result.joint_xyz[joint].end(),
                  out.joint_xyz_m[joint]);
        std::copy(result.joint_axis[joint].begin(), result.joint_axis[joint].end(),
                  out.joint_axis_body[joint]);
    }
    std::copy(result.tcp_transform.begin(), result.tcp_transform.end(), out.tcp_transform);
    std::copy(result.tcp_xyz.begin(), result.tcp_xyz.end(), out.tcp_xyz_m);
    return OA_CONTROL_OK;
}

oa_control_status Controller::challenge(oa_arm_challenge &out) noexcept {
    const bool reset_challenge = lifecycle_ == OA_LIFECYCLE_FAULT ||
                                 lifecycle_ == OA_LIFECYCLE_ESTOP;
    if (lifecycle_ != OA_LIFECYCLE_DISARMED && !reset_challenge) {
        return OA_CONTROL_ESTATE;
    }
    if (!reset_challenge && (!fresh() || !healthy() || !disabled())) {
        if (!healthy()) {
            return latch_fault(OA_CONTROL_EFAULT);
        }
        return OA_CONTROL_ESTALE;
    }
    outstanding_nonce_ = ++nonce_counter_;
    outstanding_challenge_expiry_ = now_ns_ + UINT64_C(1000000000);
    out.verify_epoch = verify_epoch_;
    out.nonce = outstanding_nonce_;
    out.expiry_ns = outstanding_challenge_expiry_;
    return OA_CONTROL_OK;
}

oa_control_status Controller::arm(const oa_arm_challenge &challenge_record) noexcept {
    if (lifecycle_ != OA_LIFECYCLE_DISARMED) {
        return OA_CONTROL_ESTATE;
    }
    if (challenge_record.verify_epoch != verify_epoch_ ||
        challenge_record.nonce != outstanding_nonce_ || outstanding_nonce_ == 0U ||
        challenge_record.expiry_ns != outstanding_challenge_expiry_ ||
        now_ns_ > outstanding_challenge_expiry_) {
        return OA_CONTROL_EIDENTITY;
    }
    if (!fresh()) {
        return OA_CONTROL_ESTALE;
    }
    if (!healthy() || !disabled() || !deadman_active_) {
        return latch_fault(!deadman_active_ ? OA_CONTROL_EESTOP : OA_CONTROL_EFAULT);
    }
    lifecycle_ = OA_LIFECYCLE_ARMING;
    for (auto &runtime : arm_) {
        runtime.set_enabled(true);
        runtime.force_state(runtime.measured_q(), JointVector{}, now_ns_);
    }
    outstanding_nonce_ = 0U;
    lifecycle_ = OA_LIFECYCLE_ARMED_IDLE;
    return publish(OA_EVENT_ARMED, OA_CONTROL_OK, 0U) ? OA_CONTROL_OK : OA_CONTROL_EBUSY;
}

bool Controller::collision_allowed() const noexcept {
    return options_.backend == OA_BACKEND_VIRTUAL &&
           options_.collision_policy == OA_COLLISION_VIRTUAL_UNCHECKED;
}

oa_control_status Controller::plan_joint(const oa_joint_move &request,
                                 std::unique_ptr<MotionPlan> &out) noexcept {
    if (lifecycle_ != OA_LIFECYCLE_ARMED_IDLE) {
        return OA_CONTROL_ESTATE;
    }
    if (!fresh()) {
        return OA_CONTROL_ESTALE;
    }
    if (!healthy()) {
        return latch_fault(OA_CONTROL_EFAULT);
    }
    if (!collision_allowed()) {
        return OA_CONTROL_ECOLLISION;
    }
    if (request.side > OA_RIGHT || request.joint >= 7U ||
        request.required_feedback_seq != arm_[request.side].feedback_sequence() ||
        request.expiry_ns <= now_ns_ || !finite(request.target_rad) ||
        !valid_scale(request.velocity_scale) || !valid_scale(request.acceleration_scale) ||
        !valid_scale(request.jerk_scale) || !finite(request.position_tol_rad) ||
        request.position_tol_rad <= 0.0 || !finite(request.velocity_tol_rad_s) ||
        request.velocity_tol_rad_s <= 0.0) {
        return OA_CONTROL_EINVAL;
    }
    const auto &motor = manifest_->config().arm[request.side].motor[request.joint];
    const double raw_target = (request.target_rad - motor.q_offset_rad) / motor.q_scale;
    if (request.target_rad < motor.lower_rad || request.target_rad > motor.upper_rad ||
        std::abs(raw_target) > motor.pmax_rad) {
        return OA_CONTROL_ELIMIT;
    }
    auto plan = std::make_unique<MotionPlan>();
    plan->kind = OA_PLAN_JOINT;
    plan->active_arm_mask = 1U << request.side;
    plan->manifest_revision = manifest_->config().manifest_revision;
    plan->model_revision = manifest_->config().model_revision;
    plan->collision_scene_revision = options_.collision_scene_revision;
    plan->controller_instance = instance_id_;
    plan->verify_epoch = verify_epoch_;
    plan->expiry_ns = request.expiry_ns;
    for (std::size_t side = 0; side < 2U; ++side) {
        plan->seed_seq[side] = arm_[side].feedback_sequence();
        plan->start_q[side] = arm_[side].measured_q();
        plan->target_q[side] = plan->start_q[side];
    }
    plan->target_q[request.side][request.joint] = request.target_rad;
    plan->joint_position_tolerance[request.side] = request.position_tol_rad;
    plan->joint_velocity_tolerance[request.side] = request.velocity_tol_rad_s;
    plan->duration_ns = trajectory_duration(plan->start_q, plan->target_q,
                                             request.velocity_scale,
                                             request.acceleration_scale,
                                             request.jerk_scale,
                                             plan->active_arm_mask);
    plan->waypoint_count = 2U;
    plan->waypoint_time_ns[0] = 0U;
    plan->waypoint_time_ns[1] = plan->duration_ns;
    for (std::size_t side = 0; side < 2U; ++side) {
        plan->waypoint_q[side][0] = plan->start_q[side];
        plan->waypoint_q[side][1] = plan->target_q[side];
    }
    out = std::move(plan);
    return OA_CONTROL_OK;
}

oa_control_status Controller::plan_preconditions() noexcept {
    /* The E-stop is sampled before anything else, so a latched stop can never be
     * overtaken by a newly submitted plan. */
    if (oa_estop_asserted() != 0U) {
        estop_latched_ = true;
        return OA_CONTROL_EESTOP;
    }
    if (lifecycle_ != OA_LIFECYCLE_ARMED_IDLE) {
        return OA_CONTROL_ESTATE;
    }
    if (!fresh()) {
        return OA_CONTROL_ESTALE;
    }
    if (!healthy()) {
        return latch_fault(OA_CONTROL_EFAULT);
    }
    if (!collision_allowed()) {
        return OA_CONTROL_ECOLLISION;
    }
    return OA_CONTROL_OK;
}

oa_control_status Controller::plan_paired(const oa_paired_tcp_move &request,
                                  std::unique_ptr<MotionPlan> &out) noexcept {
    const oa_control_status precondition = plan_preconditions();
    if (precondition != OA_CONTROL_OK) {
        return precondition;
    }
    PairedPlanRequest common{};
    common.kind = OA_PLAN_PAIRED_TCP;
    common.expiry_ns = request.expiry_ns;
    common.required_feedback_seq = {request.required_feedback_seq[0],
                                    request.required_feedback_seq[1]};
    std::copy(request.left_tcp_m, request.left_tcp_m + 3, common.target_tcp[0].begin());
    std::copy(request.right_tcp_m, request.right_tcp_m + 3, common.target_tcp[1].begin());
    common.velocity_scale = request.velocity_scale;
    common.acceleration_scale = request.acceleration_scale;
    common.jerk_scale = request.jerk_scale;
    common.tcp_tol_m = request.tcp_tol_m;
    common.collision_scene_revision = request.collision_scene_revision;
    common.max_branch_step_rad = request.max_branch_step_rad;
    common.min_singular_value = request.min_singular_value;
    return plan_paired_common(common, out);
}

oa_control_status Controller::plan_paired_common(
    const PairedPlanRequest &request, std::unique_ptr<MotionPlan> &out) noexcept {
    if (request.expiry_ns <= now_ns_ ||
        request.required_feedback_seq[0] != arm_[0].feedback_sequence() ||
        request.required_feedback_seq[1] != arm_[1].feedback_sequence() ||
        !valid_scale(request.velocity_scale) || !valid_scale(request.acceleration_scale) ||
        !valid_scale(request.jerk_scale) || !finite(request.tcp_tol_m) ||
        request.tcp_tol_m <= 0.0 || !finite(request.max_branch_step_rad) ||
        request.max_branch_step_rad <= 0.0 || !finite(request.min_singular_value) ||
        request.min_singular_value < 0.0 ||
        request.collision_scene_revision != options_.collision_scene_revision) {
        return OA_CONTROL_EINVAL;
    }
    auto plan = std::make_unique<MotionPlan>();
    plan->kind = request.kind;
    plan->active_arm_mask = 0x3U;
    plan->manifest_revision = manifest_->config().manifest_revision;
    plan->model_revision = manifest_->config().model_revision;
    plan->collision_scene_revision = request.collision_scene_revision;
    plan->contact_geometry_policy = request.contact_geometry_policy;
    plan->controller_instance = instance_id_;
    plan->verify_epoch = verify_epoch_;
    plan->expiry_ns = request.expiry_ns;
    plan->waypoint_count = 17U;
    for (std::size_t side = 0; side < 2U; ++side) {
        plan->seed_seq[side] = arm_[side].feedback_sequence();
        plan->start_q[side] = arm_[side].measured_q();
        plan->target_tcp[side] = request.target_tcp[side];
        for (const double value : plan->target_tcp[side]) {
            if (!finite(value)) {
                return OA_CONTROL_EINVAL;
            }
        }
        plan->tcp_tolerance[side] = request.tcp_tol_m;
        plan->joint_position_tolerance[side] = 5.0e-4;
        plan->joint_velocity_tolerance[side] = 2.0e-2;
        KinematicResult start_fk{};
        if (!forward(static_cast<std::uint32_t>(side), plan->start_q[side], start_fk)) {
            return OA_CONTROL_EFAULT;
        }
        plan->waypoint_q[side][0] = plan->start_q[side];
        JointVector predecessor = plan->start_q[side];
        for (std::size_t waypoint = 1U; waypoint < plan->waypoint_count; ++waypoint) {
            const double fraction = static_cast<double>(waypoint) /
                                    static_cast<double>(plan->waypoint_count - 1U);
            std::array<double, 3> waypoint_tcp{};
            for (std::size_t axis = 0; axis < 3U; ++axis) {
                waypoint_tcp[axis] = start_fk.tcp_xyz[axis] +
                                     fraction * (plan->target_tcp[side][axis] -
                                                 start_fk.tcp_xyz[axis]);
            }
            IkResult ik{};
            if (!inverse(static_cast<std::uint32_t>(side), waypoint_tcp,
                         predecessor, ik) || ik.residual > request.tcp_tol_m ||
                ik.min_singular_value < request.min_singular_value) {
                return OA_CONTROL_EUNREACHABLE;
            }
            for (std::size_t joint = 0; joint < 7U; ++joint) {
                const auto &motor = manifest_->config().arm[side].motor[joint];
                const double raw = (ik.q[joint] - motor.q_offset_rad) / motor.q_scale;
                if (ik.q[joint] < motor.lower_rad || ik.q[joint] > motor.upper_rad ||
                    std::abs(raw) > motor.pmax_rad ||
                    std::abs(ik.q[joint] - predecessor[joint]) >
                        request.max_branch_step_rad) {
                    return OA_CONTROL_EUNREACHABLE;
                }
            }
            plan->waypoint_q[side][waypoint] = ik.q;
            predecessor = ik.q;
            if (waypoint + 1U == plan->waypoint_count) {
                plan->target_q[side] = ik.q;
                plan->achieved_tcp[side] = ik.achieved;
                plan->tcp_residual[side] = ik.residual;
            }
        }
    }

    /* A proved terminal-cap contact must have one safe way out. Ordinary
     * paired planning normally carries no contact exception, but a command can
     * begin at the deliberate tangent pose left by converge. In that one
     * state, recognize a retreat only after checking every planned waypoint:
     * all protected pairs remain above the real-time floor, terminal
     * clearance never decreases while the pair is active, and the path ends
     * completely outside the scoped corridor. This is controller-side proof;
     * it does not trust the portal guard and it cannot authorize an approach. */
    if (request.contact_geometry_policy == OA_COLLISION_CONTACT_NONE) {
        KeepoutStatus strict_start{};
        KeepoutStatus scoped_start{};
        const std::array<JointVector, 2> start{plan->start_q[0], plan->start_q[1]};
        const bool strict_clear = keepout_clear(
            start, strict_start, OA_COLLISION_CONTACT_NONE);
        const bool scoped_clear = keepout_clear(
            start, scoped_start, OA_COLLISION_CONTACT_TERMINAL_CAPS);
        if (!strict_clear && scoped_clear && scoped_start.terminal_pair_active) {
            constexpr double kRetreatMonotonicEpsilonM = 1.0e-6;
            bool terminal_cleared = false;
            double previous_terminal_clearance =
                scoped_start.terminal_pair_clearance_m;
            for (std::size_t waypoint = 1U; waypoint < plan->waypoint_count; ++waypoint) {
                const std::array<JointVector, 2> q{
                    plan->waypoint_q[0][waypoint], plan->waypoint_q[1][waypoint]};
                KeepoutStatus scoped{};
                if (!keepout_clear(q, scoped, OA_COLLISION_CONTACT_TERMINAL_CAPS)) {
                    return OA_CONTROL_ECOLLISION;
                }
                if (scoped.terminal_pair_active) {
                    if (terminal_cleared ||
                        scoped.terminal_pair_clearance_m <
                            previous_terminal_clearance - kRetreatMonotonicEpsilonM) {
                        return OA_CONTROL_ECOLLISION;
                    }
                    previous_terminal_clearance = scoped.terminal_pair_clearance_m;
                } else {
                    terminal_cleared = true;
                }
            }
            if (!terminal_cleared) {
                return OA_CONTROL_ECOLLISION;
            }
            plan->contact_geometry_policy = OA_COLLISION_CONTACT_TERMINAL_CAPS;
        }
    }
    plan->collision_checked = false;
    plan->waypoint_time_ns[0] = 0U;
    for (std::size_t waypoint = 1U; waypoint < plan->waypoint_count; ++waypoint) {
        std::array<JointVector, 2> from{};
        std::array<JointVector, 2> to{};
        for (std::size_t side = 0; side < 2U; ++side) {
            from[side] = plan->waypoint_q[side][waypoint - 1U];
            to[side] = plan->waypoint_q[side][waypoint];
        }
        const std::uint64_t segment = trajectory_duration(
            from, to, request.velocity_scale, request.acceleration_scale,
            request.jerk_scale, 0x3U);
        if (segment > std::numeric_limits<std::uint64_t>::max() -
                          plan->waypoint_time_ns[waypoint - 1U]) {
            return OA_CONTROL_EUNREACHABLE;
        }
        plan->waypoint_time_ns[waypoint] =
            plan->waypoint_time_ns[waypoint - 1U] + segment;
    }
    plan->duration_ns = plan->waypoint_time_ns[plan->waypoint_count - 1U];
    out = std::move(plan);
    return OA_CONTROL_OK;
}

namespace {

std::array<double, 3> difference(const std::array<double, 3> &a,
                                 const std::array<double, 3> &b) noexcept {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

double magnitude(const std::array<double, 3> &value) noexcept {
    return std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

bool finite_point(const double value[3]) noexcept {
    return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

}  // namespace

oa_control_status Controller::plan_centroid(const oa_centroid_tcp_move &request,
                                            std::unique_ptr<MotionPlan> &out) noexcept {
    const oa_control_status precondition = plan_preconditions();
    if (precondition != OA_CONTROL_OK) {
        return precondition;
    }
    if (!finite_point(request.target_centroid_m)) {
        return OA_CONTROL_EINVAL;
    }
    /* The midpoint is taken from measured feedback, never from a previous
     * command, so the translation is relative to where the arms actually are. */
    std::array<std::array<double, 3>, 2> measured_tcp{};
    for (std::size_t side = 0; side < 2U; ++side) {
        KinematicResult fk{};
        if (!forward(static_cast<std::uint32_t>(side), arm_[side].measured_q(), fk)) {
            return OA_CONTROL_EFAULT;
        }
        measured_tcp[side] = fk.tcp_xyz;
    }
    PairedPlanRequest common{};
    common.kind = OA_PLAN_CENTROID_TCP;
    common.expiry_ns = request.expiry_ns;
    common.required_feedback_seq = {request.required_feedback_seq[0],
                                    request.required_feedback_seq[1]};
    for (std::size_t axis = 0; axis < 3U; ++axis) {
        const double centroid = 0.5 * (measured_tcp[0][axis] + measured_tcp[1][axis]);
        const double delta = request.target_centroid_m[axis] - centroid;
        if (!std::isfinite(delta)) {
            return OA_CONTROL_EINVAL;
        }
        for (std::size_t side = 0; side < 2U; ++side) {
            const double target = measured_tcp[side][axis] + delta;
            if (!std::isfinite(target)) {
                return OA_CONTROL_EINVAL;
            }
            common.target_tcp[side][axis] = target;
        }
    }
    common.velocity_scale = request.velocity_scale;
    common.acceleration_scale = request.acceleration_scale;
    common.jerk_scale = request.jerk_scale;
    common.tcp_tol_m = request.tcp_tol_m;
    common.collision_scene_revision = request.collision_scene_revision;
    common.max_branch_step_rad = request.max_branch_step_rad;
    common.min_singular_value = request.min_singular_value;
    return plan_paired_common(common, out);
}

oa_control_status Controller::plan_mirrored(const oa_mirrored_tcp_move &request,
                                            std::unique_ptr<MotionPlan> &out) noexcept {
    const oa_control_status precondition = plan_preconditions();
    if (precondition != OA_CONTROL_OK) {
        return precondition;
    }
    if (request.lead_side != OA_LEFT && request.lead_side != OA_RIGHT) {
        return OA_CONTROL_EINVAL;
    }
    if (!finite_point(request.lead_tcp_m)) {
        return OA_CONTROL_EINVAL;
    }
    const std::size_t lead = static_cast<std::size_t>(request.lead_side);
    const std::size_t follow = lead == 0U ? 1U : 0U;
    PairedPlanRequest common{};
    common.kind = OA_PLAN_MIRRORED_TCP;
    common.expiry_ns = request.expiry_ns;
    common.required_feedback_seq = {request.required_feedback_seq[0],
                                    request.required_feedback_seq[1]};
    /* The body sagittal plane is y = 0, so the mirror negates y only. */
    common.target_tcp[lead] = {request.lead_tcp_m[0], request.lead_tcp_m[1],
                               request.lead_tcp_m[2]};
    common.target_tcp[follow] = {request.lead_tcp_m[0], -request.lead_tcp_m[1],
                                 request.lead_tcp_m[2]};
    common.velocity_scale = request.velocity_scale;
    common.acceleration_scale = request.acceleration_scale;
    common.jerk_scale = request.jerk_scale;
    common.tcp_tol_m = request.tcp_tol_m;
    common.collision_scene_revision = request.collision_scene_revision;
    common.max_branch_step_rad = request.max_branch_step_rad;
    common.min_singular_value = request.min_singular_value;
    return plan_paired_common(common, out);
}

oa_control_status Controller::plan_converge(const oa_converge_tcp_move &request,
                                            std::unique_ptr<MotionPlan> &out) noexcept {
    const oa_control_status precondition = plan_preconditions();
    if (precondition != OA_CONTROL_OK) {
        return precondition;
    }
    if (!finite_point(request.target_m) || !std::isfinite(request.stop_distance_m) ||
        request.stop_distance_m < 0.0 || !std::isfinite(request.minimum_progress_m) ||
        request.minimum_progress_m < 0.0 ||
        !std::isfinite(request.contact_torque_fraction)) {
        return OA_CONTROL_EINVAL;
    }
    const std::array<double, 3> target{request.target_m[0], request.target_m[1],
                                       request.target_m[2]};
    PairedPlanRequest common{};
    common.kind = OA_PLAN_CONVERGE_TCP;
    common.expiry_ns = request.expiry_ns;
    common.required_feedback_seq = {request.required_feedback_seq[0],
                                    request.required_feedback_seq[1]};
    for (std::size_t side = 0; side < 2U; ++side) {
        KinematicResult fk{};
        if (!forward(static_cast<std::uint32_t>(side), arm_[side].measured_q(), fk)) {
            return OA_CONTROL_EFAULT;
        }
        const std::array<double, 3> ray = difference(target, fk.tcp_xyz);
        const double distance = magnitude(ray);
        if (!std::isfinite(distance)) {
            return OA_CONTROL_EINVAL;
        }
        /* Stop short of the convergence point along this arm's own approach
         * ray. An arm already inside the stop radius has nothing to travel. */
        const double travel = distance - request.stop_distance_m;
        if (travel < request.minimum_progress_m) {
            return OA_CONTROL_EUNREACHABLE;
        }
        const double scale = travel / distance;
        for (std::size_t axis = 0; axis < 3U; ++axis) {
            const double value = fk.tcp_xyz[axis] + ray[axis] * scale;
            if (!std::isfinite(value)) {
                return OA_CONTROL_EINVAL;
            }
            common.target_tcp[side][axis] = value;
        }
    }
    common.velocity_scale = request.velocity_scale;
    common.acceleration_scale = request.acceleration_scale;
    common.jerk_scale = request.jerk_scale;
    common.tcp_tol_m = request.tcp_tol_m;
    common.collision_scene_revision = request.collision_scene_revision;
    common.max_branch_step_rad = request.max_branch_step_rad;
    common.min_singular_value = request.min_singular_value;
    common.contact_geometry_policy = OA_COLLISION_CONTACT_TERMINAL_CAPS;
    const oa_control_status status = plan_paired_common(common, out);
    if (status != OA_CONTROL_OK) {
        return status;
    }
    if (!configure_contact_thresholds(*manifest_, request.contact_torque_nm,
                                      request.contact_torque_fraction,
                                      request.contact_persistence_cycles, *out)) {
        out.reset();
        return OA_CONTROL_EINVAL;
    }
    return OA_CONTROL_OK;
}

std::uint64_t Controller::trajectory_duration(
    const std::array<JointVector, 2> &start,
    const std::array<JointVector, 2> &target, const double velocity_scale,
    const double acceleration_scale, const double jerk_scale,
    const std::uint32_t arm_mask) const noexcept {
    double duration_s = static_cast<double>(options_.cycle_ns) * 10.0e-9;
    for (std::size_t side = 0; side < 2U; ++side) {
        if ((arm_mask & (1U << side)) == 0U) {
            continue;
        }
        for (std::size_t joint = 0; joint < 7U; ++joint) {
            const double distance = std::abs(target[side][joint] - start[side][joint]);
            const auto &limit = manifest_->config().arm[side].motor[joint];
            duration_s = std::max(duration_s,
                                  distance * 2.2 / (limit.max_velocity_rad_s * velocity_scale));
            duration_s = std::max(
                duration_s,
                std::sqrt(distance * 8.0 /
                          (limit.max_acceleration_rad_s2 * acceleration_scale)));
            duration_s = std::max(
                duration_s,
                std::cbrt(distance * 60.0 / (limit.max_jerk_rad_s3 * jerk_scale)));
        }
    }
    const long double ns = std::ceil(static_cast<long double>(duration_s) * 1.0e9L);
    return ns >= static_cast<long double>(std::numeric_limits<std::uint64_t>::max())
               ? std::numeric_limits<std::uint64_t>::max()
               : static_cast<std::uint64_t>(ns);
}

oa_control_status Controller::execute(const MotionPlan &plan, const oa_execute_request &request,
                              std::uint64_t &command_id) noexcept {
    if (lifecycle_ != OA_LIFECYCLE_ARMED_IDLE) {
        return OA_CONTROL_ESTATE;
    }
    if (!fresh()) {
        return OA_CONTROL_ESTALE;
    }
    if (!healthy()) {
        return latch_fault(OA_CONTROL_EFAULT);
    }
    if (plan.controller_instance != instance_id_ || plan.verify_epoch != verify_epoch_) {
        return OA_CONTROL_EIDENTITY;
    }
    if (plan.manifest_revision != manifest_->config().manifest_revision ||
        plan.model_revision != manifest_->config().model_revision ||
        plan.collision_scene_revision != options_.collision_scene_revision ||
        plan.seed_seq[0] != arm_[0].feedback_sequence() ||
        plan.seed_seq[1] != arm_[1].feedback_sequence() || !start_pose_matches(plan)) {
        return OA_CONTROL_ESTALE;
    }
    const std::uint64_t start_ns = request.start_ns == 0U ? now_ns_ : request.start_ns;
    if (start_ns < now_ns_ || request.expiry_ns <= start_ns ||
        request.expiry_ns > plan.expiry_ns || request.producer_deadline_ns <= now_ns_ ||
        (request.stop_kind != OA_STOP_DISABLE && request.stop_kind != OA_STOP_CONTROLLED) ||
        plan.duration_ns > request.expiry_ns - start_ns) {
        return OA_CONTROL_EINVAL;
    }
    executing_ = plan;
    reset_contact_report(*executing_);
    command_id_ = next_command_id_++;
    command_start_ns_ = start_ns;
    command_expiry_ns_ = request.expiry_ns;
    producer_deadline_ns_ = request.producer_deadline_ns;
    settle_start_ns_ = 0U;
    settle_feedback_seq_ = {};
    settle_feedback_intervals_ = 0U;
    active_stop_kind_ = request.stop_kind;
    command_started_ = start_ns == now_ns_;
    settling_published_ = false;
    lifecycle_ = OA_LIFECYCLE_EXECUTING;
    command_id = command_id_;
    return publish(command_started_ ? OA_EVENT_STARTED : OA_EVENT_QUEUED,
                   OA_CONTROL_OK, command_id_) ? OA_CONTROL_OK : OA_CONTROL_EBUSY;
}

bool Controller::keepout_clear(const std::array<JointVector, 2> &q,
                               KeepoutStatus &status,
                               const std::uint32_t contact_geometry_policy) const noexcept {
    status = {};
    status.minimum_clearance_m = -std::numeric_limits<double>::infinity();
    std::array<oa_fk_result, 2> fk{};
    for (std::size_t side = 0; side < 2U; ++side) {
        const oa_model *model = side == 0U ? oa_model_left_v10_bimanual()
                                            : oa_model_right_v10_bimanual();
        if (oa_fk(model, q[side].data(), &fk[side]) != OA_MODEL_OK) {
            return false;
        }
    }
    /* Monitor at the intervention floor, not the planning gate: a planner may
     * legitimately accept a path sitting exactly on the planning clearance, and
     * the measured arm always trails its reference. */
    oa_collision_report report{};
    oa_collision_contact_evidence evidence{};
    const oa_model_status evaluated = oa_collision_evaluate_scoped_fk_with_threshold(
        &fk[0], &fk[1], oa_collision_intervention_clearance_m(), contact_geometry_policy,
        &report, &evidence);
    status.violation = report.violation;
    status.side = report.side;
    status.segment_a = report.segment_a;
    status.segment_b = report.segment_b;
    status.minimum_clearance_m = report.minimum_clearance_m;
    status.terminal_pair_active = evidence.terminal_pair_active != 0U;
    status.terminal_pair_clearance_m = evidence.terminal_pair_clearance_m;
    status.tcp_separation_m = evidence.tcp_separation_m;
    status.claw_contact_active = evidence.claw_contact_active != 0U;
    status.claw_hand_gap_m = evidence.claw_hand_gap_m;
    status.minimum_other_claw_gap_m = evidence.minimum_other_claw_gap_m;
    if (evaluated != OA_MODEL_OK) {
        return false;
    }
    status.clear = report.clear != 0U;
    return status.clear;
}

/* Clearance must fall by more than this within one cycle to count as an
 * approach. Feedback is quantized, so exact equality is not reliable. */
constexpr double kClearanceWorseningEpsilon = 1.0e-6;

bool Controller::monitor_keepout() noexcept {
    /* Evaluated every cycle from measured feedback, not from the plan. A plan
     * validated at submission time can still be carried into a violation by
     * disturbance, drift, or a stale start pose, so the gate is re-proved
     * continuously while the arms are moving. */
    const std::array<JointVector, 2> measured{arm_[0].measured_q(), arm_[1].measured_q()};
    KeepoutStatus status{};
    const double previous = last_clearance_m_;
    last_clearance_m_ = status.minimum_clearance_m;
    const std::uint32_t policy = !executing_.has_value()
                                     ? OA_COLLISION_CONTACT_NONE
                                     : executing_->contact_geometry_policy;
    if (keepout_clear(measured, status, policy)) {
        contact_report_.minimum_clearance_m = status.minimum_clearance_m;
        last_clearance_m_ = status.minimum_clearance_m;
        /* Stop a converge command when measured FK reaches the expanded rail
         * safety envelope. This applies to virtual and physical backends: the
         * pinned STL meshes remain 25 mm apart instead of relying on physical
         * resistance after contact. */
        if (executing_->contact_monitored &&
            policy == OA_COLLISION_CONTACT_TERMINAL_CAPS &&
            status.terminal_pair_active &&
            status.claw_contact_active) {
            contact_report_.cause = OA_STOP_CAUSE_CONTACT;
            contact_report_.contact_detected = 1U;
            contact_report_.stop_monotonic_ns = now_ns_;
            for (std::size_t side = 0; side < 2U; ++side) {
                contact_report_.stop_feedback_seq[side] = arm_[side].feedback_sequence();
                const auto q = arm_[side].measured_q();
                std::copy(q.begin(), q.end(), contact_report_.stopped_q_rad[side]);
                KinematicResult fk{};
                if (forward(static_cast<std::uint32_t>(side), q, fk)) {
                    std::copy(fk.tcp_xyz.begin(), fk.tcp_xyz.end(),
                              contact_report_.stopped_tcp_m[side]);
                }
            }
            return false;
        }
        return true;
    }
    last_clearance_m_ = status.minimum_clearance_m;
    /* Below the floor but opening up: let it run.
     *
     * A command that legitimately ends inside the floor, converge above all,
     * leaves the next command starting there too. Vetoing purely on the
     * absolute value trapped the arms: every retreat was halted on its first
     * cycle and the pose became inescapable. Intervening only while clearance
     * is actively worsening still stops any approach, which is what the monitor
     * is for, while allowing the arms to back out of where they already are.
     *
     * The test is "not worsening" rather than "strictly improving": for the
     * first cycles of a retreat the arms have barely moved, so clearance is
     * equal rather than greater, and a strict test halted every escape on
     * cycle one. */
    if (std::isfinite(previous) && std::isfinite(status.minimum_clearance_m) &&
        status.minimum_clearance_m >= previous - kClearanceWorseningEpsilon) {
        contact_report_.minimum_clearance_m = status.minimum_clearance_m;
        return true;
    }
    contact_report_.cause = OA_STOP_CAUSE_KEEPOUT;
    contact_report_.keepout_violation = status.violation;
    contact_report_.keepout_side = status.side;
    contact_report_.keepout_segment_a = status.segment_a;
    contact_report_.keepout_segment_b = status.segment_b;
    contact_report_.minimum_clearance_m = status.minimum_clearance_m;
    contact_report_.stop_monotonic_ns = now_ns_;
    for (std::size_t side = 0; side < 2U; ++side) {
        contact_report_.stop_feedback_seq[side] = arm_[side].feedback_sequence();
        const auto q = arm_[side].measured_q();
        std::copy(q.begin(), q.end(), contact_report_.stopped_q_rad[side]);
        KinematicResult fk{};
        if (forward(static_cast<std::uint32_t>(side), q, fk)) {
            std::copy(fk.tcp_xyz.begin(), fk.tcp_xyz.end(),
                      contact_report_.stopped_tcp_m[side]);
        }
    }
    return false;
}

bool Controller::monitor_contact() noexcept {
    if (!executing_ || !executing_->contact_monitored) {
        return true;
    }
    std::uint32_t side_mask = 0U;
    std::array<std::uint32_t, 2> joint_mask{};
    std::array<JointVector, 2> measured_tau{};
    for (std::size_t side = 0; side < 2U; ++side) {
        measured_tau[side] = arm_[side].measured_tau();
        for (std::size_t joint = 0; joint < 7U; ++joint) {
            const double threshold = executing_->contact_threshold_nm[side][joint];
            if (threshold > 0.0 && std::abs(measured_tau[side][joint]) >= threshold) {
                side_mask |= (1U << side);
                joint_mask[side] |= (1U << joint);
            }
        }
    }
    if (side_mask == 0U) {
        contact_streak_ = 0U;
        return true;
    }
    /* Require the threshold to hold for consecutive cycles so a single noisy
     * quantized torque sample cannot end a command. */
    ++contact_streak_;
    if (contact_streak_ < executing_->contact_persistence_cycles) {
        return true;
    }
    contact_report_.cause = OA_STOP_CAUSE_CONTACT;
    contact_report_.contact_detected = 1U;
    contact_report_.contact_side_mask = side_mask;
    contact_report_.stop_monotonic_ns = now_ns_;
    for (std::size_t side = 0; side < 2U; ++side) {
        contact_report_.contact_joint_mask[side] = joint_mask[side];
        contact_report_.stop_feedback_seq[side] = arm_[side].feedback_sequence();
        std::copy(measured_tau[side].begin(), measured_tau[side].end(),
                  contact_report_.contact_torque_nm[side]);
        std::copy(executing_->contact_threshold_nm[side].begin(),
                  executing_->contact_threshold_nm[side].end(),
                  contact_report_.threshold_torque_nm[side]);
        const auto q = arm_[side].measured_q();
        std::copy(q.begin(), q.end(), contact_report_.stopped_q_rad[side]);
        KinematicResult fk{};
        if (forward(static_cast<std::uint32_t>(side), q, fk)) {
            std::copy(fk.tcp_xyz.begin(), fk.tcp_xyz.end(),
                      contact_report_.stopped_tcp_m[side]);
        }
    }
    return false;
}

void Controller::apply_sim_contact() noexcept {
    /* The obstacle decides only whether the arm is held. The torque the contact
     * monitor sees is the servo effort that holding produces, which the motor
     * simulator derives from the reference overshoot it cannot close. */
    for (std::size_t side = 0; side < 2U; ++side) {
        const oa_sim_contact &contact = sim_contact_[side];
        if (contact.enabled == 0U) {
            arm_[side].clear_contact();
            continue;
        }
        KinematicResult fk{};
        if (!forward(static_cast<std::uint32_t>(side), arm_[side].measured_q(), fk)) {
            arm_[side].clear_contact();
            continue;
        }
        const std::array<double, 3> offset{fk.tcp_xyz[0] - contact.center_m[0],
                                           fk.tcp_xyz[1] - contact.center_m[1],
                                           fk.tcp_xyz[2] - contact.center_m[2]};
        const double distance = magnitude(offset);
        if (!std::isfinite(distance) || !(distance < contact.radius_m)) {
            arm_[side].clear_contact();
            continue;
        }
        arm_[side].set_blocked(true, contact.reaction_gain_nm_per_rad);
    }
}

void Controller::reset_contact_report(const MotionPlan &plan) noexcept {
    contact_report_ = {};
    contact_report_.struct_size = static_cast<std::uint32_t>(sizeof(contact_report_));
    contact_report_.abi_version = OA_CONTROL_ABI_V1;
    contact_report_.cause = OA_STOP_CAUSE_NONE;
    contact_report_.minimum_clearance_m = std::numeric_limits<double>::quiet_NaN();
    contact_streak_ = 0U;
    /* Seed from the pose the command actually starts in, so a command that
     * begins inside the floor is judged on whether it improves from there. */
    KeepoutStatus start{};
    (void)keepout_clear({arm_[0].measured_q(), arm_[1].measured_q()}, start,
                        plan.contact_geometry_policy);
    last_clearance_m_ = start.minimum_clearance_m;
}

oa_control_status Controller::complete_on_contact() noexcept {
    /* Contact and keepout stops end the command successfully: the arms did what
     * was asked of them and halted on physical evidence. The controller returns
     * to armed-idle holding the measured pose rather than latching a fault. */
    const auto completed_id = command_id_;
    materialize_stop(true);
    executing_.reset();
    command_id_ = 0U;
    settle_start_ns_ = 0U;
    settle_feedback_seq_ = {};
    settle_feedback_intervals_ = 0U;
    settling_published_ = false;
    lifecycle_ = OA_LIFECYCLE_ARMED_IDLE;
    return publish(OA_EVENT_STOPPED, OA_CONTROL_OK, completed_id) ? OA_CONTROL_OK
                                                                  : OA_CONTROL_EBUSY;
}

oa_control_status Controller::contact_report(oa_contact_report &out) const noexcept {
    const std::uint32_t size = out.struct_size;
    const std::uint32_t version = out.abi_version;
    if (size < sizeof(oa_contact_report) || version != OA_CONTROL_ABI_V1) {
        return OA_CONTROL_EABI;
    }
    out = contact_report_;
    out.struct_size = static_cast<std::uint32_t>(sizeof(oa_contact_report));
    out.abi_version = OA_CONTROL_ABI_V1;
    return OA_CONTROL_OK;
}

oa_control_status Controller::set_sim_contact(const oa_sim_contact &contact) noexcept {
    if (options_.backend != OA_BACKEND_VIRTUAL) {
        return OA_CONTROL_EUNSUPPORTED;
    }
    if (contact.side != OA_LEFT && contact.side != OA_RIGHT) {
        return OA_CONTROL_EINVAL;
    }
    if (contact.enabled != 0U) {
        /* A zero reaction gain selects the per-motor default. A negative gain
         * would push the arm into the obstacle, so it is a caller error rather
         * than something to silently clamp. */
        if (!finite_point(contact.center_m) || !std::isfinite(contact.radius_m) ||
            contact.radius_m <= 0.0 || !std::isfinite(contact.reaction_gain_nm_per_rad) ||
            contact.reaction_gain_nm_per_rad < 0.0) {
            return OA_CONTROL_EINVAL;
        }
    }
    sim_contact_[static_cast<std::size_t>(contact.side)] = contact;
    if (contact.enabled == 0U) {
        arm_[static_cast<std::size_t>(contact.side)].clear_contact();
    }
    return OA_CONTROL_OK;
}

oa_control_status Controller::advance(const std::uint64_t monotonic_ns) noexcept {
    /* The emergency stop is sampled first, unconditionally, in every lifecycle
     * state and before any argument validation, feedback processing, or command
     * work. Nothing in this function can run ahead of it and nothing can mask
     * it: the latch is a lock-free global that any thread or signal handler can
     * set at any instant. */
    if (oa_estop_asserted() != 0U) {
        if (!estop_latched_) {
            estop_latched_ = true;
            contact_report_.cause = OA_STOP_CAUSE_ESTOP;
            contact_report_.stop_monotonic_ns = monotonic_ns >= now_ns_ ? monotonic_ns
                                                                        : now_ns_;
            if (monotonic_ns >= now_ns_) {
                now_ns_ = monotonic_ns;
            }
            return set_interlock(true, deadman_active_);
        }
        if (monotonic_ns >= now_ns_) {
            now_ns_ = monotonic_ns;
        }
        return OA_CONTROL_EESTOP;
    }
    estop_latched_ = false;
    if (monotonic_ns < now_ns_) {
        return OA_CONTROL_EINVAL;
    }
    if (monotonic_ns == now_ns_) {
        if (lifecycle_ == OA_LIFECYCLE_ARMED_IDLE ||
            lifecycle_ == OA_LIFECYCLE_EXECUTING ||
            lifecycle_ == OA_LIFECYCLE_DISARMED) {
            return OA_CONTROL_OK;
        }
        return lifecycle_ == OA_LIFECYCLE_FAULT ? OA_CONTROL_EFAULT : OA_CONTROL_ESTATE;
    }
    const std::uint64_t previous_ns = now_ns_;
    now_ns_ = monotonic_ns;
    if (lifecycle_ != OA_LIFECYCLE_ARMED_IDLE && lifecycle_ != OA_LIFECYCLE_EXECUTING &&
        lifecycle_ != OA_LIFECYCLE_DISARMED) {
        return lifecycle_ == OA_LIFECYCLE_FAULT ? OA_CONTROL_EFAULT : OA_CONTROL_ESTATE;
    }
    if (lifecycle_ == OA_LIFECYCLE_ARMED_IDLE ||
        lifecycle_ == OA_LIFECYCLE_EXECUTING) {
        if (!healthy()) {
            return latch_fault(OA_CONTROL_EFAULT);
        }
        const oa_control_status integrity = feedback_integrity();
        if (integrity != OA_CONTROL_OK) {
            return latch_fault(integrity);
        }
        if (now_ns_ - previous_ns > options_.cycle_ns) {
            return latch_fault(OA_CONTROL_ETIMEOUT, true);
        }
    }

    std::array<JointVector, 2> q_reference{arm_[0].measured_q(), arm_[1].measured_q()};
    std::array<JointVector, 2> dq_reference{};
    if (lifecycle_ == OA_LIFECYCLE_EXECUTING) {
        if (now_ns_ > producer_deadline_ns_) {
            return latch_fault(OA_CONTROL_ESTALE, true);
        }
        if (now_ns_ > command_expiry_ns_) {
            return latch_fault(OA_CONTROL_ETIMEOUT, true);
        }
        if (!healthy()) {
            return latch_fault(OA_CONTROL_EFAULT);
        }
        if (!command_started_ && now_ns_ >= command_start_ns_) {
            if (!fresh() || !start_pose_matches(*executing_)) {
                return latch_fault(OA_CONTROL_ESTALE);
            }
            command_started_ = true;
            if (!publish(OA_EVENT_STARTED, OA_CONTROL_OK, command_id_)) {
                return OA_CONTROL_EBUSY;
            }
        }
        const std::uint64_t elapsed = now_ns_ <= command_start_ns_ ? 0U : now_ns_ - command_start_ns_;
        std::size_t segment = 1U;
        while (segment + 1U < executing_->waypoint_count &&
               elapsed > executing_->waypoint_time_ns[segment]) {
            ++segment;
        }
        const std::uint64_t segment_start = executing_->waypoint_time_ns[segment - 1U];
        const std::uint64_t segment_end = executing_->waypoint_time_ns[segment];
        const std::uint64_t segment_duration = segment_end - segment_start;
        const std::uint64_t segment_elapsed =
            elapsed <= segment_start ? 0U : std::min(elapsed - segment_start, segment_duration);
        const double u = segment_duration == 0U
                             ? 1.0
                             : static_cast<double>(segment_elapsed) /
                                   static_cast<double>(segment_duration);
        const double position_scale = smoothstep7(u);
        const double velocity_scale = segment_elapsed >= segment_duration
                                          ? 0.0
                                          : smoothstep7_derivative(u) * 1.0e9 /
                                                static_cast<double>(segment_duration);
        for (std::size_t side = 0; side < 2U; ++side) {
            for (std::size_t joint = 0; joint < 7U; ++joint) {
                const double from = executing_->waypoint_q[side][segment - 1U][joint];
                const double delta = executing_->waypoint_q[side][segment][joint] - from;
                q_reference[side][joint] = from +
                                           delta * position_scale;
                dq_reference[side][joint] = delta * velocity_scale;
            }
        }
    }
    /* Resolve virtual mechanical resistance before stepping the plant so an
     * obstacle blocks this cycle's motion rather than the next one. */
    apply_sim_contact();
    const double dt_s = static_cast<double>(now_ns_ - previous_ns) * 1.0e-9;
    for (std::size_t side = 0; side < 2U; ++side) {
        if (!arm_[side].command_and_step(q_reference[side], dq_reference[side],
                                         now_ns_, dt_s)) {
            return latch_fault(OA_CONTROL_ECAN);
        }
    }
    if (!arm_[0].complete_fresh(now_ns_, options_.feedback_timeout_ns) ||
        !arm_[1].complete_fresh(now_ns_, options_.feedback_timeout_ns)) {
        return latch_fault(OA_CONTROL_ESTALE);
    }
    const std::uint64_t skew = arm_[0].generation_timestamp() > arm_[1].generation_timestamp()
                                   ? arm_[0].generation_timestamp() -
                                         arm_[1].generation_timestamp()
                                   : arm_[1].generation_timestamp() -
                                         arm_[0].generation_timestamp();
    if (skew > options_.max_cross_bus_skew_ns) {
        return latch_fault(OA_CONTROL_ECAN);
    }
    for (std::size_t side = 0; side < 2U; ++side) {
        if (arm_[side].snapshot(now_ns_, options_.feedback_timeout_ns).fault_mask != 0U) {
            return latch_fault(OA_CONTROL_EFAULT);
        }
    }
    /* Real-time monitors. Both are evaluated from the feedback published this
     * cycle, so they gate on where the arms measurably are rather than on where
     * the plan predicted they would be. Contact is checked first: when an arm
     * is resisted it is also the most likely to be approaching a keepout, and
     * halting on contact is the better-conditioned outcome of the two. */
    if (lifecycle_ == OA_LIFECYCLE_EXECUTING && command_started_) {
        if (!monitor_contact()) {
            return complete_on_contact();
        }
        if (!monitor_keepout()) {
            return complete_on_contact();
        }
    }
    if (lifecycle_ == OA_LIFECYCLE_EXECUTING &&
        now_ns_ >= command_start_ns_ + executing_->duration_ns) {
        if (measured_at_goal()) {
            if (!settling_published_) {
                if (!publish(OA_EVENT_SETTLING, OA_CONTROL_OK, command_id_)) {
                    return OA_CONTROL_EBUSY;
                }
                settling_published_ = true;
            }
            if (settle_start_ns_ == 0U) {
                settle_start_ns_ = now_ns_;
                settle_feedback_seq_ = {arm_[0].feedback_sequence(),
                                        arm_[1].feedback_sequence()};
                settle_feedback_intervals_ = 0U;
            } else {
                const std::array<std::uint64_t, 2> current_seq{
                    arm_[0].feedback_sequence(), arm_[1].feedback_sequence()};
                if (current_seq[0] != settle_feedback_seq_[0] &&
                    current_seq[1] != settle_feedback_seq_[1]) {
                    settle_feedback_seq_ = current_seq;
                    ++settle_feedback_intervals_;
                }
            }
            if (settle_feedback_intervals_ >= 3U &&
                now_ns_ - settle_start_ns_ >= options_.cycle_ns * 3U) {
                const auto completed_id = command_id_;
                contact_report_.cause = OA_STOP_CAUSE_PLAN_COMPLETE;
                executing_.reset();
                command_id_ = 0U;
                lifecycle_ = OA_LIFECYCLE_ARMED_IDLE;
                if (!publish(OA_EVENT_COMPLETED, OA_CONTROL_OK, completed_id)) {
                    return OA_CONTROL_EBUSY;
                }
            }
        } else {
            if (!settling_published_) {
                if (!publish(OA_EVENT_SETTLING, OA_CONTROL_OK, command_id_)) {
                    return OA_CONTROL_EBUSY;
                }
                settling_published_ = true;
            }
            settle_start_ns_ = 0U;
            settle_feedback_seq_ = {};
            settle_feedback_intervals_ = 0U;
        }
    }
    return OA_CONTROL_OK;
}

bool Controller::measured_at_goal() noexcept {
    if (!executing_) {
        return false;
    }
    for (std::size_t side = 0; side < 2U; ++side) {
        if ((executing_->active_arm_mask & (1U << side)) == 0U) {
            continue;
        }
        const auto q = arm_[side].measured_q();
        const auto dq = arm_[side].measured_dq();
        for (std::size_t joint = 0; joint < 7U; ++joint) {
            if (std::abs(q[joint] - executing_->target_q[side][joint]) >
                    executing_->joint_position_tolerance[side] ||
                std::abs(dq[joint]) > executing_->joint_velocity_tolerance[side]) {
                return false;
            }
        }
        if (executing_->kind == OA_PLAN_PAIRED_TCP) {
            KinematicResult fk{};
            if (!forward(static_cast<std::uint32_t>(side), q, fk)) {
                return false;
            }
            double squared_error = 0.0;
            for (std::size_t axis = 0; axis < 3U; ++axis) {
                const double error = fk.tcp_xyz[axis] - executing_->target_tcp[side][axis];
                squared_error += error * error;
            }
            if (std::sqrt(squared_error) > executing_->tcp_tolerance[side]) {
                return false;
            }
        }
    }
    return true;
}

oa_control_status Controller::set_sim_fault(const oa_sim_fault &fault) noexcept {
    if (options_.backend != OA_BACKEND_VIRTUAL) {
        return OA_CONTROL_EUNSUPPORTED;
    }
    if (fault.side > OA_RIGHT || (fault.freeze_mask & ~kAllJoints) != 0U ||
        (fault.drop_mask & ~kAllJoints) != 0U || (fault.fault_mask & ~kAllJoints) != 0U ||
        (fault.command_fail_mask & ~kAllJoints) != 0U ||
        (fault.fault_mask != 0U && fault.fault_status != 0U &&
         (fault.fault_status < 8U || fault.fault_status > 14U))) {
        return OA_CONTROL_EINVAL;
    }
    const std::uint8_t fault_status = fault.fault_status == 0U
                                          ? 8U
                                          : static_cast<std::uint8_t>(fault.fault_status);
    arm_[fault.side].set_injection(fault.freeze_mask, fault.drop_mask,
                                   fault.fault_mask,
                                   fault_status,
                                   fault.command_fail_mask,
                                   fault.feedback_delay_ns);
    if (fault.fault_mask != 0U) {
        arm_[fault.side].force_state(arm_[fault.side].measured_q(),
                                     arm_[fault.side].measured_dq(), now_ns_);
    }
    return OA_CONTROL_OK;
}

oa_control_status Controller::set_sim_state(const oa_sim_state &state) noexcept {
    if (options_.backend != OA_BACKEND_VIRTUAL) {
        return OA_CONTROL_EUNSUPPORTED;
    }
    if (state.side > OA_RIGHT) {
        return OA_CONTROL_EINVAL;
    }
    JointVector q{};
    JointVector dq{};
    for (std::size_t joint = 0; joint < 7U; ++joint) {
        const auto &motor = manifest_->config().arm[state.side].motor[joint];
        const double raw = (state.q[joint] - motor.q_offset_rad) / motor.q_scale;
        if (!finite(state.q[joint]) || !finite(state.dq[joint]) ||
            state.q[joint] < motor.lower_rad || state.q[joint] > motor.upper_rad ||
            std::abs(raw) > motor.pmax_rad ||
            std::abs(state.dq[joint]) > motor.max_velocity_rad_s) {
            return OA_CONTROL_ELIMIT;
        }
        q[joint] = state.q[joint];
        dq[joint] = state.dq[joint];
    }
    arm_[state.side].force_state(q, dq, now_ns_);
    return OA_CONTROL_OK;
}

oa_control_status Controller::heartbeat(const std::uint64_t command_id,
                                const std::uint64_t producer_deadline_ns) noexcept {
    if (lifecycle_ != OA_LIFECYCLE_EXECUTING || command_id == 0U ||
        command_id != command_id_) {
        return OA_CONTROL_ESTATE;
    }
    if (producer_deadline_ns <= now_ns_) {
        return latch_fault(OA_CONTROL_ESTALE, true);
    }
    producer_deadline_ns_ = producer_deadline_ns;
    return OA_CONTROL_OK;
}

oa_control_status Controller::set_interlock(const bool estop_active,
                                    const bool deadman_active) noexcept {
    deadman_active_ = deadman_active;
    if (estop_active || !deadman_active) {
        const auto failed_id = command_id_;
        executing_.reset();
        command_id_ = 0U;
        if (lifecycle_ == OA_LIFECYCLE_CLOSED ||
            lifecycle_ == OA_LIFECYCLE_VERIFYING) {
            for (auto &runtime : arm_) {
                runtime.set_enabled(false);
                runtime.retire_pending_feedback();
            }
        } else {
            materialize_stop(false);
        }
        lifecycle_ = OA_LIFECYCLE_ESTOP;
        outstanding_nonce_ = ++nonce_counter_;
        return publish(OA_EVENT_ESTOP, OA_CONTROL_EESTOP, failed_id) ? OA_CONTROL_EESTOP : OA_CONTROL_EBUSY;
    }
    return OA_CONTROL_OK;
}

oa_control_status Controller::set_collision_scene_revision(const std::uint64_t revision) noexcept {
    if (options_.backend != OA_BACKEND_VIRTUAL || revision == 0U) {
        return revision == 0U ? OA_CONTROL_EINVAL : OA_CONTROL_EUNSUPPORTED;
    }
    if (lifecycle_ == OA_LIFECYCLE_EXECUTING) {
        return OA_CONTROL_EBUSY;
    }
    options_.collision_scene_revision = revision;
    return OA_CONTROL_OK;
}

oa_control_status Controller::stop(const std::uint32_t stop_kind) noexcept {
    if (stop_kind != OA_STOP_DISABLE && stop_kind != OA_STOP_CONTROLLED) {
        return OA_CONTROL_EINVAL;
    }
    if (lifecycle_ != OA_LIFECYCLE_EXECUTING && lifecycle_ != OA_LIFECYCLE_ARMED_IDLE) {
        return OA_CONTROL_ESTATE;
    }
    lifecycle_ = OA_LIFECYCLE_STOPPING;
    const auto stopped_id = command_id_;
    if (executing_) {
        if (!publish(OA_EVENT_ABORTED, OA_CONTROL_OK, stopped_id)) {
            return OA_CONTROL_EBUSY;
        }
    }
    executing_.reset();
    command_id_ = 0U;
    if (stop_kind == OA_STOP_DISABLE) {
        materialize_stop(false);
        lifecycle_ = OA_LIFECYCLE_DISARMED;
    } else {
        materialize_stop(true);
        lifecycle_ = OA_LIFECYCLE_ARMED_IDLE;
    }
    active_stop_kind_ = OA_STOP_DISABLE;
    return publish(OA_EVENT_STOPPED, OA_CONTROL_OK, stopped_id) ? OA_CONTROL_OK : OA_CONTROL_EBUSY;
}

oa_control_status Controller::disarm(const std::uint64_t deadline_ns) noexcept {
    if (deadline_ns < now_ns_) {
        return OA_CONTROL_ETIMEOUT;
    }
    if (lifecycle_ != OA_LIFECYCLE_ARMED_IDLE && lifecycle_ != OA_LIFECYCLE_EXECUTING &&
        lifecycle_ != OA_LIFECYCLE_DISARMED) {
        return OA_CONTROL_ESTATE;
    }
    executing_.reset();
    command_id_ = 0U;
    materialize_stop(false);
    lifecycle_ = OA_LIFECYCLE_DISARMED;
    return publish(OA_EVENT_DISARMED, OA_CONTROL_OK, 0U) ? OA_CONTROL_OK : OA_CONTROL_EBUSY;
}

oa_control_status Controller::reset(const oa_reset_request &request) noexcept {
    if (lifecycle_ != OA_LIFECYCLE_FAULT && lifecycle_ != OA_LIFECYCLE_ESTOP) {
        return OA_CONTROL_ESTATE;
    }
    if (request.verify_epoch != verify_epoch_ || request.nonce != outstanding_nonce_ ||
        request.nonce == 0U || now_ns_ > outstanding_challenge_expiry_) {
        return OA_CONTROL_EIDENTITY;
    }
    for (auto &runtime : arm_) {
        runtime.set_injection(0U, 0U, 0U, 0U, 0U, 0U);
        runtime.set_enabled(false);
        runtime.retire_pending_feedback();
    }
    ++verify_epoch_;
    outstanding_nonce_ = 0U;
    lifecycle_ = OA_LIFECYCLE_CLOSED;
    return OA_CONTROL_OK;
}

oa_control_status Controller::poll_event(oa_event &out) noexcept {
    if (event_count_ == 0U) {
        return OA_CONTROL_ETIMEOUT;
    }
    out = events_[event_head_];
    event_head_ = (event_head_ + 1U) % events_.size();
    --event_count_;
    return OA_CONTROL_OK;
}

bool Controller::publish(const std::uint32_t kind, const oa_control_status cause,
                         const std::uint64_t command_id) noexcept {
    const bool overflow = event_count_ == events_.size();
    if (overflow) {
        event_head_ = (event_head_ + 1U) % events_.size();
        --event_count_;
        executing_.reset();
        command_id_ = 0U;
        materialize_stop(false);
        active_stop_kind_ = OA_STOP_DISABLE;
        lifecycle_ = OA_LIFECYCLE_FAULT;
    }
    oa_event event{};
    event.struct_size = sizeof(event);
    event.abi_version = OA_CONTROL_ABI_V1;
    event.kind = overflow ? OA_EVENT_FAULTED : kind;
    event.lifecycle = lifecycle_;
    event.t_ns = now_ns_;
    event.command_id = command_id;
    event.feedback_seq = std::min(arm_[0].feedback_sequence(), arm_[1].feedback_sequence());
    event.cause = overflow ? OA_CONTROL_EBUSY : cause;
    const std::size_t tail = (event_head_ + event_count_) % events_.size();
    events_[tail] = event;
    ++event_count_;
    return !overflow;
}

void Controller::materialize_stop(const bool enabled_hold) noexcept {
    for (auto &runtime : arm_) {
        if (options_.backend == OA_BACKEND_VIRTUAL) {
            if (runtime.feedback_sequence() != 0U) {
                runtime.materialize_stop(enabled_hold, now_ns_);
            } else {
                runtime.set_enabled(false);
                runtime.retire_pending_feedback();
            }
        } else {
            runtime.set_enabled(false);
        }
    }
}

oa_control_status Controller::latch_fault(const oa_control_status cause,
                                  const bool controlled_stop_available) noexcept {
    const auto failed_id = command_id_;
    /* Only coherent watchdog paths may retain an enabled hold. Transport or
     * motor-integrity faults always take the disabled fallback. */
    const bool controlled_stop = controlled_stop_available &&
                                 options_.backend == OA_BACKEND_VIRTUAL &&
                                 executing_.has_value() &&
                                 fresh() && healthy() &&
                                 active_stop_kind_ == OA_STOP_CONTROLLED;
    executing_.reset();
    command_id_ = 0U;
    materialize_stop(controlled_stop);
    active_stop_kind_ = OA_STOP_DISABLE;
    lifecycle_ = OA_LIFECYCLE_FAULT;
    outstanding_nonce_ = ++nonce_counter_;
    return publish(OA_EVENT_FAULTED, cause, failed_id) ? cause : OA_CONTROL_EBUSY;
}

oa_control_status Controller::feedback_integrity() const noexcept {
    if (!arm_[0].complete_fresh(now_ns_, options_.feedback_timeout_ns) ||
        !arm_[1].complete_fresh(now_ns_, options_.feedback_timeout_ns)) {
        return OA_CONTROL_ESTALE;
    }
    const std::uint64_t left = arm_[0].generation_timestamp();
    const std::uint64_t right = arm_[1].generation_timestamp();
    const std::uint64_t skew = left > right ? left - right : right - left;
    return skew <= options_.max_cross_bus_skew_ns ? OA_CONTROL_OK : OA_CONTROL_ECAN;
}

bool Controller::fresh() const noexcept {
    return feedback_integrity() == OA_CONTROL_OK;
}

bool Controller::healthy() const noexcept {
    return arm_[0].fault_free(now_ns_, options_.feedback_timeout_ns) &&
           arm_[1].fault_free(now_ns_, options_.feedback_timeout_ns);
}

bool Controller::disabled() const noexcept {
    return arm_[0].all_disabled(now_ns_, options_.feedback_timeout_ns) &&
           arm_[1].all_disabled(now_ns_, options_.feedback_timeout_ns);
}

bool Controller::start_pose_matches(const MotionPlan &plan) const noexcept {
    constexpr double kStartDriftRad = 1.0e-3;
    for (std::size_t side = 0; side < 2U; ++side) {
        const auto q = arm_[side].measured_q();
        for (std::size_t joint = 0; joint < 7U; ++joint) {
            if (std::abs(q[joint] - plan.start_q[side][joint]) > kStartDriftRad) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace openarm::control
