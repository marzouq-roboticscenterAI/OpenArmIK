/* SPDX-License-Identifier: Apache-2.0 */
#include "control_core.hpp"

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

bool DamiaoMotorSimulator::step(const double dt_s, const std::uint64_t feedback_ns,
                                const bool frozen, const bool dropped) noexcept {
    if (dropped) {
        return false;
    }
    if (!frozen) {
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
    const std::uint32_t tau_field = encode_field(0.0, config_.tmax_nm, 4095U);
    const std::uint8_t status = fault_status_ != 0U ? fault_status_ : (enabled_ ? 1U : 0U);
    feedback_frame_.data[0] = static_cast<std::uint8_t>(
        (static_cast<std::uint32_t>(status) << 4U) | (config_.embedded_motor_id & 0x0fU));
    feedback_frame_.data[1] = static_cast<std::uint8_t>((q_field >> 8U) & 0xffU);
    feedback_frame_.data[2] = static_cast<std::uint8_t>(q_field & 0xffU);
    feedback_frame_.data[3] = static_cast<std::uint8_t>((dq_field >> 4U) & 0xffU);
    feedback_frame_.data[4] = static_cast<std::uint8_t>(((dq_field & 0x0fU) << 4U) |
                                                        ((tau_field >> 8U) & 0x0fU));
    feedback_frame_.data[5] = static_cast<std::uint8_t>(tau_field & 0xffU);
    feedback_frame_.data[6] = 25U;
    feedback_frame_.data[7] = 25U;
    feedback_frame_.t_ns = feedback_ns;

    measured_.status = static_cast<std::uint8_t>(feedback_frame_.data[0] >> 4U);
    const std::uint32_t decoded_q =
        (static_cast<std::uint32_t>(feedback_frame_.data[1]) << 8U) |
        feedback_frame_.data[2];
    const std::uint32_t decoded_dq =
        (static_cast<std::uint32_t>(feedback_frame_.data[3]) << 4U) |
        (static_cast<std::uint32_t>(feedback_frame_.data[4]) >> 4U);
    const std::uint32_t decoded_tau =
        ((static_cast<std::uint32_t>(feedback_frame_.data[4]) & 0x0fU) << 8U) |
        feedback_frame_.data[5];
    measured_.raw_q = decode_field(decoded_q, config_.pmax_rad, 65535U);
    measured_.raw_dq = decode_field(decoded_dq, config_.vmax_rad_s, 4095U);
    measured_.raw_tau = decode_field(decoded_tau, config_.tmax_nm, 4095U);
    measured_.mos_c = feedback_frame_.data[6];
    measured_.coil_c = feedback_frame_.data[7];
    measured_.t_ns = feedback_frame_.t_ns;
    measured_.valid = true;
    return true;
}

void DamiaoMotorSimulator::force_state(const double q_model, const double dq_model,
                                       const std::uint64_t feedback_ns) noexcept {
    plant_raw_q_ = (q_model - config_.q_offset_rad) / config_.q_scale;
    plant_raw_dq_ = dq_model / config_.q_scale;
    command_raw_q_ = plant_raw_q_;
    command_raw_dq_ = plant_raw_dq_;
    (void)step(0.0, feedback_ns, true, false);
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
    generation_mask_ = 0U;
    const std::uint64_t feedback_ns =
        feedback_delay_ns_ > now_ns ? 0U : now_ns - feedback_delay_ns_;
    for (std::size_t joint = 0; joint < motor_.size(); ++joint) {
        motor_[joint].command(q_reference[joint], dq_reference[joint]);
        if (motor_[joint].step(dt_s, feedback_ns,
                               (freeze_mask_ & (1U << joint)) != 0U,
                               (drop_mask_ & (1U << joint)) != 0U)) {
            generation_mask_ |= 1U << joint;
        }
    }
    generation_timestamp_ = feedback_ns;
    if (generation_mask_ == kAllJoints) {
        ++feedback_seq_;
    }
    transport_.record_complete_cycle();
    return true;
}

void ArmRuntime::force_state(const JointVector &q, const JointVector &dq,
                             const std::uint64_t now_ns) noexcept {
    generation_mask_ = kAllJoints;
    generation_timestamp_ = now_ns;
    for (std::size_t joint = 0; joint < motor_.size(); ++joint) {
        motor_[joint].force_state(q[joint], dq[joint], now_ns);
    }
    ++feedback_seq_;
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
        options_.max_cross_bus_skew_ns == 0U) {
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

oa_status Controller::open_and_verify(oa_verify_report &out) noexcept {
    if (lifecycle_ != OA_LIFECYCLE_CLOSED) {
        return OA_ESTATE;
    }
    lifecycle_ = OA_LIFECYCLE_VERIFYING;
    if (options_.backend == OA_BACKEND_PHYSICAL) {
        lifecycle_ = OA_LIFECYCLE_CLOSED;
        out.verify_epoch = 0U;
        out.verified_mask = 0U;
        out.failure_mask = 0x3U;
        return OA_EUNSUPPORTED;
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
    publish(OA_EVENT_VERIFIED, OA_OK, 0U);
    return OA_OK;
}

oa_status Controller::snapshot(oa_snapshot &out) noexcept {
    if (lifecycle_ == OA_LIFECYCLE_CLOSED || lifecycle_ == OA_LIFECYCLE_VERIFYING) {
        return OA_ESTATE;
    }
    out.arm[0] = arm_[0].snapshot(now_ns_, options_.feedback_timeout_ns);
    out.arm[1] = arm_[1].snapshot(now_ns_, options_.feedback_timeout_ns);
    out.manifest_revision = manifest_->config().manifest_revision;
    out.model_revision = manifest_->config().model_revision;
    out.max_cross_bus_skew_ns = out.arm[0].t_ns > out.arm[1].t_ns
                                    ? out.arm[0].t_ns - out.arm[1].t_ns
                                    : out.arm[1].t_ns - out.arm[0].t_ns;
    out.lifecycle = lifecycle_;
    return OA_OK;
}

oa_status Controller::kinematics(const std::uint32_t side,
                                 const std::uint64_t required_seq,
                                 oa_arm_kinematics &out) noexcept {
    if (side > OA_RIGHT || !fresh()) {
        return side > OA_RIGHT ? OA_EINVAL : OA_ESTALE;
    }
    const auto &runtime = arm_[side];
    if (runtime.feedback_sequence() != required_seq) {
        return OA_ESTALE;
    }
    KinematicResult result{};
    const auto q = runtime.measured_q();
    if (!forward(side, q, result)) {
        return OA_EFAULT;
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
    return OA_OK;
}

oa_status Controller::challenge(oa_arm_challenge &out) noexcept {
    const bool reset_challenge = lifecycle_ == OA_LIFECYCLE_FAULT ||
                                 lifecycle_ == OA_LIFECYCLE_ESTOP;
    if (lifecycle_ != OA_LIFECYCLE_DISARMED && !reset_challenge) {
        return OA_ESTATE;
    }
    if (!reset_challenge && (!fresh() || !healthy() || !disabled())) {
        if (!healthy()) {
            latch_fault(OA_EFAULT);
            return OA_EFAULT;
        }
        return OA_ESTALE;
    }
    outstanding_nonce_ = ++nonce_counter_;
    outstanding_challenge_expiry_ = now_ns_ + UINT64_C(1000000000);
    out.verify_epoch = verify_epoch_;
    out.nonce = outstanding_nonce_;
    out.expiry_ns = outstanding_challenge_expiry_;
    return OA_OK;
}

oa_status Controller::arm(const oa_arm_challenge &challenge_record) noexcept {
    if (lifecycle_ != OA_LIFECYCLE_DISARMED) {
        return OA_ESTATE;
    }
    if (challenge_record.verify_epoch != verify_epoch_ ||
        challenge_record.nonce != outstanding_nonce_ || outstanding_nonce_ == 0U ||
        challenge_record.expiry_ns != outstanding_challenge_expiry_ ||
        now_ns_ > outstanding_challenge_expiry_) {
        return OA_EIDENTITY;
    }
    if (!fresh()) {
        return OA_ESTALE;
    }
    if (!healthy() || !disabled() || !deadman_active_) {
        latch_fault(!deadman_active_ ? OA_EESTOP : OA_EFAULT);
        return !deadman_active_ ? OA_EESTOP : OA_EFAULT;
    }
    lifecycle_ = OA_LIFECYCLE_ARMING;
    for (auto &runtime : arm_) {
        runtime.set_enabled(true);
        runtime.force_state(runtime.measured_q(), JointVector{}, now_ns_);
    }
    outstanding_nonce_ = 0U;
    lifecycle_ = OA_LIFECYCLE_ARMED_IDLE;
    publish(OA_EVENT_ARMED, OA_OK, 0U);
    return OA_OK;
}

bool Controller::collision_allowed() const noexcept {
    return options_.backend == OA_BACKEND_VIRTUAL &&
           options_.collision_policy == OA_COLLISION_VIRTUAL_UNCHECKED;
}

oa_status Controller::plan_joint(const oa_joint_move &request,
                                 std::unique_ptr<MotionPlan> &out) noexcept {
    if (lifecycle_ != OA_LIFECYCLE_ARMED_IDLE) {
        return OA_ESTATE;
    }
    if (!fresh()) {
        return OA_ESTALE;
    }
    if (!healthy()) {
        latch_fault(OA_EFAULT);
        return OA_EFAULT;
    }
    if (!collision_allowed()) {
        return OA_ECOLLISION;
    }
    if (request.side > OA_RIGHT || request.joint >= 7U ||
        request.required_feedback_seq != arm_[request.side].feedback_sequence() ||
        request.expiry_ns <= now_ns_ || !finite(request.target_rad) ||
        !valid_scale(request.velocity_scale) || !valid_scale(request.acceleration_scale) ||
        !valid_scale(request.jerk_scale) || !finite(request.position_tol_rad) ||
        request.position_tol_rad <= 0.0 || !finite(request.velocity_tol_rad_s) ||
        request.velocity_tol_rad_s <= 0.0) {
        return OA_EINVAL;
    }
    const auto &motor = manifest_->config().arm[request.side].motor[request.joint];
    const double raw_target = (request.target_rad - motor.q_offset_rad) / motor.q_scale;
    if (request.target_rad < motor.lower_rad || request.target_rad > motor.upper_rad ||
        std::abs(raw_target) > motor.pmax_rad) {
        return OA_ELIMIT;
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
    return OA_OK;
}

oa_status Controller::plan_paired(const oa_paired_tcp_move &request,
                                  std::unique_ptr<MotionPlan> &out) noexcept {
    if (lifecycle_ != OA_LIFECYCLE_ARMED_IDLE) {
        return OA_ESTATE;
    }
    if (!fresh()) {
        return OA_ESTALE;
    }
    if (!healthy()) {
        latch_fault(OA_EFAULT);
        return OA_EFAULT;
    }
    if (!collision_allowed()) {
        return OA_ECOLLISION;
    }
    if (request.expiry_ns <= now_ns_ ||
        request.required_feedback_seq[0] != arm_[0].feedback_sequence() ||
        request.required_feedback_seq[1] != arm_[1].feedback_sequence() ||
        !valid_scale(request.velocity_scale) || !valid_scale(request.acceleration_scale) ||
        !valid_scale(request.jerk_scale) || !finite(request.tcp_tol_m) ||
        request.tcp_tol_m <= 0.0 || !finite(request.max_branch_step_rad) ||
        request.max_branch_step_rad <= 0.0 || !finite(request.min_singular_value) ||
        request.min_singular_value < 0.0 ||
        request.collision_scene_revision != options_.collision_scene_revision) {
        return OA_EINVAL;
    }
    auto plan = std::make_unique<MotionPlan>();
    plan->kind = OA_PLAN_PAIRED_TCP;
    plan->active_arm_mask = 0x3U;
    plan->manifest_revision = manifest_->config().manifest_revision;
    plan->model_revision = manifest_->config().model_revision;
    plan->collision_scene_revision = request.collision_scene_revision;
    plan->controller_instance = instance_id_;
    plan->verify_epoch = verify_epoch_;
    plan->expiry_ns = request.expiry_ns;
    plan->waypoint_count = 17U;
    for (std::size_t side = 0; side < 2U; ++side) {
        plan->seed_seq[side] = arm_[side].feedback_sequence();
        plan->start_q[side] = arm_[side].measured_q();
        const auto *target_data = side == 0U ? request.left_tcp_m : request.right_tcp_m;
        std::copy(target_data, target_data + 3, plan->target_tcp[side].begin());
        for (const double value : plan->target_tcp[side]) {
            if (!finite(value)) {
                return OA_EINVAL;
            }
        }
        plan->tcp_tolerance[side] = request.tcp_tol_m;
        plan->joint_position_tolerance[side] = 5.0e-4;
        plan->joint_velocity_tolerance[side] = 2.0e-2;
        KinematicResult start_fk{};
        if (!forward(static_cast<std::uint32_t>(side), plan->start_q[side], start_fk)) {
            return OA_EFAULT;
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
                return OA_EUNREACHABLE;
            }
            for (std::size_t joint = 0; joint < 7U; ++joint) {
                const auto &motor = manifest_->config().arm[side].motor[joint];
                const double raw = (ik.q[joint] - motor.q_offset_rad) / motor.q_scale;
                if (ik.q[joint] < motor.lower_rad || ik.q[joint] > motor.upper_rad ||
                    std::abs(raw) > motor.pmax_rad ||
                    std::abs(ik.q[joint] - predecessor[joint]) >
                        request.max_branch_step_rad) {
                    return OA_EUNREACHABLE;
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
            return OA_EUNREACHABLE;
        }
        plan->waypoint_time_ns[waypoint] =
            plan->waypoint_time_ns[waypoint - 1U] + segment;
    }
    plan->duration_ns = plan->waypoint_time_ns[plan->waypoint_count - 1U];
    out = std::move(plan);
    return OA_OK;
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

oa_status Controller::execute(const MotionPlan &plan, const oa_execute_request &request,
                              std::uint64_t &command_id) noexcept {
    if (lifecycle_ != OA_LIFECYCLE_ARMED_IDLE) {
        return OA_ESTATE;
    }
    if (!fresh()) {
        return OA_ESTALE;
    }
    if (!healthy()) {
        latch_fault(OA_EFAULT);
        return OA_EFAULT;
    }
    if (plan.controller_instance != instance_id_ || plan.verify_epoch != verify_epoch_) {
        return OA_EIDENTITY;
    }
    if (plan.manifest_revision != manifest_->config().manifest_revision ||
        plan.model_revision != manifest_->config().model_revision ||
        plan.collision_scene_revision != options_.collision_scene_revision ||
        plan.seed_seq[0] != arm_[0].feedback_sequence() ||
        plan.seed_seq[1] != arm_[1].feedback_sequence() || !start_pose_matches(plan)) {
        return OA_ESTALE;
    }
    const std::uint64_t start_ns = request.start_ns == 0U ? now_ns_ : request.start_ns;
    if (start_ns < now_ns_ || request.expiry_ns <= start_ns ||
        request.expiry_ns > plan.expiry_ns || request.producer_deadline_ns <= now_ns_ ||
        (request.stop_kind != OA_STOP_DISABLE && request.stop_kind != OA_STOP_CONTROLLED) ||
        plan.duration_ns > request.expiry_ns - start_ns) {
        return OA_EINVAL;
    }
    executing_ = plan;
    command_id_ = next_command_id_++;
    command_start_ns_ = start_ns;
    command_expiry_ns_ = request.expiry_ns;
    producer_deadline_ns_ = request.producer_deadline_ns;
    settle_cycles_ = 0U;
    active_stop_kind_ = request.stop_kind;
    command_started_ = start_ns == now_ns_;
    settling_published_ = false;
    lifecycle_ = OA_LIFECYCLE_EXECUTING;
    command_id = command_id_;
    publish(command_started_ ? OA_EVENT_STARTED : OA_EVENT_QUEUED, OA_OK, command_id_);
    return OA_OK;
}

oa_status Controller::advance(const std::uint64_t monotonic_ns) noexcept {
    if (monotonic_ns < now_ns_) {
        return OA_EINVAL;
    }
    const std::uint64_t previous_ns = now_ns_;
    now_ns_ = monotonic_ns;
    if (lifecycle_ != OA_LIFECYCLE_ARMED_IDLE && lifecycle_ != OA_LIFECYCLE_EXECUTING &&
        lifecycle_ != OA_LIFECYCLE_DISARMED) {
        return lifecycle_ == OA_LIFECYCLE_FAULT ? OA_EFAULT : OA_ESTATE;
    }

    std::array<JointVector, 2> q_reference{arm_[0].measured_q(), arm_[1].measured_q()};
    std::array<JointVector, 2> dq_reference{};
    if (lifecycle_ == OA_LIFECYCLE_EXECUTING) {
        if (now_ns_ > producer_deadline_ns_) {
            latch_fault(OA_ESTALE);
            return OA_ESTALE;
        }
        if (now_ns_ > command_expiry_ns_) {
            latch_fault(OA_ETIMEOUT);
            return OA_ETIMEOUT;
        }
        if (!healthy()) {
            latch_fault(OA_EFAULT);
            return OA_EFAULT;
        }
        if (!command_started_ && now_ns_ >= command_start_ns_) {
            if (!fresh() || !start_pose_matches(*executing_)) {
                latch_fault(OA_ESTALE);
                return OA_ESTALE;
            }
            command_started_ = true;
            publish(OA_EVENT_STARTED, OA_OK, command_id_);
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
    const double dt_s = static_cast<double>(now_ns_ - previous_ns) * 1.0e-9;
    for (std::size_t side = 0; side < 2U; ++side) {
        if (!arm_[side].command_and_step(q_reference[side], dq_reference[side],
                                         now_ns_, dt_s)) {
            latch_fault(OA_ECAN);
            return OA_ECAN;
        }
    }
    if (!arm_[0].complete_fresh(now_ns_, options_.feedback_timeout_ns) ||
        !arm_[1].complete_fresh(now_ns_, options_.feedback_timeout_ns)) {
        latch_fault(OA_ESTALE);
        return OA_ESTALE;
    }
    const std::uint64_t skew = arm_[0].generation_timestamp() > arm_[1].generation_timestamp()
                                   ? arm_[0].generation_timestamp() -
                                         arm_[1].generation_timestamp()
                                   : arm_[1].generation_timestamp() -
                                         arm_[0].generation_timestamp();
    if (skew > options_.max_cross_bus_skew_ns) {
        latch_fault(OA_ECAN);
        return OA_ECAN;
    }
    for (std::size_t side = 0; side < 2U; ++side) {
        if (arm_[side].snapshot(now_ns_, options_.feedback_timeout_ns).fault_mask != 0U) {
            latch_fault(OA_EFAULT);
            return OA_EFAULT;
        }
    }
    if (lifecycle_ == OA_LIFECYCLE_EXECUTING &&
        now_ns_ >= command_start_ns_ + executing_->duration_ns) {
        if (measured_at_goal()) {
            if (!settling_published_) {
                publish(OA_EVENT_SETTLING, OA_OK, command_id_);
                settling_published_ = true;
            }
            ++settle_cycles_;
            if (settle_cycles_ >= 3U) {
                const auto completed_id = command_id_;
                executing_.reset();
                command_id_ = 0U;
                lifecycle_ = OA_LIFECYCLE_ARMED_IDLE;
                publish(OA_EVENT_COMPLETED, OA_OK, completed_id);
            }
        } else {
            if (!settling_published_) {
                publish(OA_EVENT_SETTLING, OA_OK, command_id_);
                settling_published_ = true;
            }
            settle_cycles_ = 0U;
        }
    }
    return OA_OK;
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

oa_status Controller::set_sim_fault(const oa_sim_fault &fault) noexcept {
    if (options_.backend != OA_BACKEND_VIRTUAL) {
        return OA_EUNSUPPORTED;
    }
    if (fault.side > OA_RIGHT || (fault.freeze_mask & ~kAllJoints) != 0U ||
        (fault.drop_mask & ~kAllJoints) != 0U || (fault.fault_mask & ~kAllJoints) != 0U ||
        (fault.command_fail_mask & ~kAllJoints) != 0U ||
        (fault.fault_mask != 0U && fault.fault_status != 0U &&
         (fault.fault_status < 8U || fault.fault_status > 14U))) {
        return OA_EINVAL;
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
    return OA_OK;
}

oa_status Controller::set_sim_state(const oa_sim_state &state) noexcept {
    if (options_.backend != OA_BACKEND_VIRTUAL) {
        return OA_EUNSUPPORTED;
    }
    if (state.side > OA_RIGHT) {
        return OA_EINVAL;
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
            return OA_ELIMIT;
        }
        q[joint] = state.q[joint];
        dq[joint] = state.dq[joint];
    }
    arm_[state.side].force_state(q, dq, now_ns_);
    return OA_OK;
}

oa_status Controller::heartbeat(const std::uint64_t command_id,
                                const std::uint64_t producer_deadline_ns) noexcept {
    if (lifecycle_ != OA_LIFECYCLE_EXECUTING || command_id == 0U ||
        command_id != command_id_) {
        return OA_ESTATE;
    }
    if (producer_deadline_ns <= now_ns_) {
        latch_fault(OA_ESTALE);
        return OA_ESTALE;
    }
    producer_deadline_ns_ = producer_deadline_ns;
    return OA_OK;
}

oa_status Controller::set_interlock(const bool estop_active,
                                    const bool deadman_active) noexcept {
    deadman_active_ = deadman_active;
    if (estop_active || !deadman_active) {
        const auto failed_id = command_id_;
        executing_.reset();
        command_id_ = 0U;
        for (auto &runtime : arm_) {
            runtime.set_enabled(false);
        }
        lifecycle_ = OA_LIFECYCLE_ESTOP;
        outstanding_nonce_ = ++nonce_counter_;
        publish(OA_EVENT_ESTOP, OA_EESTOP, failed_id);
        return OA_EESTOP;
    }
    return OA_OK;
}

oa_status Controller::set_collision_scene_revision(const std::uint64_t revision) noexcept {
    if (options_.backend != OA_BACKEND_VIRTUAL || revision == 0U) {
        return revision == 0U ? OA_EINVAL : OA_EUNSUPPORTED;
    }
    if (lifecycle_ == OA_LIFECYCLE_EXECUTING) {
        return OA_EBUSY;
    }
    options_.collision_scene_revision = revision;
    return OA_OK;
}

oa_status Controller::stop(const std::uint32_t stop_kind) noexcept {
    if (stop_kind != OA_STOP_DISABLE && stop_kind != OA_STOP_CONTROLLED) {
        return OA_EINVAL;
    }
    if (lifecycle_ != OA_LIFECYCLE_EXECUTING && lifecycle_ != OA_LIFECYCLE_ARMED_IDLE) {
        return OA_ESTATE;
    }
    lifecycle_ = OA_LIFECYCLE_STOPPING;
    const auto stopped_id = command_id_;
    if (executing_) {
        publish(OA_EVENT_ABORTED, OA_OK, stopped_id);
    }
    executing_.reset();
    command_id_ = 0U;
    if (stop_kind == OA_STOP_DISABLE) {
        for (auto &runtime : arm_) {
            runtime.set_enabled(false);
        }
        lifecycle_ = OA_LIFECYCLE_DISARMED;
    } else {
        lifecycle_ = OA_LIFECYCLE_ARMED_IDLE;
    }
    publish(OA_EVENT_STOPPED, OA_OK, stopped_id);
    return OA_OK;
}

oa_status Controller::disarm(const std::uint64_t deadline_ns) noexcept {
    if (deadline_ns < now_ns_) {
        return OA_ETIMEOUT;
    }
    if (lifecycle_ != OA_LIFECYCLE_ARMED_IDLE && lifecycle_ != OA_LIFECYCLE_EXECUTING &&
        lifecycle_ != OA_LIFECYCLE_DISARMED) {
        return OA_ESTATE;
    }
    executing_.reset();
    command_id_ = 0U;
    for (auto &runtime : arm_) {
        runtime.set_enabled(false);
        runtime.force_state(runtime.measured_q(), JointVector{}, now_ns_);
    }
    lifecycle_ = OA_LIFECYCLE_DISARMED;
    publish(OA_EVENT_DISARMED, OA_OK, 0U);
    return OA_OK;
}

oa_status Controller::reset(const oa_reset_request &request) noexcept {
    if (lifecycle_ != OA_LIFECYCLE_FAULT && lifecycle_ != OA_LIFECYCLE_ESTOP) {
        return OA_ESTATE;
    }
    if (request.verify_epoch != verify_epoch_ || request.nonce != outstanding_nonce_ ||
        request.nonce == 0U || now_ns_ > outstanding_challenge_expiry_) {
        return OA_EIDENTITY;
    }
    for (auto &runtime : arm_) {
        runtime.set_injection(0U, 0U, 0U, 0U, 0U, 0U);
        runtime.set_enabled(false);
    }
    ++verify_epoch_;
    outstanding_nonce_ = 0U;
    lifecycle_ = OA_LIFECYCLE_CLOSED;
    return OA_OK;
}

oa_status Controller::poll_event(oa_event &out) noexcept {
    if (event_count_ == 0U) {
        return OA_ETIMEOUT;
    }
    out = events_[event_head_];
    event_head_ = (event_head_ + 1U) % events_.size();
    --event_count_;
    return OA_OK;
}

void Controller::publish(const std::uint32_t kind, const oa_status cause,
                         const std::uint64_t command_id) noexcept {
    const bool overflow = event_count_ == events_.size();
    if (overflow) {
        event_head_ = (event_head_ + 1U) % events_.size();
        --event_count_;
        executing_.reset();
        command_id_ = 0U;
        for (auto &runtime : arm_) {
            runtime.set_enabled(false);
        }
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
    event.cause = overflow ? OA_EBUSY : cause;
    const std::size_t tail = (event_head_ + event_count_) % events_.size();
    events_[tail] = event;
    ++event_count_;
}

void Controller::latch_fault(const oa_status cause) noexcept {
    const auto failed_id = command_id_;
    executing_.reset();
    command_id_ = 0U;
    for (auto &runtime : arm_) {
        runtime.set_enabled(false);
    }
    lifecycle_ = OA_LIFECYCLE_FAULT;
    outstanding_nonce_ = ++nonce_counter_;
    publish(OA_EVENT_FAULTED, cause, failed_id);
}

bool Controller::fresh() const noexcept {
    if (!arm_[0].complete_fresh(now_ns_, options_.feedback_timeout_ns) ||
        !arm_[1].complete_fresh(now_ns_, options_.feedback_timeout_ns)) {
        return false;
    }
    const std::uint64_t left = arm_[0].generation_timestamp();
    const std::uint64_t right = arm_[1].generation_timestamp();
    const std::uint64_t skew = left > right ? left - right : right - left;
    return skew <= options_.max_cross_bus_skew_ns;
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
