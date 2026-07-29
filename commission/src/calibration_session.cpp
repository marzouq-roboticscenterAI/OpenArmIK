#include "calibration_session.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace openarm::commission {
namespace {

constexpr double kDirectionTolerance = 1.0e-6;

bool finite_sample(const oa_commission_encoder_sample &sample) noexcept {
    return std::isfinite(sample.q_output_rad) &&
           std::isfinite(sample.dq_output_rad_s) &&
           std::isfinite(sample.torque_output_nm) &&
           std::isfinite(sample.mos_temperature_c) &&
           std::isfinite(sample.coil_temperature_c);
}

std::uint64_t saturating_add(std::uint64_t left, std::uint64_t right) noexcept {
    if (right > std::numeric_limits<std::uint64_t>::max() - left) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    return left + right;
}

void copy_text(char (&destination)[OA_COMMISSION_TEXT_CAPACITY], const char *source) noexcept {
    std::memset(destination, 0, sizeof(destination));
    const auto length = std::min<std::size_t>(
        std::strlen(source),
        OA_COMMISSION_TEXT_CAPACITY - 1U);
    std::memcpy(destination, source, length);
}

}  // namespace

bool valid_header(std::uint32_t struct_size,
                  std::uint32_t abi_version,
                  std::uint32_t required_size) noexcept {
    return abi_version == OA_COMMISSION_ABI_V1 && struct_size >= required_size;
}

bool valid_text(const char *text, bool allow_empty) noexcept {
    if (text == nullptr) {
        return false;
    }
    const void *terminator = std::memchr(text, '\0', OA_COMMISSION_TEXT_CAPACITY);
    if (terminator == nullptr) {
        return false;
    }
    return allow_empty || text[0] != '\0';
}

void SampleAccumulator::reset(const oa_commission_encoder_sample &sample) noexcept {
    count = 1;
    first_time_ns = sample.sample_time_ns;
    sum = sample.q_output_rad;
    minimum = sample.q_output_rad;
    maximum = sample.q_output_rad;
}

void SampleAccumulator::add(const oa_commission_encoder_sample &sample) noexcept {
    if (count == 0U) {
        reset(sample);
        return;
    }
    ++count;
    sum += sample.q_output_rad;
    minimum = std::min(minimum, sample.q_output_rad);
    maximum = std::max(maximum, sample.q_output_rad);
}

double SampleAccumulator::mean() const noexcept {
    return count == 0U ? 0.0 : sum / static_cast<double>(count);
}

double SampleAccumulator::spread() const noexcept {
    return count == 0U ? 0.0 : maximum - minimum;
}

ManualCalibrationSession::ManualCalibrationSession(
    const oa_commission_manual_options &options)
    : options_(options) {}

oa_commission_status ManualCalibrationSession::validate_sample(
    std::uint64_t now_ns,
    const oa_commission_encoder_sample &sample) const noexcept {
    if (!valid_header(sample.struct_size, sample.abi_version, sizeof(sample)) ||
        !finite_sample(sample)) {
        return OA_COMMISSION_EINVAL;
    }
    if (sample.feedback_seq == 0U || sample.feedback_seq <= last_feedback_seq_ ||
        sample.sample_time_ns <= last_sample_time_ns_ || sample.sample_time_ns > now_ns ||
        now_ns - sample.sample_time_ns > options_.maximum_sample_age_ns) {
        return OA_COMMISSION_ESTALE;
    }
    if (sample.drive_enabled != 0U || sample.drive_fault != 0U) {
        return OA_COMMISSION_EINTERLOCK;
    }
    return OA_COMMISSION_OK;
}

oa_commission_status ManualCalibrationSession::sample(
    std::uint32_t reference_index,
    std::uint64_t now_ns,
    const oa_commission_encoder_sample &sample_value) noexcept {
    const std::uint32_t expected_reference =
        state_ == OA_MANUAL_COLLECT_REFERENCE_1 ? 0U : 1U;
    if ((state_ != OA_MANUAL_COLLECT_REFERENCE_1 &&
         state_ != OA_MANUAL_COLLECT_REFERENCE_2) ||
        reference_index != expected_reference) {
        return OA_COMMISSION_ESTATE;
    }
    const auto validation = validate_sample(now_ns, sample_value);
    if (validation != OA_COMMISSION_OK) {
        return validation;
    }
    last_feedback_seq_ = sample_value.feedback_seq;
    last_sample_time_ns_ = sample_value.sample_time_ns;

    auto &accumulator = accumulators_[reference_index];
    if (std::abs(sample_value.dq_output_rad_s) > options_.maximum_abs_velocity_rad_s) {
        accumulator = {};
        return OA_COMMISSION_EUNSTABLE;
    }
    if (accumulator.count == 0U) {
        accumulator.reset(sample_value);
    } else {
        accumulator.add(sample_value);
        if (accumulator.spread() > options_.maximum_position_spread_rad) {
            accumulator.reset(sample_value);
            return OA_COMMISSION_EUNSTABLE;
        }
    }
    const bool enough_samples = accumulator.count >= options_.minimum_samples;
    const bool enough_dwell = sample_value.sample_time_ns - accumulator.first_time_ns >=
                              options_.stability_dwell_ns;
    if (!enough_samples || !enough_dwell) {
        return OA_COMMISSION_OK;
    }
    return finish_reference(reference_index);
}

oa_commission_status ManualCalibrationSession::finish_reference(
    std::uint32_t reference_index) noexcept {
    means_[reference_index] = accumulators_[reference_index].mean();
    spreads_[reference_index] = accumulators_[reference_index].spread();
    if (reference_index == 0U && options_.reference_count == 2U) {
        state_ = OA_MANUAL_COLLECT_REFERENCE_2;
        return OA_COMMISSION_OK;
    }
    return calculate_candidate();
}

oa_commission_status ManualCalibrationSession::calculate_candidate() noexcept {
    if (options_.reference_count == 1U) {
        candidate_a_ = static_cast<double>(options_.known_sign);
        candidate_b_ = options_.reference_model_rad[0] - candidate_a_ * means_[0];
        state_ = OA_MANUAL_CANDIDATE;
        return OA_COMMISSION_OK;
    }

    const double output_delta = means_[1] - means_[0];
    const double model_delta =
        options_.reference_model_rad[1] - options_.reference_model_rad[0];
    if (std::abs(output_delta) < options_.minimum_reference_separation_rad ||
        std::abs(model_delta) < options_.minimum_reference_separation_rad) {
        return OA_COMMISSION_ELIMIT;
    }
    const double observed_scale = model_delta / output_delta;
    const double sign = observed_scale >= 0.0 ? 1.0 : -1.0;
    if (std::abs(std::abs(observed_scale) - 1.0) > options_.maximum_scale_error ||
        (options_.known_sign != 0 && sign != static_cast<double>(options_.known_sign))) {
        return OA_COMMISSION_ELIMIT;
    }
    candidate_a_ = sign;
    const double first_offset =
        options_.reference_model_rad[0] - candidate_a_ * means_[0];
    const double second_offset =
        options_.reference_model_rad[1] - candidate_a_ * means_[1];
    candidate_b_ = 0.5 * (first_offset + second_offset);
    state_ = OA_MANUAL_CANDIDATE;
    return OA_COMMISSION_OK;
}

oa_commission_status ManualCalibrationSession::begin_review() noexcept {
    if (state_ != OA_MANUAL_CANDIDATE) {
        return OA_COMMISSION_ESTATE;
    }
    state_ = OA_MANUAL_REVIEW;
    return OA_COMMISSION_OK;
}

oa_commission_status ManualCalibrationSession::commit(
    std::uint64_t replacement_revision,
    const char *evidence_record,
    oa_commission_mapping_patch &patch) noexcept {
    if (state_ != OA_MANUAL_REVIEW) {
        return OA_COMMISSION_ESTATE;
    }
    if (replacement_revision <= options_.expected_revision ||
        !valid_text(evidence_record)) {
        return OA_COMMISSION_EINVAL;
    }
    oa_commission_mapping_patch candidate{};
    candidate.struct_size = sizeof(candidate);
    candidate.abi_version = OA_COMMISSION_ABI_V1;
    candidate.expected_revision = options_.expected_revision;
    candidate.replacement_revision = replacement_revision;
    candidate.side = options_.side;
    candidate.joint = options_.joint;
    candidate.a = candidate_a_;
    candidate.b_rad = candidate_b_;
    copy_text(candidate.motor_serial, options_.motor_serial);
    copy_text(candidate.evidence_record, evidence_record);
    patch = candidate;
    state_ = OA_MANUAL_COMMITTED;
    return OA_COMMISSION_OK;
}

oa_commission_status ManualCalibrationSession::abort() noexcept {
    if (state_ == OA_MANUAL_COMMITTED || state_ == OA_MANUAL_ABORTED) {
        return OA_COMMISSION_ESTATE;
    }
    state_ = OA_MANUAL_ABORTED;
    candidate_a_ = 0.0;
    candidate_b_ = 0.0;
    return OA_COMMISSION_OK;
}

oa_commission_manual_report ManualCalibrationSession::report() const noexcept {
    oa_commission_manual_report value{};
    value.struct_size = sizeof(value);
    value.abi_version = OA_COMMISSION_ABI_V1;
    value.state = state_;
    value.active_reference =
        state_ == OA_MANUAL_COLLECT_REFERENCE_2 ? 1U : 0U;
    for (std::size_t index = 0; index < accumulators_.size(); ++index) {
        value.accepted_samples[index] = accumulators_[index].count;
        value.mean_output_rad[index] = means_[index];
        value.spread_output_rad[index] = spreads_[index];
    }
    value.candidate_a = candidate_a_;
    value.candidate_b_rad = candidate_b_;
    return value;
}

RecipeCalibrationSession::RecipeCalibrationSession(const oa_commission_recipe &recipe)
    : recipe_(recipe) {}

oa_commission_status RecipeCalibrationSession::fail(oa_commission_status status,
                                                    std::uint32_t reason) noexcept {
    state_ = OA_RECIPE_ABORT;
    abort_reason_ = reason;
    candidate_a_ = 0.0;
    candidate_b_ = 0.0;
    return status;
}

oa_commission_status RecipeCalibrationSession::validate_input(
    const oa_commission_recipe_input &input) noexcept {
    if (!valid_header(input.struct_size, input.abi_version, sizeof(input)) ||
        !valid_header(input.encoder.struct_size,
                      input.encoder.abi_version,
                      sizeof(input.encoder)) ||
        !finite_sample(input.encoder)) {
        return fail(OA_COMMISSION_EINVAL, OA_ABORT_BAD_SAMPLE);
    }
    if (input.encoder.feedback_seq == 0U ||
        input.encoder.feedback_seq <= last_feedback_seq_ ||
        input.encoder.sample_time_ns <= last_sample_time_ns_ ||
        input.encoder.sample_time_ns > input.now_ns ||
        input.now_ns - input.encoder.sample_time_ns > recipe_.maximum_sample_age_ns) {
        return fail(OA_COMMISSION_ESTALE, OA_ABORT_STALE);
    }
    last_feedback_seq_ = input.encoder.feedback_seq;
    last_sample_time_ns_ = input.encoder.sample_time_ns;
    if (input.encoder.drive_fault != 0U) {
        return fail(OA_COMMISSION_EFAULT, OA_ABORT_INTERLOCK);
    }
    return OA_COMMISSION_OK;
}

oa_commission_status RecipeCalibrationSession::update_energy(
    const oa_commission_recipe_input &input) noexcept {
    if (last_energy_time_ns_ == 0U) {
        last_energy_time_ns_ = input.encoder.sample_time_ns;
        return OA_COMMISSION_OK;
    }
    const double seconds = static_cast<double>(
        input.encoder.sample_time_ns - last_energy_time_ns_) * 1.0e-9;
    last_energy_time_ns_ = input.encoder.sample_time_ns;
    contact_energy_j_ += std::abs(input.encoder.torque_output_nm *
                                  input.encoder.dq_output_rad_s) * seconds;
    if (!std::isfinite(contact_energy_j_) ||
        contact_energy_j_ > recipe_.maximum_contact_energy_j) {
        return fail(OA_COMMISSION_ELIMIT, OA_ABORT_LIMIT);
    }
    return OA_COMMISSION_OK;
}

oa_commission_status RecipeCalibrationSession::enforce_motion_limits(
    const oa_commission_recipe_input &input) noexcept {
    if (input.estop_clear == 0U || input.deadman_held == 0U) {
        return fail(OA_COMMISSION_EINTERLOCK, OA_ABORT_INTERLOCK);
    }
    if (std::abs(input.encoder.dq_output_rad_s) > recipe_.maximum_speed_rad_s ||
        std::abs(input.encoder.torque_output_nm) > recipe_.maximum_torque_nm ||
        input.encoder.mos_temperature_c > recipe_.maximum_temperature_c ||
        input.encoder.coil_temperature_c > recipe_.maximum_temperature_c) {
        return fail(OA_COMMISSION_ELIMIT, OA_ABORT_LIMIT);
    }
    return update_energy(input);
}

double RecipeCalibrationSession::directional_travel(double from,
                                                    double to,
                                                    double direction) const noexcept {
    return direction * (to - from);
}

bool RecipeCalibrationSession::contact_evidence(
    const oa_commission_recipe_input &input,
    double travel) const noexcept {
    return input.encoder.drive_enabled != 0U &&
           travel >= recipe_.minimum_contact_travel_rad &&
           std::abs(input.encoder.dq_output_rad_s) <= recipe_.contact_velocity_rad_s &&
           std::abs(input.encoder.torque_output_nm) >= recipe_.minimum_contact_torque_nm;
}

oa_commission_next_action RecipeCalibrationSession::action(
    std::uint32_t kind,
    const oa_commission_recipe_input &input) const noexcept {
    oa_commission_next_action value{};
    value.struct_size = sizeof(value);
    value.abi_version = OA_COMMISSION_ABI_V1;
    value.kind = kind;
    value.state = state_;
    value.maximum_speed_rad_s = recipe_.maximum_speed_rad_s;
    value.maximum_torque_nm = recipe_.maximum_torque_nm;
    value.maximum_temperature_c = recipe_.maximum_temperature_c;
    switch (kind) {
        case OA_RECIPE_ACTION_APPROACH:
        case OA_RECIPE_ACTION_CONTACT_DWELL:
            value.valid_until_ns =
                saturating_add(phase_start_ns_, recipe_.maximum_approach_time_ns);
            value.direction = recipe_.approach_direction;
            value.maximum_travel_rad = recipe_.maximum_approach_travel_rad;
            value.target_output_rad =
                phase_start_q_ + recipe_.approach_direction *
                                     recipe_.maximum_approach_travel_rad;
            break;
        case OA_RECIPE_ACTION_RETREAT:
            value.valid_until_ns =
                saturating_add(phase_start_ns_, recipe_.maximum_retreat_time_ns);
            value.direction = -recipe_.approach_direction;
            value.maximum_travel_rad = recipe_.retreat_distance_rad;
            value.target_output_rad =
                first_stop_q_ - recipe_.approach_direction * recipe_.retreat_distance_rad;
            break;
        case OA_RECIPE_ACTION_REAPPROACH:
            value.valid_until_ns =
                saturating_add(phase_start_ns_, recipe_.maximum_approach_time_ns);
            value.direction = recipe_.approach_direction;
            value.maximum_travel_rad = recipe_.maximum_approach_travel_rad;
            value.target_output_rad =
                phase_start_q_ + recipe_.approach_direction *
                                     recipe_.maximum_approach_travel_rad;
            break;
        default:
            value.valid_until_ns = input.now_ns;
            break;
    }
    return value;
}

oa_commission_status RecipeCalibrationSession::step(
    const oa_commission_recipe_input &input,
    oa_commission_next_action &next_action) noexcept {
    if (state_ == OA_RECIPE_ABORT) {
        next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
        return OA_COMMISSION_ESTATE;
    }
    if (state_ == OA_RECIPE_COMMIT) {
        next_action = action(OA_RECIPE_ACTION_NONE, input);
        return OA_COMMISSION_ESTATE;
    }
    const auto input_status = validate_input(input);
    if (input_status != OA_COMMISSION_OK) {
        next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
        return input_status;
    }

    switch (state_) {
        case OA_RECIPE_PRECHECK:
            if (input.estop_clear == 0U || input.encoder.drive_enabled != 0U ||
                input.encoder.q_output_rad < recipe_.start_min_output_rad ||
                input.encoder.q_output_rad > recipe_.start_max_output_rad ||
                input.encoder.mos_temperature_c > recipe_.maximum_temperature_c ||
                input.encoder.coil_temperature_c > recipe_.maximum_temperature_c) {
                const auto result = fail(OA_COMMISSION_EINTERLOCK, OA_ABORT_INTERLOCK);
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return result;
            }
            state_ = OA_RECIPE_WAIT;
            next_action = action(OA_RECIPE_ACTION_HOLD_DISABLED, input);
            return OA_COMMISSION_OK;

        case OA_RECIPE_WAIT:
            if (input.estop_clear == 0U || input.encoder.drive_enabled != 0U) {
                const auto result = fail(OA_COMMISSION_EINTERLOCK, OA_ABORT_INTERLOCK);
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return result;
            }
            if (input.deadman_held == 0U || input.operator_ready == 0U) {
                next_action = action(OA_RECIPE_ACTION_HOLD_DISABLED, input);
                return OA_COMMISSION_OK;
            }
            state_ = OA_RECIPE_APPROACH;
            phase_start_ns_ = input.now_ns;
            phase_start_q_ = input.encoder.q_output_rad;
            last_energy_time_ns_ = input.encoder.sample_time_ns;
            next_action = action(OA_RECIPE_ACTION_APPROACH, input);
            return OA_COMMISSION_OK;

        case OA_RECIPE_APPROACH: {
            const auto limits = enforce_motion_limits(input);
            if (limits != OA_COMMISSION_OK) {
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return limits;
            }
            const double travel = directional_travel(
                phase_start_q_, input.encoder.q_output_rad, recipe_.approach_direction);
            if (travel < -kDirectionTolerance ||
                travel > recipe_.maximum_approach_travel_rad) {
                const auto result = fail(OA_COMMISSION_ELIMIT, OA_ABORT_LIMIT);
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return result;
            }
            if (input.now_ns - phase_start_ns_ > recipe_.maximum_approach_time_ns) {
                const auto result = fail(OA_COMMISSION_ELIMIT, OA_ABORT_TIMEOUT);
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return result;
            }
            if (contact_evidence(input, travel)) {
                state_ = OA_RECIPE_CONTACT_DWELL;
                dwell_start_ns_ = input.encoder.sample_time_ns;
                contact_samples_ = 1U;
                contact_sum_q_ = input.encoder.q_output_rad;
                next_action = action(OA_RECIPE_ACTION_CONTACT_DWELL, input);
                return OA_COMMISSION_OK;
            }
            next_action = action(OA_RECIPE_ACTION_APPROACH, input);
            return OA_COMMISSION_OK;
        }

        case OA_RECIPE_CONTACT_DWELL: {
            const auto limits = enforce_motion_limits(input);
            if (limits != OA_COMMISSION_OK) {
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return limits;
            }
            const double travel = directional_travel(
                phase_start_q_, input.encoder.q_output_rad, recipe_.approach_direction);
            if (travel > recipe_.maximum_approach_travel_rad ||
                !contact_evidence(input, travel)) {
                const auto result = fail(OA_COMMISSION_EFAULT, OA_ABORT_CONTACT);
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return result;
            }
            ++contact_samples_;
            contact_sum_q_ += input.encoder.q_output_rad;
            if (input.encoder.sample_time_ns - dwell_start_ns_ < recipe_.contact_dwell_ns ||
                contact_samples_ < recipe_.minimum_contact_samples) {
                next_action = action(OA_RECIPE_ACTION_CONTACT_DWELL, input);
                return OA_COMMISSION_OK;
            }
            first_stop_q_ = contact_sum_q_ / static_cast<double>(contact_samples_);
            state_ = OA_RECIPE_RETREAT;
            phase_start_ns_ = input.now_ns;
            phase_start_q_ = input.encoder.q_output_rad;
            next_action = action(OA_RECIPE_ACTION_RETREAT, input);
            return OA_COMMISSION_OK;
        }

        case OA_RECIPE_RETREAT: {
            const auto limits = enforce_motion_limits(input);
            if (limits != OA_COMMISSION_OK) {
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return limits;
            }
            const double travel = directional_travel(
                phase_start_q_, input.encoder.q_output_rad, -recipe_.approach_direction);
            if (travel < -kDirectionTolerance ||
                travel > recipe_.retreat_distance_rad + recipe_.repeatability_tolerance_rad) {
                const auto result = fail(OA_COMMISSION_ELIMIT, OA_ABORT_LIMIT);
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return result;
            }
            if (input.now_ns - phase_start_ns_ > recipe_.maximum_retreat_time_ns) {
                const auto result = fail(OA_COMMISSION_ELIMIT, OA_ABORT_TIMEOUT);
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return result;
            }
            if (travel >= recipe_.retreat_distance_rad &&
                std::abs(input.encoder.dq_output_rad_s) <= recipe_.contact_velocity_rad_s) {
                state_ = OA_RECIPE_REAPPROACH;
                phase_start_ns_ = input.now_ns;
                phase_start_q_ = input.encoder.q_output_rad;
                last_energy_time_ns_ = input.encoder.sample_time_ns;
                next_action = action(OA_RECIPE_ACTION_REAPPROACH, input);
                return OA_COMMISSION_OK;
            }
            next_action = action(OA_RECIPE_ACTION_RETREAT, input);
            return OA_COMMISSION_OK;
        }

        case OA_RECIPE_REAPPROACH: {
            const auto limits = enforce_motion_limits(input);
            if (limits != OA_COMMISSION_OK) {
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return limits;
            }
            const double travel = directional_travel(
                phase_start_q_, input.encoder.q_output_rad, recipe_.approach_direction);
            if (travel < -kDirectionTolerance ||
                travel > recipe_.maximum_approach_travel_rad) {
                const auto result = fail(OA_COMMISSION_ELIMIT, OA_ABORT_LIMIT);
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return result;
            }
            if (input.now_ns - phase_start_ns_ > recipe_.maximum_approach_time_ns) {
                const auto result = fail(OA_COMMISSION_ELIMIT, OA_ABORT_TIMEOUT);
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return result;
            }
            if (contact_evidence(input, travel)) {
                second_stop_q_ = input.encoder.q_output_rad;
                state_ = OA_RECIPE_REPEATABILITY;
                next_action = action(OA_RECIPE_ACTION_HOLD_DISABLED, input);
                return OA_COMMISSION_OK;
            }
            next_action = action(OA_RECIPE_ACTION_REAPPROACH, input);
            return OA_COMMISSION_OK;
        }

        case OA_RECIPE_REPEATABILITY:
            if (input.estop_clear == 0U || input.deadman_held == 0U ||
                input.encoder.drive_enabled != 0U) {
                const auto result = fail(OA_COMMISSION_EINTERLOCK, OA_ABORT_INTERLOCK);
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return result;
            }
            if (std::abs(second_stop_q_ - first_stop_q_) >
                recipe_.repeatability_tolerance_rad) {
                const auto result =
                    fail(OA_COMMISSION_EREPEATABILITY, OA_ABORT_REPEATABILITY);
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return result;
            }
            candidate_a_ = static_cast<double>(recipe_.known_sign);
            candidate_b_ = recipe_.stop_model_rad -
                           candidate_a_ * 0.5 * (first_stop_q_ + second_stop_q_);
            state_ = OA_RECIPE_CANDIDATE;
            next_action = action(OA_RECIPE_ACTION_REVIEW, input);
            return OA_COMMISSION_OK;

        case OA_RECIPE_CANDIDATE:
            if (input.estop_clear == 0U || input.encoder.drive_enabled != 0U) {
                const auto result = fail(OA_COMMISSION_EINTERLOCK, OA_ABORT_INTERLOCK);
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return result;
            }
            if (input.review_decision == OA_REVIEW_REJECT) {
                const auto result = fail(OA_COMMISSION_ESTATE, OA_ABORT_REVIEW_REJECTED);
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return result;
            }
            if (input.review_decision == OA_REVIEW_ACCEPT) {
                state_ = OA_RECIPE_REVIEW;
                next_action = action(OA_RECIPE_ACTION_COMMIT_READY, input);
                return OA_COMMISSION_OK;
            }
            next_action = action(OA_RECIPE_ACTION_REVIEW, input);
            return OA_COMMISSION_OK;

        case OA_RECIPE_REVIEW:
            if (input.estop_clear == 0U || input.encoder.drive_enabled != 0U ||
                input.review_decision == OA_REVIEW_REJECT) {
                const auto result = fail(OA_COMMISSION_ESTATE, OA_ABORT_REVIEW_REJECTED);
                next_action = action(OA_RECIPE_ACTION_ABORT_DISABLE, input);
                return result;
            }
            next_action = action(OA_RECIPE_ACTION_COMMIT_READY, input);
            return OA_COMMISSION_OK;

        default:
            return fail(OA_COMMISSION_ESTATE, OA_ABORT_BAD_SAMPLE);
    }
}

oa_commission_status RecipeCalibrationSession::commit(
    std::uint64_t replacement_revision,
    oa_commission_mapping_patch &patch) noexcept {
    if (state_ != OA_RECIPE_REVIEW) {
        return OA_COMMISSION_ESTATE;
    }
    if (replacement_revision <= recipe_.expected_revision) {
        return OA_COMMISSION_EINVAL;
    }
    oa_commission_mapping_patch candidate{};
    candidate.struct_size = sizeof(candidate);
    candidate.abi_version = OA_COMMISSION_ABI_V1;
    candidate.expected_revision = recipe_.expected_revision;
    candidate.replacement_revision = replacement_revision;
    candidate.side = recipe_.side;
    candidate.joint = recipe_.joint;
    candidate.a = candidate_a_;
    candidate.b_rad = candidate_b_;
    copy_text(candidate.motor_serial, recipe_.motor_serial);
    const char *evidence = recipe_.simulation_only != 0U
                               ? recipe_.fixture_record
                               : recipe_.qualification_record;
    copy_text(candidate.evidence_record, evidence);
    patch = candidate;
    state_ = OA_RECIPE_COMMIT;
    return OA_COMMISSION_OK;
}

oa_commission_status RecipeCalibrationSession::abort() noexcept {
    if (state_ == OA_RECIPE_ABORT || state_ == OA_RECIPE_COMMIT) {
        return OA_COMMISSION_ESTATE;
    }
    return fail(OA_COMMISSION_OK, OA_ABORT_CALLER);
}

oa_commission_recipe_report RecipeCalibrationSession::report() const noexcept {
    oa_commission_recipe_report value{};
    value.struct_size = sizeof(value);
    value.abi_version = OA_COMMISSION_ABI_V1;
    value.state = state_;
    value.abort_reason = abort_reason_;
    value.last_feedback_seq = last_feedback_seq_;
    value.first_stop_output_rad = first_stop_q_;
    value.second_stop_output_rad = second_stop_q_;
    value.candidate_a = candidate_a_;
    value.candidate_b_rad = candidate_b_;
    value.accumulated_contact_energy_j = contact_energy_j_;
    return value;
}

}  // namespace openarm::commission
