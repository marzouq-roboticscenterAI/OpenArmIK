#include "calibration_session.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <new>

using openarm::commission::ManualCalibrationSession;
using openarm::commission::RecipeCalibrationSession;
using openarm::commission::valid_header;
using openarm::commission::valid_text;

struct oa_commission_manual_session {
    explicit oa_commission_manual_session(const oa_commission_manual_options &options)
        : implementation(options) {}
    ManualCalibrationSession implementation;
};

struct oa_commission_recipe_session {
    explicit oa_commission_recipe_session(const oa_commission_recipe &recipe)
        : implementation(recipe) {}
    RecipeCalibrationSession implementation;
};

namespace {

bool finite(double value) noexcept {
    return std::isfinite(value);
}

bool valid_manual_options(const oa_commission_manual_options &options) noexcept {
    if (!valid_header(options.struct_size, options.abi_version, sizeof(options)) ||
        options.side > OA_COMMISSION_RIGHT || options.joint > 7U ||
        options.expected_revision == 0U ||
        (options.reference_count != 1U && options.reference_count != 2U) ||
        options.minimum_samples < 2U || options.maximum_sample_age_ns == 0U ||
        options.stability_dwell_ns == 0U ||
        !finite(options.reference_model_rad[0]) ||
        !finite(options.reference_model_rad[1]) ||
        !finite(options.maximum_position_spread_rad) ||
        !finite(options.maximum_abs_velocity_rad_s) ||
        !finite(options.minimum_reference_separation_rad) ||
        !finite(options.maximum_scale_error) ||
        options.maximum_position_spread_rad <= 0.0 ||
        options.maximum_abs_velocity_rad_s <= 0.0 ||
        options.minimum_reference_separation_rad <= 0.0 ||
        options.maximum_scale_error < 0.0 || options.maximum_scale_error >= 1.0 ||
        !valid_text(options.motor_serial)) {
        return false;
    }
    if (options.reference_count == 1U) {
        return options.known_sign == -1 || options.known_sign == 1;
    }
    return options.known_sign >= -1 && options.known_sign <= 1;
}

bool valid_recipe(const oa_commission_recipe &recipe) noexcept {
    if (!valid_header(recipe.struct_size, recipe.abi_version, sizeof(recipe)) ||
        (recipe.recipe_kind != OA_RECIPE_ARM_JOINT &&
         recipe.recipe_kind != OA_RECIPE_GRIPPER) ||
        recipe.side > OA_COMMISSION_RIGHT || recipe.joint > 7U ||
        (recipe.known_sign != -1 && recipe.known_sign != 1) ||
        recipe.hardware_qualified > 1U || recipe.simulation_only > 1U ||
        recipe.minimum_contact_samples < 2U || recipe.expected_revision == 0U ||
        recipe.maximum_sample_age_ns == 0U || recipe.maximum_approach_time_ns == 0U ||
        recipe.contact_dwell_ns == 0U || recipe.maximum_retreat_time_ns == 0U ||
        !valid_text(recipe.motor_serial) || !valid_text(recipe.fixture_record) ||
        !finite(recipe.stop_model_rad) || !finite(recipe.approach_direction) ||
        !finite(recipe.start_min_output_rad) ||
        !finite(recipe.start_max_output_rad) ||
        !finite(recipe.maximum_speed_rad_s) ||
        !finite(recipe.minimum_contact_travel_rad) ||
        !finite(recipe.maximum_approach_travel_rad) ||
        !finite(recipe.contact_velocity_rad_s) ||
        !finite(recipe.minimum_contact_torque_nm) ||
        !finite(recipe.maximum_torque_nm) ||
        !finite(recipe.maximum_contact_energy_j) ||
        !finite(recipe.maximum_temperature_c) ||
        !finite(recipe.retreat_distance_rad) ||
        !finite(recipe.repeatability_tolerance_rad) ||
        std::abs(std::abs(recipe.approach_direction) - 1.0) > 1.0e-12 ||
        recipe.start_min_output_rad >= recipe.start_max_output_rad ||
        recipe.maximum_speed_rad_s <= 0.0 ||
        recipe.contact_velocity_rad_s <= 0.0 ||
        recipe.contact_velocity_rad_s >= recipe.maximum_speed_rad_s ||
        recipe.minimum_contact_travel_rad <= 0.0 ||
        recipe.maximum_approach_travel_rad <= recipe.minimum_contact_travel_rad ||
        recipe.minimum_contact_torque_nm <= 0.0 ||
        recipe.maximum_torque_nm <= recipe.minimum_contact_torque_nm ||
        recipe.maximum_contact_energy_j <= 0.0 ||
        recipe.maximum_temperature_c <= 0.0 || recipe.retreat_distance_rad <= 0.0 ||
        recipe.repeatability_tolerance_rad <= 0.0 ||
        recipe.retreat_distance_rad > recipe.maximum_approach_travel_rad) {
        return false;
    }
    if (recipe.recipe_kind == OA_RECIPE_ARM_JOINT) {
        return recipe.joint <= 6U && recipe.hardware_qualified == 1U &&
               recipe.qualification_revision > 0U &&
               valid_text(recipe.qualification_record);
    }
    if (recipe.joint != 7U) {
        return false;
    }
    const bool qualified = recipe.hardware_qualified == 1U &&
                           recipe.qualification_revision > 0U &&
                           valid_text(recipe.qualification_record);
    const bool simulated = recipe.simulation_only == 1U;
    return qualified || simulated;
}

template <typename Output>
bool valid_output(const Output *output) noexcept {
    return output != nullptr &&
           valid_header(output->struct_size, output->abi_version, sizeof(Output));
}

template <typename Callable>
oa_commission_status guard(Callable &&callable) noexcept {
    try {
        return callable();
    } catch (const std::bad_alloc &) {
        return OA_COMMISSION_ENOMEM;
    } catch (...) {
        return OA_COMMISSION_EFAULT;
    }
}

}  // namespace

extern "C" oa_commission_status oa_commission_manual_create(
    const oa_commission_manual_options *options,
    oa_commission_manual_session **out_session) {
    if (options == nullptr || out_session == nullptr) {
        return OA_COMMISSION_EINVAL;
    }
    *out_session = nullptr;
    if (!valid_manual_options(*options)) {
        return options->abi_version == OA_COMMISSION_ABI_V1 ? OA_COMMISSION_EINVAL
                                                           : OA_COMMISSION_EABI;
    }
    return guard([&]() {
        auto session = std::make_unique<oa_commission_manual_session>(*options);
        *out_session = session.release();
        return OA_COMMISSION_OK;
    });
}

extern "C" void oa_commission_manual_destroy(oa_commission_manual_session *session) {
    delete session;
}

extern "C" oa_commission_status oa_commission_manual_sample(
    oa_commission_manual_session *session,
    std::uint32_t reference_index,
    std::uint64_t now_ns,
    const oa_commission_encoder_sample *sample,
    oa_commission_manual_report *out_report) {
    if (session == nullptr || sample == nullptr || !valid_output(out_report)) {
        return OA_COMMISSION_EINVAL;
    }
    return guard([&]() {
        const auto result =
            session->implementation.sample(reference_index, now_ns, *sample);
        const auto report = session->implementation.report();
        *out_report = report;
        return result;
    });
}

extern "C" oa_commission_status oa_commission_manual_begin_review(
    oa_commission_manual_session *session,
    oa_commission_manual_report *out_report) {
    if (session == nullptr || !valid_output(out_report)) {
        return OA_COMMISSION_EINVAL;
    }
    return guard([&]() {
        const auto result = session->implementation.begin_review();
        const auto report = session->implementation.report();
        *out_report = report;
        return result;
    });
}

extern "C" oa_commission_status oa_commission_manual_commit(
    oa_commission_manual_session *session,
    std::uint64_t replacement_revision,
    const char *evidence_record,
    oa_commission_mapping_patch *out_patch) {
    if (session == nullptr || !valid_output(out_patch)) {
        return OA_COMMISSION_EINVAL;
    }
    return guard([&]() {
        oa_commission_mapping_patch candidate{};
        const auto result = session->implementation.commit(
            replacement_revision, evidence_record, candidate);
        if (result == OA_COMMISSION_OK) {
            *out_patch = candidate;
        }
        return result;
    });
}

extern "C" oa_commission_status oa_commission_manual_abort(
    oa_commission_manual_session *session) {
    if (session == nullptr) {
        return OA_COMMISSION_EINVAL;
    }
    return guard([&]() { return session->implementation.abort(); });
}

extern "C" oa_commission_status oa_commission_manual_get_report(
    const oa_commission_manual_session *session,
    oa_commission_manual_report *out_report) {
    if (session == nullptr || !valid_output(out_report)) {
        return OA_COMMISSION_EINVAL;
    }
    return guard([&]() {
        const auto report = session->implementation.report();
        *out_report = report;
        return OA_COMMISSION_OK;
    });
}

extern "C" oa_commission_status oa_commission_recipe_create(
    const oa_commission_recipe *recipe,
    oa_commission_recipe_session **out_session) {
    if (recipe == nullptr || out_session == nullptr) {
        return OA_COMMISSION_EINVAL;
    }
    *out_session = nullptr;
    if (!valid_recipe(*recipe)) {
        const bool arm_unqualified =
            recipe->abi_version == OA_COMMISSION_ABI_V1 &&
            recipe->recipe_kind == OA_RECIPE_ARM_JOINT &&
            (recipe->hardware_qualified != 1U ||
             recipe->qualification_revision == 0U ||
             !valid_text(recipe->qualification_record, true) ||
             recipe->qualification_record[0] == '\0');
        if (arm_unqualified) {
            return OA_COMMISSION_EUNSUPPORTED;
        }
        return recipe->abi_version == OA_COMMISSION_ABI_V1 ? OA_COMMISSION_EINVAL
                                                          : OA_COMMISSION_EABI;
    }
    return guard([&]() {
        auto session = std::make_unique<oa_commission_recipe_session>(*recipe);
        *out_session = session.release();
        return OA_COMMISSION_OK;
    });
}

extern "C" void oa_commission_recipe_destroy(oa_commission_recipe_session *session) {
    delete session;
}

extern "C" oa_commission_status oa_commission_recipe_step(
    oa_commission_recipe_session *session,
    const oa_commission_recipe_input *input,
    oa_commission_next_action *out_action,
    oa_commission_recipe_report *out_report) {
    if (session == nullptr || input == nullptr || !valid_output(out_action) ||
        !valid_output(out_report)) {
        return OA_COMMISSION_EINVAL;
    }
    return guard([&]() {
        oa_commission_next_action action{};
        const auto result = session->implementation.step(*input, action);
        const auto report = session->implementation.report();
        *out_action = action;
        *out_report = report;
        return result;
    });
}

extern "C" oa_commission_status oa_commission_recipe_commit(
    oa_commission_recipe_session *session,
    std::uint64_t replacement_revision,
    oa_commission_mapping_patch *out_patch) {
    if (session == nullptr || !valid_output(out_patch)) {
        return OA_COMMISSION_EINVAL;
    }
    return guard([&]() {
        oa_commission_mapping_patch candidate{};
        const auto result =
            session->implementation.commit(replacement_revision, candidate);
        if (result == OA_COMMISSION_OK) {
            *out_patch = candidate;
        }
        return result;
    });
}

extern "C" oa_commission_status oa_commission_recipe_abort(
    oa_commission_recipe_session *session) {
    if (session == nullptr) {
        return OA_COMMISSION_EINVAL;
    }
    return guard([&]() { return session->implementation.abort(); });
}

extern "C" oa_commission_status oa_commission_recipe_get_report(
    const oa_commission_recipe_session *session,
    oa_commission_recipe_report *out_report) {
    if (session == nullptr || !valid_output(out_report)) {
        return OA_COMMISSION_EINVAL;
    }
    return guard([&]() {
        const auto report = session->implementation.report();
        *out_report = report;
        return OA_COMMISSION_OK;
    });
}
