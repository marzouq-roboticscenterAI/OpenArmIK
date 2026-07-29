#include "calibration_session.hpp"
#include "test_hooks.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <unordered_map>

using openarm::commission::ManualCalibrationSession;
using openarm::commission::RecipeCalibrationSession;
using openarm::commission::valid_header;
using openarm::commission::valid_text;

struct oa_commission_manual_session {
    explicit oa_commission_manual_session(const oa_commission_manual_options &options)
        : implementation(std::make_unique<ManualCalibrationSession>(options)) {}
    std::unique_ptr<ManualCalibrationSession> implementation;
};

struct oa_commission_recipe_session {
    explicit oa_commission_recipe_session(const oa_commission_recipe &recipe)
        : implementation(std::make_unique<RecipeCalibrationSession>(recipe)) {}
    std::unique_ptr<RecipeCalibrationSession> implementation;
};

namespace {

enum class HandleKind {
    Manual,
    Recipe
};

std::atomic<bool> fail_allocation{false};
std::atomic<bool> throw_exception{false};

class HandleRegistry final {
public:
    ~HandleRegistry() {
        for (const auto &handle : handles_) {
            if (handle.second.kind == HandleKind::Manual) {
                delete static_cast<oa_commission_manual_session *>(
                    const_cast<void *>(handle.first));
            } else {
                delete static_cast<oa_commission_recipe_session *>(
                    const_cast<void *>(handle.first));
            }
        }
    }

    void add(const void *handle, HandleKind kind) {
        std::lock_guard<std::mutex> lock(mutex_);
        handles_.emplace(handle, Entry{kind, true});
    }

    template <typename Callable>
    oa_commission_status use(const void *handle,
                             HandleKind kind,
                             Callable &&callable) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = handles_.find(handle);
        if (found == handles_.end() || found->second.kind != kind ||
            !found->second.active) {
            return OA_COMMISSION_EINVAL;
        }
        return callable();
    }

    template <typename Cleanup>
    bool retire(void *handle, HandleKind kind, Cleanup &&cleanup) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = handles_.find(handle);
        if (found == handles_.end() || found->second.kind != kind ||
            !found->second.active) {
            return false;
        }
        cleanup();
        found->second.active = false;
        return true;
    }

private:
    struct Entry {
        HandleKind kind;
        bool active;
    };

    std::mutex mutex_;
    std::unordered_map<const void *, Entry> handles_;
};

HandleRegistry &registry() {
    static HandleRegistry instance;
    return instance;
}

bool finite(double value) noexcept {
    return std::isfinite(value);
}

bool valid_manual_options(const oa_commission_manual_options &options) noexcept {
    if (options.side > OA_COMMISSION_RIGHT || options.joint > 7U ||
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
    if ((recipe.recipe_kind != OA_RECIPE_ARM_JOINT &&
         recipe.recipe_kind != OA_RECIPE_GRIPPER) ||
        recipe.side > OA_COMMISSION_RIGHT || recipe.joint > 7U ||
        (recipe.known_sign != -1 && recipe.known_sign != 1) ||
        recipe.hardware_qualified > 1U || recipe.simulation_only > 1U ||
        recipe.minimum_contact_samples < 2U || recipe.expected_revision == 0U ||
        recipe.maximum_sample_age_ns == 0U || recipe.maximum_approach_time_ns == 0U ||
        recipe.contact_dwell_ns == 0U || recipe.maximum_retreat_time_ns == 0U ||
        recipe.fixture_revision == 0U || !valid_text(recipe.motor_serial) ||
        !valid_text(recipe.fixture_record) ||
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
        !finite(recipe.posture_tolerance_rad) ||
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
        const std::uint32_t expected_posture_mask =
            UINT32_C(0x7f) & ~(UINT32_C(1) << recipe.joint);
        if (recipe.joint > 6U || recipe.hardware_qualified != 1U ||
            recipe.simulation_only != 0U ||
            recipe.qualification_revision == 0U ||
            !valid_text(recipe.qualification_record) ||
            recipe.simulation_evidence_revision != 0U ||
            recipe.simulation_evidence_record[0] != '\0' ||
            recipe.required_posture_mask != expected_posture_mask ||
            recipe.posture_tolerance_rad <= 0.0) {
            return false;
        }
        for (std::size_t index = 0; index < 7U; ++index) {
            if (!finite(recipe.required_posture_output_rad[index])) {
                return false;
            }
        }
        return true;
    }
    if (recipe.joint != 7U) {
        return false;
    }
    const bool qualified = recipe.hardware_qualified == 1U &&
                           recipe.simulation_only == 0U &&
                           recipe.qualification_revision > 0U &&
                           valid_text(recipe.qualification_record) &&
                           recipe.simulation_evidence_revision == 0U &&
                           recipe.simulation_evidence_record[0] == '\0';
    const bool simulated = recipe.simulation_only == 1U &&
                           recipe.hardware_qualified == 0U &&
                           recipe.qualification_revision == 0U &&
                           recipe.qualification_record[0] == '\0' &&
                           recipe.simulation_evidence_revision > 0U &&
                           valid_text(recipe.simulation_evidence_record);
    return qualified || simulated;
}

template <typename Callable>
oa_commission_status guard(Callable &&callable) noexcept {
    try {
        if (throw_exception.exchange(false)) {
            throw std::runtime_error("injected exception");
        }
        return callable();
    } catch (const std::bad_alloc &) {
        return OA_COMMISSION_ENOMEM;
    } catch (...) {
        return OA_COMMISSION_EFAULT;
    }
}

template <typename Record>
oa_commission_status validate_record(const Record *record) noexcept {
    if (record == nullptr) {
        return OA_COMMISSION_EINVAL;
    }
    const std::uint32_t struct_size = record->struct_size;
    if (struct_size < sizeof(std::uint32_t) * 2U) {
        return OA_COMMISSION_EABI;
    }
    const std::uint32_t abi_version = record->abi_version;
    return valid_header(struct_size, abi_version, sizeof(Record))
               ? OA_COMMISSION_OK
               : OA_COMMISSION_EABI;
}

}  // namespace

namespace openarm::commission::test {

void fail_next_allocation() noexcept {
    fail_allocation = true;
}

void throw_next_exception() noexcept {
    throw_exception = true;
}

}  // namespace openarm::commission::test

extern "C" oa_commission_status oa_commission_manual_create(
    const oa_commission_manual_options *options,
    oa_commission_manual_session **out_session) {
    if (options == nullptr || out_session == nullptr) {
        return OA_COMMISSION_EINVAL;
    }
    *out_session = nullptr;
    const auto header_status = validate_record(options);
    if (header_status != OA_COMMISSION_OK) {
        return header_status;
    }
    if (!valid_manual_options(*options)) {
        return OA_COMMISSION_EINVAL;
    }
    return guard([&]() {
        if (fail_allocation.exchange(false)) {
            throw std::bad_alloc();
        }
        auto session = std::make_unique<oa_commission_manual_session>(*options);
        registry().add(session.get(), HandleKind::Manual);
        *out_session = session.release();
        return OA_COMMISSION_OK;
    });
}

extern "C" void oa_commission_manual_destroy(oa_commission_manual_session *session) {
    if (session == nullptr) {
        return;
    }
    try {
        (void)registry().retire(session, HandleKind::Manual, [&]() {
            session->implementation.reset();
        });
    } catch (...) {
    }
}

extern "C" oa_commission_status oa_commission_manual_sample(
    oa_commission_manual_session *session,
    std::uint32_t reference_index,
    std::uint64_t now_ns,
    const oa_commission_encoder_sample *sample,
    oa_commission_manual_report *out_report) {
    if (session == nullptr || sample == nullptr) {
        return OA_COMMISSION_EINVAL;
    }
    const auto sample_status = validate_record(sample);
    if (sample_status != OA_COMMISSION_OK) {
        return sample_status;
    }
    const auto output_status = validate_record(out_report);
    if (output_status != OA_COMMISSION_OK) {
        return output_status;
    }
    return guard([&]() {
        return registry().use(session, HandleKind::Manual, [&]() {
            const auto result =
                session->implementation->sample(reference_index, now_ns, *sample);
            const auto report = session->implementation->report();
            *out_report = report;
            return result;
        });
    });
}

extern "C" oa_commission_status oa_commission_manual_begin_review(
    oa_commission_manual_session *session,
    oa_commission_manual_report *out_report) {
    if (session == nullptr) {
        return OA_COMMISSION_EINVAL;
    }
    const auto output_status = validate_record(out_report);
    if (output_status != OA_COMMISSION_OK) {
        return output_status;
    }
    return guard([&]() {
        return registry().use(session, HandleKind::Manual, [&]() {
            const auto result = session->implementation->begin_review();
            const auto report = session->implementation->report();
            *out_report = report;
            return result;
        });
    });
}

extern "C" oa_commission_status oa_commission_manual_commit(
    oa_commission_manual_session *session,
    std::uint64_t replacement_revision,
    const char *evidence_record,
    oa_commission_mapping_patch *out_patch) {
    if (session == nullptr) {
        return OA_COMMISSION_EINVAL;
    }
    const auto output_status = validate_record(out_patch);
    if (output_status != OA_COMMISSION_OK) {
        return output_status;
    }
    return guard([&]() {
        return registry().use(session, HandleKind::Manual, [&]() {
            oa_commission_mapping_patch candidate{};
            const auto result = session->implementation->commit(
                replacement_revision, evidence_record, candidate);
            if (result == OA_COMMISSION_OK) {
                *out_patch = candidate;
            }
            return result;
        });
    });
}

extern "C" oa_commission_status oa_commission_manual_abort(
    oa_commission_manual_session *session) {
    if (session == nullptr) {
        return OA_COMMISSION_EINVAL;
    }
    return guard([&]() {
        return registry().use(session, HandleKind::Manual, [&]() {
            return session->implementation->abort();
        });
    });
}

extern "C" oa_commission_status oa_commission_manual_get_report(
    const oa_commission_manual_session *session,
    oa_commission_manual_report *out_report) {
    if (session == nullptr) {
        return OA_COMMISSION_EINVAL;
    }
    const auto output_status = validate_record(out_report);
    if (output_status != OA_COMMISSION_OK) {
        return output_status;
    }
    return guard([&]() {
        return registry().use(session, HandleKind::Manual, [&]() {
            const auto report = session->implementation->report();
            *out_report = report;
            return OA_COMMISSION_OK;
        });
    });
}

extern "C" oa_commission_status oa_commission_recipe_create(
    const oa_commission_recipe *recipe,
    oa_commission_recipe_session **out_session) {
    if (recipe == nullptr || out_session == nullptr) {
        return OA_COMMISSION_EINVAL;
    }
    *out_session = nullptr;
    const auto header_status = validate_record(recipe);
    if (header_status != OA_COMMISSION_OK) {
        return header_status;
    }
    if (!valid_recipe(*recipe)) {
        const bool arm_unqualified =
            recipe->recipe_kind == OA_RECIPE_ARM_JOINT &&
            (recipe->hardware_qualified != 1U ||
             recipe->qualification_revision == 0U ||
             !valid_text(recipe->qualification_record, true) ||
             recipe->qualification_record[0] == '\0');
        if (arm_unqualified) {
            return OA_COMMISSION_EUNSUPPORTED;
        }
        return OA_COMMISSION_EINVAL;
    }
    return guard([&]() {
        if (fail_allocation.exchange(false)) {
            throw std::bad_alloc();
        }
        auto session = std::make_unique<oa_commission_recipe_session>(*recipe);
        registry().add(session.get(), HandleKind::Recipe);
        *out_session = session.release();
        return OA_COMMISSION_OK;
    });
}

extern "C" void oa_commission_recipe_destroy(oa_commission_recipe_session *session) {
    if (session == nullptr) {
        return;
    }
    try {
        (void)registry().retire(session, HandleKind::Recipe, [&]() {
            session->implementation.reset();
        });
    } catch (...) {
    }
}

extern "C" oa_commission_status oa_commission_recipe_step(
    oa_commission_recipe_session *session,
    const oa_commission_recipe_input *input,
    oa_commission_next_action *out_action,
    oa_commission_recipe_report *out_report) {
    if (session == nullptr || input == nullptr) {
        return OA_COMMISSION_EINVAL;
    }
    const auto input_status = validate_record(input);
    if (input_status != OA_COMMISSION_OK) {
        return input_status;
    }
    const auto encoder_status = validate_record(&input->encoder);
    if (encoder_status != OA_COMMISSION_OK) {
        return encoder_status;
    }
    const auto action_status = validate_record(out_action);
    if (action_status != OA_COMMISSION_OK) {
        return action_status;
    }
    const auto report_status = validate_record(out_report);
    if (report_status != OA_COMMISSION_OK) {
        return report_status;
    }
    return guard([&]() {
        return registry().use(session, HandleKind::Recipe, [&]() {
            oa_commission_next_action action{};
            const auto result = session->implementation->step(*input, action);
            const auto report = session->implementation->report();
            *out_action = action;
            *out_report = report;
            return result;
        });
    });
}

extern "C" oa_commission_status oa_commission_recipe_commit(
    oa_commission_recipe_session *session,
    std::uint64_t replacement_revision,
    oa_commission_mapping_patch *out_patch) {
    if (session == nullptr) {
        return OA_COMMISSION_EINVAL;
    }
    const auto output_status = validate_record(out_patch);
    if (output_status != OA_COMMISSION_OK) {
        return output_status;
    }
    return guard([&]() {
        return registry().use(session, HandleKind::Recipe, [&]() {
            oa_commission_mapping_patch candidate{};
            const auto result =
                session->implementation->commit(replacement_revision, candidate);
            if (result == OA_COMMISSION_OK) {
                *out_patch = candidate;
            }
            return result;
        });
    });
}

extern "C" oa_commission_status oa_commission_recipe_abort(
    oa_commission_recipe_session *session) {
    if (session == nullptr) {
        return OA_COMMISSION_EINVAL;
    }
    return guard([&]() {
        return registry().use(session, HandleKind::Recipe, [&]() {
            return session->implementation->abort();
        });
    });
}

extern "C" oa_commission_status oa_commission_recipe_get_report(
    const oa_commission_recipe_session *session,
    oa_commission_recipe_report *out_report) {
    if (session == nullptr) {
        return OA_COMMISSION_EINVAL;
    }
    const auto output_status = validate_record(out_report);
    if (output_status != OA_COMMISSION_OK) {
        return output_status;
    }
    return guard([&]() {
        return registry().use(session, HandleKind::Recipe, [&]() {
            const auto report = session->implementation->report();
            *out_report = report;
            return OA_COMMISSION_OK;
        });
    });
}
