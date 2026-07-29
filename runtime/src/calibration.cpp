/* SPDX-License-Identifier: Apache-2.0 */
#include "runtime_internal.hpp"

#include <algorithm>
#include <cstring>

namespace {

bool commission_text_valid(const char *text, std::size_t capacity) {
    return text != nullptr && std::memchr(text, '\0', capacity) != nullptr && text[0] != '\0';
}

oa_runtime_status snapshot_sample(const std::shared_ptr<openarm::runtime::CalibrationData> &session,
                                  oa_commission_encoder_sample &sample,
    std::uint64_t &now,
    oa_snapshot *out_snapshot = nullptr) {
    if (!session || !session->runtime || session->runtime->controller == nullptr) {
        return OA_RUNTIME_ESTATE;
    }
    if (session->finished) {
        return openarm::runtime::record_error(
            session->runtime, OA_RUNTIME_ESTATE, OA_RUNTIME_FACILITY_RUNTIME);
    }
    bool invalid_runtime_state = false;
    {
        std::lock_guard<std::mutex> lock(session->runtime->mutex);
        invalid_runtime_state = session->runtime->closing ||
                                session->runtime->owner != 2U;
    }
    if (invalid_runtime_state) {
        return openarm::runtime::record_error(
            session->runtime, OA_RUNTIME_ESTATE, OA_RUNTIME_FACILITY_RUNTIME);
    }
    oa_snapshot snapshot{};
    oa_runtime_snapshot runtime_snapshot{};
    const oa_runtime_status snapshot_status = openarm::runtime::capture_runtime_snapshot(
        session->runtime, snapshot, runtime_snapshot);
    if (snapshot_status != OA_RUNTIME_OK) return snapshot_status;
    const oa_arm_snapshot &arm = snapshot.arm[session->side];
    const oa_runtime_arm_snapshot &runtime_arm = runtime_snapshot.arm[session->side];
    sample = {};
    sample.struct_size = sizeof(sample);
    sample.abi_version = OA_COMMISSION_ABI_V1;
    sample.feedback_seq = arm.feedback_seq;
    sample.sample_time_ns = runtime_arm.measurement_runtime_monotonic_ns;
    sample.q_output_rad = arm.raw_q[session->joint];
    sample.dq_output_rad_s = arm.raw_dq[session->joint];
    sample.torque_output_nm = arm.raw_tau[session->joint];
    sample.mos_temperature_c = arm.mos_c[session->joint];
    sample.coil_temperature_c = arm.coil_c[session->joint];
    sample.drive_enabled = arm.status[session->joint] == 1U ? 1U : 0U;
    sample.drive_fault = (arm.fault_mask & (UINT32_C(1) << session->joint)) != 0U ? 1U : 0U;
    now = openarm::runtime::now_ns();
    std::uint64_t observed = session->runtime->timeline_ns.load(std::memory_order_acquire);
    while (observed < now && !session->runtime->timeline_ns.compare_exchange_weak(
                                 observed, now, std::memory_order_release,
                                 std::memory_order_acquire)) {}
    if (sample.feedback_seq == 0U || sample.sample_time_ns == 0U ||
        sample.sample_time_ns > now ||
        sample.feedback_seq <= session->last_sample_feedback_seq ||
        sample.sample_time_ns <= session->last_sample_time_ns) {
        return openarm::runtime::record_error(
            session->runtime, OA_RUNTIME_ESTALE, OA_RUNTIME_FACILITY_RUNTIME);
    }
    session->last_sample_feedback_seq = sample.feedback_seq;
    session->last_sample_time_ns = sample.sample_time_ns;
    if (out_snapshot != nullptr) {
        snapshot.arm[session->side].fresh_mask = runtime_arm.fresh_mask;
        snapshot.arm[session->side].t_ns = runtime_arm.measurement_runtime_monotonic_ns;
        *out_snapshot = snapshot;
    }
    return OA_RUNTIME_OK;
}

oa_runtime_status publish_manifest(
    const std::shared_ptr<openarm::runtime::CalibrationData> &session,
    const oa_commission_mapping_patch &patch, oa_runtime_manifest **out_manifest,
    oa_runtime_manifest_preview *out_preview) {
    if (out_manifest == nullptr || !openarm::runtime::output_valid(out_preview)) {
        return OA_RUNTIME_EABI;
    }
    *out_manifest = nullptr;
    try {
        oa_runtime_manifest_preview preview{};
        auto manifest = openarm::runtime::apply_patch(*session->base, patch, preview);
        if (!manifest) {
            *out_preview = preview;
            return preview.validation_status;
        }
        oa_runtime_manifest *const handle = openarm::runtime::manifests.insert(manifest);
        if (handle == nullptr) return OA_RUNTIME_ENOMEM;
        {
            std::lock_guard<std::mutex> lock(session->runtime->mutex);
            session->runtime->calibration_revision = patch.replacement_revision;
            if (session->runtime->owner == 2U) session->runtime->owner = 0U;
        }
        session->finished = true;
        *out_manifest = handle;
        *out_preview = preview;
        return OA_RUNTIME_OK;
    } catch (...) {
        return OA_RUNTIME_ENOMEM;
    }
}

}

extern "C" oa_runtime_status oa_runtime_calibration_manual_begin(
    oa_runtime *runtime, const oa_commission_manual_options *options,
    oa_runtime_calibration **out_calibration) {
    if (options == nullptr || options->struct_size < sizeof(*options) ||
        options->abi_version != OA_COMMISSION_ABI_V1) {
        return OA_RUNTIME_EABI;
    }
    const auto owner = openarm::runtime::runtimes.pin(runtime);
    if (!owner) return OA_RUNTIME_EINVAL;
    if (out_calibration == nullptr) {
        return openarm::runtime::record_error(
            owner, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    *out_calibration = nullptr;
    if (options->side >= 2U || options->joint >= 7U ||
        !commission_text_valid(options->motor_serial, sizeof(options->motor_serial))) {
        return openarm::runtime::record_error(
            owner, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    if (owner->options.backend != OA_RUNTIME_BACKEND_VIRTUAL || owner->controller == nullptr) {
        return openarm::runtime::record_error(
            owner, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
    }
    const oa_motor_config &motor = owner->manifest->config.arm[options->side].motor[options->joint];
    if (options->expected_revision != owner->manifest->config.manifest_revision ||
        std::strncmp(options->motor_serial, motor.serial, sizeof(motor.serial)) != 0) {
        return openarm::runtime::record_error(
            owner, OA_RUNTIME_EIDENTITY, OA_RUNTIME_FACILITY_RUNTIME);
    }
    try {
        auto session = std::make_shared<openarm::runtime::CalibrationData>();
        session->runtime = owner;
        session->base = owner->manifest;
        session->side = options->side;
        session->joint = options->joint;
        {
            std::lock_guard<std::mutex> lock(owner->mutex);
            if (owner->closing || owner->owner != 0U) {
                openarm::runtime::runtime_init(owner->last_error);
                owner->last_error.status = owner->closing ? OA_RUNTIME_ESTATE
                                                          : OA_RUNTIME_EBUSY;
                owner->last_error.facility = OA_RUNTIME_FACILITY_RUNTIME;
                return owner->last_error.status;
            }
            owner->owner = 2U;
        }
        const oa_commission_status lower = oa_commission_manual_create(options, &session->manual);
        if (lower != OA_COMMISSION_OK) {
            {
                std::lock_guard<std::mutex> lock(owner->mutex);
                owner->owner = 0U;
            }
            return openarm::runtime::record_error(
                owner, openarm::runtime::map_commission(lower),
                OA_RUNTIME_FACILITY_COMMISSION, lower);
        }
        oa_runtime_calibration *const handle = openarm::runtime::calibrations.insert(session);
        if (handle == nullptr) {
            return openarm::runtime::record_error(
                owner, OA_RUNTIME_ENOMEM, OA_RUNTIME_FACILITY_RUNTIME);
        }
        *out_calibration = handle;
        return OA_RUNTIME_OK;
    } catch (...) {
        return openarm::runtime::record_error(
            owner, OA_RUNTIME_ENOMEM, OA_RUNTIME_FACILITY_RUNTIME);
    }
}

extern "C" oa_runtime_status oa_runtime_calibration_manual_sample(
    oa_runtime_calibration *calibration, std::uint32_t reference_index,
    oa_commission_manual_report *out_report) {
    const auto session = openarm::runtime::calibrations.pin(calibration);
    if (!session || session->manual == nullptr) return OA_RUNTIME_EINVAL;
    std::lock_guard<std::mutex> session_lock(session->mutex);
    oa_commission_encoder_sample sample{};
    std::uint64_t now = 0U;
    oa_runtime_status status = snapshot_sample(session, sample, now);
    if (status != OA_RUNTIME_OK) return status;
    const oa_commission_status lower = oa_commission_manual_sample(
        session->manual, reference_index, now, &sample, out_report);
    return openarm::runtime::record_error(
        session->runtime, openarm::runtime::map_commission(lower),
        OA_RUNTIME_FACILITY_COMMISSION, lower);
}

extern "C" oa_runtime_status oa_runtime_calibration_manual_begin_review(
    oa_runtime_calibration *calibration, oa_commission_manual_report *out_report) {
    const auto session = openarm::runtime::calibrations.pin(calibration);
    if (!session || session->manual == nullptr) return OA_RUNTIME_EINVAL;
    std::lock_guard<std::mutex> session_lock(session->mutex);
    if (session->finished) {
        return openarm::runtime::record_error(
            session->runtime, OA_RUNTIME_ESTATE, OA_RUNTIME_FACILITY_RUNTIME);
    }
    const oa_commission_status lower =
        oa_commission_manual_begin_review(session->manual, out_report);
    return openarm::runtime::record_error(
        session->runtime, openarm::runtime::map_commission(lower),
        OA_RUNTIME_FACILITY_COMMISSION, lower);
}

extern "C" oa_runtime_status oa_runtime_calibration_manual_commit(
    oa_runtime_calibration *calibration, std::uint64_t replacement_revision,
    const char *evidence_record, oa_runtime_manifest **out_manifest,
    oa_runtime_manifest_preview *out_preview) {
    const auto session = openarm::runtime::calibrations.pin(calibration);
    if (!session || session->manual == nullptr) {
        return OA_RUNTIME_EINVAL;
    }
    std::lock_guard<std::mutex> session_lock(session->mutex);
    if (session->finished) {
        return openarm::runtime::record_error(
            session->runtime, OA_RUNTIME_ESTATE, OA_RUNTIME_FACILITY_RUNTIME);
    }
    if (!commission_text_valid(evidence_record, OA_COMMISSION_TEXT_CAPACITY)) {
        return openarm::runtime::record_error(
            session->runtime, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    oa_commission_mapping_patch patch{};
    patch.struct_size = sizeof(patch);
    patch.abi_version = OA_COMMISSION_ABI_V1;
    const oa_commission_status lower = oa_commission_manual_commit(
        session->manual, replacement_revision, evidence_record, &patch);
    if (lower != OA_COMMISSION_OK) {
        return openarm::runtime::record_error(
            session->runtime, openarm::runtime::map_commission(lower),
            OA_RUNTIME_FACILITY_COMMISSION, lower);
    }
    return publish_manifest(session, patch, out_manifest, out_preview);
}

extern "C" oa_runtime_status oa_runtime_calibration_recipe_begin(
    oa_runtime *runtime, const oa_commission_recipe *recipe,
    oa_runtime_calibration **out_calibration) {
    if (recipe == nullptr || recipe->struct_size < sizeof(*recipe) ||
        recipe->abi_version != OA_COMMISSION_ABI_V1) return OA_RUNTIME_EABI;
    const auto owner = openarm::runtime::runtimes.pin(runtime);
    if (!owner) return OA_RUNTIME_EINVAL;
    if (out_calibration == nullptr) {
        return openarm::runtime::record_error(
            owner, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    *out_calibration = nullptr;
    if (recipe->side >= 2U || recipe->joint >= 7U) {
        return openarm::runtime::record_error(
            owner, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    if (owner->options.backend != OA_RUNTIME_BACKEND_VIRTUAL || owner->controller == nullptr) {
        return openarm::runtime::record_error(
            owner, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
    }
    const oa_motor_config &motor = owner->manifest->config.arm[recipe->side].motor[recipe->joint];
    if (recipe->expected_revision != owner->manifest->config.manifest_revision ||
        std::strncmp(recipe->motor_serial, motor.serial, sizeof(motor.serial)) != 0) {
        return openarm::runtime::record_error(
            owner, OA_RUNTIME_EIDENTITY, OA_RUNTIME_FACILITY_RUNTIME);
    }
    try {
        auto session = std::make_shared<openarm::runtime::CalibrationData>();
        session->runtime = owner;
        session->base = owner->manifest;
        session->side = recipe->side;
        session->joint = recipe->joint;
        session->required_posture_mask = recipe->required_posture_mask;
        session->evidence_revision = recipe->simulation_only != 0U
                                         ? recipe->simulation_evidence_revision
                                         : recipe->qualification_revision;
        session->fixture_revision = recipe->fixture_revision;
        {
            std::lock_guard<std::mutex> lock(owner->mutex);
            if (owner->closing || owner->owner != 0U) {
                openarm::runtime::runtime_init(owner->last_error);
                owner->last_error.status = owner->closing ? OA_RUNTIME_ESTATE
                                                          : OA_RUNTIME_EBUSY;
                owner->last_error.facility = OA_RUNTIME_FACILITY_RUNTIME;
                return owner->last_error.status;
            }
            owner->owner = 2U;
        }
        const oa_commission_status lower = oa_commission_recipe_create(recipe, &session->recipe);
        if (lower != OA_COMMISSION_OK) {
            {
                std::lock_guard<std::mutex> lock(owner->mutex);
                owner->owner = 0U;
            }
            return openarm::runtime::record_error(
                owner, openarm::runtime::map_commission(lower),
                OA_RUNTIME_FACILITY_COMMISSION, lower);
        }
        oa_runtime_calibration *const handle = openarm::runtime::calibrations.insert(session);
        if (handle == nullptr) {
            return openarm::runtime::record_error(
                owner, OA_RUNTIME_ENOMEM, OA_RUNTIME_FACILITY_RUNTIME);
        }
        *out_calibration = handle;
        return OA_RUNTIME_OK;
    } catch (...) {
        return openarm::runtime::record_error(
            owner, OA_RUNTIME_ENOMEM, OA_RUNTIME_FACILITY_RUNTIME);
    }
}

extern "C" oa_runtime_status oa_runtime_calibration_recipe_step(
    oa_runtime_calibration *calibration, const oa_commission_recipe_input *input,
    oa_commission_next_action *out_action, oa_commission_recipe_report *out_report) {
    const auto session = openarm::runtime::calibrations.pin(calibration);
    if (!session || session->recipe == nullptr) {
        return OA_RUNTIME_EINVAL;
    }
    if (input == nullptr || input->struct_size < sizeof(*input) ||
        input->abi_version != OA_COMMISSION_ABI_V1) return OA_RUNTIME_EABI;
    std::lock_guard<std::mutex> session_lock(session->mutex);
    if (session->finished) {
        return openarm::runtime::record_error(
            session->runtime, OA_RUNTIME_ESTATE, OA_RUNTIME_FACILITY_RUNTIME);
    }
    oa_commission_encoder_sample sample{};
    oa_snapshot snapshot{};
    std::uint64_t now = 0U;
    oa_runtime_status status = snapshot_sample(session, sample, now, &snapshot);
    if (status != OA_RUNTIME_OK) return status;
    oa_commission_recipe_input bound{};
    bound.struct_size = sizeof(bound);
    bound.abi_version = OA_COMMISSION_ABI_V1;
    bound.now_ns = now;
    bound.encoder = sample;
    bound.operator_ready = input->operator_ready;
    bound.action_complete = input->action_complete;
    bound.review_decision = input->review_decision;
    {
        std::lock_guard<std::mutex> runtime_lock(session->runtime->mutex);
        bound.estop_clear = session->runtime->estop_active ? 0U : 1U;
        bound.deadman_held = session->runtime->deadman_active ? 1U : 0U;
    }
    const oa_arm_snapshot &arm = snapshot.arm[session->side];
    bound.posture_mask =
        (arm.fresh_mask & session->required_posture_mask) == session->required_posture_mask
            ? session->required_posture_mask
            : 0U;
    bound.evidence_revision = session->evidence_revision;
    bound.fixture_revision = session->fixture_revision;
    std::copy_n(arm.raw_q, OA_RUNTIME_DOF, bound.posture_output_rad);
    const oa_commission_status lower =
        oa_commission_recipe_step(session->recipe, &bound, out_action, out_report);
    return openarm::runtime::record_error(
        session->runtime, openarm::runtime::map_commission(lower),
        OA_RUNTIME_FACILITY_COMMISSION, lower);
}

extern "C" oa_runtime_status oa_runtime_calibration_recipe_commit(
    oa_runtime_calibration *calibration, std::uint64_t replacement_revision,
    oa_runtime_manifest **out_manifest, oa_runtime_manifest_preview *out_preview) {
    const auto session = openarm::runtime::calibrations.pin(calibration);
    if (!session || session->recipe == nullptr) return OA_RUNTIME_EINVAL;
    std::lock_guard<std::mutex> session_lock(session->mutex);
    if (session->finished) {
        return openarm::runtime::record_error(
            session->runtime, OA_RUNTIME_ESTATE, OA_RUNTIME_FACILITY_RUNTIME);
    }
    oa_commission_mapping_patch patch{};
    patch.struct_size = sizeof(patch);
    patch.abi_version = OA_COMMISSION_ABI_V1;
    const oa_commission_status lower =
        oa_commission_recipe_commit(session->recipe, replacement_revision, &patch);
    if (lower != OA_COMMISSION_OK) {
        return openarm::runtime::record_error(
            session->runtime, openarm::runtime::map_commission(lower),
            OA_RUNTIME_FACILITY_COMMISSION, lower);
    }
    return publish_manifest(session, patch, out_manifest, out_preview);
}

extern "C" oa_runtime_status oa_runtime_calibration_abort(
    oa_runtime_calibration *calibration) {
    const auto session = openarm::runtime::calibrations.pin(calibration);
    if (!session) return OA_RUNTIME_EINVAL;
    std::lock_guard<std::mutex> session_lock(session->mutex);
    if (session->finished) {
        return openarm::runtime::record_error(
            session->runtime, OA_RUNTIME_ESTATE, OA_RUNTIME_FACILITY_RUNTIME);
    }
    const oa_commission_status lower = session->manual != nullptr
                                           ? oa_commission_manual_abort(session->manual)
                                           : oa_commission_recipe_abort(session->recipe);
    const oa_runtime_status status = openarm::runtime::map_commission(lower);
    if (status == OA_RUNTIME_OK) {
        std::lock_guard<std::mutex> lock(session->runtime->mutex);
        session->finished = true;
        if (session->runtime->owner == 2U) session->runtime->owner = 0U;
    }
    return openarm::runtime::record_error(
        session->runtime, status, OA_RUNTIME_FACILITY_COMMISSION, lower);
}

extern "C" void oa_runtime_calibration_destroy(oa_runtime_calibration *calibration) {
    openarm::runtime::calibrations.erase(calibration);
}
