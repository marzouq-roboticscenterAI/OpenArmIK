/* SPDX-License-Identifier: Apache-2.0 */
#include "runtime_internal.hpp"

#include <cstring>

namespace {

bool commission_text_valid(const char *text, std::size_t capacity) {
    return text != nullptr && std::memchr(text, '\0', capacity) != nullptr && text[0] != '\0';
}

oa_runtime_status snapshot_sample(const std::shared_ptr<openarm::runtime::CalibrationData> &session,
                                  oa_commission_encoder_sample &sample,
                                  std::uint64_t &now) {
    if (!session || session->finished || !session->runtime ||
        session->runtime->controller == nullptr) return OA_RUNTIME_ESTATE;
    {
        std::lock_guard<std::mutex> lock(session->runtime->mutex);
        if (session->runtime->closing || session->runtime->owner != 2U) return OA_RUNTIME_ESTATE;
    }
    oa_snapshot snapshot{};
    openarm::runtime::control_init(snapshot);
    const oa_control_status lower = oa_controller_snapshot(session->runtime->controller, &snapshot);
    if (lower != OA_CONTROL_OK) return openarm::runtime::map_control(lower);
    const oa_arm_snapshot &arm = snapshot.arm[session->side];
    sample = {};
    sample.struct_size = sizeof(sample);
    sample.abi_version = OA_COMMISSION_ABI_V1;
    sample.feedback_seq = arm.feedback_seq;
    sample.sample_time_ns = arm.t_ns;
    sample.q_output_rad = arm.raw_q[session->joint];
    sample.dq_output_rad_s = arm.raw_dq[session->joint];
    sample.torque_output_nm = arm.raw_tau[session->joint];
    sample.mos_temperature_c = arm.mos_c[session->joint];
    sample.coil_temperature_c = arm.coil_c[session->joint];
    sample.drive_enabled = arm.status[session->joint] == 1U ? 1U : 0U;
    sample.drive_fault = (arm.fault_mask & (UINT32_C(1) << session->joint)) != 0U ? 1U : 0U;
    now = session->runtime->timeline_ns.load(std::memory_order_acquire);
    if (sample.feedback_seq == 0U || sample.sample_time_ns == 0U ||
        sample.sample_time_ns > now) return OA_RUNTIME_ESTALE;
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
            session->finished = true;
        }
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
    if (out_calibration == nullptr) return OA_RUNTIME_EINVAL;
    *out_calibration = nullptr;
    if (options == nullptr || options->struct_size < sizeof(*options) ||
        options->abi_version != OA_COMMISSION_ABI_V1 || options->side >= 2U ||
        options->joint >= 7U || !commission_text_valid(options->motor_serial,
                                                       sizeof(options->motor_serial))) {
        return OA_RUNTIME_EABI;
    }
    const auto owner = openarm::runtime::runtimes.pin(runtime);
    if (!owner) return OA_RUNTIME_EINVAL;
    if (owner->options.backend != OA_RUNTIME_BACKEND_VIRTUAL || owner->controller == nullptr) {
        return OA_RUNTIME_EUNSUPPORTED;
    }
    const oa_motor_config &motor = owner->manifest->config.arm[options->side].motor[options->joint];
    if (options->expected_revision != owner->manifest->config.manifest_revision ||
        std::strncmp(options->motor_serial, motor.serial, sizeof(motor.serial)) != 0) {
        return OA_RUNTIME_EIDENTITY;
    }
    try {
        auto session = std::make_shared<openarm::runtime::CalibrationData>();
        session->runtime = owner;
        session->base = owner->manifest;
        session->side = options->side;
        session->joint = options->joint;
        {
            std::lock_guard<std::mutex> lock(owner->mutex);
            if (owner->closing) return OA_RUNTIME_ESTATE;
            if (owner->owner != 0U) return OA_RUNTIME_EBUSY;
            owner->owner = 2U;
        }
        const oa_commission_status lower = oa_commission_manual_create(options, &session->manual);
        if (lower != OA_COMMISSION_OK) {
            std::lock_guard<std::mutex> lock(owner->mutex);
            owner->owner = 0U;
            return openarm::runtime::map_commission(lower);
        }
        oa_runtime_calibration *const handle = openarm::runtime::calibrations.insert(session);
        if (handle == nullptr) return OA_RUNTIME_ENOMEM;
        *out_calibration = handle;
        return OA_RUNTIME_OK;
    } catch (...) {
        return OA_RUNTIME_ENOMEM;
    }
}

extern "C" oa_runtime_status oa_runtime_calibration_manual_sample(
    oa_runtime_calibration *calibration, std::uint32_t reference_index,
    oa_commission_manual_report *out_report) {
    const auto session = openarm::runtime::calibrations.pin(calibration);
    if (!session || session->manual == nullptr) return OA_RUNTIME_EINVAL;
    oa_commission_encoder_sample sample{};
    std::uint64_t now = 0U;
    oa_runtime_status status = snapshot_sample(session, sample, now);
    if (status != OA_RUNTIME_OK) return status;
    return openarm::runtime::map_commission(oa_commission_manual_sample(
        session->manual, reference_index, now, &sample, out_report));
}

extern "C" oa_runtime_status oa_runtime_calibration_manual_begin_review(
    oa_runtime_calibration *calibration, oa_commission_manual_report *out_report) {
    const auto session = openarm::runtime::calibrations.pin(calibration);
    if (!session || session->manual == nullptr || session->finished) return OA_RUNTIME_EINVAL;
    return openarm::runtime::map_commission(
        oa_commission_manual_begin_review(session->manual, out_report));
}

extern "C" oa_runtime_status oa_runtime_calibration_manual_commit(
    oa_runtime_calibration *calibration, std::uint64_t replacement_revision,
    const char *evidence_record, oa_runtime_manifest **out_manifest,
    oa_runtime_manifest_preview *out_preview) {
    const auto session = openarm::runtime::calibrations.pin(calibration);
    if (!session || session->manual == nullptr || session->finished ||
        !commission_text_valid(evidence_record, OA_COMMISSION_TEXT_CAPACITY)) {
        return OA_RUNTIME_EINVAL;
    }
    oa_commission_mapping_patch patch{};
    patch.struct_size = sizeof(patch);
    patch.abi_version = OA_COMMISSION_ABI_V1;
    const oa_commission_status lower = oa_commission_manual_commit(
        session->manual, replacement_revision, evidence_record, &patch);
    if (lower != OA_COMMISSION_OK) return openarm::runtime::map_commission(lower);
    return publish_manifest(session, patch, out_manifest, out_preview);
}

extern "C" oa_runtime_status oa_runtime_calibration_recipe_begin(
    oa_runtime *runtime, const oa_commission_recipe *recipe,
    oa_runtime_calibration **out_calibration) {
    if (out_calibration == nullptr) return OA_RUNTIME_EINVAL;
    *out_calibration = nullptr;
    if (recipe == nullptr || recipe->struct_size < sizeof(*recipe) ||
        recipe->abi_version != OA_COMMISSION_ABI_V1 || recipe->side >= 2U ||
        recipe->joint >= 7U) return OA_RUNTIME_EABI;
    const auto owner = openarm::runtime::runtimes.pin(runtime);
    if (!owner) return OA_RUNTIME_EINVAL;
    if (owner->options.backend != OA_RUNTIME_BACKEND_VIRTUAL || owner->controller == nullptr) {
        return OA_RUNTIME_EUNSUPPORTED;
    }
    const oa_motor_config &motor = owner->manifest->config.arm[recipe->side].motor[recipe->joint];
    if (recipe->expected_revision != owner->manifest->config.manifest_revision ||
        std::strncmp(recipe->motor_serial, motor.serial, sizeof(motor.serial)) != 0) {
        return OA_RUNTIME_EIDENTITY;
    }
    try {
        auto session = std::make_shared<openarm::runtime::CalibrationData>();
        session->runtime = owner;
        session->base = owner->manifest;
        session->side = recipe->side;
        session->joint = recipe->joint;
        {
            std::lock_guard<std::mutex> lock(owner->mutex);
            if (owner->closing) return OA_RUNTIME_ESTATE;
            if (owner->owner != 0U) return OA_RUNTIME_EBUSY;
            owner->owner = 2U;
        }
        const oa_commission_status lower = oa_commission_recipe_create(recipe, &session->recipe);
        if (lower != OA_COMMISSION_OK) {
            std::lock_guard<std::mutex> lock(owner->mutex);
            owner->owner = 0U;
            return openarm::runtime::map_commission(lower);
        }
        oa_runtime_calibration *const handle = openarm::runtime::calibrations.insert(session);
        if (handle == nullptr) return OA_RUNTIME_ENOMEM;
        *out_calibration = handle;
        return OA_RUNTIME_OK;
    } catch (...) {
        return OA_RUNTIME_ENOMEM;
    }
}

extern "C" oa_runtime_status oa_runtime_calibration_recipe_step(
    oa_runtime_calibration *calibration, const oa_commission_recipe_input *input,
    oa_commission_next_action *out_action, oa_commission_recipe_report *out_report) {
    const auto session = openarm::runtime::calibrations.pin(calibration);
    if (!session || session->recipe == nullptr || session->finished || input == nullptr ||
        input->struct_size < sizeof(*input) || input->abi_version != OA_COMMISSION_ABI_V1) {
        return OA_RUNTIME_EINVAL;
    }
    oa_commission_encoder_sample sample{};
    std::uint64_t now = 0U;
    oa_runtime_status status = snapshot_sample(session, sample, now);
    if (status != OA_RUNTIME_OK) return status;
    oa_commission_recipe_input bound = *input;
    bound.now_ns = now;
    bound.encoder = sample;
    return openarm::runtime::map_commission(
        oa_commission_recipe_step(session->recipe, &bound, out_action, out_report));
}

extern "C" oa_runtime_status oa_runtime_calibration_recipe_commit(
    oa_runtime_calibration *calibration, std::uint64_t replacement_revision,
    oa_runtime_manifest **out_manifest, oa_runtime_manifest_preview *out_preview) {
    const auto session = openarm::runtime::calibrations.pin(calibration);
    if (!session || session->recipe == nullptr || session->finished) return OA_RUNTIME_EINVAL;
    oa_commission_mapping_patch patch{};
    patch.struct_size = sizeof(patch);
    patch.abi_version = OA_COMMISSION_ABI_V1;
    const oa_commission_status lower =
        oa_commission_recipe_commit(session->recipe, replacement_revision, &patch);
    if (lower != OA_COMMISSION_OK) return openarm::runtime::map_commission(lower);
    return publish_manifest(session, patch, out_manifest, out_preview);
}

extern "C" oa_runtime_status oa_runtime_calibration_abort(
    oa_runtime_calibration *calibration) {
    const auto session = openarm::runtime::calibrations.pin(calibration);
    if (!session || session->finished) return OA_RUNTIME_EINVAL;
    const oa_commission_status lower = session->manual != nullptr
                                           ? oa_commission_manual_abort(session->manual)
                                           : oa_commission_recipe_abort(session->recipe);
    const oa_runtime_status status = openarm::runtime::map_commission(lower);
    if (status == OA_RUNTIME_OK) {
        std::lock_guard<std::mutex> lock(session->runtime->mutex);
        session->finished = true;
        if (session->runtime->owner == 2U) session->runtime->owner = 0U;
    }
    return status;
}

extern "C" void oa_runtime_calibration_destroy(oa_runtime_calibration *calibration) {
    openarm::runtime::calibrations.erase(calibration);
}
