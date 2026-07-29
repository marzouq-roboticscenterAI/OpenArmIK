/* SPDX-License-Identifier: Apache-2.0 */
#include "runtime_internal.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <limits>

namespace openarm::runtime {

std::uint64_t now_ns() {
    const auto value = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(value).count());
}

oa_runtime_status map_control(oa_control_status status) {
    switch (status) {
    case OA_CONTROL_OK: return OA_RUNTIME_OK;
    case OA_CONTROL_EABI: return OA_RUNTIME_EABI;
    case OA_CONTROL_ESTATE: return OA_RUNTIME_ESTATE;
    case OA_CONTROL_ESTALE: return OA_RUNTIME_ESTALE;
    case OA_CONTROL_ETIMEOUT: return OA_RUNTIME_ETIMEOUT;
    case OA_CONTROL_EBUSY: return OA_RUNTIME_EBUSY;
    case OA_CONTROL_EIDENTITY: return OA_RUNTIME_EIDENTITY;
    case OA_CONTROL_EIO: case OA_CONTROL_ECAN: return OA_RUNTIME_EIO;
    case OA_CONTROL_ENOMEM: return OA_RUNTIME_ENOMEM;
    case OA_CONTROL_EUNSUPPORTED: return OA_RUNTIME_EUNSUPPORTED;
    case OA_CONTROL_ECOLLISION: return OA_RUNTIME_ECOLLISION;
    case OA_CONTROL_EFAULT: case OA_CONTROL_EESTOP: case OA_CONTROL_ELIMIT:
        return OA_RUNTIME_EFAULT;
    default: return OA_RUNTIME_EINVAL;
    }
}

oa_runtime_status map_commission(oa_commission_status status) {
    switch (status) {
    case OA_COMMISSION_OK: return OA_RUNTIME_OK;
    case OA_COMMISSION_EABI: return OA_RUNTIME_EABI;
    case OA_COMMISSION_ESTATE: return OA_RUNTIME_ESTATE;
    case OA_COMMISSION_ESTALE: return OA_RUNTIME_ESTALE;
    case OA_COMMISSION_EUNSUPPORTED: return OA_RUNTIME_EUNSUPPORTED;
    case OA_COMMISSION_ENOMEM: return OA_RUNTIME_ENOMEM;
    case OA_COMMISSION_EFAULT: case OA_COMMISSION_EINTERLOCK:
    case OA_COMMISSION_ELIMIT: case OA_COMMISSION_EUNSTABLE:
    case OA_COMMISSION_EREPEATABILITY: return OA_RUNTIME_EFAULT;
    default: return OA_RUNTIME_EINVAL;
    }
}

oa_runtime_status map_can(oa_can_status status) {
    switch (status) {
    case OA_CAN_OK: return OA_RUNTIME_OK;
    case OA_CAN_ETIMEOUT: return OA_RUNTIME_ETIMEOUT;
    case OA_CAN_EIO: return OA_RUNTIME_EIO;
    case OA_CAN_ENOMEM: return OA_RUNTIME_ENOMEM;
    case OA_CAN_EUNSUPPORTED: return OA_RUNTIME_EUNSUPPORTED;
    case OA_CAN_EFAULT: return OA_RUNTIME_EFAULT;
    default: return OA_RUNTIME_EINVAL;
    }
}

oa_runtime_status map_transport(oa_transport_status status) {
    switch (status) {
    case OA_TRANSPORT_OK: return OA_RUNTIME_OK;
    case OA_TRANSPORT_EABI: return OA_RUNTIME_EABI;
    case OA_TRANSPORT_EPERMISSION: return OA_RUNTIME_EPERMISSION;
    case OA_TRANSPORT_ETIMEOUT: return OA_RUNTIME_ETIMEOUT;
    case OA_TRANSPORT_ECLOSED: return OA_RUNTIME_ESTATE;
    case OA_TRANSPORT_ENOMEM: return OA_RUNTIME_ENOMEM;
    case OA_TRANSPORT_EUNSUPPORTED: return OA_RUNTIME_EUNSUPPORTED;
    case OA_TRANSPORT_EIO: case OA_TRANSPORT_ELINK: case OA_TRANSPORT_EFRAME:
        return OA_RUNTIME_EIO;
    default: return OA_RUNTIME_EINVAL;
    }
}

void set_error(const std::shared_ptr<RuntimeData> &runtime, oa_runtime_status status,
               oa_runtime_facility facility, std::uint32_t lower_code,
               std::uint32_t system_error) {
    std::lock_guard<std::mutex> lock(runtime->mutex);
    runtime_init(runtime->last_error);
    runtime->last_error.status = status;
    runtime->last_error.facility = facility;
    runtime->last_error.lower_code = lower_code;
    runtime->last_error.system_error = system_error;
}

oa_runtime_capability capabilities_for(oa_runtime_backend backend) {
    const oa_runtime_capability common = OA_RUNTIME_CAP_MODEL_FK |
        OA_RUNTIME_CAP_SINGLE_XYZ_IK | OA_RUNTIME_CAP_PAIRED_XYZ_IK |
        OA_RUNTIME_CAP_MANIFEST_PREVIEW | OA_RUNTIME_CAP_MANIFEST_PERSISTENCE;
    if (backend == OA_RUNTIME_BACKEND_VIRTUAL) {
        return common | OA_RUNTIME_CAP_INTERFACE_ENUMERATION |
            OA_RUNTIME_CAP_VIRTUAL_COORDINATES | OA_RUNTIME_CAP_VIRTUAL_JOINT_MOTION |
            OA_RUNTIME_CAP_VIRTUAL_PAIRED_XYZ_MOTION |
            OA_RUNTIME_CAP_VIRTUAL_MANUAL_CALIBRATION |
            OA_RUNTIME_CAP_VIRTUAL_SUPERVISED_CALIBRATION;
    }
    if (backend == OA_RUNTIME_BACKEND_SOCKETCAN_QUERY) {
        return common | OA_RUNTIME_CAP_INTERFACE_ENUMERATION |
            OA_RUNTIME_CAP_PHYSICAL_REGISTER_QUERY;
    }
    return common;
}

RuntimeData::~RuntimeData() {
    {
        std::lock_guard<std::mutex> lock(mutex);
        closing = true;
        wake.notify_all();
    }
    if (worker.joinable()) worker.join();
    if (controller != nullptr) oa_controller_destroy(controller);
    if (control_manifest != nullptr) oa_manifest_destroy(control_manifest);
}

PlanData::~PlanData() {
    if (plan != nullptr) oa_motion_plan_destroy(plan);
    if (runtime && paused) {
        std::lock_guard<std::mutex> lock(runtime->mutex);
        if (runtime->paused_plans > 0U) --runtime->paused_plans;
        runtime->wake.notify_all();
    }
}

CalibrationData::~CalibrationData() {
    if (manual != nullptr) oa_commission_manual_destroy(manual);
    if (recipe != nullptr) oa_commission_recipe_destroy(recipe);
    if (runtime) {
        std::lock_guard<std::mutex> lock(runtime->mutex);
        if (runtime->owner == 2U) runtime->owner = 0U;
    }
}

oa_runtime_status fill_snapshot(const oa_snapshot &source, oa_runtime_snapshot &target) {
    runtime_init(target);
    target.clock_id = OA_RUNTIME_CLOCK_MONOTONIC;
    target.units_id = OA_RUNTIME_UNITS_SI_V1;
    target.frame_id = OA_RUNTIME_FRAME_OPENARM_BODY_LINK0;
    target.lifecycle = source.lifecycle;
    target.manifest_revision = source.manifest_revision;
    target.model_revision = source.model_revision;
    target.maximum_cross_bus_skew_ns = source.max_cross_bus_skew_ns;
    for (std::size_t side = 0U; side < 2U; ++side) {
        oa_runtime_arm_snapshot &to = target.arm[side];
        runtime_init(to);
        const oa_arm_snapshot &from = source.arm[side];
        to.feedback_seq = from.feedback_seq;
        to.measurement_runtime_monotonic_ns = from.t_ns;
        to.expected_mask = from.expected_mask;
        to.fresh_mask = from.fresh_mask;
        to.fault_mask = from.fault_mask;
        std::copy_n(from.q, 7U, to.q_model_rad);
        std::copy_n(from.dq, 7U, to.dq_model_rad_s);
        std::copy_n(from.tau, 7U, to.tau_model_nm);
        std::copy_n(from.raw_q, 7U, to.q_output_rad);
        std::copy_n(from.raw_dq, 7U, to.dq_output_rad_s);
        std::copy_n(from.raw_tau, 7U, to.tau_output_nm);
        std::copy_n(from.status, 7U, to.status);
        std::copy_n(from.mos_c, 7U, to.mos_temperature_c);
        std::copy_n(from.coil_c, 7U, to.coil_temperature_c);
    }
    return OA_RUNTIME_OK;
}

}

namespace {

bool runtime_options_valid(const oa_runtime_options *options) {
    return options != nullptr && options->struct_size >= sizeof(*options) &&
           options->abi_version == OA_RUNTIME_ABI_VERSION &&
           options->backend >= OA_RUNTIME_BACKEND_VIRTUAL &&
           options->backend <= OA_RUNTIME_BACKEND_OFFLINE &&
           options->allow_unchecked_virtual_motion <= 1U &&
           options->cycle_ns >= 1000000U && options->cycle_ns <= 1000000000U &&
           options->feedback_timeout_ns >= options->cycle_ns &&
           options->maximum_cross_bus_skew_ns > 0U;
}

oa_runtime_status require_runtime(const oa_runtime *handle,
                                  std::shared_ptr<openarm::runtime::RuntimeData> &out) {
    out = openarm::runtime::runtimes.pin(handle);
    if (!out) return OA_RUNTIME_EINVAL;
    std::lock_guard<std::mutex> lock(out->mutex);
    return out->closing ? OA_RUNTIME_ESTATE : OA_RUNTIME_OK;
}

}

extern "C" oa_runtime_status oa_runtime_create(
    const oa_runtime_options *options, const oa_runtime_manifest *manifest,
    oa_runtime **out_runtime) {
    if (out_runtime == nullptr) return OA_RUNTIME_EINVAL;
    *out_runtime = nullptr;
    if (!runtime_options_valid(options)) return OA_RUNTIME_EABI;
    const auto manifest_data = openarm::runtime::manifests.pin(manifest);
    if (!manifest_data) return OA_RUNTIME_EINVAL;
    if (options->backend == OA_RUNTIME_BACKEND_VIRTUAL &&
        (manifest_data->intended_backend != OA_RUNTIME_BACKEND_VIRTUAL ||
         manifest_data->state != OA_RUNTIME_MANIFEST_ARMABLE)) {
        return OA_RUNTIME_EIDENTITY;
    }
    try {
        auto value = std::make_shared<openarm::runtime::RuntimeData>();
        value->options = *options;
        value->manifest = manifest_data;
        openarm::runtime::runtime_init(value->last_error);
        value->last_error.status = OA_RUNTIME_OK;
        value->last_error.facility = OA_RUNTIME_FACILITY_RUNTIME;
        if (options->backend == OA_RUNTIME_BACKEND_VIRTUAL) {
            oa_control_status lower = oa_manifest_create(&manifest_data->config,
                                                         &value->control_manifest);
            if (lower != OA_CONTROL_OK) return openarm::runtime::map_control(lower);
            oa_controller_options controller_options{};
            openarm::runtime::control_init(controller_options);
            controller_options.backend = OA_BACKEND_VIRTUAL;
            controller_options.collision_policy = options->allow_unchecked_virtual_motion != 0U
                                                      ? OA_COLLISION_VIRTUAL_UNCHECKED
                                                      : OA_COLLISION_REJECT_ALL;
            controller_options.cycle_ns = options->cycle_ns;
            controller_options.feedback_timeout_ns = options->feedback_timeout_ns;
            controller_options.max_cross_bus_skew_ns = options->maximum_cross_bus_skew_ns;
            controller_options.collision_scene_revision = options->collision_scene_revision;
            lower = oa_controller_create(value->control_manifest, &controller_options,
                                         &value->controller);
            if (lower != OA_CONTROL_OK) return openarm::runtime::map_control(lower);
            oa_verify_report verify{};
            openarm::runtime::control_init(verify);
            lower = oa_controller_open_and_verify(value->controller, &verify);
            if (lower != OA_CONTROL_OK) return openarm::runtime::map_control(lower);
            value->timeline_ns.store(openarm::runtime::now_ns(), std::memory_order_release);
            lower = oa_controller_advance(
                value->controller, value->timeline_ns.load(std::memory_order_acquire));
            if (lower != OA_CONTROL_OK) return openarm::runtime::map_control(lower);
            openarm::runtime::RuntimeData *const raw = value.get();
            value->worker = std::thread([raw] {
                std::unique_lock<std::mutex> lock(raw->mutex);
                while (!raw->closing) {
                    const std::uint64_t cycle = raw->options.cycle_ns;
                    if (raw->paused_plans > 0U) {
                        raw->wake.wait_for(lock, std::chrono::nanoseconds(cycle),
                                           [raw] { return raw->closing ||
                                                          raw->paused_plans == 0U; });
                        continue;
                    }
                    lock.unlock();
                    const std::uint64_t next =
                        raw->timeline_ns.load(std::memory_order_acquire) + cycle;
                    const oa_control_status status = oa_controller_advance(raw->controller, next);
                    raw->timeline_ns.store(next, std::memory_order_release);
                    if (status != OA_CONTROL_OK && status != OA_CONTROL_ESTATE &&
                        status != OA_CONTROL_ESTALE && status != OA_CONTROL_EFAULT &&
                        status != OA_CONTROL_EESTOP) {
                        std::lock_guard<std::mutex> error_lock(raw->mutex);
                        openarm::runtime::runtime_init(raw->last_error);
                        raw->last_error.status = openarm::runtime::map_control(status);
                        raw->last_error.facility = OA_RUNTIME_FACILITY_CONTROL;
                        raw->last_error.lower_code = status;
                    }
                    lock.lock();
                    raw->wake.wait_for(lock, std::chrono::nanoseconds(cycle),
                                       [raw] { return raw->closing; });
                }
            });
        }
        oa_runtime *const handle = openarm::runtime::runtimes.insert(value);
        if (handle == nullptr) return OA_RUNTIME_ENOMEM;
        *out_runtime = handle;
        return OA_RUNTIME_OK;
    } catch (...) {
        return OA_RUNTIME_ENOMEM;
    }
}

extern "C" void oa_runtime_destroy(oa_runtime *runtime) {
    const auto pinned = openarm::runtime::runtimes.pin(runtime);
    if (pinned) {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        pinned->closing = true;
        pinned->wake.notify_all();
    }
    openarm::runtime::runtimes.erase(runtime);
}

extern "C" oa_runtime_status oa_runtime_get_capabilities(
    const oa_runtime *runtime, oa_runtime_capability_report *out_report) {
    if (!openarm::runtime::output_valid(out_report)) return OA_RUNTIME_EABI;
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    const oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    oa_runtime_capability_report result{};
    openarm::runtime::runtime_init(result);
    result.backend = pinned->options.backend;
    result.clock_id = OA_RUNTIME_CLOCK_MONOTONIC;
    result.units_id = OA_RUNTIME_UNITS_SI_V1;
    result.xyz_frame_id = OA_RUNTIME_FRAME_OPENARM_BODY_LINK0;
    result.orientation_policy = OA_RUNTIME_ORIENTATION_FREE;
    result.collision_checked = 0U;
    result.capabilities = openarm::runtime::capabilities_for(pinned->options.backend);
    *out_report = result;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_now_monotonic_ns(
    const oa_runtime *runtime, oa_runtime_clock_id clock_id, std::uint64_t *out_now_ns) {
    if (out_now_ns == nullptr || clock_id != OA_RUNTIME_CLOCK_MONOTONIC) {
        return OA_RUNTIME_EINVAL;
    }
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    const oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    *out_now_ns = pinned->timeline_ns.load(std::memory_order_acquire);
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_get_last_error(
    const oa_runtime *runtime, oa_runtime_error_detail *out_detail) {
    if (!openarm::runtime::output_valid(out_detail)) return OA_RUNTIME_EABI;
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    const oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    std::lock_guard<std::mutex> lock(pinned->mutex);
    *out_detail = pinned->last_error;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_snapshot_get(
    oa_runtime *runtime, oa_runtime_snapshot *out_snapshot) {
    if (!openarm::runtime::output_valid(out_snapshot)) return OA_RUNTIME_EABI;
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (pinned->controller == nullptr) return OA_RUNTIME_EUNSUPPORTED;
    oa_snapshot snapshot{};
    openarm::runtime::control_init(snapshot);
    const oa_control_status lower = oa_controller_snapshot(pinned->controller, &snapshot);
    status = openarm::runtime::map_control(lower);
    if (status != OA_RUNTIME_OK) {
        openarm::runtime::set_error(pinned, status, OA_RUNTIME_FACILITY_CONTROL, lower);
        return status;
    }
    oa_runtime_snapshot result{};
    openarm::runtime::fill_snapshot(snapshot, result);
    *out_snapshot = result;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_get_kinematics(
    oa_runtime *runtime, std::uint32_t side, std::uint64_t required_feedback_seq,
    oa_runtime_kinematics *out_kinematics) {
    if (!openarm::runtime::output_valid(out_kinematics)) return OA_RUNTIME_EABI;
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (pinned->controller == nullptr || side >= 2U) return OA_RUNTIME_EUNSUPPORTED;
    oa_arm_kinematics kinematics{};
    openarm::runtime::control_init(kinematics);
    const oa_control_status lower = oa_controller_get_kinematics(
        pinned->controller, side, required_feedback_seq, &kinematics);
    status = openarm::runtime::map_control(lower);
    if (status != OA_RUNTIME_OK) return status;
    oa_runtime_kinematics result{};
    openarm::runtime::runtime_init(result);
    result.frame_id = OA_RUNTIME_FRAME_OPENARM_BODY_LINK0;
    result.units_id = OA_RUNTIME_UNITS_SI_V1;
    result.orientation_policy = OA_RUNTIME_ORIENTATION_FREE;
    result.side = side;
    result.feedback_seq = kinematics.feedback_seq;
    std::copy_n(kinematics.q, 7U, result.q_model_rad);
    std::copy_n(&kinematics.joint_xyz_m[0][0], 21U, &result.joint_xyz_m[0][0]);
    std::copy_n(&kinematics.joint_axis_body[0][0], 21U, &result.joint_axis_body[0][0]);
    std::copy_n(kinematics.tcp_transform, 16U, result.tcp_transform_row_major);
    std::copy_n(kinematics.tcp_xyz_m, 3U, result.tcp_xyz_m);
    *out_kinematics = result;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_set_interlock(
    oa_runtime *runtime, std::uint32_t estop_active, std::uint32_t deadman_active) {
    if (estop_active > 1U || deadman_active > 1U) return OA_RUNTIME_EINVAL;
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (pinned->controller == nullptr) return OA_RUNTIME_EUNSUPPORTED;
    {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        pinned->estop_active = estop_active != 0U;
        pinned->deadman_active = deadman_active != 0U;
    }
    return openarm::runtime::map_control(oa_controller_set_interlock(
        pinned->controller, estop_active, deadman_active));
}

extern "C" oa_runtime_status oa_runtime_arm_virtual(oa_runtime *runtime) {
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (pinned->controller == nullptr || pinned->options.backend != OA_RUNTIME_BACKEND_VIRTUAL) {
        return OA_RUNTIME_EUNSUPPORTED;
    }
    std::lock_guard<std::mutex> lock(pinned->mutex);
    if (pinned->owner != 0U) return OA_RUNTIME_EBUSY;
    if (pinned->estop_active || !pinned->deadman_active) return OA_RUNTIME_EFAULT;
    oa_arm_challenge challenge{};
    openarm::runtime::control_init(challenge);
    oa_control_status lower = oa_controller_get_arm_challenge(pinned->controller, &challenge);
    if (lower == OA_CONTROL_OK) lower = oa_controller_arm(pinned->controller, &challenge);
    status = openarm::runtime::map_control(lower);
    if (status == OA_RUNTIME_OK) pinned->owner = 1U;
    return status;
}

extern "C" oa_runtime_status oa_runtime_plan_joint(
    oa_runtime *runtime, const oa_runtime_joint_move *request,
    oa_runtime_plan **out_plan) {
    if (out_plan == nullptr) return OA_RUNTIME_EINVAL;
    *out_plan = nullptr;
    if (request == nullptr || request->struct_size < sizeof(*request) ||
        request->abi_version != OA_RUNTIME_ABI_VERSION ||
        request->clock_id != OA_RUNTIME_CLOCK_MONOTONIC ||
        request->units_id != OA_RUNTIME_UNITS_SI_V1) return OA_RUNTIME_EABI;
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (pinned->controller == nullptr) return OA_RUNTIME_EUNSUPPORTED;
    {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        if (pinned->owner != 1U) return OA_RUNTIME_EBUSY;
        ++pinned->paused_plans;
    }
    oa_snapshot current{};
    openarm::runtime::control_init(current);
    oa_control_status lower = oa_controller_snapshot(pinned->controller, &current);
    if (lower != OA_CONTROL_OK) {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        --pinned->paused_plans;
        pinned->wake.notify_all();
        return openarm::runtime::map_control(lower);
    }
    if (request->side >= 2U ||
        request->required_feedback_seq != current.arm[request->side].feedback_seq) {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        --pinned->paused_plans;
        pinned->wake.notify_all();
        return OA_RUNTIME_ESTALE;
    }
    oa_joint_move move{};
    openarm::runtime::control_init(move);
    move.expiry_ns = request->expiry_runtime_monotonic_ns;
    move.required_feedback_seq = request->required_feedback_seq;
    move.side = request->side; move.joint = request->joint;
    move.target_rad = request->target_model_rad;
    move.velocity_scale = request->velocity_scale;
    move.acceleration_scale = request->acceleration_scale;
    move.jerk_scale = request->jerk_scale;
    move.position_tol_rad = request->position_tolerance_rad;
    move.velocity_tol_rad_s = request->velocity_tolerance_rad_s;
    std::shared_ptr<openarm::runtime::PlanData> plan;
    try {
        plan = std::make_shared<openarm::runtime::PlanData>();
    } catch (...) {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        --pinned->paused_plans;
        pinned->wake.notify_all();
        return OA_RUNTIME_ENOMEM;
    }
    plan->runtime = pinned;
    plan->paused = true;
    lower = oa_controller_plan_joint(pinned->controller, &move, &plan->plan);
    status = openarm::runtime::map_control(lower);
    if (status != OA_RUNTIME_OK) return status;
    oa_runtime_plan *const handle = openarm::runtime::plans.insert(plan);
    if (handle == nullptr) return OA_RUNTIME_ENOMEM;
    *out_plan = handle;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_plan_paired_tcp_body(
    oa_runtime *runtime, const oa_runtime_paired_tcp_move *request,
    oa_runtime_plan **out_plan) {
    if (out_plan == nullptr) return OA_RUNTIME_EINVAL;
    *out_plan = nullptr;
    if (request == nullptr || request->struct_size < sizeof(*request) ||
        request->abi_version != OA_RUNTIME_ABI_VERSION ||
        request->clock_id != OA_RUNTIME_CLOCK_MONOTONIC ||
        request->units_id != OA_RUNTIME_UNITS_SI_V1 ||
        request->frame_id != OA_RUNTIME_FRAME_OPENARM_BODY_LINK0 ||
        request->orientation_policy != OA_RUNTIME_ORIENTATION_FREE) return OA_RUNTIME_EABI;
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (pinned->controller == nullptr) return OA_RUNTIME_EUNSUPPORTED;
    {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        if (pinned->owner != 1U) return OA_RUNTIME_EBUSY;
        ++pinned->paused_plans;
    }
    oa_snapshot current{};
    openarm::runtime::control_init(current);
    oa_control_status lower = oa_controller_snapshot(pinned->controller, &current);
    if (lower != OA_CONTROL_OK) {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        --pinned->paused_plans;
        pinned->wake.notify_all();
        return openarm::runtime::map_control(lower);
    }
    if (request->required_feedback_seq[0] != current.arm[0].feedback_seq ||
        request->required_feedback_seq[1] != current.arm[1].feedback_seq) {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        --pinned->paused_plans;
        pinned->wake.notify_all();
        return OA_RUNTIME_ESTALE;
    }
    oa_paired_tcp_move move{};
    openarm::runtime::control_init(move);
    move.expiry_ns = request->expiry_runtime_monotonic_ns;
    std::copy_n(request->required_feedback_seq, 2U, move.required_feedback_seq);
    std::copy_n(request->left_tcp_m, 3U, move.left_tcp_m);
    std::copy_n(request->right_tcp_m, 3U, move.right_tcp_m);
    move.velocity_scale = request->velocity_scale;
    move.acceleration_scale = request->acceleration_scale;
    move.jerk_scale = request->jerk_scale;
    move.tcp_tol_m = request->tcp_tolerance_m;
    move.collision_scene_revision = request->collision_scene_revision;
    move.max_branch_step_rad = request->maximum_branch_step_rad;
    move.min_singular_value = request->minimum_singular_value;
    std::shared_ptr<openarm::runtime::PlanData> plan;
    try {
        plan = std::make_shared<openarm::runtime::PlanData>();
    } catch (...) {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        --pinned->paused_plans;
        pinned->wake.notify_all();
        return OA_RUNTIME_ENOMEM;
    }
    plan->runtime = pinned;
    plan->paused = true;
    lower = oa_controller_plan_paired_tcp(pinned->controller, &move, &plan->plan);
    status = openarm::runtime::map_control(lower);
    if (status != OA_RUNTIME_OK) return status;
    oa_runtime_plan *const handle = openarm::runtime::plans.insert(plan);
    if (handle == nullptr) return OA_RUNTIME_ENOMEM;
    *out_plan = handle;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_plan_get_report(
    const oa_runtime_plan *plan, oa_runtime_plan_report *out_report) {
    if (!openarm::runtime::output_valid(out_report)) return OA_RUNTIME_EABI;
    const auto pinned = openarm::runtime::plans.pin(plan);
    if (!pinned) return OA_RUNTIME_EINVAL;
    oa_motion_plan_report source{};
    openarm::runtime::control_init(source);
    const oa_control_status lower = oa_motion_plan_get_report(pinned->plan, &source);
    if (lower != OA_CONTROL_OK) return openarm::runtime::map_control(lower);
    oa_runtime_plan_report result{};
    openarm::runtime::runtime_init(result);
    result.kind = source.kind; result.collision_checked = source.collision_checked;
    result.motion_authorized = 0U;
    result.frame_id = OA_RUNTIME_FRAME_OPENARM_BODY_LINK0;
    result.units_id = OA_RUNTIME_UNITS_SI_V1;
    std::copy_n(source.seed_feedback_seq, 2U, result.seed_feedback_seq);
    result.duration_ns = source.duration_ns; result.manifest_revision = source.manifest_revision;
    result.model_revision = source.model_revision;
    result.collision_scene_revision = source.collision_scene_revision;
    std::copy_n(&source.target_q[0][0], 14U, &result.target_q_model_rad[0][0]);
    std::copy_n(&source.achieved_tcp_m[0][0], 6U, &result.achieved_tcp_m[0][0]);
    std::copy_n(source.tcp_residual_m, 2U, result.tcp_residual_m);
    *out_report = result;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_execute(
    oa_runtime *runtime, const oa_runtime_plan *plan,
    const oa_runtime_execute_request *request, std::uint64_t *out_command_id) {
    if (out_command_id == nullptr) return OA_RUNTIME_EINVAL;
    *out_command_id = 0U;
    if (request == nullptr || request->struct_size < sizeof(*request) ||
        request->abi_version != OA_RUNTIME_ABI_VERSION ||
        request->clock_id != OA_RUNTIME_CLOCK_MONOTONIC) return OA_RUNTIME_EABI;
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    const auto plan_data = openarm::runtime::plans.pin(plan);
    if (!plan_data || plan_data->runtime.get() != pinned.get()) return OA_RUNTIME_EINVAL;
    if (pinned->controller == nullptr) return OA_RUNTIME_EUNSUPPORTED;
    oa_execute_request execute{};
    openarm::runtime::control_init(execute);
    execute.start_ns = request->start_runtime_monotonic_ns;
    execute.expiry_ns = request->expiry_runtime_monotonic_ns;
    execute.producer_deadline_ns = request->producer_deadline_runtime_monotonic_ns;
    execute.stop_kind = request->stop_kind;
    status = openarm::runtime::map_control(oa_controller_execute(
        pinned->controller, plan_data->plan, &execute, out_command_id));
    if (status == OA_RUNTIME_OK && plan_data->paused) {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        plan_data->paused = false;
        if (pinned->paused_plans > 0U) --pinned->paused_plans;
        pinned->wake.notify_all();
    }
    return status;
}

extern "C" oa_runtime_status oa_runtime_heartbeat(
    oa_runtime *runtime, std::uint64_t command_id, oa_runtime_clock_id clock_id,
    std::uint64_t producer_deadline_runtime_monotonic_ns) {
    if (clock_id != OA_RUNTIME_CLOCK_MONOTONIC) return OA_RUNTIME_EINVAL;
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    const oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (pinned->controller == nullptr) return OA_RUNTIME_EUNSUPPORTED;
    return openarm::runtime::map_control(oa_controller_heartbeat(
        pinned->controller, command_id, producer_deadline_runtime_monotonic_ns));
}

extern "C" oa_runtime_status oa_runtime_stop(oa_runtime *runtime, std::uint32_t stop_kind) {
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    const oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (pinned->controller == nullptr) return OA_RUNTIME_EUNSUPPORTED;
    return openarm::runtime::map_control(oa_controller_stop(pinned->controller, stop_kind));
}

extern "C" oa_runtime_status oa_runtime_disarm(
    oa_runtime *runtime, oa_runtime_clock_id clock_id, std::uint64_t deadline_ns) {
    if (clock_id != OA_RUNTIME_CLOCK_MONOTONIC) return OA_RUNTIME_EINVAL;
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (pinned->controller == nullptr) return OA_RUNTIME_EUNSUPPORTED;
    status = openarm::runtime::map_control(
        oa_controller_disarm(pinned->controller, deadline_ns));
    if (status == OA_RUNTIME_OK) {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        if (pinned->owner == 1U) pinned->owner = 0U;
    }
    return status;
}

extern "C" oa_runtime_status oa_runtime_poll_event(
    oa_runtime *runtime, std::uint64_t wait_timeout_ns, oa_runtime_event *out_event) {
    if (!openarm::runtime::output_valid(out_event)) return OA_RUNTIME_EABI;
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (pinned->controller == nullptr) return OA_RUNTIME_EUNSUPPORTED;
    const std::uint64_t now = openarm::runtime::now_ns();
    if (wait_timeout_ns > std::numeric_limits<std::uint64_t>::max() - now) {
        return OA_RUNTIME_EINVAL;
    }
    oa_event source{};
    openarm::runtime::control_init(source);
    const oa_control_status lower = oa_controller_poll_event(
        pinned->controller, wait_timeout_ns == 0U ? 0U : now + wait_timeout_ns, &source);
    status = openarm::runtime::map_control(lower);
    if (status != OA_RUNTIME_OK) return status;
    oa_runtime_event result{};
    openarm::runtime::runtime_init(result);
    result.kind = source.kind;
    result.source_facility = OA_RUNTIME_FACILITY_CONTROL;
    result.status = openarm::runtime::map_control(source.cause);
    result.source_status = source.cause;
    result.clock_id = OA_RUNTIME_CLOCK_MONOTONIC;
    result.event_runtime_monotonic_ns = source.t_ns;
    result.manifest_revision = pinned->manifest->config.manifest_revision;
    result.inventory_revision = pinned->inventory_revision;
    result.calibration_revision = pinned->calibration_revision;
    result.model_revision = pinned->manifest->config.model_revision;
    result.scene_revision = pinned->options.collision_scene_revision;
    result.feedback_seq[0] = source.feedback_seq;
    result.feedback_seq[1] = source.feedback_seq;
    result.command_id = source.command_id;
    result.lifecycle = source.lifecycle;
    result.collision_checked = 0U;
    result.motion_authorized = 0U;
    result.capabilities = openarm::runtime::capabilities_for(pinned->options.backend);
    *out_event = result;
    return OA_RUNTIME_OK;
}

extern "C" void oa_runtime_plan_destroy(oa_runtime_plan *plan) {
    openarm::runtime::plans.erase(plan);
}
