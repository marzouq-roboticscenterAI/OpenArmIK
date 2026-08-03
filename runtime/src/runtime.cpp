/* SPDX-License-Identifier: Apache-2.0 */
#include "runtime_internal.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <limits>
#include <new>
#include <unistd.h>

namespace openarm::runtime {

#ifdef OA_RUNTIME_ENABLE_TEST_HOOKS
namespace {
std::atomic<std::int64_t> allocation_failure_countdown{-1};
}

void allocation_checkpoint() {
    std::int64_t value = allocation_failure_countdown.load(std::memory_order_relaxed);
    while (value >= 0) {
        if (allocation_failure_countdown.compare_exchange_weak(
                value, value - 1, std::memory_order_relaxed)) {
            if (value == 0) throw std::bad_alloc();
            return;
        }
    }
}
#endif

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
    case OA_CONTROL_EUNREACHABLE: return OA_RUNTIME_EUNREACHABLE;
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

oa_runtime_status record_error(const std::shared_ptr<RuntimeData> &runtime,
                               oa_runtime_status status,
                               oa_runtime_facility facility,
                               std::uint32_t lower_code,
                               std::uint32_t system_error) {
    if (runtime && status != OA_RUNTIME_OK) {
        set_error(runtime, status, facility, lower_code, system_error);
    }
    return status;
}

oa_runtime_capability capabilities_for(oa_runtime_backend backend) {
    const oa_runtime_capability common = OA_RUNTIME_CAP_MANIFEST_PREVIEW |
        OA_RUNTIME_CAP_MANIFEST_PERSISTENCE;
    if (backend == OA_RUNTIME_BACKEND_VIRTUAL) {
        return common | OA_RUNTIME_CAP_INTERFACE_ENUMERATION |
            OA_RUNTIME_CAP_VIRTUAL_COORDINATES | OA_RUNTIME_CAP_VIRTUAL_JOINT_MOTION |
            OA_RUNTIME_CAP_VIRTUAL_PAIRED_XYZ_MOTION |
            OA_RUNTIME_CAP_VIRTUAL_MANUAL_CALIBRATION |
            OA_RUNTIME_CAP_VIRTUAL_SUPERVISED_CALIBRATION;
    }
    if (backend == OA_RUNTIME_BACKEND_SOCKETCAN_QUERY) {
        return common | OA_RUNTIME_CAP_INTERFACE_ENUMERATION;
    }
    return common;
}

oa_runtime_collision_policy collision_policy_for(const RuntimeData &runtime) {
    return runtime.options.backend == OA_RUNTIME_BACKEND_VIRTUAL &&
                   runtime.options.allow_unchecked_virtual_motion != 0U
               ? OA_RUNTIME_COLLISION_VIRTUAL_UNCHECKED
               : OA_RUNTIME_COLLISION_REJECT_ALL;
}

std::string coordinate_identity_for(const ManifestData &manifest,
                                    oa_runtime_collision_policy collision_policy,
                                    std::uint64_t collision_scene_revision) {
    std::string identity = "openarm-runtime-coordinate-v1\n";
    identity += std::to_string(manifest.config.model_revision);
    identity += '\n';
    for (std::uint32_t side = 0U; side < OA_RUNTIME_ARMS; ++side) {
        const oa_model *const model = side == 0U ? oa_model_left_v10_bimanual()
                                                 : oa_model_right_v10_bimanual();
        identity += oa_model_id(model);
        identity += '\n';
        identity += oa_model_data_sha256(model);
        identity += '\n';
        identity += oa_model_flattened_urdf_sha256(model);
        identity += '\n';
        identity += oa_model_source_sha256(model);
        identity += '\n';
        identity += oa_model_tip_frame(model);
        identity += '\n';
    }
    identity += std::to_string(OA_RUNTIME_FRAME_OPENARM_BODY_LINK0);
    identity += '\n';
    identity += std::to_string(OA_RUNTIME_UNITS_SI_V1);
    identity += '\n';
    identity += std::to_string(OA_RUNTIME_ORIENTATION_FREE);
    identity += '\n';
    identity += std::to_string(collision_policy);
    identity += '\n';
    identity += std::to_string(collision_scene_revision);
    identity += '\n';
    return sha256_hex(identity);
}

oa_runtime_status fill_model_identity(const RuntimeData &runtime, std::uint32_t side,
                                      oa_runtime_model_identity &target) {
    if (side >= OA_RUNTIME_ARMS) return OA_RUNTIME_EINVAL;
    const oa_model *const model = side == 0U ? oa_model_left_v10_bimanual()
                                             : oa_model_right_v10_bimanual();
    if (model == nullptr) return OA_RUNTIME_ESTATE;
    runtime_init(target);
    target.side = side;
    target.xyz_frame_id = OA_RUNTIME_FRAME_OPENARM_BODY_LINK0;
    target.units_id = OA_RUNTIME_UNITS_SI_V1;
    target.orientation_policy = OA_RUNTIME_ORIENTATION_FREE;
    target.collision_policy = collision_policy_for(runtime);
    target.model_revision = runtime.manifest->config.model_revision;
    target.tcp_revision = runtime.manifest->config.model_revision;
    target.collision_scene_revision = runtime.options.collision_scene_revision;
    std::snprintf(target.model_id, sizeof(target.model_id), "%s", oa_model_id(model));
    std::snprintf(target.provenance, sizeof(target.provenance), "%s",
                  oa_model_provenance(model));
    std::snprintf(target.model_data_sha256, sizeof(target.model_data_sha256), "%s",
                  oa_model_data_sha256(model));
    std::snprintf(target.flattened_urdf_sha256,
                  sizeof(target.flattened_urdf_sha256), "%s",
                  oa_model_flattened_urdf_sha256(model));
    std::snprintf(target.source_sha256, sizeof(target.source_sha256), "%s",
                  oa_model_source_sha256(model));
    std::snprintf(target.tcp_frame, sizeof(target.tcp_frame), "%s",
                  oa_model_tip_frame(model));
    std::snprintf(target.coordinate_identity_sha256,
                  sizeof(target.coordinate_identity_sha256), "%s",
                  runtime.coordinate_identity_digest.c_str());
    return OA_RUNTIME_OK;
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
    if (runtime) {
        std::lock_guard<std::mutex> lock(runtime->mutex);
        if (holds_authority) {
            holds_authority = false;
            if (runtime->plan_pending &&
                runtime->plan_authority_id == authority_id) {
                runtime->plan_pending = false;
                runtime->plan_expiry_ns = 0U;
                runtime->wake.notify_all();
            }
        }
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

PersistenceAuthorityData::~PersistenceAuthorityData() {
    if (directory_fd >= 0) close(directory_fd);
    authentication_key.fill(0U);
}

namespace {

void capture_feedback_clock(RuntimeData &runtime, const oa_snapshot &snapshot,
                            const std::uint64_t controller_now,
                            const std::uint64_t facade_now) {
    for (std::size_t side = 0U; side < OA_RUNTIME_ARMS; ++side) {
        const oa_arm_snapshot &arm = snapshot.arm[side];
        RuntimeData::FeedbackClockEvidence &evidence = runtime.feedback_clock[side];
        if (arm.feedback_seq == 0U || arm.feedback_seq == evidence.feedback_seq) continue;
        const std::uint64_t lag = arm.t_ns <= controller_now
                                      ? controller_now - arm.t_ns
                                      : 0U;
        std::uint64_t translated = lag <= facade_now ? facade_now - lag : 1U;
        if (translated <= evidence.runtime_monotonic_ns) {
            translated = evidence.runtime_monotonic_ns == UINT64_MAX
                             ? UINT64_MAX
                             : evidence.runtime_monotonic_ns + 1U;
        }
        evidence.feedback_seq = arm.feedback_seq;
        evidence.controller_monotonic_ns = arm.t_ns;
        evidence.runtime_monotonic_ns = translated;
    }
}

void drain_controller_events(RuntimeData &runtime, const std::uint64_t facade_now) {
    for (;;) {
        oa_event source{};
        control_init(source);
        if (oa_controller_poll_event(runtime.controller, 0U, &source) != OA_CONTROL_OK) {
            return;
        }
        const std::uint64_t controller_now =
            runtime.controller_timeline_ns.load(std::memory_order_acquire);
        const std::uint64_t lag = source.t_ns <= controller_now
                                      ? controller_now - source.t_ns
                                      : 0U;
        const std::uint64_t translated = lag <= facade_now ? facade_now - lag : 1U;
        std::lock_guard<std::mutex> lock(runtime.mutex);
        if (runtime.event_count == runtime.events.size()) {
            runtime.event_head = (runtime.event_head + 1U) % runtime.events.size();
            --runtime.event_count;
        }
        const std::size_t tail =
            (runtime.event_head + runtime.event_count) % runtime.events.size();
        runtime.events[tail] = {source, translated};
        ++runtime.event_count;
        runtime.wake.notify_all();
    }
}

} // namespace

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
        to.measurement_runtime_monotonic_ns = 0U;
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

oa_runtime_status capture_runtime_snapshot(
    const std::shared_ptr<RuntimeData> &runtime, oa_snapshot &source,
    oa_runtime_snapshot &target) {
    std::lock_guard<std::mutex> controller_lock(runtime->controller_mutex);
    control_init(source);
    const oa_control_status lower = oa_controller_snapshot(runtime->controller, &source);
    const oa_runtime_status status = map_control(lower);
    if (status != OA_RUNTIME_OK) {
        return record_error(runtime, status, OA_RUNTIME_FACILITY_CONTROL, lower);
    }
    fill_snapshot(source, target);
    const std::uint64_t facade_now = now_ns();
    for (std::size_t side = 0U; side < OA_RUNTIME_ARMS; ++side) {
        const RuntimeData::FeedbackClockEvidence &evidence = runtime->feedback_clock[side];
        oa_runtime_arm_snapshot &arm = target.arm[side];
        if (evidence.feedback_seq != arm.feedback_seq ||
            evidence.runtime_monotonic_ns == 0U ||
            evidence.runtime_monotonic_ns > facade_now) {
            arm.measurement_runtime_monotonic_ns = 0U;
            arm.fresh_mask = 0U;
            continue;
        }
        arm.measurement_runtime_monotonic_ns = evidence.runtime_monotonic_ns;
        if (facade_now - evidence.runtime_monotonic_ns >
            runtime->options.feedback_timeout_ns) {
            arm.fresh_mask = 0U;
        }
    }
    return OA_RUNTIME_OK;
}

}

namespace {

oa_runtime_status runtime_options_status(const oa_runtime_options *options) {
    if (options == nullptr || options->struct_size < sizeof(*options) ||
        options->abi_version != OA_RUNTIME_ABI_VERSION) {
        return OA_RUNTIME_EABI;
    }
    return options->backend >= OA_RUNTIME_BACKEND_VIRTUAL &&
           options->backend <= OA_RUNTIME_BACKEND_OFFLINE &&
           options->allow_unchecked_virtual_motion <= 1U &&
           options->cycle_ns >= 1000000U && options->cycle_ns <= 1000000000U &&
           options->feedback_timeout_ns >= options->cycle_ns &&
           options->maximum_cross_bus_skew_ns > 0U
               ? OA_RUNTIME_OK
               : OA_RUNTIME_EINVAL;
}

oa_runtime_status require_runtime(const oa_runtime *handle,
                                  std::shared_ptr<openarm::runtime::RuntimeData> &out) {
    if (!openarm::runtime::process_guard_ok()) return OA_RUNTIME_ESTATE;
    out = openarm::runtime::runtimes.pin(handle);
    if (!out) return OA_RUNTIME_EINVAL;
    std::lock_guard<std::mutex> lock(out->mutex);
    return out->closing ? OA_RUNTIME_ESTATE : OA_RUNTIME_OK;
}

bool translate_future_deadline(std::uint64_t facade_deadline,
                               std::uint64_t facade_now,
                               std::uint64_t controller_now,
                               std::uint64_t &controller_deadline) {
    if (facade_deadline <= facade_now) return false;
    const std::uint64_t remaining = facade_deadline - facade_now;
    if (remaining > UINT64_MAX - controller_now) return false;
    controller_deadline = controller_now + remaining;
    return true;
}

}

#ifdef OA_RUNTIME_ENABLE_TEST_HOOKS
extern "C" void oa_runtime_test_fail_allocation_after(std::int64_t countdown) {
    openarm::runtime::allocation_failure_countdown.store(countdown,
                                                         std::memory_order_relaxed);
}
#endif

extern "C" oa_runtime_status oa_runtime_create(
    const oa_runtime_options *options, const oa_runtime_manifest *manifest,
    oa_runtime **out_runtime) {
    if (!openarm::runtime::process_guard_ok()) return OA_RUNTIME_ESTATE;
    if (out_runtime == nullptr) return OA_RUNTIME_EINVAL;
    *out_runtime = nullptr;
    const oa_runtime_status options_status = runtime_options_status(options);
    if (options_status != OA_RUNTIME_OK) return options_status;
    const auto manifest_data = openarm::runtime::manifests.pin(manifest);
    if (!manifest_data) return OA_RUNTIME_EINVAL;
    if (options->backend == OA_RUNTIME_BACKEND_VIRTUAL &&
        (manifest_data->intended_backend != OA_RUNTIME_BACKEND_VIRTUAL ||
         manifest_data->state != OA_RUNTIME_MANIFEST_ARMABLE)) {
        return OA_RUNTIME_EIDENTITY;
    }
    if (options->backend == OA_RUNTIME_BACKEND_VIRTUAL &&
        manifest_data->loaded_from_file && !manifest_data->checkpoint_authorized) {
        return OA_RUNTIME_EPERMISSION;
    }
    try {
        auto value = std::make_shared<openarm::runtime::RuntimeData>();
        value->options = *options;
        value->manifest = manifest_data;
        value->timeline_ns.store(openarm::runtime::now_ns(), std::memory_order_release);
        value->controller_timeline_ns.store(
            value->timeline_ns.load(std::memory_order_acquire), std::memory_order_release);
        value->coordinate_identity_digest = openarm::runtime::coordinate_identity_for(
            *manifest_data, openarm::runtime::collision_policy_for(*value),
            options->collision_scene_revision);
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
            lower = oa_controller_advance(
                value->controller,
                value->controller_timeline_ns.load(std::memory_order_acquire));
            if (lower != OA_CONTROL_OK) return openarm::runtime::map_control(lower);
            {
                std::lock_guard<std::mutex> controller_lock(value->controller_mutex);
                oa_snapshot initial{};
                openarm::runtime::control_init(initial);
                lower = oa_controller_snapshot(value->controller, &initial);
                if (lower != OA_CONTROL_OK) return openarm::runtime::map_control(lower);
                const std::uint64_t facade_now = openarm::runtime::now_ns();
                openarm::runtime::capture_feedback_clock(
                    *value, initial,
                    value->controller_timeline_ns.load(std::memory_order_acquire),
                    facade_now);
                openarm::runtime::drain_controller_events(*value, facade_now);
            }
            openarm::runtime::RuntimeData *const raw = value.get();
            value->worker = std::thread([raw] {
                std::unique_lock<std::mutex> lock(raw->mutex);
                while (!raw->closing) {
                    const std::uint64_t cycle = raw->options.cycle_ns;
                    raw->wake.wait_for(lock, std::chrono::nanoseconds(cycle),
                                       [raw] { return raw->closing; });
                    if (raw->closing) break;
                    if (raw->plan_pending) {
                        raw->wake.wait_for(lock, std::chrono::nanoseconds(cycle),
                                           [raw] { return raw->closing ||
                                                          !raw->plan_pending; });
                        if (raw->plan_pending && raw->plan_expiry_ns != 0U &&
                            openarm::runtime::now_ns() >= raw->plan_expiry_ns) {
                            raw->plan_pending = false;
                            raw->plan_expiry_ns = 0U;
                            raw->wake.notify_all();
                        }
                        continue;
                    }
                    lock.unlock();
                    const std::uint64_t current =
                        raw->controller_timeline_ns.load(std::memory_order_acquire);
                    const std::uint64_t next =
                        current > UINT64_MAX - cycle ? UINT64_MAX : current + cycle;
                    oa_control_status status = OA_CONTROL_OK;
                    const std::uint64_t host_now = openarm::runtime::now_ns();
                    {
                        std::lock_guard<std::mutex> controller_lock(
                            raw->controller_mutex);
                        status = oa_controller_advance(raw->controller, next);
                        raw->controller_timeline_ns.store(next,
                                                          std::memory_order_release);
                        oa_snapshot observed_snapshot{};
                        openarm::runtime::control_init(observed_snapshot);
                        if (oa_controller_snapshot(raw->controller,
                                                   &observed_snapshot) ==
                            OA_CONTROL_OK) {
                            openarm::runtime::capture_feedback_clock(
                                *raw, observed_snapshot, next, host_now);
                        }
                        openarm::runtime::drain_controller_events(*raw, host_now);
                    }
                    std::uint64_t observed =
                        raw->timeline_ns.load(std::memory_order_acquire);
                    while (observed < host_now &&
                           !raw->timeline_ns.compare_exchange_weak(
                               observed, host_now, std::memory_order_release,
                               std::memory_order_acquire)) {}
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
    if (!openarm::runtime::process_guard_ok()) return;
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
    result.collision_policy = openarm::runtime::collision_policy_for(*pinned);
    result.collision_checked = 0U;
    result.model_revision = pinned->manifest->config.model_revision;
    result.capabilities = openarm::runtime::capabilities_for(pinned->options.backend);
    std::snprintf(result.coordinate_identity_sha256,
                  sizeof(result.coordinate_identity_sha256), "%s",
                  pinned->coordinate_identity_digest.c_str());
    *out_report = result;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_get_model_identity(
    const oa_runtime *runtime, std::uint32_t side,
    oa_runtime_model_identity *out_identity) {
    if (!openarm::runtime::output_valid(out_identity)) return OA_RUNTIME_EABI;
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    const oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (side >= OA_RUNTIME_ARMS) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    oa_runtime_model_identity result{};
    const oa_runtime_status identity_status =
        openarm::runtime::fill_model_identity(*pinned, side, result);
    if (identity_status != OA_RUNTIME_OK) {
        return openarm::runtime::record_error(
            pinned, identity_status, OA_RUNTIME_FACILITY_MODEL);
    }
    *out_identity = result;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_now_monotonic_ns(
    const oa_runtime *runtime, oa_runtime_clock_id clock_id, std::uint64_t *out_now_ns) {
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    const oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (out_now_ns == nullptr || clock_id != OA_RUNTIME_CLOCK_MONOTONIC) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    const std::uint64_t host_now = openarm::runtime::now_ns();
    std::uint64_t observed = pinned->timeline_ns.load(std::memory_order_acquire);
    while (observed < host_now &&
           !pinned->timeline_ns.compare_exchange_weak(
               observed, host_now, std::memory_order_release,
               std::memory_order_acquire)) {}
    *out_now_ns = std::max(observed, host_now);
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
    if (pinned->controller == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
    }
    oa_snapshot snapshot{};
    oa_runtime_snapshot result{};
    status = openarm::runtime::capture_runtime_snapshot(pinned, snapshot, result);
    if (status != OA_RUNTIME_OK) return status;
    std::snprintf(result.coordinate_identity_sha256,
                  sizeof(result.coordinate_identity_sha256), "%s",
                  pinned->coordinate_identity_digest.c_str());
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
    if (side >= OA_RUNTIME_ARMS) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    if (pinned->controller == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
    }
    oa_arm_kinematics kinematics{};
    openarm::runtime::control_init(kinematics);
    const oa_control_status lower = oa_controller_get_kinematics(
        pinned->controller, side, required_feedback_seq, &kinematics);
    status = openarm::runtime::map_control(lower);
    if (status != OA_RUNTIME_OK) {
        return openarm::runtime::record_error(
            pinned, status, OA_RUNTIME_FACILITY_CONTROL, lower);
    }
    oa_runtime_kinematics result{};
    openarm::runtime::runtime_init(result);
    result.frame_id = OA_RUNTIME_FRAME_OPENARM_BODY_LINK0;
    result.units_id = OA_RUNTIME_UNITS_SI_V1;
    result.orientation_policy = OA_RUNTIME_ORIENTATION_FREE;
    result.side = side;
    result.feedback_seq = kinematics.feedback_seq;
    oa_runtime_model_identity identity{};
    openarm::runtime::fill_model_identity(*pinned, side, identity);
    result.model_revision = identity.model_revision;
    result.tcp_revision = identity.tcp_revision;
    result.collision_policy = identity.collision_policy;
    std::snprintf(result.model_id, sizeof(result.model_id), "%s", identity.model_id);
    std::snprintf(result.tcp_frame, sizeof(result.tcp_frame), "%s", identity.tcp_frame);
    std::snprintf(result.model_data_sha256, sizeof(result.model_data_sha256), "%s",
                  identity.model_data_sha256);
    std::snprintf(result.flattened_urdf_sha256,
                  sizeof(result.flattened_urdf_sha256), "%s",
                  identity.flattened_urdf_sha256);
    std::snprintf(result.coordinate_identity_sha256,
                  sizeof(result.coordinate_identity_sha256), "%s",
                  identity.coordinate_identity_sha256);
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
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (estop_active > 1U || deadman_active > 1U) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    if (pinned->controller == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
    }
    {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        pinned->estop_active = estop_active != 0U;
        pinned->deadman_active = deadman_active != 0U;
    }
    const oa_control_status lower = oa_controller_set_interlock(
        pinned->controller, estop_active, deadman_active);
    return openarm::runtime::record_error(
        pinned, openarm::runtime::map_control(lower),
        OA_RUNTIME_FACILITY_CONTROL, lower);
}

extern "C" oa_runtime_status oa_runtime_arm_virtual(oa_runtime *runtime) {
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (pinned->controller == nullptr || pinned->options.backend != OA_RUNTIME_BACKEND_VIRTUAL) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
    }
    std::unique_lock<std::mutex> lock(pinned->mutex);
    if (pinned->owner != 0U) {
        openarm::runtime::runtime_init(pinned->last_error);
        pinned->last_error.status = OA_RUNTIME_EBUSY;
        pinned->last_error.facility = OA_RUNTIME_FACILITY_RUNTIME;
        return OA_RUNTIME_EBUSY;
    }
    if (pinned->estop_active || !pinned->deadman_active) {
        openarm::runtime::runtime_init(pinned->last_error);
        pinned->last_error.status = OA_RUNTIME_EFAULT;
        pinned->last_error.facility = OA_RUNTIME_FACILITY_RUNTIME;
        return OA_RUNTIME_EFAULT;
    }
    oa_arm_challenge challenge{};
    openarm::runtime::control_init(challenge);
    oa_control_status lower = oa_controller_get_arm_challenge(pinned->controller, &challenge);
    if (lower == OA_CONTROL_OK) lower = oa_controller_arm(pinned->controller, &challenge);
    status = openarm::runtime::map_control(lower);
    if (status == OA_RUNTIME_OK) {
        pinned->owner = 1U;
    } else {
        openarm::runtime::runtime_init(pinned->last_error);
        pinned->last_error.status = status;
        pinned->last_error.facility = OA_RUNTIME_FACILITY_CONTROL;
        pinned->last_error.lower_code = lower;
    }
    return status;
}

extern "C" oa_runtime_status oa_runtime_plan_joint(
    oa_runtime *runtime, const oa_runtime_joint_move *request,
    oa_runtime_plan **out_plan) {
    if (request == nullptr || request->struct_size < sizeof(*request) ||
        request->abi_version != OA_RUNTIME_ABI_VERSION) return OA_RUNTIME_EABI;
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (out_plan == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    *out_plan = nullptr;
    if (request->clock_id != OA_RUNTIME_CLOCK_MONOTONIC ||
        request->units_id != OA_RUNTIME_UNITS_SI_V1 ||
        request->side >= OA_RUNTIME_ARMS || request->joint >= OA_RUNTIME_DOF ||
        request->required_collision_policy !=
            openarm::runtime::collision_policy_for(*pinned)) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    if (request->required_model_revision != pinned->manifest->config.model_revision ||
        request->required_tcp_revision != pinned->manifest->config.model_revision ||
        request->collision_scene_revision !=
            pinned->options.collision_scene_revision ||
        std::strncmp(request->required_coordinate_identity_sha256,
                     pinned->coordinate_identity_digest.c_str(),
                     OA_RUNTIME_DIGEST_CAPACITY) != 0) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EIDENTITY, OA_RUNTIME_FACILITY_MODEL);
    }
    if (pinned->controller == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
    }
    std::shared_ptr<openarm::runtime::PlanData> plan;
    try {
        plan = std::make_shared<openarm::runtime::PlanData>();
    } catch (...) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_ENOMEM, OA_RUNTIME_FACILITY_RUNTIME);
    }
    plan->runtime = pinned;
    {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        if (pinned->owner != 1U || pinned->plan_pending) {
            openarm::runtime::runtime_init(pinned->last_error);
            pinned->last_error.status = OA_RUNTIME_EBUSY;
            pinned->last_error.facility = OA_RUNTIME_FACILITY_RUNTIME;
            return OA_RUNTIME_EBUSY;
        }
        pinned->plan_pending = true;
        pinned->plan_expiry_ns = request->expiry_runtime_monotonic_ns;
        pinned->plan_authority_id = pinned->next_plan_authority_id++;
        if (pinned->next_plan_authority_id == 0U) pinned->next_plan_authority_id = 1U;
        plan->authority_id = pinned->plan_authority_id;
        plan->holds_authority = true;
    }
    oa_joint_move move{};
    openarm::runtime::control_init(move);
    const std::uint64_t facade_now = openarm::runtime::now_ns();
    const std::uint64_t controller_now =
        pinned->controller_timeline_ns.load(std::memory_order_acquire);
    if (!translate_future_deadline(request->expiry_runtime_monotonic_ns,
                                   facade_now, controller_now, move.expiry_ns)) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_ETIMEOUT, OA_RUNTIME_FACILITY_RUNTIME);
    }
    plan->facade_expiry_ns = request->expiry_runtime_monotonic_ns;
    move.required_feedback_seq = request->required_feedback_seq;
    move.side = request->side; move.joint = request->joint;
    move.target_rad = request->target_model_rad;
    move.velocity_scale = request->velocity_scale;
    move.acceleration_scale = request->acceleration_scale;
    move.jerk_scale = request->jerk_scale;
    move.position_tol_rad = request->position_tolerance_rad;
    move.velocity_tol_rad_s = request->velocity_tolerance_rad_s;
    oa_control_status lower = OA_CONTROL_OK;
    {
        /* A public snapshot can become stale before this planner acquires the
         * controller. Keep the runtime's sequence check and lower-level plan
         * construction in one controller epoch so a valid stale race remains
         * ESTALE rather than leaking the lower EINVAL mismatch. */
        std::lock_guard<std::mutex> controller_lock(pinned->controller_mutex);
        oa_snapshot current{};
        openarm::runtime::control_init(current);
        lower = oa_controller_snapshot(pinned->controller, &current);
        if (lower == OA_CONTROL_OK &&
            request->required_feedback_seq != current.arm[request->side].feedback_seq) {
            return openarm::runtime::record_error(
                pinned, OA_RUNTIME_ESTALE, OA_RUNTIME_FACILITY_RUNTIME);
        }
        if (lower == OA_CONTROL_OK) {
            lower = oa_controller_plan_joint(pinned->controller, &move, &plan->plan);
        }
    }
    if (lower != OA_CONTROL_OK) {
        return openarm::runtime::record_error(
            pinned, openarm::runtime::map_control(lower),
            OA_RUNTIME_FACILITY_CONTROL, lower);
    }
    status = openarm::runtime::map_control(lower);
    oa_runtime_plan *const handle = openarm::runtime::plans.insert(plan);
    if (handle == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_ENOMEM, OA_RUNTIME_FACILITY_RUNTIME);
    }
    *out_plan = handle;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_plan_paired_tcp_body(
    oa_runtime *runtime, const oa_runtime_paired_tcp_move *request,
    oa_runtime_plan **out_plan) {
    if (request == nullptr || request->struct_size < sizeof(*request) ||
        request->abi_version != OA_RUNTIME_ABI_VERSION) return OA_RUNTIME_EABI;
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (out_plan == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    *out_plan = nullptr;
    if (request->clock_id != OA_RUNTIME_CLOCK_MONOTONIC ||
        request->units_id != OA_RUNTIME_UNITS_SI_V1 ||
        request->frame_id != OA_RUNTIME_FRAME_OPENARM_BODY_LINK0 ||
        request->orientation_policy != OA_RUNTIME_ORIENTATION_FREE ||
        request->required_collision_policy !=
            openarm::runtime::collision_policy_for(*pinned)) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    if (request->required_model_revision != pinned->manifest->config.model_revision ||
        request->required_tcp_revision[0] != pinned->manifest->config.model_revision ||
        request->required_tcp_revision[1] != pinned->manifest->config.model_revision) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EIDENTITY, OA_RUNTIME_FACILITY_MODEL);
    }
    if (std::strncmp(request->required_coordinate_identity_sha256,
                     pinned->coordinate_identity_digest.c_str(),
                     OA_RUNTIME_DIGEST_CAPACITY) != 0) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EIDENTITY, OA_RUNTIME_FACILITY_MODEL);
    }
    if (pinned->controller == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
    }
    std::shared_ptr<openarm::runtime::PlanData> plan;
    try {
        plan = std::make_shared<openarm::runtime::PlanData>();
    } catch (...) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_ENOMEM, OA_RUNTIME_FACILITY_RUNTIME);
    }
    plan->runtime = pinned;
    {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        if (pinned->owner != 1U || pinned->plan_pending) {
            openarm::runtime::runtime_init(pinned->last_error);
            pinned->last_error.status = OA_RUNTIME_EBUSY;
            pinned->last_error.facility = OA_RUNTIME_FACILITY_RUNTIME;
            return OA_RUNTIME_EBUSY;
        }
        pinned->plan_pending = true;
        pinned->plan_expiry_ns = request->expiry_runtime_monotonic_ns;
        pinned->plan_authority_id = pinned->next_plan_authority_id++;
        if (pinned->next_plan_authority_id == 0U) pinned->next_plan_authority_id = 1U;
        plan->authority_id = pinned->plan_authority_id;
        plan->holds_authority = true;
    }
    oa_paired_tcp_move move{};
    openarm::runtime::control_init(move);
    const std::uint64_t facade_now = openarm::runtime::now_ns();
    const std::uint64_t controller_now =
        pinned->controller_timeline_ns.load(std::memory_order_acquire);
    if (!translate_future_deadline(request->expiry_runtime_monotonic_ns,
                                   facade_now, controller_now, move.expiry_ns)) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_ETIMEOUT, OA_RUNTIME_FACILITY_RUNTIME);
    }
    plan->facade_expiry_ns = request->expiry_runtime_monotonic_ns;
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
    oa_control_status lower = OA_CONTROL_OK;
    {
        std::lock_guard<std::mutex> controller_lock(pinned->controller_mutex);
        oa_snapshot current{};
        openarm::runtime::control_init(current);
        lower = oa_controller_snapshot(pinned->controller, &current);
        if (lower == OA_CONTROL_OK &&
            (request->required_feedback_seq[0] != current.arm[0].feedback_seq ||
             request->required_feedback_seq[1] != current.arm[1].feedback_seq)) {
            return openarm::runtime::record_error(
                pinned, OA_RUNTIME_ESTALE, OA_RUNTIME_FACILITY_RUNTIME);
        }
        if (lower == OA_CONTROL_OK) {
            lower = oa_controller_plan_paired_tcp(pinned->controller, &move, &plan->plan);
        }
    }
    if (lower != OA_CONTROL_OK) {
        return openarm::runtime::record_error(
            pinned, openarm::runtime::map_control(lower),
            OA_RUNTIME_FACILITY_CONTROL, lower);
    }
    status = openarm::runtime::map_control(lower);
    oa_runtime_plan *const handle = openarm::runtime::plans.insert(plan);
    if (handle == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_ENOMEM, OA_RUNTIME_FACILITY_RUNTIME);
    }
    *out_plan = handle;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_plan_get_report(
    const oa_runtime_plan *plan, oa_runtime_plan_report *out_report) {
    if (!openarm::runtime::output_valid(out_report)) return OA_RUNTIME_EABI;
    const auto pinned = openarm::runtime::plans.pin(plan);
    if (!pinned) return OA_RUNTIME_EINVAL;
    std::lock_guard<std::mutex> plan_lock(pinned->mutex);
    oa_motion_plan_report source{};
    openarm::runtime::control_init(source);
    const oa_control_status lower = oa_motion_plan_get_report(pinned->plan, &source);
    if (lower != OA_CONTROL_OK) {
        return openarm::runtime::record_error(
            pinned->runtime, openarm::runtime::map_control(lower),
            OA_RUNTIME_FACILITY_CONTROL, lower);
    }
    oa_runtime_plan_report result{};
    openarm::runtime::runtime_init(result);
    result.kind = source.kind; result.collision_checked = source.collision_checked;
    result.motion_authorized = 0U;
    result.frame_id = OA_RUNTIME_FRAME_OPENARM_BODY_LINK0;
    result.units_id = OA_RUNTIME_UNITS_SI_V1;
    std::copy_n(source.seed_feedback_seq, 2U, result.seed_feedback_seq);
    result.duration_ns = source.duration_ns; result.manifest_revision = source.manifest_revision;
    result.model_revision = source.model_revision;
    result.tcp_revision[0] = pinned->runtime->manifest->config.model_revision;
    result.tcp_revision[1] = pinned->runtime->manifest->config.model_revision;
    result.collision_scene_revision = source.collision_scene_revision;
    result.collision_policy = openarm::runtime::collision_policy_for(*pinned->runtime);
    std::snprintf(result.coordinate_identity_sha256,
                  sizeof(result.coordinate_identity_sha256), "%s",
                  pinned->runtime->coordinate_identity_digest.c_str());
    std::copy_n(&source.target_q[0][0], 14U, &result.target_q_model_rad[0][0]);
    std::copy_n(&source.achieved_tcp_m[0][0], 6U, &result.achieved_tcp_m[0][0]);
    std::copy_n(source.tcp_residual_m, 2U, result.tcp_residual_m);
    *out_report = result;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_execute(
    oa_runtime *runtime, const oa_runtime_plan *plan,
    const oa_runtime_execute_request *request, std::uint64_t *out_command_id) {
    if (request == nullptr || request->struct_size < sizeof(*request) ||
        request->abi_version != OA_RUNTIME_ABI_VERSION) return OA_RUNTIME_EABI;
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (out_command_id == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    *out_command_id = 0U;
    if (request->clock_id != OA_RUNTIME_CLOCK_MONOTONIC) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    const auto plan_data = openarm::runtime::plans.pin(plan);
    if (!plan_data || plan_data->runtime.get() != pinned.get()) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    if (pinned->controller == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
    }
    std::lock_guard<std::mutex> plan_lock(plan_data->mutex);
    {
        std::lock_guard<std::mutex> runtime_lock(pinned->mutex);
        if (!plan_data->holds_authority || !pinned->plan_pending ||
            pinned->plan_authority_id != plan_data->authority_id) {
            openarm::runtime::runtime_init(pinned->last_error);
            pinned->last_error.status = OA_RUNTIME_ESTALE;
            pinned->last_error.facility = OA_RUNTIME_FACILITY_RUNTIME;
            return OA_RUNTIME_ESTALE;
        }
    }
    const std::uint64_t facade_now = openarm::runtime::now_ns();
    if (plan_data->facade_expiry_ns <= facade_now) {
        std::lock_guard<std::mutex> runtime_lock(pinned->mutex);
        plan_data->holds_authority = false;
        if (pinned->plan_pending &&
            pinned->plan_authority_id == plan_data->authority_id) {
            pinned->plan_pending = false;
            pinned->plan_expiry_ns = 0U;
            pinned->wake.notify_all();
        }
        openarm::runtime::runtime_init(pinned->last_error);
        pinned->last_error.status = OA_RUNTIME_ETIMEOUT;
        pinned->last_error.facility = OA_RUNTIME_FACILITY_RUNTIME;
        return OA_RUNTIME_ETIMEOUT;
    }
    const std::uint64_t effective_start =
        request->start_runtime_monotonic_ns == 0U
            ? facade_now
            : request->start_runtime_monotonic_ns;
    if (effective_start < facade_now ||
        request->expiry_runtime_monotonic_ns <= effective_start ||
        request->expiry_runtime_monotonic_ns > plan_data->facade_expiry_ns ||
        request->producer_deadline_runtime_monotonic_ns <= facade_now) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    oa_execute_request execute{};
    openarm::runtime::control_init(execute);
    const std::uint64_t controller_now =
        pinned->controller_timeline_ns.load(std::memory_order_acquire);
    if (request->start_runtime_monotonic_ns == 0U) {
        execute.start_ns = 0U;
    } else if (!translate_future_deadline(request->start_runtime_monotonic_ns,
                                          facade_now, controller_now,
                                          execute.start_ns)) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    if (!translate_future_deadline(request->expiry_runtime_monotonic_ns,
                                   facade_now, controller_now, execute.expiry_ns) ||
        !translate_future_deadline(
            request->producer_deadline_runtime_monotonic_ns, facade_now,
            controller_now, execute.producer_deadline_ns)) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    execute.stop_kind = request->stop_kind;
    const oa_control_status lower = oa_controller_execute(
        pinned->controller, plan_data->plan, &execute, out_command_id);
    status = openarm::runtime::map_control(lower);
    if (status == OA_RUNTIME_OK) {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        plan_data->holds_authority = false;
        if (pinned->plan_authority_id == plan_data->authority_id) {
            pinned->plan_pending = false;
            pinned->plan_expiry_ns = 0U;
        }
        pinned->wake.notify_all();
    } else {
        openarm::runtime::record_error(
            pinned, status, OA_RUNTIME_FACILITY_CONTROL, lower);
    }
    return status;
}

extern "C" oa_runtime_status oa_runtime_heartbeat(
    oa_runtime *runtime, std::uint64_t command_id, oa_runtime_clock_id clock_id,
    std::uint64_t producer_deadline_runtime_monotonic_ns) {
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    const oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (clock_id != OA_RUNTIME_CLOCK_MONOTONIC) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    if (pinned->controller == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
    }
    const std::uint64_t facade_now = openarm::runtime::now_ns();
    const std::uint64_t controller_now =
        pinned->controller_timeline_ns.load(std::memory_order_acquire);
    std::uint64_t controller_deadline = 0U;
    if (!translate_future_deadline(producer_deadline_runtime_monotonic_ns,
                                   facade_now, controller_now,
                                   controller_deadline)) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    const oa_control_status lower = oa_controller_heartbeat(
        pinned->controller, command_id, controller_deadline);
    return openarm::runtime::record_error(
        pinned, openarm::runtime::map_control(lower),
        OA_RUNTIME_FACILITY_CONTROL, lower);
}

extern "C" oa_runtime_status oa_runtime_stop(oa_runtime *runtime, std::uint32_t stop_kind) {
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    const oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (pinned->controller == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
    }
    const oa_control_status lower = oa_controller_stop(pinned->controller, stop_kind);
    return openarm::runtime::record_error(
        pinned, openarm::runtime::map_control(lower),
        OA_RUNTIME_FACILITY_CONTROL, lower);
}

extern "C" oa_runtime_status oa_runtime_disarm(
    oa_runtime *runtime, oa_runtime_clock_id clock_id, std::uint64_t deadline_ns) {
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (clock_id != OA_RUNTIME_CLOCK_MONOTONIC) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    if (pinned->controller == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
    }
    const std::uint64_t facade_now = openarm::runtime::now_ns();
    const std::uint64_t controller_now =
        pinned->controller_timeline_ns.load(std::memory_order_acquire);
    std::uint64_t controller_deadline = 0U;
    if (!translate_future_deadline(deadline_ns, facade_now, controller_now,
                                   controller_deadline)) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    const oa_control_status lower =
        oa_controller_disarm(pinned->controller, controller_deadline);
    status = openarm::runtime::map_control(lower);
    if (status == OA_RUNTIME_OK) {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        if (pinned->owner == 1U) pinned->owner = 0U;
    }
    return openarm::runtime::record_error(
        pinned, status, OA_RUNTIME_FACILITY_CONTROL, lower);
}

extern "C" oa_runtime_status oa_runtime_poll_event(
    oa_runtime *runtime, std::uint64_t wait_timeout_ns, oa_runtime_event *out_event) {
    if (!openarm::runtime::output_valid(out_event)) return OA_RUNTIME_EABI;
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (pinned->controller == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
    }
    if (wait_timeout_ns >
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    openarm::runtime::RuntimeData::EventEvidence evidence{};
    {
        std::unique_lock<std::mutex> lock(pinned->mutex);
        if (pinned->event_count == 0U && wait_timeout_ns != 0U) {
            pinned->wake.wait_for(lock, std::chrono::nanoseconds(wait_timeout_ns),
                                  [&pinned] {
                                      return pinned->closing ||
                                             pinned->event_count != 0U;
                                  });
        }
        if (pinned->event_count == 0U) {
            lock.unlock();
            return openarm::runtime::record_error(
                pinned, OA_RUNTIME_ETIMEOUT, OA_RUNTIME_FACILITY_CONTROL,
                OA_CONTROL_ETIMEOUT);
        }
        evidence = pinned->events[pinned->event_head];
        pinned->event_head = (pinned->event_head + 1U) % pinned->events.size();
        --pinned->event_count;
    }
    const oa_event &source = evidence.source;
    oa_runtime_event result{};
    openarm::runtime::runtime_init(result);
    result.kind = source.kind;
    result.source_facility = OA_RUNTIME_FACILITY_CONTROL;
    result.status = openarm::runtime::map_control(source.cause);
    result.source_status = source.cause;
    result.clock_id = OA_RUNTIME_CLOCK_MONOTONIC;
    result.event_runtime_monotonic_ns = evidence.runtime_monotonic_ns;
    result.manifest_revision = pinned->manifest->config.manifest_revision;
    {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        result.inventory_revision = pinned->inventory_revision;
        result.calibration_revision = pinned->calibration_revision;
    }
    result.model_revision = pinned->manifest->config.model_revision;
    result.scene_revision = pinned->options.collision_scene_revision;
    result.source_feedback_seq = source.feedback_seq;
    result.feedback_seq_valid_mask = 0U;
    result.measurement_timestamp_valid = 0U;
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

/* ---------------------------------------------------------------------------
 * Additive bimanual motion surface (openarm_runtime_motion.h).
 *
 * These live here rather than in a separate translation unit so they reuse the
 * same require_runtime, deadline translation, plan-authority, and identity
 * checks as the frozen V1 planners instead of re-deriving them.
 * ------------------------------------------------------------------------ */

extern "C" oa_runtime_status oa_runtime_plan_converge_tcp_body(
    oa_runtime *runtime, const oa_runtime_converge_tcp_move *request,
    oa_runtime_plan **out_plan) {
    if (request == nullptr || request->struct_size < sizeof(*request) ||
        request->abi_version != OA_RUNTIME_MOTION_ABI_VERSION ||
        request->reserved0 != 0U) {
        return OA_RUNTIME_EABI;
    }
    const oa_runtime_paired_tcp_move &base = request->base;
    if (base.struct_size < sizeof(base) || base.abi_version != OA_RUNTIME_ABI_VERSION) {
        return OA_RUNTIME_EABI;
    }
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (out_plan == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    *out_plan = nullptr;
    if (base.clock_id != OA_RUNTIME_CLOCK_MONOTONIC ||
        base.units_id != OA_RUNTIME_UNITS_SI_V1 ||
        base.frame_id != OA_RUNTIME_FRAME_OPENARM_BODY_LINK0 ||
        base.orientation_policy != OA_RUNTIME_ORIENTATION_FREE ||
        base.required_collision_policy !=
            openarm::runtime::collision_policy_for(*pinned)) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    if (base.required_model_revision != pinned->manifest->config.model_revision ||
        base.required_tcp_revision[0] != pinned->manifest->config.model_revision ||
        base.required_tcp_revision[1] != pinned->manifest->config.model_revision) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EIDENTITY, OA_RUNTIME_FACILITY_MODEL);
    }
    if (std::strncmp(base.required_coordinate_identity_sha256,
                     pinned->coordinate_identity_digest.c_str(),
                     OA_RUNTIME_DIGEST_CAPACITY) != 0) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EIDENTITY, OA_RUNTIME_FACILITY_MODEL);
    }
    if (pinned->controller == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
    }
    std::shared_ptr<openarm::runtime::PlanData> plan;
    try {
        plan = std::make_shared<openarm::runtime::PlanData>();
    } catch (...) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_ENOMEM, OA_RUNTIME_FACILITY_RUNTIME);
    }
    plan->runtime = pinned;
    {
        std::lock_guard<std::mutex> lock(pinned->mutex);
        if (pinned->owner != 1U || pinned->plan_pending) {
            openarm::runtime::runtime_init(pinned->last_error);
            pinned->last_error.status = OA_RUNTIME_EBUSY;
            pinned->last_error.facility = OA_RUNTIME_FACILITY_RUNTIME;
            return OA_RUNTIME_EBUSY;
        }
        pinned->plan_pending = true;
        pinned->plan_expiry_ns = base.expiry_runtime_monotonic_ns;
        pinned->plan_authority_id = pinned->next_plan_authority_id++;
        if (pinned->next_plan_authority_id == 0U) pinned->next_plan_authority_id = 1U;
        plan->authority_id = pinned->plan_authority_id;
        plan->holds_authority = true;
    }
    oa_converge_tcp_move move{};
    openarm::runtime::control_init(move);
    const std::uint64_t facade_now = openarm::runtime::now_ns();
    const std::uint64_t controller_now =
        pinned->controller_timeline_ns.load(std::memory_order_acquire);
    if (!translate_future_deadline(base.expiry_runtime_monotonic_ns, facade_now,
                                   controller_now, move.expiry_ns)) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_ETIMEOUT, OA_RUNTIME_FACILITY_RUNTIME);
    }
    plan->facade_expiry_ns = base.expiry_runtime_monotonic_ns;
    std::copy_n(base.required_feedback_seq, 2U, move.required_feedback_seq);
    std::copy_n(request->target_m, 3U, move.target_m);
    std::copy_n(request->contact_torque_nm, 7U, move.contact_torque_nm);
    move.contact_torque_fraction = request->contact_torque_fraction;
    move.contact_persistence_cycles = request->contact_persistence_cycles;
    move.stop_distance_m = request->stop_distance_m;
    move.minimum_progress_m = request->minimum_progress_m;
    move.velocity_scale = base.velocity_scale;
    move.acceleration_scale = base.acceleration_scale;
    move.jerk_scale = base.jerk_scale;
    move.tcp_tol_m = base.tcp_tolerance_m;
    move.collision_scene_revision = base.collision_scene_revision;
    move.max_branch_step_rad = base.maximum_branch_step_rad;
    move.min_singular_value = base.minimum_singular_value;
    oa_control_status lower = OA_CONTROL_OK;
    {
        std::lock_guard<std::mutex> controller_lock(pinned->controller_mutex);
        oa_snapshot current{};
        openarm::runtime::control_init(current);
        lower = oa_controller_snapshot(pinned->controller, &current);
        if (lower == OA_CONTROL_OK &&
            (base.required_feedback_seq[0] != current.arm[0].feedback_seq ||
             base.required_feedback_seq[1] != current.arm[1].feedback_seq)) {
            return openarm::runtime::record_error(
                pinned, OA_RUNTIME_ESTALE, OA_RUNTIME_FACILITY_RUNTIME);
        }
        if (lower == OA_CONTROL_OK) {
            lower = oa_controller_plan_converge_tcp(pinned->controller, &move, &plan->plan);
        }
    }
    if (lower != OA_CONTROL_OK) {
        return openarm::runtime::record_error(
            pinned, openarm::runtime::map_control(lower),
            OA_RUNTIME_FACILITY_CONTROL, lower);
    }
    oa_runtime_plan *const handle = openarm::runtime::plans.insert(plan);
    if (handle == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_ENOMEM, OA_RUNTIME_FACILITY_RUNTIME);
    }
    *out_plan = handle;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_get_contact_report(
    oa_runtime *runtime, oa_runtime_contact_report *out_report) {
    if (out_report == nullptr || out_report->struct_size < sizeof(*out_report) ||
        out_report->abi_version != OA_RUNTIME_MOTION_ABI_VERSION) {
        return OA_RUNTIME_EABI;
    }
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    const oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (pinned->controller == nullptr) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
    }
    oa_contact_report source{};
    openarm::runtime::control_init(source);
    oa_control_status lower = OA_CONTROL_OK;
    {
        std::lock_guard<std::mutex> controller_lock(pinned->controller_mutex);
        lower = oa_controller_get_contact_report(pinned->controller, &source);
    }
    if (lower != OA_CONTROL_OK) {
        return openarm::runtime::record_error(
            pinned, openarm::runtime::map_control(lower),
            OA_RUNTIME_FACILITY_CONTROL, lower);
    }
    oa_runtime_contact_report result{};
    result.struct_size = static_cast<std::uint32_t>(sizeof(result));
    result.abi_version = OA_RUNTIME_MOTION_ABI_VERSION;
    result.cause = source.cause;
    result.contact_detected = source.contact_detected;
    result.contact_side_mask = source.contact_side_mask;
    result.keepout_violation = source.keepout_violation;
    result.keepout_side = source.keepout_side;
    result.keepout_segment_a = source.keepout_segment_a;
    result.keepout_segment_b = source.keepout_segment_b;
    result.stop_monotonic_ns = source.stop_monotonic_ns;
    result.minimum_clearance_m = source.minimum_clearance_m;
    for (std::size_t side = 0; side < OA_RUNTIME_ARMS; ++side) {
        result.contact_joint_mask[side] = source.contact_joint_mask[side];
        result.stop_feedback_seq[side] = source.stop_feedback_seq[side];
        std::copy_n(source.contact_torque_nm[side], OA_RUNTIME_DOF,
                    result.contact_torque_nm[side]);
        std::copy_n(source.threshold_torque_nm[side], OA_RUNTIME_DOF,
                    result.threshold_torque_nm[side]);
        std::copy_n(source.stopped_q_rad[side], OA_RUNTIME_DOF,
                    result.stopped_q_rad[side]);
        std::copy_n(source.stopped_tcp_m[side], 3U, result.stopped_tcp_m[side]);
    }
    *out_report = result;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_sim_set_contact(
    oa_runtime *runtime, const oa_runtime_sim_contact *contact) {
    if (contact == nullptr || contact->struct_size < sizeof(*contact) ||
        contact->abi_version != OA_RUNTIME_MOTION_ABI_VERSION) {
        return OA_RUNTIME_EABI;
    }
    std::shared_ptr<openarm::runtime::RuntimeData> pinned;
    const oa_runtime_status status = require_runtime(runtime, pinned);
    if (status != OA_RUNTIME_OK) return status;
    if (pinned->controller == nullptr ||
        pinned->options.backend != OA_RUNTIME_BACKEND_VIRTUAL) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
    }
    oa_sim_contact lower_contact{};
    openarm::runtime::control_init(lower_contact);
    lower_contact.side = contact->side;
    lower_contact.enabled = contact->enabled;
    std::copy_n(contact->center_m, 3U, lower_contact.center_m);
    lower_contact.radius_m = contact->radius_m;
    lower_contact.reaction_gain_nm_per_rad = contact->reaction_gain_nm_per_rad;
    oa_control_status lower = OA_CONTROL_OK;
    {
        std::lock_guard<std::mutex> controller_lock(pinned->controller_mutex);
        lower = oa_controller_sim_set_contact(pinned->controller, &lower_contact);
    }
    if (lower != OA_CONTROL_OK) {
        return openarm::runtime::record_error(
            pinned, openarm::runtime::map_control(lower),
            OA_RUNTIME_FACILITY_CONTROL, lower);
    }
    return OA_RUNTIME_OK;
}

/* The emergency stop deliberately takes no handle and acquires no lock: it must
 * remain callable while any runtime is mid-cycle or holding any lock, and from
 * a signal handler. It forwards straight to the lock-free control latch that
 * every controller samples at the top of every cycle. */
extern "C" void oa_runtime_estop_assert(void) { oa_estop_assert(); }

extern "C" uint32_t oa_runtime_estop_asserted(void) { return oa_estop_asserted(); }

extern "C" oa_runtime_status oa_runtime_estop_clear(void) {
    return oa_estop_clear() == OA_CONTROL_OK ? OA_RUNTIME_OK : OA_RUNTIME_ESTATE;
}

extern "C" uint64_t oa_runtime_estop_assert_count(void) {
    return oa_estop_assert_count();
}
