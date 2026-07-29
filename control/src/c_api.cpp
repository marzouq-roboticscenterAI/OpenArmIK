/* SPDX-License-Identifier: Apache-2.0 */
#include "control_core.hpp"
#include "openarm_control.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <unordered_map>

using openarm::control::Controller;
using openarm::control::Manifest;
using openarm::control::MotionPlan;

namespace {
template <typename T>
bool valid_record(const T *record) noexcept {
    return record != nullptr && record->abi_version == OA_CONTROL_ABI_V1 &&
           record->struct_size >= sizeof(T);
}

template <typename T>
bool valid_record_prefix(const T *record, const std::uint32_t minimum_size) noexcept {
    return record != nullptr && record->abi_version == OA_CONTROL_ABI_V1 &&
           record->struct_size >= minimum_size;
}

template <typename T>
T copy_record_prefix(const T &record, T defaults) noexcept {
    const std::size_t bytes = std::min<std::size_t>(record.struct_size, sizeof(T));
    std::memcpy(&defaults, &record, bytes);
    defaults.struct_size = sizeof(T);
    defaults.abi_version = OA_CONTROL_ABI_V1;
    return defaults;
}

template <typename Callable>
oa_control_status contained(Callable &&callable) noexcept {
    try {
        return callable();
    } catch (const std::bad_alloc &) {
        return OA_CONTROL_ENOMEM;
    } catch (const std::invalid_argument &) {
        return OA_CONTROL_EINVAL;
    } catch (...) {
        return OA_CONTROL_EFAULT;
    }
}
}  // namespace

struct oa_manifest {
    std::uint8_t opaque{};
};

struct oa_controller {
    std::uint8_t opaque{};
};

struct oa_motion_plan {
    std::uint8_t opaque{};
};

namespace {
template <typename Implementation>
struct ImmutableSlot {
    std::mutex mutex;
    std::shared_ptr<const Implementation> impl;
    bool closing{};
};

template <typename Token, typename Slot>
struct TokenRegistry {
    std::mutex mutex;
    std::unordered_map<const Token *, std::shared_ptr<Slot>> active;
};

std::atomic<std::uintptr_t> next_opaque_token{static_cast<std::uintptr_t>(0x10000U)};

template <typename Token>
Token *allocate_opaque_token() {
    constexpr std::uintptr_t kStride = static_cast<std::uintptr_t>(16U);
    std::uintptr_t current = next_opaque_token.load();
    for (;;) {
        if (current > UINTPTR_MAX - kStride) {
            throw std::bad_alloc();
        }
        if (next_opaque_token.compare_exchange_weak(current, current + kStride)) {
            return reinterpret_cast<Token *>(current);
        }
    }
}

using ManifestSlot = ImmutableSlot<Manifest>;
using PlanSlot = ImmutableSlot<MotionPlan>;

TokenRegistry<oa_manifest, ManifestSlot> &manifest_registry() {
    static TokenRegistry<oa_manifest, ManifestSlot> registry;
    return registry;
}

TokenRegistry<oa_motion_plan, PlanSlot> &plan_registry() {
    static TokenRegistry<oa_motion_plan, PlanSlot> registry;
    return registry;
}

template <typename Token, typename Slot>
std::shared_ptr<Slot> pin_immutable(TokenRegistry<Token, Slot> &registry,
                                    const Token *token) {
    if (token == nullptr) {
        return {};
    }
    const std::lock_guard<std::mutex> lock(registry.mutex);
    const auto found = registry.active.find(token);
    return found == registry.active.end() ? std::shared_ptr<Slot>{} : found->second;
}

template <typename Token, typename Slot>
Token *publish_immutable(TokenRegistry<Token, Slot> &registry,
                         Token *const handle,
                         std::shared_ptr<Slot> slot) {
    const std::lock_guard<std::mutex> lock(registry.mutex);
    const auto inserted = registry.active.emplace(handle, std::move(slot));
    if (!inserted.second) {
        throw std::logic_error("opaque token collision");
    }
    return handle;
}

struct ControllerSlot {
    std::mutex mutex;
    std::condition_variable changed;
    std::unique_ptr<Controller> impl;
    bool closing{};
};

struct ControllerRegistry {
    std::mutex mutex;
    std::unordered_map<const oa_controller *, std::shared_ptr<ControllerSlot>> active;
};

ControllerRegistry &controller_registry() {
    static ControllerRegistry registry;
    return registry;
}

#ifdef OPENARM_CONTROL_ENABLE_TEST_HOOKS
std::atomic<std::int32_t> controller_create_fail_after{-1};
void controller_create_checkpoint() {
    const std::int32_t current = controller_create_fail_after.load();
    if (current < 0) {
        return;
    }
    if (current == 0) {
        controller_create_fail_after.store(-1);
        throw std::bad_alloc();
    }
    controller_create_fail_after.store(current - 1);
}
#else
void controller_create_checkpoint() noexcept {}
#endif

std::shared_ptr<ControllerSlot> pin_controller(const oa_controller *controller) {
    if (controller == nullptr) {
        return {};
    }
    auto &registry = controller_registry();
    const std::lock_guard<std::mutex> lock(registry.mutex);
    const auto found = registry.active.find(controller);
    return found == registry.active.end() ? std::shared_ptr<ControllerSlot>{} : found->second;
}

template <typename Callable>
oa_control_status with_controller(oa_controller *controller, Callable &&callable) noexcept {
    return contained([&]() -> oa_control_status {
        const auto slot = pin_controller(controller);
        if (!slot) {
            return OA_CONTROL_EINVAL;
        }
        std::unique_lock<std::mutex> lock(slot->mutex);
        if (slot->closing || !slot->impl) {
            return OA_CONTROL_ESTATE;
        }
        const oa_control_status status = callable(*slot->impl);
        lock.unlock();
        slot->changed.notify_all();
        return status;
    });
}
}  // namespace

#ifdef OPENARM_CONTROL_ENABLE_TEST_HOOKS
extern "C" void oa_control_test_fail_controller_create_after(
    const std::int32_t checkpoints) {
    controller_create_fail_after.store(checkpoints);
}

extern "C" std::size_t oa_control_test_active_controller_count(void) {
    auto &registry = controller_registry();
    const std::lock_guard<std::mutex> lock(registry.mutex);
    return registry.active.size();
}

extern "C" std::size_t oa_control_test_active_manifest_count(void) {
    auto &registry = manifest_registry();
    const std::lock_guard<std::mutex> lock(registry.mutex);
    return registry.active.size();
}

extern "C" std::size_t oa_control_test_active_plan_count(void) {
    auto &registry = plan_registry();
    const std::lock_guard<std::mutex> lock(registry.mutex);
    return registry.active.size();
}
#endif

extern "C" oa_control_status oa_manifest_create(const oa_manifest_config *config,
                                         oa_manifest **out) {
    if (!valid_record(config) || out == nullptr) {
        return config != nullptr && config->abi_version != OA_CONTROL_ABI_V1 ? OA_CONTROL_EABI
                                                                             : OA_CONTROL_EINVAL;
    }
    return contained([&]() -> oa_control_status {
        oa_manifest *const handle = allocate_opaque_token<oa_manifest>();
        auto slot = std::make_shared<ManifestSlot>();
        slot->impl = std::make_shared<const Manifest>(*config);
        *out = publish_immutable(manifest_registry(), handle, std::move(slot));
        return OA_CONTROL_OK;
    });
}

extern "C" oa_control_status oa_manifest_load(const char *path, const char *sha256_path,
                                       oa_manifest **out) {
    if (path == nullptr || sha256_path == nullptr || out == nullptr) {
        return OA_CONTROL_EINVAL;
    }
    return OA_CONTROL_EUNSUPPORTED;
}

extern "C" void oa_manifest_destroy(oa_manifest *manifest) {
    auto &registry = manifest_registry();
    std::shared_ptr<ManifestSlot> slot;
    {
        const std::lock_guard<std::mutex> lock(registry.mutex);
        const auto found = registry.active.find(manifest);
        if (found == registry.active.end()) {
            return;
        }
        slot = found->second;
        registry.active.erase(found);
    }
    {
        const std::lock_guard<std::mutex> lock(slot->mutex);
        slot->closing = true;
        slot->impl.reset();
    }
}

extern "C" oa_control_status oa_controller_create(const oa_manifest *manifest,
                                           const oa_controller_options *options,
                                           oa_controller **out) {
    if (!valid_record_prefix(options, OA_CONTROLLER_OPTIONS_V1_PREFIX_SIZE) || out == nullptr) {
        if (options != nullptr && options->abi_version != OA_CONTROL_ABI_V1) {
            return OA_CONTROL_EABI;
        }
        return OA_CONTROL_EINVAL;
    }
    const auto manifest_slot = pin_immutable(manifest_registry(), manifest);
    if (!manifest_slot) {
        return OA_CONTROL_EINVAL;
    }
    std::shared_ptr<const Manifest> manifest_impl;
    {
        const std::lock_guard<std::mutex> lock(manifest_slot->mutex);
        if (manifest_slot->closing || !manifest_slot->impl) {
            return OA_CONTROL_EINVAL;
        }
        manifest_impl = manifest_slot->impl;
    }
    oa_controller_options defaults{};
    defaults.collision_scene_revision = 1U;
    const oa_controller_options normalized = copy_record_prefix(*options, defaults);
    return contained([&]() -> oa_control_status {
        controller_create_checkpoint();
        auto slot = std::make_shared<ControllerSlot>();
        controller_create_checkpoint();
        slot->impl = std::make_unique<Controller>(manifest_impl, normalized);
        oa_controller *const handle = allocate_opaque_token<oa_controller>();
        auto &registry = controller_registry();
        {
            const std::lock_guard<std::mutex> lock(registry.mutex);
            controller_create_checkpoint();
            const auto inserted = registry.active.emplace(handle, slot);
            if (!inserted.second) {
                throw std::logic_error("opaque token collision");
            }
        }
        *out = handle;
        return OA_CONTROL_OK;
    });
}

extern "C" oa_control_status oa_controller_open_and_verify(oa_controller *controller,
                                                     oa_verify_report *out) {
    if (!valid_record(out)) {
        return out != nullptr && out->abi_version != OA_CONTROL_ABI_V1 ? OA_CONTROL_EABI : OA_CONTROL_EINVAL;
    }
    return with_controller(controller, [&](Controller &impl) -> oa_control_status {
        oa_verify_report temporary{};
        temporary.struct_size = sizeof(temporary);
        temporary.abi_version = OA_CONTROL_ABI_V1;
        const oa_control_status status = impl.open_and_verify(temporary);
        *out = temporary;
        return status;
    });
}

extern "C" oa_control_status oa_controller_snapshot(oa_controller *controller,
                                             oa_snapshot *out) {
    if (!valid_record(out)) {
        return out != nullptr && out->abi_version != OA_CONTROL_ABI_V1 ? OA_CONTROL_EABI : OA_CONTROL_EINVAL;
    }
    return with_controller(controller, [&](Controller &impl) -> oa_control_status {
        oa_snapshot temporary{};
        temporary.struct_size = sizeof(temporary);
        temporary.abi_version = OA_CONTROL_ABI_V1;
        const oa_control_status status = impl.snapshot(temporary);
        if (status == OA_CONTROL_OK) {
            *out = temporary;
        }
        return status;
    });
}

extern "C" oa_control_status oa_controller_get_kinematics(
    oa_controller *controller, const oa_side side,
    const std::uint64_t required_feedback_seq, oa_arm_kinematics *out) {
    if (!valid_record(out)) {
        return out != nullptr && out->abi_version != OA_CONTROL_ABI_V1 ? OA_CONTROL_EABI : OA_CONTROL_EINVAL;
    }
    return with_controller(controller, [&](Controller &impl) -> oa_control_status {
        oa_arm_kinematics temporary{};
        temporary.struct_size = sizeof(temporary);
        temporary.abi_version = OA_CONTROL_ABI_V1;
        const oa_control_status status =
            impl.kinematics(side, required_feedback_seq, temporary);
        if (status == OA_CONTROL_OK) {
            *out = temporary;
        }
        return status;
    });
}

extern "C" oa_control_status oa_controller_get_arm_challenge(oa_controller *controller,
                                                       oa_arm_challenge *out) {
    if (!valid_record(out)) {
        return out != nullptr && out->abi_version != OA_CONTROL_ABI_V1 ? OA_CONTROL_EABI : OA_CONTROL_EINVAL;
    }
    return with_controller(controller, [&](Controller &impl) -> oa_control_status {
        oa_arm_challenge temporary{};
        temporary.struct_size = sizeof(temporary);
        temporary.abi_version = OA_CONTROL_ABI_V1;
        const oa_control_status status = impl.challenge(temporary);
        if (status == OA_CONTROL_OK) {
            *out = temporary;
        }
        return status;
    });
}

extern "C" oa_control_status oa_controller_arm(oa_controller *controller,
                                        const oa_arm_challenge *challenge) {
    if (!valid_record(challenge)) {
        return challenge != nullptr && challenge->abi_version != OA_CONTROL_ABI_V1 ? OA_CONTROL_EABI
                                                                                    : OA_CONTROL_EINVAL;
    }
    return with_controller(controller, [&](Controller &impl) { return impl.arm(*challenge); });
}

extern "C" oa_control_status oa_controller_plan_joint(oa_controller *controller,
                                                const oa_joint_move *request,
                                                oa_motion_plan **out) {
    if (!valid_record(request) || out == nullptr) {
        return request != nullptr && request->abi_version != OA_CONTROL_ABI_V1 ? OA_CONTROL_EABI
                                                                               : OA_CONTROL_EINVAL;
    }
    return with_controller(controller, [&](Controller &impl) -> oa_control_status {
        std::unique_ptr<MotionPlan> plan;
        const oa_control_status status = impl.plan_joint(*request, plan);
        if (status != OA_CONTROL_OK) {
            return status;
        }
        oa_motion_plan *const handle = allocate_opaque_token<oa_motion_plan>();
        auto slot = std::make_shared<PlanSlot>();
        slot->impl = std::shared_ptr<const MotionPlan>(std::move(plan));
        *out = publish_immutable(plan_registry(), handle, std::move(slot));
        return OA_CONTROL_OK;
    });
}

extern "C" oa_control_status oa_controller_plan_paired_tcp(
    oa_controller *controller, const oa_paired_tcp_move *request,
    oa_motion_plan **out) {
    if (!valid_record_prefix(request, OA_PAIRED_TCP_MOVE_V1_PREFIX_SIZE) || out == nullptr) {
        return request != nullptr && request->abi_version != OA_CONTROL_ABI_V1 ? OA_CONTROL_EABI
                                                                               : OA_CONTROL_EINVAL;
    }
    oa_paired_tcp_move defaults{};
    defaults.max_branch_step_rad = 2.0;
    defaults.min_singular_value = 0.0;
    const oa_paired_tcp_move normalized = copy_record_prefix(*request, defaults);
    return with_controller(controller, [&](Controller &impl) -> oa_control_status {
        std::unique_ptr<MotionPlan> plan;
        const oa_control_status status = impl.plan_paired(normalized, plan);
        if (status != OA_CONTROL_OK) {
            return status;
        }
        oa_motion_plan *const handle = allocate_opaque_token<oa_motion_plan>();
        auto slot = std::make_shared<PlanSlot>();
        slot->impl = std::shared_ptr<const MotionPlan>(std::move(plan));
        *out = publish_immutable(plan_registry(), handle, std::move(slot));
        return OA_CONTROL_OK;
    });
}

extern "C" oa_control_status oa_motion_plan_get_report(const oa_motion_plan *plan,
                                                 oa_motion_plan_report *out) {
    if (!valid_record(out)) {
        return out != nullptr && out->abi_version != OA_CONTROL_ABI_V1 ? OA_CONTROL_EABI : OA_CONTROL_EINVAL;
    }
    const auto plan_slot = pin_immutable(plan_registry(), plan);
    if (!plan_slot) {
        return OA_CONTROL_EINVAL;
    }
    return contained([&]() -> oa_control_status {
        std::shared_ptr<const MotionPlan> plan_impl;
        {
            const std::lock_guard<std::mutex> lock(plan_slot->mutex);
            if (plan_slot->closing || !plan_slot->impl) {
                return OA_CONTROL_EINVAL;
            }
            plan_impl = plan_slot->impl;
        }
        oa_motion_plan_report temporary{};
        temporary.struct_size = sizeof(temporary);
        temporary.abi_version = OA_CONTROL_ABI_V1;
        temporary.kind = plan_impl->kind;
        temporary.collision_checked = plan_impl->collision_checked ? 1U : 0U;
        temporary.duration_ns = plan_impl->duration_ns;
        temporary.manifest_revision = plan_impl->manifest_revision;
        temporary.model_revision = plan_impl->model_revision;
        temporary.collision_scene_revision = plan_impl->collision_scene_revision;
        for (std::size_t side = 0; side < 2U; ++side) {
            temporary.seed_feedback_seq[side] = plan_impl->seed_seq[side];
            std::copy(plan_impl->target_q[side].begin(), plan_impl->target_q[side].end(),
                      temporary.target_q[side]);
            std::copy(plan_impl->achieved_tcp[side].begin(),
                      plan_impl->achieved_tcp[side].end(),
                      temporary.achieved_tcp_m[side]);
            temporary.tcp_residual_m[side] = plan_impl->tcp_residual[side];
        }
        *out = temporary;
        return OA_CONTROL_OK;
    });
}

extern "C" oa_control_status oa_controller_execute(oa_controller *controller,
                                             const oa_motion_plan *plan,
                                             const oa_execute_request *request,
                                             std::uint64_t *out_command_id) {
    if (!valid_record(request) || out_command_id == nullptr) {
        return request != nullptr && request->abi_version != OA_CONTROL_ABI_V1 ? OA_CONTROL_EABI
                                                                               : OA_CONTROL_EINVAL;
    }
    const auto plan_slot = pin_immutable(plan_registry(), plan);
    if (!plan_slot) {
        return OA_CONTROL_EINVAL;
    }
    std::shared_ptr<const MotionPlan> plan_impl;
    {
        const std::lock_guard<std::mutex> lock(plan_slot->mutex);
        if (plan_slot->closing || !plan_slot->impl) {
            return OA_CONTROL_EINVAL;
        }
        plan_impl = plan_slot->impl;
    }
    return with_controller(controller, [&](Controller &impl) -> oa_control_status {
        std::uint64_t temporary = 0U;
        const oa_control_status status = impl.execute(*plan_impl, *request, temporary);
        if (status == OA_CONTROL_OK) {
            *out_command_id = temporary;
        }
        return status;
    });
}

extern "C" oa_control_status oa_controller_advance(oa_controller *controller,
                                             const std::uint64_t monotonic_ns) {
    return with_controller(controller,
                           [&](Controller &impl) { return impl.advance(monotonic_ns); });
}

extern "C" oa_control_status oa_controller_sim_set_fault(oa_controller *controller,
                                                   const oa_sim_fault *fault) {
    if (!valid_record_prefix(fault, OA_SIM_FAULT_V1_PREFIX_SIZE)) {
        return fault != nullptr && fault->abi_version != OA_CONTROL_ABI_V1 ? OA_CONTROL_EABI
                                                                           : OA_CONTROL_EINVAL;
    }
    oa_sim_fault defaults{};
    defaults.fault_status = 8U;
    const oa_sim_fault normalized = copy_record_prefix(*fault, defaults);
    return with_controller(controller,
                           [&](Controller &impl) { return impl.set_sim_fault(normalized); });
}

extern "C" oa_control_status oa_controller_sim_set_state(oa_controller *controller,
                                                   const oa_sim_state *state) {
    if (!valid_record(state)) {
        return state != nullptr && state->abi_version != OA_CONTROL_ABI_V1 ? OA_CONTROL_EABI
                                                                           : OA_CONTROL_EINVAL;
    }
    return with_controller(controller,
                           [&](Controller &impl) { return impl.set_sim_state(*state); });
}

extern "C" oa_control_status oa_controller_heartbeat(oa_controller *controller,
                                               const std::uint64_t command_id,
                                               const std::uint64_t producer_deadline_ns) {
    return with_controller(controller, [&](Controller &impl) {
        return impl.heartbeat(command_id, producer_deadline_ns);
    });
}

extern "C" oa_control_status oa_controller_set_interlock(oa_controller *controller,
                                                   const std::uint32_t estop_active,
                                                   const std::uint32_t deadman_active) {
    if (estop_active > 1U || deadman_active > 1U) {
        return OA_CONTROL_EINVAL;
    }
    return with_controller(controller, [&](Controller &impl) {
        return impl.set_interlock(estop_active != 0U, deadman_active != 0U);
    });
}

extern "C" oa_control_status oa_controller_set_collision_scene_revision(
    oa_controller *controller, const std::uint64_t revision) {
    return with_controller(controller, [&](Controller &impl) {
        return impl.set_collision_scene_revision(revision);
    });
}

extern "C" oa_control_status oa_controller_stop(oa_controller *controller,
                                          const std::uint32_t stop_kind) {
    return with_controller(controller,
                           [&](Controller &impl) { return impl.stop(stop_kind); });
}

extern "C" oa_control_status oa_controller_disarm(oa_controller *controller,
                                            const std::uint64_t deadline_ns) {
    return with_controller(controller,
                           [&](Controller &impl) { return impl.disarm(deadline_ns); });
}

extern "C" oa_control_status oa_controller_reset_fault(oa_controller *controller,
                                                 const oa_reset_request *request) {
    if (!valid_record(request)) {
        return request != nullptr && request->abi_version != OA_CONTROL_ABI_V1 ? OA_CONTROL_EABI
                                                                               : OA_CONTROL_EINVAL;
    }
    return with_controller(controller,
                           [&](Controller &impl) { return impl.reset(*request); });
}

extern "C" oa_control_status oa_controller_poll_event(oa_controller *controller,
                                                const std::uint64_t deadline_ns,
                                                oa_event *out) {
    if (!valid_record(out)) {
        return out != nullptr && out->abi_version != OA_CONTROL_ABI_V1 ? OA_CONTROL_EABI : OA_CONTROL_EINVAL;
    }
    return contained([&]() -> oa_control_status {
        const auto slot = pin_controller(controller);
        if (!slot) {
            return OA_CONTROL_EINVAL;
        }
        std::unique_lock<std::mutex> lock(slot->mutex);
        for (;;) {
            if (slot->closing || !slot->impl) {
                return OA_CONTROL_ESTATE;
            }
            oa_event temporary{};
            temporary.struct_size = sizeof(temporary);
            temporary.abi_version = OA_CONTROL_ABI_V1;
            const oa_control_status status = slot->impl->poll_event(temporary);
            if (status == OA_CONTROL_OK) {
                *out = temporary;
                return OA_CONTROL_OK;
            }
            if (deadline_ns == 0U) {
                return OA_CONTROL_ETIMEOUT;
            }
            const auto deadline = std::chrono::steady_clock::time_point(
                std::chrono::nanoseconds(deadline_ns));
            if (std::chrono::steady_clock::now() >= deadline) {
                return OA_CONTROL_ETIMEOUT;
            }
            slot->changed.wait_until(lock, deadline);
        }
    });
}

extern "C" void oa_motion_plan_destroy(oa_motion_plan *plan) {
    auto &registry = plan_registry();
    std::shared_ptr<PlanSlot> slot;
    {
        const std::lock_guard<std::mutex> lock(registry.mutex);
        const auto found = registry.active.find(plan);
        if (found == registry.active.end()) {
            return;
        }
        slot = found->second;
        registry.active.erase(found);
    }
    {
        const std::lock_guard<std::mutex> lock(slot->mutex);
        slot->closing = true;
        slot->impl.reset();
    }
}

extern "C" void oa_controller_destroy(oa_controller *controller) {
    if (controller == nullptr) {
        return;
    }
    std::shared_ptr<ControllerSlot> slot;
    auto &registry = controller_registry();
    {
        const std::lock_guard<std::mutex> lock(registry.mutex);
        const auto found = registry.active.find(controller);
        if (found == registry.active.end()) {
            return;
        }
        slot = found->second;
        registry.active.erase(found);
    }
    {
        const std::lock_guard<std::mutex> lock(slot->mutex);
        slot->closing = true;
        if (slot->impl) {
            (void)slot->impl->disarm(UINT64_MAX);
            slot->impl.reset();
        }
    }
    slot->changed.notify_all();
}
