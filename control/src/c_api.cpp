/* SPDX-License-Identifier: Apache-2.0 */
#include "control_core.hpp"
#include "openarm_control.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <unordered_map>
#include <vector>

using openarm::control::Controller;
using openarm::control::Manifest;
using openarm::control::MotionPlan;

namespace {
constexpr std::uint64_t kManifestMagic = UINT64_C(0x4f414d414e494631);
constexpr std::uint64_t kPlanMagic = UINT64_C(0x4f41504c414e5631);

template <typename T>
bool valid_record(const T *record) noexcept {
    return record != nullptr && record->abi_version == OA_CONTROL_ABI_V1 &&
           record->struct_size >= sizeof(T);
}

template <typename Callable>
oa_status contained(Callable &&callable) noexcept {
    try {
        return callable();
    } catch (const std::bad_alloc &) {
        return OA_ENOMEM;
    } catch (const std::invalid_argument &) {
        return OA_EINVAL;
    } catch (...) {
        return OA_EFAULT;
    }
}
}  // namespace

struct oa_manifest {
    std::uint64_t magic{kManifestMagic};
    std::shared_ptr<const Manifest> impl;
};

struct oa_controller {
    std::uint8_t opaque{};
};

struct oa_motion_plan {
    std::uint64_t magic{kPlanMagic};
    std::unique_ptr<MotionPlan> impl;
};

namespace {
struct ControllerSlot {
    std::mutex mutex;
    std::condition_variable changed;
    std::unique_ptr<Controller> impl;
    bool closing{};
};

struct ControllerRegistry {
    std::mutex mutex;
    std::unordered_map<const oa_controller *, std::shared_ptr<ControllerSlot>> active;
    /* Tokens are tombstoned until process exit so stale pointers can never alias
     * a newly allocated controller. Calls overlapping destroy only use the map. */
    std::vector<std::unique_ptr<oa_controller>> tokens;
};

ControllerRegistry &controller_registry() {
    static ControllerRegistry registry;
    return registry;
}

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
oa_status with_controller(oa_controller *controller, Callable &&callable) noexcept {
    return contained([&]() -> oa_status {
        const auto slot = pin_controller(controller);
        if (!slot) {
            return OA_EINVAL;
        }
        std::unique_lock<std::mutex> lock(slot->mutex);
        if (slot->closing || !slot->impl) {
            return OA_ESTATE;
        }
        const oa_status status = callable(*slot->impl);
        lock.unlock();
        slot->changed.notify_all();
        return status;
    });
}
}  // namespace

extern "C" oa_status oa_manifest_create(const oa_manifest_config *config,
                                         oa_manifest **out) {
    if (!valid_record(config) || out == nullptr) {
        return config != nullptr && config->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI
                                                                             : OA_EINVAL;
    }
    return contained([&]() -> oa_status {
        auto handle = std::make_unique<oa_manifest>();
        handle->impl = std::make_shared<const Manifest>(*config);
        *out = handle.release();
        return OA_OK;
    });
}

extern "C" oa_status oa_manifest_load(const char *path, const char *sha256_path,
                                       oa_manifest **out) {
    if (path == nullptr || sha256_path == nullptr || out == nullptr) {
        return OA_EINVAL;
    }
    return OA_EUNSUPPORTED;
}

extern "C" void oa_manifest_destroy(oa_manifest *manifest) {
    if (manifest != nullptr && manifest->magic == kManifestMagic) {
        manifest->magic = 0U;
        delete manifest;
    }
}

extern "C" oa_status oa_controller_create(const oa_manifest *manifest,
                                           const oa_controller_options *options,
                                           oa_controller **out) {
    if (manifest == nullptr || manifest->magic != kManifestMagic || !manifest->impl ||
        !valid_record(options) || out == nullptr) {
        if (options != nullptr && options->abi_version != OA_CONTROL_ABI_V1) {
            return OA_EABI;
        }
        return OA_EINVAL;
    }
    return contained([&]() -> oa_status {
        auto token = std::make_unique<oa_controller>();
        auto slot = std::make_shared<ControllerSlot>();
        slot->impl = std::make_unique<Controller>(manifest->impl, *options);
        oa_controller *const handle = token.get();
        auto &registry = controller_registry();
        {
            const std::lock_guard<std::mutex> lock(registry.mutex);
            registry.active.emplace(handle, slot);
            registry.tokens.push_back(std::move(token));
        }
        *out = handle;
        return OA_OK;
    });
}

extern "C" oa_status oa_controller_open_and_verify(oa_controller *controller,
                                                     oa_verify_report *out) {
    if (!valid_record(out)) {
        return out != nullptr && out->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI : OA_EINVAL;
    }
    return with_controller(controller, [&](Controller &impl) -> oa_status {
        oa_verify_report temporary{};
        temporary.struct_size = sizeof(temporary);
        temporary.abi_version = OA_CONTROL_ABI_V1;
        const oa_status status = impl.open_and_verify(temporary);
        *out = temporary;
        return status;
    });
}

extern "C" oa_status oa_controller_snapshot(oa_controller *controller,
                                             oa_snapshot *out) {
    if (!valid_record(out)) {
        return out != nullptr && out->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI : OA_EINVAL;
    }
    return with_controller(controller, [&](Controller &impl) -> oa_status {
        oa_snapshot temporary{};
        temporary.struct_size = sizeof(temporary);
        temporary.abi_version = OA_CONTROL_ABI_V1;
        const oa_status status = impl.snapshot(temporary);
        if (status == OA_OK) {
            *out = temporary;
        }
        return status;
    });
}

extern "C" oa_status oa_controller_get_kinematics(
    oa_controller *controller, const oa_side side,
    const std::uint64_t required_feedback_seq, oa_arm_kinematics *out) {
    if (!valid_record(out)) {
        return out != nullptr && out->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI : OA_EINVAL;
    }
    return with_controller(controller, [&](Controller &impl) -> oa_status {
        oa_arm_kinematics temporary{};
        temporary.struct_size = sizeof(temporary);
        temporary.abi_version = OA_CONTROL_ABI_V1;
        const oa_status status =
            impl.kinematics(side, required_feedback_seq, temporary);
        if (status == OA_OK) {
            *out = temporary;
        }
        return status;
    });
}

extern "C" oa_status oa_controller_get_arm_challenge(oa_controller *controller,
                                                       oa_arm_challenge *out) {
    if (!valid_record(out)) {
        return out != nullptr && out->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI : OA_EINVAL;
    }
    return with_controller(controller, [&](Controller &impl) -> oa_status {
        oa_arm_challenge temporary{};
        temporary.struct_size = sizeof(temporary);
        temporary.abi_version = OA_CONTROL_ABI_V1;
        const oa_status status = impl.challenge(temporary);
        if (status == OA_OK) {
            *out = temporary;
        }
        return status;
    });
}

extern "C" oa_status oa_controller_arm(oa_controller *controller,
                                        const oa_arm_challenge *challenge) {
    if (!valid_record(challenge)) {
        return challenge != nullptr && challenge->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI
                                                                                    : OA_EINVAL;
    }
    return with_controller(controller, [&](Controller &impl) { return impl.arm(*challenge); });
}

extern "C" oa_status oa_controller_plan_joint(oa_controller *controller,
                                                const oa_joint_move *request,
                                                oa_motion_plan **out) {
    if (!valid_record(request) || out == nullptr) {
        return request != nullptr && request->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI
                                                                               : OA_EINVAL;
    }
    return with_controller(controller, [&](Controller &impl) -> oa_status {
        std::unique_ptr<MotionPlan> plan;
        const oa_status status = impl.plan_joint(*request, plan);
        if (status != OA_OK) {
            return status;
        }
        auto handle = std::make_unique<oa_motion_plan>();
        handle->impl = std::move(plan);
        *out = handle.release();
        return OA_OK;
    });
}

extern "C" oa_status oa_controller_plan_paired_tcp(
    oa_controller *controller, const oa_paired_tcp_move *request,
    oa_motion_plan **out) {
    if (!valid_record(request) || out == nullptr) {
        return request != nullptr && request->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI
                                                                               : OA_EINVAL;
    }
    return with_controller(controller, [&](Controller &impl) -> oa_status {
        std::unique_ptr<MotionPlan> plan;
        const oa_status status = impl.plan_paired(*request, plan);
        if (status != OA_OK) {
            return status;
        }
        auto handle = std::make_unique<oa_motion_plan>();
        handle->impl = std::move(plan);
        *out = handle.release();
        return OA_OK;
    });
}

extern "C" oa_status oa_motion_plan_get_report(const oa_motion_plan *plan,
                                                 oa_motion_plan_report *out) {
    if (plan == nullptr || plan->magic != kPlanMagic || !plan->impl ||
        !valid_record(out)) {
        return out != nullptr && out->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI : OA_EINVAL;
    }
    return contained([&]() -> oa_status {
        oa_motion_plan_report temporary{};
        temporary.struct_size = sizeof(temporary);
        temporary.abi_version = OA_CONTROL_ABI_V1;
        temporary.kind = plan->impl->kind;
        temporary.collision_checked = plan->impl->collision_checked ? 1U : 0U;
        temporary.duration_ns = plan->impl->duration_ns;
        temporary.manifest_revision = plan->impl->manifest_revision;
        temporary.model_revision = plan->impl->model_revision;
        temporary.collision_scene_revision = plan->impl->collision_scene_revision;
        for (std::size_t side = 0; side < 2U; ++side) {
            temporary.seed_feedback_seq[side] = plan->impl->seed_seq[side];
            std::copy(plan->impl->target_q[side].begin(), plan->impl->target_q[side].end(),
                      temporary.target_q[side]);
            std::copy(plan->impl->achieved_tcp[side].begin(),
                      plan->impl->achieved_tcp[side].end(),
                      temporary.achieved_tcp_m[side]);
            temporary.tcp_residual_m[side] = plan->impl->tcp_residual[side];
        }
        *out = temporary;
        return OA_OK;
    });
}

extern "C" oa_status oa_controller_execute(oa_controller *controller,
                                             const oa_motion_plan *plan,
                                             const oa_execute_request *request,
                                             std::uint64_t *out_command_id) {
    if (plan == nullptr || plan->magic != kPlanMagic ||
        !plan->impl || !valid_record(request) || out_command_id == nullptr) {
        return request != nullptr && request->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI
                                                                               : OA_EINVAL;
    }
    return with_controller(controller, [&](Controller &impl) -> oa_status {
        std::uint64_t temporary = 0U;
        const oa_status status = impl.execute(*plan->impl, *request, temporary);
        if (status == OA_OK) {
            *out_command_id = temporary;
        }
        return status;
    });
}

extern "C" oa_status oa_controller_advance(oa_controller *controller,
                                             const std::uint64_t monotonic_ns) {
    return with_controller(controller,
                           [&](Controller &impl) { return impl.advance(monotonic_ns); });
}

extern "C" oa_status oa_controller_sim_set_fault(oa_controller *controller,
                                                   const oa_sim_fault *fault) {
    if (!valid_record(fault)) {
        return fault != nullptr && fault->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI
                                                                           : OA_EINVAL;
    }
    return with_controller(controller,
                           [&](Controller &impl) { return impl.set_sim_fault(*fault); });
}

extern "C" oa_status oa_controller_sim_set_state(oa_controller *controller,
                                                   const oa_sim_state *state) {
    if (!valid_record(state)) {
        return state != nullptr && state->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI
                                                                           : OA_EINVAL;
    }
    return with_controller(controller,
                           [&](Controller &impl) { return impl.set_sim_state(*state); });
}

extern "C" oa_status oa_controller_heartbeat(oa_controller *controller,
                                               const std::uint64_t command_id,
                                               const std::uint64_t producer_deadline_ns) {
    return with_controller(controller, [&](Controller &impl) {
        return impl.heartbeat(command_id, producer_deadline_ns);
    });
}

extern "C" oa_status oa_controller_set_interlock(oa_controller *controller,
                                                   const std::uint32_t estop_active,
                                                   const std::uint32_t deadman_active) {
    if (estop_active > 1U || deadman_active > 1U) {
        return OA_EINVAL;
    }
    return with_controller(controller, [&](Controller &impl) {
        return impl.set_interlock(estop_active != 0U, deadman_active != 0U);
    });
}

extern "C" oa_status oa_controller_set_collision_scene_revision(
    oa_controller *controller, const std::uint64_t revision) {
    return with_controller(controller, [&](Controller &impl) {
        return impl.set_collision_scene_revision(revision);
    });
}

extern "C" oa_status oa_controller_stop(oa_controller *controller,
                                          const std::uint32_t stop_kind) {
    return with_controller(controller,
                           [&](Controller &impl) { return impl.stop(stop_kind); });
}

extern "C" oa_status oa_controller_disarm(oa_controller *controller,
                                            const std::uint64_t deadline_ns) {
    return with_controller(controller,
                           [&](Controller &impl) { return impl.disarm(deadline_ns); });
}

extern "C" oa_status oa_controller_reset_fault(oa_controller *controller,
                                                 const oa_reset_request *request) {
    if (!valid_record(request)) {
        return request != nullptr && request->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI
                                                                               : OA_EINVAL;
    }
    return with_controller(controller,
                           [&](Controller &impl) { return impl.reset(*request); });
}

extern "C" oa_status oa_controller_poll_event(oa_controller *controller,
                                                const std::uint64_t deadline_ns,
                                                oa_event *out) {
    if (!valid_record(out)) {
        return out != nullptr && out->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI : OA_EINVAL;
    }
    return contained([&]() -> oa_status {
        const auto slot = pin_controller(controller);
        if (!slot) {
            return OA_EINVAL;
        }
        std::unique_lock<std::mutex> lock(slot->mutex);
        for (;;) {
            if (slot->closing || !slot->impl) {
                return OA_ESTATE;
            }
            oa_event temporary{};
            temporary.struct_size = sizeof(temporary);
            temporary.abi_version = OA_CONTROL_ABI_V1;
            const oa_status status = slot->impl->poll_event(temporary);
            if (status == OA_OK) {
                *out = temporary;
                return OA_OK;
            }
            if (deadline_ns == 0U) {
                return OA_ETIMEOUT;
            }
            const auto deadline = std::chrono::steady_clock::time_point(
                std::chrono::nanoseconds(deadline_ns));
            if (std::chrono::steady_clock::now() >= deadline) {
                return OA_ETIMEOUT;
            }
            slot->changed.wait_until(lock, deadline);
        }
    });
}

extern "C" void oa_motion_plan_destroy(oa_motion_plan *plan) {
    if (plan != nullptr && plan->magic == kPlanMagic) {
        plan->magic = 0U;
        delete plan;
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
