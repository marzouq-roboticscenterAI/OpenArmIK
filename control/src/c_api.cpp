/* SPDX-License-Identifier: Apache-2.0 */
#include "control_core.hpp"
#include "openarm_control.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#include <stdexcept>

using openarm::control::Controller;
using openarm::control::Manifest;
using openarm::control::MotionPlan;

namespace {
constexpr std::uint64_t kManifestMagic = UINT64_C(0x4f414d414e494631);
constexpr std::uint64_t kControllerMagic = UINT64_C(0x4f414354524c5631);
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
    std::uint64_t magic{kControllerMagic};
    std::unique_ptr<Controller> impl;
};

struct oa_motion_plan {
    std::uint64_t magic{kPlanMagic};
    std::unique_ptr<MotionPlan> impl;
};

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
        auto handle = std::make_unique<oa_controller>();
        handle->impl = std::make_unique<Controller>(manifest->impl, *options);
        *out = handle.release();
        return OA_OK;
    });
}

extern "C" oa_status oa_controller_open_and_verify(oa_controller *controller,
                                                     oa_verify_report *out) {
    if (controller == nullptr || controller->magic != kControllerMagic ||
        !controller->impl || !valid_record(out)) {
        return out != nullptr && out->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI : OA_EINVAL;
    }
    return contained([&]() -> oa_status {
        oa_verify_report temporary{};
        temporary.struct_size = sizeof(temporary);
        temporary.abi_version = OA_CONTROL_ABI_V1;
        const oa_status status = controller->impl->open_and_verify(temporary);
        *out = temporary;
        return status;
    });
}

extern "C" oa_status oa_controller_snapshot(oa_controller *controller,
                                             oa_snapshot *out) {
    if (controller == nullptr || controller->magic != kControllerMagic ||
        !controller->impl || !valid_record(out)) {
        return out != nullptr && out->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI : OA_EINVAL;
    }
    return contained([&]() -> oa_status {
        oa_snapshot temporary{};
        temporary.struct_size = sizeof(temporary);
        temporary.abi_version = OA_CONTROL_ABI_V1;
        const oa_status status = controller->impl->snapshot(temporary);
        if (status == OA_OK) {
            *out = temporary;
        }
        return status;
    });
}

extern "C" oa_status oa_controller_get_kinematics(
    oa_controller *controller, const oa_side side,
    const std::uint64_t required_feedback_seq, oa_arm_kinematics *out) {
    if (controller == nullptr || controller->magic != kControllerMagic ||
        !controller->impl || !valid_record(out)) {
        return out != nullptr && out->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI : OA_EINVAL;
    }
    return contained([&]() -> oa_status {
        oa_arm_kinematics temporary{};
        temporary.struct_size = sizeof(temporary);
        temporary.abi_version = OA_CONTROL_ABI_V1;
        const oa_status status =
            controller->impl->kinematics(side, required_feedback_seq, temporary);
        if (status == OA_OK) {
            *out = temporary;
        }
        return status;
    });
}

extern "C" oa_status oa_controller_get_arm_challenge(oa_controller *controller,
                                                       oa_arm_challenge *out) {
    if (controller == nullptr || controller->magic != kControllerMagic ||
        !controller->impl || !valid_record(out)) {
        return out != nullptr && out->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI : OA_EINVAL;
    }
    return contained([&]() -> oa_status {
        oa_arm_challenge temporary{};
        temporary.struct_size = sizeof(temporary);
        temporary.abi_version = OA_CONTROL_ABI_V1;
        const oa_status status = controller->impl->challenge(temporary);
        if (status == OA_OK) {
            *out = temporary;
        }
        return status;
    });
}

extern "C" oa_status oa_controller_arm(oa_controller *controller,
                                        const oa_arm_challenge *challenge) {
    if (controller == nullptr || controller->magic != kControllerMagic ||
        !controller->impl || !valid_record(challenge)) {
        return challenge != nullptr && challenge->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI
                                                                                    : OA_EINVAL;
    }
    return contained([&]() { return controller->impl->arm(*challenge); });
}

extern "C" oa_status oa_controller_plan_joint(oa_controller *controller,
                                                const oa_joint_move *request,
                                                oa_motion_plan **out) {
    if (controller == nullptr || controller->magic != kControllerMagic ||
        !controller->impl || !valid_record(request) || out == nullptr) {
        return request != nullptr && request->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI
                                                                               : OA_EINVAL;
    }
    return contained([&]() -> oa_status {
        std::unique_ptr<MotionPlan> plan;
        const oa_status status = controller->impl->plan_joint(*request, plan);
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
    if (controller == nullptr || controller->magic != kControllerMagic ||
        !controller->impl || !valid_record(request) || out == nullptr) {
        return request != nullptr && request->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI
                                                                               : OA_EINVAL;
    }
    return contained([&]() -> oa_status {
        std::unique_ptr<MotionPlan> plan;
        const oa_status status = controller->impl->plan_paired(*request, plan);
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
    if (controller == nullptr || controller->magic != kControllerMagic ||
        !controller->impl || plan == nullptr || plan->magic != kPlanMagic ||
        !plan->impl || !valid_record(request) || out_command_id == nullptr) {
        return request != nullptr && request->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI
                                                                               : OA_EINVAL;
    }
    return contained([&]() -> oa_status {
        std::uint64_t temporary = 0U;
        const oa_status status = controller->impl->execute(*plan->impl, *request, temporary);
        if (status == OA_OK) {
            *out_command_id = temporary;
        }
        return status;
    });
}

extern "C" oa_status oa_controller_advance(oa_controller *controller,
                                             const std::uint64_t monotonic_ns) {
    if (controller == nullptr || controller->magic != kControllerMagic ||
        !controller->impl) {
        return OA_EINVAL;
    }
    return contained([&]() { return controller->impl->advance(monotonic_ns); });
}

extern "C" oa_status oa_controller_sim_set_fault(oa_controller *controller,
                                                   const oa_sim_fault *fault) {
    if (controller == nullptr || controller->magic != kControllerMagic ||
        !controller->impl || !valid_record(fault)) {
        return fault != nullptr && fault->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI
                                                                           : OA_EINVAL;
    }
    return contained([&]() { return controller->impl->set_sim_fault(*fault); });
}

extern "C" oa_status oa_controller_stop(oa_controller *controller,
                                          const std::uint32_t stop_kind) {
    if (controller == nullptr || controller->magic != kControllerMagic ||
        !controller->impl) {
        return OA_EINVAL;
    }
    return contained([&]() { return controller->impl->stop(stop_kind); });
}

extern "C" oa_status oa_controller_disarm(oa_controller *controller,
                                            const std::uint64_t deadline_ns) {
    if (controller == nullptr || controller->magic != kControllerMagic ||
        !controller->impl) {
        return OA_EINVAL;
    }
    return contained([&]() { return controller->impl->disarm(deadline_ns); });
}

extern "C" oa_status oa_controller_reset_fault(oa_controller *controller,
                                                 const oa_reset_request *request) {
    if (controller == nullptr || controller->magic != kControllerMagic ||
        !controller->impl || !valid_record(request)) {
        return request != nullptr && request->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI
                                                                               : OA_EINVAL;
    }
    return contained([&]() { return controller->impl->reset(*request); });
}

extern "C" oa_status oa_controller_poll_event(oa_controller *controller,
                                                const std::uint64_t deadline_ns,
                                                oa_event *out) {
    (void)deadline_ns;
    if (controller == nullptr || controller->magic != kControllerMagic ||
        !controller->impl || !valid_record(out)) {
        return out != nullptr && out->abi_version != OA_CONTROL_ABI_V1 ? OA_EABI : OA_EINVAL;
    }
    return contained([&]() -> oa_status {
        oa_event temporary{};
        temporary.struct_size = sizeof(temporary);
        temporary.abi_version = OA_CONTROL_ABI_V1;
        const oa_status status = controller->impl->poll_event(temporary);
        if (status == OA_OK) {
            *out = temporary;
        }
        return status;
    });
}

extern "C" void oa_motion_plan_destroy(oa_motion_plan *plan) {
    if (plan != nullptr && plan->magic == kPlanMagic) {
        plan->magic = 0U;
        delete plan;
    }
}

extern "C" void oa_controller_destroy(oa_controller *controller) {
    if (controller != nullptr && controller->magic == kControllerMagic) {
        if (controller->impl) {
            (void)controller->impl->disarm(UINT64_MAX);
        }
        controller->magic = 0U;
        delete controller;
    }
}
