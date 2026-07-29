/* SPDX-License-Identifier: Apache-2.0 */
#ifndef OPENARM_RUNTIME_INTERNAL_HPP
#define OPENARM_RUNTIME_INTERNAL_HPP

#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1
#include "openarm_runtime.h"
#include "openarm_can.h"
#include "openarm_control.h"
#include "openarm_model.h"
#include "openarm_transport.h"

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace openarm::runtime {

constexpr std::uint32_t kManifestTag = 2U;
constexpr std::uint32_t kRuntimeTag = 3U;
constexpr std::uint32_t kInventoryTag = 4U;
constexpr std::uint32_t kCalibrationTag = 5U;
constexpr std::uint32_t kPlanTag = 6U;
constexpr std::uint32_t kPersistenceTag = 7U;

template <typename Public, typename Value, std::uint32_t Tag>
class Registry {
public:
    Public *insert(std::shared_ptr<Value> value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (next_ > (UINTPTR_MAX >> 4U)) {
            return nullptr;
        }
        const std::uintptr_t bits = (next_++ << 4U) | Tag;
        try {
            values_.emplace(bits, std::move(value));
        } catch (...) {
            return nullptr;
        }
        return reinterpret_cast<Public *>(bits);
    }

    std::shared_ptr<Value> pin(const Public *handle) const {
        const std::uintptr_t bits = reinterpret_cast<std::uintptr_t>(handle);
        if (handle == nullptr || (bits & static_cast<std::uintptr_t>(0xfU)) != Tag) {
            return {};
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = values_.find(bits);
        return found == values_.end() ? std::shared_ptr<Value>{} : found->second;
    }

    void erase(Public *handle) {
        const std::uintptr_t bits = reinterpret_cast<std::uintptr_t>(handle);
        if (handle == nullptr || (bits & static_cast<std::uintptr_t>(0xfU)) != Tag) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        values_.erase(bits);
    }

private:
    mutable std::mutex mutex_;
    std::unordered_map<std::uintptr_t, std::shared_ptr<Value>> values_;
    std::uintptr_t next_{1U};
};

struct ManifestData {
    oa_manifest_config config{};
    oa_runtime_manifest_state state{OA_RUNTIME_MANIFEST_DRAFT};
    oa_runtime_backend intended_backend{OA_RUNTIME_BACKEND_OFFLINE};
    std::uint64_t inventory_revision{};
    std::string inventory_fingerprint;
    std::array<std::uint32_t, OA_RUNTIME_MOTORS> evidence_kind{};
    std::array<std::uint64_t, OA_RUNTIME_MOTORS> evidence_revision{};
    std::array<std::string, OA_RUNTIME_MOTORS> evidence_record{};
    std::string content_digest;
    oa_runtime_integrity_kind integrity_kind{OA_RUNTIME_INTEGRITY_UNKEYED_SHA256};
    bool authenticated{};
    std::string authentication_key_id;
    std::string authentication_tag;
};

struct InventoryData {
    std::vector<oa_runtime_interface> interfaces;
    std::vector<oa_runtime_motor_evidence> motors;
    oa_runtime_inventory_summary summary{};
};

struct RuntimeData {
    std::mutex mutex;
    std::condition_variable wake;
    oa_runtime_options options{};
    std::shared_ptr<ManifestData> manifest;
    oa_manifest *control_manifest{};
    oa_controller *controller{};
    std::thread worker;
    std::atomic<std::uint64_t> timeline_ns{0U};
    std::atomic<std::uint64_t> controller_timeline_ns{0U};
    bool closing{};
    bool estop_active{};
    bool deadman_active{};
    std::uint32_t owner{};
    bool plan_pending{};
    std::uint64_t plan_expiry_ns{};
    std::uint64_t plan_authority_id{};
    std::uint64_t next_plan_authority_id{1U};
    oa_runtime_error_detail last_error{};
    std::uint64_t inventory_revision{};
    std::uint64_t calibration_revision{};
    std::string coordinate_identity_digest;

    ~RuntimeData();
};

struct PlanData {
    std::mutex mutex;
    std::shared_ptr<RuntimeData> runtime;
    oa_motion_plan *plan{};
    std::uint64_t facade_expiry_ns{};
    std::uint64_t authority_id{};
    bool holds_authority{};
    ~PlanData();
};

struct CalibrationData {
    std::mutex mutex;
    std::shared_ptr<RuntimeData> runtime;
    std::shared_ptr<ManifestData> base;
    oa_commission_manual_session *manual{};
    oa_commission_recipe_session *recipe{};
    std::uint32_t side{};
    std::uint32_t joint{};
    std::uint32_t required_posture_mask{};
    std::uint64_t evidence_revision{};
    std::uint64_t fixture_revision{};
    std::uint64_t last_sample_feedback_seq{};
    std::uint64_t last_sample_time_ns{};
    bool finished{};
    ~CalibrationData();
};

struct PersistenceAuthorityData {
    int directory_fd{-1};
    std::array<std::uint8_t, OA_RUNTIME_PERSISTENCE_KEY_BYTES> authentication_key{};
    std::string authentication_key_id;
    ~PersistenceAuthorityData();
};

extern Registry<oa_runtime_manifest, ManifestData, kManifestTag> &manifests;
extern Registry<oa_runtime, RuntimeData, kRuntimeTag> &runtimes;
extern Registry<oa_runtime_inventory, InventoryData, kInventoryTag> &inventories;
extern Registry<oa_runtime_calibration, CalibrationData, kCalibrationTag> &calibrations;
extern Registry<oa_runtime_plan, PlanData, kPlanTag> &plans;
extern Registry<oa_runtime_persistence_authority, PersistenceAuthorityData,
                kPersistenceTag> &persistence_authorities;

template <typename T>
bool output_valid(const T *record) {
    return record != nullptr && record->struct_size >= sizeof(T) &&
           record->abi_version == OA_RUNTIME_ABI_VERSION;
}

template <typename T>
void runtime_init(T &record) {
    record = {};
    record.struct_size = sizeof(T);
    record.abi_version = OA_RUNTIME_ABI_VERSION;
}

template <typename T>
void control_init(T &record) {
    record = {};
    record.struct_size = sizeof(T);
    record.abi_version = OA_CONTROL_ABI_V1;
}

std::uint64_t now_ns();
oa_runtime_status map_control(oa_control_status status);
oa_runtime_status map_commission(oa_commission_status status);
oa_runtime_status map_can(oa_can_status status);
oa_runtime_status map_transport(oa_transport_status status);
void set_error(const std::shared_ptr<RuntimeData> &runtime, oa_runtime_status status,
               oa_runtime_facility facility, std::uint32_t lower_code,
               std::uint32_t system_error = 0U);
oa_runtime_status record_error(const std::shared_ptr<RuntimeData> &runtime,
                               oa_runtime_status status,
                               oa_runtime_facility facility,
                               std::uint32_t lower_code = 0U,
                               std::uint32_t system_error = 0U);

oa_manifest_config virtual_config();
std::shared_ptr<ManifestData> make_virtual_manifest();
bool validate_manifest_data(const ManifestData &manifest);
std::shared_ptr<ManifestData> apply_patch(const ManifestData &base,
                                         const oa_commission_mapping_patch &patch,
                                         oa_runtime_manifest_preview &preview);
std::string manifest_canonical(const ManifestData &manifest, bool include_digest);
oa_runtime_status parse_manifest(const std::string &text,
                                 std::shared_ptr<ManifestData> &out);
std::string sha256_hex(const std::uint8_t *data, std::size_t size);
inline std::string sha256_hex(const std::string &data) {
    return sha256_hex(reinterpret_cast<const std::uint8_t *>(data.data()), data.size());
}
std::string hmac_sha256_hex(
    const std::array<std::uint8_t, OA_RUNTIME_PERSISTENCE_KEY_BYTES> &key,
    const std::string &data);

oa_runtime_collision_policy collision_policy_for(const RuntimeData &runtime);
oa_runtime_status fill_model_identity(const RuntimeData &runtime, std::uint32_t side,
                                      oa_runtime_model_identity &target);
std::string coordinate_identity_for(const ManifestData &manifest,
                                    oa_runtime_collision_policy collision_policy,
                                    std::uint64_t collision_scene_revision);

oa_runtime_capability capabilities_for(oa_runtime_backend backend);
oa_runtime_status fill_snapshot(const oa_snapshot &source, oa_runtime_snapshot &target);

#ifdef OA_RUNTIME_ENABLE_TEST_HOOKS
void allocation_checkpoint();
#else
inline void allocation_checkpoint() {}
#endif

} // namespace openarm::runtime

#endif
