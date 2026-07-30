/* SPDX-License-Identifier: Apache-2.0 */
#include "runtime_internal.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <linux/if_arp.h>
#include <string>

namespace openarm::runtime {
namespace {

constexpr std::uint64_t kAllVirtualRegisterPresence = (UINT64_C(1) << 14U) - 1U;
constexpr std::size_t kMaximumPhysicalLinkEntries =
    static_cast<std::size_t>(OA_RUNTIME_MAX_INTERFACES) * 8U;

std::shared_ptr<InventoryData> virtual_inventory(std::uint64_t timestamp_ns) {
    auto result = std::make_shared<InventoryData>();
    for (std::uint32_t side = 0U; side < 2U; ++side) {
        oa_runtime_interface interface{};
        runtime_init(interface);
        std::snprintf(interface.name, sizeof(interface.name), "virtual%u", side);
        interface.ifindex = side + 1U;
        interface.flags = 0U;
        interface.mtu = 16U;
        interface.bitrate = 1000000U;
        interface.link_up = 1U;
        interface.interface_kind = OA_RUNTIME_INTERFACE_KIND_VIRTUAL;
        result->interfaces.push_back(interface);
        for (std::uint32_t joint = 0U; joint < 7U; ++joint) {
            oa_runtime_motor_evidence motor{};
            runtime_init(motor);
            std::snprintf(motor.interface_name, sizeof(motor.interface_name),
                          "virtual%u", side);
            motor.requested_send_id = static_cast<std::uint16_t>(joint + 1U);
            motor.expected_receive_id = static_cast<std::uint16_t>(joint + 0x11U);
            motor.observed_receive_id = motor.expected_receive_id;
            motor.query_sent_runtime_monotonic_ns = timestamp_ns;
            motor.response_runtime_monotonic_ns = timestamp_ns;
            motor.register_presence_mask = kAllVirtualRegisterPresence;
            motor.confidence = OA_RUNTIME_EVIDENCE_VIRTUAL_EXACT;
            motor.unresolved_assignment = 0U;
            motor.side = side;
            motor.joint = joint;
            motor.serial_number = side * 100U + joint + 1U;
            motor.hardware_version = 1U;
            motor.software_version = 1U;
            motor.firmware_subversion = 1U;
            motor.configured_send_id = joint + 1U;
            motor.configured_receive_id = joint + 0x11U;
            motor.control_mode = 1U;
            motor.bitrate = 1000000U;
            motor.timeout_ticks = 1000U;
            motor.direction = ((side + joint) & 1U) == 0U ? 1 : -1;
            motor.pmax_rad = 12.5F;
            if (joint < 2U) {
                motor.vmax_rad_s = 45.0F; motor.tmax_nm = 54.0F; motor.gear_ratio = 9.0F;
            } else if (joint < 4U) {
                motor.vmax_rad_s = 10.0F; motor.tmax_nm = 28.0F; motor.gear_ratio = 40.0F;
            } else {
                motor.vmax_rad_s = 30.0F; motor.tmax_nm = 10.0F; motor.gear_ratio = 10.0F;
            }
            std::snprintf(motor.immutable_identity, sizeof(motor.immutable_identity),
                          "VIRTUAL-%u-%u", side, joint);
            result->motors.push_back(motor);
        }
    }
    runtime_init(result->summary);
    result->summary.interface_count = 2U;
    result->summary.motor_count = 14U;
    result->summary.inventory_revision = 1U;
    result->summary.clock_id = OA_RUNTIME_CLOCK_MONOTONIC;
    std::string identity;
    for (const auto &motor : result->motors) {
        identity += motor.immutable_identity;
        identity += '\n';
    }
    const std::string fingerprint = sha256_hex(identity);
    std::snprintf(result->summary.fingerprint_sha256,
                  sizeof(result->summary.fingerprint_sha256), "%s", fingerprint.c_str());
    return result;
}

std::vector<oa_runtime_interface> physical_interfaces() {
    std::vector<oa_runtime_interface> result;
    result.reserve(OA_RUNTIME_MAX_INTERFACES);
    std::unique_ptr<DIR, int (*)(DIR *)> directory(opendir("/sys/class/net"), closedir);
    if (!directory) return result;
    std::size_t examined = 0U;
    for (dirent *entry = readdir(directory.get()); entry != nullptr;
         entry = readdir(directory.get())) {
        if (entry->d_name[0] == '.' ||
            std::strlen(entry->d_name) >= sizeof(oa_runtime_interface::name)) {
            continue;
        }
        ++examined;
        if (examined > kMaximumPhysicalLinkEntries) break;
        const std::string prefix = std::string("/sys/class/net/") + entry->d_name + '/';
        auto read_u32 = [&prefix](const char *name, bool hexadecimal,
                                  std::uint32_t &value) {
            FILE *const stream = std::fopen((prefix + name).c_str(), "re");
            if (stream == nullptr) return false;
            unsigned int parsed = 0U;
            const bool ok = hexadecimal
                                ? std::fscanf(stream, "%x", &parsed) == 1
                                : std::fscanf(stream, "%u", &parsed) == 1;
            std::fclose(stream);
            if (ok) value = parsed;
            return ok;
        };
        std::uint32_t link_type = 0U;
        if (!read_u32("type", false, link_type) || link_type != ARPHRD_CAN) continue;
        oa_runtime_interface item{};
        runtime_init(item);
        std::snprintf(item.name, sizeof(item.name), "%s", entry->d_name);
        (void)read_u32("ifindex", false, item.ifindex);
        (void)read_u32("flags", true, item.flags);
        (void)read_u32("mtu", false, item.mtu);
        FILE *const state = std::fopen((prefix + "operstate").c_str(), "re");
        if (state != nullptr) {
            char value[16]{};
            if (std::fscanf(state, "%15s", value) == 1 &&
                std::strcmp(value, "up") == 0) {
                item.link_up = 1U;
            }
            std::fclose(state);
        }
        /* Bitrates and CAN-FD state are deliberately left unknown (zero):
         * sysfs exposes no authoritative, portable read-only values. */
        item.interface_kind = OA_RUNTIME_INTERFACE_KIND_PHYSICAL;
        result.push_back(item);
        if (result.size() == OA_RUNTIME_MAX_INTERFACES) break;
    }
    std::sort(result.begin(), result.end(), [](const oa_runtime_interface &left,
                                               const oa_runtime_interface &right) {
        if (left.ifindex != right.ifindex) return left.ifindex < right.ifindex;
        return std::strcmp(left.name, right.name) < 0;
    });
    return result;
}

}
}

extern "C" oa_runtime_status oa_runtime_list_interfaces(
    const oa_runtime *runtime, oa_runtime_interface *interfaces,
    std::size_t capacity, std::size_t *out_count) {
    const auto pinned = openarm::runtime::runtimes.pin(runtime);
    if (!pinned) return OA_RUNTIME_EINVAL;
    if (out_count == nullptr || (capacity != 0U && interfaces == nullptr)) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    std::vector<oa_runtime_interface> found;
    try {
        if (pinned->options.backend == OA_RUNTIME_BACKEND_VIRTUAL) {
            found = openarm::runtime::virtual_inventory(openarm::runtime::now_ns())->interfaces;
        } else if (pinned->options.backend == OA_RUNTIME_BACKEND_SOCKETCAN_QUERY) {
            found = openarm::runtime::physical_interfaces();
        }
    } catch (...) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_ENOMEM, OA_RUNTIME_FACILITY_RUNTIME);
    }
    *out_count = found.size();
    if (capacity < found.size()) {
        return openarm::runtime::record_error(
            pinned, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    for (std::size_t i = 0U; i < found.size(); ++i) {
        if (!openarm::runtime::output_valid(&interfaces[i])) return OA_RUNTIME_EABI;
    }
    for (std::size_t i = 0U; i < found.size(); ++i) interfaces[i] = found[i];
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_inventory_query(
    oa_runtime *runtime, const oa_runtime_inventory_query_options *options,
    oa_runtime_inventory **out_inventory) {
    const auto owner = openarm::runtime::runtimes.pin(runtime);
    if (!owner) return OA_RUNTIME_EINVAL;
    if (out_inventory == nullptr) {
        return openarm::runtime::record_error(
            owner, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    *out_inventory = nullptr;
    try {
        std::shared_ptr<openarm::runtime::InventoryData> result;
        if (owner->options.backend == OA_RUNTIME_BACKEND_VIRTUAL) {
            const std::uint64_t timestamp = openarm::runtime::now_ns();
            result = openarm::runtime::virtual_inventory(timestamp);
        } else if (owner->options.backend == OA_RUNTIME_BACKEND_SOCKETCAN_QUERY) {
            (void)options;
            return openarm::runtime::record_error(
                owner, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
        } else {
            return openarm::runtime::record_error(
                owner, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
        }
        oa_runtime_inventory *const handle = openarm::runtime::inventories.insert(result);
        if (handle == nullptr) {
            return openarm::runtime::record_error(
                owner, OA_RUNTIME_ENOMEM, OA_RUNTIME_FACILITY_RUNTIME);
        }
        {
            std::lock_guard<std::mutex> lock(owner->mutex);
            owner->inventory_revision = result->summary.inventory_revision;
        }
        *out_inventory = handle;
        return OA_RUNTIME_OK;
    } catch (...) {
        return openarm::runtime::record_error(
            owner, OA_RUNTIME_ENOMEM, OA_RUNTIME_FACILITY_RUNTIME);
    }
}

extern "C" oa_runtime_status oa_runtime_inventory_get_summary(
    const oa_runtime_inventory *inventory, oa_runtime_inventory_summary *out_summary) {
    if (!openarm::runtime::output_valid(out_summary)) return OA_RUNTIME_EABI;
    const auto pinned = openarm::runtime::inventories.pin(inventory);
    if (!pinned) return OA_RUNTIME_EINVAL;
    *out_summary = pinned->summary;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_inventory_get_interface(
    const oa_runtime_inventory *inventory, std::size_t index,
    oa_runtime_interface *out_interface) {
    if (!openarm::runtime::output_valid(out_interface)) return OA_RUNTIME_EABI;
    const auto pinned = openarm::runtime::inventories.pin(inventory);
    if (!pinned || index >= pinned->interfaces.size()) return OA_RUNTIME_EINVAL;
    *out_interface = pinned->interfaces[index];
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_inventory_get_motor(
    const oa_runtime_inventory *inventory, std::size_t index,
    oa_runtime_motor_evidence *out_motor) {
    if (!openarm::runtime::output_valid(out_motor)) return OA_RUNTIME_EABI;
    const auto pinned = openarm::runtime::inventories.pin(inventory);
    if (!pinned || index >= pinned->motors.size()) return OA_RUNTIME_EINVAL;
    *out_motor = pinned->motors[index];
    return OA_RUNTIME_OK;
}

extern "C" void oa_runtime_inventory_destroy(oa_runtime_inventory *inventory) {
    openarm::runtime::inventories.erase(inventory);
}

extern "C" oa_runtime_status oa_runtime_configuration_preview_physical(
    const oa_runtime_manifest *manifest, const oa_runtime_inventory *inventory,
    oa_runtime_manifest_preview *out_preview) {
    if (!openarm::runtime::output_valid(out_preview)) return OA_RUNTIME_EABI;
    oa_runtime_manifest_preview result{};
    openarm::runtime::runtime_init(result);
    result.validation_status = OA_RUNTIME_EUNSUPPORTED;
    *out_preview = result;
    const auto manifest_data = openarm::runtime::manifests.pin(manifest);
    const auto inventory_data = openarm::runtime::inventories.pin(inventory);
    if (!manifest_data || !inventory_data) return OA_RUNTIME_EINVAL;
    result.base_revision = manifest_data->config.manifest_revision;
    result.result_revision = manifest_data->config.manifest_revision;
    *out_preview = result;
    return OA_RUNTIME_EUNSUPPORTED;
}

extern "C" oa_runtime_status oa_runtime_configuration_apply_physical(
    oa_runtime *runtime, const oa_runtime_manifest *manifest) {
    const auto owner = openarm::runtime::runtimes.pin(runtime);
    if (!owner) return OA_RUNTIME_EINVAL;
    if (!openarm::runtime::manifests.pin(manifest)) {
        return openarm::runtime::record_error(
            owner, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
    }
    return openarm::runtime::record_error(
        owner, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
}
