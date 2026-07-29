/* SPDX-License-Identifier: Apache-2.0 */
#include "runtime_internal.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>

namespace openarm::runtime {
namespace {

constexpr std::array<oa_can_register_id, 14> kRegisterIds{
    OA_CAN_RID_SERIAL_NUMBER, OA_CAN_RID_HW_VER, OA_CAN_RID_SW_VER,
    OA_CAN_RID_SUB_VER, OA_CAN_RID_ESC_ID, OA_CAN_RID_MST_ID,
    OA_CAN_RID_CTRL_MODE, OA_CAN_RID_CAN_BR, OA_CAN_RID_TIMEOUT,
    OA_CAN_RID_PMAX, OA_CAN_RID_VMAX, OA_CAN_RID_TMAX,
    OA_CAN_RID_GEAR_RATIO, OA_CAN_RID_DIRECTION};

struct TransportDeleter {
    using Cleanup = void (*)(oa_transport *);
    Cleanup cleanup{};

    void operator()(oa_transport *transport) const noexcept {
        if (transport == nullptr) return;
        if (cleanup != nullptr) {
            cleanup(transport);
            return;
        }
        oa_transport_close(transport);
        oa_transport_destroy(transport);
    }
};

using TransportPtr = std::unique_ptr<oa_transport, TransportDeleter>;

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
            motor.register_presence_mask = (UINT64_C(1) << kRegisterIds.size()) - 1U;
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
    std::array<oa_can_linux_interface, OA_RUNTIME_MAX_INTERFACES> lower{};
    for (auto &item : lower) {
        item.struct_size = sizeof(item);
        item.abi_version = OA_CAN_ABI_VERSION;
    }
    std::size_t count = 0U;
    if (oa_can_linux_list_interfaces(lower.data(), lower.size(), &count) != OA_CAN_OK) return {};
    count = std::min(count, lower.size());
    std::vector<oa_runtime_interface> result;
    result.reserve(count);
    for (std::size_t i = 0U; i < count; ++i) {
        oa_runtime_interface item{};
        runtime_init(item);
        std::snprintf(item.name, sizeof(item.name), "%s", lower[i].name);
        item.ifindex = lower[i].ifindex; item.flags = lower[i].flags;
        item.mtu = lower[i].mtu; item.bitrate = lower[i].bitrate;
        item.data_bitrate = lower[i].data_bitrate; item.link_up = lower[i].link_up;
        item.fd_enabled = lower[i].fd_enabled;
        item.interface_kind = OA_RUNTIME_INTERFACE_KIND_UNKNOWN;
        result.push_back(item);
    }
    return result;
}

void store_value(oa_runtime_motor_evidence &motor, std::size_t index,
                 const oa_can_register_value &value) {
    motor.register_presence_mask |= UINT64_C(1) << index;
    switch (kRegisterIds[index]) {
    case OA_CAN_RID_SERIAL_NUMBER: motor.serial_number = value.value_u32; break;
    case OA_CAN_RID_HW_VER: motor.hardware_version = value.value_u32; break;
    case OA_CAN_RID_SW_VER: motor.software_version = value.value_u32; break;
    case OA_CAN_RID_SUB_VER: motor.firmware_subversion = value.value_u32; break;
    case OA_CAN_RID_ESC_ID: motor.configured_send_id = value.value_u32; break;
    case OA_CAN_RID_MST_ID: motor.configured_receive_id = value.value_u32; break;
    case OA_CAN_RID_CTRL_MODE: motor.control_mode = value.value_u32; break;
    case OA_CAN_RID_CAN_BR: motor.bitrate = value.value_u32; break;
    case OA_CAN_RID_TIMEOUT: motor.timeout_ticks = value.value_u32; break;
    case OA_CAN_RID_DIRECTION: motor.direction = static_cast<std::int32_t>(value.value_u32); break;
    case OA_CAN_RID_PMAX: motor.pmax_rad = value.value_f32; break;
    case OA_CAN_RID_VMAX: motor.vmax_rad_s = value.value_f32; break;
    case OA_CAN_RID_TMAX: motor.tmax_nm = value.value_f32; break;
    case OA_CAN_RID_GEAR_RATIO: motor.gear_ratio = value.value_f32; break;
    default: break;
    }
}

}
}

#ifdef OA_RUNTIME_ENABLE_TEST_HOOKS
namespace {
std::atomic<std::uint32_t> transport_probe_cleanups{0U};

void transport_probe_cleanup(oa_transport *) {
    transport_probe_cleanups.fetch_add(1U, std::memory_order_relaxed);
}
}

extern "C" oa_runtime_status oa_runtime_test_transport_raii_probe(void) {
    transport_probe_cleanups.store(0U, std::memory_order_relaxed);
    try {
        openarm::runtime::TransportPtr lease(
            reinterpret_cast<oa_transport *>(static_cast<std::uintptr_t>(0x10U)),
            openarm::runtime::TransportDeleter{transport_probe_cleanup});
        throw std::bad_alloc();
    } catch (const std::bad_alloc &) {
        return transport_probe_cleanups.load(std::memory_order_relaxed) == 1U
                   ? OA_RUNTIME_OK
                   : OA_RUNTIME_EFAULT;
    } catch (...) {
        return OA_RUNTIME_EFAULT;
    }
}
#endif

extern "C" oa_runtime_status oa_runtime_list_interfaces(
    const oa_runtime *runtime, oa_runtime_interface *interfaces,
    std::size_t capacity, std::size_t *out_count) {
    if (out_count == nullptr || (capacity != 0U && interfaces == nullptr)) return OA_RUNTIME_EINVAL;
    const auto pinned = openarm::runtime::runtimes.pin(runtime);
    if (!pinned) return OA_RUNTIME_EINVAL;
    std::vector<oa_runtime_interface> found;
    try {
        if (pinned->options.backend == OA_RUNTIME_BACKEND_VIRTUAL) {
            found = openarm::runtime::virtual_inventory(openarm::runtime::now_ns())->interfaces;
        } else if (pinned->options.backend == OA_RUNTIME_BACKEND_SOCKETCAN_QUERY) {
            found = openarm::runtime::physical_interfaces();
        }
    } catch (...) {
        return OA_RUNTIME_ENOMEM;
    }
    *out_count = found.size();
    if (capacity < found.size()) return OA_RUNTIME_EINVAL;
    for (std::size_t i = 0U; i < found.size(); ++i) {
        if (!openarm::runtime::output_valid(&interfaces[i])) return OA_RUNTIME_EABI;
    }
    for (std::size_t i = 0U; i < found.size(); ++i) interfaces[i] = found[i];
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_inventory_query(
    oa_runtime *runtime, const oa_runtime_inventory_query_options *options,
    oa_runtime_inventory **out_inventory) {
    if (out_inventory == nullptr) return OA_RUNTIME_EINVAL;
    *out_inventory = nullptr;
    const auto owner = openarm::runtime::runtimes.pin(runtime);
    if (!owner) return OA_RUNTIME_EINVAL;
    try {
        std::shared_ptr<openarm::runtime::InventoryData> result;
        if (owner->options.backend == OA_RUNTIME_BACKEND_VIRTUAL) {
            const std::uint64_t timestamp = openarm::runtime::now_ns();
            result = openarm::runtime::virtual_inventory(timestamp);
        } else if (owner->options.backend == OA_RUNTIME_BACKEND_SOCKETCAN_QUERY) {
            if (options == nullptr || options->struct_size < sizeof(*options) ||
                options->abi_version != OA_RUNTIME_ABI_VERSION) {
                return openarm::runtime::record_error(
                    owner, OA_RUNTIME_EABI, OA_RUNTIME_FACILITY_RUNTIME);
            }
            if (options->candidate_count > OA_RUNTIME_MAX_QUERY_MOTORS ||
                options->per_query_timeout_ns == 0U ||
                options->per_query_timeout_ns > 1000000000U ||
                options->maximum_received_frames == 0U ||
                std::memchr(options->interface_name, '\0', sizeof(options->interface_name)) == nullptr) {
                return openarm::runtime::record_error(
                    owner, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
            }
            result = std::make_shared<openarm::runtime::InventoryData>();
            const auto available = openarm::runtime::physical_interfaces();
            const auto match = std::find_if(available.begin(), available.end(),
                [options](const oa_runtime_interface &item) {
                    return std::strcmp(item.name, options->interface_name) == 0;
                });
            openarm::runtime::runtime_init(result->summary);
            result->summary.clock_id = OA_RUNTIME_CLOCK_MONOTONIC;
            result->summary.inventory_revision = openarm::runtime::now_ns();
            if (match != available.end()) {
                result->interfaces.push_back(*match);
                std::vector<oa_transport_filter> filters;
                filters.reserve(options->candidate_count);
                for (std::uint32_t i = 0U; i < options->candidate_count; ++i) {
                    if (options->candidate[i].struct_size < sizeof(oa_runtime_query_candidate) ||
                        options->candidate[i].abi_version != OA_RUNTIME_ABI_VERSION) {
                        return openarm::runtime::record_error(
                            owner, OA_RUNTIME_EABI, OA_RUNTIME_FACILITY_RUNTIME);
                    }
                    if (options->candidate[i].send_id == 0U ||
                        options->candidate[i].send_id >= 0x7ffU ||
                        options->candidate[i].receive_id == 0U ||
                        options->candidate[i].receive_id >= 0x7ffU) {
                        return openarm::runtime::record_error(
                            owner, OA_RUNTIME_EINVAL, OA_RUNTIME_FACILITY_RUNTIME);
                    }
                    oa_transport_filter filter{};
                    filter.struct_size = sizeof(filter);
                    filter.abi_version = OA_TRANSPORT_ABI_VERSION;
                    filter.can_id = options->candidate[i].receive_id;
                    filter.can_mask = OA_TRANSPORT_SFF_MASK;
                    filters.push_back(filter);
                }
                oa_transport_open_options open_options{};
                open_options.struct_size = sizeof(open_options);
                open_options.abi_version = OA_TRANSPORT_ABI_VERSION;
                open_options.receive_buffer_bytes = 262144U;
                open_options.can_error_mask = OA_TRANSPORT_CAN_ERROR_MASK;
                open_options.max_deadline_horizon_ns = 2000000000U;
                oa_transport *raw_transport = nullptr;
                const oa_transport_status opened = oa_transport_open(
                    options->interface_name, &open_options,
                    filters.empty() ? nullptr : filters.data(), filters.size(), &raw_transport);
                if (opened != OA_TRANSPORT_OK) {
                    return openarm::runtime::record_error(
                        owner, openarm::runtime::map_transport(opened),
                        OA_RUNTIME_FACILITY_TRANSPORT, opened);
                }
                openarm::runtime::TransportPtr transport(raw_transport);
                openarm::runtime::allocation_checkpoint();
                for (std::uint32_t candidate_index = 0U;
                     candidate_index < options->candidate_count; ++candidate_index) {
                    const auto &candidate = options->candidate[candidate_index];
                    oa_runtime_motor_evidence motor{};
                    openarm::runtime::runtime_init(motor);
                    std::snprintf(motor.interface_name, sizeof(motor.interface_name), "%s",
                                  options->interface_name);
                    motor.requested_send_id = candidate.send_id;
                    motor.expected_receive_id = candidate.receive_id;
                    motor.side = UINT32_MAX;
                    motor.joint = UINT32_MAX;
                    motor.unresolved_assignment = 1U;
                    std::uint32_t received = 0U;
                    for (std::size_t register_index = 0U;
                         register_index < openarm::runtime::kRegisterIds.size(); ++register_index) {
                        oa_can_register_info info{};
                        info.struct_size = sizeof(info); info.abi_version = OA_CAN_ABI_VERSION;
                        if (oa_can_register_info_for_id(openarm::runtime::kRegisterIds[register_index],
                                                        &info) != OA_CAN_OK) continue;
                        oa_can_register_request request{};
                        request.struct_size = sizeof(request); request.abi_version = OA_CAN_ABI_VERSION;
                        request.send_id = candidate.send_id; request.receive_id = candidate.receive_id;
                        request.register_id = openarm::runtime::kRegisterIds[register_index];
                        request.value_type = info.value_type;
                        oa_can_frame can_frame{};
                        can_frame.struct_size = sizeof(can_frame); can_frame.abi_version = OA_CAN_ABI_VERSION;
                        if (oa_can_make_register_query_typed(&request, &can_frame) != OA_CAN_OK) continue;
                        oa_transport_frame frame{};
                        frame.struct_size = sizeof(frame); frame.abi_version = OA_TRANSPORT_ABI_VERSION;
                        frame.can_id = can_frame.can_id; frame.dlc = can_frame.dlc;
                        std::copy_n(can_frame.data, 8U, frame.data);
                        const std::uint64_t before = openarm::runtime::now_ns();
                        if (options->per_query_timeout_ns > UINT64_MAX - before) continue;
                        const std::uint64_t deadline = before + options->per_query_timeout_ns;
                        oa_transport_send_result sent{};
                        sent.struct_size = sizeof(sent); sent.abi_version = OA_TRANSPORT_ABI_VERSION;
                        const oa_transport_status send_status =
                            oa_transport_send(transport.get(), &frame, deadline, &sent);
                        if (send_status != OA_TRANSPORT_OK ||
                            sent.frame_class != OA_TRANSPORT_FRAME_REGISTER_QUERY) continue;
                        motor.query_sent_runtime_monotonic_ns = sent.sent_monotonic_ns;
                        while (received < options->maximum_received_frames) {
                            oa_transport_event event{};
                            event.struct_size = sizeof(event); event.abi_version = OA_TRANSPORT_ABI_VERSION;
                            const oa_transport_status receive_status =
                                oa_transport_receive(transport.get(), deadline, &event);
                            if (receive_status == OA_TRANSPORT_ETIMEOUT) break;
                            if (receive_status != OA_TRANSPORT_OK) { motor.stale_observed = 1U; break; }
                            ++received;
                            if (event.kind != OA_TRANSPORT_EVENT_FRAME) {
                                motor.fault_observed = 1U;
                                continue;
                            }
                            oa_can_frame response{};
                            response.struct_size = sizeof(response); response.abi_version = OA_CAN_ABI_VERSION;
                            response.can_id = event.frame.can_id; response.dlc = event.frame.dlc;
                            std::copy_n(event.frame.data, 8U, response.data);
                            oa_can_register_value value{};
                            value.struct_size = sizeof(value); value.abi_version = OA_CAN_ABI_VERSION;
                            if (oa_can_decode_register_response(&response, &request,
                                OA_CAN_REGISTER_QUERY, &value) == OA_CAN_OK) {
                                motor.observed_receive_id = value.receive_id;
                                motor.response_runtime_monotonic_ns = event.dequeue_monotonic_ns;
                                openarm::runtime::store_value(motor, register_index, value);
                                break;
                            }
                        }
                    }
                    motor.confidence = OA_RUNTIME_EVIDENCE_AMBIGUOUS;
                    std::snprintf(motor.immutable_identity, sizeof(motor.immutable_identity),
                                  "QUERY-%s-%u-%u", options->interface_name,
                                  candidate.send_id, motor.serial_number);
                    result->motors.push_back(motor);
                }
            }
            result->summary.interface_count =
                static_cast<std::uint32_t>(result->interfaces.size());
            result->summary.motor_count =
                static_cast<std::uint32_t>(result->motors.size());
            result->summary.unresolved_assignment = result->motors.empty() ? 0U : 1U;
            std::string evidence;
            for (std::size_t i = 0U; i < result->motors.size(); ++i) {
                evidence += result->motors[i].immutable_identity;
                evidence += ':';
                evidence += std::to_string(result->motors[i].register_presence_mask);
                evidence += '\n';
                if (i < 32U) {
                    result->summary.ambiguous_mask |= UINT32_C(1) << i;
                    result->summary.unknown_mask |= UINT32_C(1) << i;
                }
            }
            const std::string digest = openarm::runtime::sha256_hex(evidence);
            std::snprintf(result->summary.fingerprint_sha256,
                          sizeof(result->summary.fingerprint_sha256), "%s", digest.c_str());
        } else {
            return openarm::runtime::record_error(
                owner, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
        }
        oa_runtime_inventory *const handle = openarm::runtime::inventories.insert(result);
        if (handle == nullptr) return OA_RUNTIME_ENOMEM;
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
    const auto manifest_data = openarm::runtime::manifests.pin(manifest);
    const auto inventory_data = openarm::runtime::inventories.pin(inventory);
    if (!manifest_data || !inventory_data) return OA_RUNTIME_EINVAL;
    oa_runtime_manifest_preview result{};
    openarm::runtime::runtime_init(result);
    result.base_revision = manifest_data->config.manifest_revision;
    result.result_revision = manifest_data->config.manifest_revision;
    result.valid = inventory_data->summary.motor_count == 14U &&
                   inventory_data->summary.unresolved_assignment == 0U &&
                   inventory_data->summary.ambiguous_mask == 0U ? 1U : 0U;
    result.identity_change_mask = result.valid != 0U ? 0U : 0x3fffU;
    result.validation_status = result.valid != 0U ? OA_RUNTIME_OK : OA_RUNTIME_EIDENTITY;
    result.would_be_armable = 0U;
    *out_preview = result;
    return OA_RUNTIME_OK;
}

extern "C" oa_runtime_status oa_runtime_configuration_apply_physical(
    oa_runtime *runtime, const oa_runtime_manifest *manifest) {
    const auto owner = openarm::runtime::runtimes.pin(runtime);
    if (!owner || !openarm::runtime::manifests.pin(manifest)) return OA_RUNTIME_EINVAL;
    return openarm::runtime::record_error(
        owner, OA_RUNTIME_EUNSUPPORTED, OA_RUNTIME_FACILITY_RUNTIME);
}
