/* SPDX-License-Identifier: Apache-2.0 */
#include "transport_internal.hpp"

#include <cstring>
#include <memory>
#include <new>
#include <string>

using openarm::transport::Event;
using openarm::transport::Filter;
using openarm::transport::Frame;
using openarm::transport::OpenConfig;
using openarm::transport::Transport;

struct oa_transport {
    std::unique_ptr<Transport> implementation;
};

namespace {

constexpr std::uint64_t kDefaultDeadlineHorizonNs = UINT64_C(5000000000);
constexpr std::uint64_t kMaximumDeadlineHorizonNs = UINT64_C(60000000000);

template <typename Record>
bool validRecord(const Record *record) noexcept {
    return record != nullptr && record->struct_size >= sizeof(Record) &&
           record->abi_version == OA_TRANSPORT_ABI_VERSION;
}

Frame toInternal(const oa_transport_frame &frame) noexcept {
    Frame result;
    result.can_id = frame.can_id;
    result.dlc = frame.dlc;
    std::memcpy(result.data, frame.data, sizeof(result.data));
    return result;
}

oa_transport_frame toPublic(const Frame &frame) noexcept {
    oa_transport_frame result{};
    result.struct_size = sizeof(result);
    result.abi_version = OA_TRANSPORT_ABI_VERSION;
    result.can_id = frame.can_id;
    result.dlc = frame.dlc;
    std::memcpy(result.data, frame.data, sizeof(result.data));
    return result;
}

oa_transport_status validateFrame(const oa_transport_frame *frame) noexcept {
    if (!validRecord(frame)) {
        return frame == nullptr ? OA_TRANSPORT_EINVAL : OA_TRANSPORT_EABI;
    }
    if (frame->dlc != 8U || frame->can_id > OA_TRANSPORT_SFF_MASK) {
        return OA_TRANSPORT_EFRAME;
    }
    return OA_TRANSPORT_OK;
}

} // namespace

extern "C" oa_transport_status
oa_transport_now_monotonic_ns(uint64_t *out_now_ns) {
    if (out_now_ns == nullptr) {
        return OA_TRANSPORT_EINVAL;
    }
    std::uint64_t now_ns = 0U;
    const auto status = openarm::transport::monotonicNow(now_ns);
    if (status == OA_TRANSPORT_OK) {
        *out_now_ns = now_ns;
    }
    return status;
}

extern "C" oa_transport_status oa_transport_classify_send_frame(
    const oa_transport_frame *frame, oa_transport_frame_class *out_class) {
    if (out_class == nullptr) {
        return OA_TRANSPORT_EINVAL;
    }
    const auto validation = validateFrame(frame);
    if (validation != OA_TRANSPORT_OK) {
        return validation;
    }
    oa_transport_frame_class frame_class = OA_TRANSPORT_FRAME_UNKNOWN;
    const auto status = openarm::transport::classify(toInternal(*frame), frame_class);
    if (status == OA_TRANSPORT_OK) {
        *out_class = frame_class;
    }
    return status;
}

extern "C" oa_transport_status oa_transport_open(
    const char *interface_name, const oa_transport_open_options *options,
    const oa_transport_filter *filters, size_t filter_count,
    const oa_transport_capability *capability, oa_transport **out_transport) {
    if (interface_name == nullptr || out_transport == nullptr ||
        filter_count > OA_TRANSPORT_MAX_FILTERS ||
        (filter_count != 0U && filters == nullptr)) {
        return OA_TRANSPORT_EINVAL;
    }
    const std::size_t name_length =
        ::strnlen(interface_name, OA_TRANSPORT_INTERFACE_NAME_CAPACITY);
    if (name_length == 0U ||
        name_length >= OA_TRANSPORT_INTERFACE_NAME_CAPACITY) {
        return OA_TRANSPORT_EINVAL;
    }

    OpenConfig config;
    config.max_deadline_horizon_ns = kDefaultDeadlineHorizonNs;
    if (options != nullptr) {
        if (!validRecord(options)) {
            return OA_TRANSPORT_EABI;
        }
        config.receive_buffer_bytes = options->receive_buffer_bytes;
        config.can_error_mask = options->can_error_mask == 0U
                                    ? OA_TRANSPORT_CAN_ERROR_MASK
                                    : options->can_error_mask;
        config.max_deadline_horizon_ns = options->max_deadline_horizon_ns == 0U
                                             ? kDefaultDeadlineHorizonNs
                                             : options->max_deadline_horizon_ns;
        if ((config.can_error_mask & ~OA_TRANSPORT_CAN_ERROR_MASK) != 0U ||
            config.max_deadline_horizon_ns > kMaximumDeadlineHorizonNs) {
            return OA_TRANSPORT_ERANGE;
        }
    }
    if (config.max_deadline_horizon_ns == 0U) {
        return OA_TRANSPORT_ERANGE;
    }

    try {
        config.filters.reserve(filter_count);
        for (std::size_t index = 0U; index < filter_count; ++index) {
            if (!validRecord(&filters[index])) {
                return OA_TRANSPORT_EABI;
            }
            if (filters[index].can_id > OA_TRANSPORT_SFF_MASK ||
                filters[index].can_mask > OA_TRANSPORT_SFF_MASK ||
                (filters[index].can_id & ~filters[index].can_mask) != 0U) {
                return OA_TRANSPORT_ERANGE;
            }
            config.filters.push_back(
                Filter{filters[index].can_id, filters[index].can_mask});
        }

        std::uint32_t permissions = OA_TRANSPORT_PERMISSION_QUERY;
        std::uint64_t capability_expiry_ns = 0U;
        if (capability != nullptr) {
            if (!validRecord(capability)) {
                return OA_TRANSPORT_EABI;
            }
            if (capability->reserved != 0U || capability->permissions == 0U ||
                (capability->permissions & ~OA_TRANSPORT_PERMISSION_ALL) != 0U) {
                return OA_TRANSPORT_EINVAL;
            }
            permissions = capability->permissions;
            capability_expiry_ns = capability->expiry_monotonic_ns;
            if ((permissions & ~OA_TRANSPORT_PERMISSION_QUERY) != 0U) {
                std::uint64_t now_ns = 0U;
                const auto time_status = openarm::transport::monotonicNow(now_ns);
                if (time_status != OA_TRANSPORT_OK) {
                    return time_status;
                }
                if (capability_expiry_ns <= now_ns) {
                    return OA_TRANSPORT_EPERMISSION;
                }
            }
        }

        oa_transport_status backend_status = OA_TRANSPORT_EIO;
        auto backend = openarm::transport::makeSocketCanBackend(
            std::string(interface_name, name_length), config, backend_status);
        if (!backend) {
            return backend_status;
        }
        auto implementation = std::make_unique<Transport>(
            std::move(backend), permissions, capability_expiry_ns,
            config.max_deadline_horizon_ns);
        auto handle = std::make_unique<oa_transport>();
        handle->implementation = std::move(implementation);
        *out_transport = handle.release();
        return OA_TRANSPORT_OK;
    } catch (const std::bad_alloc &) {
        return OA_TRANSPORT_ENOMEM;
    } catch (...) {
        return OA_TRANSPORT_EIO;
    }
}

extern "C" oa_transport_status oa_transport_close(oa_transport *transport) {
    if (transport == nullptr || !transport->implementation) {
        return OA_TRANSPORT_EINVAL;
    }
    transport->implementation->close();
    return OA_TRANSPORT_OK;
}

extern "C" void oa_transport_destroy(oa_transport *transport) {
    if (transport != nullptr) {
        if (transport->implementation) {
            transport->implementation->close();
        }
        delete transport;
    }
}

extern "C" oa_transport_status oa_transport_send(
    oa_transport *transport, const oa_transport_frame *frame,
    uint64_t deadline_monotonic_ns, oa_transport_send_result *out_result) {
    if (transport == nullptr || !transport->implementation ||
        out_result == nullptr) {
        return OA_TRANSPORT_EINVAL;
    }
    if (!validRecord(out_result)) {
        return OA_TRANSPORT_EABI;
    }
    const auto validation = validateFrame(frame);
    if (validation != OA_TRANSPORT_OK) {
        return validation;
    }
    try {
        oa_transport_frame_class frame_class = OA_TRANSPORT_FRAME_UNKNOWN;
        std::uint64_t sent_ns = 0U;
        const auto status = transport->implementation->send(
            toInternal(*frame), deadline_monotonic_ns, frame_class, sent_ns);
        if (status == OA_TRANSPORT_OK) {
            oa_transport_send_result result{};
            result.struct_size = sizeof(result);
            result.abi_version = OA_TRANSPORT_ABI_VERSION;
            result.frame_class = frame_class;
            result.sent_monotonic_ns = sent_ns;
            *out_result = result;
        }
        return status;
    } catch (...) {
        return OA_TRANSPORT_EIO;
    }
}

extern "C" oa_transport_status oa_transport_receive(
    oa_transport *transport, uint64_t deadline_monotonic_ns,
    oa_transport_event *out_event) {
    if (transport == nullptr || !transport->implementation ||
        out_event == nullptr) {
        return OA_TRANSPORT_EINVAL;
    }
    if (!validRecord(out_event)) {
        return OA_TRANSPORT_EABI;
    }
    try {
        Event event;
        const auto status =
            transport->implementation->receive(deadline_monotonic_ns, event);
        if (status == OA_TRANSPORT_OK) {
            oa_transport_event result{};
            result.struct_size = sizeof(result);
            result.abi_version = OA_TRANSPORT_ABI_VERSION;
            result.kind = event.kind;
            result.flags = event.flags;
            result.dequeue_monotonic_ns = event.dequeue_monotonic_ns;
            result.kernel_timestamp_ns = event.kernel_timestamp_ns;
            result.kernel_timestamp_clock = event.kernel_timestamp_clock;
            result.can_error_mask = event.can_error_mask;
            result.rx_overflow_count = event.rx_overflow_count;
            result.rx_overflow_delta = event.rx_overflow_delta;
            result.link_up = event.link_up ? UINT8_C(1) : UINT8_C(0);
            result.frame = toPublic(event.frame);
            *out_event = result;
        }
        return status;
    } catch (...) {
        return OA_TRANSPORT_EIO;
    }
}
