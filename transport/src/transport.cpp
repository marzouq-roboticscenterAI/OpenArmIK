/* SPDX-License-Identifier: Apache-2.0 */
#include "transport_internal.hpp"

#include <chrono>
namespace openarm::transport {
namespace {

bool allBytes(const Frame &frame, std::size_t first, std::size_t last,
              std::uint8_t value) noexcept {
    for (std::size_t index = first; index < last; ++index) {
        if (frame.data[index] != value) {
            return false;
        }
    }
    return true;
}

bool validTarget(const Frame &frame) noexcept {
    const auto target = static_cast<std::uint32_t>(frame.data[0]) |
                        (static_cast<std::uint32_t>(frame.data[1]) << 8U);
    return target > 0U && target <= OA_TRANSPORT_SFF_MASK;
}

bool knownRegister(std::uint8_t register_id) noexcept {
    return register_id <= UINT8_C(36) ||
           (register_id >= UINT8_C(50) && register_id <= UINT8_C(55)) ||
           (register_id >= UINT8_C(80) && register_id <= UINT8_C(81));
}

bool writableRegister(std::uint8_t register_id) noexcept {
    return register_id <= UINT8_C(10) ||
           (register_id >= UINT8_C(21) && register_id <= UINT8_C(35));
}

} // namespace

oa_transport_status monotonicNow(std::uint64_t &out_ns) noexcept {
    using Clock = std::chrono::steady_clock;
    const auto duration = Clock::now().time_since_epoch();
    const auto nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    if (nanoseconds < 0) {
        return OA_TRANSPORT_EIO;
    }
    out_ns = static_cast<std::uint64_t>(nanoseconds);
    return OA_TRANSPORT_OK;
}

oa_transport_status classify(const Frame &frame,
                             oa_transport_frame_class &out_class) noexcept {
    if (frame.dlc != 8U || frame.can_id > OA_TRANSPORT_SFF_MASK) {
        return OA_TRANSPORT_EFRAME;
    }
    if (frame.can_id == 0U) {
        out_class = OA_TRANSPORT_FRAME_UNKNOWN;
        return OA_TRANSPORT_OK;
    }

    if (allBytes(frame, 0U, 7U, UINT8_C(0xff))) {
        switch (frame.data[7]) {
        case UINT8_C(0xfc):
            out_class = OA_TRANSPORT_FRAME_ENABLE;
            return OA_TRANSPORT_OK;
        case UINT8_C(0xfd):
            out_class = OA_TRANSPORT_FRAME_DISABLE;
            return OA_TRANSPORT_OK;
        case UINT8_C(0xfe):
            out_class = OA_TRANSPORT_FRAME_SET_ZERO;
            return OA_TRANSPORT_OK;
        case UINT8_C(0xfb):
            out_class = OA_TRANSPORT_FRAME_CLEAR_ERROR;
            return OA_TRANSPORT_OK;
        default:
            out_class = OA_TRANSPORT_FRAME_UNKNOWN;
            return OA_TRANSPORT_OK;
        }
    }

    if (frame.can_id == OA_TRANSPORT_SFF_MASK) {
        if (frame.data[2] == UINT8_C(0x33)) {
            out_class = validTarget(frame) && knownRegister(frame.data[3]) &&
                                allBytes(frame, 4U, 8U, 0U)
                            ? OA_TRANSPORT_FRAME_REGISTER_QUERY
                            : OA_TRANSPORT_FRAME_UNKNOWN;
            return OA_TRANSPORT_OK;
        }
        if (frame.data[2] == UINT8_C(0xcc)) {
            out_class = validTarget(frame) && allBytes(frame, 3U, 8U, 0U)
                            ? OA_TRANSPORT_FRAME_STATUS_QUERY
                            : OA_TRANSPORT_FRAME_UNKNOWN;
            return OA_TRANSPORT_OK;
        }
        if (frame.data[2] == UINT8_C(0x55)) {
            out_class = validTarget(frame) && writableRegister(frame.data[3])
                            ? OA_TRANSPORT_FRAME_REGISTER_WRITE
                            : OA_TRANSPORT_FRAME_UNKNOWN;
            return OA_TRANSPORT_OK;
        }
        if (frame.data[2] == UINT8_C(0xaa)) {
            out_class = validTarget(frame) && allBytes(frame, 3U, 8U, 0U)
                            ? OA_TRANSPORT_FRAME_SAVE_PARAMETERS
                            : OA_TRANSPORT_FRAME_UNKNOWN;
            return OA_TRANSPORT_OK;
        }
        out_class = OA_TRANSPORT_FRAME_UNKNOWN;
        return OA_TRANSPORT_OK;
    }

    out_class = OA_TRANSPORT_FRAME_MOTION;
    return OA_TRANSPORT_OK;
}

Transport::Transport(std::unique_ptr<Backend> backend, std::uint32_t permissions,
                     std::uint64_t capability_expiry_ns,
                     std::uint64_t max_deadline_horizon_ns)
    : backend_(std::move(backend)), permissions_(permissions),
      capability_expiry_ns_(capability_expiry_ns),
      max_deadline_horizon_ns_(max_deadline_horizon_ns) {}

Transport::~Transport() { close(); }

oa_transport_status Transport::validateDeadline(std::uint64_t deadline_ns,
                                                 std::uint64_t now_ns) const noexcept {
    if (deadline_ns < now_ns) {
        return OA_TRANSPORT_ETIMEOUT;
    }
    if (deadline_ns - now_ns > max_deadline_horizon_ns_) {
        return OA_TRANSPORT_ERANGE;
    }
    return OA_TRANSPORT_OK;
}

bool Transport::permits(oa_transport_frame_class frame_class,
                        std::uint64_t now_ns) const noexcept {
    std::uint32_t needed = 0U;
    switch (frame_class) {
    case OA_TRANSPORT_FRAME_REGISTER_QUERY:
    case OA_TRANSPORT_FRAME_STATUS_QUERY:
        needed = OA_TRANSPORT_PERMISSION_QUERY;
        break;
    case OA_TRANSPORT_FRAME_DISABLE:
        needed = OA_TRANSPORT_PERMISSION_DISABLE;
        if ((permissions_ & (OA_TRANSPORT_PERMISSION_CONTROL |
                             OA_TRANSPORT_PERMISSION_COMMISSION)) != 0U) {
            needed = 0U;
        }
        break;
    case OA_TRANSPORT_FRAME_ENABLE:
    case OA_TRANSPORT_FRAME_MOTION:
        needed = OA_TRANSPORT_PERMISSION_CONTROL;
        break;
    case OA_TRANSPORT_FRAME_REGISTER_WRITE:
    case OA_TRANSPORT_FRAME_SET_ZERO:
    case OA_TRANSPORT_FRAME_CLEAR_ERROR:
    case OA_TRANSPORT_FRAME_SAVE_PARAMETERS:
        needed = OA_TRANSPORT_PERMISSION_COMMISSION;
        break;
    default:
        return false;
    }
    if (needed != 0U && (permissions_ & needed) == 0U) {
        return false;
    }
    const auto dangerous = permissions_ & ~OA_TRANSPORT_PERMISSION_QUERY;
    return dangerous == 0U ||
           (capability_expiry_ns_ != 0U && now_ns < capability_expiry_ns_);
}

oa_transport_status Transport::send(const Frame &frame, std::uint64_t deadline_ns,
                                    oa_transport_frame_class &out_class,
                                    std::uint64_t &out_sent_ns) noexcept {
    std::lock_guard<std::mutex> lock(send_mutex_);
    if (closed_.load(std::memory_order_acquire)) {
        return OA_TRANSPORT_ECLOSED;
    }
    auto status = classify(frame, out_class);
    if (status != OA_TRANSPORT_OK) {
        return status;
    }
    std::uint64_t now_ns = 0U;
    status = backend_->now(now_ns);
    if (status != OA_TRANSPORT_OK) {
        return status;
    }
    status = validateDeadline(deadline_ns, now_ns);
    if (status != OA_TRANSPORT_OK) {
        return status;
    }
    if (!permits(out_class, now_ns)) {
        return OA_TRANSPORT_EPERMISSION;
    }
    if ((permissions_ & ~OA_TRANSPORT_PERMISSION_QUERY) != 0U &&
        deadline_ns > capability_expiry_ns_) {
        return OA_TRANSPORT_EPERMISSION;
    }
    return backend_->send(frame, deadline_ns, out_sent_ns);
}

oa_transport_status Transport::receive(std::uint64_t deadline_ns,
                                       Event &out_event) noexcept {
    std::lock_guard<std::mutex> lock(receive_mutex_);
    if (closed_.load(std::memory_order_acquire)) {
        return OA_TRANSPORT_ECLOSED;
    }
    std::uint64_t now_ns = 0U;
    auto status = backend_->now(now_ns);
    if (status != OA_TRANSPORT_OK) {
        return status;
    }
    status = validateDeadline(deadline_ns, now_ns);
    if (status != OA_TRANSPORT_OK) {
        return status;
    }
    return backend_->receive(deadline_ns, out_event);
}

void Transport::close() noexcept {
    if (!closed_.exchange(true, std::memory_order_acq_rel) && backend_) {
        backend_->close();
    }
}

} // namespace openarm::transport
