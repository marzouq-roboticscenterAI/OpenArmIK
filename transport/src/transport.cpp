/* SPDX-License-Identifier: Apache-2.0 */
#include "transport_internal.hpp"

#include "openarm_can.h"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <new>
#include <random>

#ifdef __linux__
#include <sys/random.h>
#endif

namespace openarm::transport {
namespace {

constexpr std::uint64_t kMaximumAuthorityLifetimeNs = UINT64_C(5000000000);

bool allBytes(const Frame &frame, std::size_t first, std::size_t last,
              std::uint8_t value) noexcept {
    for (std::size_t index = first; index < last; ++index) {
        if (frame.data[index] != value) {
            return false;
        }
    }
    return true;
}

std::uint16_t targetId(const Frame &frame) noexcept {
    return static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(frame.data[0]) |
        static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(frame.data[1]) << 8U));
}

bool validMotorTarget(std::uint16_t target) noexcept {
    return target > 0U && target < OA_TRANSPORT_SFF_MASK;
}

std::uint32_t readU32(const std::uint8_t *data) noexcept {
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8U) |
           (static_cast<std::uint32_t>(data[2]) << 16U) |
           (static_cast<std::uint32_t>(data[3]) << 24U);
}

bool knownRegister(std::uint8_t register_id) noexcept {
    oa_can_register_info info{};
    info.struct_size = sizeof(info);
    info.abi_version = OA_CAN_ABI_VERSION;
    return oa_can_register_info_for_id(register_id, &info) == OA_CAN_OK;
}

bool validRegisterWrite(const Frame &frame) noexcept {
    const auto target = targetId(frame);
    if (!validMotorTarget(target)) {
        return false;
    }
    oa_can_register_info info{};
    info.struct_size = sizeof(info);
    info.abi_version = OA_CAN_ABI_VERSION;
    if (oa_can_register_info_for_id(frame.data[3], &info) != OA_CAN_OK ||
        info.writable == 0U) {
        return false;
    }
    oa_can_register_write write{};
    write.struct_size = sizeof(write);
    write.abi_version = OA_CAN_ABI_VERSION;
    write.send_id = target;
    write.register_id = frame.data[3];
    write.value_type = info.value_type;
    const std::uint32_t raw = readU32(&frame.data[4]);
    if (info.value_type == OA_CAN_REGISTER_U32) {
        write.value_u32 = raw;
    } else if (info.value_type == OA_CAN_REGISTER_F32) {
        std::memcpy(&write.value_f32, &raw, sizeof(raw));
    } else {
        return false;
    }
    oa_can_frame expected{};
    expected.struct_size = sizeof(expected);
    expected.abi_version = OA_CAN_ABI_VERSION;
    if (oa_can_make_register_write(&write, &expected) != OA_CAN_OK ||
        expected.can_id != frame.can_id || expected.dlc != frame.dlc) {
        return false;
    }
    return std::memcmp(expected.data, frame.data, sizeof(frame.data)) == 0;
}

bool framesEqual(const Frame &left, const Frame &right) noexcept {
    return left.can_id == right.can_id && left.dlc == right.dlc &&
           std::memcmp(left.data, right.data, sizeof(left.data)) == 0;
}

bool dangerousClass(oa_transport_frame_class frame_class) noexcept {
    return frame_class == OA_TRANSPORT_FRAME_DISABLE ||
           frame_class == OA_TRANSPORT_FRAME_ENABLE ||
           frame_class == OA_TRANSPORT_FRAME_REGISTER_WRITE ||
           frame_class == OA_TRANSPORT_FRAME_SET_ZERO ||
           frame_class == OA_TRANSPORT_FRAME_CLEAR_ERROR ||
           frame_class == OA_TRANSPORT_FRAME_SAVE_PARAMETERS;
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

oa_transport_status secureRandom64(std::uint64_t &out_value) noexcept {
#ifdef __linux__
    while (true) {
        const ssize_t count = ::getrandom(&out_value, sizeof(out_value), 0U);
        if (count == static_cast<ssize_t>(sizeof(out_value))) {
            return out_value == 0U ? OA_TRANSPORT_EIO : OA_TRANSPORT_OK;
        }
        if (count < 0 && errno == EINTR) {
            continue;
        }
        return OA_TRANSPORT_EIO;
    }
#else
    try {
        std::random_device source;
        out_value = (static_cast<std::uint64_t>(source()) << 32U) ^
                    static_cast<std::uint64_t>(source());
        return out_value == 0U ? OA_TRANSPORT_EIO : OA_TRANSPORT_OK;
    } catch (...) {
        return OA_TRANSPORT_EIO;
    }
#endif
}

oa_transport_status classify(const Frame &frame,
                             oa_transport_frame_class &out_class) noexcept {
    if (frame.dlc != 8U || frame.can_id > OA_TRANSPORT_SFF_MASK) {
        return OA_TRANSPORT_EFRAME;
    }
    out_class = OA_TRANSPORT_FRAME_UNKNOWN;
    if (frame.can_id == 0U) {
        return OA_TRANSPORT_OK;
    }

    if (frame.can_id == OA_TRANSPORT_SFF_MASK) {
        const auto target = targetId(frame);
        if (frame.data[2] == UINT8_C(0x33) && validMotorTarget(target) &&
            knownRegister(frame.data[3]) && allBytes(frame, 4U, 8U, 0U)) {
            out_class = OA_TRANSPORT_FRAME_REGISTER_QUERY;
        } else if (frame.data[2] == UINT8_C(0xcc) &&
                   validMotorTarget(target) && allBytes(frame, 3U, 8U, 0U)) {
            out_class = OA_TRANSPORT_FRAME_STATUS_QUERY;
        } else if (frame.data[2] == UINT8_C(0x55) &&
                   validRegisterWrite(frame)) {
            out_class = OA_TRANSPORT_FRAME_REGISTER_WRITE;
        } else if (frame.data[2] == UINT8_C(0xaa) &&
                   validMotorTarget(target) && allBytes(frame, 3U, 8U, 0U)) {
            out_class = OA_TRANSPORT_FRAME_SAVE_PARAMETERS;
        }
        return OA_TRANSPORT_OK;
    }

    if (allBytes(frame, 0U, 7U, UINT8_C(0xff))) {
        switch (frame.data[7]) {
        case UINT8_C(0xfc):
            out_class = OA_TRANSPORT_FRAME_ENABLE;
            break;
        case UINT8_C(0xfd):
            out_class = OA_TRANSPORT_FRAME_DISABLE;
            break;
        case UINT8_C(0xfe):
            out_class = OA_TRANSPORT_FRAME_SET_ZERO;
            break;
        case UINT8_C(0xfb):
            out_class = OA_TRANSPORT_FRAME_CLEAR_ERROR;
            break;
        default:
            break;
        }
    }
    return OA_TRANSPORT_OK;
}

Transport::Transport(std::unique_ptr<Backend> backend,
                     std::array<std::uint64_t, 2> instance_nonce,
                     std::uint64_t max_deadline_horizon_ns)
    : backend_(std::move(backend)), instance_nonce_(instance_nonce),
      max_deadline_horizon_ns_(max_deadline_horizon_ns) {}

Transport::~Transport() { close(); }

oa_transport_status Transport::validateDeadline(std::uint64_t deadline_ns,
                                                 std::uint64_t now_ns) const noexcept {
    if (deadline_ns <= now_ns) {
        return OA_TRANSPORT_ETIMEOUT;
    }
    if (deadline_ns - now_ns > max_deadline_horizon_ns_) {
        return OA_TRANSPORT_ERANGE;
    }
    return OA_TRANSPORT_OK;
}

oa_transport_status Transport::consumeAuthority(
    const Authority *authority, const Frame &frame,
    oa_transport_frame_class frame_class, std::uint64_t now_ns,
    std::uint64_t deadline_ns) noexcept {
    if (authority == nullptr || authority->instance_nonce_ != instance_nonce_) {
        return OA_TRANSPORT_EPERMISSION;
    }
    std::lock_guard<std::mutex> lock(authority_mutex_);
    const auto found = grants_.find(authority->token_id_);
    if (found == grants_.end()) {
        return OA_TRANSPORT_EPERMISSION;
    }
    const Grant grant = found->second;
    if (grant.expiry_ns <= now_ns) {
        grants_.erase(found);
        return OA_TRANSPORT_EPERMISSION;
    }
    if (deadline_ns > grant.expiry_ns || grant.frame_class != frame_class ||
        !framesEqual(grant.exact_frame, frame)) {
        return OA_TRANSPORT_EPERMISSION;
    }
    grants_.erase(found);
    return OA_TRANSPORT_OK;
}

oa_transport_status Transport::send(const Frame &frame, std::uint64_t deadline_ns,
                                    const Authority *authority,
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
    if (out_class == OA_TRANSPORT_FRAME_UNKNOWN ||
        out_class == OA_TRANSPORT_FRAME_MOTION) {
        return OA_TRANSPORT_EFRAME;
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
    if (out_class == OA_TRANSPORT_FRAME_REGISTER_QUERY ||
        out_class == OA_TRANSPORT_FRAME_STATUS_QUERY) {
        if (authority != nullptr) {
            return OA_TRANSPORT_EPERMISSION;
        }
    } else {
        status = consumeAuthority(authority, frame, out_class, now_ns,
                                  deadline_ns);
        if (status != OA_TRANSPORT_OK) {
            return status;
        }
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

oa_transport_status Transport::issueAuthority(
    const Frame &exact_frame, std::uint64_t expiry_ns,
    std::unique_ptr<Authority> &out_authority) {
    std::lock_guard<std::mutex> close_lock(close_mutex_);
    if (closed_.load(std::memory_order_acquire)) {
        return OA_TRANSPORT_ECLOSED;
    }
    if (!backend_->permitsAuthorityIssuance()) {
        return OA_TRANSPORT_EPERMISSION;
    }
    oa_transport_frame_class frame_class = OA_TRANSPORT_FRAME_UNKNOWN;
    auto status = classify(exact_frame, frame_class);
    if (status != OA_TRANSPORT_OK || !dangerousClass(frame_class)) {
        return status == OA_TRANSPORT_OK ? OA_TRANSPORT_EFRAME : status;
    }
    std::uint64_t now_ns = 0U;
    status = backend_->now(now_ns);
    if (status != OA_TRANSPORT_OK) {
        return status;
    }
    if (expiry_ns <= now_ns || expiry_ns - now_ns > kMaximumAuthorityLifetimeNs) {
        return OA_TRANSPORT_ERANGE;
    }
    try {
        std::uint64_t token_id = 0U;
        std::lock_guard<std::mutex> authority_lock(authority_mutex_);
        do {
            status = secureRandom64(token_id);
            if (status != OA_TRANSPORT_OK) {
                return status;
            }
        } while (grants_.find(token_id) != grants_.end());
        grants_.emplace(token_id, Grant{exact_frame, frame_class, expiry_ns});
        try {
            out_authority =
                std::unique_ptr<Authority>(new Authority(instance_nonce_, token_id));
        } catch (...) {
            grants_.erase(token_id);
            throw;
        }
        return OA_TRANSPORT_OK;
    } catch (const std::bad_alloc &) {
        return OA_TRANSPORT_ENOMEM;
    } catch (...) {
        return OA_TRANSPORT_EIO;
    }
}

void Transport::close() noexcept {
    std::lock_guard<std::mutex> close_lock(close_mutex_);
    closed_.store(true, std::memory_order_release);
    if (backend_) {
        backend_->close();
    }
    {
        std::scoped_lock operation_lock(send_mutex_, receive_mutex_);
    }
    std::lock_guard<std::mutex> authority_lock(authority_mutex_);
    grants_.clear();
}

} // namespace openarm::transport
