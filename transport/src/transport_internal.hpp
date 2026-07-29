/* SPDX-License-Identifier: Apache-2.0 */
#ifndef OPENARM_TRANSPORT_INTERNAL_HPP
#define OPENARM_TRANSPORT_INTERNAL_HPP

#include "openarm_transport.h"

#include <atomic>
#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace openarm::transport {

struct Frame {
    std::uint32_t can_id{};
    std::uint8_t dlc{};
    std::uint8_t data[8]{};
};

struct Event {
    oa_transport_event_kind kind{OA_TRANSPORT_EVENT_FRAME};
    std::uint32_t flags{};
    std::uint64_t dequeue_monotonic_ns{};
    std::uint64_t kernel_timestamp_ns{};
    oa_transport_clock kernel_timestamp_clock{OA_TRANSPORT_CLOCK_NONE};
    std::uint32_t can_error_mask{};
    std::uint32_t rx_overflow_count{};
    std::uint32_t rx_overflow_delta{};
    bool link_up{};
    Frame frame{};
};

struct Filter {
    std::uint32_t can_id{};
    std::uint32_t can_mask{};
};

struct OpenConfig {
    std::uint32_t receive_buffer_bytes{};
    std::uint32_t can_error_mask{OA_TRANSPORT_CAN_ERROR_MASK};
    std::uint64_t max_deadline_horizon_ns{};
    std::vector<Filter> filters;
};

class Backend {
public:
    virtual ~Backend() = default;
    virtual oa_transport_status now(std::uint64_t &out_ns) noexcept = 0;
    virtual oa_transport_status send(const Frame &frame, std::uint64_t deadline_ns,
                                     std::uint64_t &out_sent_ns) noexcept = 0;
    virtual oa_transport_status receive(std::uint64_t deadline_ns,
                                        Event &out_event) noexcept = 0;
    virtual void close() noexcept = 0;
    virtual bool permitsAuthorityIssuance() const noexcept = 0;
};

std::unique_ptr<Backend> makeSocketCanBackend(const std::string &interface_name,
                                              const OpenConfig &config,
                                              oa_transport_status &out_status);

oa_transport_status monotonicNow(std::uint64_t &out_ns) noexcept;
oa_transport_status secureRandom64(std::uint64_t &out_value) noexcept;
oa_transport_status classify(const Frame &frame,
                             oa_transport_frame_class &out_class) noexcept;

class Authority final {
public:
    Authority(const Authority &) = delete;
    Authority &operator=(const Authority &) = delete;

private:
    friend class Transport;
    Authority(std::array<std::uint64_t, 2> instance_nonce,
              std::uint64_t token_id) noexcept
        : instance_nonce_(instance_nonce), token_id_(token_id) {}

    std::array<std::uint64_t, 2> instance_nonce_{};
    std::uint64_t token_id_{};
};

class Transport final {
public:
    Transport(std::unique_ptr<Backend> backend,
              std::array<std::uint64_t, 2> instance_nonce,
              std::uint64_t max_deadline_horizon_ns);
    ~Transport();

    Transport(const Transport &) = delete;
    Transport &operator=(const Transport &) = delete;

    oa_transport_status send(const Frame &frame, std::uint64_t deadline_ns,
                             const Authority *authority,
                             oa_transport_frame_class &out_class,
                             std::uint64_t &out_sent_ns) noexcept;
    oa_transport_status receive(std::uint64_t deadline_ns,
                                Event &out_event) noexcept;
    void close() noexcept;

    oa_transport_status issueAuthority(const Frame &exact_frame,
                                       std::uint64_t expiry_ns,
                                       std::unique_ptr<Authority> &out_authority);

private:
    oa_transport_status validateDeadline(std::uint64_t deadline_ns,
                                         std::uint64_t now_ns) const noexcept;
    struct Grant {
        Frame exact_frame{};
        oa_transport_frame_class frame_class{OA_TRANSPORT_FRAME_UNKNOWN};
        std::uint64_t expiry_ns{};
    };

    oa_transport_status consumeAuthority(const Authority *authority,
                                         const Frame &frame,
                                         oa_transport_frame_class frame_class,
                                         std::uint64_t now_ns,
                                         std::uint64_t deadline_ns) noexcept;

    std::unique_ptr<Backend> backend_;
    const std::array<std::uint64_t, 2> instance_nonce_;
    const std::uint64_t max_deadline_horizon_ns_;
    std::atomic<bool> closed_{false};
    std::mutex close_mutex_;
    std::mutex send_mutex_;
    std::mutex receive_mutex_;
    std::mutex authority_mutex_;
    std::unordered_map<std::uint64_t, Grant> grants_;
};

} // namespace openarm::transport

#endif
