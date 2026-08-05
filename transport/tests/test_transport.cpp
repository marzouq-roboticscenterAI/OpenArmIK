/* SPDX-License-Identifier: Apache-2.0 */
#include "socketcan_backend_internal.hpp"
#include "transport_internal.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>

#ifdef __linux__
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#endif

namespace {

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: "   \
                      << #condition << '\n';                                    \
            std::exit(EXIT_FAILURE);                                            \
        }                                                                       \
    } while (false)

using openarm::transport::Authority;
using openarm::transport::Backend;
using openarm::transport::Event;
using openarm::transport::Frame;
using openarm::transport::Transport;

class FakeBackend final : public Backend {
public:
    explicit FakeBackend(bool permits_authority = true) noexcept
        : permits_authority_(permits_authority) {}

    oa_transport_status now(std::uint64_t &out_ns) noexcept override {
        out_ns = now_ns_.load(std::memory_order_acquire);
        return OA_TRANSPORT_OK;
    }

    oa_transport_status send(const Frame &frame, std::uint64_t deadline_ns,
                             std::uint64_t &out_sent_ns) noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now_ns = now_ns_.load(std::memory_order_acquire);
        if (closed_.load(std::memory_order_acquire)) {
            return OA_TRANSPORT_ECLOSED;
        }
        if (deadline_ns <= now_ns) {
            return OA_TRANSPORT_ETIMEOUT;
        }
        sent_frames_.push_back(frame);
        out_sent_ns = now_ns;
        return OA_TRANSPORT_OK;
    }

    oa_transport_status receive(std::uint64_t deadline_ns,
                                Event &out_event) noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now_ns = now_ns_.load(std::memory_order_acquire);
        if (closed_.load(std::memory_order_acquire)) {
            return OA_TRANSPORT_ECLOSED;
        }
        if (deadline_ns <= now_ns || events_.empty()) {
            return OA_TRANSPORT_ETIMEOUT;
        }
        out_event = events_.front();
        events_.pop_front();
        return OA_TRANSPORT_OK;
    }

    void close() noexcept override {
        closed_.store(true, std::memory_order_release);
        std::lock_guard<std::mutex> lock(mutex_);
    }

    bool permitsAuthorityIssuance() const noexcept override {
        return permits_authority_;
    }

    void setNow(std::uint64_t value) noexcept {
        now_ns_.store(value, std::memory_order_release);
    }

    void push(Event event) {
        std::lock_guard<std::mutex> lock(mutex_);
        events_.push_back(event);
    }

    std::size_t sendCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return sent_frames_.size();
    }

private:
    const bool permits_authority_;
    std::atomic<std::uint64_t> now_ns_{100U};
    std::atomic<bool> closed_{false};
    mutable std::mutex mutex_;
    std::deque<Event> events_;
    std::deque<Frame> sent_frames_;
};

class BlockingBackend final : public Backend {
public:
    oa_transport_status now(std::uint64_t &out_ns) noexcept override {
        out_ns = 100U;
        return OA_TRANSPORT_OK;
    }

    oa_transport_status send(const Frame &, std::uint64_t,
                             std::uint64_t &) noexcept override {
        std::unique_lock<std::mutex> operation_lock(send_mutex_);
        {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            send_entered_ = true;
        }
        state_changed_.notify_all();
        std::unique_lock<std::mutex> state_lock(state_mutex_);
        state_changed_.wait(state_lock, [this] { return closed_; });
        return OA_TRANSPORT_ECLOSED;
    }

    oa_transport_status receive(std::uint64_t, Event &) noexcept override {
        std::unique_lock<std::mutex> operation_lock(receive_mutex_);
        {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            receive_entered_ = true;
        }
        state_changed_.notify_all();
        std::unique_lock<std::mutex> state_lock(state_mutex_);
        state_changed_.wait(state_lock, [this] { return closed_; });
        return OA_TRANSPORT_ECLOSED;
    }

    void close() noexcept override {
        {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            closed_ = true;
        }
        state_changed_.notify_all();
        std::scoped_lock operation_lock(send_mutex_, receive_mutex_);
    }

    bool permitsAuthorityIssuance() const noexcept override { return true; }

    void waitUntilBothEntered() {
        std::unique_lock<std::mutex> lock(state_mutex_);
        state_changed_.wait(lock,
                            [this] { return send_entered_ && receive_entered_; });
    }

private:
    std::mutex send_mutex_;
    std::mutex receive_mutex_;
    std::mutex state_mutex_;
    std::condition_variable state_changed_;
    bool send_entered_{};
    bool receive_entered_{};
    bool closed_{};
};

Frame systemFrame(std::uint8_t operation, std::uint8_t register_id = 0U) {
    Frame frame;
    frame.can_id = OA_TRANSPORT_SFF_MASK;
    frame.dlc = 8U;
    frame.data[0] = 1U;
    frame.data[2] = operation;
    frame.data[3] = register_id;
    return frame;
}

Frame registerWriteU32(std::uint8_t register_id, std::uint32_t value) {
    Frame frame = systemFrame(0x55U, register_id);
    frame.data[4] = static_cast<std::uint8_t>(value);
    frame.data[5] = static_cast<std::uint8_t>(value >> 8U);
    frame.data[6] = static_cast<std::uint8_t>(value >> 16U);
    frame.data[7] = static_cast<std::uint8_t>(value >> 24U);
    return frame;
}

Frame specialFrame(std::uint8_t operation, std::uint32_t can_id = 1U) {
    Frame frame;
    frame.can_id = can_id;
    frame.dlc = 8U;
    std::memset(frame.data, 0xff, 7U);
    frame.data[7] = operation;
    return frame;
}

Frame unknownFrame() {
    Frame frame;
    frame.can_id = 1U;
    frame.dlc = 8U;
    frame.data[0] = 0x80U;
    return frame;
}

void testExhaustiveFailClosedClassification() {
    struct Case {
        Frame frame;
        oa_transport_frame_class expected;
    } cases[] = {
        {systemFrame(0x33U), OA_TRANSPORT_FRAME_REGISTER_QUERY},
        {systemFrame(0xccU), OA_TRANSPORT_FRAME_STATUS_QUERY},
        {registerWriteU32(10U, 1U), OA_TRANSPORT_FRAME_REGISTER_WRITE},
        {systemFrame(0xaaU), OA_TRANSPORT_FRAME_SAVE_PARAMETERS},
        {specialFrame(0xfcU), OA_TRANSPORT_FRAME_ENABLE},
        {specialFrame(0xfdU), OA_TRANSPORT_FRAME_DISABLE},
        {specialFrame(0xfeU), OA_TRANSPORT_FRAME_SET_ZERO},
        {specialFrame(0xfbU), OA_TRANSPORT_FRAME_CLEAR_ERROR},
        {unknownFrame(), OA_TRANSPORT_FRAME_UNKNOWN},
        {specialFrame(0xfcU, OA_TRANSPORT_SFF_MASK),
         OA_TRANSPORT_FRAME_UNKNOWN},
        {registerWriteU32(10U, 9U), OA_TRANSPORT_FRAME_UNKNOWN},
        {registerWriteU32(21U, UINT32_C(0x7fc00000)),
         OA_TRANSPORT_FRAME_UNKNOWN},
    };
    for (const auto &item : cases) {
        oa_transport_frame_class actual = OA_TRANSPORT_FRAME_UNKNOWN;
        CHECK(openarm::transport::classify(item.frame, actual) ==
              OA_TRANSPORT_OK);
        CHECK(actual == item.expected);
    }
    auto malformed = systemFrame(0x33U);
    malformed.data[4] = 1U;
    oa_transport_frame_class actual = 0U;
    CHECK(openarm::transport::classify(malformed, actual) == OA_TRANSPORT_OK);
    CHECK(actual == OA_TRANSPORT_FRAME_UNKNOWN);
    malformed.dlc = 7U;
    CHECK(openarm::transport::classify(malformed, actual) ==
          OA_TRANSPORT_EFRAME);
}

void testOpaqueInstanceBoundOneShotAuthority() {
    auto first_backend = std::make_unique<FakeBackend>();
    FakeBackend *first_fake = first_backend.get();
    Transport first(std::move(first_backend), {11U, 12U}, 1000U);
    auto second_backend = std::make_unique<FakeBackend>();
    Transport second(std::move(second_backend), {21U, 22U}, 1000U);
    const Frame enable = specialFrame(0xfcU);
    std::unique_ptr<Authority> authority;
    CHECK(first.issueAuthority(enable, 500U, authority) == OA_TRANSPORT_OK);
    CHECK(authority != nullptr);

    oa_transport_frame_class frame_class = 0U;
    std::uint64_t sent_ns = 0U;
    CHECK(second.send(enable, 200U, authority.get(), frame_class, sent_ns) ==
          OA_TRANSPORT_EPERMISSION);
    CHECK(first.send(specialFrame(0xfdU), 200U, authority.get(), frame_class,
                     sent_ns) == OA_TRANSPORT_EPERMISSION);
    CHECK(first.send(enable, 200U, authority.get(), frame_class, sent_ns) ==
          OA_TRANSPORT_OK);
    CHECK(first_fake->sendCount() == 1U);
    CHECK(first.send(enable, 200U, authority.get(), frame_class, sent_ns) ==
          OA_TRANSPORT_EPERMISSION);
    CHECK(first_fake->sendCount() == 1U);

    std::unique_ptr<Authority> deadline_authority;
    CHECK(first.issueAuthority(enable, 500U, deadline_authority) ==
          OA_TRANSPORT_OK);
    CHECK(first.send(enable, 501U, deadline_authority.get(), frame_class,
                     sent_ns) == OA_TRANSPORT_EPERMISSION);

    auto physical_backend = std::make_unique<FakeBackend>(false);
    Transport physical(std::move(physical_backend), {31U, 32U}, 1000U);
    std::unique_ptr<Authority> forbidden;
    CHECK(physical.issueAuthority(enable, 500U, forbidden) ==
          OA_TRANSPORT_EPERMISSION);
}

void testPublicQueryOnlyAndDeadlines() {
    auto backend = std::make_unique<FakeBackend>();
    FakeBackend *fake = backend.get();
    Transport transport(std::move(backend), {1U, 2U}, 1000U);
    oa_transport_frame_class frame_class = 0U;
    std::uint64_t sent_ns = 0U;
    CHECK(transport.send(systemFrame(0x33U), 200U, nullptr, frame_class,
                         sent_ns) == OA_TRANSPORT_OK);
    CHECK(fake->sendCount() == 1U);
    CHECK(transport.send(specialFrame(0xfcU), 200U, nullptr, frame_class,
                         sent_ns) == OA_TRANSPORT_EPERMISSION);
    CHECK(transport.send(unknownFrame(), 200U, nullptr, frame_class, sent_ns) ==
          OA_TRANSPORT_EFRAME);
    CHECK(fake->sendCount() == 1U);
    CHECK(transport.send(systemFrame(0x33U), 100U, nullptr, frame_class,
                         sent_ns) == OA_TRANSPORT_ETIMEOUT);
    CHECK(transport.send(systemFrame(0x33U), 1101U, nullptr, frame_class,
                         sent_ns) == OA_TRANSPORT_ERANGE);
    Event event;
    CHECK(transport.receive(100U, event) == OA_TRANSPORT_ETIMEOUT);
}

void testCloseIsCompletedIoBarrier() {
    auto backend = std::make_unique<BlockingBackend>();
    BlockingBackend *blocking = backend.get();
    Transport transport(std::move(backend), {41U, 42U}, UINT64_C(10000000000));
    auto send_result = std::async(std::launch::async, [&transport] {
        oa_transport_frame_class frame_class = 0U;
        std::uint64_t sent_ns = 0U;
        return transport.send(systemFrame(0x33U), UINT64_C(5000000000), nullptr,
                              frame_class, sent_ns);
    });
    auto receive_result = std::async(std::launch::async, [&transport] {
        Event event;
        return transport.receive(UINT64_C(5000000000), event);
    });
    blocking->waitUntilBothEntered();
    transport.close();
    CHECK(send_result.wait_for(std::chrono::seconds(1)) ==
          std::future_status::ready);
    CHECK(receive_result.wait_for(std::chrono::seconds(1)) ==
          std::future_status::ready);
    CHECK(send_result.get() == OA_TRANSPORT_ECLOSED);
    CHECK(receive_result.get() == OA_TRANSPORT_ECLOSED);
    transport.close();
}

void testReceiveDiagnostics() {
    auto backend = std::make_unique<FakeBackend>();
    FakeBackend *fake = backend.get();
    Transport transport(std::move(backend), {51U, 52U}, 1000U);
    Event source;
    source.kind = OA_TRANSPORT_EVENT_CAN_ERROR;
    source.flags = OA_TRANSPORT_EVENT_KERNEL_TIMESTAMP |
                   OA_TRANSPORT_EVENT_RX_OVERFLOW;
    source.kernel_timestamp_ns = 123456U;
    source.can_error_mask = 0x40U;
    source.rx_overflow_delta = 2U;
    fake->push(source);
    Event actual;
    CHECK(transport.receive(200U, actual) == OA_TRANSPORT_OK);
    CHECK(actual.kind == source.kind);
    CHECK(actual.flags == source.flags);
    CHECK(actual.can_error_mask == source.can_error_mask);
}

#ifdef __linux__
void appendLinkMessage(std::array<unsigned char, 128> &buffer,
                       std::size_t &offset, std::uint16_t type, int ifindex,
                       unsigned int flags) {
    struct nlmsghdr header {};
    header.nlmsg_len = sizeof(header) + sizeof(struct ifinfomsg);
    header.nlmsg_type = type;
    struct ifinfomsg info {};
    info.ifi_index = ifindex;
    info.ifi_flags = flags;
    std::memcpy(buffer.data() + offset, &header, sizeof(header));
    std::memcpy(buffer.data() + offset + sizeof(header), &info, sizeof(info));
    offset += (static_cast<std::size_t>(header.nlmsg_len) + 3U) &
              ~std::size_t{3U};
}

void testNetlinkPreservesDownThenUp() {
    CHECK(!openarm::transport::socketCanBackendPermitsAuthorityIssuance());
    CHECK(!openarm::transport::netlinkReceiveWasTruncated(128U, 128U, 0));
    CHECK(openarm::transport::netlinkReceiveWasTruncated(128U, 128U,
                                                         MSG_TRUNC));
    CHECK(openarm::transport::netlinkReceiveWasTruncated(129U, 128U, 0));
    std::array<unsigned char, 128> buffer{};
    std::size_t length = 0U;
    appendLinkMessage(buffer, length, RTM_DELLINK, 42, 0U);
    appendLinkMessage(buffer, length, RTM_NEWLINK, 42,
                      static_cast<unsigned int>(IFF_UP | IFF_RUNNING));
    openarm::transport::LinkTransitionBatch batch;
    CHECK(openarm::transport::parseLinkDatagram(buffer.data(), length, 42U,
                                                batch) == OA_TRANSPORT_OK);
    CHECK(batch.count == 2U);
    CHECK(!batch.states[0]);
    CHECK(batch.states[1]);

    struct nlmsghdr overrun {};
    overrun.nlmsg_len = sizeof(overrun);
    overrun.nlmsg_type = NLMSG_OVERRUN;
    CHECK(openarm::transport::parseLinkDatagram(
              &overrun, sizeof(overrun), 42U, batch) == OA_TRANSPORT_EIO);
    CHECK(openarm::transport::parseLinkDatagram(buffer.data(), length - 1U, 42U,
                                                batch) == OA_TRANSPORT_EFRAME);
}
#endif

void testPublicAbiValidation() {
    oa_transport_frame frame{};
    frame.struct_size = sizeof(frame);
    frame.abi_version = OA_TRANSPORT_ABI_VERSION;
    frame.can_id = OA_TRANSPORT_SFF_MASK;
    frame.dlc = 8U;
    frame.data[0] = 1U;
    frame.data[2] = 0x33U;
    oa_transport_frame_class frame_class = 0U;
    CHECK(oa_transport_classify_send_frame(&frame, &frame_class) ==
          OA_TRANSPORT_OK);
    CHECK(frame_class == OA_TRANSPORT_FRAME_REGISTER_QUERY);
    frame_class = UINT32_C(0xa5a5a5a5);
    frame.abi_version += 1U;
    CHECK(oa_transport_classify_send_frame(&frame, &frame_class) ==
          OA_TRANSPORT_EABI);
    CHECK(frame_class == UINT32_C(0xa5a5a5a5));

    oa_transport *handle =
        reinterpret_cast<oa_transport *>(static_cast<std::uintptr_t>(1U));
    oa_transport_filter filter{};
    filter.struct_size = sizeof(filter);
    filter.abi_version = OA_TRANSPORT_ABI_VERSION;
    filter.can_id = 2U;
    filter.can_mask = 1U;
    CHECK(oa_transport_open("not-a-can", nullptr, &filter, 1U, &handle) ==
          OA_TRANSPORT_ERANGE);
    CHECK(handle == reinterpret_cast<oa_transport *>(
                        static_cast<std::uintptr_t>(1U)));
    oa_transport_open_options options{};
    options.struct_size = sizeof(options);
    options.abi_version = OA_TRANSPORT_ABI_VERSION + 1U;
    CHECK(oa_transport_open("not-a-can", &options, nullptr, 0U, &handle) ==
          OA_TRANSPORT_EABI);
    handle = nullptr;
    const oa_transport_status loopback_status =
        oa_transport_open("lo", nullptr, nullptr, 0U, &handle);
    CHECK(loopback_status == OA_TRANSPORT_EUNSUPPORTED ||
          loopback_status == OA_TRANSPORT_EIO);
    CHECK(handle == nullptr);
}

} // namespace

int main() {
    testExhaustiveFailClosedClassification();
    testOpaqueInstanceBoundOneShotAuthority();
    testPublicQueryOnlyAndDeadlines();
    testCloseIsCompletedIoBarrier();
    testReceiveDiagnostics();
#ifdef __linux__
    testNetlinkPreservesDownThenUp();
#endif
    testPublicAbiValidation();
    std::cout << "openarm transport tests passed\n";
    return EXIT_SUCCESS;
}
