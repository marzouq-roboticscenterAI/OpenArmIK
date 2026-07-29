/* SPDX-License-Identifier: Apache-2.0 */
#include "transport_internal.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>

namespace {

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: "   \
                      << #condition << '\n';                                    \
            std::exit(EXIT_FAILURE);                                            \
        }                                                                       \
    } while (false)

using openarm::transport::Backend;
using openarm::transport::Event;
using openarm::transport::Frame;
using openarm::transport::Transport;

class FakeBackend final : public Backend {
public:
    oa_transport_status now(std::uint64_t &out_ns) noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        out_ns = now_ns_;
        return OA_TRANSPORT_OK;
    }

    oa_transport_status send(const Frame &frame, std::uint64_t deadline_ns,
                             std::uint64_t &out_sent_ns) noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_) {
            return OA_TRANSPORT_ECLOSED;
        }
        if (deadline_ns < now_ns_) {
            return OA_TRANSPORT_ETIMEOUT;
        }
        sent_frames_.push_back(frame);
        out_sent_ns = now_ns_;
        return OA_TRANSPORT_OK;
    }

    oa_transport_status receive(std::uint64_t deadline_ns,
                                Event &out_event) noexcept override {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this, deadline_ns] {
            return closed_ || !events_.empty() || now_ns_ >= deadline_ns;
        });
        if (closed_) {
            return OA_TRANSPORT_ECLOSED;
        }
        if (events_.empty()) {
            return OA_TRANSPORT_ETIMEOUT;
        }
        out_event = events_.front();
        events_.pop_front();
        return OA_TRANSPORT_OK;
    }

    void close() noexcept override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }
        condition_.notify_all();
    }

    void setNow(std::uint64_t now_ns) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            now_ns_ = now_ns;
        }
        condition_.notify_all();
    }

    void push(Event event) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            events_.push_back(event);
        }
        condition_.notify_all();
    }

    std::size_t sendCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return sent_frames_.size();
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::uint64_t now_ns_{100U};
    bool closed_{};
    std::deque<Event> events_;
    std::deque<Frame> sent_frames_;
};

Frame systemFrame(std::uint8_t operation) {
    Frame frame;
    frame.can_id = OA_TRANSPORT_SFF_MASK;
    frame.dlc = 8U;
    frame.data[0] = 1U;
    frame.data[2] = operation;
    return frame;
}

Frame specialFrame(std::uint8_t operation) {
    Frame frame;
    frame.can_id = 1U;
    frame.dlc = 8U;
    std::memset(frame.data, 0xff, 7U);
    frame.data[7] = operation;
    return frame;
}

Frame motionFrame() {
    Frame frame;
    frame.can_id = 1U;
    frame.dlc = 8U;
    frame.data[0] = 0x80U;
    return frame;
}

void testClassification() {
    struct Case {
        Frame frame;
        oa_transport_frame_class expected;
    } cases[] = {
        {systemFrame(0x33U), OA_TRANSPORT_FRAME_REGISTER_QUERY},
        {systemFrame(0xccU), OA_TRANSPORT_FRAME_STATUS_QUERY},
        {[] {
             auto frame = systemFrame(0x55U);
             frame.data[3] = 21U;
             return frame;
         }(), OA_TRANSPORT_FRAME_REGISTER_WRITE},
        {systemFrame(0xaaU), OA_TRANSPORT_FRAME_SAVE_PARAMETERS},
        {specialFrame(0xfcU), OA_TRANSPORT_FRAME_ENABLE},
        {specialFrame(0xfdU), OA_TRANSPORT_FRAME_DISABLE},
        {specialFrame(0xfeU), OA_TRANSPORT_FRAME_SET_ZERO},
        {specialFrame(0xfbU), OA_TRANSPORT_FRAME_CLEAR_ERROR},
        {motionFrame(), OA_TRANSPORT_FRAME_MOTION},
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
    malformed = specialFrame(0xaaU);
    CHECK(openarm::transport::classify(malformed, actual) == OA_TRANSPORT_OK);
    CHECK(actual == OA_TRANSPORT_FRAME_UNKNOWN);
}

void testQueryOnlyPolicyAndDeadlines() {
    auto backend = std::make_unique<FakeBackend>();
    FakeBackend *fake = backend.get();
    Transport transport(std::move(backend), OA_TRANSPORT_PERMISSION_QUERY, 0U,
                        1000U);
    oa_transport_frame_class frame_class = 0U;
    std::uint64_t sent_ns = 0U;
    CHECK(transport.send(systemFrame(0x33U), 200U, frame_class, sent_ns) ==
          OA_TRANSPORT_OK);
    CHECK(frame_class == OA_TRANSPORT_FRAME_REGISTER_QUERY);
    CHECK(sent_ns == 100U);
    CHECK(fake->sendCount() == 1U);
    CHECK(transport.send(motionFrame(), 200U, frame_class, sent_ns) ==
          OA_TRANSPORT_EPERMISSION);
    CHECK(transport.send(specialFrame(0xfdU), 200U, frame_class, sent_ns) ==
          OA_TRANSPORT_EPERMISSION);
    auto register_write = systemFrame(0x55U);
    register_write.data[3] = 21U;
    CHECK(transport.send(register_write, 200U, frame_class, sent_ns) ==
          OA_TRANSPORT_EPERMISSION);
    CHECK(fake->sendCount() == 1U);
    CHECK(transport.send(systemFrame(0x33U), 99U, frame_class, sent_ns) ==
          OA_TRANSPORT_ETIMEOUT);
    CHECK(transport.send(systemFrame(0x33U), 1101U, frame_class, sent_ns) ==
          OA_TRANSPORT_ERANGE);
}

void testExpiringCapabilities() {
    {
        auto backend = std::make_unique<FakeBackend>();
        FakeBackend *fake = backend.get();
        Transport control(std::move(backend), OA_TRANSPORT_PERMISSION_CONTROL,
                          500U, 1000U);
        oa_transport_frame_class frame_class = 0U;
        std::uint64_t sent_ns = 0U;
        CHECK(control.send(motionFrame(), 200U, frame_class, sent_ns) ==
              OA_TRANSPORT_OK);
        CHECK(control.send(motionFrame(), 501U, frame_class, sent_ns) ==
              OA_TRANSPORT_EPERMISSION);
        auto register_write = systemFrame(0x55U);
        register_write.data[3] = 21U;
        CHECK(control.send(register_write, 200U, frame_class, sent_ns) ==
              OA_TRANSPORT_EPERMISSION);
        fake->setNow(500U);
        CHECK(control.send(motionFrame(), 600U, frame_class, sent_ns) ==
              OA_TRANSPORT_EPERMISSION);
    }
    {
        auto backend = std::make_unique<FakeBackend>();
        Transport commission(std::move(backend),
                             OA_TRANSPORT_PERMISSION_COMMISSION, 500U, 1000U);
        oa_transport_frame_class frame_class = 0U;
        std::uint64_t sent_ns = 0U;
        auto register_write = systemFrame(0x55U);
        register_write.data[3] = 21U;
        CHECK(commission.send(register_write, 200U, frame_class, sent_ns) ==
              OA_TRANSPORT_OK);
        CHECK(commission.send(specialFrame(0xfeU), 200U, frame_class, sent_ns) ==
              OA_TRANSPORT_OK);
        CHECK(commission.send(systemFrame(0xaaU), 200U, frame_class, sent_ns) ==
              OA_TRANSPORT_OK);
        CHECK(commission.send(motionFrame(), 200U, frame_class, sent_ns) ==
              OA_TRANSPORT_EPERMISSION);
    }
}

void testReceiveDiagnostics() {
    auto backend = std::make_unique<FakeBackend>();
    FakeBackend *fake = backend.get();
    Transport transport(std::move(backend), OA_TRANSPORT_PERMISSION_QUERY, 0U,
                        1000U);
    Event source;
    source.kind = OA_TRANSPORT_EVENT_CAN_ERROR;
    source.flags = OA_TRANSPORT_EVENT_KERNEL_TIMESTAMP |
                   OA_TRANSPORT_EVENT_RX_OVERFLOW;
    source.dequeue_monotonic_ns = 110U;
    source.kernel_timestamp_ns = 123456U;
    source.kernel_timestamp_clock = OA_TRANSPORT_CLOCK_REALTIME;
    source.can_error_mask = 0x40U;
    source.rx_overflow_count = 9U;
    source.rx_overflow_delta = 2U;
    source.link_up = false;
    source.frame.can_id = 0x20000040U;
    source.frame.dlc = 8U;
    fake->push(source);
    Event actual;
    CHECK(transport.receive(200U, actual) == OA_TRANSPORT_OK);
    CHECK(actual.kind == source.kind);
    CHECK(actual.flags == source.flags);
    CHECK(actual.kernel_timestamp_ns == source.kernel_timestamp_ns);
    CHECK(actual.can_error_mask == source.can_error_mask);
    CHECK(actual.rx_overflow_delta == source.rx_overflow_delta);
}

void testCloseInterruptsReceive() {
    auto backend = std::make_unique<FakeBackend>();
    Transport transport(std::move(backend), OA_TRANSPORT_PERMISSION_QUERY, 0U,
                        UINT64_C(10000000000));
    auto result = std::async(std::launch::async, [&transport] {
        Event event;
        return transport.receive(UINT64_C(5000000000), event);
    });
    CHECK(result.wait_for(std::chrono::milliseconds(20)) ==
          std::future_status::timeout);
    transport.close();
    CHECK(result.wait_for(std::chrono::seconds(1)) == std::future_status::ready);
    CHECK(result.get() == OA_TRANSPORT_ECLOSED);
    transport.close();
}

void testPublicClassifierAbi() {
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
    CHECK(oa_transport_close(nullptr) == OA_TRANSPORT_EINVAL);

    oa_transport *handle =
        reinterpret_cast<oa_transport *>(static_cast<std::uintptr_t>(1U));
    oa_transport_filter filter{};
    filter.struct_size = sizeof(filter);
    filter.abi_version = OA_TRANSPORT_ABI_VERSION;
    filter.can_id = 2U;
    filter.can_mask = 1U;
    CHECK(oa_transport_open("not-a-can", nullptr, &filter, 1U, nullptr,
                            &handle) == OA_TRANSPORT_ERANGE);
    CHECK(handle == reinterpret_cast<oa_transport *>(
                        static_cast<std::uintptr_t>(1U)));
    oa_transport_open_options options{};
    options.struct_size = sizeof(options);
    options.abi_version = OA_TRANSPORT_ABI_VERSION + 1U;
    CHECK(oa_transport_open("not-a-can", &options, nullptr, 0U, nullptr,
                            &handle) == OA_TRANSPORT_EABI);
    CHECK(handle == reinterpret_cast<oa_transport *>(
                        static_cast<std::uintptr_t>(1U)));
    handle = nullptr;
    CHECK(oa_transport_open("lo", nullptr, nullptr, 0U, nullptr, &handle) ==
          OA_TRANSPORT_EUNSUPPORTED);
    CHECK(handle == nullptr);
}

} // namespace

int main() {
    testClassification();
    testQueryOnlyPolicyAndDeadlines();
    testExpiringCapabilities();
    testReceiveDiagnostics();
    testCloseInterruptsReceive();
    testPublicClassifierAbi();
    std::cout << "openarm transport tests passed\n";
    return EXIT_SUCCESS;
}
