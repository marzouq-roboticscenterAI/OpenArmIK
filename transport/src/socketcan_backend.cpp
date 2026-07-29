/* SPDX-License-Identifier: Apache-2.0 */
#include "transport_internal.hpp"

#ifdef __linux__

#include <array>
#include <cerrno>
#include <cstring>
#include <limits>
#include <new>
#include <utility>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace openarm::transport {
namespace {

enum class Ready { Can, Link, Closed, Timeout, Failed };

class SocketCanBackend final : public Backend {
public:
    SocketCanBackend(int can_fd, int wake_fd, int link_fd, unsigned int ifindex,
                     bool initial_link_up) noexcept
        : can_fd_(can_fd), wake_fd_(wake_fd), link_fd_(link_fd), ifindex_(ifindex),
          link_up_(initial_link_up), initial_link_pending_(true) {}

    ~SocketCanBackend() override {
        close();
        if (link_fd_ >= 0) {
            (void)::close(link_fd_);
        }
        if (wake_fd_ >= 0) {
            (void)::close(wake_fd_);
        }
        if (can_fd_ >= 0) {
            (void)::close(can_fd_);
        }
    }

    oa_transport_status now(std::uint64_t &out_ns) noexcept override {
        return monotonicNow(out_ns);
    }

    oa_transport_status send(const Frame &frame, std::uint64_t deadline_ns,
                             std::uint64_t &out_sent_ns) noexcept override {
        std::lock_guard<std::mutex> lock(send_mutex_);
        struct can_frame wire {};
        wire.can_id = frame.can_id;
        wire.can_dlc = frame.dlc;
        std::memcpy(wire.data, frame.data, sizeof(wire.data));
        while (true) {
            if (closed_.load(std::memory_order_acquire)) {
                return OA_TRANSPORT_ECLOSED;
            }
            std::uint64_t now_ns = 0U;
            if (monotonicNow(now_ns) != OA_TRANSPORT_OK) {
                return OA_TRANSPORT_EIO;
            }
            if (now_ns >= deadline_ns) {
                return OA_TRANSPORT_ETIMEOUT;
            }
            const ssize_t written =
                ::send(can_fd_, &wire, sizeof(wire), MSG_DONTWAIT | MSG_NOSIGNAL);
            if (written == static_cast<ssize_t>(sizeof(wire))) {
                return monotonicNow(out_sent_ns);
            }
            if (written >= 0) {
                return OA_TRANSPORT_EIO;
            }
            if (errno == ENETDOWN || errno == ENETUNREACH || errno == ENODEV) {
                return OA_TRANSPORT_ELINK;
            }
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != ENOBUFS &&
                errno != EINTR) {
                return OA_TRANSPORT_EIO;
            }
            if (errno == EINTR) {
                continue;
            }
            const Ready ready = wait(deadline_ns, true);
            if (ready == Ready::Closed) {
                return OA_TRANSPORT_ECLOSED;
            }
            if (ready == Ready::Timeout) {
                return OA_TRANSPORT_ETIMEOUT;
            }
            if (ready == Ready::Failed) {
                return OA_TRANSPORT_EIO;
            }
        }
    }

    oa_transport_status receive(std::uint64_t deadline_ns,
                                Event &out_event) noexcept override {
        std::lock_guard<std::mutex> lock(receive_mutex_);
        if (initial_link_pending_) {
            initial_link_pending_ = false;
            return makeLinkEvent(link_up_, out_event);
        }
        while (true) {
            if (closed_.load(std::memory_order_acquire)) {
                return OA_TRANSPORT_ECLOSED;
            }
            const Ready ready = wait(deadline_ns, false);
            if (ready == Ready::Closed) {
                return OA_TRANSPORT_ECLOSED;
            }
            if (ready == Ready::Timeout) {
                return OA_TRANSPORT_ETIMEOUT;
            }
            if (ready == Ready::Failed) {
                return OA_TRANSPORT_EIO;
            }
            if (ready == Ready::Link) {
                bool changed = false;
                bool up = link_up_;
                const auto status = drainLinkEvents(changed, up);
                if (status != OA_TRANSPORT_OK) {
                    return status;
                }
                if (changed) {
                    link_up_ = up;
                    return makeLinkEvent(up, out_event);
                }
                continue;
            }
            const auto status = receiveCan(out_event);
            if (status == OA_TRANSPORT_ETIMEOUT) {
                continue;
            }
            return status;
        }
    }

    void close() noexcept override {
        if (!closed_.exchange(true, std::memory_order_acq_rel)) {
            const std::uint64_t value = 1U;
            const ssize_t result = ::write(wake_fd_, &value, sizeof(value));
            (void)result;
        }
    }

private:
    Ready wait(std::uint64_t deadline_ns, bool write) noexcept {
        while (true) {
            std::uint64_t now_ns = 0U;
            if (monotonicNow(now_ns) != OA_TRANSPORT_OK) {
                return Ready::Failed;
            }
            if (now_ns >= deadline_ns) {
                return Ready::Timeout;
            }
            const std::uint64_t remaining = deadline_ns - now_ns;
            struct timespec timeout {};
            timeout.tv_sec = static_cast<time_t>(remaining / UINT64_C(1000000000));
            timeout.tv_nsec =
                static_cast<long>(remaining % UINT64_C(1000000000));
            std::array<struct pollfd, 3> descriptors{};
            descriptors[0].fd = can_fd_;
            descriptors[0].events = static_cast<short>(write ? POLLOUT : POLLIN);
            descriptors[1].fd = wake_fd_;
            descriptors[1].events = POLLIN;
            descriptors[2].fd = write ? -1 : link_fd_;
            descriptors[2].events = POLLIN;
            const int result = ::ppoll(descriptors.data(), descriptors.size(), &timeout,
                                       nullptr);
            if (result == 0) {
                return Ready::Timeout;
            }
            if (result < 0) {
                if (errno == EINTR) {
                    continue;
                }
                return Ready::Failed;
            }
            if ((descriptors[1].revents & POLLIN) != 0) {
                return Ready::Closed;
            }
            if (!write && (descriptors[2].revents & POLLIN) != 0) {
                return Ready::Link;
            }
            if ((descriptors[0].revents &
                 static_cast<short>((write ? POLLOUT : POLLIN) | POLLERR |
                                    POLLHUP | POLLNVAL)) != 0) {
                return Ready::Can;
            }
        }
    }

    oa_transport_status makeLinkEvent(bool up, Event &out_event) noexcept {
        Event event;
        event.kind = OA_TRANSPORT_EVENT_LINK;
        event.link_up = up;
        const auto status = monotonicNow(event.dequeue_monotonic_ns);
        if (status == OA_TRANSPORT_OK) {
            out_event = event;
        }
        return status;
    }

    oa_transport_status drainLinkEvents(bool &out_changed,
                                        bool &out_up) noexcept {
        out_changed = false;
        std::array<unsigned char, 8192> buffer{};
        while (true) {
            const ssize_t count =
                ::recv(link_fd_, buffer.data(), buffer.size(), MSG_DONTWAIT);
            if (count < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    return OA_TRANSPORT_OK;
                }
                if (errno == EINTR) {
                    continue;
                }
                return OA_TRANSPORT_EIO;
            }
            if (count == 0) {
                return OA_TRANSPORT_EIO;
            }
            std::size_t offset = 0U;
            const auto length = static_cast<std::size_t>(count);
            while (length - offset >= sizeof(struct nlmsghdr)) {
                struct nlmsghdr header {};
                std::memcpy(&header, buffer.data() + offset, sizeof(header));
                const auto message_length =
                    static_cast<std::size_t>(header.nlmsg_len);
                if (message_length < sizeof(header) ||
                    message_length > length - offset) {
                    return OA_TRANSPORT_EFRAME;
                }
                if (header.nlmsg_type == NLMSG_ERROR ||
                    header.nlmsg_type == NLMSG_OVERRUN) {
                    return OA_TRANSPORT_EIO;
                }
                if ((header.nlmsg_type == RTM_NEWLINK ||
                     header.nlmsg_type == RTM_DELLINK) &&
                    message_length >= sizeof(header) + sizeof(struct ifinfomsg)) {
                    struct ifinfomsg info {};
                    std::memcpy(&info, buffer.data() + offset + sizeof(header),
                                sizeof(info));
                    if (info.ifi_index > 0 &&
                        static_cast<unsigned int>(info.ifi_index) == ifindex_) {
                        out_changed = true;
                        out_up = header.nlmsg_type == RTM_NEWLINK &&
                                 (info.ifi_flags & IFF_UP) != 0U &&
                                 (info.ifi_flags & IFF_RUNNING) != 0U;
                    }
                }
                const std::size_t aligned = (message_length + 3U) & ~std::size_t{3U};
                if (aligned > length - offset) {
                    if (message_length != length - offset) {
                        return OA_TRANSPORT_EFRAME;
                    }
                    offset += message_length;
                } else {
                    offset += aligned;
                }
            }
            if (offset != length) {
                return OA_TRANSPORT_EFRAME;
            }
        }
    }

    oa_transport_status receiveCan(Event &out_event) noexcept {
        struct can_frame wire {};
        std::array<unsigned char, 256> control{};
        struct iovec vector {};
        vector.iov_base = &wire;
        vector.iov_len = sizeof(wire);
        struct msghdr message {};
        message.msg_iov = &vector;
        message.msg_iovlen = 1U;
        message.msg_control = control.data();
        message.msg_controllen = control.size();
        const ssize_t count = ::recvmsg(can_fd_, &message, MSG_DONTWAIT);
        if (count < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                return OA_TRANSPORT_ETIMEOUT;
            }
            if (errno == ENETDOWN || errno == ENETUNREACH || errno == ENODEV) {
                link_up_ = false;
                return makeLinkEvent(false, out_event);
            }
            return OA_TRANSPORT_EIO;
        }
        if (count != static_cast<ssize_t>(sizeof(wire)) ||
            (message.msg_flags & (MSG_TRUNC | MSG_CTRUNC)) != 0 ||
            wire.can_dlc != 8U) {
            return OA_TRANSPORT_EFRAME;
        }

        Event event;
        event.frame.can_id = wire.can_id;
        event.frame.dlc = wire.can_dlc;
        std::memcpy(event.frame.data, wire.data, sizeof(event.frame.data));
        const bool error_frame = (wire.can_id & CAN_ERR_FLAG) != 0U;
        if (!error_frame &&
            ((wire.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)) != 0U ||
             (wire.can_id & CAN_SFF_MASK) > OA_TRANSPORT_SFF_MASK)) {
            return OA_TRANSPORT_EFRAME;
        }
        event.kind = error_frame ? OA_TRANSPORT_EVENT_CAN_ERROR
                                 : OA_TRANSPORT_EVENT_FRAME;
        if (error_frame &&
            (wire.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)) != 0U) {
            return OA_TRANSPORT_EFRAME;
        }
        event.can_error_mask = error_frame ? wire.can_id & CAN_ERR_MASK : 0U;
        event.link_up = link_up_;
        event.rx_overflow_count = last_overflow_count_;

        for (struct cmsghdr *header = CMSG_FIRSTHDR(&message); header != nullptr;
             header = CMSG_NXTHDR(&message, header)) {
            if (header->cmsg_level != SOL_SOCKET) {
                continue;
            }
            if (header->cmsg_type == SCM_TIMESTAMPNS &&
                header->cmsg_len >= CMSG_LEN(sizeof(struct timespec))) {
                struct timespec timestamp {};
                std::memcpy(&timestamp, CMSG_DATA(header), sizeof(timestamp));
                if (timestamp.tv_sec >= 0 && timestamp.tv_nsec >= 0 &&
                    timestamp.tv_nsec < 1000000000L) {
                    const auto seconds =
                        static_cast<std::uint64_t>(timestamp.tv_sec);
                    event.kernel_timestamp_ns =
                        seconds * UINT64_C(1000000000) +
                        static_cast<std::uint64_t>(timestamp.tv_nsec);
                    event.kernel_timestamp_clock = OA_TRANSPORT_CLOCK_REALTIME;
                    event.flags |= OA_TRANSPORT_EVENT_KERNEL_TIMESTAMP;
                }
            } else if (header->cmsg_type == SO_RXQ_OVFL &&
                       header->cmsg_len >= CMSG_LEN(sizeof(std::uint32_t))) {
                std::uint32_t overflow = 0U;
                std::memcpy(&overflow, CMSG_DATA(header), sizeof(overflow));
                event.rx_overflow_count = overflow;
                event.rx_overflow_delta = overflow - last_overflow_count_;
                last_overflow_count_ = overflow;
                if (event.rx_overflow_delta != 0U) {
                    event.flags |= OA_TRANSPORT_EVENT_RX_OVERFLOW;
                }
            }
        }
        const auto status = monotonicNow(event.dequeue_monotonic_ns);
        if (status == OA_TRANSPORT_OK) {
            out_event = event;
        }
        return status;
    }

    int can_fd_;
    int wake_fd_;
    int link_fd_;
    unsigned int ifindex_;
    bool link_up_;
    bool initial_link_pending_;
    std::uint32_t last_overflow_count_{};
    std::atomic<bool> closed_{false};
    std::mutex send_mutex_;
    std::mutex receive_mutex_;
};

bool validInterfaceName(const std::string &name) noexcept {
    return !name.empty() && name.size() < IFNAMSIZ && name.find('/') == std::string::npos;
}

oa_transport_status queryInterface(int fd, const std::string &name,
                                   unsigned int &out_ifindex,
                                   bool &out_link_up) noexcept {
    const unsigned int ifindex = ::if_nametoindex(name.c_str());
    if (ifindex == 0U) {
        return OA_TRANSPORT_EINVAL;
    }
    std::array<char, IFNAMSIZ> resolved{};
    if (::if_indextoname(ifindex, resolved.data()) == nullptr || name != resolved.data()) {
        return OA_TRANSPORT_EINVAL;
    }
    struct ifreq request {};
    std::memcpy(request.ifr_name, name.c_str(), name.size() + 1U);
    if (::ioctl(fd, SIOCGIFMTU, &request) < 0 || request.ifr_mtu != CAN_MTU) {
        return OA_TRANSPORT_EUNSUPPORTED;
    }
    std::memset(&request, 0, sizeof(request));
    std::memcpy(request.ifr_name, name.c_str(), name.size() + 1U);
    if (::ioctl(fd, SIOCGIFHWADDR, &request) < 0 ||
        request.ifr_hwaddr.sa_family != ARPHRD_CAN) {
        return OA_TRANSPORT_EUNSUPPORTED;
    }
    std::memset(&request, 0, sizeof(request));
    std::memcpy(request.ifr_name, name.c_str(), name.size() + 1U);
    if (::ioctl(fd, SIOCGIFFLAGS, &request) < 0) {
        return OA_TRANSPORT_EIO;
    }
    out_ifindex = ifindex;
    out_link_up = (request.ifr_flags & IFF_UP) != 0 &&
                  (request.ifr_flags & IFF_RUNNING) != 0;
    return OA_TRANSPORT_OK;
}

} // namespace

std::unique_ptr<Backend> makeSocketCanBackend(const std::string &interface_name,
                                              const OpenConfig &config,
                                              oa_transport_status &out_status) {
    out_status = OA_TRANSPORT_EIO;
    if (!validInterfaceName(interface_name)) {
        out_status = OA_TRANSPORT_EINVAL;
        return nullptr;
    }
    std::vector<struct can_filter> wire_filters;
    wire_filters.reserve(config.filters.size());
    for (const auto &filter : config.filters) {
        struct can_filter wire {};
        wire.can_id = filter.can_id;
        wire.can_mask = filter.can_mask | CAN_EFF_FLAG | CAN_RTR_FLAG;
        wire_filters.push_back(wire);
    }
    const int can_fd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC,
                                CAN_RAW);
    if (can_fd < 0) {
        return nullptr;
    }
    unsigned int ifindex = 0U;
    bool link_up = false;
    out_status = queryInterface(can_fd, interface_name, ifindex, link_up);
    if (out_status != OA_TRANSPORT_OK) {
        (void)::close(can_fd);
        return nullptr;
    }
    if (!wire_filters.empty()) {
        const std::size_t byte_count = wire_filters.size() * sizeof(wire_filters[0]);
        if (byte_count > static_cast<std::size_t>(
                             std::numeric_limits<socklen_t>::max()) ||
            ::setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_FILTER, wire_filters.data(),
                         static_cast<socklen_t>(byte_count)) < 0) {
            (void)::close(can_fd);
            out_status = OA_TRANSPORT_EIO;
            return nullptr;
        }
    }
    const can_err_mask_t error_mask = config.can_error_mask;
    const int enabled = 1;
    const int disabled = 0;
    if (::setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &error_mask,
                     sizeof(error_mask)) < 0 ||
        ::setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &disabled,
                     sizeof(disabled)) < 0 ||
        ::setsockopt(can_fd, SOL_SOCKET, SO_TIMESTAMPNS, &enabled,
                     sizeof(enabled)) < 0 ||
        ::setsockopt(can_fd, SOL_SOCKET, SO_RXQ_OVFL, &enabled,
                     sizeof(enabled)) < 0) {
        (void)::close(can_fd);
        out_status = OA_TRANSPORT_EIO;
        return nullptr;
    }
    if (config.receive_buffer_bytes != 0U) {
        if (config.receive_buffer_bytes >
            static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
            (void)::close(can_fd);
            out_status = OA_TRANSPORT_ERANGE;
            return nullptr;
        }
        const int receive_buffer_bytes =
            static_cast<int>(config.receive_buffer_bytes);
        if (::setsockopt(can_fd, SOL_SOCKET, SO_RCVBUF, &receive_buffer_bytes,
                         sizeof(receive_buffer_bytes)) < 0) {
            (void)::close(can_fd);
            out_status = OA_TRANSPORT_EIO;
            return nullptr;
        }
    }
    struct sockaddr_can address {};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(ifindex);
    if (::bind(can_fd, reinterpret_cast<const struct sockaddr *>(&address),
               sizeof(address)) < 0) {
        (void)::close(can_fd);
        out_status = OA_TRANSPORT_EIO;
        return nullptr;
    }

    const int wake_fd = ::eventfd(0U, EFD_NONBLOCK | EFD_CLOEXEC);
    if (wake_fd < 0) {
        (void)::close(can_fd);
        out_status = OA_TRANSPORT_EIO;
        return nullptr;
    }
    const int link_fd =
        ::socket(AF_NETLINK, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, NETLINK_ROUTE);
    if (link_fd < 0) {
        (void)::close(wake_fd);
        (void)::close(can_fd);
        out_status = OA_TRANSPORT_EIO;
        return nullptr;
    }
    struct sockaddr_nl link_address {};
    link_address.nl_family = AF_NETLINK;
    link_address.nl_groups = RTMGRP_LINK;
    if (::bind(link_fd, reinterpret_cast<const struct sockaddr *>(&link_address),
               sizeof(link_address)) < 0) {
        (void)::close(link_fd);
        (void)::close(wake_fd);
        (void)::close(can_fd);
        out_status = OA_TRANSPORT_EIO;
        return nullptr;
    }
    out_status = OA_TRANSPORT_OK;
    std::unique_ptr<Backend> backend(
        new (std::nothrow)
            SocketCanBackend(can_fd, wake_fd, link_fd, ifindex, link_up));
    if (!backend) {
        (void)::close(link_fd);
        (void)::close(wake_fd);
        (void)::close(can_fd);
        out_status = OA_TRANSPORT_ENOMEM;
    }
    return backend;
}

} // namespace openarm::transport

#else

namespace openarm::transport {

std::unique_ptr<Backend> makeSocketCanBackend(const std::string &,
                                              const OpenConfig &,
                                              oa_transport_status &out_status) {
    out_status = OA_TRANSPORT_EUNSUPPORTED;
    return nullptr;
}

} // namespace openarm::transport

#endif
