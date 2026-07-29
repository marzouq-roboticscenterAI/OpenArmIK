/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_can_linux_internal.h"

#include <string.h>

#ifdef __linux__
#include <linux/can/netlink.h>
#include <linux/if.h>
#include <linux/if_link.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <stddef.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

typedef struct oa_attr_view {
    uint16_t type;
    const unsigned char *payload;
    size_t payload_length;
    size_t next_offset;
} oa_attr_view;

static size_t oa_align_netlink(size_t value) {
    return (value + 3u) & ~(size_t)3u;
}

static int oa_padding_is_zero(const unsigned char *data, size_t length) {
    size_t i;
    for (i = 0u; i < length; ++i) {
        if (data[i] != 0u) return 0;
    }
    return 1;
}

static oa_can_status oa_next_attribute(const unsigned char *data, size_t length,
                                       size_t offset, oa_attr_view *out) {
    struct rtattr attribute;
    size_t attribute_length;
    size_t aligned_length;
    if (offset > length || length - offset < sizeof(attribute)) return OA_CAN_EFRAME;
    (void)memcpy(&attribute, data + offset, sizeof(attribute));
    attribute_length = attribute.rta_len;
    if (attribute_length < sizeof(attribute) || attribute_length > length - offset) return OA_CAN_EFRAME;
    aligned_length = oa_align_netlink(attribute_length);
    if (aligned_length > length - offset) {
        if (attribute_length != length - offset) return OA_CAN_EFRAME;
        aligned_length = attribute_length;
    }
    out->type = (uint16_t)(attribute.rta_type & NLA_TYPE_MASK);
    out->payload = data + offset + sizeof(attribute);
    out->payload_length = attribute_length - sizeof(attribute);
    out->next_offset = offset + aligned_length;
    return OA_CAN_OK;
}

static oa_can_status oa_parse_can_data(const unsigned char *data, size_t length,
                                       oa_can_linux_interface *item) {
    size_t offset = 0u;
    while (offset < length) {
        oa_attr_view attribute;
        oa_can_status status;
        if (length - offset < sizeof(struct rtattr)) {
            return oa_padding_is_zero(data + offset, length - offset) ? OA_CAN_OK : OA_CAN_EFRAME;
        }
        status = oa_next_attribute(data, length, offset, &attribute);
        if (status != OA_CAN_OK) return status;
        if (attribute.type == IFLA_CAN_BITTIMING) {
            struct can_bittiming timing;
            if (attribute.payload_length < sizeof(timing)) return OA_CAN_EFRAME;
            (void)memcpy(&timing, attribute.payload, sizeof(timing));
            item->bitrate = timing.bitrate;
        } else if (attribute.type == IFLA_CAN_DATA_BITTIMING) {
            struct can_bittiming timing;
            if (attribute.payload_length < sizeof(timing)) return OA_CAN_EFRAME;
            (void)memcpy(&timing, attribute.payload, sizeof(timing));
            item->data_bitrate = timing.bitrate;
        } else if (attribute.type == IFLA_CAN_CTRLMODE) {
            struct can_ctrlmode mode;
            if (attribute.payload_length < sizeof(mode)) return OA_CAN_EFRAME;
            (void)memcpy(&mode, attribute.payload, sizeof(mode));
            item->fd_enabled = (uint8_t)((mode.flags & CAN_CTRLMODE_FD) != 0u);
        }
        offset = attribute.next_offset;
    }
    return OA_CAN_OK;
}

static oa_can_status oa_parse_linkinfo(const unsigned char *data, size_t length,
                                       oa_can_linux_interface *item, int *out_is_can) {
    const unsigned char *info_data = NULL;
    size_t info_data_length = 0u;
    size_t offset = 0u;
    int is_can = 0;
    while (offset < length) {
        oa_attr_view attribute;
        oa_can_status status;
        if (length - offset < sizeof(struct rtattr)) {
            if (!oa_padding_is_zero(data + offset, length - offset)) return OA_CAN_EFRAME;
            break;
        }
        status = oa_next_attribute(data, length, offset, &attribute);
        if (status != OA_CAN_OK) return status;
        if (attribute.type == IFLA_INFO_KIND) {
            const unsigned char *terminator = (const unsigned char *)memchr(attribute.payload, '\0',
                                                                            attribute.payload_length);
            if (terminator == NULL) return OA_CAN_EFRAME;
            if ((size_t)(terminator - attribute.payload) == 3u &&
                memcmp(attribute.payload, "can", 3u) == 0) is_can = 1;
        } else if (attribute.type == IFLA_INFO_DATA) {
            info_data = attribute.payload;
            info_data_length = attribute.payload_length;
        }
        offset = attribute.next_offset;
    }
    if (is_can && info_data != NULL) {
        oa_can_status status = oa_parse_can_data(info_data, info_data_length, item);
        if (status != OA_CAN_OK) return status;
    }
    *out_is_can = is_can;
    return OA_CAN_OK;
}

static oa_can_status oa_parse_link(const unsigned char *message, size_t message_length,
                                   oa_can_linux_interface *item, int *out_is_can) {
    struct ifinfomsg info;
    size_t offset;
    if (message_length < sizeof(info)) return OA_CAN_EFRAME;
    (void)memcpy(&info, message, sizeof(info));
    if (info.ifi_index <= 0) return OA_CAN_EFRAME;
    (void)memset(item, 0, sizeof(*item));
    item->struct_size = (uint32_t)sizeof(*item);
    item->abi_version = OA_CAN_ABI_VERSION;
    item->ifindex = (uint32_t)info.ifi_index;
    item->flags = info.ifi_flags;
    item->link_up = (uint8_t)((info.ifi_flags & IFF_RUNNING) != 0u);
    *out_is_can = 0;
    offset = oa_align_netlink(sizeof(info));
    if (offset > message_length) return OA_CAN_EFRAME;
    while (offset < message_length) {
        oa_attr_view attribute;
        oa_can_status status;
        if (message_length - offset < sizeof(struct rtattr)) {
            if (!oa_padding_is_zero(message + offset, message_length - offset)) return OA_CAN_EFRAME;
            break;
        }
        status = oa_next_attribute(message, message_length, offset, &attribute);
        if (status != OA_CAN_OK) return status;
        if (attribute.type == IFLA_IFNAME) {
            const unsigned char *terminator = (const unsigned char *)memchr(attribute.payload, '\0',
                                                                            attribute.payload_length);
            size_t name_length;
            if (terminator == NULL) return OA_CAN_EFRAME;
            name_length = (size_t)(terminator - attribute.payload);
            if (name_length == 0u || name_length >= sizeof(item->name)) return OA_CAN_EFRAME;
            (void)memcpy(item->name, attribute.payload, name_length);
            item->name[name_length] = '\0';
        } else if (attribute.type == IFLA_MTU) {
            if (attribute.payload_length < sizeof(item->mtu)) return OA_CAN_EFRAME;
            (void)memcpy(&item->mtu, attribute.payload, sizeof(item->mtu));
        } else if (attribute.type == IFLA_LINKINFO) {
            status = oa_parse_linkinfo(attribute.payload, attribute.payload_length, item, out_is_can);
            if (status != OA_CAN_OK) return status;
        }
        offset = attribute.next_offset;
    }
    if (*out_is_can != 0 && item->name[0] == '\0') return OA_CAN_EFRAME;
    return OA_CAN_OK;
}

oa_can_status oa_can_linux_parse_datagram(const void *data, size_t length,
                                           uint32_t expected_sequence,
                                           uint32_t sender_port_id,
                                           int was_truncated,
                                           oa_can_linux_interface *interfaces,
                                           size_t capacity,
                                           size_t *in_out_count,
                                           int *out_done) {
    const unsigned char *bytes = (const unsigned char *)data;
    size_t offset = 0u;
    size_t found;
    if (data == NULL || in_out_count == NULL || out_done == NULL ||
        (capacity != 0u && interfaces == NULL) || sender_port_id != 0u || was_truncated != 0 ||
        length == 0u) return OA_CAN_EFRAME;
    found = *in_out_count;
    *out_done = 0;
    while (offset < length) {
        struct nlmsghdr header;
        size_t message_length;
        size_t aligned_length;
        if (length - offset < sizeof(header)) return OA_CAN_EFRAME;
        (void)memcpy(&header, bytes + offset, sizeof(header));
        message_length = header.nlmsg_len;
        if (message_length < sizeof(header) || message_length > length - offset ||
            header.nlmsg_seq != expected_sequence || (header.nlmsg_flags & NLM_F_DUMP_INTR) != 0u) {
            return OA_CAN_EFRAME;
        }
        if (header.nlmsg_type == NLMSG_DONE) {
            *in_out_count = found;
            *out_done = 1;
            return OA_CAN_OK;
        }
        if (header.nlmsg_type == NLMSG_ERROR || header.nlmsg_type == NLMSG_OVERRUN) return OA_CAN_EIO;
        if (header.nlmsg_type == RTM_NEWLINK) {
            oa_can_linux_interface item;
            oa_can_status status;
            int is_can;
            status = oa_parse_link(bytes + offset + sizeof(header), message_length - sizeof(header),
                                   &item, &is_can);
            if (status != OA_CAN_OK) return status;
            if (is_can != 0) {
                if (found == SIZE_MAX) return OA_CAN_ERANGE;
                if (found < capacity) {
                    if ((size_t)interfaces[found].struct_size < sizeof(interfaces[found]) ||
                        interfaces[found].abi_version != OA_CAN_ABI_VERSION) return OA_CAN_EINVAL;
                    interfaces[found] = item;
                }
                ++found;
            }
        }
        aligned_length = oa_align_netlink(message_length);
        if (aligned_length > length - offset) {
            if (message_length != length - offset) return OA_CAN_EFRAME;
            aligned_length = message_length;
        }
        offset += aligned_length;
    }
    *in_out_count = found;
    return OA_CAN_OK;
}

oa_can_status oa_can_linux_list_interfaces(oa_can_linux_interface *interfaces,
                                           size_t capacity, size_t *out_count) {
    struct {
        struct nlmsghdr header;
        struct ifinfomsg info;
    } request;
    struct sockaddr_nl local_address;
    struct sockaddr_nl kernel_address;
    struct timeval timeout;
    size_t found = 0u;
    uint32_t sequence = UINT32_C(0x4f414331);
    unsigned int datagrams;
    int socket_fd;
    if (out_count == NULL || (capacity != 0u && interfaces == NULL)) return OA_CAN_EINVAL;
    *out_count = 0u;
    socket_fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
    if (socket_fd < 0) return OA_CAN_EIO;
    (void)memset(&local_address, 0, sizeof(local_address));
    local_address.nl_family = AF_NETLINK;
    if (bind(socket_fd, (const struct sockaddr *)&local_address, sizeof(local_address)) < 0) {
        (void)close(socket_fd);
        return OA_CAN_EIO;
    }
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        (void)close(socket_fd);
        return OA_CAN_EIO;
    }
    (void)memset(&request, 0, sizeof(request));
    request.header.nlmsg_len = (uint32_t)sizeof(request);
    request.header.nlmsg_type = RTM_GETLINK;
    request.header.nlmsg_flags = NLM_F_REQUEST | NLM_F_DUMP;
    request.header.nlmsg_seq = sequence;
    request.info.ifi_family = AF_UNSPEC;
    (void)memset(&kernel_address, 0, sizeof(kernel_address));
    kernel_address.nl_family = AF_NETLINK;
    {
        ssize_t sent = sendto(socket_fd, &request, sizeof(request), 0,
                              (const struct sockaddr *)&kernel_address, sizeof(kernel_address));
        if (sent < 0 || (size_t)sent != sizeof(request)) {
            (void)close(socket_fd);
            return OA_CAN_EIO;
        }
    }
    for (datagrams = 0u; datagrams < 256u; ++datagrams) {
        union {
            max_align_t alignment;
            unsigned char bytes[8192];
        } buffer;
        struct sockaddr_nl sender;
        struct iovec vector;
        struct msghdr message;
        ssize_t received;
        int done;
        oa_can_status status;
        (void)memset(&sender, 0, sizeof(sender));
        vector.iov_base = buffer.bytes;
        vector.iov_len = sizeof(buffer.bytes);
        (void)memset(&message, 0, sizeof(message));
        message.msg_name = &sender;
        message.msg_namelen = (socklen_t)sizeof(sender);
        message.msg_iov = &vector;
        message.msg_iovlen = 1u;
        received = recvmsg(socket_fd, &message, 0);
        if (received <= 0 || message.msg_namelen < sizeof(sender) || sender.nl_family != AF_NETLINK) {
            (void)close(socket_fd);
            return OA_CAN_EIO;
        }
        status = oa_can_linux_parse_datagram(buffer.bytes, (size_t)received, sequence,
                                              sender.nl_pid, (message.msg_flags & MSG_TRUNC) != 0,
                                              interfaces, capacity, &found, &done);
        if (status != OA_CAN_OK) {
            (void)close(socket_fd);
            return status;
        }
        if (done != 0) {
            (void)close(socket_fd);
            *out_count = found;
            return OA_CAN_OK;
        }
    }
    (void)close(socket_fd);
    return OA_CAN_EIO;
}
#else
oa_can_status oa_can_linux_parse_datagram(const void *data, size_t length,
                                           uint32_t expected_sequence,
                                           uint32_t sender_port_id,
                                           int was_truncated,
                                           oa_can_linux_interface *interfaces,
                                           size_t capacity,
                                           size_t *in_out_count,
                                           int *out_done) {
    (void)data;
    (void)length;
    (void)expected_sequence;
    (void)sender_port_id;
    (void)was_truncated;
    (void)interfaces;
    (void)capacity;
    (void)in_out_count;
    (void)out_done;
    return OA_CAN_EUNSUPPORTED;
}

oa_can_status oa_can_linux_list_interfaces(oa_can_linux_interface *interfaces,
                                           size_t capacity, size_t *out_count) {
    (void)interfaces;
    (void)capacity;
    if (out_count == NULL) return OA_CAN_EINVAL;
    *out_count = 0u;
    return OA_CAN_EUNSUPPORTED;
}
#endif
