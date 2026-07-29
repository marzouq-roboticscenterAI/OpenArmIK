/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_can.h"

#ifdef __linux__
#include <linux/can/netlink.h>
#include <linux/if_link.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>

static void oa_parse_can_data(struct rtattr *attribute, oa_can_linux_interface *out) {
    int remaining = RTA_PAYLOAD(attribute);
    struct rtattr *nested = (struct rtattr *)RTA_DATA(attribute);
    for (; RTA_OK(nested, remaining); nested = RTA_NEXT(nested, remaining)) {
        if (nested->rta_type == IFLA_CAN_BITTIMING && RTA_PAYLOAD(nested) >= (int)sizeof(struct can_bittiming)) {
            out->bitrate = ((const struct can_bittiming *)RTA_DATA(nested))->bitrate;
        } else if (nested->rta_type == IFLA_CAN_DATA_BITTIMING &&
                   RTA_PAYLOAD(nested) >= (int)sizeof(struct can_bittiming)) {
            out->data_bitrate = ((const struct can_bittiming *)RTA_DATA(nested))->bitrate;
        } else if (nested->rta_type == IFLA_CAN_CTRLMODE &&
                   RTA_PAYLOAD(nested) >= (int)sizeof(struct can_ctrlmode)) {
            const struct can_ctrlmode *mode = (const struct can_ctrlmode *)RTA_DATA(nested);
            out->fd_enabled = (uint8_t)((mode->flags & CAN_CTRLMODE_FD) != 0u);
        }
    }
}

static int oa_parse_linkinfo(struct rtattr *attribute, oa_can_linux_interface *out) {
    int remaining = RTA_PAYLOAD(attribute);
    struct rtattr *nested = (struct rtattr *)RTA_DATA(attribute);
    int is_can = 0;
    for (; RTA_OK(nested, remaining); nested = RTA_NEXT(nested, remaining)) {
        if (nested->rta_type == IFLA_INFO_KIND && strcmp((const char *)RTA_DATA(nested), "can") == 0) {
            is_can = 1;
        } else if (nested->rta_type == IFLA_INFO_DATA) {
            oa_parse_can_data(nested, out);
        }
    }
    return is_can;
}

oa_can_status oa_can_linux_list_interfaces(oa_can_linux_interface *interfaces,
                                           size_t capacity, size_t *out_count) {
    struct {
        struct nlmsghdr header;
        struct ifinfomsg info;
    } request;
    char buffer[8192];
    size_t found = 0u;
    int socket_fd;
    uint32_t sequence = 0x4f414331u;
    if (out_count == NULL || (capacity != 0u && interfaces == NULL)) return OA_CAN_EINVAL;
    *out_count = 0u;
    socket_fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
    if (socket_fd < 0) return OA_CAN_EIO;
    (void)memset(&request, 0, sizeof(request));
    request.header.nlmsg_len = NLMSG_LENGTH(sizeof(request.info));
    request.header.nlmsg_type = RTM_GETLINK;
    request.header.nlmsg_flags = NLM_F_REQUEST | NLM_F_DUMP;
    request.header.nlmsg_seq = sequence;
    request.info.ifi_family = AF_UNSPEC;
    if (send(socket_fd, &request, request.header.nlmsg_len, 0) < 0) {
        (void)close(socket_fd);
        return OA_CAN_EIO;
    }
    for (;;) {
        ssize_t received = recv(socket_fd, buffer, sizeof(buffer), 0);
        struct nlmsghdr *header;
        int remaining;
        if (received < 0) {
            (void)close(socket_fd);
            return OA_CAN_EIO;
        }
        remaining = (int)received;
        for (header = (struct nlmsghdr *)buffer; NLMSG_OK(header, remaining); header = NLMSG_NEXT(header, remaining)) {
            struct ifinfomsg *info;
            struct rtattr *attribute;
            int attr_remaining;
            oa_can_linux_interface item;
            int is_can = 0;
            if (header->nlmsg_type == NLMSG_DONE) {
                (void)close(socket_fd);
                *out_count = found;
                return OA_CAN_OK;
            }
            if (header->nlmsg_type == NLMSG_ERROR) {
                (void)close(socket_fd);
                return OA_CAN_EIO;
            }
            if (header->nlmsg_type != RTM_NEWLINK) continue;
            info = (struct ifinfomsg *)NLMSG_DATA(header);
            (void)memset(&item, 0, sizeof(item));
            item.struct_size = (uint32_t)sizeof(item);
            item.abi_version = OA_CAN_ABI_VERSION;
            item.ifindex = (uint32_t)info->ifi_index;
            item.flags = info->ifi_flags;
            item.link_up = (uint8_t)((info->ifi_flags & IFF_RUNNING) != 0u);
            attr_remaining = IFLA_PAYLOAD(header);
            for (attribute = IFLA_RTA(info); RTA_OK(attribute, attr_remaining); attribute = RTA_NEXT(attribute, attr_remaining)) {
                if (attribute->rta_type == IFLA_IFNAME) {
                    (void)strncpy(item.name, (const char *)RTA_DATA(attribute), sizeof(item.name) - 1u);
                } else if (attribute->rta_type == IFLA_MTU && RTA_PAYLOAD(attribute) >= (int)sizeof(uint32_t)) {
                    item.mtu = *(const uint32_t *)RTA_DATA(attribute);
                } else if (attribute->rta_type == IFLA_LINKINFO) {
                    is_can = oa_parse_linkinfo(attribute, &item);
                }
            }
            if (is_can) {
                if (found < capacity) interfaces[found] = item;
                ++found;
            }
        }
    }
}
#else
oa_can_status oa_can_linux_list_interfaces(oa_can_linux_interface *interfaces,
                                           size_t capacity, size_t *out_count) {
    (void)interfaces;
    (void)capacity;
    if (out_count == NULL) return OA_CAN_EINVAL;
    *out_count = 0u;
    return OA_CAN_EUNSUPPORTED;
}
#endif
