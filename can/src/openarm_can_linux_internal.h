/* SPDX-License-Identifier: Apache-2.0 */
#ifndef OPENARM_CAN_LINUX_INTERNAL_H
#define OPENARM_CAN_LINUX_INTERNAL_H

#include "openarm_can.h"

oa_can_status oa_can_linux_parse_datagram(const void *data, size_t length,
                                           uint32_t expected_sequence,
                                           uint32_t sender_port_id,
                                           int was_truncated,
                                           oa_can_linux_interface *interfaces,
                                           size_t capacity,
                                           size_t *in_out_count,
                                           int *out_done);

#endif
