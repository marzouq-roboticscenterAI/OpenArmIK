/* SPDX-License-Identifier: Apache-2.0 */
#ifndef OPENARM_TRANSPORT_H
#define OPENARM_TRANSPORT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OA_TRANSPORT_ABI_VERSION UINT32_C(1)
#define OA_TRANSPORT_INTERFACE_NAME_CAPACITY 16u
#define OA_TRANSPORT_MAX_FILTERS 64u
#define OA_TRANSPORT_SFF_MASK UINT32_C(0x000007ff)
#define OA_TRANSPORT_CAN_ERROR_MASK UINT32_C(0x1fffffff)

typedef uint32_t oa_transport_status;
#define OA_TRANSPORT_OK UINT32_C(0)
#define OA_TRANSPORT_EINVAL UINT32_C(1)
#define OA_TRANSPORT_EABI UINT32_C(2)
#define OA_TRANSPORT_ERANGE UINT32_C(3)
#define OA_TRANSPORT_EPERMISSION UINT32_C(4)
#define OA_TRANSPORT_ETIMEOUT UINT32_C(5)
#define OA_TRANSPORT_ECLOSED UINT32_C(6)
#define OA_TRANSPORT_EIO UINT32_C(7)
#define OA_TRANSPORT_ELINK UINT32_C(8)
#define OA_TRANSPORT_EFRAME UINT32_C(9)
#define OA_TRANSPORT_ENOMEM UINT32_C(10)
#define OA_TRANSPORT_EUNSUPPORTED UINT32_C(11)

typedef uint32_t oa_transport_permission;
#define OA_TRANSPORT_PERMISSION_QUERY (UINT32_C(1) << 0)
#define OA_TRANSPORT_PERMISSION_DISABLE (UINT32_C(1) << 1)
#define OA_TRANSPORT_PERMISSION_CONTROL (UINT32_C(1) << 2)
#define OA_TRANSPORT_PERMISSION_COMMISSION (UINT32_C(1) << 3)
#define OA_TRANSPORT_PERMISSION_ALL                                              \
    (OA_TRANSPORT_PERMISSION_QUERY | OA_TRANSPORT_PERMISSION_DISABLE |          \
     OA_TRANSPORT_PERMISSION_CONTROL | OA_TRANSPORT_PERMISSION_COMMISSION)

typedef uint32_t oa_transport_frame_class;
#define OA_TRANSPORT_FRAME_REGISTER_QUERY UINT32_C(1)
#define OA_TRANSPORT_FRAME_STATUS_QUERY UINT32_C(2)
#define OA_TRANSPORT_FRAME_DISABLE UINT32_C(3)
#define OA_TRANSPORT_FRAME_ENABLE UINT32_C(4)
#define OA_TRANSPORT_FRAME_MOTION UINT32_C(5)
#define OA_TRANSPORT_FRAME_REGISTER_WRITE UINT32_C(6)
#define OA_TRANSPORT_FRAME_SET_ZERO UINT32_C(7)
#define OA_TRANSPORT_FRAME_CLEAR_ERROR UINT32_C(8)
#define OA_TRANSPORT_FRAME_SAVE_PARAMETERS UINT32_C(9)
#define OA_TRANSPORT_FRAME_UNKNOWN UINT32_C(10)

typedef uint32_t oa_transport_event_kind;
#define OA_TRANSPORT_EVENT_FRAME UINT32_C(1)
#define OA_TRANSPORT_EVENT_CAN_ERROR UINT32_C(2)
#define OA_TRANSPORT_EVENT_LINK UINT32_C(3)

typedef uint32_t oa_transport_event_flag;
#define OA_TRANSPORT_EVENT_KERNEL_TIMESTAMP (UINT32_C(1) << 0)
#define OA_TRANSPORT_EVENT_RX_OVERFLOW (UINT32_C(1) << 1)

typedef uint32_t oa_transport_clock;
#define OA_TRANSPORT_CLOCK_NONE UINT32_C(0)
#define OA_TRANSPORT_CLOCK_REALTIME UINT32_C(1)

typedef struct oa_transport oa_transport;

typedef struct oa_transport_frame {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t can_id;
    uint8_t dlc;
    uint8_t data[8];
} oa_transport_frame;

typedef struct oa_transport_filter {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t can_id;
    uint32_t can_mask;
} oa_transport_filter;

typedef struct oa_transport_open_options {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t receive_buffer_bytes;
    uint32_t can_error_mask;
    uint64_t max_deadline_horizon_ns;
} oa_transport_open_options;

/*
 * Passing NULL selects query-only operation. Dangerous permissions require a
 * nonzero, future CLOCK_MONOTONIC expiry and are rechecked for every send.
 */
typedef struct oa_transport_capability {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t permissions;
    uint32_t reserved;
    uint64_t expiry_monotonic_ns;
} oa_transport_capability;

typedef struct oa_transport_send_result {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_transport_frame_class frame_class;
    uint32_t reserved;
    uint64_t sent_monotonic_ns;
} oa_transport_send_result;

typedef struct oa_transport_event {
    uint32_t struct_size;
    uint32_t abi_version;
    oa_transport_event_kind kind;
    uint32_t flags;
    uint64_t dequeue_monotonic_ns;
    uint64_t kernel_timestamp_ns;
    oa_transport_clock kernel_timestamp_clock;
    uint32_t can_error_mask;
    uint32_t rx_overflow_count;
    uint32_t rx_overflow_delta;
    uint8_t link_up;
    uint8_t reserved[7];
    oa_transport_frame frame;
} oa_transport_event;

oa_transport_status oa_transport_now_monotonic_ns(uint64_t *out_now_ns);

oa_transport_status oa_transport_classify_send_frame(
    const oa_transport_frame *frame, oa_transport_frame_class *out_class);

/*
 * Opens an existing classic-CAN interface without changing link state, bitrate,
 * MTU, or any motor. filters may be NULL only when filter_count is zero.
 */
oa_transport_status oa_transport_open(
    const char *interface_name, const oa_transport_open_options *options,
    const oa_transport_filter *filters, size_t filter_count,
    const oa_transport_capability *capability, oa_transport **out_transport);

/* close is idempotent and may interrupt a concurrent receive or send. */
oa_transport_status oa_transport_close(oa_transport *transport);

/* destroy must be called only after all other calls except close have returned. */
void oa_transport_destroy(oa_transport *transport);

oa_transport_status oa_transport_send(
    oa_transport *transport, const oa_transport_frame *frame,
    uint64_t deadline_monotonic_ns, oa_transport_send_result *out_result);

oa_transport_status oa_transport_receive(
    oa_transport *transport, uint64_t deadline_monotonic_ns,
    oa_transport_event *out_event);

#ifdef __cplusplus
}
#endif

#endif
