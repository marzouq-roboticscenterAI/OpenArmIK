/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_transport.h"

#include <stddef.h>
#include <stdint.h>

_Static_assert(OA_TRANSPORT_ABI_VERSION == UINT32_C(1), "ABI version");
_Static_assert(sizeof(((oa_transport_frame *)0)->data) == 8u, "classic CAN");
_Static_assert(offsetof(oa_transport_frame, struct_size) == 0u, "record header");

int main(void) {
    oa_transport_frame frame = {0};
    oa_transport_frame_class frame_class = 0u;
    frame.struct_size = (uint32_t)sizeof(frame);
    frame.abi_version = OA_TRANSPORT_ABI_VERSION;
    frame.can_id = OA_TRANSPORT_SFF_MASK;
    frame.dlc = 8u;
    frame.data[0] = 1u;
    frame.data[2] = 0x33u;
    if (oa_transport_classify_send_frame(&frame, &frame_class) != OA_TRANSPORT_OK) {
        return 1;
    }
    return frame_class == OA_TRANSPORT_FRAME_REGISTER_QUERY ? 0 : 2;
}
