#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1
#include "openarm_can.h"
#include "openarm_model.h"
#include "openarm_commission.h"
#include "openarm_transport.h"
#include "openarm_control.h"

#include <stddef.h>
#include <stdint.h>

int main(void) {
    oa_can_frame frame = {0};
    uint64_t now_ns = 0U;
    oa_manifest *manifest = NULL;
    oa_model_status model_status = OA_MODEL_OK;
    oa_control_status control_status = OA_CONTROL_OK;
    frame.struct_size = (uint32_t)sizeof(frame);
    frame.abi_version = OA_CAN_ABI_VERSION;
    if (oa_can_make_disable(1U, &frame) != OA_CAN_OK ||
        model_status != OA_MODEL_OK || control_status != OA_CONTROL_OK ||
        oa_model_left_v10_bimanual() == NULL ||
        oa_transport_now_monotonic_ns(&now_ns) != OA_TRANSPORT_OK ||
        oa_manifest_load(NULL, NULL, &manifest) != OA_CONTROL_EINVAL) {
        return 1;
    }
    oa_commission_manual_destroy(NULL);
    return now_ns == 0U ? 1 : 0;
}
