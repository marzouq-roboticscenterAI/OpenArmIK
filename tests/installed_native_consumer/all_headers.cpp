#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1
#include "openarm_control.h"
#include "openarm_units.h"
#include "openarm_transport.h"
#include "openarm_commission.h"
#include "openarm_model.h"
#include "openarm_can.h"
#include "openarm_runtime.h"
#include "openarm_runtime_units.h"

#include <cstdint>

int main() {
    oa_can_frame frame{};
    std::uint64_t now_ns = 0U;
    oa_manifest *manifest = nullptr;
    oa_model_status model_status = OA_MODEL_OK;
    oa_control_status control_status = OA_CONTROL_OK;
    oa_runtime_manifest *runtime_manifest = nullptr;
    frame.struct_size = static_cast<std::uint32_t>(sizeof(frame));
    frame.abi_version = OA_CAN_ABI_VERSION;
    if (oa_can_make_disable(1U, &frame) != OA_CAN_OK ||
        model_status != OA_MODEL_OK || control_status != OA_CONTROL_OK ||
        oa_model_right_v10_bimanual() == nullptr ||
        oa_transport_now_monotonic_ns(&now_ns) != OA_TRANSPORT_OK ||
        oa_manifest_load(nullptr, nullptr, &manifest) != OA_CONTROL_EINVAL ||
        oa_runtime_manifest_create_virtual(&runtime_manifest) != OA_RUNTIME_OK) {
        return 1;
    }
    oa_runtime_manifest_destroy(runtime_manifest);
    oa_commission_recipe_destroy(nullptr);
    return now_ns == 0U ? 1 : 0;
}
