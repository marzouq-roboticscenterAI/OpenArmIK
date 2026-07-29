#include "openarm_runtime.h"

int main(void) {
    oa_runtime_manifest *manifest = 0;
    oa_runtime_manifest_summary summary = {0};
    summary.struct_size = (uint32_t)sizeof(summary);
    summary.abi_version = OA_RUNTIME_ABI_VERSION;
    if (oa_runtime_manifest_create_virtual(&manifest) != OA_RUNTIME_OK ||
        oa_runtime_manifest_get_summary(manifest, &summary) != OA_RUNTIME_OK ||
        summary.state != OA_RUNTIME_MANIFEST_ARMABLE) {
        return 1;
    }
    oa_runtime_manifest_destroy(manifest);
    oa_runtime_manifest_destroy(manifest);
    return 0;
}
