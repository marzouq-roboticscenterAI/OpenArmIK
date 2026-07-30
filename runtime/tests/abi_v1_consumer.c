#include "openarm_runtime.h"
#include "layout_contract.h"
#include "symbol_references.h"
#include <string.h>
typedef oa_runtime_status (*save_signature)(const oa_runtime_manifest *, const oa_runtime_persistence_authority *, const char *);
int main(void) {
    oa_runtime_manifest *manifest = NULL;
    oa_runtime_manifest_summary summary;
    save_signature save = &oa_runtime_manifest_save;
    if (oa_runtime_manifest_create_virtual(&manifest) != OA_RUNTIME_OK || manifest == NULL) return 1;
    memset(&summary, 0x5a, sizeof(summary));
    summary.struct_size = (uint32_t)(sizeof(summary) - 1U);
    summary.abi_version = OA_RUNTIME_ABI_VERSION;
    if (oa_runtime_manifest_get_summary(manifest, &summary) != OA_RUNTIME_EABI ||
        summary.struct_size != (uint32_t)(sizeof(summary) - 1U)) return 2;
    memset(&summary, 0, sizeof(summary));
    summary.struct_size = (uint32_t)sizeof(summary);
    summary.abi_version = OA_RUNTIME_ABI_VERSION;
    if (oa_runtime_manifest_get_summary(manifest, &summary) != OA_RUNTIME_OK ||
        summary.state != OA_RUNTIME_MANIFEST_ARMABLE || summary.struct_size != sizeof(summary)) return 3;
    if (save(manifest, NULL, "abi-v1-canary") != OA_RUNTIME_EINVAL) return 4;
    oa_runtime_manifest_destroy(manifest);
    return 0;
}
