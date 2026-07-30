#include "openarm_runtime.h"
#include "layout_contract.h"
#include "symbol_references.h"
#include <type_traits>
using save_signature = oa_runtime_status (*)(const oa_runtime_manifest *, const oa_runtime_persistence_authority *, const char *);
static_assert(std::is_same_v<decltype(&oa_runtime_manifest_save), save_signature>);
int main() {
    oa_runtime_manifest *manifest = nullptr;
    oa_runtime_manifest_summary summary{};
    if (oa_runtime_manifest_create_virtual(&manifest) != OA_RUNTIME_OK || manifest == nullptr) return 1;
    summary.struct_size = sizeof(summary);
    summary.abi_version = OA_RUNTIME_ABI_VERSION;
    if (oa_runtime_manifest_get_summary(manifest, &summary) != OA_RUNTIME_OK ||
        summary.state != OA_RUNTIME_MANIFEST_ARMABLE ||
        summary.integrity_kind != OA_RUNTIME_INTEGRITY_UNKEYED_SHA256) return 2;
    save_signature save = &oa_runtime_manifest_save;
    if (save(manifest, nullptr, "abi-v1-canary") != OA_RUNTIME_EINVAL) return 3;
    oa_runtime_manifest_destroy(manifest);
    return 0;
}
