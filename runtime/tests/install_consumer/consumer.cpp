#if defined(OA_RUNTIME_USE_CURRENT_HEADER)
#include <openarm_runtime.h>
#else
#include <openarm_runtime_abi_v1/openarm_runtime.h>
#endif
#include <openarm_runtime_abi_v1/layout_contract.h>
#include <openarm_runtime_abi_v1/symbol_references.h>
#include <type_traits>
using save_signature = oa_runtime_status (*)(const oa_runtime_manifest *, const oa_runtime_persistence_authority *, const char *);
static_assert(std::is_same_v<decltype(&oa_runtime_manifest_save), save_signature>);
int main() {
    oa_runtime_manifest *manifest = nullptr;
    if (oa_runtime_manifest_create_virtual(&manifest) != OA_RUNTIME_OK) return 1;
    oa_runtime_manifest_destroy(manifest);
    return 0;
}
