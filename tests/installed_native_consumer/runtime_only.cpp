#include "openarm_runtime.h"

#include <cstdint>

int main() {
    oa_runtime_manifest *manifest = nullptr;
    oa_runtime *runtime = nullptr;
    oa_runtime_options options{};
    options.struct_size = sizeof(options);
    options.abi_version = OA_RUNTIME_ABI_VERSION;
    options.backend = OA_RUNTIME_BACKEND_SOCKETCAN_QUERY;
    options.cycle_ns = 1000000U;
    options.feedback_timeout_ns = 50000000U;
    options.maximum_cross_bus_skew_ns = 1000000U;
    options.collision_scene_revision = 1U;
    if (oa_runtime_manifest_create_virtual(&manifest) != OA_RUNTIME_OK ||
        oa_runtime_create(&options, manifest, &runtime) != OA_RUNTIME_OK) {
        return 1;
    }
    oa_runtime_inventory *inventory = reinterpret_cast<oa_runtime_inventory *>(
        static_cast<std::uintptr_t>(1U));
    const bool disabled =
        oa_runtime_inventory_query(runtime, nullptr, &inventory) ==
            OA_RUNTIME_EUNSUPPORTED &&
        inventory == nullptr;
    oa_runtime_destroy(runtime);
    oa_runtime_manifest_destroy(manifest);
    return disabled ? 0 : 1;
}
