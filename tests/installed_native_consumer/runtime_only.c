#include "openarm_runtime.h"

#include <stdint.h>

static void init_options(oa_runtime_options *options, oa_runtime_backend backend) {
    *options = (oa_runtime_options){0};
    options->struct_size = (uint32_t)sizeof(*options);
    options->abi_version = OA_RUNTIME_ABI_VERSION;
    options->backend = backend;
    options->cycle_ns = UINT64_C(1000000);
    options->feedback_timeout_ns = UINT64_C(50000000);
    options->maximum_cross_bus_skew_ns = UINT64_C(1000000);
    options->collision_scene_revision = UINT64_C(1);
}

int main(void) {
    oa_runtime_manifest *manifest = NULL;
    oa_runtime *runtime = NULL;
    oa_runtime_inventory *inventory = (oa_runtime_inventory *)(uintptr_t)1U;
    oa_runtime_capability_report report = {0};
    oa_runtime_options options;
    if (oa_runtime_manifest_create_virtual(&manifest) != OA_RUNTIME_OK) return 1;
    init_options(&options, OA_RUNTIME_BACKEND_SOCKETCAN_QUERY);
    if (oa_runtime_create(&options, manifest, &runtime) != OA_RUNTIME_OK) return 1;
    report.struct_size = (uint32_t)sizeof(report);
    report.abi_version = OA_RUNTIME_ABI_VERSION;
    if (oa_runtime_get_capabilities(runtime, &report) != OA_RUNTIME_OK ||
        (report.capabilities & OA_RUNTIME_CAP_PHYSICAL_REGISTER_QUERY) != 0U ||
        oa_runtime_inventory_query(runtime, NULL, &inventory) !=
            OA_RUNTIME_EUNSUPPORTED ||
        inventory != NULL ||
        oa_runtime_configuration_apply_physical(runtime, manifest) !=
            OA_RUNTIME_EUNSUPPORTED) {
        return 1;
    }
    oa_runtime_destroy(runtime);
    oa_runtime_manifest_destroy(manifest);
    return 0;
}
