#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1
#include "openarm_can.h"
#include "openarm_units.h"
#include "openarm_model.h"
#include "openarm_route.h"
#include "openarm_commission.h"
#include "openarm_transport.h"
#include "openarm_control.h"
#include "openarm_runtime.h"
#include "openarm_runtime_units.h"

_Static_assert(sizeof(((oa_route_request *)0)->target_tcp_m[0][0]) == sizeof(double),
               "route coordinates narrowed");

#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static int checkpoint_cleared(
    const oa_runtime_persistence_checkpoint *checkpoint) {
    size_t index = 0U;
    if (checkpoint->struct_size != (uint32_t)sizeof(*checkpoint) ||
        checkpoint->abi_version != OA_RUNTIME_ABI_VERSION ||
        checkpoint->revision != 0U) {
        return 0;
    }
    for (index = 0U; index < sizeof(checkpoint->content_sha256); ++index) {
        if (checkpoint->content_sha256[index] != '\0') return 0;
    }
    return 1;
}

static int persistence_checkpoint_alias_probe(oa_runtime_manifest *manifest) {
    char directory[128] = {0};
    char artifact[160] = {0};
    uint8_t key[OA_RUNTIME_PERSISTENCE_KEY_BYTES] = {0};
    oa_runtime_persistence_checkpoint checkpoint = {0};
    oa_runtime_persistence_checkpoint committed = {0};
    oa_runtime_persistence_authority *authority = NULL;
    oa_runtime_manifest *loaded = NULL;
    oa_runtime_status status = OA_RUNTIME_EIO;
    size_t index = 0U;
    int result = 1;

    if (snprintf(directory, sizeof(directory),
                 "/tmp/openarm-runtime-installed-alias-%ld", (long)getpid()) <= 0 ||
        snprintf(artifact, sizeof(artifact), "%s/manifest.oarm", directory) <= 0 ||
        mkdir(directory, 0700) != 0) {
        return 1;
    }
    for (index = 0U; index < sizeof(key); ++index) {
        key[index] = (uint8_t)(index + 1U);
    }
    checkpoint.struct_size = (uint32_t)sizeof(checkpoint);
    checkpoint.abi_version = OA_RUNTIME_ABI_VERSION;
    if (oa_runtime_persistence_authority_open_v2(
            directory, key, "installed-alias", &checkpoint, &authority) !=
        OA_RUNTIME_OK) {
        goto cleanup;
    }

    /* Initial and idempotent save support the same checkpoint as input/output. */
    if (oa_runtime_manifest_save_v2(manifest, authority, "manifest.oarm",
                                    &checkpoint, &checkpoint) != OA_RUNTIME_OK ||
        checkpoint.revision != 1U) {
        goto cleanup;
    }
    committed = checkpoint;
    if (oa_runtime_manifest_save_v2(manifest, authority, "manifest.oarm",
                                    &checkpoint, &checkpoint) != OA_RUNTIME_OK ||
        checkpoint.revision != committed.revision ||
        strcmp(checkpoint.content_sha256, committed.content_sha256) != 0) {
        goto cleanup;
    }

    if (oa_runtime_manifest_load_authenticated_v2(
            authority, "manifest.oarm", &checkpoint, &loaded, &checkpoint) !=
            OA_RUNTIME_OK ||
        loaded == NULL || checkpoint.revision != committed.revision ||
        strcmp(checkpoint.content_sha256, committed.content_sha256) != 0) {
        goto cleanup;
    }
    oa_runtime_manifest_destroy(loaded);
    loaded = NULL;
    if (oa_runtime_manifest_recover_v2(
            authority, "manifest.oarm", &checkpoint, &loaded, &checkpoint) !=
            OA_RUNTIME_OK ||
        loaded == NULL || checkpoint.revision != committed.revision ||
        strcmp(checkpoint.content_sha256, committed.content_sha256) != 0) {
        goto cleanup;
    }
    oa_runtime_manifest_destroy(loaded);
    loaded = NULL;

    checkpoint = committed;
    checkpoint.content_sha256[0] = 'A';
    status = oa_runtime_manifest_load_authenticated_v2(
        authority, "manifest.oarm", &checkpoint, &loaded, &checkpoint);
    if (status != OA_RUNTIME_EINVAL || loaded != NULL ||
        !checkpoint_cleared(&checkpoint)) {
        goto cleanup;
    }
    checkpoint = committed;
    checkpoint.content_sha256[0] = 'A';
    status = oa_runtime_manifest_save_v2(
        manifest, authority, "manifest.oarm", &checkpoint, &checkpoint);
    if (status != OA_RUNTIME_EINVAL || !checkpoint_cleared(&checkpoint)) {
        goto cleanup;
    }
    checkpoint = committed;
    checkpoint.content_sha256[0] = 'A';
    status = oa_runtime_manifest_recover_v2(
        authority, "manifest.oarm", &checkpoint, &loaded, &checkpoint);
    if (status != OA_RUNTIME_EINVAL || loaded != NULL ||
        !checkpoint_cleared(&checkpoint)) {
        goto cleanup;
    }
    result = 0;

cleanup:
    if (loaded != NULL) oa_runtime_manifest_destroy(loaded);
    oa_runtime_persistence_authority_destroy(authority);
    (void)unlink(artifact);
    (void)rmdir(directory);
    return result;
}

int main(void) {
    oa_can_frame frame = {0};
    uint64_t now_ns = 0U;
    oa_manifest *manifest = NULL;
    oa_model_status model_status = OA_MODEL_OK;
    oa_control_status control_status = OA_CONTROL_OK;
    oa_runtime_manifest *runtime_manifest = NULL;
    frame.struct_size = (uint32_t)sizeof(frame);
    frame.abi_version = OA_CAN_ABI_VERSION;
    if (oa_can_make_disable(1U, &frame) != OA_CAN_OK ||
        model_status != OA_MODEL_OK || control_status != OA_CONTROL_OK ||
        oa_model_left_v10_bimanual() == NULL ||
        oa_transport_now_monotonic_ns(&now_ns) != OA_TRANSPORT_OK ||
        oa_manifest_load(NULL, NULL, &manifest) != OA_CONTROL_EINVAL ||
        oa_runtime_manifest_create_virtual(&runtime_manifest) != OA_RUNTIME_OK ||
        persistence_checkpoint_alias_probe(runtime_manifest) != 0) {
        return 1;
    }
    oa_runtime_manifest_destroy(runtime_manifest);
    oa_commission_manual_destroy(NULL);
    return now_ns == 0U ? 1 : 0;
}
