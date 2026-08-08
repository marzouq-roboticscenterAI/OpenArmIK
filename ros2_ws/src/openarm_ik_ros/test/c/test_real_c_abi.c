/* SPDX-License-Identifier: Apache-2.0 */
#include <openarm_real.h>

#include <float.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
        ++failures; \
    } \
} while (0)

int main(void) {
    oa_real_client *client = NULL;
    oa_real_result result;
    oa_real_snapshot snapshot;
    oa_vec3d target = {1.0, 2.0, 3.0};

    CHECK(sizeof(double) == 8U);
    CHECK(FLT_RADIX == 2 && DBL_MANT_DIG == 53);
    CHECK(sizeof(target) == 3U * sizeof(double));
    CHECK(offsetof(oa_real_snapshot, joint_position_rad) % _Alignof(double) == 0U);
    CHECK(sizeof(snapshot.joint_position_rad[0][0]) == sizeof(double));
    CHECK(sizeof(snapshot.tcp_m[0][0]) == sizeof(double));

    memset(&result, 0xa5, sizeof(result));
    oa_real_result_init(&result);
    CHECK(result.abi_version == OA_REAL_ABI_VERSION);
    CHECK(result.struct_size == sizeof(result));
    CHECK(result.status == OA_REAL_OK);
    memset(&snapshot, 0xa5, sizeof(snapshot));
    oa_real_snapshot_init(&snapshot);
    CHECK(snapshot.abi_version == OA_REAL_ABI_VERSION);
    CHECK(snapshot.struct_size == sizeof(snapshot));
    CHECK(snapshot.encoder_state_valid == 0U);
    CHECK(snapshot.active_side_mask == 0U);

    CHECK(strcmp(oa_real_status_string(OA_REAL_OK), "ok") == 0);
    CHECK(strcmp(oa_real_status_string(OA_REAL_ESTALE), "stale_encoder_state") == 0);
    CHECK(oa_real_client_create(NULL) == OA_REAL_EINVAL);
    CHECK(oa_real_client_read(NULL, 1U, &snapshot) == OA_REAL_EINVAL);
    CHECK(oa_real_client_move_tcp(NULL, OA_REAL_SIDE_LEFT, &target,
              OA_LENGTH_UNIT_CENTIMETRES, 0.5, 1U, &result) == OA_REAL_EINVAL);

    CHECK(oa_real_client_create(&client) == OA_REAL_OK);
    CHECK(client != NULL);
    if (client != NULL) {
        oa_real_result invalid = result;
        invalid.abi_version = 0U;
        CHECK(oa_real_client_wait_ready(client, 1U, &invalid) == OA_REAL_EABI);
        oa_real_snapshot invalid_snapshot = snapshot;
        invalid_snapshot.struct_size = 0U;
        CHECK(oa_real_client_read(client, 1U, &invalid_snapshot) == OA_REAL_EABI);
    }
    oa_real_client_destroy(client);
    oa_real_client_destroy(NULL);

    if (failures != 0) return 1;
    puts("production real-arm C ABI canary passed");
    return 0;
}
