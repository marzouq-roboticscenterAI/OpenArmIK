/* SPDX-License-Identifier: Apache-2.0 */
#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1
#include <openarm_model.h>
#include <openarm_control.h>

int main(void) {
    const oa_model_status model_status = OA_MODEL_OK;
    const oa_control_status control_status = OA_CONTROL_OK;
    return model_status == OA_MODEL_OK && control_status == OA_CONTROL_OK ? 0 : 1;
}
