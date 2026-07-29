/* SPDX-License-Identifier: Apache-2.0 */
#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1
#include <openarm_control.h>
#include <openarm_model.h>

int main() {
    const oa_control_status control_status = OA_CONTROL_OK;
    const oa_model_status model_status = OA_MODEL_OK;
    return control_status == OA_CONTROL_OK && model_status == OA_MODEL_OK ? 0 : 1;
}
