/* SPDX-License-Identifier: Apache-2.0 */
#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1
#include <openarm_control.h>
#include <openarm_model.h>

int main() {
    const oa_model *model = oa_model_right_v10_bimanual();
    double lower = 0.0;
    double upper = 0.0;
    const oa_model_status model_status =
        oa_model_limits(model, 0U, &lower, &upper);
    const oa_control_status control_status = oa_controller_advance(nullptr, UINT64_C(1));
    return model != nullptr && lower < upper && model_status == OA_MODEL_OK &&
                   control_status == OA_CONTROL_EINVAL
               ? 0
               : 1;
}
