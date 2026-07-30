/* SPDX-License-Identifier: Apache-2.0 */
#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1
#include <openarm_model.h>
#include <openarm_control.h>

typedef oa_control_status (*plan_with_units_signature)(
    oa_controller *, const oa_paired_tcp_move_with_units *, oa_motion_plan **);

int main(void) {
    const oa_model *model = oa_model_left_v10_bimanual();
    double lower = 0.0;
    double upper = 0.0;
    const oa_model_status model_status =
        oa_model_limits(model, 0U, &lower, &upper);
    const oa_control_status control_status = oa_controller_advance(NULL, UINT64_C(1));
    const plan_with_units_signature plan_with_units =
        &oa_controller_plan_paired_tcp_with_units;
    const oa_vec3d centimetres = {1.000000000000001, 2.0, 3.0};
    oa_vec3d metres = {0.0, 0.0, 0.0};
    return model != NULL && lower < upper && model_status == OA_MODEL_OK &&
                   control_status == OA_CONTROL_EINVAL &&
                   plan_with_units != NULL &&
                   oa_vec3d_convert(&centimetres,
                                     OA_LENGTH_UNIT_CENTIMETRES,
                                     OA_LENGTH_UNIT_METRES, &metres) ==
                       OA_UNITS_OK &&
                   metres.x == centimetres.x * 0.01
               ? 0
               : 1;
}
