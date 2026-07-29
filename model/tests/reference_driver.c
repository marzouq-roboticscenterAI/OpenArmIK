/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_model.h"
#include <stdio.h>

int main(void) {
    int side;
    double q[OA_DOF];
    size_t i, row;
    oa_fk_result fk;
    oa_jacobian jacobian;
    const oa_model *model;
    while (scanf("%d", &side) == 1) {
        for (i = 0; i < OA_DOF; ++i) if (scanf("%lf", &q[i]) != 1) return 2;
        model = side == 0 ? oa_model_left_v10_bimanual() : oa_model_right_v10_bimanual();
        if (oa_fk(model, q, &fk) != OA_MODEL_OK || oa_geometric_jacobian(model, q, &jacobian) != OA_MODEL_OK) return 3;
        for (i = 0; i < 16; ++i) printf("%.17g ", fk.hand_tcp.m[i]);
        for (row = 0; row < 6; ++row) for (i = 0; i < OA_DOF; ++i) printf("%.17g ", jacobian.value[row][i]);
        putchar('\n');
    }
    return 0;
}
