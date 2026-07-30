/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_units.h"

#include <float.h>
#include <math.h>

static int unit_scale(oa_length_unit unit, double *scale) {
    switch (unit) {
        case OA_LENGTH_UNIT_METRES:
            *scale = 1.0;
            return 1;
        case OA_LENGTH_UNIT_CENTIMETRES:
            *scale = 0.01;
            return 1;
        case OA_LENGTH_UNIT_INCHES:
            *scale = 0.0254;
            return 1;
        default:
            return 0;
    }
}

oa_units_status oa_vec3d_convert(const oa_vec3d *input,
                                 oa_length_unit input_unit,
                                 oa_length_unit output_unit,
                                 oa_vec3d *output) {
    double input_scale;
    double output_scale;
    double factor;
    double absolute_factor;
    double threshold;
    oa_vec3d converted;

    if (input == NULL || output == NULL ||
        !unit_scale(input_unit, &input_scale) ||
        !unit_scale(output_unit, &output_scale)) {
        return OA_UNITS_EINVAL;
    }
    if (!isfinite(input->x) || !isfinite(input->y) || !isfinite(input->z)) {
        return OA_UNITS_ENONFINITE;
    }

    factor = input_scale / output_scale;
    absolute_factor = fabs(factor);
    if (absolute_factor > 1.0) {
        threshold = DBL_MAX / absolute_factor;
        if (fabs(input->x) >= threshold || fabs(input->y) >= threshold ||
            fabs(input->z) >= threshold) {
            return OA_UNITS_EOVERFLOW;
        }
    }
    converted.x = input->x * factor;
    converted.y = input->y * factor;
    converted.z = input->z * factor;
    if (!isfinite(converted.x) || !isfinite(converted.y) ||
        !isfinite(converted.z)) {
        return OA_UNITS_EOVERFLOW;
    }
    *output = converted;
    return OA_UNITS_OK;
}
