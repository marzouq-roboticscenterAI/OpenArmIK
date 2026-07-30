/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_units.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
            ++failures;                                                        \
        }                                                                      \
    } while (0)

static int unchanged(const oa_vec3d *actual, const oa_vec3d *expected) {
    return memcmp(actual, expected, sizeof(*actual)) == 0;
}

static void test_unit_pairs(void) {
    static const oa_length_unit units[] = {
        OA_LENGTH_UNIT_METRES,
        OA_LENGTH_UNIT_CENTIMETRES,
        OA_LENGTH_UNIT_INCHES
    };
    static const double scales[] = {1.0, 0.01, 0.0254};
    const oa_vec3d input = {
        12.345678901234567,
        -98.765432109876543,
        0.000000000000001
    };
    size_t from;
    size_t to;

    for (from = 0; from < 3; ++from) {
        for (to = 0; to < 3; ++to) {
            oa_vec3d output = {0.0, 0.0, 0.0};
            const double factor = scales[from] / scales[to];
            CHECK(oa_vec3d_convert(&input, units[from], units[to], &output) ==
                  OA_UNITS_OK);
            CHECK(output.x == input.x * factor);
            CHECK(output.y == input.y * factor);
            CHECK(output.z == input.z * factor);
        }
    }
}

static void test_precision_and_alias(void) {
    const oa_vec3d precise = {
        12.345678901234567,
        -0.123456789012345,
        0.000000123456789
    };
    oa_vec3d output = {0.0, 0.0, 0.0};
    oa_vec3d alias = precise;
    const double narrowed_x = (double)(float)precise.x * 0.01;

    CHECK(oa_vec3d_convert(&precise, OA_LENGTH_UNIT_CENTIMETRES,
                           OA_LENGTH_UNIT_METRES, &output) == OA_UNITS_OK);
    CHECK(output.x == precise.x * 0.01);
    CHECK(output.x != narrowed_x);

    CHECK(oa_vec3d_convert(&alias, OA_LENGTH_UNIT_INCHES,
                           OA_LENGTH_UNIT_METRES, &alias) == OA_UNITS_OK);
    CHECK(alias.x == precise.x * 0.0254);
    CHECK(alias.y == precise.y * 0.0254);
    CHECK(alias.z == precise.z * 0.0254);
}

static void test_failure_contract(void) {
    const oa_vec3d sentinel = {7.25, -8.5, 9.75};
    oa_vec3d output = sentinel;
    oa_vec3d invalid = {NAN, 1.0, 2.0};
    oa_vec3d overflow = {DBL_MAX, 1.0, 2.0};
    oa_vec3d alias = sentinel;

    CHECK(oa_vec3d_convert(NULL, OA_LENGTH_UNIT_METRES,
                           OA_LENGTH_UNIT_METRES, &output) == OA_UNITS_EINVAL);
    CHECK(unchanged(&output, &sentinel));
    CHECK(oa_vec3d_convert(&sentinel, UINT32_C(0),
                           OA_LENGTH_UNIT_METRES, &output) == OA_UNITS_EINVAL);
    CHECK(unchanged(&output, &sentinel));
    CHECK(oa_vec3d_convert(&sentinel, OA_LENGTH_UNIT_METRES,
                           UINT32_MAX, &output) == OA_UNITS_EINVAL);
    CHECK(unchanged(&output, &sentinel));
    CHECK(oa_vec3d_convert(&invalid, OA_LENGTH_UNIT_METRES,
                           OA_LENGTH_UNIT_METRES, &output) ==
          OA_UNITS_ENONFINITE);
    CHECK(unchanged(&output, &sentinel));
    invalid.x = INFINITY;
    CHECK(oa_vec3d_convert(&invalid, OA_LENGTH_UNIT_METRES,
                           OA_LENGTH_UNIT_METRES, &output) ==
          OA_UNITS_ENONFINITE);
    CHECK(unchanged(&output, &sentinel));
    CHECK(oa_vec3d_convert(&overflow, OA_LENGTH_UNIT_METRES,
                           OA_LENGTH_UNIT_CENTIMETRES, &output) ==
          OA_UNITS_EOVERFLOW);
    CHECK(unchanged(&output, &sentinel));
    CHECK(oa_vec3d_convert(&alias, UINT32_MAX, OA_LENGTH_UNIT_METRES,
                           &alias) == OA_UNITS_EINVAL);
    CHECK(unchanged(&alias, &sentinel));
    CHECK(oa_vec3d_convert(&sentinel, OA_LENGTH_UNIT_METRES,
                           OA_LENGTH_UNIT_METRES, NULL) == OA_UNITS_EINVAL);
}

int main(void) {
    test_unit_pairs();
    test_precision_and_alias();
    test_failure_contract();
    if (failures != 0) {
        fprintf(stderr, "%d unit conversion failures\n", failures);
        return 1;
    }
    puts("OpenArm binary64 unit conversion tests passed");
    return 0;
}
