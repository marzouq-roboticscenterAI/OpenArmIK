/* SPDX-License-Identifier: Apache-2.0 */
#ifndef OPENARM_UNITS_H
#define OPENARM_UNITS_H

#include <float.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t oa_length_unit;
#define OA_LENGTH_UNIT_METRES UINT32_C(1)
#define OA_LENGTH_UNIT_CENTIMETRES UINT32_C(2)
#define OA_LENGTH_UNIT_INCHES UINT32_C(3)

typedef uint32_t oa_units_status;
#define OA_UNITS_OK UINT32_C(0)
#define OA_UNITS_EINVAL UINT32_C(1)
#define OA_UNITS_ENONFINITE UINT32_C(2)
#define OA_UNITS_EOVERFLOW UINT32_C(3)

typedef struct oa_vec3d {
    double x;
    double y;
    double z;
} oa_vec3d;

#if defined(__cplusplus)
static_assert(sizeof(double) == 8, "OpenArm coordinates require 64-bit double");
static_assert(FLT_RADIX == 2, "OpenArm coordinates require binary radix");
static_assert(DBL_MANT_DIG == 53, "OpenArm coordinates require IEEE binary64");
static_assert(DBL_MAX_EXP == 1024, "OpenArm coordinates require binary64 maximum exponent");
static_assert(DBL_MIN_EXP == -1021, "OpenArm coordinates require binary64 minimum exponent");
static_assert(sizeof(oa_length_unit) == 4, "oa_length_unit must be uint32_t");
static_assert(sizeof(oa_vec3d) == 3 * sizeof(double), "oa_vec3d must be three doubles");
static_assert(alignof(oa_vec3d) == alignof(double), "oa_vec3d alignment changed");
static_assert(offsetof(oa_vec3d, x) == 0, "oa_vec3d.x layout changed");
static_assert(offsetof(oa_vec3d, y) == sizeof(double), "oa_vec3d.y layout changed");
static_assert(offsetof(oa_vec3d, z) == 2 * sizeof(double), "oa_vec3d.z layout changed");
#else
_Static_assert(sizeof(double) == 8, "OpenArm coordinates require 64-bit double");
_Static_assert(FLT_RADIX == 2, "OpenArm coordinates require binary radix");
_Static_assert(DBL_MANT_DIG == 53, "OpenArm coordinates require IEEE binary64");
_Static_assert(DBL_MAX_EXP == 1024, "OpenArm coordinates require binary64 maximum exponent");
_Static_assert(DBL_MIN_EXP == -1021, "OpenArm coordinates require binary64 minimum exponent");
_Static_assert(sizeof(oa_length_unit) == 4, "oa_length_unit must be uint32_t");
_Static_assert(sizeof(oa_vec3d) == 3 * sizeof(double), "oa_vec3d must be three doubles");
_Static_assert(_Alignof(oa_vec3d) == _Alignof(double), "oa_vec3d alignment changed");
_Static_assert(offsetof(oa_vec3d, x) == 0, "oa_vec3d.x layout changed");
_Static_assert(offsetof(oa_vec3d, y) == sizeof(double), "oa_vec3d.y layout changed");
_Static_assert(offsetof(oa_vec3d, z) == 2 * sizeof(double), "oa_vec3d.z layout changed");
#endif

/* Convert all three coordinates with the binary64 scale associated with each
 * unit. input and output may be the same object. On failure output is unchanged.
 * Expanding conversions conservatively reject magnitudes at or above the
 * DBL_MAX/|factor| boundary, independent of the caller's floating-point rounding
 * mode. */
oa_units_status oa_vec3d_convert(const oa_vec3d *input,
                                 oa_length_unit input_unit,
                                 oa_length_unit output_unit,
                                 oa_vec3d *output);

#ifdef __cplusplus
}
#endif
#endif
