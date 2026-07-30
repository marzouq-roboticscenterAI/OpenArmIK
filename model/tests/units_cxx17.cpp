/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_units.h"

#include <cfloat>
#include <cstdint>
#include <limits>
#include <type_traits>

static_assert(DBL_MANT_DIG == 53);
static_assert(std::numeric_limits<double>::digits == 53);
static_assert(std::numeric_limits<double>::radix == 2);
static_assert(std::numeric_limits<double>::max_exponent == 1024);
static_assert(std::numeric_limits<double>::min_exponent == -1021);
static_assert(std::numeric_limits<double>::is_iec559);
static_assert(sizeof(oa_length_unit) == sizeof(std::uint32_t));
static_assert(std::is_same_v<decltype(oa_vec3d::x), double>);
static_assert(std::is_same_v<decltype(oa_vec3d::y), double>);
static_assert(std::is_same_v<decltype(oa_vec3d::z), double>);
static_assert(sizeof(oa_vec3d) == 24);

int main() {
    oa_vec3d input{1.000000000000001, 2.0, 3.0};
    oa_vec3d output{};
    return oa_vec3d_convert(&input, OA_LENGTH_UNIT_METRES,
                            OA_LENGTH_UNIT_METRES, &output) == OA_UNITS_OK &&
                   output.x == input.x
               ? 0
               : 1;
}
