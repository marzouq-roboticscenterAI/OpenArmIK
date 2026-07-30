/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_runtime_units.h"

#include <type_traits>

static_assert(std::is_same_v<
              decltype(((oa_runtime_paired_tcp_move_with_units *)nullptr)->left_tcp.x),
              double>);
static_assert(std::is_same_v<decltype(oa_vec3d::x), double>);

int main() {
    oa_runtime_paired_tcp_move_with_units request{};
    request.struct_size = sizeof(request);
    request.abi_version = OA_RUNTIME_ABI_VERSION;
    request.coordinate_unit = OA_LENGTH_UNIT_METRES;
    request.left_tcp.x = 1.000000000000001;
    request.right_tcp.x = 1.000000000000001;
    return request.left_tcp.x == request.right_tcp.x ? 0 : 1;
}
