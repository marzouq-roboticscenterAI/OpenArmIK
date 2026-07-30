/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_runtime_units.h"

#include <float.h>
#include <stddef.h>

_Static_assert(DBL_MANT_DIG == 53, "coordinates require binary64");
_Static_assert(sizeof(((oa_runtime_paired_tcp_move_with_units *)0)->left_tcp.x) ==
                   sizeof(double),
               "runtime coordinate input narrowed");

int main(void) {
    oa_runtime_paired_tcp_move_with_units request = {0};
    oa_runtime_plan *unchanged = (oa_runtime_plan *)(uintptr_t)UINT32_C(0x100);
    request.struct_size = (uint32_t)sizeof(request);
    request.abi_version = OA_RUNTIME_ABI_VERSION;
    request.coordinate_unit = UINT32_C(0);
    return oa_runtime_plan_paired_tcp_body_with_units(NULL, &request,
                                                       &unchanged) ==
                       OA_RUNTIME_EINVAL &&
                   unchanged == (oa_runtime_plan *)(uintptr_t)UINT32_C(0x100)
               ? 0
               : 1;
}
