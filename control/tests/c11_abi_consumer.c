/* SPDX-License-Identifier: Apache-2.0 */
#include "openarm_control.h"

#include <stddef.h>

_Static_assert(offsetof(oa_controller_options, struct_size) == 0, "size is first");
_Static_assert(offsetof(oa_controller_options, abi_version) == 4, "version is second");
_Static_assert(OA_CONTROL_DOF == 7, "frozen v1 dof");
_Static_assert(OA_CONTROL_ARMS == 2, "frozen v1 arm count");

int main(void) {
    oa_controller_options short_record = {0};
    oa_controller *controller = NULL;
    short_record.struct_size = 1U;
    short_record.abi_version = OA_CONTROL_ABI_V1;
    if (oa_controller_create(NULL, &short_record, &controller) != OA_EINVAL) {
        return 1;
    }
    if (oa_controller_get_arm_challenge(NULL, NULL) != OA_EINVAL) {
        return 2;
    }
    return 0;
}
