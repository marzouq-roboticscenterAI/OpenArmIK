/* SPDX-License-Identifier: Apache-2.0 */
#ifdef OPENARM_DISABLE_LEGACY_GENERIC_STATUS
#undef OPENARM_DISABLE_LEGACY_GENERIC_STATUS
#endif
#include "openarm_control.h"

_Static_assert(sizeof(oa_status) == sizeof(uint32_t),
               "legacy control status representation changed");
_Static_assert((oa_status)-1 > 0, "legacy control status signedness changed");
_Static_assert(OA_OK == OA_CONTROL_OK && OA_EINVAL == OA_CONTROL_EINVAL &&
                   OA_EUNSUPPORTED == OA_CONTROL_EUNSUPPORTED,
               "legacy control status values changed");
