/* SPDX-License-Identifier: Apache-2.0 */
#ifdef OPENARM_DISABLE_LEGACY_GENERIC_STATUS
#undef OPENARM_DISABLE_LEGACY_GENERIC_STATUS
#endif
#include "openarm_model.h"

_Static_assert(sizeof(oa_status) == sizeof(int32_t),
               "legacy model status representation changed");
_Static_assert((oa_status)-1 < 0, "legacy model status signedness changed");
_Static_assert(OA_OK == OA_MODEL_OK && OA_EINVAL == OA_MODEL_EINVAL &&
                   OA_EBUDGET == OA_MODEL_EBUDGET,
               "legacy model status values changed");
