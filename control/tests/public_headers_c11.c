/* SPDX-License-Identifier: Apache-2.0 */
#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1
#include "openarm_can.h"
#include "openarm_model.h"
#include "openarm_transport.h"
#include "openarm_commission.h"
#include "openarm_control.h"

#if defined(OA_OK) || defined(OA_EINVAL)
#error "generic status aliases must be disabled"
#endif

_Static_assert(sizeof(oa_model_status) == sizeof(int32_t),
               "model status representation changed");
_Static_assert(sizeof(oa_control_status) == sizeof(uint32_t),
               "control status representation changed");
_Static_assert((oa_model_status)-1 < 0, "model status signedness changed");
_Static_assert((oa_control_status)-1 > 0, "control status signedness changed");
_Static_assert(offsetof(oa_ik_diagnostics, status) == 8,
               "model diagnostics layout changed");
_Static_assert(offsetof(oa_event, cause) == 40,
               "control event layout changed");
_Static_assert(OA_MODEL_OK == 0 && OA_MODEL_EBUDGET == 7,
               "model status values changed");
_Static_assert(OA_CONTROL_OK == 0 && OA_CONTROL_EUNSUPPORTED == 16,
               "control status values changed");

oa_model_status oa_public_headers_c11_model_status(void) {
    return OA_MODEL_EINVAL;
}

oa_control_status oa_public_headers_c11_control_status(void) {
    return OA_CONTROL_ESTATE;
}
