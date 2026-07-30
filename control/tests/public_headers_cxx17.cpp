/* SPDX-License-Identifier: Apache-2.0 */
#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1
#include "openarm_control.h"
#include "openarm_commission.h"
#include "openarm_transport.h"
#include "openarm_model.h"
#include "openarm_can.h"

#include <cstdint>
#include <type_traits>

#if defined(OA_OK) || defined(OA_EINVAL)
#error "generic status aliases must be disabled"
#endif

static_assert(std::is_same_v<oa_model_status, std::int32_t>);
static_assert(std::is_same_v<oa_control_status, std::uint32_t>);
static_assert(std::is_same_v<decltype(oa_vec3d::x), double>);
static_assert(sizeof(oa_vec3d) == 3 * sizeof(double));
static_assert(offsetof(oa_ik_diagnostics, status) == 8);
static_assert(offsetof(oa_event, cause) == 40);
static_assert(OA_MODEL_OK == 0 && OA_MODEL_EBUDGET == 7);
static_assert(OA_CONTROL_OK == 0 && OA_CONTROL_EUNSUPPORTED == 16);

oa_model_status oa_public_headers_cxx17_model_status() {
    return OA_MODEL_EINVAL;
}

oa_control_status oa_public_headers_cxx17_control_status() {
    return OA_CONTROL_ESTATE;
}
