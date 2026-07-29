/* SPDX-License-Identifier: Apache-2.0 */
#include "kinematics.hpp"

#include "openarm_model.h"

#include <algorithm>
#include <cmath>

namespace openarm::control {
namespace {
const oa_model *model_for(const std::uint32_t side) noexcept {
    return side == 0U ? oa_model_left_v10_bimanual() : oa_model_right_v10_bimanual();
}
}  // namespace

bool forward(const std::uint32_t side, const JointVector &q,
             KinematicResult &out) noexcept {
    oa_fk_result result{};
    if (oa_fk(model_for(side), q.data(), &result) != OA_MODEL_OK) {
        return false;
    }
    for (std::size_t joint = 0; joint < q.size(); ++joint) {
        for (std::size_t axis = 0; axis < 3U; ++axis) {
            out.joint_xyz[joint][axis] = result.joint_pre[joint].m[axis * 4U + 3U];
            out.joint_axis[joint][axis] = result.joint_axis_body[joint][axis];
        }
    }
    std::copy(std::begin(result.hand_tcp.m), std::end(result.hand_tcp.m),
              out.tcp_transform.begin());
    for (std::size_t axis = 0; axis < 3U; ++axis) {
        out.tcp_xyz[axis] = result.hand_tcp.m[axis * 4U + 3U];
    }
    return true;
}

bool inverse(const std::uint32_t side, const std::array<double, 3> &target,
             const JointVector &seed, IkResult &out) noexcept {
    oa_ik_options options{};
    options.abi_version = OA_MODEL_ABI_VERSION;
    options.struct_size = sizeof(options);
    std::copy(seed.begin(), seed.end(), options.seed);
    std::copy(seed.begin(), seed.end(), options.posture);
    std::fill(std::begin(options.posture_weight), std::end(options.posture_weight), 1.0);
    options.position_tolerance_m = 1.0e-6;
    options.max_joint_step_rad = 0.12;
    options.damping_min = 1.0e-5;
    options.damping_max = 0.1;
    options.limit_margin_rad = 1.0e-5;
    options.max_iterations = 500U;

    oa_ik_diagnostics diagnostics{};
    const oa_model_status status = oa_ik_position_v2(
        model_for(side), target.data(), &options, OA_IK_DIAGNOSTICS_VERSION,
        sizeof(diagnostics), &diagnostics);
    if (status != OA_MODEL_OK || !std::isfinite(diagnostics.position_error_m)) {
        return false;
    }
    std::copy(std::begin(diagnostics.q), std::end(diagnostics.q), out.q.begin());
    std::copy(std::begin(diagnostics.achieved_position_m),
              std::end(diagnostics.achieved_position_m), out.achieved.begin());
    out.residual = diagnostics.position_error_m;
    out.min_singular_value = diagnostics.min_singular_value;
    out.collision_checked = diagnostics.collision_checked != 0U;
    return true;
}

bool model_limit(const std::uint32_t side, const std::size_t joint, double &lower,
                 double &upper) noexcept {
    return oa_model_limits(model_for(side), joint, &lower, &upper) == OA_MODEL_OK;
}

}  // namespace openarm::control
