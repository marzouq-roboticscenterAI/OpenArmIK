/* SPDX-License-Identifier: Apache-2.0 */
#ifndef OPENARM_CONTROL_KINEMATICS_HPP
#define OPENARM_CONTROL_KINEMATICS_HPP

#include <array>
#include <cstdint>

namespace openarm::control {

using JointVector = std::array<double, 7>;

struct KinematicResult {
    std::array<std::array<double, 3>, 7> joint_xyz{};
    std::array<std::array<double, 3>, 7> joint_axis{};
    std::array<double, 16> tcp_transform{};
    std::array<double, 3> tcp_xyz{};
};

struct IkResult {
    JointVector q{};
    std::array<double, 3> achieved{};
    double residual{};
    bool collision_checked{};
};

bool forward(std::uint32_t side, const JointVector &q, KinematicResult &out) noexcept;
bool inverse(std::uint32_t side, const std::array<double, 3> &target,
             const JointVector &seed, IkResult &out) noexcept;
bool model_limit(std::uint32_t side, std::size_t joint, double &lower,
                 double &upper) noexcept;

}  // namespace openarm::control
#endif
