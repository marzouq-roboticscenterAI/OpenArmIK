// SPDX-License-Identifier: Apache-2.0
#ifndef OPENARM_IK_ROS__MOTION_PROFILE_HPP_
#define OPENARM_IK_ROS__MOTION_PROFILE_HPP_

#include <cmath>

namespace openarm_ik_ros
{

inline constexpr double kLegacyMotionLimitScale = 0.5;
inline constexpr double kMinimumMotionLimitScale = 0.5;
inline constexpr double kMaximumMotionLimitScale = 1.0;

inline bool valid_motion_limit_scale(const double value) noexcept
{
  return std::isfinite(value) && value >= kMinimumMotionLimitScale &&
         value <= kMaximumMotionLimitScale;
}

}  // namespace openarm_ik_ros

#endif  // OPENARM_IK_ROS__MOTION_PROFILE_HPP_
