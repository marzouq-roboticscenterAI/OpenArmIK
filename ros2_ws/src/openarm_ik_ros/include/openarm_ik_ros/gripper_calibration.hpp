// SPDX-License-Identifier: Apache-2.0
#pragma once

extern "C" {
#include "openarm_can.h"
}

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace openarm_ik_ros::real
{

/// Persistent motor-to-URDF mapping for the two OpenArm v1.0 grippers.
///
/// Side 0 is robot-left and side 1 is robot-right. The public C CAN ABI owns
/// the interpolation; this class owns only machine-specific persistence.
class GripperCalibration
{
public:
  static constexpr std::size_t kSideCount = 2U;
  static constexpr double kMaximumOpeningM = 0.044;
  static constexpr double kOfficialMotorTravelRad = -1.0472;

  bool ready() const noexcept;
  bool capture_closed(const std::array<double, kSideCount> & raw_motor_rad);
  bool save(const std::string & path, std::string & error) const;
  bool load(const std::string & path, std::string & detail);

  const oa_can_gripper_calibration & side(std::size_t side) const;
  double opening_m(std::size_t side, double raw_motor_rad) const;
  double velocity_m_s(std::size_t side, double raw_motor_velocity_rad_s) const;

private:
  std::array<oa_can_gripper_calibration, kSideCount> calibration_{};
  bool ready_{false};
};

}  // namespace openarm_ik_ros::real
