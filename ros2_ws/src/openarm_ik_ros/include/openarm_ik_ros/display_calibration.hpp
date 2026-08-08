// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <string>

namespace openarm_ik_ros::real
{
constexpr std::size_t kDisplaySideCount = 2U;
constexpr std::size_t kDisplayJointCount = 7U;

struct JointDisplayCalibration
{
  int direction{1};
  double reference_rad{0.0};
  double offset_rad{0.0};
};

/// Persistent, double-precision affine mapping from encoder to URDF space.
///
/// q_urdf = direction * (q_encoder - reference) + offset
///
/// Side 0 is robot-left and side 1 is robot-right throughout this class.
class DisplayCalibration
{
public:
  using RawArm = std::array<double, kDisplayJointCount>;

  double position(std::size_t side, std::size_t joint, double raw_rad) const;
  /// Exact inverse of position(). Coordinate calculations remain binary64.
  double raw_position(std::size_t side, std::size_t joint, double model_rad) const;
  double signed_value(std::size_t side, std::size_t joint, double raw_value) const;

  const JointDisplayCalibration & joint(std::size_t side, std::size_t joint) const;
  bool flip_direction(std::size_t side, std::size_t joint);
  bool add_offset(std::size_t side, std::size_t joint, double delta_rad);
  bool set_current_position(
    std::size_t side, std::size_t joint, double raw_rad, double target_rad);
  bool capture_current_as_zero(std::size_t side, const RawArm & raw_rad);
  void clear_references_and_offsets();

  bool save(const std::string & path, std::string & error) const;
  bool load(const std::string & path, std::string & detail);

private:
  static bool valid_index(std::size_t side, std::size_t joint);
  std::array<std::array<JointDisplayCalibration, kDisplayJointCount>, kDisplaySideCount>
  joints_{};
};
}  // namespace openarm_ik_ros::real
