// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/display_calibration.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace openarm_ik_ros::real
{
namespace
{
constexpr const char * kVersion = "OPENARM_REAL_DISPLAY_CALIBRATION_V2";

bool finite_joint(const JointDisplayCalibration & value)
{
  return (value.direction == -1 || value.direction == 1) &&
         std::isfinite(value.reference_rad) && std::isfinite(value.offset_rad);
}
}  // namespace

bool DisplayCalibration::valid_index(const std::size_t side, const std::size_t joint)
{
  return side < kDisplaySideCount && joint < kDisplayJointCount;
}

double DisplayCalibration::position(
  const std::size_t side, const std::size_t joint, const double raw_rad) const
{
  if (!valid_index(side, joint) || !std::isfinite(raw_rad)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto & calibration = joints_[side][joint];
  return static_cast<double>(calibration.direction) *
         (raw_rad - calibration.reference_rad) + calibration.offset_rad;
}

double DisplayCalibration::raw_position(
  const std::size_t side, const std::size_t joint, const double model_rad) const
{
  if (!valid_index(side, joint) || !std::isfinite(model_rad)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto & calibration = joints_[side][joint];
  return calibration.reference_rad + static_cast<double>(calibration.direction) *
         (model_rad - calibration.offset_rad);
}

double DisplayCalibration::signed_value(
  const std::size_t side, const std::size_t joint, const double raw_value) const
{
  if (!valid_index(side, joint) || !std::isfinite(raw_value)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return static_cast<double>(joints_[side][joint].direction) * raw_value;
}

const JointDisplayCalibration & DisplayCalibration::joint(
  const std::size_t side, const std::size_t joint) const
{
  return joints_.at(side).at(joint);
}

bool DisplayCalibration::flip_direction(const std::size_t side, const std::size_t joint)
{
  if (!valid_index(side, joint)) {
    return false;
  }
  joints_[side][joint].direction = -joints_[side][joint].direction;
  return true;
}

bool DisplayCalibration::add_offset(
  const std::size_t side, const std::size_t joint, const double delta_rad)
{
  if (!valid_index(side, joint) || !std::isfinite(delta_rad)) {
    return false;
  }
  const double adjusted = joints_[side][joint].offset_rad + delta_rad;
  if (!std::isfinite(adjusted)) {
    return false;
  }
  joints_[side][joint].offset_rad = adjusted;
  return true;
}

bool DisplayCalibration::set_current_position(
  const std::size_t side, const std::size_t joint, const double raw_rad,
  const double target_rad)
{
  if (!valid_index(side, joint) || !std::isfinite(raw_rad) || !std::isfinite(target_rad)) {
    return false;
  }
  auto & calibration = joints_[side][joint];
  calibration.offset_rad = target_rad -
    static_cast<double>(calibration.direction) * (raw_rad - calibration.reference_rad);
  return std::isfinite(calibration.offset_rad);
}

bool DisplayCalibration::capture_current_as_zero(
  const std::size_t side, const RawArm & raw_rad)
{
  if (side >= kDisplaySideCount) {
    return false;
  }
  for (const double value : raw_rad) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (std::size_t joint_index = 0; joint_index < kDisplayJointCount; ++joint_index) {
    joints_[side][joint_index].reference_rad = raw_rad[joint_index];
    joints_[side][joint_index].offset_rad = 0.0;
  }
  return true;
}

void DisplayCalibration::clear_references_and_offsets()
{
  for (auto & side : joints_) {
    for (auto & joint_value : side) {
      joint_value.reference_rad = 0.0;
      joint_value.offset_rad = 0.0;
    }
  }
}

bool DisplayCalibration::save(const std::string & path, std::string & error) const
{
  const std::string temporary_path = path + ".tmp";
  std::ofstream file(temporary_path, std::ios::trunc);
  if (!file) {
    error = "could not open temporary calibration file " + temporary_path;
    return false;
  }
  file << kVersion << '\n' << std::setprecision(17);
  for (std::size_t side = 0; side < kDisplaySideCount; ++side) {
    for (std::size_t joint_index = 0; joint_index < kDisplayJointCount; ++joint_index) {
      const auto & value = joints_[side][joint_index];
      if (!finite_joint(value)) {
        error = "refusing to save a non-finite display calibration";
        file.close();
        (void)std::remove(temporary_path.c_str());
        return false;
      }
      file << side << ' ' << joint_index << ' ' << value.direction << ' '
           << value.reference_rad << ' ' << value.offset_rad << '\n';
    }
  }
  file.close();
  if (!file) {
    error = "failed while writing calibration file " + temporary_path;
    (void)std::remove(temporary_path.c_str());
    return false;
  }
  if (std::rename(temporary_path.c_str(), path.c_str()) != 0) {
    error = "could not replace calibration file " + path;
    (void)std::remove(temporary_path.c_str());
    return false;
  }
  error.clear();
  return true;
}

bool DisplayCalibration::load(const std::string & path, std::string & detail)
{
  std::ifstream file(path);
  if (!file) {
    detail = "no saved display calibration";
    return false;
  }

  std::string first_line;
  std::getline(file, first_line);
  DisplayCalibration parsed;
  if (first_line == kVersion) {
    std::array<std::array<bool, kDisplayJointCount>, kDisplaySideCount> seen{};
    for (std::size_t record = 0; record < kDisplaySideCount * kDisplayJointCount; ++record) {
      std::size_t side = 0;
      std::size_t joint_index = 0;
      JointDisplayCalibration value;
      if (!(file >> side >> joint_index >> value.direction >> value.reference_rad >>
          value.offset_rad) || !valid_index(side, joint_index) || !finite_joint(value) ||
        seen[side][joint_index])
      {
        detail = "saved V2 display calibration is malformed; defaults retained";
        return false;
      }
      seen[side][joint_index] = true;
      parsed.joints_[side][joint_index] = value;
    }
    joints_ = parsed.joints_;
    detail = "loaded V2 per-joint display calibration";
    return true;
  }

  // Legacy format: two rows of seven raw encoder zeroes followed by one
  // direction per arm. Convert it exactly to the new per-joint representation.
  file.clear();
  file.seekg(0);
  std::array<std::array<double, kDisplayJointCount>, kDisplaySideCount> references{};
  for (auto & side : references) {
    for (double & value : side) {
      if (!(file >> value) || !std::isfinite(value)) {
        detail = "saved legacy display calibration is malformed; defaults retained";
        return false;
      }
    }
  }
  std::array<double, kDisplaySideCount> arm_directions{1.0, 1.0};
  (void)(file >> arm_directions[0] >> arm_directions[1]);
  for (std::size_t side = 0; side < kDisplaySideCount; ++side) {
    const int direction = arm_directions[side] < 0.0 ? -1 : 1;
    for (std::size_t joint_index = 0; joint_index < kDisplayJointCount; ++joint_index) {
      parsed.joints_[side][joint_index].direction = direction;
      parsed.joints_[side][joint_index].reference_rad = references[side][joint_index];
    }
  }
  joints_ = parsed.joints_;
  detail = "migrated legacy arm-wide calibration to V2 per-joint calibration";
  return true;
}
}  // namespace openarm_ik_ros::real
