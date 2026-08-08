// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/gripper_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace openarm_ik_ros::real
{
namespace
{
constexpr const char * kVersion = "OPENARM_REAL_GRIPPER_CALIBRATION_V1";

oa_can_gripper_calibration make_side(const double closed_motor_rad)
{
  oa_can_gripper_calibration value{};
  value.struct_size = static_cast<std::uint32_t>(sizeof(value));
  value.abi_version = OA_CAN_ABI_VERSION;
  value.closed_motor_position_rad = closed_motor_rad;
  value.open_motor_position_rad = closed_motor_rad +
    GripperCalibration::kOfficialMotorTravelRad;
  value.maximum_opening_m = GripperCalibration::kMaximumOpeningM;
  return value;
}

bool valid(const oa_can_gripper_calibration & value)
{
  double motor = 0.0;
  return value.struct_size == sizeof(value) && value.abi_version == OA_CAN_ABI_VERSION &&
         oa_can_gripper_motor_position(&value, 0.0, &motor) == OA_CAN_OK;
}
}  // namespace

bool GripperCalibration::ready() const noexcept
{
  return ready_;
}

bool GripperCalibration::capture_closed(
  const std::array<double, kSideCount> & raw_motor_rad)
{
  for (const double value : raw_motor_rad) {
    if (!std::isfinite(value)) {return false;}
  }
  for (std::size_t side_index = 0U; side_index < kSideCount; ++side_index) {
    calibration_[side_index] = make_side(raw_motor_rad[side_index]);
  }
  ready_ = true;
  return true;
}

bool GripperCalibration::save(const std::string & path, std::string & error) const
{
  if (!ready_ || !valid(calibration_[0]) || !valid(calibration_[1])) {
    error = "refusing to save an incomplete gripper calibration";
    return false;
  }
  const std::string temporary_path = path + ".tmp";
  std::ofstream file(temporary_path, std::ios::trunc);
  if (!file) {
    error = "could not open temporary gripper calibration " + temporary_path;
    return false;
  }
  file << kVersion << '\n' << std::setprecision(17);
  for (std::size_t side_index = 0U; side_index < kSideCount; ++side_index) {
    const auto & value = calibration_[side_index];
    file << side_index << ' ' << value.closed_motor_position_rad << ' '
         << value.open_motor_position_rad << ' ' << value.maximum_opening_m << '\n';
  }
  file.close();
  if (!file) {
    error = "failed while writing gripper calibration " + temporary_path;
    (void)std::remove(temporary_path.c_str());
    return false;
  }
  if (std::rename(temporary_path.c_str(), path.c_str()) != 0) {
    error = "could not replace gripper calibration " + path;
    (void)std::remove(temporary_path.c_str());
    return false;
  }
  error.clear();
  return true;
}

bool GripperCalibration::load(const std::string & path, std::string & detail)
{
  std::ifstream file(path);
  if (!file) {
    detail = "no saved gripper calibration";
    return false;
  }
  std::string version;
  std::getline(file, version);
  if (version != kVersion) {
    detail = "saved gripper calibration has an unsupported version";
    return false;
  }
  std::array<oa_can_gripper_calibration, kSideCount> parsed{};
  std::array<bool, kSideCount> seen{};
  for (std::size_t record = 0U; record < kSideCount; ++record) {
    std::size_t side_index = 0U;
    double closed = 0.0;
    double open = 0.0;
    double maximum = 0.0;
    if (!(file >> side_index >> closed >> open >> maximum) || side_index >= kSideCount ||
      seen[side_index])
    {
      detail = "saved gripper calibration is malformed";
      return false;
    }
    parsed[side_index] = make_side(closed);
    parsed[side_index].open_motor_position_rad = open;
    parsed[side_index].maximum_opening_m = maximum;
    if (!valid(parsed[side_index])) {
      detail = "saved gripper calibration is non-finite or degenerate";
      return false;
    }
    seen[side_index] = true;
  }
  calibration_ = parsed;
  ready_ = true;
  detail = "loaded V1 double-precision gripper calibration";
  return true;
}

const oa_can_gripper_calibration & GripperCalibration::side(
  const std::size_t side_index) const
{
  if (!ready_ || side_index >= kSideCount) {
    throw std::out_of_range("gripper calibration is unavailable");
  }
  return calibration_[side_index];
}

double GripperCalibration::opening_m(
  const std::size_t side_index, const double raw_motor_rad) const
{
  if (!ready_ || side_index >= kSideCount || !std::isfinite(raw_motor_rad)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double opening = 0.0;
  if (oa_can_gripper_opening(
      &calibration_[side_index], raw_motor_rad, &opening) != OA_CAN_OK)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  // Encoder quantization can place an endpoint a fraction outside its nominal
  // interval. RViz and command-state consumers see the physical interval while
  // the raw value remains available to the watchdog and torque controller.
  return std::clamp(opening, 0.0, calibration_[side_index].maximum_opening_m);
}

double GripperCalibration::velocity_m_s(
  const std::size_t side_index, const double raw_motor_velocity_rad_s) const
{
  if (!ready_ || side_index >= kSideCount || !std::isfinite(raw_motor_velocity_rad_s)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto & value = calibration_[side_index];
  return raw_motor_velocity_rad_s * value.maximum_opening_m /
         (value.open_motor_position_rad - value.closed_motor_position_rad);
}

}  // namespace openarm_ik_ros::real
