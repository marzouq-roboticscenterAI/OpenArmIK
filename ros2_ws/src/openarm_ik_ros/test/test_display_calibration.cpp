// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/display_calibration.hpp"
#include "openarm_ik_ros/gripper_calibration.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

namespace
{
using openarm_ik_ros::real::DisplayCalibration;
using openarm_ik_ros::real::GripperCalibration;

constexpr double kTolerance = 1.0e-12;

TEST(DisplayCalibration, CapturedReferenceAndDirectionArePerJoint)
{
  DisplayCalibration calibration;
  const DisplayCalibration::RawArm relaxed{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
  ASSERT_TRUE(calibration.capture_current_as_zero(0U, relaxed));

  EXPECT_NEAR(calibration.position(0U, 2U, 3.25), 0.25, kTolerance);
  EXPECT_NEAR(calibration.position(0U, 3U, 4.25), 0.25, kTolerance);
  ASSERT_TRUE(calibration.flip_direction(0U, 2U));
  EXPECT_NEAR(calibration.position(0U, 2U, 3.25), -0.25, kTolerance);
  EXPECT_NEAR(calibration.position(0U, 3U, 4.25), 0.25, kTolerance);
  EXPECT_EQ(calibration.joint(0U, 2U).direction, -1);
}

TEST(DisplayCalibration, OffsetAndSetCurrentUseDoublePrecision)
{
  DisplayCalibration calibration;
  const DisplayCalibration::RawArm relaxed{};
  ASSERT_TRUE(calibration.capture_current_as_zero(1U, relaxed));
  ASSERT_TRUE(calibration.add_offset(1U, 4U, 0.123456789012345));
  EXPECT_NEAR(calibration.position(1U, 4U, 0.25), 0.373456789012345, kTolerance);

  ASSERT_TRUE(calibration.set_current_position(1U, 4U, 0.25, -0.375));
  EXPECT_NEAR(calibration.position(1U, 4U, 0.25), -0.375, kTolerance);
}

TEST(DisplayCalibration, RawPositionIsExactBinary64Inverse)
{
  DisplayCalibration calibration;
  DisplayCalibration::RawArm relaxed{};
  relaxed[3] = -0.92259861142900768;
  ASSERT_TRUE(calibration.capture_current_as_zero(1U, relaxed));
  ASSERT_TRUE(calibration.flip_direction(1U, 3U));
  ASSERT_TRUE(calibration.add_offset(1U, 3U, 0.0123456789012345));
  const double model = 0.7654321098765432;
  const double raw = calibration.raw_position(1U, 3U, model);
  EXPECT_DOUBLE_EQ(calibration.position(1U, 3U, raw), model);
}

TEST(DisplayCalibration, V2PersistenceRoundTripsWithoutFloatNarrowing)
{
  const std::string path = "/tmp/openarm_display_calibration_test_v2.txt";
  DisplayCalibration original;
  DisplayCalibration::RawArm raw{};
  raw[6] = 1.2345678901234567;
  ASSERT_TRUE(original.capture_current_as_zero(0U, raw));
  ASSERT_TRUE(original.flip_direction(0U, 6U));
  ASSERT_TRUE(original.add_offset(0U, 6U, -0.9876543210987654));
  std::string detail;
  ASSERT_TRUE(original.save(path, detail)) << detail;

  DisplayCalibration loaded;
  ASSERT_TRUE(loaded.load(path, detail)) << detail;
  EXPECT_EQ(loaded.joint(0U, 6U).direction, -1);
  EXPECT_DOUBLE_EQ(loaded.joint(0U, 6U).reference_rad, raw[6]);
  EXPECT_DOUBLE_EQ(loaded.joint(0U, 6U).offset_rad, -0.9876543210987654);
  (void)std::remove(path.c_str());
}

TEST(DisplayCalibration, LegacyArmWideFileMigratesExactly)
{
  const std::string path = "/tmp/openarm_display_calibration_test_legacy.txt";
  {
    std::ofstream legacy(path);
    ASSERT_TRUE(legacy.good());
    legacy << "1 2 3 4 5 6 7\n8 9 10 11 12 13 14\n-1 1\n";
  }
  DisplayCalibration calibration;
  std::string detail;
  ASSERT_TRUE(calibration.load(path, detail)) << detail;
  EXPECT_NE(detail.find("migrated"), std::string::npos);
  EXPECT_EQ(calibration.joint(0U, 0U).direction, -1);
  EXPECT_EQ(calibration.joint(1U, 0U).direction, 1);
  EXPECT_DOUBLE_EQ(calibration.joint(0U, 6U).reference_rad, 7.0);
  EXPECT_DOUBLE_EQ(calibration.joint(1U, 6U).reference_rad, 14.0);
  EXPECT_DOUBLE_EQ(calibration.position(0U, 0U, 1.25), -0.25);
  (void)std::remove(path.c_str());
}

TEST(GripperCalibration, CapturesClosedAndMapsOfficialTravelInBinary64)
{
  GripperCalibration calibration;
  ASSERT_TRUE(calibration.capture_closed({0.3571234567890123, -0.4123456789012345}));
  ASSERT_TRUE(calibration.ready());
  EXPECT_DOUBLE_EQ(calibration.side(0U).closed_motor_position_rad, 0.3571234567890123);
  EXPECT_DOUBLE_EQ(
    calibration.side(0U).open_motor_position_rad,
    0.3571234567890123 + GripperCalibration::kOfficialMotorTravelRad);
  EXPECT_DOUBLE_EQ(calibration.opening_m(0U, calibration.side(0U).closed_motor_position_rad), 0.0);
  EXPECT_DOUBLE_EQ(
    calibration.opening_m(1U, calibration.side(1U).open_motor_position_rad),
    GripperCalibration::kMaximumOpeningM);
  EXPECT_DOUBLE_EQ(
    calibration.velocity_m_s(0U, GripperCalibration::kOfficialMotorTravelRad),
    GripperCalibration::kMaximumOpeningM);
  EXPECT_DOUBLE_EQ(
    calibration.velocity_m_s(1U, -0.5 * GripperCalibration::kOfficialMotorTravelRad),
    -0.5 * GripperCalibration::kMaximumOpeningM);
  EXPECT_TRUE(std::isnan(calibration.velocity_m_s(2U, 1.0)));
  EXPECT_TRUE(std::isnan(calibration.velocity_m_s(0U, NAN)));
}

TEST(GripperCalibration, PersistenceRoundTripsWithoutFloatNarrowing)
{
  const std::string path = "/tmp/openarm_gripper_calibration_test.txt";
  GripperCalibration original;
  ASSERT_TRUE(original.capture_closed({0.12345678901234567, -0.7654321098765432}));
  std::string detail;
  ASSERT_TRUE(original.save(path, detail)) << detail;
  GripperCalibration loaded;
  ASSERT_TRUE(loaded.load(path, detail)) << detail;
  EXPECT_DOUBLE_EQ(
    loaded.side(0U).closed_motor_position_rad,
    original.side(0U).closed_motor_position_rad);
  EXPECT_DOUBLE_EQ(
    loaded.side(1U).open_motor_position_rad,
    original.side(1U).open_motor_position_rad);
  EXPECT_DOUBLE_EQ(loaded.side(1U).maximum_opening_m, GripperCalibration::kMaximumOpeningM);
  (void)std::remove(path.c_str());
}
}  // namespace
