// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/display_calibration.hpp"
#include "openarm_ik_ros/real_control_session.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <string>
#include <utility>

namespace
{
using openarm_ik_ros::real::DisplayCalibration;
using openarm_ik_ros::real::RealControlConfig;
using openarm_ik_ros::real::RealControlSession;

class RealControlSessionPassiveTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_EQ(oa_runtime_estop_clear(), OA_RUNTIME_OK);
    DisplayCalibration calibration;
    DisplayCalibration::RawArm zero{};
    ASSERT_TRUE(calibration.capture_current_as_zero(0U, zero));
    ASSERT_TRUE(calibration.capture_current_as_zero(1U, zero));
    ASSERT_TRUE(calibration.save(path_, detail_)) << detail_;
  }

  void TearDown() override
  {
    ASSERT_EQ(oa_runtime_estop_clear(), OA_RUNTIME_OK);
    (void)std::remove(path_.c_str());
  }

  const std::string path_{"/tmp/openarm_real_control_session_test.calibration"};
  std::string detail_;
};

TEST_F(RealControlSessionPassiveTest, ConstructionAndStopArePassive)
{
  RealControlConfig config;
  config.calibration_path = path_;
  config.interface_for_side = {"definitely-not-can-left", "definitely-not-can-right"};
  RealControlSession session(std::move(config), {}, {});

  EXPECT_FALSE(session.connected());
  EXPECT_FALSE(session.armed());
  EXPECT_FALSE(session.busy());
  EXPECT_FALSE(session.estop_asserted());
  EXPECT_NE(session.status_json().find("\"connected\":false"), std::string::npos);
  EXPECT_NE(session.status_json().find("\"busy\":false"), std::string::npos);

  std::string reason;
  EXPECT_TRUE(session.stop_motion(reason));
  EXPECT_EQ(reason, "no physical motion is active");
  EXPECT_FALSE(session.reserve("test", reason));
  EXPECT_EQ(reason, "physical_motors_not_connected");
  session.close();
}

TEST_F(RealControlSessionPassiveTest, EstopSupremacyDoesNotRequireAConnection)
{
  RealControlConfig config;
  config.calibration_path = path_;
  RealControlSession session(std::move(config), {}, {});

  std::string detail;
  EXPECT_TRUE(session.emergency_stop(detail));
  EXPECT_TRUE(session.estop_asserted());
  EXPECT_FALSE(session.connected());
  EXPECT_FALSE(session.armed());
  EXPECT_NE(detail.find("no open CAN sockets"), std::string::npos);
  EXPECT_EQ(detail.find("confirmed disabled"), std::string::npos);
  EXPECT_NE(session.status_json().find("\"estop\":true"), std::string::npos);

  EXPECT_TRUE(session.clear_emergency_stop(detail));
  EXPECT_FALSE(session.estop_asserted());
  EXPECT_FALSE(session.connected());
  EXPECT_FALSE(session.armed());
  EXPECT_NE(detail.find("explicit Connect"), std::string::npos);
  session.close();
}

TEST_F(RealControlSessionPassiveTest, LeftOnlyModeExposesOnlyTheLeftBus)
{
  RealControlConfig config;
  config.calibration_path = path_;
  config.active_side_mask = 1U;
  config.interface_for_side = {"left-test-bus", "right-must-remain-closed"};
  RealControlSession session(std::move(config), {}, {});

  const std::string status = session.status_json();
  EXPECT_NE(status.find("\"active_side_mask\":1"), std::string::npos);
  EXPECT_NE(status.find("left-test-bus"), std::string::npos);
  EXPECT_EQ(status.find("right-must-remain-closed"), std::string::npos);
  session.close();
}

TEST_F(RealControlSessionPassiveTest, InvalidActiveSideMaskIsRejected)
{
  RealControlConfig config;
  config.calibration_path = path_;
  config.active_side_mask = 0U;
  EXPECT_THROW(RealControlSession(std::move(config), {}, {}), std::invalid_argument);
}
}  // namespace
