// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/paired_transaction.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace
{
constexpr std::int64_t kNow = 2000000000LL;
constexpr std::int64_t kExpiry = 1000000000LL;

std::array<double, 3> fk_position(const oa_model * model, const std::array<double, OA_DOF> & q)
{
  oa_fk_result fk{};
  EXPECT_EQ(oa_fk(model, q.data(), &fk), OA_OK);
  return {fk.hand_tcp.m[3], fk.hand_tcp.m[7], fk.hand_tcp.m[11]};
}

openarm_ik_ros::PairedTarget valid_request()
{
  std::array<double, OA_DOF> left_q = {0.15, -0.10, 0.10, 0.30, -0.10, 0.10, -0.10};
  std::array<double, OA_DOF> right_q = {-0.15, 0.10, -0.10, 0.30, 0.10, -0.10, 0.10};
  openarm_ik_ros::PairedTarget request;
  request.pose_count = 2U;
  request.frame_id = "world";
  request.stamp_nanoseconds = kNow;
  request.left = fk_position(oa_model_left_v10_bimanual(), left_q);
  request.right = fk_position(oa_model_right_v10_bimanual(), right_q);
  return request;
}

TEST(PairedTransaction, PublishesExactGeneratedJointOrder)
{
  openarm_ik_ros::PairedTransactionProcessor processor(kExpiry);
  const auto names = processor.joint_names();
  ASSERT_EQ(names.size(), 16U);
  for (std::size_t index = 0; index < OA_DOF; ++index) {
    EXPECT_EQ(names[index], "openarm_left_joint" + std::to_string(index + 1U));
    EXPECT_EQ(names[index + OA_DOF], "openarm_right_joint" + std::to_string(index + 1U));
  }
  EXPECT_EQ(names[14], "openarm_left_finger_joint1");
  EXPECT_EQ(names[15], "openarm_right_finger_joint1");
}

TEST(PairedTransaction, InitialPoseIsLegalAndCoherent)
{
  openarm_ik_ros::PairedTransactionProcessor processor(kExpiry);
  for (const oa_model * model : {oa_model_left_v10_bimanual(), oa_model_right_v10_bimanual()}) {
    for (std::size_t index = 0; index < OA_DOF; ++index) {
      double lower{};
      double upper{};
      ASSERT_EQ(oa_model_limits(model, index, &lower, &upper), OA_OK);
      const auto & q = model == oa_model_left_v10_bimanual() ? processor.left_q() : processor.right_q();
      EXPECT_GE(q[index], lower);
      EXPECT_LE(q[index], upper);
    }
  }
}

TEST(PairedTransaction, CommitsBothArmsTogetherAndMeetsFkTolerance)
{
  openarm_ik_ros::PairedTransactionProcessor processor(kExpiry);
  const auto request = valid_request();
  const auto result = processor.process(request, kNow);
  ASSERT_TRUE(result.committed) << result.reason;
  EXPECT_EQ(result.left.status, OA_OK);
  EXPECT_EQ(result.right.status, OA_OK);
  EXPECT_EQ(result.left.collision_checked, 0U);
  EXPECT_EQ(result.right.collision_checked, 0U);
  EXPECT_LE(result.left.position_error_m, 1e-4);
  EXPECT_LE(result.right.position_error_m, 1e-4);
  const std::array<double, OA_DOF> zero{};
  EXPECT_NE(processor.left_q(), zero);
  EXPECT_NE(processor.right_q(), zero);
  const auto left_achieved = fk_position(oa_model_left_v10_bimanual(), processor.left_q());
  const auto right_achieved = fk_position(oa_model_right_v10_bimanual(), processor.right_q());
  for (std::size_t axis = 0; axis < 3U; ++axis) {
    EXPECT_NEAR(left_achieved[axis], request.left[axis], 1e-4);
    EXPECT_NEAR(right_achieved[axis], request.right[axis], 1e-4);
  }
}

TEST(PairedTransaction, RejectsInvalidRequestsWithoutChangingEitherArm)
{
  openarm_ik_ros::PairedTransactionProcessor processor(kExpiry);
  const auto first = processor.process(valid_request(), kNow);
  ASSERT_TRUE(first.committed);
  const auto left_before = processor.left_q();
  const auto right_before = processor.right_q();
  auto request = valid_request();
  request.pose_count = 1U;
  EXPECT_FALSE(processor.process(request, kNow).committed);
  request = valid_request();
  request.frame_id = "base_link";
  EXPECT_FALSE(processor.process(request, kNow).committed);
  request = valid_request();
  request.stamp_nanoseconds = kNow - kExpiry - 1LL;
  EXPECT_FALSE(processor.process(request, kNow).committed);
  request = valid_request();
  request.left[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(processor.process(request, kNow).committed);
  request = valid_request();
  request.left = {10.0, 10.0, 10.0};
  EXPECT_FALSE(processor.process(request, kNow).committed);
  EXPECT_EQ(processor.left_q(), left_before);
  EXPECT_EQ(processor.right_q(), right_before);
}

}  // namespace
