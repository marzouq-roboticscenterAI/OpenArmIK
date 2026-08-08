// SPDX-License-Identifier: Apache-2.0
//
// Guards the one thing that decides which physical arm shows up on which side
// of the screen: the mirrored joint-limit signature.
//
// The risk being tested for is a discriminator that always answers the same
// way. A scorer that returned a constant, or that only ever saw joints whose
// limits are identical between the sides, would still have "identified" the
// live arm correctly by luck. So every directional check here is paired with
// its mirror image, and the pair must come out opposite.
#include "openarm_ik_ros/real_observer_core.hpp"

#include <gtest/gtest.h>

#include <array>

namespace
{
using openarm_ik_ros::real::joint_limit_misfit;
using openarm_ik_ros::real::map_motor_records_by_id;
using openarm_ik_ros::real::reply_matches_expected;
using openarm_ik_ros::real::BusReading;
using openarm_ik_ros::real::MotorRecord;
using openarm_ik_ros::real::kJointsPerArm;
using openarm_ik_ros::real::kLeftSide;
using openarm_ik_ros::real::kRightSide;

using Pose = std::array<double, kJointsPerArm>;

/// The pose actually measured on can1 from the connected arm, 2026-08-04. That
/// bus is the LEFT arm; this was confirmed on 2026-08-06 by moving each physical
/// arm while the strictly read-only observer streamed encoder deltas.
///
/// Joint 4 sits outside [0, 2.443] because the motor zeros are not commissioned
/// to the URDF zeros; that is deliberate, and a test below pins the fact that
/// it cancels out of the comparison rather than biasing it.
constexpr Pose kMeasured{-1.0454, -0.9939, -0.1135, -0.9783, 0.2295, -0.1042, -0.0138};

/// The same arm reflected: joints 1 and 2 are the only ones whose limits differ
/// between the sides, so negating them is what a mirrored pose looks like.
Pose mirrored(const Pose & pose)
{
  Pose out = pose;
  out[0] = -pose[0];
  out[1] = -pose[1];
  return out;
}

TEST(RealObserverIdentification, TheMeasuredCan1PosePrefersTheConfirmedLeftSide)
{
  // The observed pose and operator-confirmed side agree for this sample. This
  // does not make absolute-angle identification authoritative: the encoder
  // zeros are not commissioned to the URDF zeros, so another pose or rebuilt
  // arm need not retain this signature. The observer therefore keeps heuristic
  // confidence low and launchers pin the operator-confirmed channel mapping.
  EXPECT_LT(joint_limit_misfit(kMeasured, kLeftSide), joint_limit_misfit(kMeasured, kRightSide))
    << "the measured can1 pose should prefer its operator-confirmed left side";
}

TEST(RealObserverIdentification, MirroringTheSamePoseFlipsTheAnswer)
{
  // Establishes that the scorer is directional rather than constant. Note what
  // this does and does not buy: it proves the arithmetic responds to its input,
  // not that an uncommissioned absolute encoder angle is a side identity.
  const Pose reflected = mirrored(kMeasured);
  EXPECT_GT(joint_limit_misfit(reflected, kLeftSide), joint_limit_misfit(reflected, kRightSide));
}

TEST(RealObserverIdentification, MirroringSwapsTheTwoScoresExactly)
{
  // Stronger than an inequality: the left/right limit boxes are reflections of
  // each other, so a reflected pose must score exactly as the original did on
  // the opposite side. This would fail if the limits were merely different
  // rather than mirrored, or if a non-mirrored joint leaked into the decision.
  const Pose reflected = mirrored(kMeasured);
  EXPECT_NEAR(
    joint_limit_misfit(reflected, kRightSide), joint_limit_misfit(kMeasured, kLeftSide), 1.0e-9);
  EXPECT_NEAR(
    joint_limit_misfit(reflected, kLeftSide), joint_limit_misfit(kMeasured, kRightSide), 1.0e-9);
}

TEST(RealObserverIdentification, TheDecisionMarginClearsTheNoiseFloor)
{
  // The observer refuses to call it below 0.05 rad. Record the margin the real
  // arm produced so a future change that quietly erodes it is visible here.
  const double margin =
    joint_limit_misfit(kMeasured, kRightSide) - joint_limit_misfit(kMeasured, kLeftSide);
  EXPECT_GT(margin, 0.05);
  EXPECT_NEAR(margin, 0.8194, 1.0e-3);
}

TEST(RealObserverIdentification, AnUncommissionedJointCancelsInsteadOfBiasing)
{
  // Joint 4 read -0.978 rad, outside [0, 2.443] for both sides. Both sides must
  // be penalised identically, otherwise an uncommissioned zero would drag the
  // identification toward whichever side happened to absorb the error.
  Pose only_joint_four{};
  only_joint_four[3] = -0.9783;
  EXPECT_NEAR(
    joint_limit_misfit(only_joint_four, kLeftSide),
    joint_limit_misfit(only_joint_four, kRightSide), 1.0e-12);
  EXPECT_GT(joint_limit_misfit(only_joint_four, kLeftSide), 0.9);
}

TEST(RealObserverIdentification, ASymmetricPoseIsCorrectlyIndecisive)
{
  // The zero pose is inside both boxes, so neither side is favoured and the
  // observer must decline rather than guess.
  const Pose neutral{};
  EXPECT_NEAR(joint_limit_misfit(neutral, kLeftSide), 0.0, 1.0e-12);
  EXPECT_NEAR(joint_limit_misfit(neutral, kRightSide), 0.0, 1.0e-12);
}

TEST(RealObserverIdentification, JointsWithSharedLimitsCannotDecideAnything)
{
  // Joints 3 and 5 to 7 have identical ranges on both sides. Driving them hard
  // out of range must move both scores by the same amount.
  for (const std::size_t joint : {std::size_t{2}, std::size_t{4}, std::size_t{5},
      std::size_t{6}})
  {
    Pose pose{};
    pose[joint] = 3.0;
    EXPECT_NEAR(
      joint_limit_misfit(pose, kLeftSide), joint_limit_misfit(pose, kRightSide), 1.0e-12)
      << "joint index " << joint << " must not influence the side decision";
  }
}

TEST(RealObserverMapping, MissingJointCannotShiftGripperIntoJ7)
{
  std::vector<MotorRecord> records;
  for (std::uint16_t id : {1U, 3U, 4U, 5U, 6U, 7U, 8U}) {
    MotorRecord record;
    record.send_id = id;
    record.position_rad = static_cast<double>(id);
    records.push_back(record);
  }
  BusReading reading;
  EXPECT_FALSE(map_motor_records_by_id(records, reading));
  EXPECT_FALSE(reading.complete);
  EXPECT_TRUE(reading.joint_valid[0]);
  EXPECT_FALSE(reading.joint_valid[1]);
  EXPECT_TRUE(reading.joint_valid[6]);
  EXPECT_DOUBLE_EQ(reading.position_rad[6], 7.0);
  EXPECT_TRUE(reading.has_gripper);
  EXPECT_DOUBLE_EQ(reading.gripper_rad, 8.0);
}

TEST(RealObserverMapping, ExactIdsMapToTheirJointAndGripperSlots)
{
  std::vector<MotorRecord> records;
  for (std::uint16_t id = 8U; id >= 1U; --id) {
    MotorRecord record;
    record.send_id = id;
    record.position_rad = static_cast<double>(id) * 0.1;
    record.velocity_rad_s = static_cast<double>(id) * 0.2;
    record.torque_nm = static_cast<double>(id) * 0.3;
    records.push_back(record);
  }
  BusReading reading;
  ASSERT_TRUE(map_motor_records_by_id(records, reading));
  ASSERT_TRUE(reading.complete);
  ASSERT_TRUE(reading.has_gripper);
  for (std::size_t joint = 0; joint < kJointsPerArm; ++joint) {
    EXPECT_DOUBLE_EQ(reading.position_rad[joint], static_cast<double>(joint + 1U) * 0.1);
    EXPECT_TRUE(reading.joint_valid[joint]);
  }
  EXPECT_DOUBLE_EQ(reading.gripper_rad, 0.8);
}

TEST(RealObserverMapping, LocalLoopbackRefreshCannotBecomeJointFeedback)
{
  const std::vector<std::uint16_t> expected{1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};

  // This is the arbitration ID of every DaMiao refresh-status command. A
  // second local raw-CAN socket receives it through loopback, but it is never
  // encoder feedback even though byte zero contains the requested motor ID.
  EXPECT_FALSE(reply_matches_expected(0x7ffU, 2U, expected, 0x10U));

  EXPECT_TRUE(reply_matches_expected(0x12U, 2U, expected, 0x10U));
  EXPECT_FALSE(reply_matches_expected(0x12U, 1U, expected, 0x10U));
  EXPECT_FALSE(reply_matches_expected(0x19U, 9U, expected, 0x10U));
}
}  // namespace
