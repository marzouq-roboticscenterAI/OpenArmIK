// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/portal_core.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <unistd.h>

namespace portal = openarm_ik_ros::portal;

namespace
{
portal::JointVector solve(std::size_t side, const portal::Point & target)
{
  portal::JointVector seed{};
  oa_ik_options options{};
  options.abi_version = OA_MODEL_ABI_VERSION;
  options.struct_size = sizeof(options);
  std::copy(seed.begin(), seed.end(), options.seed);
  std::copy(seed.begin(), seed.end(), options.posture);
  std::fill(std::begin(options.posture_weight), std::end(options.posture_weight), 1.0);
  options.position_tolerance_m = 1.0e-6;
  options.max_joint_step_rad = 0.12;
  options.damping_min = 1.0e-5;
  options.damping_max = 0.1;
  options.limit_margin_rad = 1.0e-5;
  options.max_iterations = 500;
  oa_ik_diagnostics diagnostics{};
  const oa_model * model = side == 0 ? oa_model_left_v10_bimanual() :
    oa_model_right_v10_bimanual();
  EXPECT_EQ(oa_ik_position_v2(
      model, target.data(), &options, OA_IK_DIAGNOSTICS_VERSION,
      sizeof(diagnostics), &diagnostics), OA_MODEL_OK);
  std::copy(std::begin(diagnostics.q), std::end(diagnostics.q), seed.begin());
  return seed;
}

std::uint64_t own_start_ticks()
{
  std::ifstream input("/proc/self/stat");
  std::string line;
  std::getline(input, line);
  const std::size_t close = line.rfind(')');
  std::istringstream fields(line.substr(close + 2));
  std::string field;
  for (std::size_t index = 3; index <= 22; ++index) {
    fields >> field;
  }
  std::uint64_t value = 0;
  const auto parsed = std::from_chars(field.data(), field.data() + field.size(), value);
  EXPECT_EQ(parsed.ec, std::errc{});
  return value;
}
}  // namespace

TEST(StrictJson, AcceptsOnlyExactMoveSchema)
{
  portal::MoveRequest request;
  std::string reason;
  EXPECT_TRUE(portal::StrictJson::parse_move(
    R"({"side":"left","x":0.2,"y":0.3,"z":8.5e-1})", request, reason));
  EXPECT_EQ(request.side, portal::MoveRequest::Side::left);
  EXPECT_DOUBLE_EQ(request.target[0], 0.2);
  EXPECT_FALSE(portal::StrictJson::parse_move(
    R"({"side":"left","x":0.2,"y":0.3,"z":"0.8"})", request, reason));
  EXPECT_FALSE(portal::StrictJson::parse_move(
    R"({"side":"left","x":0.2,"y":0.3,"z":0.8,"extra":1})", request, reason));
  EXPECT_FALSE(portal::StrictJson::parse_move(
    R"({"side":"left","x":0.2,"x":0.3,"y":0.3,"z":0.8})", request, reason));
  EXPECT_FALSE(portal::StrictJson::parse_move("  ", request, reason));
}

TEST(MutationPolicy, RequiresExactLocalOriginAuthorityTokenAndType)
{
  portal::MutationPolicy policy("127.0.0.1:8080", "secret");
  std::string reason;
  portal::MutationHeaders valid{
    "127.0.0.1:8080", "http://127.0.0.1:8080", "secret", "application/json", 2};
  EXPECT_TRUE(policy.validate(valid, reason));
  auto invalid = valid;
  invalid.host = "localhost:8080";
  EXPECT_FALSE(policy.validate(invalid, reason));
  invalid = valid;
  invalid.origin = "http://evil.invalid";
  EXPECT_FALSE(policy.validate(invalid, reason));
  invalid = valid;
  invalid.csrf = "wrong";
  EXPECT_FALSE(policy.validate(invalid, reason));
  invalid = valid;
  invalid.content_type = "text/plain";
  EXPECT_FALSE(policy.validate(invalid, reason));
  invalid = valid;
  invalid.content_length = 513;
  EXPECT_FALSE(policy.validate(invalid, reason));
}

TEST(NominalPathGuard, RejectsNonfiniteAndUnprovenStates)
{
  portal::GuardInput input;
  input.request.side = portal::MoveRequest::Side::left;
  input.request.target = {0.2, 0.3, 0.85};
  input.measured_q[0][0] = std::numeric_limits<double>::quiet_NaN();
  const portal::GuardResult nonfinite = portal::NominalPathGuard().validate(input);
  EXPECT_FALSE(nonfinite.accepted);
  EXPECT_NE(nonfinite.reason.find("finite model bounds"), std::string::npos);

  input.measured_q = {};
  input.request.target = {50.0, 50.0, 50.0};
  const portal::GuardResult unreachable = portal::NominalPathGuard().validate(input);
  EXPECT_FALSE(unreachable.accepted);
}

TEST(NominalPathGuard, ValidatesAStationaryRegressionPoseThroughPublicFkIk)
{
  const portal::Point left{0.2, 0.3, 0.85};
  const portal::Point right{0.2, -0.3, 0.85};
  portal::GuardInput input;
  input.measured_q[0] = solve(0, left);
  input.measured_q[1] = solve(1, right);
  input.request.side = portal::MoveRequest::Side::left;
  input.request.target = left;
  const portal::GuardResult result = portal::NominalPathGuard().validate(input);
  EXPECT_TRUE(result.accepted) << result.reason;
  EXPECT_GE(result.minimum_nominal_clearance_m, 0.025);
}

TEST(JsonEscape, EscapesControlAndDelimiterCharacters)
{
  EXPECT_EQ(portal::json_escape("a\"b\\c\n"), "a\\\"b\\\\c\\n");
}

TEST(ProcessIdentity, RequiresExactLivePidAndStartTicks)
{
  const std::uint64_t ticks = own_start_ticks();
  ASSERT_NE(ticks, 0U);
  EXPECT_TRUE(portal::process_identity_matches(getpid(), ticks));
  EXPECT_FALSE(portal::process_identity_matches(getpid(), ticks + 1));
  EXPECT_FALSE(portal::process_identity_matches(0, ticks));
}
