// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/portal_core.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iomanip>
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

portal::Point tcp(std::size_t side, const portal::JointVector & q)
{
  oa_fk_result result{};
  const oa_model * model = side == 0 ? oa_model_left_v10_bimanual() :
    oa_model_right_v10_bimanual();
  EXPECT_EQ(oa_fk(model, q.data(), &result), OA_MODEL_OK);
  return {result.hand_tcp.m[3], result.hand_tcp.m[7], result.hand_tcp.m[11]};
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

TEST(NominalPathGuard, AcceptsExactCanonicalNeutralStateWithPoleMargin)
{
  constexpr double measured_neutral = 6.67582207984907e-05;
  portal::GuardInput input;
  for (auto & side : input.measured_q) {side.fill(measured_neutral);}
  const portal::Point left = tcp(0, input.measured_q[0]);
  const portal::Point right = tcp(1, input.measured_q[1]);
  EXPECT_NEAR(left[0], -0.00002710217965846259, 1.0e-12);
  EXPECT_NEAR(left[1], 0.15346860855059954, 1.0e-12);
  EXPECT_NEAR(left[2], 0.07599955201969341, 1.0e-12);
  EXPECT_NEAR(right[0], 0.00008077910443504927, 1.0e-12);
  EXPECT_NEAR(right[1], -0.15352682530236783, 1.0e-12);
  EXPECT_NEAR(right[2], 0.07599955704732647, 1.0e-12);
  input.request.side = portal::MoveRequest::Side::left;
  input.request.target = left;
  const portal::GuardResult result = portal::NominalPathGuard().validate(input);
  EXPECT_TRUE(result.accepted) << result.reason;
  EXPECT_GE(result.minimum_nominal_clearance_m, 0.025);
}

TEST(NominalPathGuard, PresetsParseAndPassButNearbyPoleApproachFails)
{
  constexpr double measured_neutral = 6.67582207984907e-05;
  portal::GuardInput input;
  for (auto & side : input.measured_q) {side.fill(measured_neutral);}
  const portal::NominalTestSamples left_samples =
    portal::nominal_test_samples(portal::MoveRequest::Side::left);
  const portal::NominalTestSamples right_samples =
    portal::nominal_test_samples(portal::MoveRequest::Side::right);
  EXPECT_EQ(left_samples.small_forward_up, (portal::Point{0.019973, 0.143469, 0.096000}));
  EXPECT_EQ(left_samples.medium_forward_up, (portal::Point{0.029973, 0.143469, 0.106000}));
  EXPECT_EQ(right_samples.small_forward_up, (portal::Point{0.020081, -0.143527, 0.096000}));
  EXPECT_EQ(right_samples.medium_forward_up, (portal::Point{0.030081, -0.143527, 0.106000}));
  for (const auto side : {portal::MoveRequest::Side::left, portal::MoveRequest::Side::right}) {
    const portal::NominalTestSamples samples = portal::nominal_test_samples(side);
    for (const portal::Point & target : {samples.small_forward_up, samples.medium_forward_up}) {
      std::ostringstream json;
      json << std::fixed << std::setprecision(6) <<
        "{\"side\":\"" << (side == portal::MoveRequest::Side::left ? "left" : "right") <<
        "\",\"x\":" << target[0] << ",\"y\":" << target[1] <<
        ",\"z\":" << target[2] << '}';
      std::string reason;
      ASSERT_TRUE(portal::StrictJson::parse_move(json.str(), input.request, reason)) << reason;
      const portal::GuardResult result = portal::NominalPathGuard().validate(input);
      EXPECT_TRUE(result.accepted) << json.str() << ": " << result.reason;
      EXPECT_GE(result.minimum_nominal_clearance_m, 0.025);
    }
  }

  input.request.side = portal::MoveRequest::Side::left;
  input.request.target = {0.019973, 0.118469, 0.096000};
  const portal::GuardResult rejected = portal::NominalPathGuard().validate(input);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_NE(rejected.reason.find("central pole keepout"), std::string::npos) << rejected.reason;
}

TEST(NominalPathGuard, RetainsClearanceForDocumentedNearbyJointThreePosture)
{
  portal::GuardInput input;
  input.measured_q[0][2] = 0.15;
  input.measured_q[1][2] = -0.15;
  input.request.side = portal::MoveRequest::Side::left;
  input.request.target = tcp(0, input.measured_q[0]);
  const portal::GuardResult result = portal::NominalPathGuard().validate(input);
  EXPECT_TRUE(result.accepted) << result.reason;
  EXPECT_GE(result.minimum_nominal_clearance_m, 0.025);
}

TEST(FiniteShaftClearance, RejectsAProximalMovingCapsuleApproach)
{
  constexpr double shaft_radius = 0.04242640687119285;
  constexpr double bottom = 0.008;
  constexpr double top = 0.758;
  constexpr double arm_radius = 0.05;
  const double collision = portal::finite_cylinder_capsule_clearance(
    {0.16, 0.0, 0.20}, {0.10, 0.0, 0.20},
    shaft_radius, bottom, top, arm_radius);
  const double clear = portal::finite_cylinder_capsule_clearance(
    {0.16, 0.0, 0.20}, {0.13, 0.0, 0.20},
    shaft_radius, bottom, top, arm_radius);
  EXPECT_LT(collision, 0.025);
  EXPECT_GE(clear, 0.025);
}

TEST(FiniteShaftClearance, IncludesTopAndBottomCapsAndDiagonalRims)
{
  constexpr double shaft_radius = 0.04242640687119285;
  constexpr double bottom = 0.008;
  constexpr double top = 0.758;
  constexpr double capsule_radius = 0.05;
  auto clearance = [&](const portal::Point & point) {
      return portal::finite_cylinder_capsule_clearance(
        point, point, shaft_radius, bottom, top, capsule_radius);
    };

  EXPECT_LT(clearance({0.0, 0.0, top + capsule_radius + 0.024}), 0.025);
  EXPECT_GE(clearance({0.0, 0.0, top + capsule_radius + 0.026}), 0.025);
  EXPECT_LT(clearance({0.0, 0.0, bottom - capsule_radius - 0.024}), 0.025);
  EXPECT_GE(clearance({0.0, 0.0, bottom - capsule_radius - 0.026}), 0.025);

  const double radial = shaft_radius + 0.030;
  EXPECT_LT(clearance({radial, 0.0, top + 0.065}), 0.025);
  EXPECT_GE(clearance({radial, 0.0, top + 0.070}), 0.025);
  EXPECT_LT(clearance({radial, 0.0, bottom - 0.065}), 0.025);
  EXPECT_GE(clearance({radial, 0.0, bottom - 0.070}), 0.025);
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

TEST(ProcessIdentity, RequiresExactResolvedExecutablePath)
{
  std::array<char, 4096> executable{};
  const ssize_t length = readlink("/proc/self/exe", executable.data(), executable.size() - 1);
  ASSERT_GT(length, 0);
  const std::string exact(executable.data(), static_cast<std::size_t>(length));
  EXPECT_TRUE(portal::process_executable_matches(getpid(), exact));
  EXPECT_FALSE(portal::process_executable_matches(getpid(), exact + ".other"));
  EXPECT_FALSE(portal::process_executable_matches(getpid(), "test_portal_core"));
}

TEST(Freshness, RevalidatesProducerAndReceiptAgesAtUseTime)
{
  constexpr std::int64_t second = 1000000000;
  const portal::FreshnessEvidence fresh{10 * second, 20 * second};
  EXPECT_TRUE(portal::fresh_at_use(fresh, 10 * second + 500, 20 * second + 500, 1000));
  EXPECT_FALSE(portal::fresh_at_use(fresh, 10 * second + 1001, 20 * second + 500, 1000));
  EXPECT_FALSE(portal::fresh_at_use(fresh, 10 * second + 500, 20 * second + 1001, 1000));
  EXPECT_FALSE(portal::fresh_at_use(fresh, 10 * second - 1, 20 * second + 1, 1000));
  EXPECT_FALSE(portal::fresh_at_use({0, 20 * second}, 10 * second, 20 * second, 1000));
}

TEST(GuardHandoff, AcceptsEquivalentNewerHealthyGenerations)
{
  portal::GuardInput guarded;
  guarded.state_sequence = 4;
  guarded.diagnostic_sequence = 7;
  guarded.state_freshness = {9000, 19000};
  guarded.diagnostic_freshness = {9000, 19000};
  portal::GuardHandoffEvidence current;
  current.measured_q = guarded.measured_q;
  current.measured_q[1][6] += 0.5e-6;
  current.state_sequence = 5;
  current.diagnostic_sequence = 8;
  current.state_freshness = {10000, 20000};
  current.diagnostic_freshness = {10000, 20000};
  current.have_state = true;
  current.diagnostic_valid = true;
  EXPECT_TRUE(portal::guard_handoff_valid(guarded, current, 10500, 20500, 1000, 1000));
}

TEST(GuardHandoff, RejectsOneJointDriftAndReplayedEvidence)
{
  portal::GuardInput guarded;
  guarded.state_sequence = 4;
  guarded.diagnostic_sequence = 7;
  guarded.state_freshness = {9000, 19000};
  guarded.diagnostic_freshness = {9000, 19000};
  portal::GuardHandoffEvidence current;
  current.measured_q = guarded.measured_q;
  current.measured_q[0][3] += 2.0e-6;
  current.state_sequence = 5;
  current.diagnostic_sequence = 8;
  current.state_freshness = {10000, 20000};
  current.diagnostic_freshness = {10000, 20000};
  current.have_state = true;
  current.diagnostic_valid = true;
  EXPECT_FALSE(portal::guard_handoff_valid(guarded, current, 10500, 20500, 1000, 1000));

  current.measured_q = guarded.measured_q;
  current.state_freshness = guarded.state_freshness;
  EXPECT_FALSE(portal::guard_handoff_valid(guarded, current, 9500, 19500, 1000, 1000));
}

TEST(GuardHandoff, RejectsStaleOrFaultedDiagnostics)
{
  portal::GuardInput guarded;
  guarded.state_sequence = 4;
  guarded.diagnostic_sequence = 7;
  guarded.state_freshness = {9000, 19000};
  guarded.diagnostic_freshness = {9000, 19000};
  portal::GuardHandoffEvidence current;
  current.measured_q = guarded.measured_q;
  current.state_sequence = 5;
  current.diagnostic_sequence = 8;
  current.state_freshness = {10000, 20000};
  current.diagnostic_freshness = {9499, 19499};
  current.have_state = true;
  current.diagnostic_valid = true;
  EXPECT_FALSE(portal::guard_handoff_valid(guarded, current, 10500, 20500, 1000, 1000));

  current.diagnostic_freshness = {10000, 20000};
  current.diagnostic_valid = false;
  EXPECT_FALSE(portal::guard_handoff_valid(guarded, current, 10500, 20500, 1000, 1000));
}

TEST(XCompositeVersion, RequiresNamedPixmapProtocolMinimum)
{
  EXPECT_FALSE(portal::xcomposite_version_supported(0, 1));
  EXPECT_TRUE(portal::xcomposite_version_supported(0, 2));
  EXPECT_TRUE(portal::xcomposite_version_supported(0, 4));
  EXPECT_TRUE(portal::xcomposite_version_supported(1, 0));
  EXPECT_FALSE(portal::xcomposite_version_supported(-1, 99));
}

TEST(TrueColorPixels, UsesValidatedVisualMasksAndRejectsBlackFrames)
{
  const portal::TrueColorMasks rgb888{0xff0000, 0x00ff00, 0x0000ff};
  ASSERT_TRUE(portal::truecolor_masks_valid(rgb888));
  EXPECT_EQ(portal::truecolor_pixel_rgb(0x804020, rgb888),
    (std::array<unsigned char, 3>{128, 64, 32}));
  const portal::TrueColorMasks rgb565{0xf800, 0x07e0, 0x001f};
  ASSERT_TRUE(portal::truecolor_masks_valid(rgb565));
  EXPECT_EQ(portal::truecolor_pixel_rgb(0xffff, rgb565),
    (std::array<unsigned char, 3>{255, 255, 255}));
  EXPECT_FALSE(portal::truecolor_masks_valid({0, 0x00ff00, 0x0000ff}));
  EXPECT_FALSE(portal::truecolor_masks_valid({0xff0000, 0xff0000, 0x0000ff}));
  EXPECT_FALSE(portal::rgb_frame_has_nonblack_pixel({0, 0, 0, 0, 0, 0}));
  EXPECT_TRUE(portal::rgb_frame_has_nonblack_pixel({0, 0, 0, 0, 1, 0}));
  EXPECT_FALSE(portal::rgb_frame_has_nonblack_pixel({1, 0}));
}
