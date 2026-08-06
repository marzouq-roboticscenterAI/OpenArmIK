// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/portal_core.hpp"

#include <gtest/gtest.h>
#include <openarm_control_msgs/action/move_paired_tcp.hpp>
#include <openarm_control_msgs/action/move_paired_tcp_scaled.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <sstream>
#include <set>
#include <string>
#include <type_traits>
#include <vector>

namespace portal = openarm_ik_ros::portal;
using PairedAction = openarm_control_msgs::action::MovePairedTcp;
using ScaledPairedAction = openarm_control_msgs::action::MovePairedTcpScaled;

static_assert(std::is_same_v<portal::Point::value_type, double>);
static_assert(std::is_same_v<decltype(PairedAction::Goal{}.left_tcp_m.x), double>);
static_assert(std::is_same_v<decltype(PairedAction::Goal{}.right_tcp_m.z), double>);
static_assert(std::is_same_v<decltype(ScaledPairedAction::Goal{}.motion_limit_scale), double>);

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

bool inverse_from(
  std::size_t side, const portal::Point & target, const portal::JointVector & seed,
  portal::JointVector & output)
{
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
  const oa_model * model = side == 0U ?
    oa_model_left_v10_bimanual() : oa_model_right_v10_bimanual();
  if (oa_ik_position_v2(
      model, target.data(), &options, OA_IK_DIAGNOSTICS_VERSION,
      sizeof(diagnostics), &diagnostics) != OA_MODEL_OK ||
    diagnostics.position_error_m > 1.0e-5)
  {
    return false;
  }
  std::copy(std::begin(diagnostics.q), std::end(diagnostics.q), output.begin());
  return true;
}

portal::JointVector quantize_endpoint(std::size_t side, const portal::JointVector & input)
{
  portal::JointVector output{};
  for (std::size_t joint = 0; joint < output.size(); ++joint) {
    const double scale = ((side + joint) & 1U) == 0U ? 1.0 : -1.0;
    constexpr double offset = 0.125;
    constexpr double span = 12.5;
    constexpr double maximum = 65535.0;
    const double raw = (input[joint] - offset) / scale;
    const double unit = (std::clamp(raw, -span, span) + span) / (2.0 * span);
    const double encoded = std::round(unit * maximum);
    output[joint] = scale * (encoded / maximum * (2.0 * span) - span) + offset;
  }
  return output;
}

bool guarded_path_endpoint(
  std::size_t side, const portal::JointVector & start, const portal::Point & target,
  portal::JointVector & output)
{
  portal::JointVector current = start;
  const portal::Point begin = tcp(side, start);
  for (std::size_t sample = 1U; sample < 17U; ++sample) {
    const double fraction = static_cast<double>(sample) / 16.0;
    portal::Point waypoint{};
    for (std::size_t axis = 0; axis < waypoint.size(); ++axis) {
      waypoint[axis] = begin[axis] + fraction * (target[axis] - begin[axis]);
    }
    portal::JointVector next{};
    if (!inverse_from(side, waypoint, current, next)) {return false;}
    for (std::size_t joint = 0; joint < current.size(); ++joint) {
      if (std::abs(next[joint] - current[joint]) > 0.35) {return false;}
    }
    current = next;
  }
  output = quantize_endpoint(side, current);
  return true;
}

std::string read_file(const char * path)
{
  std::ifstream input(path);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

}  // namespace

TEST(StrictJson, AcceptsOnlyExactMoveSchema)
{
  portal::MoveRequest request;
  request.motion_limit_scale = 1.0;
  std::string reason;
  EXPECT_TRUE(portal::StrictJson::parse_move(
    R"({"side":"left","x":0.2,"y":0.3,"z":8.5e-1})", request, reason));
  EXPECT_EQ(request.side, portal::MoveRequest::Side::left);
  EXPECT_DOUBLE_EQ(request.target[0], 0.2);
  EXPECT_DOUBLE_EQ(request.motion_limit_scale, openarm_ik_ros::kLegacyMotionLimitScale);
  EXPECT_FALSE(portal::StrictJson::parse_move(
    R"({"side":"left","x":0.2,"y":0.3,"z":"0.8"})", request, reason));
  EXPECT_FALSE(portal::StrictJson::parse_move(
    R"({"side":"left","x":0.2,"y":0.3,"z":0.8,"extra":1})", request, reason));
  EXPECT_FALSE(portal::StrictJson::parse_move(
    R"({"side":"left","x":0.2,"x":0.3,"y":0.3,"z":0.8})", request, reason));
  EXPECT_FALSE(portal::StrictJson::parse_move("  ", request, reason));
  EXPECT_FALSE(portal::StrictJson::parse_move(
    R"({"side":"left","unit":"m","x":0.2,"y":0.3,"z":0.8})", request, reason));
}

TEST(StrictJson, V2RequiresExactExplicitCoordinateUnitSchema)
{
  portal::UnitMoveRequest request;
  std::string reason;
  EXPECT_TRUE(portal::StrictJson::parse_move_v2(
    R"({"side":"right","unit":"cm","x":2.54,"y":-14.25,"z":9.6})", request,
    reason)) << reason;
  EXPECT_EQ(request.side, portal::MoveRequest::Side::right);
  EXPECT_EQ(request.coordinate_unit, OA_LENGTH_UNIT_CENTIMETRES);
  EXPECT_DOUBLE_EQ(request.target.x, 2.54);
  EXPECT_FALSE(portal::StrictJson::parse_move_v2(
    R"({"side":"left","x":2,"y":3,"z":4})", request, reason));
  EXPECT_FALSE(portal::StrictJson::parse_move_v2(
    R"({"side":"left","unit":"feet","x":2,"y":3,"z":4})", request, reason));
  EXPECT_FALSE(portal::StrictJson::parse_move_v2(
    R"({"side":"left","unit":"m","unit":"cm","x":2,"y":3,"z":4})", request,
    reason));
  EXPECT_FALSE(portal::StrictJson::parse_move_v2(
    R"({"side":"left","unit":"in","x":"2","y":3,"z":4})", request, reason));
  EXPECT_FALSE(portal::StrictJson::parse_move_v2(
    R"({"side":"left","unit":"m","x":0x10,"y":3,"z":4})", request, reason));
}

TEST(StrictJson, V3RequiresExactExplicitMotionLimitSchema)
{
  portal::UnitMoveRequest request;
  std::string reason;
  EXPECT_TRUE(portal::StrictJson::parse_move_v3(
    R"({"side":"right","unit":"cm","x":2.54,"y":-14.25,"z":9.6,"motion_limit_scale":0.8})",
    request, reason)) << reason;
  EXPECT_EQ(request.side, portal::MoveRequest::Side::right);
  EXPECT_EQ(request.coordinate_unit, OA_LENGTH_UNIT_CENTIMETRES);
  EXPECT_DOUBLE_EQ(request.motion_limit_scale, 0.8);
  portal::MoveRequest normalized;
  ASSERT_TRUE(portal::normalise_move_to_metres(request, normalized, reason)) << reason;
  EXPECT_DOUBLE_EQ(normalized.motion_limit_scale, 0.8);
  for (const char * body : {
      R"({"side":"left","unit":"m","x":0.2,"y":0.3,"z":0.8})",
      R"({"side":"left","unit":"m","x":0.2,"y":0.3,"z":0.8,"motion_limit_scale":0.49})",
      R"({"side":"left","unit":"m","x":0.2,"y":0.3,"z":0.8,"motion_limit_scale":1.01})",
      R"({"side":"left","unit":"m","x":0.2,"y":0.3,"z":0.8,"motion_limit_scale":"0.8"})",
      R"({"side":"left","unit":"m","x":0.2,"y":0.3,"z":0.8,"motion_limit_scale":0.8,"motion_limit_scale":0.9})",
      R"({"side":"left","unit":"m","x":0.2,"y":0.3,"z":0.8,"motion_limit_scale":0.8,"extra":1})"})
  {
    EXPECT_FALSE(portal::StrictJson::parse_move_v3(body, request, reason)) << body;
  }
  EXPECT_TRUE(portal::StrictJson::parse_move_v3(
    R"({"motion_limit_scale":0.5,"z":0.8,"x":0.2,"unit":"m","side":"left","y":0.3})",
    request, reason)) << reason;
  EXPECT_DOUBLE_EQ(request.motion_limit_scale, 0.5);
  EXPECT_TRUE(portal::StrictJson::parse_move_v3(
    R"({"side":"left","unit":"m","x":0.2,"y":0.3,"z":0.8,"motion_limit_scale":1.0})",
    request, reason)) << reason;
  EXPECT_DOUBLE_EQ(request.motion_limit_scale, 1.0);
}

TEST(CoordinateUnits, V2NormalizesMetresCentimetresAndInchesOnce)
{
  struct Example
  {
    const char * json;
    oa_length_unit unit;
  };
  const Example examples[] = {
    {R"({"side":"left","unit":"m","x":0.0254,"y":-0.0508,"z":0.0762})",
      OA_LENGTH_UNIT_METRES},
    {R"({"side":"left","unit":"cm","x":2.54,"y":-5.08,"z":7.62})",
      OA_LENGTH_UNIT_CENTIMETRES},
    {R"({"side":"left","unit":"in","x":1,"y":-2,"z":3})", OA_LENGTH_UNIT_INCHES},
  };
  for (const Example & example : examples) {
    portal::UnitMoveRequest request;
    portal::MoveRequest normalized;
    std::string reason;
    ASSERT_TRUE(portal::StrictJson::parse_move_v2(example.json, request, reason)) << reason;
    EXPECT_EQ(request.coordinate_unit, example.unit);
    ASSERT_TRUE(portal::normalise_move_to_metres(request, normalized, reason)) << reason;
    EXPECT_NEAR(normalized.target[0], 0.0254, 2.0e-17);
    EXPECT_NEAR(normalized.target[1], -0.0508, 2.0e-17);
    EXPECT_NEAR(normalized.target[2], 0.0762, 2.0e-17);
    EXPECT_DOUBLE_EQ(normalized.motion_limit_scale, openarm_ik_ros::kLegacyMotionLimitScale);
  }

  constexpr double canonical = 0.020081;
  struct DisplayUnit
  {
    const char * token;
    double units_per_metre;
  };
  const DisplayUnit units[] = {{"m", 1.0}, {"cm", 100.0}, {"in", 1.0 / 0.0254}};
  for (const DisplayUnit & unit : units) {
    const double displayed = canonical * unit.units_per_metre;
    const std::string body = std::string{"{\"side\":\"left\",\"unit\":\""} + unit.token +
      "\",\"x\":" + portal::json_number(displayed) + ",\"y\":0,\"z\":0}";
    portal::UnitMoveRequest request;
    portal::MoveRequest normalized;
    std::string reason;
    ASSERT_TRUE(portal::StrictJson::parse_move_v2(body, request, reason)) << reason;
    ASSERT_TRUE(portal::normalise_move_to_metres(request, normalized, reason)) << reason;
    const double one_ulp = std::nextafter(canonical, std::numeric_limits<double>::infinity()) -
      canonical;
    EXPECT_LE(std::abs(normalized.target[0] - canonical), one_ulp);
  }
}

TEST(CoordinateUnits, Binary64SurvivesJsonNormalizationAndRosActionAssignment)
{
  constexpr double precise = 0.12345678901234566;
  const std::string encoded = portal::json_number(precise);
  double decoded = 0.0;
  const auto parsed = std::from_chars(encoded.data(), encoded.data() + encoded.size(), decoded);
  ASSERT_EQ(parsed.ec, std::errc{});
  ASSERT_EQ(parsed.ptr, encoded.data() + encoded.size());
  EXPECT_DOUBLE_EQ(decoded, precise);
  EXPECT_NE(decoded, static_cast<double>(static_cast<float>(decoded)));

  portal::UnitMoveRequest request;
  std::string reason;
  ASSERT_TRUE(portal::StrictJson::parse_move_v2(
    std::string{"{\"side\":\"left\",\"unit\":\"m\",\"x\":"} + encoded +
    R"(,"y":0.2,"z":0.3})", request, reason)) << reason;
  portal::MoveRequest normalized;
  ASSERT_TRUE(portal::normalise_move_to_metres(request, normalized, reason)) << reason;
  PairedAction::Goal goal;
  goal.left_tcp_m.x = normalized.target[0];
  EXPECT_DOUBLE_EQ(goal.left_tcp_m.x, precise);
  EXPECT_NE(goal.left_tcp_m.x, static_cast<double>(static_cast<float>(goal.left_tcp_m.x)));
}

TEST(CoordinateUnits, StateJsonDeclaresMetresAndRoundTripsBinary64)
{
  constexpr double precise = 0.12345678901234566;
  const std::array<portal::Point, 2> tcp{{
    {precise, -0.25, 0.75}, {0.5, -precise, 1.0}}};
  const std::string state = portal::portal_state_json(
    true, false, tcp, "fresh \"state\"", "no command");
  EXPECT_NE(state.find("\"coordinate_unit\":\"m\""), std::string::npos);
  EXPECT_NE(state.find(portal::json_number(precise)), std::string::npos);
  EXPECT_NE(state.find("fresh \\\"state\\\""), std::string::npos);
  EXPECT_EQ(state.find(std::to_string(static_cast<float>(precise))), std::string::npos);
}

TEST(PortalPage, UsesSameOriginExternalAssetsAndSerializedCanonicalTargets)
{
  const std::string page = portal::portal_page("test-token");
  EXPECT_NE(page.find("Coordinate display/input units"), std::string::npos);
  EXPECT_NE(page.find("value=\"cm\" checked"), std::string::npos);
  EXPECT_NE(page.find("value=\"in\""), std::string::npos);
  EXPECT_NE(page.find("value=\"m\""), std::string::npos);
  EXPECT_NE(page.find("id=\"motion-limit-scale\" type=\"range\" min=\"50\" max=\"100\" step=\"5\" value=\"80\""), std::string::npos);
  EXPECT_NE(page.find("/web/portal.css"), std::string::npos);
  EXPECT_NE(page.find("/web/portal.js"), std::string::npos);
  // The 3D view is a real RViz window, so the page must not reintroduce the
  // retired in-browser WebGL proxy or its controls.
  EXPECT_EQ(page.find("/web/viewer.js"), std::string::npos);
  EXPECT_EQ(page.find("viewer-canvas"), std::string::npos);
  EXPECT_EQ(page.find("viewer-neutral-palette"), std::string::npos);
  EXPECT_EQ(page.find("<style>"), std::string::npos);
  EXPECT_NE(page.find("id=\"portal-targets\""), std::string::npos);
  EXPECT_NE(page.find("\"near_max_forward\""), std::string::npos);
  EXPECT_NE(page.find("\"High far\""), std::string::npos);
  EXPECT_NE(page.find(portal::json_number(0.48)), std::string::npos);
  EXPECT_NE(page.find(portal::json_number(-0.67)), std::string::npos);
  EXPECT_EQ(page.find("__CSRF__"), std::string::npos);
  EXPECT_NE(page.find("name=\"portal-csrf\" content=\"test-token\""), std::string::npos);
}

TEST(StrictJsonMoveBoth, RequiresEverySixCoordinateAndRejectsExtras)
{
  portal::UnitMoveRequest parsed;
  portal::Point right{};
  std::string reason;
  const std::string good =
    R"({"unit":"m","left_x":0.28,"left_y":0.20,"left_z":0.36,)"
    R"("right_x":0.33,"right_y":-0.24,"right_z":0.50,"motion_limit_scale":0.9})";
  ASSERT_TRUE(portal::StrictJson::parse_move_both(good, parsed, right, reason)) << reason;
  EXPECT_DOUBLE_EQ(parsed.target.x, 0.28);
  EXPECT_DOUBLE_EQ(parsed.target.y, 0.20);
  EXPECT_DOUBLE_EQ(parsed.target.z, 0.36);
  EXPECT_DOUBLE_EQ(right[0], 0.33);
  EXPECT_DOUBLE_EQ(right[1], -0.24);
  EXPECT_DOUBLE_EQ(right[2], 0.50);
  EXPECT_DOUBLE_EQ(parsed.motion_limit_scale, 0.9);

  // Every coordinate is mandatory; a partial pair must not silently hold an arm.
  const char * const incomplete[] = {
    R"({"unit":"m","left_x":0.28,"left_y":0.20,"left_z":0.36,"right_x":0.33,"right_y":-0.24,"motion_limit_scale":0.9})",
    R"({"unit":"m","left_y":0.20,"left_z":0.36,"right_x":0.33,"right_y":-0.24,"right_z":0.5,"motion_limit_scale":0.9})",
    R"({"left_x":0.28,"left_y":0.20,"left_z":0.36,"right_x":0.33,"right_y":-0.24,"right_z":0.5,"motion_limit_scale":0.9})",
    R"({"unit":"m","left_x":0.28,"left_y":0.20,"left_z":0.36,"right_x":0.33,"right_y":-0.24,"right_z":0.5})",
  };
  for (const char * body : incomplete) {
    EXPECT_FALSE(portal::StrictJson::parse_move_both(body, parsed, right, reason)) << body;
  }
  // Unknown, duplicate and non-finite fields are refused, not ignored.
  const char * const malformed[] = {
    R"({"unit":"m","left_x":0.28,"left_y":0.20,"left_z":0.36,"right_x":0.33,"right_y":-0.24,"right_z":0.5,"bogus":1,"motion_limit_scale":0.9})",
    R"({"unit":"m","left_x":0.28,"left_x":0.29,"left_y":0.20,"left_z":0.36,"right_x":0.33,"right_y":-0.24,"right_z":0.5,"motion_limit_scale":0.9})",
    R"({"unit":"m","left_x":nan,"left_y":0.20,"left_z":0.36,"right_x":0.33,"right_y":-0.24,"right_z":0.5,"motion_limit_scale":0.9})",
    R"({"unit":"m","left_x":0.28,"left_y":0.20,"left_z":0.36,"right_x":0.33,"right_y":-0.24,"right_z":0.5,"motion_limit_scale":1.4})",
  };
  for (const char * body : malformed) {
    EXPECT_FALSE(portal::StrictJson::parse_move_both(body, parsed, right, reason)) << body;
  }
}

TEST(NominalPathGuard, DualRequestMovesBothArmsAndIsNeverProjected)
{
  portal::GuardInput input;
  input.measured_q = {};   // canonical neutral
  input.request.dual = true;
  input.request.dual_target[0] = {0.30, 0.24, 0.40};
  input.request.dual_target[1] = {0.30, -0.24, 0.40};
  const portal::GuardResult accepted = portal::NominalPathGuard().validate_or_project(input);
  ASSERT_TRUE(accepted.accepted) << accepted.reason;
  // Both arms are commanded, not one held at its measured pose.
  EXPECT_NEAR(accepted.commanded_tcp[0][1], 0.24, 1.0e-9);
  EXPECT_NEAR(accepted.commanded_tcp[1][1], -0.24, 1.0e-9);
  EXPECT_GE(accepted.minimum_nominal_clearance_m, 0.025);
  EXPECT_FALSE(accepted.target_projected);

  // A dual request that drives the arms together is rejected outright. It must
  // never be shortened: stopping one arm early while the other continues would
  // change the relative geometry the samples were validated against.
  portal::GuardInput unsafe = input;
  unsafe.request.dual_target[0] = {0.30, 0.01, 0.40};
  unsafe.request.dual_target[1] = {0.30, -0.01, 0.40};
  const portal::GuardResult rejected = portal::NominalPathGuard().validate_or_project(unsafe);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_FALSE(rejected.target_projected);

  // Non-finite coordinates on either arm are refused.
  for (std::size_t side = 0; side < 2; ++side) {
    portal::GuardInput poisoned = input;
    poisoned.request.dual_target[side][2] =
      std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(portal::NominalPathGuard().validate_or_project(poisoned).accepted);
  }
}

TEST(PortalPage, CarriesStrictInputAndSafetyContracts)
{
  const std::string page = portal::portal_page("token");
  EXPECT_NE(page.find("Virtual simulation only."), std::string::npos);
  EXPECT_NE(page.find("not physically certified coordinates"), std::string::npos);
  EXPECT_NE(page.find("Controller collision checked: <strong>NO</strong>"), std::string::npos);
  EXPECT_NE(page.find("not a hardwired E-stop"), std::string::npos);
  EXPECT_NE(page.find("/api/rviz/stream"), std::string::npos);
  EXPECT_NE(page.find("id=\"both\""), std::string::npos);
  EXPECT_NE(page.find("real <strong>RViz</strong> pixels"), std::string::npos);
  // The stream used to be one-way and the page said "display-only". It now
  // replays pointer events into the real RViz window, so that claim would be
  // false and asserting it would pin a lie. What still has to be disclosed is
  // that input reaches RViz and that it can be refused, so those are checked
  // instead.
  EXPECT_EQ(page.find("display-only"), std::string::npos)
    << "the page must not claim the RViz stream is display-only now that "
       "pointer events are replayed into it";
  EXPECT_NE(page.find("pointer events are replayed"), std::string::npos);
  EXPECT_NE(page.find("Input is refused"), std::string::npos);
  EXPECT_NE(page.find("not collision checking"), std::string::npos);
  EXPECT_NE(page.find("never used as control feedback"), std::string::npos);
  EXPECT_NE(page.find("no portal-switchable coordinate grid"), std::string::npos);
  EXPECT_NE(page.find("Virtual guard test inputs (cm)"), std::string::npos);
  EXPECT_NE(page.find("[5000, 5000, 5000]"), std::string::npos);
  EXPECT_NE(page.find("[40, 5, 40]"), std::string::npos);
  EXPECT_NE(page.find("[48, -17, 35]"), std::string::npos);
  EXPECT_NE(page.find("farthest validated straight-line prefix"), std::string::npos);
  EXPECT_NE(page.find("Near-full audited reach"), std::string::npos);
  EXPECT_EQ(page.find("Full reach:"), std::string::npos);
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

TEST(SafeRequestPolicy, GatesEveryReadAndRequiresExactMutationOrigin)
{
  portal::SafeRequestPolicy policy("127.0.0.1:8080");
  std::string reason;
  portal::SafeRequestHeaders valid{
    "127.0.0.1:8080", "", "same-origin", 1, 0, 1};
  EXPECT_TRUE(policy.validate_read(valid, reason)) << reason;
  EXPECT_FALSE(policy.validate_mutation(valid, reason));
  valid.origin = "http://127.0.0.1:8080";
  valid.origin_count = 1;
  EXPECT_TRUE(policy.validate_mutation(valid, reason)) << reason;
  for (const auto invalid_site : {"cross-site", "same-site", "none-other", ""}) {
    auto invalid = valid;
    invalid.sec_fetch_site = invalid_site;
    invalid.sec_fetch_site_count = 1;
    EXPECT_FALSE(policy.validate_read(invalid, reason));
  }
  auto invalid = valid;
  invalid.host_count = 2;
  EXPECT_FALSE(policy.validate_read(invalid, reason));
  invalid = valid;
  invalid.host = "attacker.invalid";
  EXPECT_FALSE(policy.validate_read(invalid, reason));
  invalid = valid;
  invalid.origin_count = 2;
  EXPECT_FALSE(policy.validate_read(invalid, reason));
  invalid = valid;
  invalid.origin = "null";
  EXPECT_FALSE(policy.validate_read(invalid, reason));
  invalid = valid;
  invalid.sec_fetch_site_count = 0;
  EXPECT_TRUE(policy.validate_read(invalid, reason));
}

TEST(CommandReservationGate, StopInvalidatesInFlightGuardBeforeSubmission)
{
  portal::CommandReservationGate gate;
  std::uint64_t first = 0U;
  EXPECT_TRUE(gate.begin(first));
  EXPECT_NE(first, 0U);
  EXPECT_TRUE(gate.active());
  EXPECT_FALSE(gate.begin(first));
  EXPECT_TRUE(gate.cancel());
  EXPECT_FALSE(gate.active());
  EXPECT_FALSE(gate.valid(first));
  EXPECT_FALSE(gate.consume(first));

  std::uint64_t second = 0U;
  EXPECT_TRUE(gate.begin(second));
  EXPECT_GT(second, first);
  EXPECT_FALSE(gate.valid(first));
  EXPECT_TRUE(gate.valid(second));
  EXPECT_TRUE(gate.consume(second));
  EXPECT_FALSE(gate.active());
  EXPECT_FALSE(gate.cancel());

  std::uint64_t third = 0U;
  EXPECT_TRUE(gate.begin(third));
  EXPECT_TRUE(gate.release(third));
  EXPECT_FALSE(gate.active());
}

TEST(ViewerSnapshot, SerializesStrictJointOrderAndBinary64Positions)
{
  portal::ViewerSnapshot snapshot;
  snapshot.have_state = true;
  snapshot.fresh = true;
  snapshot.sequence = 42;
  snapshot.producer_time_ns = 1234567890;
  snapshot.receipt_steady_ns = 9000000000;
  snapshot.position_rad[0] = 0.12345678901234566;
  snapshot.position_rad[13] = -0.75;
  const std::string json = portal::viewer_state_json(snapshot, 9001250000);
  EXPECT_NE(json.find("\"schema\":1"), std::string::npos);
  EXPECT_NE(json.find("\"sequence\":\"42\""), std::string::npos);
  EXPECT_NE(json.find("\"producer_time_ns\":\"1234567890\""), std::string::npos);
  EXPECT_NE(json.find("\"receipt_age_ms\":1.25"), std::string::npos);
  EXPECT_NE(json.find("openarm_left_joint1"), std::string::npos);
  EXPECT_NE(json.find("openarm_right_joint7"), std::string::npos);
  EXPECT_NE(json.find(portal::json_number(snapshot.position_rad[0])), std::string::npos);
  snapshot.have_state = false;
  const std::string absent = portal::viewer_state_json(snapshot, 9001250000);
  EXPECT_NE(absent.find("\"have_state\":false"), std::string::npos);
  EXPECT_EQ(absent.find("position_rad"), std::string::npos);
}

TEST(JointStateMapping, RequiresExactUnambiguousCanonicalSet)
{
  std::vector<std::string> names;
  std::vector<double> positions;
  for (std::size_t side = 0; side < 2U; ++side) {
    const oa_model * model = side == 0U ?
      oa_model_left_v10_bimanual() : oa_model_right_v10_bimanual();
    for (std::size_t joint = 0; joint < OA_DOF; ++joint) {
      names.emplace_back(oa_model_joint_name(model, joint));
      positions.push_back(static_cast<double>(side * OA_DOF + joint) + 0.125);
    }
  }
  std::array<portal::JointVector, 2> mapped{};
  ASSERT_TRUE(portal::map_canonical_joint_state(names, positions, mapped));
  EXPECT_DOUBLE_EQ(mapped[0][0], 0.125);
  EXPECT_DOUBLE_EQ(mapped[1][6], 13.125);

  std::reverse(names.begin(), names.end());
  std::reverse(positions.begin(), positions.end());
  ASSERT_TRUE(portal::map_canonical_joint_state(names, positions, mapped));
  EXPECT_DOUBLE_EQ(mapped[0][0], 0.125);
  EXPECT_DOUBLE_EQ(mapped[1][6], 13.125);

  auto invalid_names = names;
  auto invalid_positions = positions;
  invalid_names.push_back("extra_joint");
  invalid_positions.push_back(99.0);
  EXPECT_FALSE(portal::map_canonical_joint_state(invalid_names, invalid_positions, mapped));
  invalid_names = names;
  invalid_positions = positions;
  invalid_names.pop_back();
  invalid_positions.pop_back();
  EXPECT_FALSE(portal::map_canonical_joint_state(invalid_names, invalid_positions, mapped));
  invalid_names = names;
  invalid_positions = positions;
  invalid_names.back() = invalid_names.front();
  invalid_positions.back() = -123.0;
  EXPECT_FALSE(portal::map_canonical_joint_state(invalid_names, invalid_positions, mapped));
  invalid_names = names;
  invalid_positions = positions;
  invalid_names.back() = "unknown_joint";
  EXPECT_FALSE(portal::map_canonical_joint_state(invalid_names, invalid_positions, mapped));
  invalid_positions = positions;
  invalid_positions.pop_back();
  EXPECT_FALSE(portal::map_canonical_joint_state(names, invalid_positions, mapped));
  invalid_positions = positions;
  invalid_positions[3] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(portal::map_canonical_joint_state(names, invalid_positions, mapped));
}

TEST(ViewerKinematicsOracle, AsymmetricLinkAndFixedFingerTransformsMatchPublicModel)
{
  const std::array<portal::JointVector, 2> q{{
    {{0.11,-0.22,0.33,-0.44,0.55,-0.66,0.77}},
    {{-0.17,0.28,-0.39,0.46,-0.58,0.69,-0.73}},
  }};
  const std::array<std::array<double, 16>, 2> browser_link7{{
    {{-0.2161366642,-0.5748510957,-0.7891964316,0,
      -0.2587691844,-0.7456699610,0.6140152812,0,
      -0.9414475560,0.3369309008,0.0124128778,0,
      -0.1303039938,0.2732512355,0.3110575974,1}},
    {{0.1511632949,0.8105254769,-0.5658605695,0,
      0.7984836102,-0.4375739694,-0.4134647548,0,
      -0.5827295184,-0.3893296719,-0.7133364081,0,
      0.0184688978,-0.2327470928,0.2813513875,1}},
  }};
  const std::array<portal::Point, 2> fixed_finger_mesh_translation{{
    {{0.4071400464,0.1186301410,0.2764943242}},
    {{0.3798556924,-0.0349969082,0.6611446142}},
  }};
  for (std::size_t side = 0; side < 2U; ++side) {
    oa_fk_result fk{};
    const oa_model * model = side == 0U ?
      oa_model_left_v10_bimanual() : oa_model_right_v10_bimanual();
    ASSERT_EQ(oa_fk(model, q[side].data(), &fk), OA_MODEL_OK);
    for (std::size_t column = 0; column < 4U; ++column) {
      for (std::size_t row = 0; row < 4U; ++row) {
        EXPECT_NEAR(
          browser_link7[side][column * 4U + row],
          fk.link_post[6].m[row * 4U + column], 2.0e-6);
      }
    }
    const portal::Point local{{0.0, side == 0U ? -0.045 : 0.045, -0.558501}};
    for (std::size_t row = 0; row < 3U; ++row) {
      const double world = fk.link_post[6].m[row * 4U + 3U] +
        fk.link_post[6].m[row * 4U] * local[0] +
        fk.link_post[6].m[row * 4U + 1U] * local[1] +
        fk.link_post[6].m[row * 4U + 2U] * local[2];
      EXPECT_NEAR(world, fixed_finger_mesh_translation[side][row], 2.0e-6);
    }
  }
}

TEST(ViewerScript, UsesSequentialThirtyHertzPollingAndLocalOnlyCameraEvents)
{
  const std::string viewer = read_file(OPENARM_VIEWER_JS_PATH);
  const std::string page = read_file(OPENARM_PORTAL_JS_PATH);
  ASSERT_FALSE(viewer.empty());
  ASSERT_FALSE(page.empty());
  EXPECT_NE(viewer.find("const PERIOD_MS = 1000 / 30"), std::string::npos);
  EXPECT_NE(viewer.find("let pollInFlight = false"), std::string::npos);
  EXPECT_NE(viewer.find("if (pollInFlight) return; pollInFlight = true;"), std::string::npos);
  EXPECT_NE(viewer.find("fetch('/api/view-state'"), std::string::npos);
  EXPECT_NE(viewer.find("while (nextPollDeadline <= now)"), std::string::npos);
  EXPECT_NE(viewer.find("sequence <= acceptedSequence"), std::string::npos);
  EXPECT_NE(viewer.find("VIEW STALE"), std::string::npos);
  EXPECT_NE(viewer.find("requestAnimationFrame(draw)"), std::string::npos);
  EXPECT_NE(viewer.find("metricsRing.length > 512"), std::string::npos);
  EXPECT_NE(viewer.find("gl.fenceSync"), std::string::npos);
  EXPECT_NE(viewer.find("openarm-viewer-draw-"), std::string::npos);
  EXPECT_NE(viewer.find("openarm-viewer-pose-"), std::string::npos);
  EXPECT_NE(viewer.find("MAX_PIXELS = 1920 * 1080"), std::string::npos);
  EXPECT_NE(viewer.find("webglcontextlost"), std::string::npos);
  EXPECT_NE(viewer.find("addEventListener('pointermove'"), std::string::npos);
  EXPECT_NE(viewer.find("addEventListener('wheel'"), std::string::npos);
  EXPECT_NE(viewer.find("const active = new Map()"), std::string::npos);
  EXPECT_NE(viewer.find("if (active.size >= 2)"), std::string::npos);
  EXPECT_EQ(viewer.find("addEventListener('touchmove'"), std::string::npos);
  EXPECT_NE(viewer.find("reset-view"), std::string::npos);
  EXPECT_NE(viewer.find("viewer-neutral-palette"), std::string::npos);
  EXPECT_NE(viewer.find("paletteName = 'blue'"), std::string::npos);
  EXPECT_NE(viewer.find("nonfinite STL vertex"), std::string::npos);
  EXPECT_NE(viewer.find("MAX_ENCODED_BYTES = 2498724"), std::string::npos);
  EXPECT_NE(viewer.find("MAX_GPU_BYTES = MAX_TRIANGLES * 9 * 4"), std::string::npos);
  EXPECT_NE(viewer.find("document.addEventListener('visibilitychange'"), std::string::npos);
  EXPECT_NE(viewer.find("MAX_CONTEXT_LOSSES = 2"), std::string::npos);
  EXPECT_EQ(viewer.find("method: 'POST'"), std::string::npos);
  EXPECT_EQ(viewer.find("/api/move"), std::string::npos);
  EXPECT_EQ(viewer.find("http://"), std::string::npos);
  EXPECT_EQ(viewer.find("https://"), std::string::npos);
  EXPECT_EQ(viewer.find("setInterval"), std::string::npos);
  EXPECT_NE(page.find("const metresPerUnit = {m: 1.0, cm: 0.01, in: 0.0254}"), std::string::npos);
  EXPECT_NE(page.find("post('/api/v3/move'"), std::string::npos);
  EXPECT_NE(page.find("motion_limit_scale"), std::string::npos);
  EXPECT_NE(page.find("renderPresets('left'); renderPresets('right')"), std::string::npos);
  EXPECT_NE(page.find("result.projected"), std::string::npos);
  EXPECT_NE(page.find("result.achieved_fraction"), std::string::npos);
  EXPECT_NE(page.find("guard queued only"), std::string::npos);
  EXPECT_EQ(page.find("guard moved only"), std::string::npos);
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

TEST(NominalPathGuard, BestEffortProjectsImpossibleTargetsForBothArms)
{
  constexpr double measured_neutral = 6.67582207984907e-05;
  portal::GuardInput input;
  for (auto & side : input.measured_q) {side.fill(measured_neutral);}
  for (std::size_t selected = 0U; selected < 2U; ++selected) {
    input.request.side = selected == 0U ?
      portal::MoveRequest::Side::left : portal::MoveRequest::Side::right;
    input.request.target = {50.0, selected == 0U ? 50.0 : -50.0, 50.0};
    const portal::GuardResult exact = portal::NominalPathGuard().validate(input);
    ASSERT_FALSE(exact.accepted);

    const portal::GuardResult projected =
      portal::NominalPathGuard().validate_or_project(input);
    ASSERT_TRUE(projected.accepted) << "side=" << selected << ": " << projected.reason <<
      "; exact_keepout=" << exact.sampled_keepout_violation <<
      "; exact_failure_fraction=" << exact.failure_path_fraction <<
      "; exact_reason=" << exact.reason;
    EXPECT_TRUE(projected.target_projected);
    EXPECT_FALSE(projected.limited_by_keepout);
    EXPECT_EQ(projected.requested_tcp, input.request.target);
    EXPECT_GT(projected.achieved_fraction, 0.007);
    EXPECT_LT(projected.achieved_fraction, 0.009);
    EXPECT_GE(projected.minimum_nominal_clearance_m, 0.025);
    const portal::Point measured = tcp(selected, input.measured_q[selected]);
    double displacement_squared = 0.0;
    for (std::size_t axis = 0; axis < 3U; ++axis) {
      const double delta = projected.commanded_tcp[selected][axis] - measured[axis];
      displacement_squared += delta * delta;
    }
    EXPECT_GT(std::sqrt(displacement_squared), 0.60);

    portal::GuardInput replay = input;
    replay.request.target = projected.commanded_tcp[selected];
    const portal::GuardResult revalidated = portal::NominalPathGuard().validate(replay);
    EXPECT_TRUE(revalidated.accepted) << revalidated.reason;
  }
}

TEST(NominalPathGuard, BestEffortNormalizesExtremeFiniteRayWithoutCollapsingToNoOp)
{
  constexpr double measured_neutral = 6.67582207984907e-05;
  portal::GuardInput input;
  for (auto & side : input.measured_q) {side.fill(measured_neutral);}
  input.request.side = portal::MoveRequest::Side::left;
  input.request.target = {1.0e300, 1.0e300, 1.0e300};
  const portal::GuardResult projected =
    portal::NominalPathGuard().validate_or_project(input);
  ASSERT_TRUE(projected.accepted) << projected.reason;
  EXPECT_TRUE(projected.target_projected);
  EXPECT_GT(projected.achieved_fraction, 0.0);
  EXPECT_LT(projected.achieved_fraction, 1.0e-299);
  EXPECT_GE(projected.minimum_nominal_clearance_m, 0.025);
  const portal::Point measured = tcp(0U, input.measured_q[0]);
  double displacement_squared = 0.0;
  for (std::size_t axis = 0; axis < 3U; ++axis) {
    const double delta = projected.commanded_tcp[0][axis] - measured[axis];
    displacement_squared += delta * delta;
  }
  EXPECT_GT(std::sqrt(displacement_squared), 0.60);

  portal::GuardInput replay = input;
  replay.request.target = projected.commanded_tcp[0];
  const portal::GuardResult revalidated = portal::NominalPathGuard().validate(replay);
  EXPECT_TRUE(revalidated.accepted) << revalidated.reason;
}

TEST(NominalPathGuard, BestEffortStopsAtSampledPoleKeepoutForBothArms)
{
  constexpr double measured_neutral = 6.67582207984907e-05;
  portal::GuardInput input;
  for (auto & side : input.measured_q) {side.fill(measured_neutral);}
  for (std::size_t selected = 0U; selected < 2U; ++selected) {
    input.request.side = selected == 0U ?
      portal::MoveRequest::Side::left : portal::MoveRequest::Side::right;
    input.request.target = {0.40, selected == 0U ? 0.05 : -0.05, 0.40};
    const portal::GuardResult exact = portal::NominalPathGuard().validate(input);
    ASSERT_FALSE(exact.accepted);

    const portal::GuardResult projected =
      portal::NominalPathGuard().validate_or_project(input);
    ASSERT_TRUE(projected.accepted) << "side=" << selected << ": " << projected.reason <<
      "; exact_keepout=" << exact.sampled_keepout_violation <<
      "; exact_failure_fraction=" << exact.failure_path_fraction <<
      "; exact_reason=" << exact.reason;
    EXPECT_TRUE(projected.target_projected);
    EXPECT_TRUE(projected.limited_by_keepout);
    EXPECT_GT(projected.achieved_fraction, 0.10);
    EXPECT_LT(projected.achieved_fraction, 0.20);
    EXPECT_GE(projected.minimum_nominal_clearance_m, 0.025);
    EXPECT_NE(projected.limiting_reason.find("central pole keepout"), std::string::npos);
    EXPECT_GT(projected.keepout_barrier_distance_m, 0.0);
    const portal::Point measured = tcp(selected, input.measured_q[selected]);
    double projected_distance_squared = 0.0;
    for (std::size_t axis = 0; axis < 3U; ++axis) {
      const double delta = projected.commanded_tcp[selected][axis] - measured[axis];
      projected_distance_squared += delta * delta;
    }
    EXPECT_LT(std::sqrt(projected_distance_squared), projected.keepout_barrier_distance_m);

    portal::GuardInput replay = input;
    replay.request.target = projected.commanded_tcp[selected];
    const portal::GuardResult revalidated = portal::NominalPathGuard().validate(replay);
    EXPECT_TRUE(revalidated.accepted) << revalidated.reason;
  }
}

TEST(NominalPathGuard, BestEffortStopsAtSampledInterArmKeepoutFromCrossPresetState)
{
  constexpr double measured_neutral = 6.67582207984907e-05;
  std::array<portal::JointVector, 2> neutral{};
  for (auto & side : neutral) {side.fill(measured_neutral);}
  const portal::Point left_start =
    portal::nominal_targets(portal::MoveRequest::Side::left)[6].point;
  const portal::Point right_start =
    portal::nominal_targets(portal::MoveRequest::Side::right)[6].point;
  portal::GuardInput input;
  ASSERT_TRUE(guarded_path_endpoint(0U, neutral[0], left_start, input.measured_q[0]));
  ASSERT_TRUE(guarded_path_endpoint(1U, neutral[1], right_start, input.measured_q[1]));
  input.request.side = portal::MoveRequest::Side::left;
  input.request.target = right_start;

  const portal::GuardResult exact = portal::NominalPathGuard().validate(input);
  ASSERT_FALSE(exact.accepted);
  ASSERT_TRUE(exact.sampled_keepout_violation) << exact.reason;
  EXPECT_NE(exact.reason.find("arm-arm capsule"), std::string::npos);

  const portal::GuardResult projected =
    portal::NominalPathGuard().validate_or_project(input);
  ASSERT_TRUE(projected.accepted) << projected.reason;
  EXPECT_TRUE(projected.target_projected);
  EXPECT_TRUE(projected.limited_by_keepout);
  EXPECT_GT(projected.achieved_fraction, 0.45);
  EXPECT_LT(projected.achieved_fraction, 0.52);
  EXPECT_GE(projected.minimum_nominal_clearance_m, 0.025);
  EXPECT_NE(projected.limiting_reason.find("arm-arm capsule"), std::string::npos);
  EXPECT_GT(projected.keepout_barrier_distance_m, 0.0);
  const portal::Point measured = tcp(0U, input.measured_q[0]);
  double projected_distance_squared = 0.0;
  for (std::size_t axis = 0; axis < 3U; ++axis) {
    const double delta = projected.commanded_tcp[0][axis] - measured[axis];
    projected_distance_squared += delta * delta;
  }
  EXPECT_LT(std::sqrt(projected_distance_squared), projected.keepout_barrier_distance_m);

  portal::GuardInput replay = input;
  replay.request.target = projected.commanded_tcp[0];
  const portal::GuardResult revalidated = portal::NominalPathGuard().validate(replay);
  EXPECT_TRUE(revalidated.accepted) << revalidated.reason;
}

TEST(NominalPathGuard, BestEffortRejectsSubMillimetrePrefixAsNoMotion)
{
  constexpr double measured_neutral = 6.67582207984907e-05;
  portal::GuardInput input;
  for (auto & side : input.measured_q) {side.fill(measured_neutral);}
  input.request.side = portal::MoveRequest::Side::left;
  // This legacy demo ray encounters a sampled keepout before a meaningful
  // validated prefix.  It must not be reported as a successful stationary move.
  input.request.target = {0.28, 0.80, 0.60};
  const portal::GuardResult projected =
    portal::NominalPathGuard().validate_or_project(input);
  EXPECT_FALSE(projected.accepted);
  EXPECT_FALSE(projected.target_projected);
  EXPECT_TRUE(projected.limited_by_keepout);
  EXPECT_GT(projected.keepout_barrier_distance_m, 0.0);
  EXPECT_NE(projected.reason.find("no motion submitted"), std::string::npos);
  EXPECT_NE(projected.limiting_reason.find("central pole keepout"), std::string::npos);
}

TEST(NominalPathGuard, BestEffortFailsClosedForInvalidMeasuredStateAndNonfiniteTarget)
{
  portal::GuardInput input;
  input.request.side = portal::MoveRequest::Side::left;
  input.request.target = {50.0, 50.0, 50.0};
  input.measured_q[0][0] = std::numeric_limits<double>::quiet_NaN();
  portal::GuardResult result = portal::NominalPathGuard().validate_or_project(input);
  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.target_projected);

  input.measured_q = {};
  input.request.target = {0.2, std::numeric_limits<double>::infinity(), 0.3};
  result = portal::NominalPathGuard().validate_or_project(input);
  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.target_projected);
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

TEST(NominalPathGuard, AllNinePresetsPerArmParseAndPassButNearbyPoleApproachFails)
{
  constexpr double measured_neutral = 6.67582207984907e-05;
  portal::GuardInput input;
  for (auto & side : input.measured_q) {side.fill(measured_neutral);}
  const auto & left_targets = portal::nominal_targets(portal::MoveRequest::Side::left);
  const auto & right_targets = portal::nominal_targets(portal::MoveRequest::Side::right);
  ASSERT_EQ(left_targets.size(), 9U);
  ASSERT_EQ(right_targets.size(), 9U);
  static constexpr std::array<std::string_view, 9> ids{{
    "near_low", "outer_low", "near_mid", "outer_mid", "forward_mid",
    "forward_outer", "near_max_forward", "outer_high", "high_far"}};
  static constexpr std::array<std::string_view, 9> labels{{
    "Near low", "Outer low", "Near mid", "Outer mid", "Forward mid",
    "Forward outer", "Near-max forward", "Outer high", "High far"}};
  static constexpr std::array<portal::Point, 9> expected_left{{
    {0.150000, 0.220000, 0.150000}, {0.150000, 0.400000, 0.150000},
    {0.150000, 0.220000, 0.300000}, {0.150000, 0.400000, 0.300000},
    {0.300000, 0.220000, 0.300000}, {0.300000, 0.500000, 0.300000},
    {0.480000, 0.170000, 0.350000}, {0.250000, 0.580000, 0.450000},
    {0.280000, 0.670000, 0.520000}}};
  static constexpr std::array<portal::Point, 9> expected_right{{
    {0.150000, -0.220000, 0.150000}, {0.150000, -0.400000, 0.150000},
    {0.150000, -0.220000, 0.300000}, {0.150000, -0.400000, 0.300000},
    {0.300000, -0.220000, 0.300000}, {0.300000, -0.500000, 0.300000},
    {0.480000, -0.170000, 0.350000}, {0.250000, -0.580000, 0.450000},
    {0.280000, -0.670000, 0.520000}}};
  std::set<std::string_view> unique_ids;
  for (std::size_t index = 0; index < ids.size(); ++index) {
    EXPECT_EQ(left_targets[index].id, ids[index]);
    EXPECT_EQ(right_targets[index].id, ids[index]);
    EXPECT_EQ(left_targets[index].label, labels[index]);
    EXPECT_EQ(right_targets[index].label, labels[index]);
    EXPECT_EQ(left_targets[index].point, expected_left[index]);
    EXPECT_EQ(right_targets[index].point, expected_right[index]);
    unique_ids.insert(left_targets[index].id);
  }
  EXPECT_EQ(unique_ids.size(), ids.size());
  for (const auto side : {portal::MoveRequest::Side::left, portal::MoveRequest::Side::right}) {
    const auto & targets = portal::nominal_targets(side);
    for (const portal::NominalTarget & target : targets) {
      std::ostringstream json;
      json << std::fixed << std::setprecision(6) <<
        "{\"side\":\"" << (side == portal::MoveRequest::Side::left ? "left" : "right") <<
        "\",\"x\":" << target.point[0] << ",\"y\":" << target.point[1] <<
        ",\"z\":" << target.point[2] << '}';
      std::string reason;
      ASSERT_TRUE(portal::StrictJson::parse_move(json.str(), input.request, reason)) << reason;
      const portal::GuardResult result = portal::NominalPathGuard().validate(input);
      EXPECT_TRUE(result.accepted) << json.str() << ": " << result.reason;
      EXPECT_GE(result.minimum_nominal_clearance_m, 0.025);
      const std::size_t selected = side == portal::MoveRequest::Side::left ? 0U : 1U;
      const std::size_t opposite = 1U - selected;
      EXPECT_EQ(result.commanded_tcp[selected], target.point);
      EXPECT_EQ(result.commanded_tcp[opposite], tcp(opposite, input.measured_q[opposite]));
      const portal::GuardResult best_effort =
        portal::NominalPathGuard().validate_or_project(input);
      ASSERT_TRUE(best_effort.accepted) << best_effort.reason;
      EXPECT_FALSE(best_effort.target_projected);
      EXPECT_DOUBLE_EQ(best_effort.achieved_fraction, 1.0);
      EXPECT_EQ(best_effort.commanded_tcp[selected], target.point);

      for (const auto & unit : std::array<std::pair<std::string_view, double>, 3>{{
          {"m", 1.0}, {"cm", 100.0}, {"in", 1.0 / 0.0254}}})
      {
        std::ostringstream v2;
        v2 << "{\"side\":\"" <<
          (side == portal::MoveRequest::Side::left ? "left" : "right") <<
          "\",\"unit\":\"" << unit.first << "\",\"x\":" <<
          portal::json_number(target.point[0] * unit.second) << ",\"y\":" <<
          portal::json_number(target.point[1] * unit.second) << ",\"z\":" <<
          portal::json_number(target.point[2] * unit.second) << '}';
        portal::UnitMoveRequest encoded;
        portal::MoveRequest normalized;
        ASSERT_TRUE(portal::StrictJson::parse_move_v2(v2.str(), encoded, reason)) << reason;
        ASSERT_TRUE(portal::normalise_move_to_metres(encoded, normalized, reason)) << reason;
        EXPECT_EQ(normalized.side, side);
        for (std::size_t axis = 0; axis < 3U; ++axis) {
          const double magnitude = std::abs(target.point[axis]);
          const double one_ulp =
            std::nextafter(magnitude, std::numeric_limits<double>::infinity()) - magnitude;
          EXPECT_LE(std::abs(normalized.target[axis] - target.point[axis]), one_ulp);
        }
      }
    }
  }

  input.request.side = portal::MoveRequest::Side::left;
  input.request.target = {0.019973, 0.118469, 0.096000};
  const portal::GuardResult rejected = portal::NominalPathGuard().validate(input);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_NE(rejected.reason.find("central pole keepout"), std::string::npos) << rejected.reason;
}

TEST(NominalTargets, HighFarUsesPinnedOpenSourceUrdfMeasurements)
{
  const oa_model * model = oa_model_left_v10_bimanual();
  ASSERT_NE(model, nullptr);
  ASSERT_NE(oa_model_provenance(model), nullptr);
  EXPECT_NE(
    std::string(oa_model_provenance(model)).find(
      "enactic/openarm_description@6c7b720f1ba48e8bafa3a3dc752c45f397b42221"),
    std::string::npos);

  portal::JointVector zero{};
  oa_fk_result fk{};
  ASSERT_EQ(oa_fk(model, zero.data(), &fk), OA_MODEL_OK);
  auto distance = [](const oa_transform & from, const oa_transform & to) {
      const double x = to.m[3] - from.m[3];
      const double y = to.m[7] - from.m[7];
      const double z = to.m[11] - from.m[11];
      return std::sqrt(x * x + y * y + z * z);
    };
  double measured_centreline_reach_m = 0.0;
  for (std::size_t joint = 0; joint + 1U < OA_DOF; ++joint) {
    measured_centreline_reach_m += distance(fk.joint_pre[joint], fk.joint_pre[joint + 1U]);
  }
  measured_centreline_reach_m += distance(fk.joint_pre[OA_DOF - 1U], fk.hand_tcp);
  const double source_measurement_sum_m =
    std::hypot(0.0301, 0.0600) + std::hypot(0.0301, 0.06625) +
    std::hypot(0.0315, 0.15375) + std::hypot(0.0315, 0.0955) +
    std::hypot(0.0375, 0.1205) + 0.0375 + 0.1025 + 0.0835;
  EXPECT_NEAR(measured_centreline_reach_m, source_measurement_sum_m, 1.0e-12);
  EXPECT_GT(measured_centreline_reach_m, 0.747);
  EXPECT_LT(measured_centreline_reach_m, 0.748);

  const portal::NominalTarget & high_far =
    portal::nominal_targets(portal::MoveRequest::Side::left).back();
  ASSERT_EQ(high_far.id, "high_far");
  ASSERT_EQ(high_far.point, (portal::Point{0.28, 0.67, 0.52}));
  const portal::Point shoulder{{
    fk.joint_pre[0].m[3], fk.joint_pre[0].m[7], fk.joint_pre[0].m[11]}};
  const double high_far_shoulder_distance_m = std::sqrt(
    std::pow(high_far.point[0] - shoulder[0], 2) +
    std::pow(high_far.point[1] - shoulder[1], 2) +
    std::pow(high_far.point[2] - shoulder[2], 2));
  EXPECT_GT(high_far_shoulder_distance_m / measured_centreline_reach_m, 0.89);

  constexpr double measured_neutral = 6.67582207984907e-05;
  portal::JointVector neutral{};
  neutral.fill(measured_neutral);
  const portal::Point neutral_tcp = tcp(0, neutral);
  const double tcp_displacement_m = std::sqrt(
    std::pow(high_far.point[0] - neutral_tcp[0], 2) +
    std::pow(high_far.point[1] - neutral_tcp[1], 2) +
    std::pow(high_far.point[2] - neutral_tcp[2], 2));
  EXPECT_GT(tcp_displacement_m, 0.736);
  EXPECT_LT(tcp_displacement_m, 0.737);
}

TEST(NominalPathGuard, EndpointQuantizedCrossStateMatrixRetainsAuditedClearance)
{
  constexpr double measured_neutral = 6.67582207984907e-05;
  std::array<portal::JointVector, 2> neutral{};
  for (auto & side : neutral) {side.fill(measured_neutral);}
  std::array<std::array<portal::JointVector, 9>, 2> endpoints{};
  for (std::size_t side = 0; side < 2U; ++side) {
    const auto portal_side = side == 0U ?
      portal::MoveRequest::Side::left : portal::MoveRequest::Side::right;
    const auto & targets = portal::nominal_targets(portal_side);
    for (std::size_t target = 0; target < targets.size(); ++target) {
      ASSERT_TRUE(guarded_path_endpoint(
        side, neutral[side], targets[target].point, endpoints[side][target]));
    }
  }
  std::size_t checked = 0U;
  double minimum = std::numeric_limits<double>::infinity();
  for (std::size_t left = 0; left <= 9U; ++left) {
    for (std::size_t right = 0; right <= 9U; ++right) {
      portal::GuardInput input;
      input.measured_q[0] = left == 0U ? neutral[0] : endpoints[0][left - 1U];
      input.measured_q[1] = right == 0U ? neutral[1] : endpoints[1][right - 1U];
      for (std::size_t side = 0; side < 2U; ++side) {
        input.request.side = side == 0U ?
          portal::MoveRequest::Side::left : portal::MoveRequest::Side::right;
        for (const portal::NominalTarget & target : portal::nominal_targets(input.request.side)) {
          input.request.target = target.point;
          const portal::GuardResult result = portal::NominalPathGuard().validate(input);
          ASSERT_TRUE(result.accepted) << "state=(" << left << ',' << right << ") side=" <<
            side << " target=" << target.id << " reason=" << result.reason;
          minimum = std::min(minimum, result.minimum_nominal_clearance_m);
          ++checked;
        }
      }
    }
  }
  EXPECT_EQ(checked, 1800U);
  EXPECT_GE(minimum, 0.0265);
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
