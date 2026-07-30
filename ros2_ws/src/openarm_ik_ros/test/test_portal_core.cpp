// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/portal_core.hpp"

#include <gtest/gtest.h>
#include <openarm_control_msgs/action/move_paired_tcp.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>

namespace portal = openarm_ik_ros::portal;
using PairedAction = openarm_control_msgs::action::MovePairedTcp;

static_assert(std::is_same_v<portal::Point::value_type, double>);
static_assert(std::is_same_v<decltype(PairedAction::Goal{}.left_tcp_m.x), double>);
static_assert(std::is_same_v<decltype(PairedAction::Goal{}.right_tcp_m.z), double>);

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

std::string read_file(const char * path)
{
  std::ifstream input(path);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
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
  }

  constexpr double canonical = 0.020081;
  struct DisplayUnit
  {
    const char * token;
    double units_per_metre;
  };
  const DisplayUnit units[] = {{"cm", 100.0}, {"in", 1.0 / 0.0254}};
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
  EXPECT_NE(page.find("Coordinate display units"), std::string::npos);
  EXPECT_NE(page.find("value=\"cm\" checked"), std::string::npos);
  EXPECT_NE(page.find("value=\"in\""), std::string::npos);
  EXPECT_NE(page.find("/web/portal.css"), std::string::npos);
  EXPECT_NE(page.find("/web/portal.js"), std::string::npos);
  EXPECT_NE(page.find("/web/viewer.js"), std::string::npos);
  EXPECT_EQ(page.find("<style>"), std::string::npos);
  EXPECT_NE(page.find("id=\"portal-targets\""), std::string::npos);
  EXPECT_NE(page.find("\"forward_high\""), std::string::npos);
  EXPECT_NE(page.find("\"High far\""), std::string::npos);
  EXPECT_NE(page.find(portal::json_number(0.059973)), std::string::npos);
  EXPECT_NE(page.find(portal::json_number(0.060081)), std::string::npos);
  EXPECT_EQ(page.find("__CSRF__"), std::string::npos);
  EXPECT_NE(page.find("name=\"portal-csrf\" content=\"test-token\""), std::string::npos);
}

TEST(PortalPage, CarriesStrictInputAndSafetyContracts)
{
  const std::string page = portal::portal_page("token");
  EXPECT_NE(page.find("Virtual simulation only."), std::string::npos);
  EXPECT_NE(page.find("not physically safe coordinates"), std::string::npos);
  EXPECT_NE(page.find("Controller collision checked: <strong>NO</strong>"), std::string::npos);
  EXPECT_NE(page.find("not a hardwired E-stop"), std::string::npos);
  EXPECT_NE(page.find("visual proxy — not collision checking"), std::string::npos);
  EXPECT_NE(page.find("OpenArm measured-pose viewer"), std::string::npos);
  EXPECT_NE(page.find("no portal-switchable coordinate grid"), std::string::npos);
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
  EXPECT_NE(viewer.find("addEventListener('touchmove'"), std::string::npos);
  EXPECT_NE(viewer.find("reset-view"), std::string::npos);
  EXPECT_EQ(viewer.find("method: 'POST'"), std::string::npos);
  EXPECT_EQ(viewer.find("/api/move"), std::string::npos);
  EXPECT_EQ(viewer.find("http://"), std::string::npos);
  EXPECT_EQ(viewer.find("https://"), std::string::npos);
  EXPECT_EQ(viewer.find("setInterval"), std::string::npos);
  EXPECT_NE(page.find("const metresPerUnit = {cm: 0.01, in: 0.0254}"), std::string::npos);
  EXPECT_NE(page.find("post('/api/v2/move'"), std::string::npos);
  EXPECT_NE(page.find("renderPresets('left'); renderPresets('right')"), std::string::npos);
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

TEST(NominalPathGuard, AllNinePresetsPerArmParseAndPassButNearbyPoleApproachFails)
{
  constexpr double measured_neutral = 6.67582207984907e-05;
  portal::GuardInput input;
  for (auto & side : input.measured_q) {side.fill(measured_neutral);}
  const auto & left_targets = portal::nominal_targets(portal::MoveRequest::Side::left);
  const auto & right_targets = portal::nominal_targets(portal::MoveRequest::Side::right);
  ASSERT_EQ(left_targets.size(), 9U);
  ASSERT_EQ(right_targets.size(), 9U);
  EXPECT_EQ(left_targets.front().id, "small");
  EXPECT_EQ(left_targets.back().id, "far_high");
  EXPECT_EQ(left_targets[2].point, (portal::Point{0.039973, 0.143469, 0.116000}));
  EXPECT_EQ(right_targets[8].point, (portal::Point{0.050081, -0.153527, 0.136000}));
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
