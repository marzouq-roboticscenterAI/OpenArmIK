// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/portal_core.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <utility>
#include <vector>

namespace openarm_ik_ros::portal
{
namespace
{
// Link envelope, required clearance, and central-shaft keepout now live in the
// model library (openarm_collision.h) so the pre-flight guard and the real-time
// execution monitor gate on the same constants.
constexpr std::size_t kSamples = 17;
constexpr std::size_t kProjectionSearchSteps = 64;
constexpr std::size_t kProjectionRefinementRounds = 3;
constexpr std::size_t kProjectionRefinementSamples = 16;
constexpr double kMinimumProjectedMotionM = 0.001;
// Twice the pinned 0.748 m centreline reach bounds TCP travel between any two
// reachable poses.  Two metres adds margin while bounding huge finite inputs.
constexpr double kProjectionMaximumRayM = 2.0;
// DaMiao position feedback has a 25/65535 = 3.8148e-4 rad code step.  Keep
// handoff equivalence far below one encoder code while covering floating-point
// publication/serialization noise; an actual adjacent-code change still fails.
constexpr double kGuardJointEquivalenceTolerance = 1.0e-6;

Point origin(const oa_transform & transform)
{
  return {transform.m[3], transform.m[7], transform.m[11]};
}

std::vector<std::string_view> split_fields(std::string_view body)
{
  std::vector<std::string_view> fields;
  std::size_t begin = 0;
  bool quoted = false;
  for (std::size_t i = 0; i < body.size(); ++i) {
    if (body[i] == '"') {
      quoted = !quoted;
    } else if (body[i] == ',' && !quoted) {
      fields.push_back(body.substr(begin, i - begin));
      begin = i + 1;
    }
  }
  fields.push_back(body.substr(begin));
  return fields;
}

std::string_view trim(std::string_view value)
{
  while (!value.empty() && (value.front() == ' ' || value.front() == '\t' ||
    value.front() == '\r' || value.front() == '\n'))
  {
    value.remove_prefix(1);
  }
  while (!value.empty() && (value.back() == ' ' || value.back() == '\t' ||
    value.back() == '\r' || value.back() == '\n'))
  {
    value.remove_suffix(1);
  }
  return value;
}

bool parse_number(std::string_view value, double & out)
{
  value = trim(value);
  if (value.empty() || value.size() > 32) {
    return false;
  }
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), out);
  return parsed.ec == std::errc{} && parsed.ptr == value.data() + value.size() &&
         std::isfinite(out);
}

bool parse_side(std::string_view value, MoveRequest::Side & side)
{
  if (value == "\"left\"") {
    side = MoveRequest::Side::left;
    return true;
  }
  if (value == "\"right\"") {
    side = MoveRequest::Side::right;
    return true;
  }
  return false;
}

bool parse_length_unit(std::string_view value, oa_length_unit & unit)
{
  if (value == "\"m\"") {
    unit = OA_LENGTH_UNIT_METRES;
    return true;
  }
  if (value == "\"cm\"") {
    unit = OA_LENGTH_UNIT_CENTIMETRES;
    return true;
  }
  if (value == "\"in\"") {
    unit = OA_LENGTH_UNIT_INCHES;
    return true;
  }
  return false;
}
}  // namespace

bool StrictJson::parse_move(std::string_view body, MoveRequest & out, std::string & reason)
{
  if (body.size() < 2 || body.size() > 512) {
    reason = "JSON body length is invalid";
    return false;
  }
  body = trim(body);
  if (body.size() < 2) {
    reason = "JSON object required";
    return false;
  }
  if (body.front() != '{' || body.back() != '}') {
    reason = "JSON object required";
    return false;
  }
  body.remove_prefix(1);
  body.remove_suffix(1);
  MoveRequest parsed;
  bool have_side = false;
  std::array<bool, 3> have_axis{};
  for (std::string_view field : split_fields(body)) {
    const std::size_t colon = field.find(':');
    if (colon == std::string_view::npos || field.find(':', colon + 1) != std::string_view::npos) {
      reason = "malformed JSON field";
      return false;
    }
    const std::string_view key = trim(field.substr(0, colon));
    const std::string_view value = trim(field.substr(colon + 1));
    if (key == "\"side\"") {
      if (have_side || (value != "\"left\"" && value != "\"right\"")) {
        reason = "side must be unique and left or right";
        return false;
      }
      have_side = true;
      parsed.side = value == "\"left\"" ? MoveRequest::Side::left : MoveRequest::Side::right;
      continue;
    }
    std::size_t axis = 3;
    if (key == "\"x\"") {axis = 0;} else if (key == "\"y\"") {axis = 1;} else if (
      key == "\"z\"") {axis = 2;}
    if (axis == 3 || have_axis[axis] || !parse_number(value, parsed.target[axis])) {
      reason = "only unique finite numeric x, y, z fields are allowed";
      return false;
    }
    have_axis[axis] = true;
  }
  if (!have_side || !have_axis[0] || !have_axis[1] || !have_axis[2]) {
    reason = "side, x, y, and z are required";
    return false;
  }
  out = parsed;
  return true;
}

bool StrictJson::parse_move_v2(
  std::string_view body, UnitMoveRequest & out, std::string & reason)
{
  if (body.size() < 2 || body.size() > 512) {
    reason = "JSON body length is invalid";
    return false;
  }
  body = trim(body);
  if (body.size() < 2 || body.front() != '{' || body.back() != '}') {
    reason = "JSON object required";
    return false;
  }
  body.remove_prefix(1);
  body.remove_suffix(1);
  UnitMoveRequest parsed;
  bool have_side = false;
  bool have_unit = false;
  std::array<bool, 3> have_axis{};
  for (std::string_view field : split_fields(body)) {
    const std::size_t colon = field.find(':');
    if (colon == std::string_view::npos || field.find(':', colon + 1) != std::string_view::npos) {
      reason = "malformed JSON field";
      return false;
    }
    const std::string_view key = trim(field.substr(0, colon));
    const std::string_view value = trim(field.substr(colon + 1));
    if (key == "\"side\"") {
      if (have_side || !parse_side(value, parsed.side)) {
        reason = "side must be unique and left or right";
        return false;
      }
      have_side = true;
      continue;
    }
    if (key == "\"unit\"") {
      if (have_unit || !parse_length_unit(value, parsed.coordinate_unit)) {
        reason = "unit must be unique and m, cm, or in";
        return false;
      }
      have_unit = true;
      continue;
    }
    std::size_t axis = 3;
    if (key == "\"x\"") {axis = 0;} else if (key == "\"y\"") {axis = 1;} else if (
      key == "\"z\"") {axis = 2;}
    double coordinate = 0.0;
    if (axis == 3 || have_axis[axis] || !parse_number(value, coordinate)) {
      reason = "only unique finite numeric x, y, z fields are allowed";
      return false;
    }
    if (axis == 0) {parsed.target.x = coordinate;} else if (axis == 1) {
      parsed.target.y = coordinate;
    } else {
      parsed.target.z = coordinate;
    }
    have_axis[axis] = true;
  }
  if (!have_side || !have_unit || !have_axis[0] || !have_axis[1] || !have_axis[2]) {
    reason = "side, unit, x, y, and z are required";
    return false;
  }
  out = parsed;
  return true;
}

bool StrictJson::parse_move_v3(
  std::string_view body, UnitMoveRequest & out, std::string & reason)
{
  if (body.size() < 2 || body.size() > 512) {
    reason = "JSON body length is invalid";
    return false;
  }
  body = trim(body);
  if (body.size() < 2 || body.front() != '{' || body.back() != '}') {
    reason = "JSON object required";
    return false;
  }
  body.remove_prefix(1);
  body.remove_suffix(1);
  UnitMoveRequest parsed;
  bool have_side = false;
  bool have_unit = false;
  bool have_motion_limit_scale = false;
  std::array<bool, 3> have_axis{};
  for (std::string_view field : split_fields(body)) {
    const std::size_t colon = field.find(':');
    if (colon == std::string_view::npos || field.find(':', colon + 1) != std::string_view::npos) {
      reason = "malformed JSON field";
      return false;
    }
    const std::string_view key = trim(field.substr(0, colon));
    const std::string_view value = trim(field.substr(colon + 1));
    if (key == "\"side\"") {
      if (have_side || !parse_side(value, parsed.side)) {
        reason = "side must be unique and left or right";
        return false;
      }
      have_side = true;
      continue;
    }
    if (key == "\"unit\"") {
      if (have_unit || !parse_length_unit(value, parsed.coordinate_unit)) {
        reason = "unit must be unique and m, cm, or in";
        return false;
      }
      have_unit = true;
      continue;
    }
    if (key == "\"motion_limit_scale\"") {
      if (have_motion_limit_scale || !parse_number(value, parsed.motion_limit_scale) ||
        !valid_motion_limit_scale(parsed.motion_limit_scale))
      {
        reason = "motion_limit_scale must be unique, finite, and in [0.5, 1.0]";
        return false;
      }
      have_motion_limit_scale = true;
      continue;
    }
    std::size_t axis = 3;
    if (key == "\"x\"") {axis = 0;} else if (key == "\"y\"") {axis = 1;} else if (
      key == "\"z\"") {axis = 2;}
    double coordinate = 0.0;
    if (axis == 3 || have_axis[axis] || !parse_number(value, coordinate)) {
      reason = "only unique finite numeric x, y, z fields are allowed";
      return false;
    }
    if (axis == 0) {parsed.target.x = coordinate;} else if (axis == 1) {
      parsed.target.y = coordinate;
    } else {
      parsed.target.z = coordinate;
    }
    have_axis[axis] = true;
  }
  if (!have_side || !have_unit || !have_axis[0] || !have_axis[1] || !have_axis[2] ||
    !have_motion_limit_scale)
  {
    reason = "side, unit, x, y, z, and motion_limit_scale are required";
    return false;
  }
  out = parsed;
  return true;
}

// Both arms at once: left_x/y/z and right_x/y/z, a shared unit, and one
// motion_limit_scale. Same strictness as the single-arm parsers, so unknown,
// duplicate, or non-finite fields are refused rather than ignored.
bool StrictJson::parse_move_both(
  std::string_view body, UnitMoveRequest & out, Point & right_target, std::string & reason)
{
  if (body.size() < 2 || body.size() > 512) {
    reason = "JSON body length is invalid";
    return false;
  }
  body = trim(body);
  if (body.size() < 2 || body.front() != '{' || body.back() != '}') {
    reason = "JSON object required";
    return false;
  }
  body.remove_prefix(1);
  body.remove_suffix(1);
  UnitMoveRequest parsed;
  Point right{};
  bool have_unit = false;
  bool have_motion_limit_scale = false;
  std::array<bool, 3> have_left{};
  std::array<bool, 3> have_right{};
  for (std::string_view field : split_fields(body)) {
    const std::size_t colon = field.find(':');
    if (colon == std::string_view::npos || field.find(':', colon + 1) != std::string_view::npos) {
      reason = "malformed JSON field";
      return false;
    }
    const std::string_view key = trim(field.substr(0, colon));
    const std::string_view value = trim(field.substr(colon + 1));
    if (key == "\"unit\"") {
      if (have_unit || !parse_length_unit(value, parsed.coordinate_unit)) {
        reason = "unit must be unique and m, cm, or in";
        return false;
      }
      have_unit = true;
      continue;
    }
    if (key == "\"motion_limit_scale\"") {
      if (have_motion_limit_scale || !parse_number(value, parsed.motion_limit_scale) ||
        !valid_motion_limit_scale(parsed.motion_limit_scale))
      {
        reason = "motion_limit_scale must be unique, finite, and in [0.5, 1.0]";
        return false;
      }
      have_motion_limit_scale = true;
      continue;
    }
    std::size_t axis = 3;
    bool is_left = false;
    if (key == "\"left_x\"") {axis = 0; is_left = true;} else if (key == "\"left_y\"") {
      axis = 1; is_left = true;
    } else if (key == "\"left_z\"") {axis = 2; is_left = true;} else if (
      key == "\"right_x\"") {axis = 0;} else if (key == "\"right_y\"") {axis = 1;} else if (
      key == "\"right_z\"") {axis = 2;}
    double coordinate = 0.0;
    auto & seen = is_left ? have_left : have_right;
    if (axis == 3 || seen[axis] || !parse_number(value, coordinate)) {
      reason = "only unique finite numeric left_/right_ x, y, z fields are allowed";
      return false;
    }
    if (is_left) {
      if (axis == 0) {parsed.target.x = coordinate;} else if (axis == 1) {
        parsed.target.y = coordinate;
      } else {
        parsed.target.z = coordinate;
      }
    } else {
      right[axis] = coordinate;
    }
    seen[axis] = true;
  }
  if (!have_unit || !have_motion_limit_scale ||
    !have_left[0] || !have_left[1] || !have_left[2] ||
    !have_right[0] || !have_right[1] || !have_right[2])
  {
    reason = "unit, motion_limit_scale, and all six left_/right_ coordinates are required";
    return false;
  }
  parsed.side = MoveRequest::Side::left;
  out = parsed;
  right_target = right;
  return true;
}

bool StrictJson::empty_object(std::string_view body)
{
  return trim(body) == "{}";
}

MutationPolicy::MutationPolicy(std::string authority, std::string csrf_token)
: authority_(std::move(authority)), origin_("http://" + authority_), csrf_token_(std::move(csrf_token))
{
}

bool MutationPolicy::validate(const MutationHeaders & headers, std::string & reason) const
{
  if (headers.host != authority_ || headers.origin != origin_) {
    reason = "same-origin Host and Origin are required";
    return false;
  }
  if (headers.csrf != csrf_token_) {
    reason = "invalid CSRF token";
    return false;
  }
  if (headers.content_type != "application/json") {
    reason = "Content-Type must be application/json";
    return false;
  }
  if (headers.content_length > 512) {
    reason = "request body is too large";
    return false;
  }
  return true;
}

SafeRequestPolicy::SafeRequestPolicy(std::string authority)
: authority_(std::move(authority)), origin_("http://" + authority_)
{
}

bool SafeRequestPolicy::validate_read(
  const SafeRequestHeaders & headers, std::string & reason) const
{
  if (headers.host_count != 1 || headers.host != authority_) {
    reason = "exact loopback Host is required";
    return false;
  }
  if (headers.origin_count > 1 ||
    (headers.origin_count == 1 && headers.origin != origin_))
  {
    reason = "Origin must be absent or exact same-origin";
    return false;
  }
  if (headers.sec_fetch_site_count > 1 ||
    (headers.sec_fetch_site_count == 1 && headers.sec_fetch_site != "none" &&
    headers.sec_fetch_site != "same-origin"))
  {
    reason = "Sec-Fetch-Site must be absent, none, or same-origin";
    return false;
  }
  return true;
}

bool SafeRequestPolicy::validate_mutation(
  const SafeRequestHeaders & headers, std::string & reason) const
{
  if (!validate_read(headers, reason)) {
    return false;
  }
  if (headers.origin_count != 1 || headers.origin != origin_) {
    reason = "exact same-origin Origin is required for mutation";
    return false;
  }
  return true;
}

bool CommandReservationGate::begin(std::uint64_t & token)
{
  if (active_token_ != 0U || generation_ == std::numeric_limits<std::uint64_t>::max()) {
    return false;
  }
  active_token_ = ++generation_;
  token = active_token_;
  return true;
}

bool CommandReservationGate::valid(std::uint64_t token) const
{
  return token != 0U && token == active_token_;
}

bool CommandReservationGate::consume(std::uint64_t token)
{
  if (!valid(token)) {return false;}
  active_token_ = 0U;
  return true;
}

bool CommandReservationGate::release(std::uint64_t token)
{
  return consume(token);
}

bool CommandReservationGate::cancel()
{
  const bool was_active = active_token_ != 0U;
  active_token_ = 0U;
  return was_active;
}

bool CommandReservationGate::active() const
{
  return active_token_ != 0U;
}

bool NominalPathGuard::forward(std::size_t side, const JointVector & q, oa_fk_result & result)
{
  const oa_model * model = side == 0 ? oa_model_left_v10_bimanual() :
    oa_model_right_v10_bimanual();
  return oa_fk(model, q.data(), &result) == OA_MODEL_OK;
}

bool NominalPathGuard::validate_q(std::size_t side, const JointVector & q, std::string & reason)
{
  const oa_model * model = side == 0 ? oa_model_left_v10_bimanual() :
    oa_model_right_v10_bimanual();
  for (std::size_t joint = 0; joint < q.size(); ++joint) {
    double lower = 0.0;
    double upper = 0.0;
    if (!std::isfinite(q[joint]) || oa_model_limits(model, joint, &lower, &upper) != OA_MODEL_OK ||
      q[joint] < lower || q[joint] > upper)
    {
      reason = "measured or planned joint state is outside finite model bounds";
      return false;
    }
  }
  return true;
}

bool NominalPathGuard::inverse(
  std::size_t side, const Point & target, const JointVector & seed,
  JointVector & q, std::string & reason)
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
  const oa_model * model = side == 0 ? oa_model_left_v10_bimanual() :
    oa_model_right_v10_bimanual();
  const oa_model_status status = oa_ik_position_v2(
    model, target.data(), &options, OA_IK_DIAGNOSTICS_VERSION,
    sizeof(diagnostics), &diagnostics);
  if (status != OA_MODEL_OK || diagnostics.collision_checked != 0 ||
    !std::isfinite(diagnostics.position_error_m) || diagnostics.position_error_m > 1.0e-5)
  {
    reason = "public IK could not prove this nominal waypoint reachable";
    return false;
  }
  std::copy(std::begin(diagnostics.q), std::end(diagnostics.q), q.begin());
  return validate_q(side, q, reason);
}

bool NominalPathGuard::scene_clear(
  const std::array<oa_fk_result, 2> & fk, double & clearance, std::string & reason)
{
  // The keepout model itself lives in the model library so that this pre-flight
  // guard and the real-time execution monitor cannot drift apart.
  oa_collision_scene scene{};
  if (oa_collision_scene_from_fk(&fk[0], &fk[1], &scene) != OA_MODEL_OK) {
    clearance = -std::numeric_limits<double>::infinity();
    reason = "nominal scene could not be constructed from forward kinematics";
    return false;
  }
  oa_collision_report report{};
  const oa_model_status status = oa_collision_evaluate(&scene, &report);
  clearance = report.minimum_clearance_m;
  if (status != OA_MODEL_OK) {
    reason = "nominal scene contains a non-finite coordinate";
    return false;
  }
  if (report.clear != 0u) {
    return true;
  }
  if (report.violation == OA_COLLISION_VIOLATION_ARM_ARM) {
    reason = "nominal arm-arm capsule clearance is not proven";
    return false;
  }
  reason = "nominal central pole keepout clearance is not proven for arm " +
    std::to_string(report.side) + " segment " + std::to_string(report.segment_a) +
    " (clearance " + std::to_string(report.minimum_clearance_m) + " m)";
  return false;
}

GuardResult NominalPathGuard::validate(const GuardInput & input) const
{
  GuardResult result;
  result.requested_tcp = input.request.target;
  result.minimum_nominal_clearance_m = std::numeric_limits<double>::infinity();
  std::array<oa_fk_result, 2> measured_fk{};
  for (std::size_t side = 0; side < 2; ++side) {
    if (!validate_q(side, input.measured_q[side], result.reason) ||
      !forward(side, input.measured_q[side], measured_fk[side]))
    {
      if (result.reason.empty()) {result.reason = "public FK rejected measured state";}
      return result;
    }
    result.measured_tcp[side] = origin(measured_fk[side].hand_tcp);
    result.commanded_tcp[side] = result.measured_tcp[side];
  }
  const std::size_t selected = input.request.side == MoveRequest::Side::left ? 0 : 1;
  const bool dual = input.request.dual;
  const auto finite_point = [](const Point & value) {
      return std::all_of(
        value.begin(), value.end(), [](double item) {return std::isfinite(item);});
    };
  if (dual) {
    if (!finite_point(input.request.dual_target[0]) ||
      !finite_point(input.request.dual_target[1]))
    {
      result.reason = "both target XYZ triples must contain only finite metres";
      return result;
    }
    result.commanded_tcp[0] = input.request.dual_target[0];
    result.commanded_tcp[1] = input.request.dual_target[1];
    result.requested_tcp = input.request.dual_target[0];
  } else {
    if (!finite_point(input.request.target)) {
      result.reason = "target XYZ must contain only finite metres";
      return result;
    }
    result.commanded_tcp[selected] = input.request.target;
  }
  std::array<JointVector, 2> q = input.measured_q;
  for (std::size_t sample = 0; sample < kSamples; ++sample) {
    const double u = static_cast<double>(sample) / static_cast<double>(kSamples - 1);
    if (sample > 0) {
      Point waypoint{};
      for (std::size_t axis = 0; axis < 3; ++axis) {
        waypoint[axis] = result.measured_tcp[selected][axis] +
          u * (input.request.target[axis] - result.measured_tcp[selected][axis]);
      }
      for (std::size_t side = 0; side < 2; ++side) {
        Point dual_waypoint{};
        if (dual) {
          for (std::size_t axis = 0; axis < 3; ++axis) {
            dual_waypoint[axis] = result.measured_tcp[side][axis] +
              u * (input.request.dual_target[side][axis] - result.measured_tcp[side][axis]);
          }
        }
        // Both arms sweep their own straight line together, so the sampled
        // clearance below is evaluated on the pair actually in motion rather
        // than on one arm against a stationary partner.
        const Point side_waypoint = dual ? dual_waypoint :
          (side == selected ? waypoint : result.measured_tcp[side]);
        JointVector next{};
        if (!inverse(side, side_waypoint, q[side], next, result.reason)) {
          result.failure_path_fraction = u;
          return result;
        }
        for (std::size_t joint = 0; joint < OA_DOF; ++joint) {
          if (std::abs(next[joint] - q[side][joint]) > 0.35) {
            result.reason = "nominal IK branch continuity is not proven";
            result.failure_path_fraction = u;
            return result;
          }
        }
        q[side] = next;
      }
    }
    std::array<oa_fk_result, 2> fk{};
    if (!forward(0, q[0], fk[0]) || !forward(1, q[1], fk[1])) {
      result.reason = "public FK rejected a nominal path sample";
      result.failure_path_fraction = u;
      return result;
    }
    double clearance = 0.0;
    if (!scene_clear(fk, clearance, result.reason)) {
      result.sampled_keepout_violation = true;
      result.failure_path_fraction = u;
      return result;
    }
    result.minimum_nominal_clearance_m = std::min(result.minimum_nominal_clearance_m, clearance);
  }
  result.accepted = true;
  result.achieved_fraction = 1.0;
  result.reason = "accepted by sampled nominal virtual guard; not physical certification";
  return result;
}

GuardResult NominalPathGuard::validate_or_project(const GuardInput & input) const
{
  // Best-effort projection shortens a single straight-line ray. With both arms
  // moving there are two rays and no single fraction to shorten, and stopping
  // one arm early while the other continues would change the relative geometry
  // the samples were validated against. A dual request is therefore accepted
  // exactly or rejected with its reason, never silently shortened.
  if (input.request.dual) {
    return validate(input);
  }
  GuardResult requested = validate(input);
  if (requested.accepted) {
    return requested;
  }
  if (!std::all_of(
      input.request.target.begin(), input.request.target.end(),
      [](double value) {return std::isfinite(value);}))
  {
    return requested;
  }

  const std::size_t selected = input.request.side == MoveRequest::Side::left ? 0U : 1U;
  GuardInput stationary = input;
  stationary.request.target = requested.measured_tcp[selected];
  GuardResult best = validate(stationary);
  if (!best.accepted) {
    requested.reason = "current measured scene is not safe enough for best-effort motion: " +
      best.reason;
    return requested;
  }

  const Point start = best.measured_tcp[selected];
  const Point target = input.request.target;
  const std::string requested_failure = requested.reason;
  Point delta{};
  double delta_scale = 0.0;
  for (std::size_t axis = 0; axis < delta.size(); ++axis) {
    delta[axis] = target[axis] - start[axis];
    delta_scale = std::max(delta_scale, std::abs(delta[axis]));
  }
  if (!(delta_scale > 0.0) || !std::isfinite(delta_scale)) {
    requested.reason = "finite target ray could not be normalized for best-effort motion";
    return requested;
  }
  Point ray_unit{};
  double scaled_norm_squared = 0.0;
  for (std::size_t axis = 0; axis < ray_unit.size(); ++axis) {
    ray_unit[axis] = delta[axis] / delta_scale;
    scaled_norm_squared += ray_unit[axis] * ray_unit[axis];
  }
  const double scaled_norm = std::sqrt(scaled_norm_squared);
  if (!(scaled_norm > 0.0) || !std::isfinite(scaled_norm)) {
    requested.reason = "finite target ray had no usable direction";
    return requested;
  }
  for (double & value : ray_unit) {value /= scaled_norm;}
  const double total_distance = delta_scale >
    std::numeric_limits<double>::max() / scaled_norm ?
    std::numeric_limits<double>::infinity() : delta_scale * scaled_norm;
  const double search_distance = std::isfinite(total_distance) ?
    std::min(total_distance, kProjectionMaximumRayM) : kProjectionMaximumRayM;
  std::string limiting_reason = requested_failure;
  bool observed_keepout = false;
  double keepout_barrier = search_distance;

  struct AcceptedCandidate
  {
    double distance;
    GuardResult result;
  };
  std::vector<AcceptedCandidate> accepted_candidates;
  accepted_candidates.push_back({0.0, best});
  double accepted_distance = 0.0;

  auto candidate_at = [&](double distance) {
      Point candidate{};
      for (std::size_t axis = 0; axis < candidate.size(); ++axis) {
        candidate[axis] = start[axis] + ray_unit[axis] * distance;
      }
      return candidate;
    };
  auto evaluate = [&](double distance) {
      GuardInput candidate = input;
      candidate.request.target = candidate_at(distance);
      return validate(candidate);
    };

  auto observe_keepout = [&](double candidate_distance, const GuardResult & candidate) {
      if (!candidate.sampled_keepout_violation ||
        !std::isfinite(candidate.failure_path_fraction) ||
        candidate.failure_path_fraction < 0.0 || candidate.failure_path_fraction > 1.0)
      {
        return false;
      }
      const double collision_distance = candidate_distance * candidate.failure_path_fraction;
      if (!std::isfinite(collision_distance)) {return false;}
      if (!observed_keepout || collision_distance < keepout_barrier) {
        keepout_barrier = collision_distance;
        limiting_reason = candidate.reason;
      }
      observed_keepout = true;
      return true;
    };

  // The exact request is itself a sampled path.  Preserve any observed
  // collision waypoint that lies inside the bounded physical search ray.
  if (requested.sampled_keepout_violation && std::isfinite(total_distance)) {
    const double requested_collision = total_distance * requested.failure_path_fraction;
    if (std::isfinite(requested_collision) && requested_collision <= search_distance) {
      keepout_barrier = requested_collision;
      observed_keepout = true;
      limiting_reason = requested.reason;
    }
  }

  auto select_farthest_allowed = [&]() {
      accepted_distance = 0.0;
      best = accepted_candidates.front().result;
      for (const AcceptedCandidate & candidate : accepted_candidates) {
        const bool before_barrier = !observed_keepout || candidate.distance < keepout_barrier;
        if (before_barrier && candidate.distance > accepted_distance) {
          accepted_distance = candidate.distance;
          best = candidate.result;
        }
      }
    };

  // A numerical IK failure at one endpoint spacing is not monotonic evidence,
  // so keep looking for farther endpoints.  A keepout is different: retain the
  // actual failing waypoint distance (candidate endpoint times path fraction)
  // as a hard boundary.  Previously accepted endpoints beyond a newly observed
  // boundary are discarded.  Every retained candidate independently validates
  // all 17 waypoints from the same measured state.
  for (std::size_t step = 1; step <= kProjectionSearchSteps; ++step) {
    const double initial_limit = observed_keepout ? keepout_barrier : search_distance;
    const double distance = initial_limit * static_cast<double>(step) /
      static_cast<double>(kProjectionSearchSteps);
    if (observed_keepout && !(distance < keepout_barrier)) {break;}
    GuardResult candidate = evaluate(distance);
    if (candidate.accepted) {
      accepted_candidates.push_back({distance, std::move(candidate)});
      continue;
    }
    if (observe_keepout(distance, candidate)) {break;}
    if (!observed_keepout) {limiting_reason = candidate.reason;}
  }
  select_farthest_allowed();

  // Refine by scanning each interval, not by binary-searching IK failures.
  // This permits a farther numerical IK success after a nearer failure while
  // still honoring every observed keepout as an irreversible upper boundary.
  for (std::size_t round = 0; round < kProjectionRefinementRounds; ++round) {
    const double upper = observed_keepout ? keepout_barrier : search_distance;
    const double span = upper - accepted_distance;
    if (!(span > kMinimumProjectedMotionM / 16.0)) {break;}
    const double lower = accepted_distance;
    const std::size_t sample_count = kProjectionRefinementSamples * (round + 1U);
    for (std::size_t sample = 1; sample <= sample_count; ++sample) {
      const double distance = lower + span * static_cast<double>(sample) /
        static_cast<double>(sample_count);
      if (observed_keepout && !(distance < keepout_barrier)) {break;}
      GuardResult candidate = evaluate(distance);
      if (candidate.accepted) {
        accepted_candidates.push_back({distance, std::move(candidate)});
        continue;
      }
      if (observe_keepout(distance, candidate)) {break;}
      if (!observed_keepout) {limiting_reason = candidate.reason;}
    }
    select_farthest_allowed();
  }

  if (!(accepted_distance >= kMinimumProjectedMotionM)) {
    requested.reason = "requested target was unreachable or unsafe, and no sampled safe "
      "straight-line progress of at least 0.001 m could be proven; no motion submitted; "
      "limiter: " + limiting_reason;
    requested.limiting_reason = limiting_reason;
    requested.limited_by_keepout = observed_keepout;
    requested.keepout_barrier_distance_m = observed_keepout ? keepout_barrier : 0.0;
    return requested;
  }

  const double accepted_fraction = std::clamp(
    std::isfinite(total_distance) ? accepted_distance / total_distance :
    (accepted_distance / delta_scale) / scaled_norm, 0.0, 1.0);
  best.target_projected = true;
  best.limited_by_keepout = observed_keepout;
  best.requested_tcp = target;
  best.achieved_fraction = accepted_fraction;
  best.limiting_reason = limiting_reason;
  best.keepout_barrier_distance_m = observed_keepout ? keepout_barrier : 0.0;
  best.reason = "requested target was unreachable or unsafe; using farthest sampled validated "
    "straight-line prefix (" + json_number(accepted_fraction * 100.0) +
    "%); limiter: " + limiting_reason;
  return best;
}

bool fresh_at_use(
  const FreshnessEvidence & evidence, std::int64_t now_time_ns,
  std::int64_t now_steady_ns, std::int64_t maximum_age_ns)
{
  if (evidence.producer_time_ns <= 0 || evidence.receipt_steady_ns <= 0 ||
    now_time_ns < evidence.producer_time_ns || now_steady_ns < evidence.receipt_steady_ns ||
    maximum_age_ns <= 0)
  {
    return false;
  }
  return now_time_ns - evidence.producer_time_ns <= maximum_age_ns &&
         now_steady_ns - evidence.receipt_steady_ns <= maximum_age_ns;
}

bool guard_handoff_valid(
  const GuardInput & guarded, const GuardHandoffEvidence & current,
  std::int64_t now_time_ns, std::int64_t now_steady_ns,
  std::int64_t state_maximum_age_ns, std::int64_t diagnostic_maximum_age_ns)
{
  auto monotonic_generation = [](
      std::uint64_t guarded_sequence, const FreshnessEvidence & guarded_freshness,
      std::uint64_t current_sequence, const FreshnessEvidence & current_freshness)
    {
      if (guarded_sequence == 0 || current_sequence < guarded_sequence) {return false;}
      if (current_sequence == guarded_sequence) {
        return current_freshness.producer_time_ns == guarded_freshness.producer_time_ns &&
               current_freshness.receipt_steady_ns == guarded_freshness.receipt_steady_ns;
      }
      return current_freshness.producer_time_ns > guarded_freshness.producer_time_ns &&
             current_freshness.receipt_steady_ns > guarded_freshness.receipt_steady_ns;
    };
  if (!current.have_state || !current.diagnostic_valid ||
    !fresh_at_use(
      current.state_freshness, now_time_ns, now_steady_ns, state_maximum_age_ns) ||
    !fresh_at_use(
      current.diagnostic_freshness, now_time_ns, now_steady_ns,
      diagnostic_maximum_age_ns) ||
    !monotonic_generation(
      guarded.state_sequence, guarded.state_freshness,
      current.state_sequence, current.state_freshness) ||
    !monotonic_generation(
      guarded.diagnostic_sequence, guarded.diagnostic_freshness,
      current.diagnostic_sequence, current.diagnostic_freshness))
  {
    return false;
  }
  for (std::size_t side = 0; side < current.measured_q.size(); ++side) {
    const oa_model * model = side == 0 ? oa_model_left_v10_bimanual() :
      oa_model_right_v10_bimanual();
    for (std::size_t joint = 0; joint < current.measured_q[side].size(); ++joint) {
      const double guarded_value = guarded.measured_q[side][joint];
      const double current_value = current.measured_q[side][joint];
      double lower = 0.0;
      double upper = 0.0;
      if (!std::isfinite(guarded_value) || !std::isfinite(current_value) ||
        oa_model_limits(model, joint, &lower, &upper) != OA_MODEL_OK ||
        current_value < lower || current_value > upper ||
        std::abs(current_value - guarded_value) > kGuardJointEquivalenceTolerance)
      {
        return false;
      }
    }
  }
  return true;
}

bool normalise_move_to_metres(
  const UnitMoveRequest & input, MoveRequest & output, std::string & reason)
{
  oa_vec3d metres{};
  const oa_units_status status = oa_vec3d_convert(
    &input.target, input.coordinate_unit, OA_LENGTH_UNIT_METRES, &metres);
  if (status != OA_UNITS_OK) {
    reason = status == OA_UNITS_ENONFINITE ?
      "target XYZ must contain only finite coordinates" :
      (status == OA_UNITS_EOVERFLOW ? "target XYZ conversion overflowed" :
      "target XYZ coordinate unit is invalid");
    return false;
  }
  MoveRequest converted;
  converted.side = input.side;
  converted.target = {metres.x, metres.y, metres.z};
  converted.motion_limit_scale = input.motion_limit_scale;
  output = converted;
  return true;
}

bool map_canonical_joint_state(
  const std::vector<std::string> & names, const std::vector<double> & positions,
  std::array<JointVector, 2> & output)
{
  constexpr std::size_t kJointCount = OA_DOF * 2U;
  if (names.size() != kJointCount || positions.size() != kJointCount) {
    return false;
  }
  std::array<bool, kJointCount> seen{};
  std::array<JointVector, 2> mapped{};
  for (std::size_t input = 0; input < kJointCount; ++input) {
    if (!std::isfinite(positions[input])) {
      return false;
    }
    std::size_t canonical = kJointCount;
    for (std::size_t side = 0; side < 2U && canonical == kJointCount; ++side) {
      const oa_model * model = side == 0U ?
        oa_model_left_v10_bimanual() : oa_model_right_v10_bimanual();
      for (std::size_t joint = 0; joint < OA_DOF; ++joint) {
        if (names[input] == oa_model_joint_name(model, joint)) {
          canonical = side * OA_DOF + joint;
          break;
        }
      }
    }
    if (canonical == kJointCount || seen[canonical]) {
      return false;
    }
    seen[canonical] = true;
    mapped[canonical / OA_DOF][canonical % OA_DOF] = positions[input];
  }
  if (!std::all_of(seen.begin(), seen.end(), [](bool value) {return value;})) {
    return false;
  }
  output = mapped;
  return true;
}

const NominalTargetTable & nominal_targets(MoveRequest::Side side)
{
  static constexpr NominalTargetTable left{{
    {"near_low", "Near low", {0.150000, 0.220000, 0.150000}},
    {"outer_low", "Outer low", {0.150000, 0.400000, 0.150000}},
    {"near_mid", "Near mid", {0.150000, 0.220000, 0.300000}},
    {"outer_mid", "Outer mid", {0.150000, 0.400000, 0.300000}},
    {"forward_mid", "Forward mid", {0.300000, 0.220000, 0.300000}},
    {"forward_outer", "Forward outer", {0.300000, 0.500000, 0.300000}},
    {"near_max_forward", "Near-max forward", {0.480000, 0.170000, 0.350000}},
    {"outer_high", "Outer high", {0.250000, 0.580000, 0.450000}},
    {"high_far", "High far", {0.280000, 0.670000, 0.520000}},
  }};
  static constexpr NominalTargetTable right{{
    {"near_low", "Near low", {0.150000, -0.220000, 0.150000}},
    {"outer_low", "Outer low", {0.150000, -0.400000, 0.150000}},
    {"near_mid", "Near mid", {0.150000, -0.220000, 0.300000}},
    {"outer_mid", "Outer mid", {0.150000, -0.400000, 0.300000}},
    {"forward_mid", "Forward mid", {0.300000, -0.220000, 0.300000}},
    {"forward_outer", "Forward outer", {0.300000, -0.500000, 0.300000}},
    {"near_max_forward", "Near-max forward", {0.480000, -0.170000, 0.350000}},
    {"outer_high", "Outer high", {0.250000, -0.580000, 0.450000}},
    {"high_far", "High far", {0.280000, -0.670000, 0.520000}},
  }};
  return side == MoveRequest::Side::left ? left : right;
}

NominalTestSamples nominal_test_samples(MoveRequest::Side side)
{
  const NominalTargetTable & targets = nominal_targets(side);
  return {targets[0].point, targets[1].point};
}

std::string json_number(double value)
{
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return output.str();
}

std::string viewer_state_json(const ViewerSnapshot & snapshot, std::int64_t now_steady_ns)
{
  static constexpr std::array<std::string_view, OA_DOF * 2> kJointOrder{{
    "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
    "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
    "openarm_left_joint7", "openarm_right_joint1", "openarm_right_joint2",
    "openarm_right_joint3", "openarm_right_joint4", "openarm_right_joint5",
    "openarm_right_joint6", "openarm_right_joint7",
  }};
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << "{\"schema\":1,\"have_state\":" << (snapshot.have_state ? "true" : "false") <<
    ",\"fresh\":" << (snapshot.fresh ? "true" : "false") << ",\"sequence\":\"" <<
    snapshot.sequence << "\",\"producer_time_ns\":\"" << snapshot.producer_time_ns <<
    "\",\"receipt_age_ms\":";
  if (!snapshot.have_state || snapshot.receipt_steady_ns <= 0 ||
    now_steady_ns < snapshot.receipt_steady_ns)
  {
    output << "null";
  } else {
    output << json_number(static_cast<double>(now_steady_ns - snapshot.receipt_steady_ns) / 1.0e6);
  }
  output << ",\"joint_order\":[";
  for (std::size_t index = 0; index < kJointOrder.size(); ++index) {
    if (index != 0) {output << ',';}
    output << '\"' << kJointOrder[index] << '\"';
  }
  output << ']';
  if (snapshot.have_state) {
    output << ",\"position_rad\":[";
    for (std::size_t index = 0; index < snapshot.position_rad.size(); ++index) {
      if (index != 0) {output << ',';}
      output << json_number(snapshot.position_rad[index]);
    }
    output << ']';
  }
  output << '}';
  return output.str();
}

std::string portal_state_json(
  bool state_fresh, bool command_active, const std::array<Point, 2> & tcp,
  std::string_view summary, std::string_view command)
{
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << "{\"coordinate_unit\":\"m\",\"state_fresh\":" <<
    (state_fresh ? "true" : "false") << ",\"command_active\":" <<
    (command_active ? "true" : "false") << ",\"left\":[" <<
    json_number(tcp[0][0]) << ',' << json_number(tcp[0][1]) << ',' <<
    json_number(tcp[0][2]) << "],\"right\":[" << json_number(tcp[1][0]) << ',' <<
    json_number(tcp[1][1]) << ',' << json_number(tcp[1][2]) << "],\"summary\":\"" <<
    json_escape(summary) << "\",\"command\":\"" << json_escape(command) << "\"}";
  return output.str();
}

double finite_cylinder_capsule_clearance(
  const Point & a, const Point & b, double cylinder_radius,
  double cylinder_bottom, double cylinder_top, double capsule_radius)
{
  // Delegates to the canonical keepout geometry in the model library. The
  // pre-flight guard and the real-time execution monitor must gate on bit-identical
  // clearances, so there is exactly one implementation.
  return oa_collision_finite_cylinder_capsule_clearance(
    a.data(), b.data(), cylinder_radius, cylinder_bottom, cylinder_top, capsule_radius);
}

std::string json_escape(std::string_view value)
{
  std::string output;
  output.reserve(value.size());
  for (const char character : value) {
    switch (character) {
      case '"': output += "\\\""; break;
      case '\\': output += "\\\\"; break;
      case '\b': output += "\\b"; break;
      case '\f': output += "\\f"; break;
      case '\n': output += "\\n"; break;
      case '\r': output += "\\r"; break;
      case '\t': output += "\\t"; break;
      default:
        if (static_cast<unsigned char>(character) >= 0x20) {output += character;}
    }
  }
  return output;
}

}  // namespace openarm_ik_ros::portal
