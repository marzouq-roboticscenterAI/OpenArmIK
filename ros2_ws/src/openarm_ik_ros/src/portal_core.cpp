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
constexpr double kArmRadius = 0.050;
constexpr double kToolRadius = 0.075;
constexpr double kRequiredClearance = 0.025;
// The canonical body mesh contains a 60 x 60 mm central shaft.  Its
// circumscribed cylinder conservatively covers the square; the former 115 mm
// radius projected unrelated base/upper-mount extents through the workspace.
constexpr double kPoleRadius = 0.04242640687119285;
constexpr double kPoleBottom = 0.008;
constexpr double kPoleTop = 0.758;
constexpr std::size_t kSamples = 17;
// DaMiao position feedback has a 25/65535 = 3.8148e-4 rad code step.  Keep
// handoff equivalence far below one encoder code while covering floating-point
// publication/serialization noise; an actual adjacent-code change still fails.
constexpr double kGuardJointEquivalenceTolerance = 1.0e-6;

double dot(const Point & a, const Point & b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Point subtract(const Point & a, const Point & b)
{
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Point add_scaled(const Point & a, const Point & b, double scale)
{
  return {a[0] + b[0] * scale, a[1] + b[1] * scale, a[2] + b[2] * scale};
}

double norm(const Point & a)
{
  return std::sqrt(dot(a, a));
}

Point origin(const oa_transform & transform)
{
  return {transform.m[3], transform.m[7], transform.m[11]};
}

double segment_distance(const Point & p1, const Point & q1, const Point & p2, const Point & q2)
{
  const Point d1 = subtract(q1, p1);
  const Point d2 = subtract(q2, p2);
  const Point r = subtract(p1, p2);
  const double a = dot(d1, d1);
  const double e = dot(d2, d2);
  const double f = dot(d2, r);
  double s = 0.0;
  double t = 0.0;
  if (a <= 1.0e-18 && e <= 1.0e-18) {
    return norm(r);
  }
  if (a <= 1.0e-18) {
    t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = dot(d1, r);
    if (e <= 1.0e-18) {
      s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = dot(d1, d2);
      const double denominator = a * e - b * b;
      if (denominator > 1.0e-18) {
        s = std::clamp((b * f - c * e) / denominator, 0.0, 1.0);
      }
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }
  return norm(subtract(add_scaled(p1, d1, s), add_scaled(p2, d2, t)));
}

double point_cylinder_distance_squared(
  const Point & point, double radius, double bottom, double top)
{
  const double radial_gap = std::max(0.0, std::hypot(point[0], point[1]) - radius);
  const double axial_gap = point[2] < bottom ? bottom - point[2] :
    (point[2] > top ? point[2] - top : 0.0);
  return radial_gap * radial_gap + axial_gap * axial_gap;
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
      out.side = value == "\"left\"" ? MoveRequest::Side::left : MoveRequest::Side::right;
      continue;
    }
    std::size_t axis = 3;
    if (key == "\"x\"") {axis = 0;} else if (key == "\"y\"") {axis = 1;} else if (
      key == "\"z\"") {axis = 2;}
    if (axis == 3 || have_axis[axis] || !parse_number(value, out.target[axis])) {
      reason = "only unique finite numeric x, y, z fields are allowed";
      return false;
    }
    have_axis[axis] = true;
  }
  if (!have_side || !have_axis[0] || !have_axis[1] || !have_axis[2]) {
    reason = "side, x, y, and z are required";
    return false;
  }
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
  std::array<std::array<Point, 8>, 2> points{};
  for (std::size_t side = 0; side < 2; ++side) {
    for (std::size_t joint = 0; joint < OA_DOF; ++joint) {
      points[side][joint] = origin(fk[side].joint_pre[joint]);
    }
    points[side][7] = origin(fk[side].hand_tcp);
  }
  clearance = std::numeric_limits<double>::infinity();
  for (std::size_t left = 0; left < 7; ++left) {
    for (std::size_t right = 0; right < 7; ++right) {
      const double radii = (left == 6 ? kToolRadius : kArmRadius) +
        (right == 6 ? kToolRadius : kArmRadius);
      const double value = segment_distance(
        points[0][left], points[0][left + 1], points[1][right], points[1][right + 1]) - radii;
      clearance = std::min(clearance, value);
      if (std::isnan(value) || value < kRequiredClearance) {
        reason = "nominal arm-arm capsule clearance is not proven";
        return false;
      }
    }
  }
  for (std::size_t side = 0; side < 2; ++side) {
    for (std::size_t segment = 0; segment < 7; ++segment) {
      // Canonical link1 begins at J1 and lies wholly outward along the J1
      // radial axis. Joint1 only rolls its cross-section about that axis, so
      // no link1 vertex lies radially inward of this centerline. Using the
      // generic isotropic radius here would fabricate inward mount volume;
      // the centerline is its conservative shaft-facing envelope.
      const double radius = segment == 0 ? 0.0 : (segment == 6 ? kToolRadius : kArmRadius);
      const double value = finite_cylinder_capsule_clearance(
        points[side][segment], points[side][segment + 1],
        kPoleRadius, kPoleBottom, kPoleTop, radius);
      clearance = std::min(clearance, value);
      if (std::isnan(value) || value < kRequiredClearance) {
        reason = "nominal central pole keepout clearance is not proven for arm " +
          std::to_string(side) + " segment " + std::to_string(segment) +
          " (clearance " + std::to_string(value) + " m)";
        return false;
      }
    }
  }
  return true;
}

GuardResult NominalPathGuard::validate(const GuardInput & input) const
{
  GuardResult result;
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
  if (!std::all_of(
      input.request.target.begin(), input.request.target.end(),
      [](double value) {return std::isfinite(value);}))
  {
    result.reason = "target XYZ must contain only finite metres";
    return result;
  }
  result.commanded_tcp[selected] = input.request.target;
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
        const Point side_waypoint = side == selected ? waypoint : result.measured_tcp[side];
        JointVector next{};
        if (!inverse(side, side_waypoint, q[side], next, result.reason)) {
          return result;
        }
        for (std::size_t joint = 0; joint < OA_DOF; ++joint) {
          if (std::abs(next[joint] - q[side][joint]) > 0.35) {
            result.reason = "nominal IK branch continuity is not proven";
            return result;
          }
        }
        q[side] = next;
      }
    }
    std::array<oa_fk_result, 2> fk{};
    if (!forward(0, q[0], fk[0]) || !forward(1, q[1], fk[1])) {
      result.reason = "public FK rejected a nominal path sample";
      return result;
    }
    double clearance = 0.0;
    if (!scene_clear(fk, clearance, result.reason)) {
      return result;
    }
    result.minimum_nominal_clearance_m = std::min(result.minimum_nominal_clearance_m, clearance);
  }
  result.accepted = true;
  result.reason = "accepted by sampled nominal virtual guard; not physical certification";
  return result;
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
    {"small", "Small forward/up", {0.019973, 0.143469, 0.096000}},
    {"medium", "Medium forward/up", {0.029973, 0.143469, 0.106000}},
    {"large", "Large forward/up", {0.039973, 0.143469, 0.116000}},
    {"forward_low", "Low reach", {0.039973, 0.153469, 0.086000}},
    {"forward_mid", "Mid reach", {0.049973, 0.153469, 0.096000}},
    {"forward_high", "Far reach", {0.059973, 0.153469, 0.106000}},
    {"high", "High", {0.029973, 0.153469, 0.136000}},
    {"mid_high", "High near", {0.019973, 0.153469, 0.126000}},
    {"far_high", "High far", {0.049973, 0.153469, 0.136000}},
  }};
  static constexpr NominalTargetTable right{{
    {"small", "Small forward/up", {0.020081, -0.143527, 0.096000}},
    {"medium", "Medium forward/up", {0.030081, -0.143527, 0.106000}},
    {"large", "Large forward/up", {0.040081, -0.143527, 0.116000}},
    {"forward_low", "Low reach", {0.040081, -0.153527, 0.086000}},
    {"forward_mid", "Mid reach", {0.050081, -0.153527, 0.096000}},
    {"forward_high", "Far reach", {0.060081, -0.153527, 0.106000}},
    {"high", "High", {0.030081, -0.153527, 0.136000}},
    {"mid_high", "High near", {0.020081, -0.153527, 0.126000}},
    {"far_high", "High far", {0.050081, -0.153527, 0.136000}},
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
  if (!std::all_of(a.begin(), a.end(), [](double value) {return std::isfinite(value);}) ||
    !std::all_of(b.begin(), b.end(), [](double value) {return std::isfinite(value);}) ||
    !std::isfinite(cylinder_radius) || !std::isfinite(cylinder_bottom) ||
    !std::isfinite(cylinder_top) || !std::isfinite(capsule_radius) ||
    cylinder_radius < 0.0 || cylinder_bottom > cylinder_top || capsule_radius < 0.0)
  {
    return -std::numeric_limits<double>::infinity();
  }
  const Point direction = subtract(b, a);
  auto distance_squared = [&](double amount) {
      return point_cylinder_distance_squared(
        add_scaled(a, direction, amount), cylinder_radius, cylinder_bottom, cylinder_top);
    };
  // Squared distance to a closed convex set is convex along a segment. Ternary
  // minimization therefore covers the cylindrical side, caps, and rim without
  // axial clipping holes. Bias the result downward for fail-closed rounding.
  double low = 0.0;
  double high = 1.0;
  for (std::size_t iteration = 0; iteration < 96; ++iteration) {
    const double first = (2.0 * low + high) / 3.0;
    const double second = (low + 2.0 * high) / 3.0;
    if (distance_squared(first) <= distance_squared(second)) {
      high = second;
    } else {
      low = first;
    }
  }
  const double minimum = std::min({
      distance_squared(0.0), distance_squared(1.0),
      distance_squared((low + high) / 2.0)});
  return std::max(0.0, std::sqrt(minimum) - 1.0e-9) - capsule_radius;
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
