// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/portal_core.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <fstream>
#include <limits>
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
constexpr double kBodyRadius = 0.115;
constexpr double kBodyTop = 0.775;
constexpr std::size_t kSamples = 17;

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

double radial_segment_distance(const Point & a, const Point & b)
{
  const Point d = subtract(b, a);
  double minimum_t = 0.0;
  double maximum_t = 1.0;
  if (std::abs(d[2]) <= 1.0e-18) {
    if (a[2] < 0.0 || a[2] > kBodyTop) {
      return std::numeric_limits<double>::infinity();
    }
  } else {
    const double at_bottom = (0.0 - a[2]) / d[2];
    const double at_top = (kBodyTop - a[2]) / d[2];
    minimum_t = std::max(0.0, std::min(at_bottom, at_top));
    maximum_t = std::min(1.0, std::max(at_bottom, at_top));
    if (minimum_t > maximum_t) {
      return std::numeric_limits<double>::infinity();
    }
  }
  const double denominator = d[0] * d[0] + d[1] * d[1];
  const double t = denominator > 1.0e-18 ?
    std::clamp(
      -(a[0] * d[0] + a[1] * d[1]) / denominator, minimum_t, maximum_t) : minimum_t;
  const Point p = add_scaled(a, d, t);
  return std::hypot(p[0], p[1]);
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
  for (std::size_t left = 1; left < 7; ++left) {
    for (std::size_t right = 1; right < 7; ++right) {
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
    for (std::size_t segment = 2; segment < 7; ++segment) {
      const double radius = segment == 6 ? kToolRadius : kArmRadius;
      const double value = radial_segment_distance(
        points[side][segment], points[side][segment + 1]) - kBodyRadius - radius;
      clearance = std::min(clearance, value);
      if (std::isnan(value) || value < kRequiredClearance) {
        reason = "nominal central pole/body keepout clearance is not proven for arm " +
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

bool process_identity_matches(std::int64_t pid, std::uint64_t expected_start_ticks)
{
  if (pid <= 1 || expected_start_ticks == 0) {
    return false;
  }
  std::ifstream stream("/proc/" + std::to_string(pid) + "/stat");
  std::string line;
  if (!stream || !std::getline(stream, line)) {
    return false;
  }
  const std::size_t close = line.rfind(')');
  if (close == std::string::npos || close + 2 >= line.size()) {
    return false;
  }
  std::istringstream fields(line.substr(close + 2));
  std::string field;
  for (std::size_t index = 3; index <= 22; ++index) {
    if (!(fields >> field)) {
      return false;
    }
    if (index == 22) {
      std::uint64_t value = 0;
      const auto parsed = std::from_chars(field.data(), field.data() + field.size(), value);
      return parsed.ec == std::errc{} && parsed.ptr == field.data() + field.size() &&
             value == expected_start_ticks;
    }
  }
  return false;
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
