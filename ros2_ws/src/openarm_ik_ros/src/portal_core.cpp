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
#include <unistd.h>

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

bool process_executable_matches(std::int64_t pid, std::string_view expected_path)
{
  if (pid <= 1 || expected_path.empty() || expected_path.front() != '/' ||
    expected_path.size() >= 4096)
  {
    return false;
  }
  std::array<char, 4096> resolved{};
  const std::string link = "/proc/" + std::to_string(pid) + "/exe";
  const ssize_t length = readlink(link.c_str(), resolved.data(), resolved.size() - 1);
  return length > 0 && static_cast<std::size_t>(length) == expected_path.size() &&
         std::equal(expected_path.begin(), expected_path.end(), resolved.begin());
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

bool xcomposite_version_supported(int major, int minor)
{
  return major > 0 || (major == 0 && minor >= 2);
}

NominalTestSamples nominal_test_samples(MoveRequest::Side side)
{
  if (side == MoveRequest::Side::left) {
    return {{0.019973, 0.143469, 0.096000}, {0.029973, 0.143469, 0.106000}};
  }
  return {{0.020081, -0.143527, 0.096000}, {0.030081, -0.143527, 0.106000}};
}

bool truecolor_masks_valid(const TrueColorMasks & masks)
{
  const auto contiguous = [](std::uint64_t mask) {
      if (mask == 0) {return false;}
      while ((mask & 1U) == 0U) {mask >>= 1U;}
      return (mask & (mask + 1U)) == 0U;
    };
  return contiguous(masks.red) && contiguous(masks.green) && contiguous(masks.blue) &&
         (masks.red & masks.green) == 0U && (masks.red & masks.blue) == 0U &&
         (masks.green & masks.blue) == 0U;
}

std::array<unsigned char, 3> truecolor_pixel_rgb(
  std::uint64_t pixel, const TrueColorMasks & masks)
{
  auto channel = [pixel](std::uint64_t mask) {
      unsigned int shift = 0;
      while (((mask >> shift) & 1U) == 0U) {++shift;}
      const std::uint64_t maximum = mask >> shift;
      return static_cast<unsigned char>(((pixel & mask) >> shift) * 255U / maximum);
    };
  if (!truecolor_masks_valid(masks)) {return {0, 0, 0};}
  return {channel(masks.red), channel(masks.green), channel(masks.blue)};
}

bool rgb_frame_has_nonblack_pixel(const std::vector<unsigned char> & rgb)
{
  return rgb.size() >= 3 && rgb.size() % 3 == 0 &&
         std::any_of(rgb.begin(), rgb.end(), [](unsigned char value) {return value != 0;});
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
