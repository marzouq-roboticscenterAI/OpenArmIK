// SPDX-License-Identifier: Apache-2.0
#ifndef OPENARM_IK_ROS__PORTAL_CORE_HPP_
#define OPENARM_IK_ROS__PORTAL_CORE_HPP_

#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1
#include <openarm_model.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace openarm_ik_ros::portal
{

using JointVector = std::array<double, OA_DOF>;
using Point = std::array<double, 3>;

struct FreshnessEvidence
{
  std::int64_t producer_time_ns{0};
  std::int64_t receipt_steady_ns{0};
};

struct MoveRequest
{
  enum class Side {left, right};
  Side side{Side::left};
  Point target{};
};

struct GuardInput
{
  std::array<JointVector, 2> measured_q{};
  std::uint64_t state_sequence{0};
  std::uint64_t diagnostic_sequence{0};
  FreshnessEvidence state_freshness{};
  FreshnessEvidence diagnostic_freshness{};
  MoveRequest request{};
};

struct GuardHandoffEvidence
{
  std::array<JointVector, 2> measured_q{};
  std::uint64_t state_sequence{0};
  std::uint64_t diagnostic_sequence{0};
  FreshnessEvidence state_freshness{};
  FreshnessEvidence diagnostic_freshness{};
  bool have_state{false};
  bool diagnostic_valid{false};
};

struct GuardResult
{
  bool accepted{false};
  std::string reason;
  std::array<Point, 2> measured_tcp{};
  std::array<Point, 2> commanded_tcp{};
  double minimum_nominal_clearance_m{0.0};
};

struct MutationHeaders
{
  std::string_view host;
  std::string_view origin;
  std::string_view csrf;
  std::string_view content_type;
  std::size_t content_length{0};
};

struct NominalTestSamples
{
  Point small_forward_up{};
  Point medium_forward_up{};
};

struct TrueColorMasks
{
  std::uint64_t red{0};
  std::uint64_t green{0};
  std::uint64_t blue{0};
};

class StrictJson
{
public:
  static bool parse_move(std::string_view body, MoveRequest & out, std::string & reason);
  static bool empty_object(std::string_view body);
};

class MutationPolicy
{
public:
  MutationPolicy(std::string authority, std::string csrf_token);
  bool validate(const MutationHeaders & headers, std::string & reason) const;

private:
  std::string authority_;
  std::string origin_;
  std::string csrf_token_;
};

class NominalPathGuard
{
public:
  GuardResult validate(const GuardInput & input) const;

private:
  static bool forward(std::size_t side, const JointVector & q, oa_fk_result & result);
  static bool inverse(
    std::size_t side, const Point & target, const JointVector & seed,
    JointVector & q, std::string & reason);
  static bool validate_q(std::size_t side, const JointVector & q, std::string & reason);
  static bool scene_clear(
    const std::array<oa_fk_result, 2> & fk, double & clearance, std::string & reason);
};

bool process_identity_matches(std::int64_t pid, std::uint64_t expected_start_ticks);
bool process_executable_matches(std::int64_t pid, std::string_view expected_path);
bool fresh_at_use(
  const FreshnessEvidence & evidence, std::int64_t now_time_ns,
  std::int64_t now_steady_ns, std::int64_t maximum_age_ns);
bool guard_handoff_valid(
  const GuardInput & guarded, const GuardHandoffEvidence & current,
  std::int64_t now_time_ns, std::int64_t now_steady_ns,
  std::int64_t state_maximum_age_ns, std::int64_t diagnostic_maximum_age_ns);
bool xcomposite_version_supported(int major, int minor);
NominalTestSamples nominal_test_samples(MoveRequest::Side side);
bool truecolor_masks_valid(const TrueColorMasks & masks);
std::array<unsigned char, 3> truecolor_pixel_rgb(
  std::uint64_t pixel, const TrueColorMasks & masks);
bool rgb_frame_has_nonblack_pixel(const std::vector<unsigned char> & rgb);
double finite_cylinder_capsule_clearance(
  const Point & a, const Point & b, double cylinder_radius,
  double cylinder_bottom, double cylinder_top, double capsule_radius);
std::string json_escape(std::string_view value);

}  // namespace openarm_ik_ros::portal

#endif  // OPENARM_IK_ROS__PORTAL_CORE_HPP_
