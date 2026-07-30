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

struct UnitMoveRequest
{
  MoveRequest::Side side{MoveRequest::Side::left};
  oa_length_unit coordinate_unit{OA_LENGTH_UNIT_METRES};
  oa_vec3d target{};
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

struct SafeRequestHeaders
{
  std::string_view host;
  std::string_view origin;
  std::string_view sec_fetch_site;
  std::size_t host_count{0};
  std::size_t origin_count{0};
  std::size_t sec_fetch_site_count{0};
};

struct ViewerSnapshot
{
  std::array<double, OA_DOF * 2> position_rad{};
  std::uint64_t sequence{0};
  std::int64_t producer_time_ns{0};
  std::int64_t receipt_steady_ns{0};
  bool have_state{false};
  bool fresh{false};
};

struct NominalTarget
{
  std::string_view id;
  std::string_view label;
  Point point{};
};

using NominalTargetTable = std::array<NominalTarget, 9>;

struct NominalTestSamples
{
  Point small_forward_up{};
  Point medium_forward_up{};
};

class StrictJson
{
public:
  static bool parse_move(std::string_view body, MoveRequest & out, std::string & reason);
  static bool parse_move_v2(std::string_view body, UnitMoveRequest & out, std::string & reason);
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

class SafeRequestPolicy
{
public:
  explicit SafeRequestPolicy(std::string authority);
  bool validate_read(const SafeRequestHeaders & headers, std::string & reason) const;
  bool validate_mutation(const SafeRequestHeaders & headers, std::string & reason) const;

private:
  std::string authority_;
  std::string origin_;
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

bool fresh_at_use(
  const FreshnessEvidence & evidence, std::int64_t now_time_ns,
  std::int64_t now_steady_ns, std::int64_t maximum_age_ns);
bool guard_handoff_valid(
  const GuardInput & guarded, const GuardHandoffEvidence & current,
  std::int64_t now_time_ns, std::int64_t now_steady_ns,
  std::int64_t state_maximum_age_ns, std::int64_t diagnostic_maximum_age_ns);
bool normalise_move_to_metres(
  const UnitMoveRequest & input, MoveRequest & output, std::string & reason);
bool map_canonical_joint_state(
  const std::vector<std::string> & names, const std::vector<double> & positions,
  std::array<JointVector, 2> & output);
NominalTestSamples nominal_test_samples(MoveRequest::Side side);
const NominalTargetTable & nominal_targets(MoveRequest::Side side);
std::string json_number(double value);
std::string viewer_state_json(const ViewerSnapshot & snapshot, std::int64_t now_steady_ns);
std::string portal_state_json(
  bool state_fresh, bool command_active, const std::array<Point, 2> & tcp,
  std::string_view summary, std::string_view command);
std::string portal_page(std::string_view csrf);
double finite_cylinder_capsule_clearance(
  const Point & a, const Point & b, double cylinder_radius,
  double cylinder_bottom, double cylinder_top, double capsule_radius);
std::string json_escape(std::string_view value);

}  // namespace openarm_ik_ros::portal

#endif  // OPENARM_IK_ROS__PORTAL_CORE_HPP_
