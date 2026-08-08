// SPDX-License-Identifier: Apache-2.0
#ifndef OPENARM_IK_ROS__PORTAL_CORE_HPP_
#define OPENARM_IK_ROS__PORTAL_CORE_HPP_

#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1
#include <openarm_collision.h>
#include <openarm_model.h>
#include <openarm_route.h>

#include "openarm_ik_ros/motion_profile.hpp"

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
  double motion_limit_scale{kLegacyMotionLimitScale};
  // Both arms move to their own targets at the same time. `side` and `target`
  // are unused in this mode; dual_target is indexed 0 left, 1 right.
  bool dual{false};
  std::array<Point, 2> dual_target{};
};

struct UnitMoveRequest
{
  MoveRequest::Side side{MoveRequest::Side::left};
  oa_length_unit coordinate_unit{OA_LENGTH_UNIT_METRES};
  oa_vec3d target{};
  double motion_limit_scale{kLegacyMotionLimitScale};
};

struct GuardInput
{
  std::array<JointVector, 2> measured_q{};
  std::uint64_t state_sequence{0};
  std::uint64_t diagnostic_sequence{0};
  FreshnessEvidence state_freshness{};
  FreshnessEvidence diagnostic_freshness{};
  MoveRequest request{};
  oa_collision_contact_policy contact_policy{OA_COLLISION_CONTACT_NONE};
  bool require_terminal_contact{false};
  bool terminal_retreat{false};
  // Used by dynamic re-planning after a nominally single-arm request has been
  // represented as paired endpoints. -1 means neither side is preserved.
  int preserved_side{-1};
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
  bool target_projected{false};
  bool sampled_keepout_violation{false};
  bool limited_by_keepout{false};
  std::string reason;
  std::string limiting_reason;
  std::array<Point, 2> measured_tcp{};
  std::array<Point, 2> commanded_tcp{};
  Point requested_tcp{};
  double achieved_fraction{0.0};
  double minimum_nominal_clearance_m{0.0};
  double failure_path_fraction{1.0};
  double keepout_barrier_distance_m{0.0};
  bool terminal_pair_active{false};
  double terminal_pair_clearance_m{0.0};
  double terminal_tcp_separation_m{0.0};
  bool claw_contact_active{false};
  double claw_hand_gap_m{0.0};
  double minimum_other_claw_gap_m{0.0};
  // Common radial stop selected from exact pinned-mesh evidence for a
  // convergence command. Zero for every non-contact guard result.
  double contact_stop_distance_m{0.0};
};

struct GuardedRoute
{
  bool accepted{false};
  bool routed{false};
  bool used_clearance_recovery{false};
  std::string reason;
  GuardResult final;
  // Ordered paired TCP endpoints in body-frame metres. The measured start is
  // intentionally omitted; each element is one independently checked edge.
  std::vector<std::array<Point, 2>> waypoint_tcp;
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
  Point near_low{};
  Point outer_low{};
};

class StrictJson
{
public:
  static bool parse_move(std::string_view body, MoveRequest & out, std::string & reason);
  static bool parse_move_v2(std::string_view body, UnitMoveRequest & out, std::string & reason);
  static bool parse_move_v3(std::string_view body, UnitMoveRequest & out, std::string & reason);
  static bool parse_move_both(std::string_view body, UnitMoveRequest & out,
    Point & right_target, std::string & reason);
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

class CommandReservationGate
{
public:
  bool begin(std::uint64_t & token);
  bool valid(std::uint64_t token) const;
  bool consume(std::uint64_t token);
  bool release(std::uint64_t token);
  bool cancel();
  bool active() const;

private:
  std::uint64_t generation_{0};
  std::uint64_t active_token_{0};
};

class NominalPathGuard
{
public:
  GuardResult validate(const GuardInput & input) const;
  GuardResult validate_or_project(const GuardInput & input) const;
  // Re-proves one already selected paired endpoint from the newest measured
  // joints immediately before controller submission. Ordinary legs retain the
  // native-C route planner's conservative keepout proof. A terminal retreat
  // instead uses the exact, monotonic contact-exit proof because its measured
  // start is intentionally inside the ordinary claw keepout.
  GuardResult revalidate_direct_leg(
    const std::array<JointVector, 2> & measured_q,
    const std::array<Point, 2> & endpoint, bool terminal_retreat,
    int preserved_side = -1) const;
  // Ordinary commands first try an exact native-C collision-aware route. A
  // single-arm request retains the legacy best-effort projection only when no
  // exact graph route can be proven. Scoped contact and terminal retreats stay
  // on their dedicated, more restrictive validators.
  GuardedRoute route_or_project(const GuardInput & input) const;
  // Finds a branch-specific convergence endpoint using the same sampled IK
  // path and exact hand/finger meshes used by the real-time monitor. `input`
  // is updated with the accepted paired endpoints only on success.
  GuardResult validate_convergence_contact(
    GuardInput & input, const std::array<Point, 2> & measured_tcp,
    double nominal_stop_distance_m, double minimum_progress_m) const;

private:
  static bool forward(std::size_t side, const JointVector & q, oa_fk_result & result);
  static bool inverse(
    std::size_t side, const Point & target, const JointVector & seed,
    JointVector & q, std::string & reason);
  // tolerance_rad is for measured state only. Planned waypoints are checked
  // strictly, at zero tolerance.
  static bool validate_q(
    std::size_t side, const JointVector & q, std::string & reason,
    double tolerance_rad = 0.0);
  static bool scene_clear(
    const std::array<oa_fk_result, 2> & fk, oa_collision_contact_policy contact_policy,
    double & clearance, oa_collision_contact_evidence & contact, std::string & reason);
};

bool fresh_at_use(
  const FreshnessEvidence & evidence, std::int64_t now_time_ns,
  std::int64_t now_steady_ns, std::int64_t maximum_age_ns);
bool guard_handoff_valid(
  const GuardInput & guarded, const GuardHandoffEvidence & current,
  std::int64_t now_time_ns, std::int64_t now_steady_ns,
  std::int64_t state_maximum_age_ns, std::int64_t diagnostic_maximum_age_ns,
  std::string * failure_reason = nullptr);
bool normalise_move_to_metres(
  const UnitMoveRequest & input, MoveRequest & output, std::string & reason);
double nominal_contact_stop_distance_m();
bool prepare_convergence_guard_targets(
  MoveRequest & request, const std::array<Point, 2> & measured_tcp,
  double stop_distance_m, double minimum_progress_m, std::string & reason);
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
