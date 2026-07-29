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

namespace openarm_ik_ros::portal
{

using JointVector = std::array<double, OA_DOF>;
using Point = std::array<double, 3>;

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
  MoveRequest request{};
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
std::string json_escape(std::string_view value);

}  // namespace openarm_ik_ros::portal

#endif  // OPENARM_IK_ROS__PORTAL_CORE_HPP_
