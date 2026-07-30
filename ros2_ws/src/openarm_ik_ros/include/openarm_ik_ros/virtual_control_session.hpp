// SPDX-License-Identifier: Apache-2.0
#ifndef OPENARM_IK_ROS__VIRTUAL_CONTROL_SESSION_HPP_
#define OPENARM_IK_ROS__VIRTUAL_CONTROL_SESSION_HPP_

#include "openarm_runtime.h"

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace openarm_ik_ros
{

constexpr std::uint32_t kLeftSide = 0U;
constexpr std::uint32_t kRightSide = 1U;
constexpr std::uint32_t kLifecycleDisarmed = 2U;
constexpr std::uint32_t kLifecycleArmedIdle = 4U;
constexpr std::uint32_t kLifecycleExecuting = 5U;
constexpr std::uint32_t kLifecycleFault = 7U;
constexpr std::uint32_t kLifecycleEstop = 8U;

enum class AdapterState
{
  starting,
  idle,
  reserved,
  executing,
  stopped_requires_restart,
  fault,
  closing
};

struct MeasuredState
{
  oa_runtime_snapshot snapshot{};
  std::uint64_t runtime_now_ns{};
};

struct CommandFeedback
{
  std::uint32_t lifecycle{};
  std::uint32_t event{};
  std::uint64_t command_id{};
  std::uint64_t feedback_seq[2]{};
  double measured_progress{};
};

struct CommandResult
{
  enum class Outcome {completed, canceled, rejected, aborted};
  Outcome outcome{Outcome::aborted};
  // Compatibility field for the existing ROS actions. It is populated only
  // when Runtime reports an actual lower Control status.
  std::uint32_t control_status{};
  oa_runtime_status runtime_status{OA_RUNTIME_ESTATE};
  oa_runtime_facility runtime_facility{OA_RUNTIME_FACILITY_RUNTIME};
  std::uint32_t lower_status{};
  std::uint32_t system_error{};
  std::uint64_t command_id{};
  std::uint64_t seed_feedback_seq[2]{};
  std::uint64_t plan_duration_ns{};
  std::uint64_t terminal_feedback_seq[2]{};
  std::uint32_t lifecycle{};
  std::uint32_t event{};
  std::uint32_t cause{};
  bool collision_checked{};
  bool motion_authorized{};
  std::string reason;
};

struct SessionCommand
{
  enum class Kind {joint, paired_tcp};
  Kind kind{Kind::joint};
  std::string owner;
  std::uint32_t side{kLeftSide};
  std::uint32_t joint{};
  double target_rad{};
  std::array<double, 3> left_tcp_m{};
  std::array<double, 3> right_tcp_m{};
  std::function<bool(const CommandFeedback &)> feedback;
  std::function<bool(const CommandResult &)> terminal;
#ifdef OPENARM_IK_ROS_TESTING
  // Internal deterministic lifecycle-test barrier; absent from production builds.
  std::function<void(std::uint64_t)> cancel_captured_for_test;
#endif
};

struct SessionHealth
{
  AdapterState adapter_state{AdapterState::starting};
  oa_runtime_snapshot snapshot{};
  std::uint64_t runtime_now_ns{};
  std::uint64_t command_id{};
  std::uint32_t last_event{};
  oa_runtime_error_detail last_error{};
  oa_runtime_capability_report capabilities{};
  oa_runtime_manifest_summary manifest{};
  oa_runtime_inventory_summary inventory{};
  oa_runtime_model_identity model_identity[2]{};
  std::uint64_t plan_seed_feedback_seq[2]{};
  std::uint64_t plan_duration_ns{};
  std::uint64_t terminal_feedback_seq[2]{};
  std::string owner;
  std::string reason;
};

class VirtualControlSession final
{
public:
  using StateCallback = std::function<bool(const MeasuredState &)>;
  using HealthCallback = std::function<void()>;

  explicit VirtualControlSession(StateCallback state_callback, HealthCallback health_callback);
  ~VirtualControlSession();
  VirtualControlSession(const VirtualControlSession &) = delete;
  VirtualControlSession & operator=(const VirtualControlSession &) = delete;

  bool reserve(const std::string & owner, std::string & reason);
  bool submit(SessionCommand command, std::string & reason);
  bool cancel(const std::string & owner);
  void release(const std::string & owner, const std::string & reason);
  SessionHealth health() const;
  void close() noexcept;

  static const std::array<std::string, 14> & joint_names();
  static bool map_joint(const std::string & name, std::uint32_t & side, std::uint32_t & joint);
  static bool joint_target_in_limits(std::uint32_t side, std::uint32_t joint, double target);
  static const char * adapter_state_name(AdapterState state) noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
#endif
