// SPDX-License-Identifier: Apache-2.0
#ifndef OPENARM_IK_ROS__VIRTUAL_CONTROL_SESSION_HPP_
#define OPENARM_IK_ROS__VIRTUAL_CONTROL_SESSION_HPP_

#include "openarm_control.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace openarm_ik_ros
{

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
  oa_snapshot snapshot{};
  std::uint64_t controller_now_ns{};
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
  oa_control_status control_status{OA_CONTROL_ESTATE};
  std::uint64_t command_id{};
  std::uint64_t seed_feedback_seq[2]{};
  std::uint64_t plan_duration_ns{};
  std::uint64_t terminal_feedback_seq[2]{};
  std::uint32_t lifecycle{};
  std::uint32_t event{};
  oa_control_status cause{OA_CONTROL_OK};
  bool collision_checked{};
  std::string reason;
};

struct SessionCommand
{
  enum class Kind {joint, paired_tcp};
  Kind kind{Kind::joint};
  std::string owner;
  oa_side side{OA_LEFT};
  std::uint32_t joint{};
  double target_rad{};
  std::array<double, 3> left_tcp_m{};
  std::array<double, 3> right_tcp_m{};
  std::function<bool(const CommandFeedback &)> feedback;
  std::function<bool(const CommandResult &)> terminal;
};

struct SessionHealth
{
  AdapterState adapter_state{AdapterState::starting};
  oa_snapshot snapshot{};
  std::uint64_t controller_now_ns{};
  std::uint64_t command_id{};
  std::uint32_t last_event{};
  oa_control_status last_cause{OA_CONTROL_OK};
  std::uint64_t verify_epoch{};
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
  static bool map_joint(const std::string & name, oa_side & side, std::uint32_t & joint);
  static bool joint_target_in_limits(oa_side side, std::uint32_t joint, double target);
  static const char * adapter_state_name(AdapterState state) noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
#endif
