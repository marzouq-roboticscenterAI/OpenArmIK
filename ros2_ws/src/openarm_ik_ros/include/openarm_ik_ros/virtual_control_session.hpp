// SPDX-License-Identifier: Apache-2.0
#ifndef OPENARM_IK_ROS__VIRTUAL_CONTROL_SESSION_HPP_
#define OPENARM_IK_ROS__VIRTUAL_CONTROL_SESSION_HPP_

#include "openarm_ik_ros/motion_profile.hpp"

#include "openarm_runtime.h"
#include "openarm_runtime_motion.h"

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
  struct Gripper
  {
    bool calibrated{false};
    double opening_m{};
    double velocity_m_s{};
    double motor_position_rad{};
    double motor_velocity_rad_s{};
    double motor_torque_nm{};
  };
  std::array<Gripper, 2> gripper{};
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
  enum class Kind {joint, paired_tcp, centroid_tcp, mirrored_tcp, converge_tcp, neutral, gripper};
  Kind kind{Kind::joint};
  std::string owner;
  std::uint32_t side{kLeftSide};
  std::uint32_t joint{};
  double target_rad{};
  std::array<double, 3> left_tcp_m{};
  std::array<double, 3> right_tcp_m{};
  // centroid_tcp: where the claw midpoint should end up.
  // converge_tcp: the point both claws advance on until contact.
  std::array<double, 3> target_m{};
  double stop_distance_m{0.05};
  double contact_torque_fraction{0.0};
  double minimum_progress_m{0.001};
  double motion_limit_scale{kLegacyMotionLimitScale};
  // -1 moves both arms; 0/1 preserves robot-left/right at the exact measured
  // start joint vector throughout Cartesian routing.
  int preserved_side{-1};
  // Gripper commands use bit 0 for robot-left and bit 1 for robot-right.
  std::uint32_t gripper_side_mask{};
  double gripper_opening_m{};
  double gripper_speed_m_s{};
  double gripper_torque_limit_nm{};
  bool gripper_stop_on_contact{false};
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

class ControlSession
{
public:
  using StateCallback = std::function<bool(const MeasuredState &)>;
  using HealthCallback = std::function<void()>;

  virtual ~ControlSession() = default;
  virtual bool reserve(const std::string & owner, std::string & reason) = 0;
  virtual bool submit(SessionCommand command, std::string & reason) = 0;
  virtual bool cancel(const std::string & owner) = 0;
  virtual void release(const std::string & owner, const std::string & reason) = 0;
  virtual SessionHealth health() const = 0;
  virtual void close() noexcept = 0;
};

class VirtualControlSession final : public ControlSession
{
public:
  using StateCallback = ControlSession::StateCallback;
  using HealthCallback = ControlSession::HealthCallback;

  explicit VirtualControlSession(StateCallback state_callback, HealthCallback health_callback);
  ~VirtualControlSession();
  VirtualControlSession(const VirtualControlSession &) = delete;
  VirtualControlSession & operator=(const VirtualControlSession &) = delete;

  bool reserve(const std::string & owner, std::string & reason) override;
  bool submit(SessionCommand command, std::string & reason) override;
  bool cancel(const std::string & owner) override;
  void release(const std::string & owner, const std::string & reason) override;
  SessionHealth health() const override;
  void close() noexcept override;

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
