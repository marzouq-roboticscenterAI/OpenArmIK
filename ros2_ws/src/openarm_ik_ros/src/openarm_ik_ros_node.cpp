// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/virtual_control_session.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <fcntl.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <openarm_control_msgs/action/move_joint.hpp>
#include <openarm_control_msgs/action/move_paired_tcp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sys/file.h>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <unistd.h>

#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{
using MoveJoint = openarm_control_msgs::action::MoveJoint;
using MovePairedTcp = openarm_control_msgs::action::MovePairedTcp;
using JointGoalHandle = rclcpp_action::ServerGoalHandle<MoveJoint>;
using PairedGoalHandle = rclcpp_action::ServerGoalHandle<MovePairedTcp>;
using openarm_ik_ros::CommandFeedback;
using openarm_ik_ros::CommandResult;
using openarm_ik_ros::MeasuredState;
using openarm_ik_ros::SessionCommand;
using openarm_ik_ros::VirtualControlSession;

class AuthorityLock final
{
public:
  AuthorityLock()
  {
    const char * domain = std::getenv("ROS_DOMAIN_ID");
    const std::string domain_id = domain == nullptr ? "0" : domain;
    const std::string path = "/tmp/openarm_ik_ros_joint_state_" +
      std::to_string(static_cast<unsigned long>(getuid())) + "_" + domain_id + ".lock";
    descriptor_ = open(path.c_str(), O_CREAT | O_CLOEXEC | O_RDWR, 0600);
    if (descriptor_ < 0 || flock(descriptor_, LOCK_EX | LOCK_NB) != 0) {
      if (descriptor_ >= 0) {
        close(descriptor_);
      }
      throw std::runtime_error("another local JointState authority owns this ROS domain");
    }
  }

  ~AuthorityLock()
  {
    if (descriptor_ >= 0) {
      (void)flock(descriptor_, LOCK_UN);
      close(descriptor_);
    }
  }

  AuthorityLock(const AuthorityLock &) = delete;
  AuthorityLock & operator=(const AuthorityLock &) = delete;

private:
  int descriptor_{-1};
};

struct DiagnosticRecord
{
  std::string action{"startup"};
  std::string owner;
  std::string reason{"ready"};
  std::string outcome{"none"};
  std::int64_t request_stamp_ns{};
  bool committed{};
  std::uint32_t control_status{};
  oa_runtime_status runtime_status{OA_RUNTIME_OK};
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
};

std::string uuid_string(const rclcpp_action::GoalUUID & uuid)
{
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const auto byte : uuid) {
    output << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return output.str();
}

diagnostic_msgs::msg::KeyValue field(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = std::move(key);
  item.value = std::move(value);
  return item;
}

std::string boolean(const bool value)
{
  return value ? "true" : "false";
}

bool finite_point(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

template<typename Result>
void fill_common_result(
  Result & output, const CommandResult & result, const rclcpp_action::GoalUUID & uuid)
{
  output.outcome = result.outcome == CommandResult::Outcome::completed ? Result::OUTCOME_COMPLETED :
    (result.outcome == CommandResult::Outcome::canceled ? Result::OUTCOME_CANCELED :
    (result.outcome == CommandResult::Outcome::rejected ? Result::OUTCOME_REJECTED :
    Result::OUTCOME_ABORTED));
  output.control_status = result.control_status;
  output.goal_id.uuid = uuid;
  output.command_id = result.command_id;
  output.lifecycle = result.lifecycle;
  output.event = result.event;
  output.cause = result.cause;
  output.collision_checked = result.collision_checked;
  output.reason = result.reason;
}

class OpenArmIkRosNode final : public rclcpp::Node
{
public:
  OpenArmIkRosNode()
  : Node("openarm_ik_ros"), authority_lock_()
  {
    bool use_sim_time = false;
    (void)get_parameter("use_sim_time", use_sim_time);
    if (use_sim_time) {
      throw std::invalid_argument("use_sim_time is unsupported by the steady-clock virtual controller");
    }
    const auto expiry_ms = declare_parameter<std::int64_t>("request_expiry_ms", 1000LL);
    if (expiry_ms < 1LL || expiry_ms > 60000LL) {
      throw std::invalid_argument("request_expiry_ms must be in [1, 60000]");
    }
    request_expiry_ns_ = expiry_ms * 1000000LL;

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    joint_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/openarm_ik/diagnostics", qos);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, *this, false);

    session_ = std::make_unique<VirtualControlSession>(
      [this](const MeasuredState & state) {return publish_measured(state);},
      [this]() {diagnostic_dirty_.store(true);});

    joint_server_ = rclcpp_action::create_server<MoveJoint>(
      this, "/openarm_ik/move_joint",
      [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveJoint::Goal> goal) {
        return on_joint_goal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<JointGoalHandle> goal) {return on_joint_cancel(goal);},
      [this](const std::shared_ptr<JointGoalHandle> goal) {accept_joint(goal);});
    paired_server_ = rclcpp_action::create_server<MovePairedTcp>(
      this, "/openarm_ik/move_paired_tcp",
      [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MovePairedTcp::Goal> goal) {
        return on_paired_goal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<PairedGoalHandle> goal) {return on_paired_cancel(goal);},
      [this](const std::shared_ptr<PairedGoalHandle> goal) {accept_paired(goal);});

    legacy_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/openarm_ik/paired_xyz", qos,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr message) {on_legacy(message);});
    diagnostics_timer_ = create_wall_timer(
      std::chrono::milliseconds(250), [this]() {
        if (diagnostic_dirty_.exchange(false) || ++diagnostic_ticks_ >= 4U) {
          diagnostic_ticks_ = 0U;
          publish_diagnostics();
        }
      });
    diagnostic_dirty_.store(true);
    RCLCPP_WARN(
      get_logger(),
      "Virtual measured-feedback control active; collision checking is unavailable and physical control is unsupported.");
  }

  ~OpenArmIkRosNode() override
  {
    if (session_) {
      session_->close();
    }
  }

private:
  bool valid_stamp(const builtin_interfaces::msg::Time & stamp, std::string & reason) const
  {
    const rclcpp::Time request_time(stamp, get_clock()->get_clock_type());
    const auto request_ns = request_time.nanoseconds();
    const auto now_ns = now().nanoseconds();
    if (request_ns <= 0) {
      reason = "zero_request_stamp";
      return false;
    }
    if (request_ns > now_ns) {
      reason = "future_request_stamp";
      return false;
    }
    if (now_ns - request_ns > request_expiry_ns_) {
      reason = "stale_request_stamp";
      return false;
    }
    return true;
  }

  rclcpp_action::GoalResponse on_joint_goal(
    const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const MoveJoint::Goal> goal)
  {
    std::string reason;
    std::uint32_t side{};
    std::uint32_t joint{};
    if (!valid_stamp(goal->stamp, reason) ||
      !VirtualControlSession::map_joint(goal->joint_name, side, joint) ||
      !VirtualControlSession::joint_target_in_limits(side, joint, goal->target_rad))
    {
      record_rejection("move_joint", uuid_string(uuid), reason.empty() ? "invalid_joint_goal" : reason);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!session_->reserve(uuid_string(uuid), reason)) {
      record_rejection("move_joint", uuid_string(uuid), reason);
      return rclcpp_action::GoalResponse::REJECT;
    }
    record_request("move_joint", uuid_string(uuid), rclcpp::Time(goal->stamp).nanoseconds());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_joint_cancel(const std::shared_ptr<JointGoalHandle> goal)
  {
    return session_->cancel(uuid_string(goal->get_goal_id())) ?
      rclcpp_action::CancelResponse::ACCEPT : rclcpp_action::CancelResponse::REJECT;
  }

  void accept_joint(const std::shared_ptr<JointGoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    std::uint32_t side{};
    std::uint32_t joint{};
    const std::string owner = uuid_string(goal_handle->get_goal_id());
    if (!VirtualControlSession::map_joint(goal->joint_name, side, joint)) {
      abort_joint_before_submit(goal_handle, owner, "joint_name_changed_after_acceptance");
      return;
    }
    SessionCommand command;
    command.kind = SessionCommand::Kind::joint;
    command.owner = owner;
    command.side = side;
    command.joint = joint;
    command.target_rad = goal->target_rad;
    command.feedback = [this, goal_handle](const CommandFeedback & value) {
        if (!goal_publishable(goal_handle)) {
          return false;
        }
        try {
          auto feedback = std::make_shared<MoveJoint::Feedback>();
          feedback->lifecycle = value.lifecycle;
          feedback->event = value.event;
          feedback->command_id = value.command_id;
          feedback->left_feedback_seq = value.feedback_seq[0];
          feedback->right_feedback_seq = value.feedback_seq[1];
          feedback->measured_progress = value.measured_progress;
          goal_handle->publish_feedback(feedback);
          return true;
        } catch (...) {
          return false;
        }
      };
    const auto goal_id = goal_handle->get_goal_id();
    const auto request_stamp = rclcpp::Time(goal->stamp).nanoseconds();
    command.terminal = [this, goal_handle, goal_id, owner, request_stamp, side](
      const CommandResult & value) {
        auto result = std::make_shared<MoveJoint::Result>();
        fill_common_result(*result, value, goal_id);
        result->seed_feedback_seq = value.seed_feedback_seq[side];
        result->plan_duration_ns = value.plan_duration_ns;
        result->terminal_feedback_seq = value.terminal_feedback_seq[side];
        record_terminal("move_joint", owner, request_stamp, value);
        return finish_goal(goal_handle, result, value.outcome);
      };
    std::string reason;
    if (!session_->submit(std::move(command), reason)) {
      abort_joint_before_submit(goal_handle, owner, reason);
    }
  }

  rclcpp_action::GoalResponse on_paired_goal(
    const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const MovePairedTcp::Goal> goal)
  {
    std::string reason;
    if (!valid_stamp(goal->header.stamp, reason) || goal->header.frame_id.empty() ||
      !finite_point(goal->left_tcp_m) || !finite_point(goal->right_tcp_m))
    {
      record_rejection("move_paired_tcp", uuid_string(uuid),
        reason.empty() ? "invalid_paired_goal" : reason);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!session_->reserve(uuid_string(uuid), reason)) {
      record_rejection("move_paired_tcp", uuid_string(uuid), reason);
      return rclcpp_action::GoalResponse::REJECT;
    }
    record_request("move_paired_tcp", uuid_string(uuid), rclcpp::Time(goal->header.stamp).nanoseconds());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_paired_cancel(const std::shared_ptr<PairedGoalHandle> goal)
  {
    return session_->cancel(uuid_string(goal->get_goal_id())) ?
      rclcpp_action::CancelResponse::ACCEPT : rclcpp_action::CancelResponse::REJECT;
  }

  bool transform_pair(
    const std_msgs::msg::Header & header, const geometry_msgs::msg::Point & left,
    const geometry_msgs::msg::Point & right, std::array<double, 3> & left_body,
    std::array<double, 3> & right_body, std::string & reason)
  {
    try {
      geometry_msgs::msg::PointStamped left_input;
      left_input.header = header;
      left_input.point = left;
      geometry_msgs::msg::PointStamped right_input;
      right_input.header = header;
      right_input.point = right;
      const auto left_output = tf_buffer_->transform(
        left_input, "openarm_body_link0", tf2::durationFromSec(0.0));
      const auto right_output = tf_buffer_->transform(
        right_input, "openarm_body_link0", tf2::durationFromSec(0.0));
      left_body = {left_output.point.x, left_output.point.y, left_output.point.z};
      right_body = {right_output.point.x, right_output.point.y, right_output.point.z};
      return std::all_of(left_body.begin(), left_body.end(), [](double value) {return std::isfinite(value);}) &&
        std::all_of(right_body.begin(), right_body.end(), [](double value) {return std::isfinite(value);});
    } catch (const tf2::TransformException & error) {
      reason = "transform_unavailable:" + std::string(error.what());
      return false;
    }
  }

  void accept_paired(const std::shared_ptr<PairedGoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const std::string owner = uuid_string(goal_handle->get_goal_id());
    std::array<double, 3> left{};
    std::array<double, 3> right{};
    std::string reason;
    if (!transform_pair(goal->header, goal->left_tcp_m, goal->right_tcp_m, left, right, reason)) {
      abort_paired_before_submit(goal_handle, owner, reason.empty() ? "invalid_transformed_target" : reason);
      return;
    }
    SessionCommand command;
    command.kind = SessionCommand::Kind::paired_tcp;
    command.owner = owner;
    command.left_tcp_m = left;
    command.right_tcp_m = right;
    command.feedback = [this, goal_handle](const CommandFeedback & value) {
        if (!goal_publishable(goal_handle)) {
          return false;
        }
        try {
          auto feedback = std::make_shared<MovePairedTcp::Feedback>();
          feedback->lifecycle = value.lifecycle;
          feedback->event = value.event;
          feedback->command_id = value.command_id;
          feedback->left_feedback_seq = value.feedback_seq[0];
          feedback->right_feedback_seq = value.feedback_seq[1];
          feedback->measured_progress = value.measured_progress;
          goal_handle->publish_feedback(feedback);
          return true;
        } catch (...) {
          return false;
        }
      };
    const auto goal_id = goal_handle->get_goal_id();
    const auto request_stamp = rclcpp::Time(goal->header.stamp).nanoseconds();
    command.terminal = [this, goal_handle, goal_id, owner, request_stamp](
      const CommandResult & value) {
        auto result = std::make_shared<MovePairedTcp::Result>();
        fill_common_result(*result, value, goal_id);
        result->left_seed_feedback_seq = value.seed_feedback_seq[0];
        result->right_seed_feedback_seq = value.seed_feedback_seq[1];
        result->plan_duration_ns = value.plan_duration_ns;
        result->left_terminal_feedback_seq = value.terminal_feedback_seq[0];
        result->right_terminal_feedback_seq = value.terminal_feedback_seq[1];
        record_terminal("move_paired_tcp", owner, request_stamp, value);
        return finish_goal(goal_handle, result, value.outcome);
      };
    if (!session_->submit(std::move(command), reason)) {
      abort_paired_before_submit(goal_handle, owner, reason);
    }
  }

  template<typename GoalHandle, typename Result>
  bool finish_goal(
    const std::shared_ptr<GoalHandle> & goal, const std::shared_ptr<Result> & result,
    const CommandResult::Outcome outcome) noexcept
  {
    try {
      if (!goal_publishable(goal)) {
        return false;
      }
      if (outcome == CommandResult::Outcome::completed) {
        goal->succeed(result);
      } else if (outcome == CommandResult::Outcome::canceled) {
        goal->canceled(result);
      } else {
        goal->abort(result);
      }
      return true;
    } catch (...) {
      return false;
    }
  }

  template<typename GoalHandle>
  bool goal_publishable(const std::shared_ptr<GoalHandle> & goal) noexcept
  {
    try {
      const auto context = get_node_base_interface()->get_context();
      return context && context->is_valid() && goal && goal->is_active();
    } catch (...) {
      return false;
    }
  }

  void abort_joint_before_submit(
    const std::shared_ptr<JointGoalHandle> & goal, const std::string & owner,
    const std::string & reason)
  {
    session_->release(owner, reason);
    auto result = std::make_shared<MoveJoint::Result>();
    CommandResult internal;
    internal.outcome = CommandResult::Outcome::rejected;
    internal.runtime_status = OA_RUNTIME_EINVAL;
    internal.reason = reason;
    const auto goal_id = goal->get_goal_id();
    fill_common_result(*result, internal, goal_id);
    record_terminal(
      "move_joint", owner, rclcpp::Time(goal->get_goal()->stamp).nanoseconds(), internal);
    (void)finish_goal(goal, result, internal.outcome);
  }

  void abort_paired_before_submit(
    const std::shared_ptr<PairedGoalHandle> & goal, const std::string & owner,
    const std::string & reason)
  {
    session_->release(owner, reason);
    auto result = std::make_shared<MovePairedTcp::Result>();
    CommandResult internal;
    internal.outcome = CommandResult::Outcome::rejected;
    internal.runtime_status = OA_RUNTIME_EINVAL;
    internal.reason = reason;
    const auto goal_id = goal->get_goal_id();
    fill_common_result(*result, internal, goal_id);
    record_terminal(
      "move_paired_tcp", owner,
      rclcpp::Time(goal->get_goal()->header.stamp).nanoseconds(), internal);
    (void)finish_goal(goal, result, internal.outcome);
  }

  void on_legacy(const geometry_msgs::msg::PoseArray::SharedPtr message)
  {
    std::string reason;
    if (message->poses.size() != 2U || !valid_stamp(message->header.stamp, reason) ||
      !finite_point(message->poses[0].position) || !finite_point(message->poses[1].position))
    {
      record_rejection("deprecated_paired_xyz", "legacy", reason.empty() ? "invalid_legacy_goal" : reason);
      return;
    }
    const std::string owner = "legacy:" + std::to_string(rclcpp::Time(message->header.stamp).nanoseconds());
    if (!session_->reserve(owner, reason)) {
      record_rejection("deprecated_paired_xyz", owner, reason);
      return;
    }
    std::array<double, 3> left{};
    std::array<double, 3> right{};
    if (!transform_pair(
        message->header, message->poses[0].position, message->poses[1].position,
        left, right, reason))
    {
      session_->release(owner, reason);
      record_rejection("deprecated_paired_xyz", owner, reason);
      return;
    }
    const auto request_stamp = rclcpp::Time(message->header.stamp).nanoseconds();
    record_request("deprecated_paired_xyz", owner, request_stamp);
    SessionCommand command;
    command.kind = SessionCommand::Kind::paired_tcp;
    command.owner = owner;
    command.left_tcp_m = left;
    command.right_tcp_m = right;
    command.terminal = [this, owner, request_stamp](const CommandResult & result) {
        record_terminal("deprecated_paired_xyz", owner, request_stamp, result);
        return true;
      };
    if (!session_->submit(std::move(command), reason)) {
      session_->release(owner, reason);
      record_rejection("deprecated_paired_xyz", owner, reason);
    }
  }

  bool publish_measured(const MeasuredState & measured)
  {
    const auto oldest = std::min(
      measured.snapshot.arm[0].measurement_runtime_monotonic_ns,
      measured.snapshot.arm[1].measurement_runtime_monotonic_ns);
    if (oldest == 0U || oldest > measured.runtime_now_ns) {
      return false;
    }
    const std::uint64_t age = measured.runtime_now_ns - oldest;
    const auto ros_now = now().nanoseconds();
    if (ros_now <= 0 || age > static_cast<std::uint64_t>(ros_now)) {
      return false;
    }
    const auto stamp = ros_now - static_cast<std::int64_t>(age);
    {
      std::lock_guard<std::mutex> lock(clock_mutex_);
      if (last_ros_now_ns_ != 0 && ros_now < last_ros_now_ns_) {
        return false;
      }
      if (last_runtime_now_ns_ != 0) {
        const auto ros_delta = ros_now - last_ros_now_ns_;
        if (measured.runtime_now_ns < last_runtime_now_ns_) {
          return false;
        }
        const auto runtime_delta = measured.runtime_now_ns - last_runtime_now_ns_;
        const auto difference = std::llabs(
          ros_delta - static_cast<std::int64_t>(runtime_delta));
        if (difference > 250000000LL || stamp <= last_measurement_stamp_ns_) {
          return false;
        }
      }
      last_ros_now_ns_ = ros_now;
      last_runtime_now_ns_ = measured.runtime_now_ns;
      last_measurement_stamp_ns_ = stamp;
    }

    sensor_msgs::msg::JointState state;
    state.header.stamp = rclcpp::Time(stamp, get_clock()->get_clock_type());
    const auto & names = VirtualControlSession::joint_names();
    state.name.assign(names.begin(), names.end());
    state.position.reserve(14U);
    state.velocity.reserve(14U);
    state.effort.reserve(14U);
    for (std::size_t side = 0; side < 2U; ++side) {
      for (std::size_t joint = 0; joint < 7U; ++joint) {
        state.position.push_back(measured.snapshot.arm[side].q_model_rad[joint]);
        state.velocity.push_back(measured.snapshot.arm[side].dq_model_rad_s[joint]);
        state.effort.push_back(measured.snapshot.arm[side].tau_model_nm[joint]);
      }
    }
    joint_publisher_->publish(std::move(state));
    return true;
  }

  void record_request(const std::string & action, const std::string & owner, std::int64_t stamp)
  {
    DiagnosticRecord next;
    next.action = action;
    next.owner = owner;
    next.request_stamp_ns = stamp;
    next.reason = "accepted";
    next.outcome = "pending";
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    diagnostic_record_ = std::move(next);
    diagnostic_dirty_.store(true);
  }

  void record_rejection(
    const std::string & action, const std::string & owner, const std::string & reason)
  {
    DiagnosticRecord next;
    next.action = action;
    next.owner = owner;
    next.reason = reason;
    next.outcome = "rejected";
    next.runtime_status = OA_RUNTIME_ESTATE;
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    diagnostic_record_ = std::move(next);
    diagnostic_dirty_.store(true);
  }

  void record_terminal(
    const std::string & action, const std::string & owner, const std::int64_t stamp,
    const CommandResult & result)
  {
    DiagnosticRecord next;
    next.action = action;
    next.owner = owner;
    next.request_stamp_ns = stamp;
    next.reason = result.reason;
    next.committed = result.outcome == CommandResult::Outcome::completed;
    next.outcome = result.outcome == CommandResult::Outcome::completed ? "completed" :
      (result.outcome == CommandResult::Outcome::canceled ? "canceled" :
      (result.outcome == CommandResult::Outcome::rejected ? "rejected" : "aborted"));
    next.control_status = result.control_status;
    next.runtime_status = result.runtime_status;
    next.runtime_facility = result.runtime_facility;
    next.lower_status = result.lower_status;
    next.system_error = result.system_error;
    next.command_id = result.command_id;
    next.seed_feedback_seq[0] = result.seed_feedback_seq[0];
    next.seed_feedback_seq[1] = result.seed_feedback_seq[1];
    next.plan_duration_ns = result.plan_duration_ns;
    next.terminal_feedback_seq[0] = result.terminal_feedback_seq[0];
    next.terminal_feedback_seq[1] = result.terminal_feedback_seq[1];
    next.lifecycle = result.lifecycle;
    next.event = result.event;
    next.cause = result.cause;
    next.collision_checked = result.collision_checked;
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    diagnostic_record_ = std::move(next);
    diagnostic_dirty_.store(true);
  }

  void publish_diagnostics()
  {
    const auto health = session_->health();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "openarm_ik_ros/virtual_control";
    status.hardware_id = "openarm_v10_virtual";
    const bool error = health.adapter_state == openarm_ik_ros::AdapterState::fault ||
      health.adapter_state == openarm_ik_ros::AdapterState::closing;
    status.level = error ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = error ? health.reason : "virtual backend; collision unchecked";
    const auto now_runtime = health.runtime_now_ns;
    const auto left_time = health.snapshot.arm[0].measurement_runtime_monotonic_ns;
    const auto right_time = health.snapshot.arm[1].measurement_runtime_monotonic_ns;
    const auto left_age = now_runtime >= left_time ? now_runtime - left_time : 0U;
    const auto right_age = now_runtime >= right_time ? now_runtime - right_time : 0U;
    const auto skew = left_time > right_time ? left_time - right_time : right_time - left_time;
    status.values = {
      field("backend", "virtual"),
      field("runtime_authority", "openarm_runtime"),
      field("runtime_backend", std::to_string(health.capabilities.backend)),
      field("capability_bits", std::to_string(health.capabilities.capabilities)),
      field("runtime_clock_id", std::to_string(health.capabilities.clock_id)),
      field("runtime_units_id", std::to_string(health.capabilities.units_id)),
      field("runtime_xyz_frame_id", std::to_string(health.capabilities.xyz_frame_id)),
      field("runtime_coordinate_identity_sha256",
        health.capabilities.coordinate_identity_sha256),
      field("virtual_execution_enabled", "true"),
      field("physical_motion_authorized", "false"),
      field("physical_motion_capability", boolean(
        (health.capabilities.capabilities & OA_RUNTIME_CAP_PHYSICAL_MOTION) != 0U)),
      field("physical_discovery_endpoint_exposed", "false"),
      field("single_xyz_capability", boolean(
        (health.capabilities.capabilities & OA_RUNTIME_CAP_SINGLE_XYZ_IK) != 0U)),
      field("collision_policy", "virtual_unchecked"),
      field("collision_checked", "false"),
      field("orientation_constrained", "false"),
      field("state_source", "oa_snapshot_encoder_feedback"),
      field("runtime_state_source", "oa_runtime_snapshot_encoder_feedback"),
      field("adapter_state", VirtualControlSession::adapter_state_name(health.adapter_state)),
      field("lifecycle", std::to_string(health.snapshot.lifecycle)),
      field("executing", boolean(health.command_id != 0U)),
      field("active_owner", health.owner),
      field("command_id", std::to_string(health.command_id)),
      field("left_plan_seed_feedback_seq",
        std::to_string(health.plan_seed_feedback_seq[0])),
      field("right_plan_seed_feedback_seq",
        std::to_string(health.plan_seed_feedback_seq[1])),
      field("plan_duration_ns", std::to_string(health.plan_duration_ns)),
      field("left_terminal_feedback_seq", std::to_string(health.terminal_feedback_seq[0])),
      field("right_terminal_feedback_seq", std::to_string(health.terminal_feedback_seq[1])),
      field("last_event", std::to_string(health.last_event)),
      field("last_cause", std::to_string(health.last_error.lower_code)),
      field("last_runtime_status", std::to_string(health.last_error.status)),
      field("last_runtime_facility", std::to_string(health.last_error.facility)),
      field("last_runtime_lower_status", std::to_string(health.last_error.lower_code)),
      field("last_runtime_system_error", std::to_string(health.last_error.system_error)),
      field("manifest_revision", std::to_string(health.snapshot.manifest_revision)),
      field("model_revision", std::to_string(health.snapshot.model_revision)),
      field("manifest_state", std::to_string(health.manifest.state)),
      field("manifest_intended_backend", std::to_string(health.manifest.intended_backend)),
      field("manifest_content_sha256", health.manifest.content_sha256),
      field("manifest_integrity_kind", std::to_string(health.manifest.integrity_kind)),
      field("manifest_authenticated", boolean(health.manifest.authenticated != 0U)),
      field("manifest_checkpoint_authorized",
        boolean(health.manifest.checkpoint_authorized != 0U)),
      field("persistence_status", "built_in_immutable_manifest_not_persisted"),
      field("calibration_status", "runtime_capable_ros_endpoint_not_exposed"),
      field("discovery_status", "virtual_exact_inventory"),
      field("inventory_revision", std::to_string(health.inventory.inventory_revision)),
      field("inventory_interface_count", std::to_string(health.inventory.interface_count)),
      field("inventory_motor_count", std::to_string(health.inventory.motor_count)),
      field("inventory_unknown_mask", std::to_string(health.inventory.unknown_mask)),
      field("inventory_ambiguous_mask", std::to_string(health.inventory.ambiguous_mask)),
      field("inventory_unresolved_assignment",
        std::to_string(health.inventory.unresolved_assignment)),
      field("inventory_fingerprint_sha256", health.inventory.fingerprint_sha256),
      field("left_model_id", health.model_identity[0].model_id),
      field("right_model_id", health.model_identity[1].model_id),
      field("left_tcp_frame", health.model_identity[0].tcp_frame),
      field("right_tcp_frame", health.model_identity[1].tcp_frame),
      field("left_tcp_revision", std::to_string(health.model_identity[0].tcp_revision)),
      field("right_tcp_revision", std::to_string(health.model_identity[1].tcp_revision)),
      field("collision_scene_revision",
        std::to_string(health.model_identity[0].collision_scene_revision)),
      field("verify_epoch_available", "false"),
      field("verify_epoch", "0"),
      field("left_expected_mask", std::to_string(health.snapshot.arm[0].expected_mask)),
      field("left_fresh_mask", std::to_string(health.snapshot.arm[0].fresh_mask)),
      field("left_fault_mask", std::to_string(health.snapshot.arm[0].fault_mask)),
      field("right_expected_mask", std::to_string(health.snapshot.arm[1].expected_mask)),
      field("right_fresh_mask", std::to_string(health.snapshot.arm[1].fresh_mask)),
      field("right_fault_mask", std::to_string(health.snapshot.arm[1].fault_mask)),
      field("left_feedback_seq", std::to_string(health.snapshot.arm[0].feedback_seq)),
      field("right_feedback_seq", std::to_string(health.snapshot.arm[1].feedback_seq)),
      field("left_feedback_t_ns", std::to_string(left_time)),
      field("right_feedback_t_ns", std::to_string(right_time)),
      field("left_feedback_age_ns", std::to_string(left_age)),
      field("right_feedback_age_ns", std::to_string(right_age)),
      field("pair_skew_ns", std::to_string(skew))};
    {
      std::lock_guard<std::mutex> lock(diagnostic_mutex_);
      const auto & record = diagnostic_record_;
      status.values.push_back(field("last_action", record.action));
      status.values.push_back(field("last_goal_id", record.owner));
      status.values.push_back(field("request_stamp_ns", std::to_string(record.request_stamp_ns)));
      status.values.push_back(field("committed", boolean(record.committed)));
      status.values.push_back(field("reason", record.reason));
      status.values.push_back(field("outcome", record.outcome));
      status.values.push_back(field("result_control_status", std::to_string(record.control_status)));
      status.values.push_back(field("result_runtime_status", std::to_string(record.runtime_status)));
      status.values.push_back(field(
        "result_runtime_facility", std::to_string(record.runtime_facility)));
      status.values.push_back(field("result_lower_status", std::to_string(record.lower_status)));
      status.values.push_back(field("result_system_error", std::to_string(record.system_error)));
      status.values.push_back(field("result_command_id", std::to_string(record.command_id)));
      status.values.push_back(field(
        "result_left_plan_seed_feedback_seq", std::to_string(record.seed_feedback_seq[0])));
      status.values.push_back(field(
        "result_right_plan_seed_feedback_seq", std::to_string(record.seed_feedback_seq[1])));
      status.values.push_back(field(
        "result_plan_duration_ns", std::to_string(record.plan_duration_ns)));
      status.values.push_back(field(
        "result_left_terminal_feedback_seq", std::to_string(record.terminal_feedback_seq[0])));
      status.values.push_back(field(
        "result_right_terminal_feedback_seq", std::to_string(record.terminal_feedback_seq[1])));
      status.values.push_back(field("result_lifecycle", std::to_string(record.lifecycle)));
      status.values.push_back(field("result_event", std::to_string(record.event)));
      status.values.push_back(field("result_cause", std::to_string(record.cause)));
      status.values.push_back(field(
        "result_collision_checked", boolean(record.collision_checked)));
      status.values.push_back(field("legacy_topic_deprecated", "true"));
      status.values.push_back(field("legacy_mapping", "pose[0]=left,pose[1]=right"));
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(std::move(array));
  }

  AuthorityLock authority_lock_;
  std::int64_t request_expiry_ns_{};
  std::unique_ptr<VirtualControlSession> session_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp_action::Server<MoveJoint>::SharedPtr joint_server_;
  rclcpp_action::Server<MovePairedTcp>::SharedPtr paired_server_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr legacy_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::atomic<bool> diagnostic_dirty_{false};
  std::uint32_t diagnostic_ticks_{};
  std::mutex diagnostic_mutex_;
  DiagnosticRecord diagnostic_record_;
  std::mutex clock_mutex_;
  std::int64_t last_ros_now_ns_{};
  std::uint64_t last_runtime_now_ns_{};
  std::int64_t last_measurement_stamp_ns_{};
};
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<OpenArmIkRosNode>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::fprintf(stderr, "openarm_ik_ros_node: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
