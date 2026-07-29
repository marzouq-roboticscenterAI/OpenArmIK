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
    oa_side side{};
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
    oa_side side{};
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
    command.feedback = [goal_handle](const CommandFeedback & value) {
        auto feedback = std::make_shared<MoveJoint::Feedback>();
        feedback->lifecycle = value.lifecycle;
        feedback->event = value.event;
        feedback->command_id = value.command_id;
        feedback->left_feedback_seq = value.feedback_seq[0];
        feedback->right_feedback_seq = value.feedback_seq[1];
        feedback->measured_progress = value.measured_progress;
        goal_handle->publish_feedback(feedback);
      };
    command.terminal = [this, goal_handle](const CommandResult & value) {
        auto result = std::make_shared<MoveJoint::Result>();
        fill_common_result(*result, value, goal_handle->get_goal_id());
        const auto goal = goal_handle->get_goal();
        oa_side side{};
        std::uint32_t joint{};
        (void)VirtualControlSession::map_joint(goal->joint_name, side, joint);
        result->seed_feedback_seq = value.seed_feedback_seq[side];
        result->terminal_feedback_seq = value.terminal_feedback_seq[side];
        finish_goal(goal_handle, result, value.outcome);
        record_terminal(value);
        diagnostic_dirty_.store(true);
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
    command.feedback = [goal_handle](const CommandFeedback & value) {
        auto feedback = std::make_shared<MovePairedTcp::Feedback>();
        feedback->lifecycle = value.lifecycle;
        feedback->event = value.event;
        feedback->command_id = value.command_id;
        feedback->left_feedback_seq = value.feedback_seq[0];
        feedback->right_feedback_seq = value.feedback_seq[1];
        feedback->measured_progress = value.measured_progress;
        goal_handle->publish_feedback(feedback);
      };
    command.terminal = [this, goal_handle](const CommandResult & value) {
        auto result = std::make_shared<MovePairedTcp::Result>();
        fill_common_result(*result, value, goal_handle->get_goal_id());
        result->left_seed_feedback_seq = value.seed_feedback_seq[0];
        result->right_seed_feedback_seq = value.seed_feedback_seq[1];
        result->left_terminal_feedback_seq = value.terminal_feedback_seq[0];
        result->right_terminal_feedback_seq = value.terminal_feedback_seq[1];
        finish_goal(goal_handle, result, value.outcome);
        record_terminal(value);
        diagnostic_dirty_.store(true);
      };
    if (!session_->submit(std::move(command), reason)) {
      abort_paired_before_submit(goal_handle, owner, reason);
    }
  }

  template<typename GoalHandle, typename Result>
  static void finish_goal(
    const std::shared_ptr<GoalHandle> & goal, const std::shared_ptr<Result> & result,
    const CommandResult::Outcome outcome)
  {
    if (outcome == CommandResult::Outcome::completed) {
      goal->succeed(result);
    } else if (outcome == CommandResult::Outcome::canceled) {
      goal->canceled(result);
    } else {
      goal->abort(result);
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
    internal.control_status = OA_CONTROL_EINVAL;
    internal.reason = reason;
    fill_common_result(*result, internal, goal->get_goal_id());
    goal->abort(result);
  }

  void abort_paired_before_submit(
    const std::shared_ptr<PairedGoalHandle> & goal, const std::string & owner,
    const std::string & reason)
  {
    session_->release(owner, reason);
    auto result = std::make_shared<MovePairedTcp::Result>();
    CommandResult internal;
    internal.outcome = CommandResult::Outcome::rejected;
    internal.control_status = OA_CONTROL_EINVAL;
    internal.reason = reason;
    fill_common_result(*result, internal, goal->get_goal_id());
    goal->abort(result);
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
    record_request("deprecated_paired_xyz", owner, rclcpp::Time(message->header.stamp).nanoseconds());
    SessionCommand command;
    command.kind = SessionCommand::Kind::paired_tcp;
    command.owner = owner;
    command.left_tcp_m = left;
    command.right_tcp_m = right;
    command.terminal = [this](const CommandResult & result) {
        std::lock_guard<std::mutex> lock(diagnostic_mutex_);
        last_committed_ = result.outcome == CommandResult::Outcome::completed;
        last_reason_ = result.reason;
        diagnostic_dirty_.store(true);
      };
    if (!session_->submit(std::move(command), reason)) {
      session_->release(owner, reason);
      record_rejection("deprecated_paired_xyz", owner, reason);
    }
  }

  bool publish_measured(const MeasuredState & measured)
  {
    const auto oldest = std::min(measured.snapshot.arm[0].t_ns, measured.snapshot.arm[1].t_ns);
    if (oldest > measured.controller_now_ns) {
      return false;
    }
    const std::uint64_t age = measured.controller_now_ns - oldest;
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
      if (last_controller_now_ns_ != 0) {
        const auto ros_delta = ros_now - last_ros_now_ns_;
        const auto controller_delta = measured.controller_now_ns - last_controller_now_ns_;
        const auto difference = std::llabs(
          ros_delta - static_cast<std::int64_t>(controller_delta));
        if (difference > 250000000LL || stamp <= last_measurement_stamp_ns_) {
          return false;
        }
      }
      last_ros_now_ns_ = ros_now;
      last_controller_now_ns_ = measured.controller_now_ns;
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
        state.position.push_back(measured.snapshot.arm[side].q[joint]);
        state.velocity.push_back(measured.snapshot.arm[side].dq[joint]);
        state.effort.push_back(measured.snapshot.arm[side].tau[joint]);
      }
    }
    joint_publisher_->publish(std::move(state));
    return true;
  }

  void record_request(const std::string & action, const std::string & owner, std::int64_t stamp)
  {
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    last_action_ = action;
    last_owner_ = owner;
    last_request_stamp_ns_ = stamp;
    last_reason_ = "accepted";
    last_committed_ = false;
    diagnostic_dirty_.store(true);
  }

  void record_rejection(
    const std::string & action, const std::string & owner, const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    last_action_ = action;
    last_owner_ = owner;
    last_reason_ = reason;
    last_committed_ = false;
    diagnostic_dirty_.store(true);
  }

  void record_terminal(const CommandResult & result)
  {
    std::lock_guard<std::mutex> lock(diagnostic_mutex_);
    last_committed_ = result.outcome == CommandResult::Outcome::completed;
    last_reason_ = result.reason;
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
    const auto now_controller = health.controller_now_ns;
    const auto left_age = now_controller >= health.snapshot.arm[0].t_ns ?
      now_controller - health.snapshot.arm[0].t_ns : 0U;
    const auto right_age = now_controller >= health.snapshot.arm[1].t_ns ?
      now_controller - health.snapshot.arm[1].t_ns : 0U;
    const auto skew = health.snapshot.arm[0].t_ns > health.snapshot.arm[1].t_ns ?
      health.snapshot.arm[0].t_ns - health.snapshot.arm[1].t_ns :
      health.snapshot.arm[1].t_ns - health.snapshot.arm[0].t_ns;
    status.values = {
      field("backend", "virtual"),
      field("virtual_execution_enabled", "true"),
      field("physical_motion_authorized", "false"),
      field("collision_policy", "virtual_unchecked"),
      field("collision_checked", "false"),
      field("orientation_constrained", "false"),
      field("state_source", "oa_snapshot_encoder_feedback"),
      field("adapter_state", VirtualControlSession::adapter_state_name(health.adapter_state)),
      field("lifecycle", std::to_string(health.snapshot.lifecycle)),
      field("executing", boolean(health.command_id != 0U)),
      field("active_owner", health.owner),
      field("command_id", std::to_string(health.command_id)),
      field("last_event", std::to_string(health.last_event)),
      field("last_cause", std::to_string(health.last_cause)),
      field("manifest_revision", std::to_string(health.snapshot.manifest_revision)),
      field("model_revision", std::to_string(health.snapshot.model_revision)),
      field("collision_scene_revision", "1"),
      field("verify_epoch", std::to_string(health.verify_epoch)),
      field("left_expected_mask", std::to_string(health.snapshot.arm[0].expected_mask)),
      field("left_fresh_mask", std::to_string(health.snapshot.arm[0].fresh_mask)),
      field("left_fault_mask", std::to_string(health.snapshot.arm[0].fault_mask)),
      field("right_expected_mask", std::to_string(health.snapshot.arm[1].expected_mask)),
      field("right_fresh_mask", std::to_string(health.snapshot.arm[1].fresh_mask)),
      field("right_fault_mask", std::to_string(health.snapshot.arm[1].fault_mask)),
      field("left_feedback_seq", std::to_string(health.snapshot.arm[0].feedback_seq)),
      field("right_feedback_seq", std::to_string(health.snapshot.arm[1].feedback_seq)),
      field("left_feedback_t_ns", std::to_string(health.snapshot.arm[0].t_ns)),
      field("right_feedback_t_ns", std::to_string(health.snapshot.arm[1].t_ns)),
      field("left_feedback_age_ns", std::to_string(left_age)),
      field("right_feedback_age_ns", std::to_string(right_age)),
      field("pair_skew_ns", std::to_string(skew))};
    {
      std::lock_guard<std::mutex> lock(diagnostic_mutex_);
      status.values.push_back(field("last_action", last_action_));
      status.values.push_back(field("last_goal_id", last_owner_));
      status.values.push_back(field("request_stamp_ns", std::to_string(last_request_stamp_ns_)));
      status.values.push_back(field("committed", boolean(last_committed_)));
      status.values.push_back(field("reason", last_reason_));
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
  std::string last_action_{"startup"};
  std::string last_owner_;
  std::string last_reason_{"ready"};
  std::int64_t last_request_stamp_ns_{};
  bool last_committed_{};
  std::mutex clock_mutex_;
  std::int64_t last_ros_now_ns_{};
  std::uint64_t last_controller_now_ns_{};
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
