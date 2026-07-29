// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/paired_transaction.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
using openarm_ik_ros::PairedTarget;
using openarm_ik_ros::PairedTransactionProcessor;

class OpenArmIkRosNode final : public rclcpp::Node
{
public:
  OpenArmIkRosNode()
  : Node("openarm_ik_ros"),
    processor_(openarm_ik_ros::kMinimumExpiryMilliseconds * 1000000LL)
  {
    const auto expiry_ms = declare_parameter<std::int64_t>("request_expiry_ms", 1000LL);
    if (expiry_ms < openarm_ik_ros::kMinimumExpiryMilliseconds ||
      expiry_ms > openarm_ik_ros::kMaximumExpiryMilliseconds)
    {
      throw std::invalid_argument("request_expiry_ms must be in [1, 60000]");
    }
    processor_ = PairedTransactionProcessor(expiry_ms * 1000000LL);
    names_ = processor_.joint_names();
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    joint_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/openarm_ik/diagnostics", qos);
    target_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/openarm_ik/paired_xyz", qos,
      std::bind(&OpenArmIkRosNode::on_target, this, std::placeholders::_1));
    publish_state();
    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&OpenArmIkRosNode::publish_state, this));
    RCLCPP_WARN(
      get_logger(),
      "Virtual visualization only: position-only IK leaves orientation free and performs no collision checking.");
  }

private:
  static diagnostic_msgs::msg::KeyValue field(std::string key, std::string value)
  {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = std::move(key);
    pair.value = std::move(value);
    return pair;
  }

  static std::string scalar(const double value)
  {
    std::ostringstream out;
    out.precision(17);
    out << value;
    return out.str();
  }

  static std::string tcp_xyz(const oa_ik_diagnostics & result)
  {
    std::ostringstream out;
    out.precision(17);
    out << result.achieved_hand_tcp.m[3] << ',' << result.achieved_hand_tcp.m[7] << ',' <<
      result.achieved_hand_tcp.m[11];
    return out.str();
  }

  static std::string tcp_matrix(const oa_ik_diagnostics & result)
  {
    std::ostringstream out;
    out.precision(17);
    for (std::size_t index = 0; index < 16U; ++index) {
      if (index != 0U) {
        out << ',';
      }
      out << result.achieved_hand_tcp.m[index];
    }
    return out.str();
  }

  void on_target(const geometry_msgs::msg::PoseArray::SharedPtr message)
  {
    PairedTarget request;
    request.pose_count = message->poses.size();
    request.frame_id = message->header.frame_id;
    request.stamp_nanoseconds = rclcpp::Time(message->header.stamp).nanoseconds();
    if (message->poses.size() == 2U) {
      for (std::size_t axis = 0; axis < 3U; ++axis) {
        request.left[axis] = axis == 0U ? message->poses[0].position.x :
          (axis == 1U ? message->poses[0].position.y : message->poses[0].position.z);
        request.right[axis] = axis == 0U ? message->poses[1].position.x :
          (axis == 1U ? message->poses[1].position.y : message->poses[1].position.z);
      }
    }
    const auto result = processor_.process(request, now().nanoseconds());
    publish_diagnostics(result);
    if (result.committed) {
      publish_state();
    }
  }

  void publish_state()
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    message.name = names_;
    message.position.reserve(names_.size());
    for (const double position : processor_.left_q()) {
      message.position.push_back(position);
    }
    for (const double position : processor_.right_q()) {
      message.position.push_back(position);
    }
    message.position.push_back(0.0);
    message.position.push_back(0.0);
    joint_publisher_->publish(std::move(message));
  }

  void publish_diagnostics(const openarm_ik_ros::TransactionResult & result)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "openarm_ik_ros/paired_position_ik";
    status.hardware_id = "virtual";
    status.level = result.committed ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = result.reason;
    status.values = {
      field("sequence", std::to_string(result.sequence)),
      field("committed", result.committed ? "true" : "false"),
      field("achieved_available", result.achieved_available ? "true" : "false"),
      field("backend", "virtual"),
      field("collision_checked", "false"),
      field("orientation", "free"),
      field("redundancy_policy", PairedTransactionProcessor::continuity_policy()),
      field("continuity_seed", "last_committed_joint_posture"),
      field("position_tolerance_m", "0.0001"),
      field("max_joint_step_rad", "0.15"),
      field("damping_min", "0.0001"),
      field("damping_max", "0.3"),
      field("posture_weight", "1.0_each_joint"),
      field("max_iterations", "300"),
      field("left_status", std::to_string(result.left.status)),
      field("right_status", std::to_string(result.right.status))};
    if (result.achieved_available) {
      status.values.push_back(field("left_residual_m", scalar(result.left.position_error_m)));
      status.values.push_back(field("right_residual_m", scalar(result.right.position_error_m)));
      status.values.push_back(field("left_achieved_hand_tcp_xyz", tcp_xyz(result.left)));
      status.values.push_back(field("right_achieved_hand_tcp_xyz", tcp_xyz(result.right)));
      status.values.push_back(field("left_achieved_hand_tcp_matrix", tcp_matrix(result.left)));
      status.values.push_back(field("right_achieved_hand_tcp_matrix", tcp_matrix(result.right)));
    }
    diagnostic_msgs::msg::DiagnosticArray report;
    report.header.stamp = now();
    report.status.push_back(std::move(status));
    diagnostics_publisher_->publish(std::move(report));
  }

  PairedTransactionProcessor processor_;
  std::vector<std::string> names_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr target_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenArmIkRosNode>());
  rclcpp::shutdown();
  return 0;
}
