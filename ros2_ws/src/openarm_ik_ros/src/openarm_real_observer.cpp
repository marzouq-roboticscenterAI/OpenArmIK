// SPDX-License-Identifier: Apache-2.0
//
// Read-only observer for a physically connected OpenArm v1.0.
//
// This node exists for one job: show what the real arms are doing. It reads
// DaMiao motor feedback from can0 and can1, works out which arm is on which
// interface, and publishes /joint_states so robot_state_publisher and RViz
// mirror the hardware.
//
// It cannot move the arms, and that is enforced structurally rather than by
// convention. The only frames it will transmit are the two read-only DaMiao
// primitives, refresh-status and register-query; transmit() rejects anything
// else before it reaches the socket. There is no path here to enable a motor,
// write a register, set a zero, or send a motion command.
//
// It also starts passive. Nothing is opened and nothing is transmitted until
// something calls the connect service, so plugging in and launching the stack
// is inert.
//
// What this node does NOT establish, and must not be read as establishing:
//   - that the motor zero positions correspond to the URDF zero pose. Nothing
//     here commissions a zero, a direction, or a gear ratio. Joint angles are
//     published under the identity mapping unless one is supplied, so a pose
//     that looks wrong in RViz means the mapping needs commissioning, not that
//     the reading failed.
//   - that the arm assignment is safe to command from. It is good enough to
//     label a view; it is not a commissioning record.
#include "openarm_ik_ros/real_observer_core.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace
{
using namespace std::chrono_literals;
using openarm_ik_ros::real::ArmAssignment;
using openarm_ik_ros::real::BusReading;
using openarm_ik_ros::real::ObserverConfig;
using openarm_ik_ros::real::RealObserver;

class RealObserverNode : public rclcpp::Node
{
public:
  RealObserverNode()
  : rclcpp::Node("openarm_real_observer")
  {
    ObserverConfig config;
    config.interfaces[0] = declare_parameter<std::string>("interface_a", "can0");
    config.interfaces[1] = declare_parameter<std::string>("interface_b", "can1");
    config.first_send_id =
      static_cast<std::uint16_t>(declare_parameter<int>("first_send_id", 0x01));
    config.last_send_id =
      static_cast<std::uint16_t>(declare_parameter<int>("last_send_id", 0x0E));
    config.receive_id_offset =
      static_cast<std::uint16_t>(declare_parameter<int>("receive_id_offset", 0x10));
    config.reply_timeout_ms = declare_parameter<int>("reply_timeout_ms", 40);
    // "left" or "right" pins the assignment when the automatic methods cannot
    // separate the two buses. Empty leaves it automatic.
    config.forced_side_for_interface_a =
      declare_parameter<std::string>("interface_a_side", "");

    observer_ = std::make_unique<RealObserver>(config);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    joint_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/openarm_real/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    connect_service_ = create_service<std_srvs::srv::Trigger>(
      "/openarm_real/connect",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        std::string detail;
        const bool ok = observer_->connect(detail);
        response->success = ok;
        response->message = detail;
        publish_status();
        RCLCPP_INFO(get_logger(), "connect requested: %s", detail.c_str());
      });
    disconnect_service_ = create_service<std_srvs::srv::Trigger>(
      "/openarm_real/disconnect",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        observer_->disconnect();
        response->success = true;
        response->message = "disconnected; sockets closed and publishing stopped";
        publish_status();
        RCLCPP_INFO(get_logger(), "disconnected");
      });

    timer_ = create_wall_timer(10ms, [this]() {poll();});
    publish_status();
    RCLCPP_WARN(
      get_logger(),
      "Real-arm observer is READ ONLY and starts passive. It transmits only "
      "DaMiao refresh-status and register-query frames and has no path to "
      "enable, zero, or move a motor. Joint angles use an uncommissioned "
      "mapping: a pose that looks wrong in RViz means the mapping needs "
      "commissioning.");
  }

private:
  void poll()
  {
    // Being passive means sending nothing on the CAN bus. It does not mean
    // going quiet on ROS: robot_state_publisher derives TF from /joint_states,
    // so if this stops publishing, the robot vanishes from RViz entirely and an
    // idle stack looks like a broken one. Publish the rest pose until there is
    // something measured to show.
    std::array<BusReading, 2> readings{};
    const bool measured = observer_->connected() && observer_->read_once(readings);

    if (observer_->connected() && !measured) {
      ++consecutive_failures_;
      if (consecutive_failures_ == 50U) {
        RCLCPP_WARN(get_logger(), "no complete reading for 500 ms: %s",
          observer_->detail().c_str());
        publish_status();
      }
    } else {
      consecutive_failures_ = 0U;
    }

    publish_joint_state(readings, measured);
    if (++status_divider_ >= 100U) {
      status_divider_ = 0U;
      publish_status();
    }
  }

  void publish_joint_state(const std::array<BusReading, 2> & readings, const bool measured)
  {
    const ArmAssignment assignment = observer_->assignment();
    // An unresolved assignment means we do not know which arm is which, so
    // measured angles would be as likely to land on the wrong side as the
    // right one. Fall back to the rest pose rather than guess.
    const bool usable = measured && assignment.resolved;
    sensor_msgs::msg::JointState state;
    state.header.stamp = now();
    const auto & names = openarm_ik_ros::real::canonical_joint_names();
    state.name.assign(names.begin(), names.end());
    state.position.assign(names.size(), 0.0);
    state.velocity.assign(names.size(), 0.0);
    state.effort.assign(names.size(), 0.0);
    if (usable) {
      for (std::size_t bus = 0; bus < 2U; ++bus) {
        if (!readings[bus].complete) {
          continue;   // Unpopulated bus: leave that arm at the rest pose.
        }
        const std::size_t side = assignment.side_of_interface[bus];
        for (std::size_t joint = 0; joint < 7U; ++joint) {
          const std::size_t index = side * 7U + joint;
          state.position[index] = readings[bus].position_rad[joint];
          state.velocity[index] = readings[bus].velocity_rad_s[joint];
          state.effort[index] = readings[bus].torque_nm[joint];
        }
      }
    }
    joint_publisher_->publish(std::move(state));
  }

  void publish_status()
  {
    std_msgs::msg::String message;
    message.data = observer_->status_json();
    status_publisher_->publish(std::move(message));
  }

  std::unique_ptr<RealObserver> observer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  unsigned consecutive_failures_{0};
  unsigned status_divider_{0};
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealObserverNode>());
  rclcpp::shutdown();
  return 0;
}
