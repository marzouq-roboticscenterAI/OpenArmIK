// SPDX-License-Identifier: Apache-2.0
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <openarm_control_msgs/action/move_joint.hpp>
#include <openarm_control_msgs/action/move_paired_tcp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
using namespace std::chrono_literals;
using MoveJoint = openarm_control_msgs::action::MoveJoint;
using MovePairedTcp = openarm_control_msgs::action::MovePairedTcp;

double number(const char * text)
{
  char * end = nullptr;
  const double value = std::strtod(text, &end);
  if (end == text || *end != '\0' || !std::isfinite(value)) {
    throw std::invalid_argument("invalid finite number: " + std::string(text));
  }
  return value;
}

int status(const std::shared_ptr<rclcpp::Node> & node)
{
  bool received = false;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto subscription = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/openarm_ik/diagnostics", rclcpp::QoS(10).reliable(),
    [&received](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
      for (const auto & item : message->status) {
        if (item.name != "openarm_ik_ros/virtual_control") {
          continue;
        }
        std::cout << item.message << '\n';
        for (const auto & value : item.values) {
          std::cout << value.key << '=' << value.value << '\n';
        }
        received = true;
      }
    });
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (rclcpp::ok() && !received && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  return received ? 0 : 2;
}

template<typename Action>
int run_goal(
  const std::shared_ptr<rclcpp::Node> & node,
  const typename Action::Goal & goal, const std::string & action_name)
{
  auto client = rclcpp_action::create_client<Action>(node, action_name);
  if (!client->wait_for_action_server(3s)) {
    std::cerr << "action server unavailable\n";
    return 2;
  }
  auto goal_future = client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node, goal_future, 3s) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    std::cerr << "goal response timeout\n";
    return 3;
  }
  const auto handle = goal_future.get();
  if (!handle) {
    std::cerr << "goal rejected\n";
    return 4;
  }
  auto result_future = client->async_get_result(handle);
  if (rclcpp::spin_until_future_complete(node, result_future, 45s) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto cancel_future = client->async_cancel_goal(handle);
    if (rclcpp::spin_until_future_complete(node, cancel_future, 3s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      std::cerr << "terminal result timeout; cancel response timeout\n";
      return 5;
    }
    if (rclcpp::spin_until_future_complete(node, result_future, 5s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      std::cerr << "terminal result timeout; cancellation terminal unconfirmed\n";
      return 5;
    }
  }
  const auto wrapped = result_future.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
    wrapped.result->outcome != Action::Result::OUTCOME_COMPLETED)
  {
    std::cerr << wrapped.result->reason << '\n';
    return wrapped.code == rclcpp_action::ResultCode::CANCELED ? 6 : 7;
  }
  std::cout << "completed command_id=" << wrapped.result->command_id << '\n';
  return 0;
}

void usage()
{
  std::cerr << "Usage:\n"
    "  openarm_control_cli status\n"
    "  openarm_control_cli move-joint JOINT_NAME TARGET_RAD\n"
    "  openarm_control_cli move-paired-tcp FRAME LEFT_X LEFT_Y LEFT_Z RIGHT_X RIGHT_Y RIGHT_Z\n";
}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 1;
  try {
    auto node = std::make_shared<rclcpp::Node>("openarm_control_cli");
    if (argc == 2 && std::string(argv[1]) == "status") {
      result = status(node);
    } else if (argc == 4 && std::string(argv[1]) == "move-joint") {
      MoveJoint::Goal goal;
      goal.stamp = node->now();
      goal.joint_name = argv[2];
      goal.target_rad = number(argv[3]);
      result = run_goal<MoveJoint>(node, goal, "/openarm_ik/move_joint");
    } else if (argc == 9 && std::string(argv[1]) == "move-paired-tcp") {
      MovePairedTcp::Goal goal;
      goal.header.stamp = node->now();
      goal.header.frame_id = argv[2];
      goal.left_tcp_m.x = number(argv[3]);
      goal.left_tcp_m.y = number(argv[4]);
      goal.left_tcp_m.z = number(argv[5]);
      goal.right_tcp_m.x = number(argv[6]);
      goal.right_tcp_m.y = number(argv[7]);
      goal.right_tcp_m.z = number(argv[8]);
      result = run_goal<MovePairedTcp>(node, goal, "/openarm_ik/move_paired_tcp");
    } else {
      usage();
    }
  } catch (const std::exception & error) {
    std::cerr << "openarm_control_cli: " << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
