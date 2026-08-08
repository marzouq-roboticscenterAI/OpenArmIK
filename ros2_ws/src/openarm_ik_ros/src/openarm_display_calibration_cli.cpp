// SPDX-License-Identifier: Apache-2.0
// Compiled operator client for the read-only real-arm RViz calibration layer.
#include <openarm_control_msgs/srv/adjust_display_joint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <type_traits>

namespace
{
using AdjustDisplayJoint = openarm_control_msgs::srv::AdjustDisplayJoint;
using namespace std::chrono_literals;

static_assert(
  std::is_same_v<decltype(AdjustDisplayJoint::Request{}.value_degrees), double>,
  "display calibration service values must remain IEEE-754 binary64 doubles");

int usage(const char * program)
{
  std::cerr << "Usage:\n"
            << "  " << program << " query robot-left|robot-right J\n"
            << "  " << program << " flip robot-left|robot-right J\n"
            << "  " << program << " offset robot-left|robot-right J DELTA_DEGREES\n"
            << "  " << program << " set-current robot-left|robot-right J DISPLAY_DEGREES\n"
            << "  " << program << " capture-relaxed\n";
  return 2;
}

bool parse_joint(const char * text, std::uint8_t & joint)
{
  char * end = nullptr;
  errno = 0;
  const long value = std::strtol(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0' || value < 1L || value > 7L) {
    return false;
  }
  joint = static_cast<std::uint8_t>(value);
  return true;
}

bool parse_degrees(const char * text, double & degrees)
{
  char * end = nullptr;
  errno = 0;
  degrees = std::strtod(text, &end);
  return errno == 0 && end != text && *end == '\0' && std::isfinite(degrees);
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "capture-relaxed") {
    rclcpp::init(argc, argv);
    const auto node = rclcpp::Node::make_shared("openarm_display_calibration_cli");
    const auto client = node->create_client<std_srvs::srv::Trigger>(
      "/openarm_real/capture_zero");
    if (!client->wait_for_service(3s)) {
      std::cerr << "The read-only real-arm observer is not running.\n";
      rclcpp::shutdown();
      return 1;
    }
    auto future = client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    if (rclcpp::spin_until_future_complete(node, future, 5s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      std::cerr << "Timed out while capturing the relaxed display reference.\n";
      rclcpp::shutdown();
      return 1;
    }
    const auto response = future.get();
    std::cout << response->message << '\n';
    rclcpp::shutdown();
    return response->success ? 0 : 1;
  }
  if (argc < 4 || argc > 5) {
    return usage(argv[0]);
  }
  const std::string operation = argv[1];
  const std::string side = argv[2];
  if (side != "robot-left" && side != "robot-right") {
    return usage(argv[0]);
  }

  auto request = std::make_shared<AdjustDisplayJoint::Request>();
  request->side = side;
  if (!parse_joint(argv[3], request->joint)) {
    return usage(argv[0]);
  }
  if (operation == "query" && argc == 4) {
    request->operation = AdjustDisplayJoint::Request::QUERY;
  } else if (operation == "flip" && argc == 4) {
    request->operation = AdjustDisplayJoint::Request::FLIP_DIRECTION;
  } else if (operation == "offset" && argc == 5) {
    request->operation = AdjustDisplayJoint::Request::ADD_OFFSET_DEGREES;
  } else if (operation == "set-current" && argc == 5) {
    request->operation = AdjustDisplayJoint::Request::SET_CURRENT_DEGREES;
  } else {
    return usage(argv[0]);
  }
  if (argc == 5 && !parse_degrees(argv[4], request->value_degrees)) {
    return usage(argv[0]);
  }

  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared("openarm_display_calibration_cli");
  const auto client = node->create_client<AdjustDisplayJoint>(
    "/openarm_real/adjust_display_joint");
  if (!client->wait_for_service(3s)) {
    std::cerr << "The read-only real-arm observer is not running.\n";
    rclcpp::shutdown();
    return 1;
  }
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, 5s) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    std::cerr << "Timed out waiting for the display-calibration response.\n";
    rclcpp::shutdown();
    return 1;
  }
  const auto response = future.get();
  std::cout << response->message << '\n'
            << "  reference: " << response->reference_degrees << " deg\n"
            << "  raw encoder: ";
  if (response->has_live_reading) {
    std::cout << response->raw_encoder_degrees << " deg\n"
              << "  displayed: " << response->displayed_degrees << " deg\n";
  } else {
    std::cout << "no live reading\n"
              << "  displayed: no live reading\n";
  }
  rclcpp::shutdown();
  return response->success ? 0 : 1;
}
