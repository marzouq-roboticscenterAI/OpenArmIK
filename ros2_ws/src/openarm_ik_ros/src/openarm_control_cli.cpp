// SPDX-License-Identifier: Apache-2.0
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <openarm_control_msgs/action/move_joint.hpp>
#include <openarm_control_msgs/action/move_paired_tcp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <future>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
using namespace std::chrono_literals;
using MoveJoint = openarm_control_msgs::action::MoveJoint;
using MovePairedTcp = openarm_control_msgs::action::MovePairedTcp;

#ifndef OPENARM_CLI_RESULT_TIMEOUT_MS
#define OPENARM_CLI_RESULT_TIMEOUT_MS 45000
#endif
#ifndef OPENARM_CLI_CANCEL_TIMEOUT_MS
#define OPENARM_CLI_CANCEL_TIMEOUT_MS 3000
#endif
#ifndef OPENARM_CLI_CANCEL_RESULT_TIMEOUT_MS
#define OPENARM_CLI_CANCEL_RESULT_TIMEOUT_MS 5000
#endif

constexpr auto kResultTimeout = std::chrono::milliseconds(OPENARM_CLI_RESULT_TIMEOUT_MS);
constexpr auto kCancelTimeout = std::chrono::milliseconds(OPENARM_CLI_CANCEL_TIMEOUT_MS);
constexpr auto kCancelResultTimeout =
  std::chrono::milliseconds(OPENARM_CLI_CANCEL_RESULT_TIMEOUT_MS);
constexpr auto kWaitPollInterval = 50ms;
constexpr auto kServerLossGrace = 750ms;

static_assert(kResultTimeout > 0ms);
static_assert(kCancelTimeout > 0ms);
static_assert(kCancelResultTimeout > 0ms);

enum class PostAcceptanceWait
{
  awaited_ready,
  terminal_ready,
  timeout,
  server_lost,
  context_shutdown
};

template<typename Future>
bool future_ready(const Future & future)
{
  return future.wait_for(0ms) == std::future_status::ready;
}

template<typename AwaitedFuture, typename TerminalFuture, typename Client>
PostAcceptanceWait wait_after_acceptance(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const AwaitedFuture & awaited_future,
  const TerminalFuture & terminal_future,
  const std::shared_ptr<Client> & client,
  const std::shared_ptr<rclcpp::Context> & context,
  const std::chrono::steady_clock::duration timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::optional<std::chrono::steady_clock::time_point> server_absent_since;
  const auto ready = [&]() -> std::optional<PostAcceptanceWait> {
      if (future_ready(terminal_future)) {
        return PostAcceptanceWait::terminal_ready;
      }
      if (future_ready(awaited_future)) {
        return PostAcceptanceWait::awaited_ready;
      }
      return std::nullopt;
    };
  const auto spin_once = [&](const std::chrono::steady_clock::duration duration) {
      try {
        executor.spin_once(duration);
      } catch (...) {
        if (ready() || (context && !context->is_valid())) {
          return;
        }
        throw;
      }
    };
  for (;;) {
    if (const auto state = ready()) {
      return *state;
    }
    if (!context || !context->is_valid()) {
      return PostAcceptanceWait::context_shutdown;
    }
    const auto before_spin = std::chrono::steady_clock::now();
    if (before_spin >= deadline) {
      spin_once(0ms);
      if (const auto state = ready()) {
        return *state;
      }
      return PostAcceptanceWait::timeout;
    }
    const auto slice = std::min(
      std::chrono::steady_clock::duration(kWaitPollInterval), deadline - before_spin);
    spin_once(slice);
    if (const auto state = ready()) {
      return *state;
    }
    if (!context->is_valid()) {
      return PostAcceptanceWait::context_shutdown;
    }

    bool server_ready = false;
    try {
      server_ready = client->action_server_is_ready();
    } catch (...) {
      if (!context->is_valid()) {
        return PostAcceptanceWait::context_shutdown;
      }
    }
    const auto after_spin = std::chrono::steady_clock::now();
    if (server_ready) {
      server_absent_since.reset();
    } else if (!server_absent_since) {
      server_absent_since = after_spin;
    } else if (after_spin - *server_absent_since >= kServerLossGrace) {
      spin_once(0ms);
      if (const auto state = ready()) {
        return *state;
      }
      return PostAcceptanceWait::server_lost;
    }
  }
}

int report_post_acceptance_loss(const PostAcceptanceWait outcome)
{
  if (outcome == PostAcceptanceWait::server_lost) {
    std::cerr << "action server lost after goal acceptance\n";
    return 8;
  }
  if (outcome == PostAcceptanceWait::context_shutdown) {
    std::cerr << "ROS context shut down after goal acceptance\n";
    return 8;
  }
  return -1;
}

template<typename Action>
int report_terminal_result(
  const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult & wrapped)
{
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
    wrapped.result->outcome != Action::Result::OUTCOME_COMPLETED)
  {
    std::cerr << wrapped.result->reason << '\n';
    return wrapped.code == rclcpp_action::ResultCode::CANCELED ? 6 : 7;
  }
  std::cout << "completed command_id=" << wrapped.result->command_id << '\n';
  return 0;
}

template<typename Action, typename ResultFuture>
std::optional<int> consume_terminal_if_ready(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const ResultFuture & result_future,
  const std::shared_ptr<rclcpp::Context> & context)
{
  if (!future_ready(result_future) && context && context->is_valid()) {
    try {
      executor.spin_once(0ms);
    } catch (...) {
      if (context->is_valid()) {
        throw;
      }
    }
  }
  if (!future_ready(result_future)) {
    return std::nullopt;
  }
  return report_terminal_result<Action>(result_future.get());
}

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
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto goal_future = client->async_send_goal(goal);
  if (executor.spin_until_future_complete(goal_future, 3s) !=
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
  const auto context = node->get_node_base_interface()->get_context();
#ifdef OPENARM_CLI_TEST_CONTEXT_SHUTDOWN_AFTER_ACCEPT
  rclcpp::TimerBase::SharedPtr context_shutdown_timer;
  if (std::getenv("OPENARM_CLI_TEST_CONTEXT_SHUTDOWN") != nullptr) {
    context_shutdown_timer = node->create_wall_timer(100ms, [context]() {
        if (context->is_valid()) {
          context->shutdown("isolated CLI context-shutdown test");
        }
      });
  }
  (void)context_shutdown_timer;
#endif
  auto result_future = client->async_get_result(handle);
  auto wait = wait_after_acceptance(
    executor, result_future, result_future, client, context, kResultTimeout);
  if (const auto terminal = consume_terminal_if_ready<Action>(
      executor, result_future, context))
  {
    return *terminal;
  }
  if (const int loss = report_post_acceptance_loss(wait); loss >= 0) {
    return loss;
  }
  if (wait == PostAcceptanceWait::timeout) {
    auto cancel_future = client->async_cancel_goal(handle);
    wait = wait_after_acceptance(
      executor, cancel_future, result_future, client, context, kCancelTimeout);
    if (const auto terminal = consume_terminal_if_ready<Action>(
        executor, result_future, context))
    {
      return *terminal;
    }
    if (const int loss = report_post_acceptance_loss(wait); loss >= 0) {
      return loss;
    }
    if (wait == PostAcceptanceWait::timeout) {
      std::cerr << "terminal result timeout; cancel response timeout\n";
      return 5;
    }
    wait = wait_after_acceptance(
      executor, result_future, result_future, client, context, kCancelResultTimeout);
    if (const auto terminal = consume_terminal_if_ready<Action>(
        executor, result_future, context))
    {
      return *terminal;
    }
    if (const int loss = report_post_acceptance_loss(wait); loss >= 0) {
      return loss;
    }
    if (wait == PostAcceptanceWait::timeout) {
      std::cerr << "terminal result timeout; cancellation terminal unconfirmed\n";
      return 5;
    }
  }
  throw std::runtime_error("terminal future readiness was lost");
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
