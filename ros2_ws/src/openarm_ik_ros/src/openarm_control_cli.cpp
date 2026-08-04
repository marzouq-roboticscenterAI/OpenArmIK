// SPDX-License-Identifier: Apache-2.0
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <openarm_control_msgs/action/move_joint.hpp>
#include <openarm_control_msgs/action/move_paired_tcp.hpp>
#include <openarm_control_msgs/action/move_paired_tcp_scaled.hpp>
#include <openarm_runtime_motion.h>
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
using MovePairedTcpScaled = openarm_control_msgs::action::MovePairedTcpScaled;

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

// ---------------------------------------------------------------- demos
//
// Every waypoint below was measured against the real-time keepout monitor
// rather than guessed. The recorded minimum clearances are in the comments;
// the planning gate is 25 mm and the monitor intervenes at 10 mm.
struct Waypoint
{
  const char * label;
  double left[3];
  double right[3];
};

int run_sequence(
  const rclcpp::Node::SharedPtr & node, const Waypoint * steps, const std::size_t count,
  const double scale)
{
  for (std::size_t index = 0; index < count; ++index) {
    MovePairedTcpScaled::Goal goal;
    goal.header.stamp = node->now();
    goal.header.frame_id = "openarm_body_link0";
    goal.left_tcp_m.x = steps[index].left[0];
    goal.left_tcp_m.y = steps[index].left[1];
    goal.left_tcp_m.z = steps[index].left[2];
    goal.right_tcp_m.x = steps[index].right[0];
    goal.right_tcp_m.y = steps[index].right[1];
    goal.right_tcp_m.z = steps[index].right[2];
    goal.motion_limit_scale = scale;
    std::cout << "step " << (index + 1) << "/" << count << ": " << steps[index].label
              << std::endl;
    const int result = run_goal<MovePairedTcpScaled>(
      node, goal, "/openarm_ik/move_paired_tcp_scaled");
    if (result != 0) {
      std::cerr << "sequence stopped at step " << (index + 1) << " (" <<
        steps[index].label << ")\n";
      return result;
    }
  }
  return 0;
}

// Both claws swing together and apart in the frontal plane. At the closed
// waypoint the claws are 24 cm apart with about 31 mm of measured clearance,
// so they come visibly close without the keepout monitor intervening.
int demo_clap(const rclcpp::Node::SharedPtr & node, const int cycles)
{
  static const Waypoint open_pose{"open", {0.30, 0.26, 0.35}, {0.30, -0.26, 0.35}};
  static const Waypoint closed_pose{"clap", {0.30, 0.12, 0.35}, {0.30, -0.12, 0.35}};
  int result = run_sequence(node, &open_pose, 1, 1.0);
  if (result != 0) {return result;}
  for (int cycle = 0; cycle < cycles; ++cycle) {
    const Waypoint beat[] = {closed_pose, open_pose};
    result = run_sequence(node, beat, 2, 1.0);
    if (result != 0) {return result;}
  }
  return 0;
}

// The arms reach across one another at separated heights, left high and right
// low. A true crossing is not reachable on this robot: each tool is modelled as
// a 75 mm capsule, so the two tools need 175 mm of centre separation, and every
// geometry that carries a claw past the centreline brings the two tool segments
// inside that. The deepest approach that clears is 4 cm either side of the
// centreline, measured at about 22 mm of clearance.
int demo_cross(const rclcpp::Node::SharedPtr & node, const int cycles)
{
  static const Waypoint open_pose{"open", {0.30, 0.26, 0.45}, {0.30, -0.26, 0.45}};
  static const Waypoint split{"split height", {0.30, 0.26, 0.58}, {0.30, -0.26, 0.32}};
  static const Waypoint crossed{"reach across", {0.30, 0.04, 0.58}, {0.30, -0.04, 0.32}};
  int result = run_sequence(node, &open_pose, 1, 1.0);
  if (result != 0) {return result;}
  for (int cycle = 0; cycle < cycles; ++cycle) {
    const Waypoint pass[] = {split, crossed, split};
    result = run_sequence(node, pass, 3, 1.0);
    if (result != 0) {return result;}
  }
  return run_sequence(node, &open_pose, 1, 1.0);
}

// Pick the scene box up, carry it, and set it down.
//
// The node grasps the box when the claws close to under 26 cm around it and
// releases when they open past 34 cm, both judged from measured forward
// kinematics. The waypoints below therefore approach wide, close on the box at
// its resting pose, lift, traverse, lower, and open.
int demo_pick_place(const rclcpp::Node::SharedPtr & node)
{
  static const Waypoint steps[] = {
    {"approach the box, claws wide", {0.34, 0.24, 0.30}, {0.34, -0.24, 0.30}},
    {"close on the box", {0.34, 0.11, 0.30}, {0.34, -0.11, 0.30}},
    {"lift", {0.34, 0.11, 0.48}, {0.34, -0.11, 0.48}},
    {"carry across", {0.26, 0.11, 0.50}, {0.26, -0.11, 0.50}},
    {"lower to place", {0.26, 0.11, 0.32}, {0.26, -0.11, 0.32}},
    {"release", {0.26, 0.24, 0.32}, {0.26, -0.24, 0.32}},
    {"withdraw", {0.24, 0.26, 0.42}, {0.24, -0.26, 0.42}},
  };
  return run_sequence(node, steps, sizeof(steps) / sizeof(steps[0]), 1.0);
}

// One claw is commanded and the other mirrors it across the body sagittal
// plane, matching oa_controller_plan_mirrored_tcp.
int demo_mirror(
  const rclcpp::Node::SharedPtr & node, const std::string & lead, const double x,
  const double y, const double z)
{
  Waypoint step{"mirrored", {x, y, z}, {x, -y, z}};
  if (lead == "right") {
    step.left[1] = -y;
    step.right[1] = y;
  }
  return run_sequence(node, &step, 1, 1.0);
}

void usage()
{
  std::cerr << "Usage:\n"
    "  openarm_control_cli status\n"
    "  openarm_control_cli move-joint JOINT_NAME TARGET_RAD\n"
    "  openarm_control_cli move-paired-tcp FRAME LEFT_X_METRES LEFT_Y_METRES LEFT_Z_METRES "
    "RIGHT_X_METRES RIGHT_Y_METRES RIGHT_Z_METRES\n"
    "  openarm_control_cli mirror left|right X_METRES Y_METRES Z_METRES\n"
    "  openarm_control_cli clap [CYCLES]\n"
    "  openarm_control_cli estop | estop-release\n"
    "  openarm_control_cli cross [CYCLES]\n"
    "  openarm_control_cli pick-place\n";
}
}

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {
    usage();
    return 0;
  }
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
    } else if (argc == 2 && std::string(argv[1]) == "estop") {
      // Lock-free and handle-free: it does not need the action server to be up.
      oa_runtime_estop_assert();
      std::cout << "emergency stop engaged (assertions="
                << oa_runtime_estop_assert_count()
                << "); software interlock, not a hardwired E-stop" << std::endl;
      result = oa_runtime_estop_asserted() != 0U ? 0 : 1;
    } else if (argc == 2 && std::string(argv[1]) == "estop-release") {
      result = oa_runtime_estop_clear() == OA_RUNTIME_OK ? 0 : 1;
      std::cout << (oa_runtime_estop_asserted() == 0U ? "emergency stop released"
                                                      : "emergency stop still engaged")
                << std::endl;
    } else if ((argc == 2 || argc == 3) && std::string(argv[1]) == "clap") {
      result = demo_clap(node, argc == 3 ? static_cast<int>(number(argv[2])) : 3);
    } else if ((argc == 2 || argc == 3) && std::string(argv[1]) == "cross") {
      result = demo_cross(node, argc == 3 ? static_cast<int>(number(argv[2])) : 2);
    } else if (argc == 2 && std::string(argv[1]) == "pick-place") {
      result = demo_pick_place(node);
    } else if (argc == 6 && std::string(argv[1]) == "mirror") {
      const std::string lead = argv[2];
      if (lead != "left" && lead != "right") {
        std::cerr << "mirror lead must be left or right\n";
        result = 2;
      } else {
        result = demo_mirror(node, lead, number(argv[3]), number(argv[4]), number(argv[5]));
      }
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
