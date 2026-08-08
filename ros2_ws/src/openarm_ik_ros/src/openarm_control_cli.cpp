// SPDX-License-Identifier: Apache-2.0
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <openarm_control_msgs/action/move_joint.hpp>
#include <openarm_control_msgs/action/move_gripper.hpp>
#include <openarm_control_msgs/action/move_bimanual.hpp>
#include <openarm_control_msgs/action/move_paired_tcp.hpp>
#include <openarm_control_msgs/action/move_paired_tcp_scaled.hpp>
#include <openarm_collision.h>
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
#include <type_traits>

namespace
{
using namespace std::chrono_literals;
using MoveJoint = openarm_control_msgs::action::MoveJoint;
using MoveGripper = openarm_control_msgs::action::MoveGripper;
using MovePairedTcp = openarm_control_msgs::action::MovePairedTcp;
using MovePairedTcpScaled = openarm_control_msgs::action::MovePairedTcpScaled;
using MoveBimanual = openarm_control_msgs::action::MoveBimanual;

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
  typename Action::Goal goal, const std::string & action_name)
{
  auto client = rclcpp_action::create_client<Action>(node, action_name);
  if (!client->wait_for_action_server(3s)) {
    std::cerr << "action server unavailable\n";
    return 2;
  }
  // DDS discovery can consume most or all of the controller's one-second
  // request-validity window. Every CLI command is a "move now" request, so
  // stamp it only after the server is known rather than in main() before
  // discovery begins.
  if constexpr (std::is_same_v<Action, MoveJoint> || std::is_same_v<Action, MoveGripper>) {
    goal.stamp = node->now();
  } else {
    goal.header.stamp = node->now();
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

int run_bimanual(
  const rclcpp::Node::SharedPtr & node, std::uint8_t mode, double x,
  double y, double z, double stop_distance = -1.0,
  std::uint8_t lead_side = 0U);

// Both claws swing together, enter the narrowly scoped expanded rail envelope,
// then retreat. The physical STL meshes remain separated and every other claw
// pair, arm pair, and the pole stay monitored.
int demo_clap(const rclcpp::Node::SharedPtr & node, const int cycles)
{
  static const Waypoint open_pose{"open", {0.34, 0.22, 0.86}, {0.34, -0.22, 0.86}};
  int result = run_sequence(node, &open_pose, 1, 1.0);
  if (result != 0) {return result;}
  for (int cycle = 0; cycle < cycles; ++cycle) {
    result = run_bimanual(
      node, MoveBimanual::Goal::MODE_CONVERGE, 0.34, 0.0, 0.86,
      0.045 + 0.5 * oa_collision_claw_rail_clearance_m());
    if (result != 0) {return result;}
    result = run_sequence(node, &open_pose, 1, 1.0);
    if (result != 0) {return result;}
  }
  return 0;
}

// A genuine crossing: the left claw ends in the right half of the workspace and
// the right claw in the left half, commanded simultaneously.
//
// The two arms pass stacked in Z rather than side by side. A capsule cannot be
// shrunk to allow a side-by-side pass: the gripper measures 57 x 168 x 16 mm,
// so its true radial extent about the tool axis is 88.8 mm and the two tool
// capsules need 175 mm of centre separation however the approach is staggered
// in x. Separating them vertically instead clears comfortably, measured at
// 26.5 mm at the crossed waypoint.
//
// The window is narrow. Crossing deeper than about 4 cm either side is either
// unreachable or brings the leading tool inside the trailing forearm; pulling
// either arm back in x instead brings its tool near the central shaft.
int demo_cross(const rclcpp::Node::SharedPtr & node, const int cycles)
{
  static const Waypoint open_pose{"open", {0.30, 0.26, 0.45}, {0.30, -0.26, 0.45}};
  // Reaching the stacked pose in one move drops the right arm 0.23 m, which is
  // marginal for the portal guard's branch continuity check, so it is halved.
  static const Waypoint half{"part heights", {0.30, 0.26, 0.55}, {0.30, -0.26, 0.30}};
  static const Waypoint split{"stack: left high, right low",
    {0.30, 0.26, 0.62}, {0.30, -0.26, 0.22}};
  static const Waypoint crossed{"cross: left claw right, right claw left",
    {0.30, -0.04, 0.62}, {0.30, 0.04, 0.22}};
  // The lead-in matters: the guard refuses a single straight line from neutral
  // to the open pose on IK branch continuity, though it accepts it one step at
  // a time.
  static const Waypoint lead_in{"lead-in", {0.30, 0.26, 0.35}, {0.30, -0.26, 0.35}};
  const Waypoint entry[] = {lead_in, open_pose};
  int result = run_sequence(node, entry, 2, 1.0);
  if (result != 0) {return result;}
  for (int cycle = 0; cycle < cycles; ++cycle) {
    const Waypoint pass[] = {half, split, crossed, split, half};
    result = run_sequence(node, pass, 5, 1.0);
    if (result != 0) {return result;}
  }
  const Waypoint exit_steps[] = {open_pose, lead_in};
  return run_sequence(node, exit_steps, 2, 1.0);
}

// Retreat to a wide, guard-clear pose.
//
// converge ends with the claws deliberately close, which is nearer than the
// portal's pre-flight guard will plan out of, so the portal refuses further
// best-effort moves until the arms are separated again. The action path used
// here does not go through that guard, so this always works.
int demo_home(const rclcpp::Node::SharedPtr & node)
{
  static const Waypoint open_pose{"home: claws wide", {0.30, 0.26, 0.45}, {0.30, -0.26, 0.45}};
  return run_sequence(node, &open_pose, 1, 1.0);
}

// Bimanual modes that need measured state or the contact monitor, routed
// through the node's MoveBimanual action rather than resolved here.
int run_bimanual(
  const rclcpp::Node::SharedPtr & node, const std::uint8_t mode, const double x,
  const double y, const double z, const double stop_distance,
  const std::uint8_t lead_side)
{
  MoveBimanual::Goal goal;
  goal.header.stamp = node->now();
  goal.header.frame_id = "openarm_body_link0";
  goal.mode = mode;
  goal.lead_side = lead_side;
  goal.motion_limit_scale = 1.0;
  goal.stop_distance_m = mode == MoveBimanual::Goal::MODE_CONVERGE && stop_distance < 0.0 ?
    oa_collision_tool_radius_m() - 0.002 : stop_distance;
  goal.contact_torque_fraction = 0.0;   // library default
  if (mode == MoveBimanual::Goal::MODE_MIRRORED) {
    goal.left_tcp_m.x = x; goal.left_tcp_m.y = y; goal.left_tcp_m.z = z;
  } else {
    goal.target_m.x = x; goal.target_m.y = y; goal.target_m.z = z;
  }
  const int result = run_goal<MoveBimanual>(node, goal, "/openarm_ik/move_bimanual");
  if (result == 0 && mode == MoveBimanual::Goal::MODE_CONVERGE) {
    std::cout << "converge stopped on proved nominal claw-mesh contact; run 'home' "
                 "or use the portal's guarded retreat before another approach"
              << std::endl;
  }
  return result;
}

// Pick the scene box up, carry it, and set it down.
//
// The node grasps the box when the claws close to under 32 cm around it and
// releases when they open past 40 cm, both judged from measured forward
// kinematics.
int demo_pick_place(const rclcpp::Node::SharedPtr & node)
{
  static const Waypoint steps[] = {
    {"approach the box, claws wide", {0.34, 0.24, 0.30}, {0.34, -0.24, 0.30}},
    {"close on the box", {0.34, 0.15, 0.30}, {0.34, -0.15, 0.30}},
    {"lift", {0.34, 0.15, 0.48}, {0.34, -0.15, 0.48}},
    {"carry across", {0.26, 0.15, 0.50}, {0.26, -0.15, 0.50}},
    {"lower to place", {0.26, 0.15, 0.32}, {0.26, -0.15, 0.32}},
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
  return run_bimanual(
    node, MoveBimanual::Goal::MODE_MIRRORED, x, y, z, 0.05,
    lead == "right" ? 1U : 0U);
}

void usage()
{
  std::cerr << "Usage:\n"
    "  openarm_control_cli status\n"
    "  openarm_control_cli move-joint JOINT_NAME TARGET_RAD\n"
    "  openarm_control_cli gripper open|close|grasp left|right|both [TORQUE_NM] [SPEED_M_S]\n"
    "  openarm_control_cli move-paired-tcp FRAME LEFT_X_METRES LEFT_Y_METRES LEFT_Z_METRES "
    "RIGHT_X_METRES RIGHT_Y_METRES RIGHT_Z_METRES\n"
    "  openarm_control_cli mirror left|right X_METRES Y_METRES Z_METRES\n"
    "  openarm_control_cli clap [CYCLES]\n"
    "  openarm_control_cli estop | estop-release\n"
    "  openarm_control_cli cross [CYCLES]\n"
    "  openarm_control_cli pick-place\n"
    "  openarm_control_cli centroid X_METRES Y_METRES Z_METRES\n"
    "  openarm_control_cli converge X_METRES Y_METRES Z_METRES [STOP_DISTANCE_METRES]\n"
    "  openarm_control_cli home\n";
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
    } else if (argc >= 4 && argc <= 6 && std::string(argv[1]) == "gripper") {
      const std::string operation = argv[2];
      const std::string side = argv[3];
      if ((operation != "open" && operation != "close" && operation != "grasp") ||
        (side != "left" && side != "right" && side != "both"))
      {
        throw std::invalid_argument("gripper operation or side is invalid");
      }
      MoveGripper::Goal goal;
      goal.stamp = node->now();
      goal.side_mask = side == "left" ? MoveGripper::Goal::SIDE_LEFT :
        (side == "right" ? MoveGripper::Goal::SIDE_RIGHT : MoveGripper::Goal::SIDE_BOTH);
      goal.target_opening_m = operation == "open" ? 0.044 : 0.0;
      goal.maximum_motor_torque_nm = argc >= 5 ? number(argv[4]) : 0.25;
      goal.maximum_opening_speed_m_s = argc >= 6 ? number(argv[5]) : 0.0044;
      goal.stop_on_contact = operation == "grasp";
      result = run_goal<MoveGripper>(node, goal, "/openarm_ik/move_gripper");
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
    } else if (argc == 5 && std::string(argv[1]) == "centroid") {
      result = run_bimanual(
        node, MoveBimanual::Goal::MODE_CENTROID, number(argv[2]), number(argv[3]),
        number(argv[4]));
    } else if ((argc == 5 || argc == 6) && std::string(argv[1]) == "converge") {
      result = run_bimanual(
        node, MoveBimanual::Goal::MODE_CONVERGE, number(argv[2]), number(argv[3]),
        number(argv[4]), argc == 6 ? number(argv[5]) : -1.0);
    } else if (argc == 2 && std::string(argv[1]) == "home") {
      result = demo_home(node);
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
