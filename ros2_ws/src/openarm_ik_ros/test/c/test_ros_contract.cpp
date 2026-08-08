// SPDX-License-Identifier: Apache-2.0
// Exercise measured state, actions, live portal routing, authority, TF, and
// shutdown. C++ port of the former test_ros_contract.py.
#include <action_msgs/msg/goal_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <openarm_control_msgs/action/move_paired_tcp_scaled.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <json-c/json.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <csignal>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

namespace
{
using namespace std::chrono_literals;
using MovePairedTcpScaled = openarm_control_msgs::action::MovePairedTcpScaled;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using JointState = sensor_msgs::msg::JointState;

std::vector<std::string> g_cleanup_errors;

[[noreturn]] void fail(const std::string & message)
{
  std::cerr << "test_ros_contract: " << message << '\n';
  std::exit(1);
}

void require(const bool condition, const std::string & message)
{
  if (!condition) {fail(message);}
}

// --------------------------------------------------------------- subprocess

struct RunResult
{
  int exit_code{-1};
  std::string out;
  std::string err;
};

RunResult run_command(
  const std::vector<std::string> & argv, const std::vector<std::string> & env,
  const std::chrono::steady_clock::duration timeout)
{
  int out_fds[2];
  int err_fds[2];
  if (pipe(out_fds) != 0 || pipe(err_fds) != 0) {fail("pipe failed");}
  const pid_t child = fork();
  if (child < 0) {fail("fork failed");}
  if (child == 0) {
    close(out_fds[0]);
    close(err_fds[0]);
    dup2(out_fds[1], STDOUT_FILENO);
    dup2(err_fds[1], STDERR_FILENO);
    close(out_fds[1]);
    close(err_fds[1]);
    setsid();
    for (const std::string & entry : env) {putenv(strdup(entry.c_str()));}
    std::vector<char *> raw;
    for (const std::string & item : argv) {raw.push_back(const_cast<char *>(item.c_str()));}
    raw.push_back(nullptr);
    execvp(raw[0], raw.data());
    _exit(127);
  }
  close(out_fds[1]);
  close(err_fds[1]);
  RunResult result;
  std::mutex mutex;
  auto drain = [&mutex](int fd, std::string & target) {
      char chunk[4096];
      for (;;) {
        const ssize_t bytes = read(fd, chunk, sizeof(chunk));
        if (bytes > 0) {
          std::lock_guard<std::mutex> lock(mutex);
          target.append(chunk, static_cast<std::size_t>(bytes));
          continue;
        }
        if (bytes < 0 && (errno == EINTR || errno == EAGAIN)) {continue;}
        break;
      }
      close(fd);
    };
  std::thread out_reader(drain, out_fds[0], std::ref(result.out));
  std::thread err_reader(drain, err_fds[0], std::ref(result.err));
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  int status = 0;
  bool reaped = false;
  while (std::chrono::steady_clock::now() < deadline) {
    if (waitpid(child, &status, WNOHANG) == child) {reaped = true; break;}
    std::this_thread::sleep_for(10ms);
  }
  if (!reaped) {
    ::kill(-child, SIGKILL);
    waitpid(child, &status, 0);
  }
  out_reader.join();
  err_reader.join();
  result.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  return result;
}

// A long-lived child in its own session, stopped through the process group.
class Background
{
public:
  Background(const std::vector<std::string> & argv, const std::vector<std::string> & env)
  {
    int fds[2];
    if (pipe(fds) != 0) {fail("pipe failed");}
    pid_ = fork();
    if (pid_ < 0) {fail("fork failed");}
    if (pid_ == 0) {
      close(fds[0]);
      dup2(fds[1], STDOUT_FILENO);
      dup2(fds[1], STDERR_FILENO);
      close(fds[1]);
      setsid();
      for (const std::string & entry : env) {putenv(strdup(entry.c_str()));}
      std::vector<char *> raw;
      for (const std::string & item : argv) {
        raw.push_back(const_cast<char *>(item.c_str()));
      }
      raw.push_back(nullptr);
      execvp(raw[0], raw.data());
      _exit(127);
    }
    close(fds[1]);
    fd_ = fds[0];
    reader_ = std::thread([this]() {
        char chunk[4096];
        for (;;) {
          const ssize_t bytes = read(fd_, chunk, sizeof(chunk));
          if (bytes > 0) {
            std::lock_guard<std::mutex> lock(mutex_);
            output_.append(chunk, static_cast<std::size_t>(bytes));
            continue;
          }
          if (bytes < 0 && (errno == EINTR || errno == EAGAIN)) {continue;}
          break;
        }
      });
  }

  ~Background() {stop("process");}

  bool alive()
  {
    if (reaped_) {return false;}
    int status = 0;
    if (waitpid(pid_, &status, WNOHANG) == pid_) {
      status_ = status;
      reaped_ = true;
      return false;
    }
    return true;
  }

  std::string output()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_;
  }

  // Mirrors the Python stop_process_group(): SIGINT, a two-second bound, then
  // SIGKILL, recording rather than throwing so cleanup always continues.
  void stop(const std::string & label)
  {
    if (stopped_) {return;}
    stopped_ = true;
    const auto started = std::chrono::steady_clock::now();
    if (!reaped_) {::kill(-pid_, SIGINT);}
    const auto deadline = started + 2s;
    while (alive() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
    if (alive()) {
      g_cleanup_errors.push_back(label + " exceeded the two-second shutdown bound");
      ::kill(-pid_, SIGKILL);
      const auto kill_deadline = std::chrono::steady_clock::now() + 2s;
      while (alive() && std::chrono::steady_clock::now() < kill_deadline) {
        std::this_thread::sleep_for(10ms);
      }
    }
    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    if (elapsed >= 2.0) {
      g_cleanup_errors.push_back(label + " missed the two-second shutdown bound");
    }
    if (reader_.joinable()) {reader_.join();}
    if (fd_ >= 0) {close(fd_); fd_ = -1;}
    const int code = WIFEXITED(status_) ? WEXITSTATUS(status_) : -1;
    if (code != 0) {
      g_cleanup_errors.push_back(
        label + " exited with status " + std::to_string(code));
    }
  }

private:
  pid_t pid_{-1};
  int fd_{-1};
  int status_{0};
  bool reaped_{false};
  bool stopped_{false};
  std::thread reader_;
  std::mutex mutex_;
  std::string output_;
};

// -------------------------------------------------------------- HTTP client

struct HttpResponse
{
  int status{0};
  std::string body;
  bool ok{false};
};

HttpResponse portal_request(
  const int port, const std::string & method, const std::string & target,
  const std::string & payload = std::string(), const std::string & csrf = std::string())
{
  HttpResponse response;
  const int handle = socket(AF_INET, SOCK_STREAM, 0);
  if (handle < 0) {return response;}
  struct timeval timeout;
  timeout.tv_sec = 2;
  timeout.tv_usec = 0;
  setsockopt(handle, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(handle, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  struct sockaddr_in address;
  std::memset(&address, 0, sizeof(address));
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<uint16_t>(port));
  inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
  if (connect(handle, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) != 0) {
    close(handle);
    return response;
  }
  std::ostringstream request;
  request << method << ' ' << target << " HTTP/1.1\r\n"
          << "Host: 127.0.0.1:" << port << "\r\n"
          << "Connection: close\r\n";
  if (!payload.empty()) {
    request << "Content-Type: application/json\r\n"
            << "Origin: http://127.0.0.1:" << port << "\r\n"
            << "Sec-Fetch-Site: same-origin\r\n"
            << "X-CSRF-Token: " << csrf << "\r\n"
            << "Content-Length: " << payload.size() << "\r\n";
  }
  request << "\r\n" << payload;
  const std::string text = request.str();
  if (write(handle, text.data(), text.size()) < 0) {
    close(handle);
    return response;
  }
  std::string raw;
  char chunk[4096];
  for (;;) {
    const ssize_t bytes = read(handle, chunk, sizeof(chunk));
    if (bytes > 0) {raw.append(chunk, static_cast<std::size_t>(bytes)); continue;}
    if (bytes < 0 && errno == EINTR) {continue;}
    break;
  }
  close(handle);
  if (raw.rfind("HTTP/1.", 0) != 0) {return response;}
  const std::size_t status_begin = raw.find(' ');
  if (status_begin == std::string::npos) {return response;}
  response.status = std::atoi(raw.c_str() + status_begin + 1);
  const std::size_t split = raw.find("\r\n\r\n");
  if (split != std::string::npos) {response.body = raw.substr(split + 4u);}
  response.ok = true;
  return response;
}

// ---------------------------------------------------------------- utilities

std::map<std::string, std::string> diagnostic_fields(const DiagnosticArray & message)
{
  std::map<std::string, std::string> report;
  if (message.status.empty()) {return report;}
  for (const auto & item : message.status[0].values) {
    report[item.key] = item.value;
  }
  return report;
}

class Contract
{
public:
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::mutex mutex;
  std::vector<JointState> states;
  std::vector<DiagnosticArray> diagnostics;

  JointState last_state()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return states.back();
  }

  std::map<std::string, std::string> last_report()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return diagnostics.empty() ? std::map<std::string, std::string>{}
    : diagnostic_fields(diagnostics.back());
  }

  std::size_t state_count()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return states.size();
  }

  std::size_t diagnostic_count()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return diagnostics.size();
  }

  template<typename Predicate>
  void wait_for(
    Predicate predicate, const std::chrono::steady_clock::duration timeout = 8s,
    const std::string & what = "ROS contract event")
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_once(100ms);
      if (predicate()) {return;}
    }
    if (!predicate()) {fail("timed out waiting for " + what);}
  }
};

std::vector<std::string> expected_joint_names()
{
  std::vector<std::string> names;
  for (const char * side : {"left", "right"}) {
    for (int index = 1; index <= 7; ++index) {
      names.push_back(std::string("openarm_") + side + "_joint" + std::to_string(index));
    }
  }
  names.push_back("openarm_left_finger_joint1");
  names.push_back("openarm_right_finger_joint1");
  return names;
}

std::vector<std::string> expected_world_children()
{
  std::vector<std::string> frames{"openarm_body_link0"};
  for (const char * side : {"left", "right"}) {
    for (int index = 0; index < 8; ++index) {
      frames.push_back(std::string("openarm_") + side + "_link" + std::to_string(index));
    }
  }
  for (const char * side : {"left", "right"}) {
    for (const char * suffix : {"hand", "hand_tcp", "left_finger", "right_finger"}) {
      frames.push_back(std::string("openarm_") + side + "_" + suffix);
    }
  }
  return frames;
}

MovePairedTcpScaled::Goal scaled_goal(
  const rclcpp::Node::SharedPtr & node, const double scale,
  const std::array<double, 3> & left = {0.30, 0.22, 0.30},
  const std::array<double, 3> & right = {0.30, -0.22, 0.30})
{
  MovePairedTcpScaled::Goal goal;
  goal.header.stamp = node->now();
  goal.header.frame_id = "openarm_body_link0";
  goal.left_tcp_m.x = left[0];
  goal.left_tcp_m.y = left[1];
  goal.left_tcp_m.z = left[2];
  goal.right_tcp_m.x = right[0];
  goal.right_tcp_m.y = right[1];
  goal.right_tcp_m.z = right[2];
  goal.motion_limit_scale = scale;
  return goal;
}

bool json_bool_field(const std::string & body, const char * key)
{
  json_object * const root = json_tokener_parse(body.c_str());
  if (root == nullptr) {return false;}
  json_object * value = nullptr;
  bool result = false;
  if (json_object_object_get_ex(root, key, &value)) {
    result = json_object_get_boolean(value) != 0;
  }
  json_object_put(root);
  return result;
}

double json_double_field(const std::string & body, const char * key)
{
  json_object * const root = json_tokener_parse(body.c_str());
  if (root == nullptr) {return std::numeric_limits<double>::quiet_NaN();}
  json_object * value = nullptr;
  double result = std::numeric_limits<double>::quiet_NaN();
  if (json_object_object_get_ex(root, key, &value)) {
    result = json_object_get_double(value);
  }
  json_object_put(root);
  return result;
}
}  // namespace

int main(int argc, char ** argv)
{
  const int domain = 100 + static_cast<int>(getpid() % 100);
  const std::string domain_value = std::to_string(domain);
  setenv("ROS_DOMAIN_ID", domain_value.c_str(), 1);
  setenv("PYTHONDONTWRITEBYTECODE", "1", 1);
  const std::vector<std::string> env{
    "ROS_DOMAIN_ID=" + domain_value, "PYTHONDONTWRITEBYTECODE=1"};

  auto launch = std::make_unique<Background>(
    std::vector<std::string>{"ros2", "launch", "openarm_ik_ros",
      "openarm_ik_rviz.launch.xml", "rviz:=false"},
    env);

  rclcpp::init(argc, argv);
  Contract contract;
  contract.node = std::make_shared<rclcpp::Node>("openarm_virtual_control_contract_test");
  contract.executor.add_node(contract.node);

  const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
  auto state_subscription = contract.node->create_subscription<JointState>(
    "/joint_states", qos, [&contract](const JointState::SharedPtr message) {
      std::lock_guard<std::mutex> lock(contract.mutex);
      contract.states.push_back(*message);
    });
  auto diagnostic_subscription = contract.node->create_subscription<DiagnosticArray>(
    "/openarm_ik/diagnostics", qos, [&contract](const DiagnosticArray::SharedPtr message) {
      std::lock_guard<std::mutex> lock(contract.mutex);
      contract.diagnostics.push_back(*message);
    });

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(contract.node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, contract.node,
      false);

  std::unique_ptr<Background> portal;
  int exit_code = 0;
  try {
    contract.wait_for(
      [&contract]() {
        return contract.state_count() >= 3u && contract.diagnostic_count() > 0u;
      }, 8s, "initial joint state and diagnostics");

    const JointState state = contract.last_state();
    require(state.name == expected_joint_names(), "unexpected joint name set");
    require(state.position.size() == 16u, "expected 16 positions including J8 finger state");
    require(state.velocity.size() == 16u, "expected 16 velocities including J8 finger state");
    require(state.effort.size() == 16u, "expected 16 efforts including J8 finger state");
    require(state.header.stamp.sec > 0, "joint state has no stamp");
    require(
      contract.node->get_publishers_info_by_topic("/joint_states").size() == 1u,
      "expected exactly one /joint_states publisher");
    require(
      contract.node->get_publishers_info_by_topic("/tf").size() == 1u,
      "expected exactly one /tf publisher");
    require(
      contract.node->get_publishers_info_by_topic("/tf_static").size() == 1u,
      "expected exactly one /tf_static publisher");

    const std::vector<std::string> frames = expected_world_children();
    require(frames.size() == 25u, "expected 25 world children");
    contract.wait_for(
      [&tf_buffer, &frames]() {
        for (const std::string & frame : frames) {
          if (!tf_buffer->canTransform("world", frame, tf2::TimePointZero)) {return false;}
        }
        return true;
      }, 8s, "the complete TF tree");

    std::map<std::string, std::string> report = contract.last_report();
    {
      std::lock_guard<std::mutex> lock(contract.mutex);
      require(
        contract.diagnostics.back().status[0].level == 1,
        "diagnostics level is not WARN");
    }
    const std::vector<std::pair<std::string, std::string>> expectations{
      {"backend", "virtual"},
      {"runtime_authority", "openarm_runtime"},
      {"capability_bits", "3576"},
      {"runtime_state_source", "oa_runtime_snapshot_encoder_feedback"},
      {"collision_checked", "false"},
      {"state_source", "oa_snapshot_encoder_feedback"},
      {"physical_motion_authorized", "false"},
      {"physical_motion_capability", "false"},
      {"physical_discovery_endpoint_exposed", "false"},
      {"single_xyz_capability", "false"},
      {"manifest_state", "4"},
      {"manifest_authenticated", "false"},
      {"manifest_checkpoint_authorized", "false"},
      {"persistence_status", "built_in_immutable_manifest_not_persisted"},
      {"calibration_status", "runtime_capable_ros_endpoint_not_exposed"},
      {"discovery_status", "virtual_exact_inventory"},
      {"inventory_interface_count", "2"},
      {"inventory_motor_count", "14"},
      {"inventory_unresolved_assignment", "0"},
      {"left_fresh_mask", "127"},
      {"right_fresh_mask", "127"},
    };
    for (const auto & entry : expectations) {
      require(
        report[entry.first] == entry.second,
        "diagnostics " + entry.first + " is '" + report[entry.first] + "', expected '" +
        entry.second + "'");
    }
    require(
      report["runtime_coordinate_identity_sha256"].size() == 64u,
      "coordinate identity digest is not 64 characters");

    // ---- live portal ----
    const int portal_port = 31000 + static_cast<int>(getpid() % 10000);
    portal = std::make_unique<Background>(
      std::vector<std::string>{"ros2", "run", "openarm_ik_ros", "openarm_portal",
        "--port", std::to_string(portal_port)},
      env);
    {
      const auto deadline = std::chrono::steady_clock::now() + 8s;
      for (;;) {
        require(portal->alive(), "portal exited early:\n" + portal->output());
        const HttpResponse health = portal_request(portal_port, "GET", "/api/health");
        if (health.ok && health.status == 200) {break;}
        require(
          std::chrono::steady_clock::now() < deadline, "portal health deadline exceeded");
        std::this_thread::sleep_for(50ms);
      }
    }
    const HttpResponse page = portal_request(portal_port, "GET", "/");
    require(page.status == 200, "portal page did not return 200");
    std::smatch match;
    const std::regex csrf_pattern("name=\"portal-csrf\" content=\"([0-9a-f]{64})\"");
    require(
      std::regex_search(page.body, match, csrf_pattern), "portal page has no CSRF token");
    const std::string csrf = match[1].str();
    {
      const auto deadline = std::chrono::steady_clock::now() + 8s;
      for (;;) {
        const HttpResponse portal_state =
          portal_request(portal_port, "GET", "/api/state");
        if (portal_state.status == 200 && json_bool_field(portal_state.body, "state_fresh")) {
          break;
        }
        require(
          std::chrono::steady_clock::now() < deadline, "portal state deadline exceeded");
        std::this_thread::sleep_for(50ms);
      }
    }

    // ---- scaled action boundary ----
    auto scaled_action = rclcpp_action::create_client<MovePairedTcpScaled>(
      contract.node, "/openarm_ik/move_paired_tcp_scaled");
    require(
      scaled_action->wait_for_action_server(8s), "scaled action server did not appear");
    for (const double invalid : {0.49, 1.01, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()})
    {
      auto future = scaled_action->async_send_goal(scaled_goal(contract.node, invalid));
      contract.wait_for(
        [&future]() {return future.wait_for(0s) == std::future_status::ready;}, 8s,
        "invalid-scale goal response");
      require(future.get() == nullptr, "an out-of-range motion limit scale was accepted");
    }

    // ---- portal v3 move drives the scaled action to completion ----
    const HttpResponse move = portal_request(
      portal_port, "POST", "/api/v3/move",
      "{\"side\":\"left\",\"unit\":\"m\",\"x\":0.3,\"y\":0.22,\"z\":0.3,"
      "\"motion_limit_scale\":0.8}",
      csrf);
    require(move.status == 202, "portal move returned " + std::to_string(move.status) +
      ": " + move.body);
    require(
      std::abs(json_double_field(move.body, "motion_limit_scale") - 0.8) < 1e-12,
      "portal did not echo the requested motion limit scale");
    contract.wait_for(
      [&contract]() {
        const auto current = contract.last_report();
        return current.count("last_action") &&
        current.at("last_action") == "move_paired_tcp_scaled" &&
        current.count("outcome") && current.at("outcome") == "completed" &&
        current.count("active_owner") && current.at("active_owner").empty();
      }, 30s, "the scaled portal command to complete");
    {
      const auto scaled_report = contract.last_report();
      require(
        std::stoll(scaled_report.at("result_plan_duration_ns")) > 0,
        "scaled plan duration is not positive");
      require(
        std::stoll(scaled_report.at("result_left_terminal_feedback_seq")) >=
        std::stoll(scaled_report.at("result_left_plan_seed_feedback_seq")),
        "left terminal feedback precedes its seed");
      require(
        std::stoll(scaled_report.at("result_right_terminal_feedback_seq")) >=
        std::stoll(scaled_report.at("result_right_plan_seed_feedback_seq")),
        "right terminal feedback precedes its seed");
    }

    // ---- single-authority and CLI boundaries ----
    const RunResult duplicate = run_command(
      {"ros2", "run", "openarm_ik_ros", "openarm_ik_ros_node"}, env, 5s);
    require(duplicate.exit_code != 0, "a duplicate JointState authority started");
    require(
      duplicate.err.find("another local JointState authority") != std::string::npos,
      "duplicate authority was not diagnosed:\n" + duplicate.err);

    const RunResult invalid_name = run_command(
      {"ros2", "run", "openarm_ik_ros", "openarm_control_cli", "move-joint",
        "unknown_joint", "0.1"}, env, 5s);
    require(
      invalid_name.exit_code == 4,
      "unknown joint exit " + std::to_string(invalid_name.exit_code));
    require(
      invalid_name.err.find("goal rejected") != std::string::npos,
      "unknown joint was not rejected:\n" + invalid_name.err);

    const RunResult invalid_frame = run_command(
      {"ros2", "run", "openarm_ik_ros", "openarm_control_cli", "move-paired-tcp",
        "missing_frame", "0.2", "0.3", "0.85", "0.2", "-0.3", "0.85"}, env, 5s);
    require(
      invalid_frame.exit_code == 7,
      "missing frame exit " + std::to_string(invalid_frame.exit_code) + "\n" +
      invalid_frame.err);
    require(
      invalid_frame.err.find("transform_unavailable") != std::string::npos,
      "missing frame was not diagnosed:\n" + invalid_frame.err);

    // ---- deprecated PoseArray path ----
    geometry_msgs::msg::PoseArray legacy;
    legacy.header.stamp = contract.node->now();
    legacy.header.frame_id = "openarm_body_link0";
    geometry_msgs::msg::Pose left_pose;
    left_pose.position.x = 0.20;
    left_pose.position.y = 0.30;
    left_pose.position.z = 0.85;
    geometry_msgs::msg::Pose right_pose;
    right_pose.position.x = 0.20;
    right_pose.position.y = -0.30;
    right_pose.position.z = 0.85;
    legacy.poses = {left_pose, right_pose};
    const long long legacy_stamp_ns =
      static_cast<long long>(legacy.header.stamp.sec) * 1000000000LL +
      legacy.header.stamp.nanosec;
    const std::string legacy_owner = "legacy:" + std::to_string(legacy_stamp_ns);
    auto legacy_publisher =
      contract.node->create_publisher<geometry_msgs::msg::PoseArray>(
      "/openarm_ik/paired_xyz", qos);
    contract.wait_for(
      [&legacy_publisher]() {return legacy_publisher->get_subscription_count() == 1u;}, 8s,
      "the deprecated PoseArray subscriber");
    legacy_publisher->publish(legacy);
    contract.wait_for(
      [&contract, &legacy_owner]() {
        const auto current = contract.last_report();
        return current.count("active_owner") && current.at("active_owner") == legacy_owner &&
        current.count("executing") && current.at("executing") == "true";
      }, 8s, "the deprecated command to start");

    const RunResult rejected_during_legacy = run_command(
      {"ros2", "run", "openarm_ik_ros", "openarm_control_cli", "move-joint",
        "openarm_left_joint4", "0.1"}, env, 5s);
    require(
      rejected_during_legacy.exit_code == 4,
      "a concurrent goal was not rejected during the deprecated command");

    contract.wait_for(
      [&contract, &legacy_owner]() {
        const auto current = contract.last_report();
        return current.count("last_action") &&
        current.at("last_action") == "deprecated_paired_xyz" &&
        current.count("last_goal_id") && current.at("last_goal_id") == legacy_owner &&
        current.count("committed") && current.at("committed") == "true" &&
        current.count("active_owner") && current.at("active_owner").empty() &&
        current.count("adapter_state") && current.at("adapter_state") == "idle";
      }, 35s, "the deprecated command to commit");
    {
      const auto legacy_report = contract.last_report();
      require(
        legacy_report.at("request_stamp_ns") == std::to_string(legacy_stamp_ns),
        "deprecated request stamp was not carried through");
      require(legacy_report.at("outcome") == "completed", "deprecated command not completed");
      require(
        std::stoll(legacy_report.at("result_left_plan_seed_feedback_seq")) > 0 &&
        std::stoll(legacy_report.at("result_right_plan_seed_feedback_seq")) > 0,
        "deprecated command has no seed feedback");
      require(
        std::stoll(legacy_report.at("result_plan_duration_ns")) > 0,
        "deprecated plan duration is not positive");
      require(
        std::stoll(legacy_report.at("result_left_terminal_feedback_seq")) >=
        std::stoll(legacy_report.at("result_left_plan_seed_feedback_seq")) &&
        std::stoll(legacy_report.at("result_right_terminal_feedback_seq")) >=
        std::stoll(legacy_report.at("result_right_plan_seed_feedback_seq")),
        "deprecated terminal feedback precedes its seed");
    }

    // ---- measured joint command ----
    const long long before_sequence = std::stoll(report["left_feedback_seq"]);
    const double joint_target = contract.last_state().position[3] < 0.6 ? 0.8 : 0.2;
    std::ostringstream target_text;
    target_text << joint_target;
    const RunResult command = run_command(
      {"ros2", "run", "openarm_ik_ros", "openarm_control_cli", "move-joint",
        "openarm_left_joint4", target_text.str()}, env, 15s);
    require(command.exit_code == 0, "move-joint failed:\n" + command.err);
    require(
      command.out.find("completed command_id=") != std::string::npos,
      "move-joint did not report completion:\n" + command.out);
    contract.wait_for(
      [&contract, joint_target]() {
        const auto current = contract.last_report();
        if (!current.count("committed") || current.at("committed") != "true") {return false;}
        if (contract.state_count() == 0u) {return false;}
        return std::abs(
          const_cast<Contract &>(contract).last_state().position[3] - joint_target) < 5e-4;
      }, 8s, "the measured joint command to land");
    require(
      std::stoll(contract.last_report().at("left_feedback_seq")) > before_sequence,
      "feedback sequence did not advance");

    // ---- cancellation ----
    auto cancel_goal_future = scaled_action->async_send_goal(
      scaled_goal(contract.node, 0.8, {0.20, 0.30, 0.30}, {0.20, -0.30, 0.30}));
    contract.wait_for(
      [&cancel_goal_future]() {
        return cancel_goal_future.wait_for(0s) == std::future_status::ready;
      }, 8s, "the cancellable goal response");
    auto cancel_handle = cancel_goal_future.get();
    require(cancel_handle != nullptr, "the cancellable goal was rejected");
    contract.wait_for(
      [&contract]() {
        const auto current = contract.last_report();
        return current.count("last_action") &&
        current.at("last_action") == "move_paired_tcp_scaled" &&
        current.count("executing") && current.at("executing") == "true" &&
        current.count("active_owner") && !current.at("active_owner").empty();
      }, 8s, "the cancellable goal to start executing");
    auto cancel_response_future = scaled_action->async_cancel_goal(cancel_handle);
    contract.wait_for(
      [&cancel_response_future]() {
        return cancel_response_future.wait_for(0s) == std::future_status::ready;
      }, 8s, "the cancel response");
    require(
      !cancel_response_future.get()->goals_canceling.empty(),
      "the server did not accept the cancellation");
    auto canceled_result_future = scaled_action->async_get_result(cancel_handle);
    contract.wait_for(
      [&canceled_result_future]() {
        return canceled_result_future.wait_for(0s) == std::future_status::ready;
      }, 8s, "the canceled terminal result");
    const auto canceled = canceled_result_future.get();
    require(
      canceled.code == rclcpp_action::ResultCode::CANCELED,
      "the canceled goal did not report CANCELED");
    require(
      canceled.result->outcome == MovePairedTcpScaled::Result::OUTCOME_CANCELED,
      "the canceled goal outcome is not OUTCOME_CANCELED");
    require(!canceled.result->collision_checked, "a canceled goal claimed collision checking");
  } catch (const std::exception & error) {
    std::cerr << "test_ros_contract: " << error.what() << '\n';
    exit_code = 1;
  }

  if (portal != nullptr) {portal->stop("portal");}
  tf_listener.reset();
  tf_buffer.reset();
  contract.executor.remove_node(contract.node);
  contract.node.reset();
  if (rclcpp::ok()) {rclcpp::shutdown();}
  launch->stop("headless launch");
  launch.reset();

  if (!g_cleanup_errors.empty()) {
    std::string joined;
    for (const std::string & item : g_cleanup_errors) {
      if (!joined.empty()) {joined += "; ";}
      joined += item;
    }
    std::cerr << "test_ros_contract: " << joined << '\n';
    return 1;
  }
  return exit_code;
}
