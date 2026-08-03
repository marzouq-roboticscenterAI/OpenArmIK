// SPDX-License-Identifier: Apache-2.0
// Exercise bounded SIGINT shutdown at queued, active, settling, and completed
// phases. C++ port of the former test_active_sigint.py; ROS 2 exposes actions
// through rclcpp_action, so this is C++ rather than C like the other ports.
#include <openarm_control_msgs/action/move_joint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <csignal>
#include <sys/wait.h>
#include <unistd.h>

namespace
{
using namespace std::chrono_literals;
using MoveJoint = openarm_control_msgs::action::MoveJoint;
using GoalHandle = rclcpp_action::ClientGoalHandle<MoveJoint>;

constexpr const char * kPhases[] = {"queued", "started", "settling", "completed"};

[[noreturn]] void fail(const std::string & message)
{
  std::cerr << "test_active_sigint: " << message << '\n';
  std::exit(1);
}

// A node process whose combined output is drained continuously, so it can never
// block on a full pipe while the test waits for a phase.
class NodeProcess
{
public:
  NodeProcess()
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
      execlp(
        "ros2", "ros2", "run", "openarm_ik_ros", "openarm_ik_ros_node",
        static_cast<char *>(nullptr));
      _exit(127);
    }
    close(fds[1]);
    read_fd_ = fds[0];
    drain_ = std::thread([this]() {
        char chunk[4096];
        for (;;) {
          const ssize_t bytes = read(read_fd_, chunk, sizeof(chunk));
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

  ~NodeProcess()
  {
    if (!reaped_) {
      ::kill(-pid_, SIGKILL);
      int status = 0;
      waitpid(pid_, &status, 0);
      reaped_ = true;
    }
    if (drain_.joinable()) {drain_.join();}
    if (read_fd_ >= 0) {close(read_fd_);}
  }

  NodeProcess(const NodeProcess &) = delete;
  NodeProcess & operator=(const NodeProcess &) = delete;

  std::string output()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return output_;
  }

  // Sends SIGINT to the process group and requires a clean exit within 2 s.
  void stop_and_require_bounded_shutdown()
  {
    const auto started = std::chrono::steady_clock::now();
    ::kill(-pid_, SIGINT);
    int status = 0;
    bool exited = false;
    while (std::chrono::steady_clock::now() - started < 2s) {
      const pid_t done = waitpid(pid_, &status, WNOHANG);
      if (done == pid_) {exited = true; break;}
      if (done < 0) {fail("waitpid failed");}
      std::this_thread::sleep_for(10ms);
    }
    if (!exited) {
      ::kill(-pid_, SIGKILL);
      waitpid(pid_, &status, 0);
      reaped_ = true;
      fail("active adapter exceeded the two-second SIGINT bound");
    }
    reaped_ = true;
    if (drain_.joinable()) {drain_.join();}
    const std::string text = output();
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      fail("node exited abnormally:\n" + text);
    }
    if (text.find("terminate called") != std::string::npos) {
      fail("node reported an uncaught exception:\n" + text);
    }
    if (text.find("goal does not exist") != std::string::npos) {
      fail("node lost track of its goal during shutdown:\n" + text);
    }
  }

private:
  pid_t pid_{-1};
  int read_fd_{-1};
  bool reaped_{false};
  std::thread drain_;
  std::mutex mutex_;
  std::string output_;
};

template<typename Predicate>
bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor, Predicate predicate,
  const std::chrono::steady_clock::duration timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(20ms);
    if (predicate()) {return true;}
  }
  return predicate();
}

int run_phase(const std::string & phase)
{
  NodeProcess process;
  auto node = std::make_shared<rclcpp::Node>("openarm_sigint_" + phase);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto client = rclcpp_action::create_client<MoveJoint>(node, "/openarm_ik/move_joint");
  if (!client->wait_for_action_server(5s)) {
    fail("action server did not appear:\n" + process.output());
  }

  std::mutex feedback_mutex;
  std::vector<double> feedback;
  rclcpp_action::Client<MoveJoint>::SendGoalOptions options;
  options.feedback_callback =
    [&feedback_mutex, &feedback](
    GoalHandle::SharedPtr, const std::shared_ptr<const MoveJoint::Feedback> value) {
      std::lock_guard<std::mutex> lock(feedback_mutex);
      feedback.push_back(value->measured_progress);
    };
  auto any_feedback = [&feedback_mutex, &feedback](auto predicate) {
      std::lock_guard<std::mutex> lock(feedback_mutex);
      for (const double item : feedback) {
        if (predicate(item)) {return true;}
      }
      return false;
    };

  MoveJoint::Goal goal;
  goal.stamp = node->now();
  goal.joint_name = "openarm_right_joint1";
  goal.target_rad = 0.8;
  auto goal_future = client->async_send_goal(goal, options);

  if (phase == "queued") {
    std::this_thread::sleep_for(10ms);
    process.stop_and_require_bounded_shutdown();
    return 0;
  }

  if (!spin_until(
      executor, [&goal_future]() {
        return goal_future.wait_for(0s) == std::future_status::ready;
      }, 3s))
  {
    fail("goal response did not arrive");
  }
  auto handle = goal_future.get();
  if (handle == nullptr) {fail("goal was rejected");}
  auto result_future = client->async_get_result(handle);

  if (phase == "started") {
    if (!spin_until(
        executor, [&any_feedback]() {
          return any_feedback([](double value) {return value > 0.0 && value < 0.8;});
        }, 5s))
    {
      fail("no in-flight progress feedback observed");
    }
  } else if (phase == "settling") {
    if (!spin_until(
        executor, [&result_future, &any_feedback]() {
          return result_future.wait_for(0s) != std::future_status::ready &&
          any_feedback([](double value) {return value >= 0.98;});
        }, 15s))
    {
      fail("settling phase was not observed");
    }
  } else {
    if (!spin_until(
        executor, [&result_future]() {
          return result_future.wait_for(0s) == std::future_status::ready;
        }, 15s))
    {
      fail("goal did not complete");
    }
  }
  process.stop_and_require_bounded_shutdown();
  return 0;
}

// Re-runs this executable once per phase, each in its own ROS domain so the
// phases cannot observe one another's nodes.
int run_all(const char * self)
{
  for (std::size_t index = 0; index < sizeof(kPhases) / sizeof(kPhases[0]); ++index) {
    const int domain = 180 + static_cast<int>((getpid() + index) % 50u);
    const pid_t child = fork();
    if (child < 0) {fail("fork failed");}
    if (child == 0) {
      setsid();
      const std::string value = std::to_string(domain);
      setenv("ROS_DOMAIN_ID", value.c_str(), 1);
      execlp(self, self, kPhases[index], static_cast<char *>(nullptr));
      _exit(127);
    }
    int status = 0;
    const auto started = std::chrono::steady_clock::now();
    bool exited = false;
    while (std::chrono::steady_clock::now() - started < 25s) {
      const pid_t done = waitpid(child, &status, WNOHANG);
      if (done == child) {exited = true; break;}
      if (done < 0) {fail("waitpid failed");}
      std::this_thread::sleep_for(20ms);
    }
    if (!exited) {
      ::kill(-child, SIGKILL);
      waitpid(child, &status, 0);
      fail(std::string(kPhases[index]) + " SIGINT phase timed out");
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      fail(std::string(kPhases[index]) + " SIGINT phase failed");
    }
  }
  return 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 1) {
    return run_all(argv[0]);
  }
  const std::string phase = argv[1];
  bool known = false;
  for (const char * candidate : kPhases) {
    if (phase == candidate) {known = true;}
  }
  if (!known) {fail("unknown phase: " + phase);}
  rclcpp::init(argc, argv);
  int result = 1;
  try {
    result = run_phase(phase);
  } catch (const std::exception & error) {
    std::cerr << "test_active_sigint: " << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
