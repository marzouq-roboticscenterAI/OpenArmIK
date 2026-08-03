// SPDX-License-Identifier: Apache-2.0
// Verify bounded CLI behavior across action-server and context loss.
// C++ port of the former test_cli_server_lifecycle.py. The fixture is an
// rclcpp_action server, so this cannot be plain C.
#include <openarm_control_msgs/action/move_joint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <csignal>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

namespace
{
using namespace std::chrono_literals;
using MoveJoint = openarm_control_msgs::action::MoveJoint;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<MoveJoint>;

std::string g_self;

[[noreturn]] void fail(const std::string & message)
{
  std::cerr << "test_cli_server_lifecycle: " << message << '\n';
  std::exit(1);
}

void write_marker(const std::string & name)
{
  const char * const path = std::getenv("OPENARM_CLI_TEST_MARKER");
  if (path == nullptr) {return;}
  std::ofstream stream(path, std::ios::app);
  stream << name << '\n';
}

// ------------------------------------------------------------------ fixture

class Fixture : public rclcpp::Node
{
public:
  explicit Fixture(std::string mode)
  : rclcpp::Node("openarm_cli_lifecycle_fixture"), mode_(std::move(mode))
  {
    using namespace std::placeholders;
    server_ = rclcpp_action::create_server<MoveJoint>(
      this, "/openarm_ik/move_joint",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveJoint::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
      },
      std::bind(&Fixture::cancel, this, _1),
      std::bind(&Fixture::accepted, this, _1));
    write_marker("ready");
  }

  ~Fixture() override
  {
    for (auto & worker : workers_) {
      if (worker.joinable()) {worker.join();}
    }
  }

private:
  void accepted(const std::shared_ptr<ServerGoalHandle> handle)
  {
    write_marker("accepted");
    if (mode_ == "queued") {
      // Left deferred: the client must observe server loss while queued.
      queued_handle_ = handle;
      return;
    }
    handle->execute();
    workers_.emplace_back([this, handle]() {execute(handle);});
  }

  rclcpp_action::CancelResponse cancel(const std::shared_ptr<ServerGoalHandle>)
  {
    write_marker("cancel_requested");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cancel_received_ = true;
    }
    cancel_signal_.notify_all();
    double delay = 0.0;
    if (mode_ == "success_before_cancel_response") {delay = 0.35;} else if (
      mode_ == "aborted_during_cancel_response") {delay = 0.45;} else if (
      mode_ == "success_at_cancel_timeout") {delay = 0.9;}
    if (delay > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(delay));
    }
    if (mode_ == "success_after_cancel_response") {
      return rclcpp_action::CancelResponse::REJECT;
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void wait_for_cancel()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cancel_signal_.wait(lock, [this]() {return cancel_received_ || !rclcpp::ok();});
  }

  void execute(const std::shared_ptr<ServerGoalHandle> handle)
  {
    write_marker("started");
    auto result = std::make_shared<MoveJoint::Result>();

    if (mode_ == "slow") {
      const auto deadline = std::chrono::steady_clock::now() + 1s;
      while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(20ms);
      }
      if (rclcpp::ok()) {
        result->outcome = MoveJoint::Result::OUTCOME_COMPLETED;
        result->command_id = 41;
        result->reason = "fixture completed";
        handle->succeed(result);
      }
      return;
    }

    if (mode_ == "settling") {
      auto feedback = std::make_shared<MoveJoint::Feedback>();
      feedback->measured_progress = 0.99;
      handle->publish_feedback(feedback);
      write_marker("settling");
    }

    struct Race
    {
      const char * mode;
      double delay;
      const char * outcome;
    };
    static const Race races[] = {
      {"success_before_cancel_response", 0.05, "succeeded"},
      {"aborted_during_cancel_response", 0.22, "aborted"},
      {"success_at_cancel_timeout", 0.42, "succeeded"},
      {"success_after_cancel_response", 0.20, "succeeded"},
      {"canceled_after_cancel_response", 0.15, "canceled"},
    };
    for (const Race & race : races) {
      if (mode_ != race.mode) {continue;}
      wait_for_cancel();
      const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(race.delay);
      while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(10ms);
      }
      result->command_id = 42;
      const std::string outcome = race.outcome;
      if (outcome == "succeeded") {
        result->outcome = MoveJoint::Result::OUTCOME_COMPLETED;
        result->reason = mode_ + " completed";
        handle->succeed(result);
      } else if (outcome == "aborted") {
        result->outcome = MoveJoint::Result::OUTCOME_ABORTED;
        result->reason = mode_ + " fixture abort";
        handle->abort(result);
      } else {
        const auto cancel_deadline = std::chrono::steady_clock::now() + 500ms;
        while (rclcpp::ok() && !handle->is_canceling() &&
          std::chrono::steady_clock::now() < cancel_deadline)
        {
          std::this_thread::sleep_for(10ms);
        }
        result->outcome = MoveJoint::Result::OUTCOME_CANCELED;
        result->reason = mode_ + " fixture cancel";
        handle->canceled(result);
      }
      write_marker("terminal_" + outcome);
      return;
    }

    while (rclcpp::ok()) {
      if (mode_ == "cancel_race" && handle->is_canceling()) {
        std::this_thread::sleep_for(50ms);
        result->outcome = MoveJoint::Result::OUTCOME_CANCELED;
        result->reason = "fixture canceled";
        handle->canceled(result);
        write_marker("canceled");
        return;
      }
      std::this_thread::sleep_for(20ms);
    }
  }

  std::string mode_;
  rclcpp_action::Server<MoveJoint>::SharedPtr server_;
  std::shared_ptr<ServerGoalHandle> queued_handle_;
  std::vector<std::thread> workers_;
  std::mutex mutex_;
  std::condition_variable cancel_signal_;
  bool cancel_received_{false};
};

int run_server(const std::string & mode)
{
  auto fixture = std::make_shared<Fixture>(mode);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(fixture);
  executor.spin();
  return 0;
}

// ------------------------------------------------------------------- driver

struct Capture
{
  int exit_code{-1};
  std::string out;
  std::string err;
  bool running{false};
};

// A child process whose stdout and stderr are captured separately, matching the
// Python harness which asserted on each stream independently.
class Child
{
public:
  Child(const std::vector<std::string> & argv, const std::vector<std::string> & env)
  {
    int out_fds[2];
    int err_fds[2];
    if (pipe(out_fds) != 0 || pipe(err_fds) != 0) {fail("pipe failed");}
    pid_ = fork();
    if (pid_ < 0) {fail("fork failed");}
    if (pid_ == 0) {
      close(out_fds[0]);
      close(err_fds[0]);
      dup2(out_fds[1], STDOUT_FILENO);
      dup2(err_fds[1], STDERR_FILENO);
      close(out_fds[1]);
      close(err_fds[1]);
      for (const std::string & entry : env) {
        putenv(strdup(entry.c_str()));
      }
      std::vector<char *> raw;
      raw.reserve(argv.size() + 1u);
      for (const std::string & item : argv) {raw.push_back(const_cast<char *>(item.c_str()));}
      raw.push_back(nullptr);
      execvp(raw[0], raw.data());
      _exit(127);
    }
    close(out_fds[1]);
    close(err_fds[1]);
    out_fd_ = out_fds[0];
    err_fd_ = err_fds[0];
    out_reader_ = std::thread([this]() {drain(out_fd_, out_);});
    err_reader_ = std::thread([this]() {drain(err_fd_, err_);});
  }

  ~Child() {kill_now();}

  Child(const Child &) = delete;
  Child & operator=(const Child &) = delete;

  bool exited()
  {
    if (reaped_) {return true;}
    int status = 0;
    const pid_t done = waitpid(pid_, &status, WNOHANG);
    if (done == pid_) {
      status_ = status;
      reaped_ = true;
    }
    return reaped_;
  }

  // Waits for exit, returning captured streams. Returns running=true on timeout.
  Capture communicate(const std::chrono::steady_clock::duration timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!exited() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(5ms);
    }
    Capture capture;
    if (!reaped_) {
      capture.running = true;
      return capture;
    }
    join_readers();
    capture.exit_code = WIFEXITED(status_) ? WEXITSTATUS(status_) : -1;
    std::lock_guard<std::mutex> lock(mutex_);
    capture.out = out_;
    capture.err = err_;
    return capture;
  }

  void interrupt() {if (!reaped_) {::kill(pid_, SIGINT);}}

  void kill_now()
  {
    if (!reaped_) {
      ::kill(pid_, SIGKILL);
      int status = 0;
      waitpid(pid_, &status, 0);
      status_ = status;
      reaped_ = true;
    }
    join_readers();
  }

  // Mirrors the Python stop(): SIGINT, then SIGKILL after two seconds.
  void stop()
  {
    if (exited()) {join_readers(); return;}
    interrupt();
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!exited() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
    if (!reaped_) {kill_now();}
    join_readers();
  }

private:
  void drain(const int fd, std::string & target)
  {
    char chunk[4096];
    for (;;) {
      const ssize_t bytes = read(fd, chunk, sizeof(chunk));
      if (bytes > 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        target.append(chunk, static_cast<std::size_t>(bytes));
        continue;
      }
      if (bytes < 0 && (errno == EINTR || errno == EAGAIN)) {continue;}
      break;
    }
  }

  void join_readers()
  {
    if (out_reader_.joinable()) {out_reader_.join();}
    if (err_reader_.joinable()) {err_reader_.join();}
    if (out_fd_ >= 0) {close(out_fd_); out_fd_ = -1;}
    if (err_fd_ >= 0) {close(err_fd_); err_fd_ = -1;}
  }

  pid_t pid_{-1};
  int out_fd_{-1};
  int err_fd_{-1};
  int status_{0};
  bool reaped_{false};
  std::thread out_reader_;
  std::thread err_reader_;
  std::mutex mutex_;
  std::string out_;
  std::string err_;
};

void wait_marker(
  const std::string & path, const std::string & expected,
  const std::chrono::steady_clock::duration timeout = 5s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    std::ifstream stream(path);
    std::string line;
    while (std::getline(stream, line)) {
      if (line == expected) {return;}
    }
    std::this_thread::sleep_for(20ms);
  }
  fail("timed out waiting for fixture marker " + expected);
}

struct Case
{
  std::unique_ptr<Child> server;
  std::unique_ptr<Child> client;
  std::string marker;
};

Case start_case(
  const std::string & executable, const std::string & mode, const int domain,
  const std::string & directory, const bool context_shutdown = false)
{
  Case active;
  active.marker = directory + "/" + mode + ".marker";
  std::vector<std::string> env{
    "ROS_DOMAIN_ID=" + std::to_string(domain),
    "OPENARM_CLI_TEST_MARKER=" + active.marker};
  if (context_shutdown) {env.emplace_back("OPENARM_CLI_TEST_CONTEXT_SHUTDOWN=1");}

  active.server = std::make_unique<Child>(
    std::vector<std::string>{g_self, "--server", mode}, env);
  wait_marker(active.marker, "ready");
  active.client = std::make_unique<Child>(
    std::vector<std::string>{executable, "move-joint", "openarm_left_joint1", "0.1"},
    env);
  return active;
}

bool contains(const std::string & haystack, const std::string & needle)
{
  return haystack.find(needle) != std::string::npos;
}

void verify_server_loss(
  const std::string & executable, const std::string & mode, const int domain,
  const std::string & directory)
{
  Case active = start_case(executable, mode, domain, directory);
  const std::string phase =
    mode == "queued" ? "accepted" : (mode == "started" ? "started" : "settling");
  wait_marker(active.marker, phase);
  const auto started = std::chrono::steady_clock::now();
  active.server->stop();
  const Capture capture = active.client->communicate(2s);
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  if (capture.running) {fail(mode + ": client did not exit within two seconds");}
  if (capture.exit_code != 8) {
    fail(mode + ": exit " + std::to_string(capture.exit_code) + " expected 8\n" +
      capture.err);
  }
  if (!contains(capture.err, "action server lost after goal acceptance")) {
    fail(mode + ": missing server-loss diagnosis\n" + capture.err);
  }
  if (contains(capture.err, "terminal result timeout")) {
    fail(mode + ": reported a terminal timeout instead of server loss\n" + capture.err);
  }
  if (elapsed > 2.0) {fail(mode + ": took " + std::to_string(elapsed) + " s");}
}

void verify_slow_success(
  const std::string & executable, const int domain, const std::string & directory)
{
  Case active = start_case(executable, "slow", domain, directory);
  wait_marker(active.marker, "started");
  const Capture capture = active.client->communicate(3s);
  if (capture.running) {fail("slow: client did not exit");}
  if (capture.exit_code != 0) {
    fail("slow: exit " + std::to_string(capture.exit_code) + "\n" + capture.err);
  }
  if (!contains(capture.out, "completed command_id=41")) {
    fail("slow: missing completion line\n" + capture.out);
  }
  if (contains(capture.err, "lost")) {fail("slow: reported loss\n" + capture.err);}
  active.server->stop();
}

void verify_cancel_race(
  const std::string & executable, const int domain, const std::string & directory)
{
  Case active = start_case(executable, "cancel_race", domain, directory);
  wait_marker(active.marker, "started");
  const Capture capture = active.client->communicate(4s);
  if (capture.running) {fail("cancel_race: client did not exit");}
  if (capture.exit_code != 6) {
    fail("cancel_race: exit " + std::to_string(capture.exit_code) + "\n" + capture.err);
  }
  if (!contains(capture.err, "fixture canceled")) {
    fail("cancel_race: missing cancel reason\n" + capture.err);
  }
  wait_marker(active.marker, "cancel_requested");
  wait_marker(active.marker, "canceled");
  if (contains(capture.err, "lost")) {fail("cancel_race: reported loss\n" + capture.err);}
  active.server->stop();
}

void verify_terminal_precedence(
  const std::string & executable, const std::string & mode, const int expected_code,
  const std::string & expected_text, const int domain, const std::string & directory)
{
  Case active = start_case(executable, mode, domain, directory);
  wait_marker(active.marker, "started");
  const Capture capture = active.client->communicate(4s);
  if (capture.running) {fail(mode + ": client did not exit");}
  if (capture.exit_code != expected_code) {
    fail(mode + ": exit " + std::to_string(capture.exit_code) + " expected " +
      std::to_string(expected_code) + "\nstdout:\n" + capture.out + "\nstderr:\n" +
      capture.err);
  }
  const std::string & stream = expected_code == 0 ? capture.out : capture.err;
  if (!contains(stream, expected_text)) {
    fail(mode + ": missing '" + expected_text + "'\nstdout:\n" + capture.out +
      "\nstderr:\n" + capture.err);
  }
  if (contains(capture.err, "terminal result timeout")) {
    fail(mode + ": unexpected terminal timeout\n" + capture.err);
  }
  if (contains(capture.err, "lost after goal acceptance")) {
    fail(mode + ": unexpected server loss\n" + capture.err);
  }
  wait_marker(active.marker, "cancel_requested");
  const std::string terminal = expected_code == 0 ? "terminal_succeeded" :
    (expected_code == 6 ? "terminal_canceled" : "terminal_aborted");
  wait_marker(active.marker, terminal);
  active.server->stop();
}

void verify_context_shutdown(
  const std::string & executable, const int domain, const std::string & directory)
{
  Case active = start_case(executable, "started", domain, directory, true);
  wait_marker(active.marker, "started");
  const auto started = std::chrono::steady_clock::now();
  const Capture capture = active.client->communicate(2s);
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  if (capture.running) {fail("context shutdown: client did not exit");}
  if (capture.exit_code != 8) {
    fail("context shutdown: exit " + std::to_string(capture.exit_code) + "\n" +
      capture.err);
  }
  if (!contains(capture.err, "ROS context shut down after goal acceptance")) {
    fail("context shutdown: missing diagnosis\n" + capture.err);
  }
  if (contains(capture.err, "terminal result timeout")) {
    fail("context shutdown: unexpected terminal timeout\n" + capture.err);
  }
  if (elapsed > 2.0) {fail("context shutdown took " + std::to_string(elapsed) + " s");}
  active.server->stop();
}

int run_driver(const std::string & executable, const std::string & production)
{
  // Keep fixture discovery disjoint from the 100-199 and 180-229 ranges used by
  // the existing ROS contract and SIGINT tests.
  const int base_domain = 10 + static_cast<int>(getpid() % 5);
  char directory[] = "/tmp/openarm-cli-lifecycle-XXXXXX";
  if (mkdtemp(directory) == nullptr) {fail("cannot create temporary directory");}
  const std::string root = directory;

  const char * const loss_modes[] = {"queued", "started", "settling"};
  for (int offset = 0; offset < 3; ++offset) {
    verify_server_loss(production, loss_modes[offset], base_domain + offset, root);
  }
  verify_slow_success(executable, base_domain + 3, root);
  verify_cancel_race(executable, base_domain + 4, root);
  verify_context_shutdown(executable, base_domain + 5, root);

  struct TerminalCase
  {
    const char * mode;
    int code;
    const char * text;
  };
  static const TerminalCase terminal_cases[] = {
    {"success_before_cancel_response", 0, "completed command_id=42"},
    {"aborted_during_cancel_response", 7, "fixture abort"},
    {"success_at_cancel_timeout", 0, "completed command_id=42"},
    {"success_after_cancel_response", 0, "completed command_id=42"},
    {"canceled_after_cancel_response", 6, "fixture cancel"},
  };
  int offset = 6;
  for (const TerminalCase & item : terminal_cases) {
    verify_terminal_precedence(
      executable, item.mode, item.code, item.text, base_domain + offset, root);
    ++offset;
  }
  return 0;
}

const char * argument(int argc, char ** argv, const char * name)
{
  for (int index = 1; index + 1 < argc; ++index) {
    if (std::strcmp(argv[index], name) == 0) {return argv[index + 1];}
  }
  return nullptr;
}
}  // namespace

int main(int argc, char ** argv)
{
  g_self = argv[0];
  const char * const server_mode = argument(argc, argv, "--server");
  if (server_mode != nullptr) {
    rclcpp::init(argc, argv);
    const int result = run_server(server_mode);
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return result;
  }
  const char * const executable = argument(argc, argv, "--executable");
  const char * const production = argument(argc, argv, "--production-executable");
  if (executable == nullptr || production == nullptr) {
    fail("--executable and --production-executable are required");
  }
  return run_driver(executable, production);
}
