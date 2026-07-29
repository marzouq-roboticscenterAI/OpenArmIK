// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/virtual_control_session.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace
{
using namespace std::chrono_literals;
using openarm_ik_ros::CommandResult;
using openarm_ik_ros::MeasuredState;
using openarm_ik_ros::SessionCommand;
using openarm_ik_ros::VirtualControlSession;

struct Recorder
{
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<MeasuredState> states;
  std::optional<CommandResult> result;
  std::size_t terminal_count{};

  bool state(const MeasuredState & value)
  {
    std::lock_guard<std::mutex> lock(mutex);
    states.push_back(value);
    condition.notify_all();
    return true;
  }

  void terminal(const CommandResult & value)
  {
    std::lock_guard<std::mutex> lock(mutex);
    result = value;
    ++terminal_count;
    condition.notify_all();
  }

  bool wait_result(std::chrono::seconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, timeout, [this]() {return result.has_value();});
  }
};

template<typename Predicate>
bool wait_health(
  VirtualControlSession & session, Predicate predicate,
  const std::chrono::steady_clock::duration timeout = 2s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate(session.health())) {
      return true;
    }
    std::this_thread::sleep_for(2ms);
  }
  return predicate(session.health());
}

TEST(VirtualControlSession, CanonicalNamesLimitsAndSingleReservation)
{
  Recorder recorder;
  VirtualControlSession session(
    [&recorder](const MeasuredState & value) {return recorder.state(value);}, []() {});
  EXPECT_EQ(VirtualControlSession::joint_names().size(), 14U);
  oa_side side{};
  std::uint32_t joint{};
  EXPECT_TRUE(VirtualControlSession::map_joint("openarm_left_joint7", side, joint));
  EXPECT_EQ(side, OA_LEFT);
  EXPECT_EQ(joint, 6U);
  EXPECT_TRUE(VirtualControlSession::map_joint("openarm_right_joint1", side, joint));
  EXPECT_EQ(side, OA_RIGHT);
  EXPECT_EQ(joint, 0U);
  EXPECT_FALSE(VirtualControlSession::map_joint("joint1", side, joint));
  EXPECT_TRUE(VirtualControlSession::joint_target_in_limits(OA_LEFT, 0U, -3.490659));
  EXPECT_FALSE(VirtualControlSession::joint_target_in_limits(OA_LEFT, 0U, -3.490660));
  EXPECT_FALSE(VirtualControlSession::joint_target_in_limits(OA_RIGHT, 0U, NAN));
  std::string reason;
  EXPECT_TRUE(session.reserve("first", reason));
  EXPECT_FALSE(session.reserve("second", reason));
  EXPECT_EQ(reason, "busy");
  session.release("first", "test_release");
  EXPECT_TRUE(session.reserve("second", reason));
  session.release("second", "test_release");
}

TEST(VirtualControlSession, PublishesLaggingMeasuredStateAndCompletesOnFeedback)
{
  Recorder recorder;
  VirtualControlSession session(
    [&recorder](const MeasuredState & value) {return recorder.state(value);}, []() {});
  std::string reason;
  ASSERT_TRUE(session.reserve("joint", reason));
  SessionCommand command;
  command.kind = SessionCommand::Kind::joint;
  command.owner = "joint";
  command.side = OA_LEFT;
  command.joint = 3U;
  command.target_rad = 0.2;
  command.terminal = [&recorder](const CommandResult & value) {recorder.terminal(value);};
  ASSERT_TRUE(session.submit(std::move(command), reason));
  ASSERT_TRUE(recorder.wait_result(15s));
  ASSERT_EQ(recorder.result->outcome, CommandResult::Outcome::completed) <<
    recorder.result->reason << " status=" << recorder.result->control_status;
  ASSERT_GT(recorder.states.size(), 5U);
  std::size_t intermediate = 0U;
  std::uint64_t prior_sequence = 0U;
  for (const auto & state : recorder.states) {
    EXPECT_EQ(state.snapshot.arm[0].fresh_mask, 0x7fU);
    EXPECT_EQ(state.snapshot.arm[1].fresh_mask, 0x7fU);
    EXPECT_LE(state.snapshot.arm[0].t_ns, state.controller_now_ns);
    EXPECT_LE(state.snapshot.arm[1].t_ns, state.controller_now_ns);
    if (prior_sequence != 0U) {
      EXPECT_GT(state.snapshot.arm[0].feedback_seq, prior_sequence);
    }
    prior_sequence = state.snapshot.arm[0].feedback_seq;
    const double q = state.snapshot.arm[0].q[3];
    if (q > 1.0e-3 && q < 0.19) {
      ++intermediate;
    }
  }
  EXPECT_GT(intermediate, 2U);
  const auto & terminal = recorder.states.back().snapshot.arm[0];
  EXPECT_NEAR(terminal.q[3], 0.2, 5.0e-4);
  EXPECT_NEAR(terminal.dq[3], 0.0, 2.0e-2);
  EXPECT_GT(recorder.result->terminal_feedback_seq[0], recorder.result->seed_feedback_seq[0]);
}

TEST(VirtualControlSession, PairedNamedTargetsReachMeasuredCompletion)
{
  Recorder recorder;
  VirtualControlSession session(
    [&recorder](const MeasuredState & value) {return recorder.state(value);}, []() {});
  std::string reason;
  ASSERT_TRUE(session.reserve("paired", reason));
  SessionCommand command;
  command.kind = SessionCommand::Kind::paired_tcp;
  command.owner = "paired";
  command.left_tcp_m = {0.20, 0.30, 0.85};
  command.right_tcp_m = {0.20, -0.30, 0.85};
  command.terminal = [&recorder](const CommandResult & value) {recorder.terminal(value);};
  ASSERT_TRUE(session.submit(std::move(command), reason));
  ASSERT_TRUE(recorder.wait_result(40s));
  EXPECT_EQ(recorder.result->outcome, CommandResult::Outcome::completed) <<
    recorder.result->reason << " status=" << recorder.result->control_status;
  EXPECT_FALSE(recorder.result->collision_checked);
  EXPECT_GT(recorder.result->terminal_feedback_seq[0], recorder.result->seed_feedback_seq[0]);
  EXPECT_GT(recorder.result->terminal_feedback_seq[1], recorder.result->seed_feedback_seq[1]);
}

TEST(VirtualControlSession, CancelDisableStopsAndRejectsLaterCommands)
{
  Recorder recorder;
  VirtualControlSession session(
    [&recorder](const MeasuredState & value) {return recorder.state(value);}, []() {});
  std::string reason;
  ASSERT_TRUE(session.reserve("cancel", reason));
  SessionCommand command;
  command.kind = SessionCommand::Kind::joint;
  command.owner = "cancel";
  command.side = OA_RIGHT;
  command.joint = 0U;
  command.target_rad = 0.8;
  command.terminal = [&recorder](const CommandResult & value) {recorder.terminal(value);};
  ASSERT_TRUE(session.submit(std::move(command), reason));
  std::this_thread::sleep_for(30ms);
  ASSERT_TRUE(session.cancel("cancel"));
  ASSERT_TRUE(recorder.wait_result(2s));
  EXPECT_EQ(recorder.result->outcome, CommandResult::Outcome::canceled);
  EXPECT_EQ(recorder.result->command_id, 1U);
  EXPECT_FALSE(session.reserve("later", reason));
  EXPECT_EQ(reason, "stopped_requires_restart");
  const auto begin = std::chrono::steady_clock::now();
  session.close();
  EXPECT_LT(std::chrono::steady_clock::now() - begin, 2s);
  session.close();
}

TEST(VirtualControlSession, ReservedCancelStopsThenTerminatesAcceptedCommandExactlyOnce)
{
  Recorder recorder;
  VirtualControlSession session(
    [&recorder](const MeasuredState & value) {return recorder.state(value);}, []() {});
  std::string reason;
  ASSERT_TRUE(session.reserve("reserved-cancel", reason));
  ASSERT_TRUE(session.cancel("reserved-cancel"));
  ASSERT_TRUE(wait_health(session, [](const auto & health) {
      return health.adapter_state == openarm_ik_ros::AdapterState::stopped_requires_restart;
    }));
  const auto stopped = session.health();
  EXPECT_EQ(stopped.owner, "reserved-cancel");
  EXPECT_EQ(stopped.snapshot.lifecycle, OA_LIFECYCLE_DISARMED);

  SessionCommand command;
  command.kind = SessionCommand::Kind::joint;
  command.owner = "reserved-cancel";
  command.side = OA_LEFT;
  command.joint = 0U;
  command.target_rad = 0.1;
  command.terminal = [&recorder](const CommandResult & value) {recorder.terminal(value);};
  ASSERT_TRUE(session.submit(std::move(command), reason));
  ASSERT_TRUE(recorder.wait_result(2s));
  EXPECT_EQ(recorder.terminal_count, 1U);
  EXPECT_EQ(recorder.result->outcome, CommandResult::Outcome::canceled);
  EXPECT_EQ(recorder.result->command_id, 0U);
  EXPECT_EQ(recorder.result->lifecycle, OA_LIFECYCLE_DISARMED);
  EXPECT_EQ(recorder.result->event, OA_EVENT_STOPPED);
  EXPECT_FALSE(session.reserve("later", reason));
  EXPECT_EQ(reason, "stopped_requires_restart");
}

TEST(VirtualControlSession, ReleasingCanceledReservationCannotRearmDisarmedController)
{
  Recorder recorder;
  VirtualControlSession session(
    [&recorder](const MeasuredState & value) {return recorder.state(value);}, []() {});
  std::string reason;
  ASSERT_TRUE(session.reserve("released-cancel", reason));
  ASSERT_TRUE(session.cancel("released-cancel"));
  ASSERT_TRUE(wait_health(session, [](const auto & health) {
      return health.adapter_state == openarm_ik_ros::AdapterState::stopped_requires_restart;
    }));
  session.release("released-cancel", "validation_failed");
  const auto health = session.health();
  EXPECT_TRUE(health.owner.empty());
  EXPECT_EQ(health.adapter_state, openarm_ik_ros::AdapterState::stopped_requires_restart);
  EXPECT_EQ(health.snapshot.lifecycle, OA_LIFECYCLE_DISARMED);
  EXPECT_FALSE(session.reserve("later", reason));
  EXPECT_EQ(reason, "stopped_requires_restart");
}

TEST(VirtualControlSession, CompletionRetainsOwnershipThroughTerminalCallback)
{
  Recorder recorder;
  std::mutex terminal_mutex;
  std::condition_variable terminal_condition;
  bool entered = false;
  bool release = false;
  VirtualControlSession session(
    [&recorder](const MeasuredState & value) {return recorder.state(value);}, []() {});
  std::string reason;
  ASSERT_TRUE(session.reserve("completion-owner", reason));
  SessionCommand command;
  command.kind = SessionCommand::Kind::joint;
  command.owner = "completion-owner";
  command.side = OA_LEFT;
  command.joint = 3U;
  command.target_rad = 0.02;
  command.terminal = [&](const CommandResult & value) {
      {
        std::lock_guard<std::mutex> lock(terminal_mutex);
        entered = true;
        terminal_condition.notify_all();
      }
      std::unique_lock<std::mutex> lock(terminal_mutex);
      terminal_condition.wait(lock, [&]() {return release;});
      recorder.terminal(value);
    };
  ASSERT_TRUE(session.submit(std::move(command), reason));
  bool callback_entered = false;
  {
    std::unique_lock<std::mutex> lock(terminal_mutex);
    callback_entered = terminal_condition.wait_for(lock, 10s, [&]() {return entered;});
    if (!callback_entered) {
      release = true;
      terminal_condition.notify_all();
    }
  }
  ASSERT_TRUE(callback_entered);
  EXPECT_FALSE(session.reserve("too-early", reason));
  EXPECT_EQ(reason, "busy");
  {
    std::lock_guard<std::mutex> lock(terminal_mutex);
    release = true;
    terminal_condition.notify_all();
  }
  ASSERT_TRUE(recorder.wait_result(2s));
  ASSERT_TRUE(wait_health(session, [](const auto & health) {
      return health.adapter_state == openarm_ik_ros::AdapterState::idle;
    }));
  EXPECT_TRUE(session.reserve("after-terminal", reason));
  session.release("after-terminal", "test_release");
}

TEST(VirtualControlSession, ThrowingTerminalCallbackCannotTerminateWorker)
{
  Recorder recorder;
  std::atomic<std::size_t> terminal_count{};
  VirtualControlSession session(
    [&recorder](const MeasuredState & value) {return recorder.state(value);}, []() {});
  std::string reason;
  ASSERT_TRUE(session.reserve("throwing-terminal", reason));
  SessionCommand command;
  command.kind = SessionCommand::Kind::joint;
  command.owner = "throwing-terminal";
  command.side = OA_LEFT;
  command.joint = 3U;
  command.target_rad = 0.02;
  command.terminal = [&](const CommandResult &) {
      ++terminal_count;
      throw std::runtime_error("intentional terminal callback failure");
    };
  ASSERT_TRUE(session.submit(std::move(command), reason));
  ASSERT_TRUE(wait_health(session, [&](const auto & health) {
      return terminal_count.load() == 1U &&
             health.adapter_state == openarm_ik_ros::AdapterState::idle;
    }, 10s));
  const auto begin = std::chrono::steady_clock::now();
  session.close();
  EXPECT_LT(std::chrono::steady_clock::now() - begin, 2s);
  EXPECT_EQ(terminal_count.load(), 1U);
}

TEST(VirtualControlSession, ActiveShutdownStopsAndReportsMeasuredProvenanceOnce)
{
  Recorder recorder;
  VirtualControlSession session(
    [&recorder](const MeasuredState & value) {return recorder.state(value);}, []() {});
  std::string reason;
  ASSERT_TRUE(session.reserve("shutdown-active", reason));
  SessionCommand command;
  command.kind = SessionCommand::Kind::joint;
  command.owner = "shutdown-active";
  command.side = OA_RIGHT;
  command.joint = 0U;
  command.target_rad = 0.8;
  command.terminal = [&recorder](const CommandResult & value) {recorder.terminal(value);};
  ASSERT_TRUE(session.submit(std::move(command), reason));
  ASSERT_TRUE(wait_health(session, [](const auto & health) {
      return health.adapter_state == openarm_ik_ros::AdapterState::executing;
    }));
  const auto begin = std::chrono::steady_clock::now();
  session.close();
  EXPECT_LT(std::chrono::steady_clock::now() - begin, 2s);
  ASSERT_TRUE(recorder.result.has_value());
  EXPECT_EQ(recorder.terminal_count, 1U);
  EXPECT_EQ(recorder.result->outcome, CommandResult::Outcome::aborted);
  EXPECT_EQ(recorder.result->lifecycle, OA_LIFECYCLE_DISARMED);
  EXPECT_EQ(recorder.result->event, OA_EVENT_ABORTED);
  EXPECT_GT(recorder.result->seed_feedback_seq[0], 0U);
  EXPECT_GT(recorder.result->seed_feedback_seq[1], 0U);
  EXPECT_GT(recorder.result->plan_duration_ns, 0U);
  EXPECT_GE(
    recorder.result->terminal_feedback_seq[0], recorder.result->seed_feedback_seq[0]);
  EXPECT_GE(
    recorder.result->terminal_feedback_seq[1], recorder.result->seed_feedback_seq[1]);
}
}
