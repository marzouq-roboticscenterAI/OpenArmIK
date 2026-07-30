// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/virtual_control_session.hpp"
#include "openarm_ik_ros/portal_core.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <stdexcept>
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

  bool terminal(const CommandResult & value)
  {
    std::lock_guard<std::mutex> lock(mutex);
    result = value;
    ++terminal_count;
    condition.notify_all();
    return true;
  }

  bool wait_result(std::chrono::seconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, timeout, [this]() {return result.has_value();});
  }

  bool wait_state(std::chrono::seconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, timeout, [this]() {return !states.empty();});
  }

  MeasuredState latest_state_and_clear_result()
  {
    std::lock_guard<std::mutex> lock(mutex);
    result.reset();
    return states.back();
  }

  std::size_t state_count()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return states.size();
  }

  std::vector<MeasuredState> captured_states()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return states;
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
  std::uint32_t side{};
  std::uint32_t joint{};
  EXPECT_TRUE(VirtualControlSession::map_joint("openarm_left_joint7", side, joint));
  EXPECT_EQ(side, openarm_ik_ros::kLeftSide);
  EXPECT_EQ(joint, 6U);
  EXPECT_TRUE(VirtualControlSession::map_joint("openarm_right_joint1", side, joint));
  EXPECT_EQ(side, openarm_ik_ros::kRightSide);
  EXPECT_EQ(joint, 0U);
  EXPECT_FALSE(VirtualControlSession::map_joint("joint1", side, joint));
  EXPECT_TRUE(VirtualControlSession::joint_target_in_limits(
    openarm_ik_ros::kLeftSide, 0U, -3.490659));
  EXPECT_FALSE(VirtualControlSession::joint_target_in_limits(
    openarm_ik_ros::kLeftSide, 0U, -3.490660));
  EXPECT_FALSE(VirtualControlSession::joint_target_in_limits(
    openarm_ik_ros::kRightSide, 0U, NAN));
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
  command.side = openarm_ik_ros::kLeftSide;
  command.joint = 3U;
  command.target_rad = 0.2;
  command.terminal = [&recorder](const CommandResult & value) {
      return recorder.terminal(value);
    };
  ASSERT_TRUE(session.submit(std::move(command), reason));
  ASSERT_TRUE(recorder.wait_result(15s));
  ASSERT_EQ(recorder.result->outcome, CommandResult::Outcome::completed) <<
    recorder.result->reason << " status=" << recorder.result->control_status;
  const auto states = recorder.captured_states();
  ASSERT_GT(states.size(), 5U);
  std::size_t intermediate = 0U;
  std::uint64_t prior_sequence = 0U;
  for (const auto & state : states) {
    EXPECT_EQ(state.snapshot.arm[0].fresh_mask, 0x7fU);
    EXPECT_EQ(state.snapshot.arm[1].fresh_mask, 0x7fU);
    EXPECT_LE(state.snapshot.arm[0].measurement_runtime_monotonic_ns, state.runtime_now_ns);
    EXPECT_LE(state.snapshot.arm[1].measurement_runtime_monotonic_ns, state.runtime_now_ns);
    if (prior_sequence != 0U) {
      EXPECT_GT(state.snapshot.arm[0].feedback_seq, prior_sequence);
    }
    prior_sequence = state.snapshot.arm[0].feedback_seq;
    const double q = state.snapshot.arm[0].q_model_rad[3];
    if (q > 1.0e-3 && q < 0.19) {
      ++intermediate;
    }
  }
  EXPECT_GT(intermediate, 2U);
  const auto & terminal = states.back().snapshot.arm[0];
  EXPECT_NEAR(terminal.q_model_rad[3], 0.2, 5.0e-4);
  EXPECT_NEAR(terminal.dq_model_rad_s[3], 0.0, 2.0e-2);
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
  command.terminal = [&recorder](const CommandResult & value) {
      return recorder.terminal(value);
    };
  ASSERT_TRUE(session.submit(std::move(command), reason));
  ASSERT_TRUE(recorder.wait_result(40s));
  EXPECT_EQ(recorder.result->outcome, CommandResult::Outcome::completed) <<
    recorder.result->reason << " status=" << recorder.result->control_status;
  EXPECT_FALSE(recorder.result->collision_checked);
  EXPECT_GT(recorder.result->terminal_feedback_seq[0], recorder.result->seed_feedback_seq[0]);
  EXPECT_GT(recorder.result->terminal_feedback_seq[1], recorder.result->seed_feedback_seq[1]);
}

TEST(VirtualControlSession, AllPortalTargetsCompleteFromFreshMeasuredFeedback)
{
  namespace portal = openarm_ik_ros::portal;
  Recorder recorder;
  VirtualControlSession session(
    [&recorder](const MeasuredState & value) {return recorder.state(value);}, []() {});
  ASSERT_TRUE(recorder.wait_state(2s));
  for (std::size_t target_index = 0; target_index < 9U; ++target_index) {
    for (std::size_t selected = 0; selected < 2U; ++selected) {
      const auto side = selected == 0U ?
        portal::MoveRequest::Side::left : portal::MoveRequest::Side::right;
      const portal::NominalTarget & target = portal::nominal_targets(side)[target_index];
      const MeasuredState measured = recorder.latest_state_and_clear_result();
      portal::GuardInput input;
      for (std::size_t arm = 0; arm < 2U; ++arm) {
        std::copy_n(
          measured.snapshot.arm[arm].q_model_rad, OA_RUNTIME_DOF,
          input.measured_q[arm].begin());
      }
      input.request.side = side;
      input.request.target = target.point;
      const portal::GuardResult guard = portal::NominalPathGuard().validate(input);
      ASSERT_TRUE(guard.accepted) << target.id << " side=" << selected << ": " << guard.reason;
      EXPECT_EQ(guard.commanded_tcp[selected], target.point);
      const std::size_t opposite = 1U - selected;
      oa_fk_result opposite_fk{};
      const oa_model * opposite_model = opposite == 0U ?
        oa_model_left_v10_bimanual() : oa_model_right_v10_bimanual();
      ASSERT_EQ(oa_fk(opposite_model, input.measured_q[opposite].data(), &opposite_fk), OA_MODEL_OK);
      const portal::Point opposite_measured{{
        opposite_fk.hand_tcp.m[3], opposite_fk.hand_tcp.m[7], opposite_fk.hand_tcp.m[11]}};
      EXPECT_EQ(guard.commanded_tcp[opposite], opposite_measured);

      const std::string owner = std::string(target.id) + (selected == 0U ? "-left" : "-right");
      std::string reason;
      ASSERT_TRUE(session.reserve(owner, reason)) << reason;
      SessionCommand command;
      command.kind = SessionCommand::Kind::paired_tcp;
      command.owner = owner;
      command.left_tcp_m = guard.commanded_tcp[0];
      command.right_tcp_m = guard.commanded_tcp[1];
      command.terminal = [&recorder](const CommandResult & value) {
          return recorder.terminal(value);
        };
      ASSERT_TRUE(session.submit(std::move(command), reason)) << reason;
      ASSERT_TRUE(recorder.wait_result(40s)) << target.id << " side=" << selected;
      ASSERT_TRUE(recorder.result.has_value());
      EXPECT_EQ(recorder.result->outcome, CommandResult::Outcome::completed) <<
        recorder.result->reason;
      EXPECT_FALSE(recorder.result->collision_checked);
      EXPECT_FALSE(recorder.result->motion_authorized);
      EXPECT_GT(
        recorder.result->terminal_feedback_seq[selected],
        recorder.result->seed_feedback_seq[selected]);
      ASSERT_TRUE(wait_health(session, [](const auto & health) {
        return health.adapter_state == openarm_ik_ros::AdapterState::idle;
      }, 2s));
    }
  }
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
  command.side = openarm_ik_ros::kRightSide;
  command.joint = 0U;
  command.target_rad = 0.8;
  command.terminal = [&recorder](const CommandResult & value) {
      return recorder.terminal(value);
    };
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
  EXPECT_EQ(stopped.snapshot.lifecycle, openarm_ik_ros::kLifecycleDisarmed);

  SessionCommand command;
  command.kind = SessionCommand::Kind::joint;
  command.owner = "reserved-cancel";
  command.side = openarm_ik_ros::kLeftSide;
  command.joint = 0U;
  command.target_rad = 0.1;
  command.terminal = [&recorder](const CommandResult & value) {
      return recorder.terminal(value);
    };
  ASSERT_TRUE(session.submit(std::move(command), reason));
  ASSERT_TRUE(recorder.wait_result(2s));
  EXPECT_EQ(recorder.terminal_count, 1U);
  EXPECT_EQ(recorder.result->outcome, CommandResult::Outcome::canceled);
  EXPECT_EQ(recorder.result->command_id, 0U);
  EXPECT_EQ(recorder.result->lifecycle, openarm_ik_ros::kLifecycleDisarmed);
  EXPECT_EQ(recorder.result->event, OA_RUNTIME_EVENT_STOPPED);
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
  EXPECT_EQ(health.snapshot.lifecycle, openarm_ik_ros::kLifecycleDisarmed);
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
  command.side = openarm_ik_ros::kLeftSide;
  command.joint = 3U;
  command.target_rad = 0.0;
  command.terminal = [&](const CommandResult & value) {
      {
        std::lock_guard<std::mutex> lock(terminal_mutex);
        entered = true;
        terminal_condition.notify_all();
      }
      std::unique_lock<std::mutex> lock(terminal_mutex);
      terminal_condition.wait(lock, [&]() {return release;});
      return recorder.terminal(value);
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
  command.side = openarm_ik_ros::kLeftSide;
  command.joint = 3U;
  command.target_rad = 0.02;
  command.terminal = [&](const CommandResult &) -> bool {
      ++terminal_count;
      throw std::runtime_error("intentional terminal callback failure");
    };
  ASSERT_TRUE(session.submit(std::move(command), reason));
  ASSERT_TRUE(wait_health(session, [&](const auto & health) {
      return terminal_count.load() == 1U &&
             health.adapter_state == openarm_ik_ros::AdapterState::fault;
    }, 10s));
  const auto health = session.health();
  EXPECT_EQ(health.last_error.status, OA_RUNTIME_EFAULT);
  EXPECT_EQ(health.last_error.facility, OA_RUNTIME_FACILITY_RUNTIME);
  EXPECT_EQ(health.last_error.lower_code, 0U);
  EXPECT_EQ(health.snapshot.lifecycle, openarm_ik_ros::kLifecycleDisarmed);
  EXPECT_EQ(health.reason, "terminal_callback_failed");
  EXPECT_TRUE(health.owner.empty());
  const auto states_before = recorder.state_count();
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(recorder.state_count(), states_before);
  EXPECT_FALSE(session.reserve("after-terminal-failure", reason));
  EXPECT_EQ(reason, "adapter_fault");
  const auto begin = std::chrono::steady_clock::now();
  session.close();
  EXPECT_LT(std::chrono::steady_clock::now() - begin, 2s);
  EXPECT_EQ(terminal_count.load(), 1U);
  EXPECT_EQ(session.health().adapter_state, openarm_ik_ros::AdapterState::fault);
}

TEST(VirtualControlSession, ThrowingFeedbackStopsPublicationAndReportsAuthoritativeState)
{
  Recorder recorder;
  std::atomic<std::size_t> feedback_count{};
  VirtualControlSession session(
    [&recorder](const MeasuredState & value) {return recorder.state(value);}, []() {});
  std::string reason;
  ASSERT_TRUE(session.reserve("feedback-failure", reason));
  SessionCommand command;
  command.kind = SessionCommand::Kind::joint;
  command.owner = "feedback-failure";
  command.side = openarm_ik_ros::kLeftSide;
  command.joint = 3U;
  command.target_rad = 0.2;
  command.feedback = [&feedback_count](const openarm_ik_ros::CommandFeedback &) -> bool {
      ++feedback_count;
      throw std::runtime_error("intentional feedback callback failure");
    };
  command.terminal = [&recorder](const CommandResult & value) {
      return recorder.terminal(value);
    };
  ASSERT_TRUE(session.submit(std::move(command), reason));
  ASSERT_TRUE(wait_health(session, [](const auto & health) {
      return health.adapter_state == openarm_ik_ros::AdapterState::fault;
    }, 10s));
  ASSERT_TRUE(recorder.wait_result(2s));
  const auto health = session.health();
  EXPECT_EQ(feedback_count.load(), 1U);
  EXPECT_EQ(recorder.terminal_count, 1U);
  EXPECT_EQ(recorder.result->outcome, CommandResult::Outcome::aborted);
  EXPECT_EQ(recorder.result->control_status, 0U);
  EXPECT_EQ(recorder.result->runtime_status, OA_RUNTIME_EFAULT);
  EXPECT_EQ(recorder.result->runtime_facility, OA_RUNTIME_FACILITY_RUNTIME);
  EXPECT_EQ(recorder.result->cause, 0U);
  EXPECT_EQ(recorder.result->event, OA_RUNTIME_EVENT_FAULTED);
  EXPECT_EQ(recorder.result->lifecycle, health.snapshot.lifecycle);
  EXPECT_EQ(recorder.result->lifecycle, openarm_ik_ros::kLifecycleDisarmed);
  EXPECT_EQ(health.last_error.status, OA_RUNTIME_EFAULT);
  EXPECT_EQ(health.reason, "feedback_callback_failed");
  EXPECT_TRUE(health.owner.empty());
  const auto states_before = recorder.state_count();
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(recorder.state_count(), states_before);
  EXPECT_FALSE(session.reserve("after-feedback-failure", reason));
  EXPECT_EQ(reason, "adapter_fault");
}

TEST(VirtualControlSession, ThrowingStateCallbackStopsPublicationAndTerminatesActiveOnce)
{
  Recorder recorder;
  std::atomic<bool> throw_state{};
  std::atomic<std::size_t> callback_count{};
  VirtualControlSession session(
    [&recorder, &throw_state, &callback_count](const MeasuredState & value) {
      ++callback_count;
      if (throw_state.exchange(false)) {
        throw std::runtime_error("intentional state callback failure");
      }
      return recorder.state(value);
    }, []() {});
  std::string reason;
  ASSERT_TRUE(session.reserve("state-failure", reason));
  SessionCommand command;
  command.kind = SessionCommand::Kind::joint;
  command.owner = "state-failure";
  command.side = openarm_ik_ros::kRightSide;
  command.joint = 0U;
  command.target_rad = 0.8;
  command.terminal = [&recorder](const CommandResult & value) {
      return recorder.terminal(value);
    };
  ASSERT_TRUE(session.submit(std::move(command), reason));
  ASSERT_TRUE(wait_health(session, [](const auto & health) {
      return health.adapter_state == openarm_ik_ros::AdapterState::executing;
    }, 10s));
  throw_state = true;
  ASSERT_TRUE(wait_health(session, [](const auto & health) {
      return health.adapter_state == openarm_ik_ros::AdapterState::fault;
    }, 5s));
  ASSERT_TRUE(recorder.wait_result(2s));
  const auto health = session.health();
  EXPECT_EQ(recorder.terminal_count, 1U);
  EXPECT_EQ(recorder.result->outcome, CommandResult::Outcome::aborted);
  EXPECT_EQ(recorder.result->control_status, 0U);
  EXPECT_EQ(recorder.result->runtime_status, OA_RUNTIME_EFAULT);
  EXPECT_EQ(recorder.result->runtime_facility, OA_RUNTIME_FACILITY_RUNTIME);
  EXPECT_EQ(recorder.result->cause, 0U);
  EXPECT_EQ(recorder.result->event, OA_RUNTIME_EVENT_FAULTED);
  EXPECT_EQ(recorder.result->lifecycle, openarm_ik_ros::kLifecycleDisarmed);
  EXPECT_EQ(health.snapshot.lifecycle, openarm_ik_ros::kLifecycleDisarmed);
  EXPECT_EQ(health.last_error.status, OA_RUNTIME_EFAULT);
  EXPECT_EQ(health.reason, "state_callback_failed");
  EXPECT_TRUE(health.owner.empty());
  const auto callbacks_before = callback_count.load();
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(callback_count.load(), callbacks_before);
  EXPECT_EQ(recorder.terminal_count, 1U);
  EXPECT_FALSE(session.reserve("after-state-failure", reason));
  EXPECT_EQ(reason, "adapter_fault");
}

TEST(VirtualControlSession, SlowIdleConsumerDoesNotOwnRuntimeCadence)
{
  Recorder recorder;
  std::atomic<std::size_t> callback_count{};
  VirtualControlSession session(
    [&recorder, &callback_count](const MeasuredState & value) {
      if (++callback_count == 2U) {
        std::this_thread::sleep_for(35ms);
      }
      return recorder.state(value);
    }, []() {});
  ASSERT_TRUE(wait_health(session, [&callback_count](const auto & health) {
      return callback_count.load() >= 5U &&
             health.adapter_state == openarm_ik_ros::AdapterState::idle;
    }, 5s));
  const auto health = session.health();
  EXPECT_EQ(health.snapshot.lifecycle, openarm_ik_ros::kLifecycleArmedIdle);
  EXPECT_EQ(health.last_error.status, OA_RUNTIME_OK);
  EXPECT_EQ(health.reason, "ready");
  const auto callbacks_before = callback_count.load();
  std::this_thread::sleep_for(100ms);
  EXPECT_GT(callback_count.load(), callbacks_before);
  std::string reason;
  EXPECT_TRUE(session.reserve("after-idle-stall", reason));
  session.release("after-idle-stall", "test_release");
}

TEST(VirtualControlSession, SlowActiveConsumerDoesNotDuplicateRuntimeCadence)
{
  Recorder recorder;
  std::atomic<bool> stall{};
  VirtualControlSession session(
    [&recorder, &stall](const MeasuredState & value) {
      if (stall.exchange(false)) {
        std::this_thread::sleep_for(35ms);
      }
      return recorder.state(value);
    }, []() {});
  std::string reason;
  ASSERT_TRUE(session.reserve("deadline-fault", reason));
  SessionCommand command;
  command.kind = SessionCommand::Kind::joint;
  command.owner = "deadline-fault";
  command.side = openarm_ik_ros::kRightSide;
  command.joint = 0U;
  command.target_rad = 0.8;
  command.terminal = [&recorder](const CommandResult & value) {
      return recorder.terminal(value);
    };
  ASSERT_TRUE(session.submit(std::move(command), reason));
  ASSERT_TRUE(wait_health(session, [](const auto & health) {
      return health.adapter_state == openarm_ik_ros::AdapterState::executing;
    }, 10s));
  stall = true;
  ASSERT_TRUE(recorder.wait_result(15s));
  const auto health = session.health();
  EXPECT_EQ(recorder.terminal_count, 1U);
  EXPECT_EQ(recorder.result->outcome, CommandResult::Outcome::completed);
  EXPECT_EQ(recorder.result->control_status, 0U);
  EXPECT_EQ(recorder.result->runtime_status, OA_RUNTIME_OK);
  EXPECT_EQ(recorder.result->event, OA_RUNTIME_EVENT_COMPLETED);
  EXPECT_EQ(recorder.result->reason, "completed_measured_feedback");
  EXPECT_GT(recorder.result->seed_feedback_seq[0], 0U);
  EXPECT_GT(recorder.result->seed_feedback_seq[1], 0U);
  EXPECT_GT(recorder.result->plan_duration_ns, 0U);
  EXPECT_EQ(health.adapter_state, openarm_ik_ros::AdapterState::idle);
  EXPECT_EQ(health.reason, "completed");
  EXPECT_TRUE(health.owner.empty());
  const auto states_before = recorder.state_count();
  std::this_thread::sleep_for(100ms);
  EXPECT_GT(recorder.state_count(), states_before);
  EXPECT_EQ(recorder.terminal_count, 1U);
  EXPECT_TRUE(session.reserve("after-active-stall", reason));
  session.release("after-active-stall", "test_release");
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
  command.side = openarm_ik_ros::kRightSide;
  command.joint = 0U;
  command.target_rad = 0.8;
  command.terminal = [&recorder](const CommandResult & value) {
      return recorder.terminal(value);
    };
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
  EXPECT_EQ(recorder.result->lifecycle, openarm_ik_ros::kLifecycleDisarmed);
  EXPECT_EQ(recorder.result->event, OA_RUNTIME_EVENT_ABORTED);
  EXPECT_GT(recorder.result->seed_feedback_seq[0], 0U);
  EXPECT_GT(recorder.result->seed_feedback_seq[1], 0U);
  EXPECT_GT(recorder.result->plan_duration_ns, 0U);
  EXPECT_GE(
    recorder.result->terminal_feedback_seq[0], recorder.result->seed_feedback_seq[0]);
  EXPECT_GE(
    recorder.result->terminal_feedback_seq[1], recorder.result->seed_feedback_seq[1]);
}

TEST(VirtualControlSession, CompletionBeforeDisableStopHasOneTerminalAndDisarmedHealth)
{
  Recorder recorder;
  std::mutex barrier_mutex;
  std::condition_variable barrier_condition;
  bool captured = false;
  bool release = false;
  VirtualControlSession session(
    [&recorder](const MeasuredState & value) {return recorder.state(value);}, []() {});
  std::string reason;
  ASSERT_TRUE(session.reserve("completion-boundary", reason));
  SessionCommand command;
  command.kind = SessionCommand::Kind::joint;
  command.owner = "completion-boundary";
  command.side = openarm_ik_ros::kLeftSide;
  command.joint = 3U;
  command.target_rad = 0.0;
  command.terminal = [&recorder](const CommandResult & value) {return recorder.terminal(value);};
  command.cancel_captured_for_test = [&](std::uint64_t command_id) {
      EXPECT_EQ(command_id, 1U);
      std::unique_lock<std::mutex> lock(barrier_mutex);
      captured = true;
      barrier_condition.notify_all();
      barrier_condition.wait(lock, [&]() {return release;});
    };
  ASSERT_TRUE(session.submit(std::move(command), reason));
  ASSERT_TRUE(wait_health(session, [](const auto & health) {
      return health.adapter_state == openarm_ik_ros::AdapterState::executing;
    }, 10s));
  std::this_thread::sleep_for(60ms);
  ASSERT_TRUE(session.cancel("completion-boundary"));
  {
    std::unique_lock<std::mutex> lock(barrier_mutex);
    ASSERT_TRUE(barrier_condition.wait_for(lock, 2s, [&]() {return captured;}));
  }
  std::this_thread::sleep_for(60ms);
  {
    std::lock_guard<std::mutex> lock(barrier_mutex);
    release = true;
    barrier_condition.notify_all();
  }
  ASSERT_TRUE(recorder.wait_result(5s));
  EXPECT_EQ(recorder.terminal_count, 1U);
  EXPECT_EQ(recorder.result->outcome, CommandResult::Outcome::completed);
  EXPECT_TRUE(wait_health(session, [](const auto & health) {
      return health.adapter_state == openarm_ik_ros::AdapterState::stopped_requires_restart;
    }, 2s));
  EXPECT_FALSE(session.reserve("after-completion", reason));
  EXPECT_EQ(reason, "stopped_requires_restart");
}

TEST(VirtualControlSession, ReentrantHealthCallbackNeverRunsUnderSessionMutex)
{
  Recorder recorder;
  VirtualControlSession * session_ptr = nullptr;
  std::atomic<std::size_t> callbacks{};
  VirtualControlSession session(
    [&recorder](const MeasuredState & value) {return recorder.state(value);},
    [&]() {
      if (session_ptr != nullptr) {
        (void)session_ptr->health();
        ++callbacks;
      }
    });
  session_ptr = &session;
  std::string reason;
  ASSERT_TRUE(session.reserve("reentrant-health", reason));
  ASSERT_TRUE(wait_health(session, [&](const auto &) {return callbacks.load() >= 1U;}));
  SessionCommand command;
  command.kind = SessionCommand::Kind::joint;
  command.owner = "reentrant-health";
  command.side = openarm_ik_ros::kLeftSide;
  command.joint = 3U;
  command.target_rad = 0.02;
  command.terminal = [&recorder](const CommandResult & value) {return recorder.terminal(value);};
  ASSERT_TRUE(session.submit(std::move(command), reason));
  ASSERT_TRUE(recorder.wait_result(10s));
  EXPECT_GE(callbacks.load(), 2U);
  const auto begin = std::chrono::steady_clock::now();
  session.close();
  EXPECT_LT(std::chrono::steady_clock::now() - begin, 2s);
}

TEST(VirtualControlSession, HealthCallbackCanRequestCloseWithoutSelfJoin)
{
  Recorder recorder;
  VirtualControlSession * session_ptr = nullptr;
  std::atomic<bool> requested_close{};
  VirtualControlSession session(
    [&recorder](const MeasuredState & value) {return recorder.state(value);},
    [&]() {
      if (session_ptr != nullptr && !requested_close.exchange(true)) {
        session_ptr->close();
      }
    });
  session_ptr = &session;
  std::string reason;
  ASSERT_TRUE(session.reserve("close-from-health", reason));
  ASSERT_TRUE(wait_health(session, [&](const auto & health) {
      return requested_close.load() && health.adapter_state == openarm_ik_ros::AdapterState::closing;
    }, 2s));
  const auto begin = std::chrono::steady_clock::now();
  session.close();
  EXPECT_LT(std::chrono::steady_clock::now() - begin, 2s);
}
}
