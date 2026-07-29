// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/virtual_control_session.hpp"

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

namespace openarm_ik_ros
{
namespace
{
constexpr std::uint64_t kCycleNs = 5000000ULL;
constexpr std::uint64_t kMaximumCycleNs = 20000000ULL;
constexpr std::uint64_t kFeedbackTimeoutNs = 100000000ULL;
constexpr std::uint64_t kMaximumSkewNs = 1000000ULL;
constexpr std::uint64_t kPlanHorizonNs = 40000000000ULL;
constexpr std::uint64_t kExecutionMarginNs = 5000000000ULL;
constexpr std::uint64_t kHeartbeatAheadNs = 100000000ULL;
constexpr std::uint64_t kMaximumDurationNs = 30000000000ULL;

constexpr std::array<double, 7> kLeftLower{
  -3.490659, -3.3161253267948965, -1.570796, 0.0, -1.570796, -0.785398, -1.570796};
constexpr std::array<double, 7> kLeftUpper{
  1.396263, 0.17453267320510335, 1.570796, 2.443461, 1.570796, 0.785398, 1.570796};
constexpr std::array<double, 7> kRightLower{
  -1.396263, -0.17453267320510335, -1.570796, 0.0, -1.570796, -0.785398, -1.570796};
constexpr std::array<double, 7> kRightUpper{
  3.490659, 3.3161253267948965, 1.570796, 2.443461, 1.570796, 0.785398, 1.570796};

template<typename T>
void init(T & value)
{
  value = {};
  value.struct_size = sizeof(value);
  value.abi_version = OA_CONTROL_ABI_V1;
}

bool checked_add(const std::uint64_t left, const std::uint64_t right, std::uint64_t & result)
{
  if (left > std::numeric_limits<std::uint64_t>::max() - right) {
    return false;
  }
  result = left + right;
  return true;
}

std::uint64_t elapsed_ns(
  const std::chrono::steady_clock::time_point origin,
  const std::chrono::steady_clock::time_point now)
{
  const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - origin).count();
  return elapsed <= 0 ? 0U : static_cast<std::uint64_t>(elapsed);
}

}

class VirtualControlSession::Impl
{
public:
  Impl(StateCallback state_callback, HealthCallback health_callback)
  : state_callback_(std::move(state_callback)), health_callback_(std::move(health_callback))
  {
    worker_ = std::thread([this]() {run();});
    std::unique_lock<std::mutex> lock(mutex_);
    startup_cv_.wait(lock, [this]() {return startup_complete_;});
    if (!startup_error_.empty()) {
      lock.unlock();
      if (worker_.joinable()) {
        worker_.join();
      }
      throw std::runtime_error(startup_error_);
    }
  }

  ~Impl()
  {
    close();
  }

  bool reserve(const std::string & owner, std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closing_) {
      reason = "closing";
      return false;
    }
    if (health_.adapter_state == AdapterState::fault) {
      reason = "adapter_fault";
      return false;
    }
    if (health_.adapter_state == AdapterState::stopped_requires_restart) {
      reason = "stopped_requires_restart";
      return false;
    }
    if (!health_.owner.empty() || pending_.has_value() || active_.has_value()) {
      reason = "busy";
      return false;
    }
    health_.owner = owner;
    health_.adapter_state = AdapterState::reserved;
    health_.reason = "reserved";
    notify_health_unlocked();
    return true;
  }

  bool submit(SessionCommand command, std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closing_ || health_.owner != command.owner || pending_.has_value() || active_.has_value()) {
      reason = closing_ ? "closing" : "reservation_mismatch";
      return false;
    }
    pending_ = std::move(command);
    cv_.notify_all();
    return true;
  }

  bool cancel(const std::string & owner)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closing_ || health_.owner != owner) {
      return false;
    }
    cancel_owner_ = owner;
    cv_.notify_all();
    return true;
  }

  void release(const std::string & owner, const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (health_.owner == owner && !pending_.has_value() && !active_.has_value()) {
      health_.owner.clear();
      health_.adapter_state = AdapterState::idle;
      health_.reason = reason;
      notify_health_unlocked();
    }
  }

  SessionHealth health() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return health_;
  }

  void close() noexcept
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closing_) {
        return;
      }
      closing_ = true;
      health_.adapter_state = AdapterState::closing;
      health_.reason = "shutdown";
      cv_.notify_all();
    }
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  struct Active
  {
    SessionCommand command;
    oa_motion_plan_report report{};
    std::uint64_t command_id{};
    std::uint64_t start_ns{};
    double start_q[2][7]{};
  };

  void signal_startup(std::string error = {})
  {
    std::lock_guard<std::mutex> lock(mutex_);
    startup_error_ = std::move(error);
    startup_complete_ = true;
    startup_cv_.notify_all();
  }

  void run() noexcept
  {
    try {
      startup();
    } catch (const std::exception & error) {
      destroy_handles();
      signal_startup(error.what());
      return;
    }
    signal_startup();

    auto next_cycle = origin_ + std::chrono::nanoseconds(kCycleNs);
    for (;;) {
      bool should_close = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_until(lock, next_cycle, [this]() {
          return closing_ || pending_.has_value() || !cancel_owner_.empty();
        });
        should_close = closing_;
      }
      if (should_close) {
        shutdown_on_owner_thread();
        break;
      }

      process_cancel();
      process_pending();

      const auto now = std::chrono::steady_clock::now();
      if (now < next_cycle) {
        continue;
      }
      const std::uint64_t controller_now = elapsed_ns(origin_, now);
      if (!heartbeat(controller_now)) {
        break;
      }
      const oa_control_status advance_status = oa_controller_advance(controller_, controller_now);
      if (advance_status != OA_CONTROL_OK) {
        fault(advance_status, "advance_failed");
        break;
      }
      controller_now_ns_ = controller_now;
      if (!publish_measured()) {
        break;
      }
      drain_events();
      next_cycle += std::chrono::nanoseconds(kCycleNs);
      if (next_cycle <= now) {
        next_cycle = now + std::chrono::nanoseconds(kCycleNs);
      }
    }
    destroy_handles();
  }

  void startup()
  {
    oa_control_status status = oa_manifest_create_openarm_v10_virtual(&manifest_);
    if (status != OA_CONTROL_OK) {
      throw std::runtime_error("standard virtual manifest creation failed: " + std::to_string(status));
    }
    oa_controller_options options{};
    init(options);
    options.backend = OA_BACKEND_VIRTUAL;
    options.collision_policy = OA_COLLISION_VIRTUAL_UNCHECKED;
    options.cycle_ns = kMaximumCycleNs;
    options.feedback_timeout_ns = kFeedbackTimeoutNs;
    options.max_cross_bus_skew_ns = kMaximumSkewNs;
    options.collision_scene_revision = 1U;
    status = oa_controller_create(manifest_, &options, &controller_);
    if (status != OA_CONTROL_OK) {
      throw std::runtime_error("virtual controller creation failed: " + std::to_string(status));
    }
    oa_verify_report verify{};
    init(verify);
    status = oa_controller_open_and_verify(controller_, &verify);
    if (status != OA_CONTROL_OK || verify.verified_mask != 0x3U || verify.failure_mask != 0U) {
      throw std::runtime_error("virtual controller verification failed: " + std::to_string(status));
    }
    oa_arm_challenge challenge{};
    init(challenge);
    status = oa_controller_get_arm_challenge(controller_, &challenge);
    if (status != OA_CONTROL_OK) {
      throw std::runtime_error("virtual arm challenge failed: " + std::to_string(status));
    }
    status = oa_controller_arm(controller_, &challenge);
    if (status != OA_CONTROL_OK) {
      throw std::runtime_error("virtual arming failed: " + std::to_string(status));
    }
    origin_ = std::chrono::steady_clock::now();
    controller_now_ns_ = 0U;
    if (!snapshot_valid(true)) {
      throw std::runtime_error("initial measured snapshot is invalid");
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      health_.adapter_state = AdapterState::idle;
      health_.verify_epoch = verify.verify_epoch;
      health_.reason = "ready";
    }
    if (!state_callback_(MeasuredState{snapshot_, controller_now_ns_})) {
      throw std::runtime_error("initial measured state publication failed");
    }
  }

  bool snapshot_valid(const bool initial)
  {
    oa_snapshot next{};
    init(next);
    const oa_control_status status = oa_controller_snapshot(controller_, &next);
    if (status != OA_CONTROL_OK) {
      return false;
    }
    for (std::size_t side = 0; side < 2U; ++side) {
      const auto & arm = next.arm[side];
      if (arm.expected_mask != 0x7fU || arm.fresh_mask != arm.expected_mask ||
        arm.fault_mask != 0U || (!initial && arm.feedback_seq <= snapshot_.arm[side].feedback_seq))
      {
        return false;
      }
    }
    const std::uint64_t skew = next.arm[0].t_ns > next.arm[1].t_ns ?
      next.arm[0].t_ns - next.arm[1].t_ns : next.arm[1].t_ns - next.arm[0].t_ns;
    if (skew > next.max_cross_bus_skew_ns ||
      (next.lifecycle != OA_LIFECYCLE_ARMED_IDLE && next.lifecycle != OA_LIFECYCLE_EXECUTING))
    {
      return false;
    }
    snapshot_ = next;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      health_.snapshot = snapshot_;
      health_.controller_now_ns = controller_now_ns_;
    }
    return true;
  }

  bool publish_measured()
  {
    if (!snapshot_valid(false)) {
      fault(OA_CONTROL_ESTALE, "invalid_measured_snapshot");
      return false;
    }
    if (!state_callback_(MeasuredState{snapshot_, controller_now_ns_})) {
      fault(OA_CONTROL_ETIMEOUT, "publication_clock_invalid");
      return false;
    }
    publish_feedback();
    return true;
  }

  void process_pending()
  {
    std::optional<SessionCommand> pending;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!pending_) {
        return;
      }
      pending = std::move(pending_);
      pending_.reset();
    }

    oa_motion_plan *plan = nullptr;
    oa_control_status status = OA_CONTROL_EINVAL;
    std::uint64_t plan_expiry{};
    if (!checked_add(controller_now_ns_, kPlanHorizonNs, plan_expiry)) {
      reject_on_owner(std::move(*pending), OA_CONTROL_EINVAL, "plan_deadline_overflow");
      return;
    }
    if (pending->kind == SessionCommand::Kind::joint) {
      oa_joint_move move{};
      init(move);
      move.expiry_ns = plan_expiry;
      move.required_feedback_seq = snapshot_.arm[pending->side].feedback_seq;
      move.side = pending->side;
      move.joint = pending->joint;
      move.target_rad = pending->target_rad;
      move.velocity_scale = 0.5;
      move.acceleration_scale = 0.5;
      move.jerk_scale = 0.5;
      move.position_tol_rad = 5.0e-4;
      move.velocity_tol_rad_s = 2.0e-2;
      status = oa_controller_plan_joint(controller_, &move, &plan);
    } else {
      oa_paired_tcp_move move{};
      init(move);
      move.expiry_ns = plan_expiry;
      move.required_feedback_seq[0] = snapshot_.arm[0].feedback_seq;
      move.required_feedback_seq[1] = snapshot_.arm[1].feedback_seq;
      std::copy(pending->left_tcp_m.begin(), pending->left_tcp_m.end(), move.left_tcp_m);
      std::copy(pending->right_tcp_m.begin(), pending->right_tcp_m.end(), move.right_tcp_m);
      move.velocity_scale = 0.5;
      move.acceleration_scale = 0.5;
      move.jerk_scale = 0.5;
      move.tcp_tol_m = 1.0e-3;
      move.collision_scene_revision = 1U;
      move.max_branch_step_rad = 2.0;
      move.min_singular_value = 0.0;
      status = oa_controller_plan_paired_tcp(controller_, &move, &plan);
    }
    if (status != OA_CONTROL_OK) {
      reject_on_owner(std::move(*pending), status, "planning_failed");
      return;
    }

    oa_motion_plan_report report{};
    init(report);
    status = oa_motion_plan_get_report(plan, &report);
    if (status != OA_CONTROL_OK || report.duration_ns > kMaximumDurationNs) {
      oa_motion_plan_destroy(plan);
      reject_on_owner(
        std::move(*pending), status == OA_CONTROL_OK ? OA_CONTROL_ETIMEOUT : status,
        "plan_duration_rejected");
      return;
    }
    std::uint64_t expiry{};
    if (!checked_add(controller_now_ns_, report.duration_ns, expiry) ||
      !checked_add(expiry, kExecutionMarginNs, expiry))
    {
      oa_motion_plan_destroy(plan);
      reject_on_owner(std::move(*pending), OA_CONTROL_EINVAL, "execution_deadline_overflow");
      return;
    }
    std::uint64_t producer_deadline{};
    if (!checked_add(controller_now_ns_, kHeartbeatAheadNs, producer_deadline)) {
      oa_motion_plan_destroy(plan);
      reject_on_owner(std::move(*pending), OA_CONTROL_EINVAL, "heartbeat_deadline_overflow");
      return;
    }
    oa_execute_request execute{};
    init(execute);
    execute.start_ns = controller_now_ns_;
    execute.expiry_ns = expiry;
    execute.producer_deadline_ns = producer_deadline;
    execute.stop_kind = OA_STOP_DISABLE;
    std::uint64_t command_id{};
    status = oa_controller_execute(controller_, plan, &execute, &command_id);
    oa_motion_plan_destroy(plan);
    if (status != OA_CONTROL_OK) {
      reject_on_owner(std::move(*pending), status, "execute_failed");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      Active active;
      active.command = std::move(*pending);
      active.report = report;
      active.command_id = command_id;
      active.start_ns = controller_now_ns_;
      for (std::size_t side = 0; side < 2U; ++side) {
        std::copy(
          std::begin(snapshot_.arm[side].q), std::end(snapshot_.arm[side].q),
          std::begin(active.start_q[side]));
      }
      active_ = std::move(active);
      health_.adapter_state = AdapterState::executing;
      health_.command_id = command_id;
      health_.reason = "executing";
      notify_health_unlocked();
    }
    drain_events();
  }

  void reject_on_owner(SessionCommand command, oa_control_status status, const std::string & reason)
  {
    CommandResult result;
    result.outcome = CommandResult::Outcome::rejected;
    result.control_status = status;
    result.seed_feedback_seq[0] = snapshot_.arm[0].feedback_seq;
    result.seed_feedback_seq[1] = snapshot_.arm[1].feedback_seq;
    result.terminal_feedback_seq[0] = snapshot_.arm[0].feedback_seq;
    result.terminal_feedback_seq[1] = snapshot_.arm[1].feedback_seq;
    result.lifecycle = snapshot_.lifecycle;
    result.collision_checked = false;
    result.reason = reason;
    if (command.terminal) {
      command.terminal(result);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    health_.owner.clear();
    health_.adapter_state = AdapterState::idle;
    health_.reason = reason;
    health_.last_cause = status;
    notify_health_unlocked();
  }

  bool heartbeat(const std::uint64_t now_ns)
  {
    std::uint64_t deadline{};
    std::uint64_t command_id{};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_) {
        return true;
      }
      command_id = active_->command_id;
    }
    if (!checked_add(now_ns, kHeartbeatAheadNs, deadline)) {
      fault(OA_CONTROL_EINVAL, "heartbeat_deadline_overflow");
      return false;
    }
    const auto status = oa_controller_heartbeat(controller_, command_id, deadline);
    if (status != OA_CONTROL_OK) {
      fault(status, "heartbeat_failed");
      return false;
    }
    return true;
  }

  void drain_events()
  {
    for (;;) {
      oa_event event{};
      init(event);
      const oa_control_status status = oa_controller_poll_event(controller_, 0U, &event);
      if (status == OA_CONTROL_ETIMEOUT) {
        return;
      }
      if (status != OA_CONTROL_OK) {
        fault(status, "event_poll_failed");
        return;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        health_.last_event = event.kind;
        health_.last_cause = event.cause;
        notify_health_unlocked();
      }
      if (event.kind == OA_EVENT_COMPLETED) {
        complete_active(event);
      } else if (event.kind == OA_EVENT_FAULTED || event.kind == OA_EVENT_ESTOP) {
        fault(event.cause, "controller_fault_event");
        return;
      }
    }
  }

  void complete_active(const oa_event & event)
  {
    std::optional<Active> completed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || event.command_id != active_->command_id) {
        return;
      }
      completed = std::move(active_);
      active_.reset();
      health_.command_id = 0U;
      health_.owner.clear();
      health_.adapter_state = AdapterState::idle;
      health_.reason = "completed";
    }
    CommandResult result;
    result.outcome = CommandResult::Outcome::completed;
    result.control_status = OA_CONTROL_OK;
    result.command_id = event.command_id;
    result.seed_feedback_seq[0] = completed->report.seed_feedback_seq[0];
    result.seed_feedback_seq[1] = completed->report.seed_feedback_seq[1];
    result.terminal_feedback_seq[0] = snapshot_.arm[0].feedback_seq;
    result.terminal_feedback_seq[1] = snapshot_.arm[1].feedback_seq;
    result.lifecycle = event.lifecycle;
    result.event = event.kind;
    result.cause = event.cause;
    result.collision_checked = completed->report.collision_checked != 0U;
    result.reason = "completed_measured_feedback";
    if (completed->command.terminal) {
      completed->command.terminal(result);
    }
    health_callback_();
  }

  void publish_feedback()
  {
    std::function<void(const CommandFeedback &)> callback;
    CommandFeedback feedback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || !active_->command.feedback) {
        return;
      }
      callback = active_->command.feedback;
      feedback.lifecycle = snapshot_.lifecycle;
      feedback.event = health_.last_event;
      feedback.command_id = active_->command_id;
      feedback.feedback_seq[0] = snapshot_.arm[0].feedback_seq;
      feedback.feedback_seq[1] = snapshot_.arm[1].feedback_seq;
      double start_squared = 0.0;
      double remaining_squared = 0.0;
      for (std::size_t side = 0; side < 2U; ++side) {
        for (std::size_t joint = 0; joint < 7U; ++joint) {
          const double start_delta =
            active_->report.target_q[side][joint] - active_->start_q[side][joint];
          const double remaining_delta =
            active_->report.target_q[side][joint] - snapshot_.arm[side].q[joint];
          start_squared += start_delta * start_delta;
          remaining_squared += remaining_delta * remaining_delta;
        }
      }
      feedback.measured_progress = start_squared <= 1.0e-18 ? 1.0 :
        std::clamp(1.0 - std::sqrt(remaining_squared / start_squared), 0.0, 1.0);
    }
    callback(feedback);
  }

  void process_cancel()
  {
    std::string owner;
    std::optional<SessionCommand> pending;
    std::optional<Active> active;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (cancel_owner_.empty()) {
        return;
      }
      owner = std::move(cancel_owner_);
      cancel_owner_.clear();
      if (pending_ && pending_->owner == owner) {
        pending = std::move(pending_);
        pending_.reset();
      }
      if (active_ && active_->command.owner == owner) {
        active = std::move(active_);
        active_.reset();
      }
    }
    if (active) {
      (void)oa_controller_stop(controller_, OA_STOP_DISABLE);
      drain_events();
    }
    SessionCommand * command = pending ? &*pending : (active ? &active->command : nullptr);
    if (command != nullptr && command->terminal) {
      CommandResult result;
      result.outcome = CommandResult::Outcome::canceled;
      result.control_status = OA_CONTROL_OK;
      result.command_id = active ? active->command_id : 0U;
      if (active) {
        result.seed_feedback_seq[0] = active->report.seed_feedback_seq[0];
        result.seed_feedback_seq[1] = active->report.seed_feedback_seq[1];
      }
      result.terminal_feedback_seq[0] = snapshot_.arm[0].feedback_seq;
      result.terminal_feedback_seq[1] = snapshot_.arm[1].feedback_seq;
      result.lifecycle = OA_LIFECYCLE_DISARMED;
      result.event = OA_EVENT_ABORTED;
      result.reason = "canceled_disable_stop";
      command->terminal(result);
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      health_.owner.clear();
      health_.command_id = 0U;
      health_.adapter_state = AdapterState::stopped_requires_restart;
      health_.reason = "canceled_disable_stop";
      notify_health_unlocked();
    }
  }

  void fault(const oa_control_status cause, const std::string & reason)
  {
    (void)oa_controller_stop(controller_, OA_STOP_DISABLE);
    std::optional<Active> active;
    std::optional<SessionCommand> pending;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active = std::move(active_);
      active_.reset();
      pending = std::move(pending_);
      pending_.reset();
      health_.adapter_state = AdapterState::fault;
      health_.last_cause = cause;
      health_.reason = reason;
      health_.command_id = 0U;
      health_.owner.clear();
      notify_health_unlocked();
    }
    SessionCommand * command = active ? &active->command : (pending ? &*pending : nullptr);
    if (command != nullptr && command->terminal) {
      CommandResult result;
      result.outcome = CommandResult::Outcome::aborted;
      result.control_status = cause;
      result.command_id = active ? active->command_id : 0U;
      result.terminal_feedback_seq[0] = snapshot_.arm[0].feedback_seq;
      result.terminal_feedback_seq[1] = snapshot_.arm[1].feedback_seq;
      result.lifecycle = OA_LIFECYCLE_FAULT;
      result.event = OA_EVENT_FAULTED;
      result.cause = cause;
      result.reason = reason;
      command->terminal(result);
    }
  }

  void shutdown_on_owner_thread()
  {
    std::optional<Active> active;
    std::optional<SessionCommand> pending;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active = std::move(active_);
      active_.reset();
      pending = std::move(pending_);
      pending_.reset();
    }
    if (active) {
      (void)oa_controller_stop(controller_, OA_STOP_DISABLE);
    } else if (snapshot_.lifecycle == OA_LIFECYCLE_ARMED_IDLE) {
      (void)oa_controller_disarm(controller_, controller_now_ns_ + kHeartbeatAheadNs);
    }
    SessionCommand * command = active ? &active->command : (pending ? &*pending : nullptr);
    if (command != nullptr && command->terminal) {
      CommandResult result;
      result.outcome = CommandResult::Outcome::aborted;
      result.control_status = OA_CONTROL_ESTATE;
      result.command_id = active ? active->command_id : 0U;
      result.terminal_feedback_seq[0] = snapshot_.arm[0].feedback_seq;
      result.terminal_feedback_seq[1] = snapshot_.arm[1].feedback_seq;
      result.lifecycle = OA_LIFECYCLE_DISARMED;
      result.event = OA_EVENT_ABORTED;
      result.reason = "shutdown";
      command->terminal(result);
    }
  }

  void destroy_handles() noexcept
  {
    oa_controller_destroy(controller_);
    controller_ = nullptr;
    oa_manifest_destroy(manifest_);
    manifest_ = nullptr;
  }

  void notify_health_unlocked()
  {
    cv_.notify_all();
    health_callback_();
  }

  StateCallback state_callback_;
  HealthCallback health_callback_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::condition_variable startup_cv_;
  std::thread worker_;
  bool startup_complete_{};
  bool closing_{};
  std::string startup_error_;
  std::string cancel_owner_;
  std::optional<SessionCommand> pending_;
  std::optional<Active> active_;
  SessionHealth health_;
  oa_manifest * manifest_{};
  oa_controller * controller_{};
  oa_snapshot snapshot_{};
  std::chrono::steady_clock::time_point origin_{};
  std::uint64_t controller_now_ns_{};
};

VirtualControlSession::VirtualControlSession(
  StateCallback state_callback, HealthCallback health_callback)
: impl_(std::make_unique<Impl>(std::move(state_callback), std::move(health_callback)))
{
}

VirtualControlSession::~VirtualControlSession() = default;

bool VirtualControlSession::reserve(const std::string & owner, std::string & reason)
{
  return impl_->reserve(owner, reason);
}

bool VirtualControlSession::submit(SessionCommand command, std::string & reason)
{
  return impl_->submit(std::move(command), reason);
}

bool VirtualControlSession::cancel(const std::string & owner)
{
  return impl_->cancel(owner);
}

void VirtualControlSession::release(const std::string & owner, const std::string & reason)
{
  impl_->release(owner, reason);
}

SessionHealth VirtualControlSession::health() const
{
  return impl_->health();
}

void VirtualControlSession::close() noexcept
{
  if (impl_) {
    impl_->close();
  }
}

const std::array<std::string, 14> & VirtualControlSession::joint_names()
{
  static const std::array<std::string, 14> names{
    "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
    "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
    "openarm_left_joint7", "openarm_right_joint1", "openarm_right_joint2",
    "openarm_right_joint3", "openarm_right_joint4", "openarm_right_joint5",
    "openarm_right_joint6", "openarm_right_joint7"};
  return names;
}

bool VirtualControlSession::map_joint(
  const std::string & name, oa_side & side, std::uint32_t & joint)
{
  const auto & names = joint_names();
  const auto found = std::find(names.begin(), names.end(), name);
  if (found == names.end()) {
    return false;
  }
  const auto index = static_cast<std::size_t>(std::distance(names.begin(), found));
  side = index < 7U ? OA_LEFT : OA_RIGHT;
  joint = static_cast<std::uint32_t>(index % 7U);
  return true;
}

bool VirtualControlSession::joint_target_in_limits(
  const oa_side side, const std::uint32_t joint, const double target)
{
  if (side > OA_RIGHT || joint >= 7U || !std::isfinite(target)) {
    return false;
  }
  const auto & lower = side == OA_LEFT ? kLeftLower : kRightLower;
  const auto & upper = side == OA_LEFT ? kLeftUpper : kRightUpper;
  return target >= lower[joint] && target <= upper[joint];
}

const char * VirtualControlSession::adapter_state_name(const AdapterState state) noexcept
{
  switch (state) {
    case AdapterState::starting: return "starting";
    case AdapterState::idle: return "idle";
    case AdapterState::reserved: return "reserved";
    case AdapterState::executing: return "executing";
    case AdapterState::stopped_requires_restart: return "stopped_requires_restart";
    case AdapterState::fault: return "fault";
    case AdapterState::closing: return "closing";
  }
  return "unknown";
}

}
