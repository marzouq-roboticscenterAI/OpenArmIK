// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/virtual_control_session.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
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
constexpr std::uint64_t kPollCycleNs = 5000000ULL;
constexpr std::uint64_t kFeedbackTimeoutNs = 100000000ULL;
constexpr std::uint64_t kMaximumSkewNs = 1000000ULL;
constexpr std::uint64_t kPlanHorizonNs = 40000000000ULL;
constexpr std::uint64_t kExecutionMarginNs = 5000000000ULL;
constexpr std::uint64_t kHeartbeatAheadNs = 100000000ULL;
constexpr std::uint64_t kMaximumDurationNs = 30000000000ULL;
constexpr std::uint64_t kCollisionSceneRevision = 1U;
constexpr std::uint32_t kExpectedMask = 0x7fU;

template<typename T>
void init(T & value)
{
  value = {};
  value.struct_size = sizeof(value);
  value.abi_version = OA_RUNTIME_ABI_VERSION;
}

bool checked_add(const std::uint64_t left, const std::uint64_t right, std::uint64_t & result)
{
  if (left > std::numeric_limits<std::uint64_t>::max() - right) {
    return false;
  }
  result = left + right;
  return true;
}

bool valid_digest(const char * digest)
{
  return digest != nullptr && std::strlen(digest) == 64U;
}

struct ManifestDescription
{
  oa_runtime_manifest_summary summary{};
  std::array<oa_runtime_motor_manifest, 14> motor{};
};

const ManifestDescription & standard_manifest()
{
  static const ManifestDescription description = []() {
      oa_runtime_manifest * manifest = nullptr;
      if (oa_runtime_manifest_create_virtual(&manifest) != OA_RUNTIME_OK || manifest == nullptr) {
        throw std::runtime_error("canonical runtime virtual manifest creation failed");
      }
      ManifestDescription value;
      init(value.summary);
      if (oa_runtime_manifest_get_summary(manifest, &value.summary) != OA_RUNTIME_OK) {
        oa_runtime_manifest_destroy(manifest);
        throw std::runtime_error("canonical runtime virtual manifest summary failed");
      }
      for (std::uint32_t side = 0U; side < OA_RUNTIME_ARMS; ++side) {
        for (std::uint32_t joint = 0U; joint < OA_RUNTIME_DOF; ++joint) {
          auto & motor = value.motor[side * OA_RUNTIME_DOF + joint];
          init(motor);
          if (oa_runtime_manifest_get_motor(manifest, side, joint, &motor) != OA_RUNTIME_OK) {
            oa_runtime_manifest_destroy(manifest);
            throw std::runtime_error("canonical runtime virtual motor manifest failed");
          }
        }
      }
      oa_runtime_manifest_destroy(manifest);
      return value;
    }();
  return description;
}

bool allowed_lifecycle(const std::uint32_t lifecycle)
{
  return lifecycle == kLifecycleArmedIdle || lifecycle == kLifecycleExecuting ||
         lifecycle == kLifecycleDisarmed || lifecycle == kLifecycleFault ||
         lifecycle == kLifecycleEstop;
}

}

class VirtualControlSession::Impl
{
public:
  Impl(StateCallback state_callback, HealthCallback health_callback)
  : state_callback_(std::move(state_callback)), health_callback_(std::move(health_callback))
  {
    init(health_.last_error);
    health_.last_error.status = OA_RUNTIME_OK;
    health_.last_error.facility = OA_RUNTIME_FACILITY_RUNTIME;
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
    if (closing_ || health_.owner != owner || !terminalizing_owner_.empty()) {
      return false;
    }
    cancel_owner_ = owner;
    cv_.notify_all();
    return true;
  }

  void release(const std::string & owner, const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (health_.owner == owner && !pending_.has_value() && !active_.has_value() &&
      terminalizing_owner_.empty())
    {
      health_.owner.clear();
      if (cancelled_reservation_owner_ == owner) {
        cancelled_reservation_owner_.clear();
      }
      if (health_.adapter_state != AdapterState::stopped_requires_restart &&
        health_.adapter_state != AdapterState::fault &&
        health_.adapter_state != AdapterState::closing)
      {
        health_.adapter_state = AdapterState::idle;
      }
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
      if (health_.adapter_state != AdapterState::fault) {
        health_.adapter_state = AdapterState::closing;
        health_.reason = "shutdown";
      }
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
    oa_runtime_plan_report report{};
    std::uint64_t command_id{};
    double start_q[2][7]{};
  };

  enum class RefreshResult {updated, unchanged, failed};

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

    try {
      auto next_poll = std::chrono::steady_clock::now() + std::chrono::nanoseconds(kPollCycleNs);
      for (;;) {
        bool should_close = false;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          cv_.wait_until(lock, next_poll, [this]() {
            return closing_ || pending_.has_value() || !cancel_owner_.empty();
          });
          if (!closing_ && health_.adapter_state == AdapterState::stopped_requires_restart &&
            !pending_.has_value() && cancel_owner_.empty())
          {
            cv_.wait(lock, [this]() {
              return closing_ || pending_.has_value() || !cancel_owner_.empty();
            });
          }
          should_close = closing_;
        }
        if (should_close) {
          shutdown_on_owner_thread();
          break;
        }

        process_cancel();
        process_pending();
        if (adapter_faulted()) {
          break;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (health_.adapter_state == AdapterState::stopped_requires_restart) {
            continue;
          }
        }

        const auto steady_now = std::chrono::steady_clock::now();
        if (steady_now < next_poll) {
          continue;
        }
        if (!heartbeat()) {
          break;
        }
        const RefreshResult refreshed = refresh_snapshot(true, true);
        if (refreshed == RefreshResult::failed) {
          fault_from_status(OA_RUNTIME_ESTALE, "invalid_runtime_snapshot");
          break;
        }
        if (refreshed == RefreshResult::updated && !publish_measured()) {
          break;
        }
        drain_events();
        if (adapter_faulted()) {
          break;
        }
        next_poll += std::chrono::nanoseconds(kPollCycleNs);
        if (next_poll <= steady_now) {
          next_poll = steady_now + std::chrono::nanoseconds(kPollCycleNs);
        }
      }
    } catch (...) {
      try {
        fault_local(OA_RUNTIME_EFAULT, "worker_exception");
      } catch (...) {
      }
    }
    destroy_handles();
  }

  void startup()
  {
    oa_runtime_status status = oa_runtime_manifest_create_virtual(&manifest_);
    if (status != OA_RUNTIME_OK || manifest_ == nullptr) {
      throw std::runtime_error("runtime virtual manifest creation failed: " + std::to_string(status));
    }
    init(health_.manifest);
    status = oa_runtime_manifest_get_summary(manifest_, &health_.manifest);
    if (status != OA_RUNTIME_OK || health_.manifest.state != OA_RUNTIME_MANIFEST_ARMABLE ||
      health_.manifest.intended_backend != OA_RUNTIME_BACKEND_VIRTUAL ||
      !valid_digest(health_.manifest.content_sha256))
    {
      throw std::runtime_error("runtime virtual manifest identity is invalid");
    }

    oa_runtime_options options{};
    init(options);
    options.backend = OA_RUNTIME_BACKEND_VIRTUAL;
    options.allow_unchecked_virtual_motion = 1U;
    options.cycle_ns = kPollCycleNs;
    options.feedback_timeout_ns = kFeedbackTimeoutNs;
    options.maximum_cross_bus_skew_ns = kMaximumSkewNs;
    options.collision_scene_revision = kCollisionSceneRevision;
    status = oa_runtime_create(&options, manifest_, &runtime_);
    if (status != OA_RUNTIME_OK || runtime_ == nullptr) {
      throw std::runtime_error("virtual runtime creation failed: " + std::to_string(status));
    }

    init(health_.capabilities);
    status = oa_runtime_get_capabilities(runtime_, &health_.capabilities);
    const oa_runtime_capability required = OA_RUNTIME_CAP_VIRTUAL_COORDINATES |
      OA_RUNTIME_CAP_VIRTUAL_JOINT_MOTION | OA_RUNTIME_CAP_VIRTUAL_PAIRED_XYZ_MOTION |
      OA_RUNTIME_CAP_MANIFEST_PREVIEW | OA_RUNTIME_CAP_MANIFEST_PERSISTENCE;
    const oa_runtime_capability forbidden = OA_RUNTIME_CAP_PHYSICAL_CONFIGURATION |
      OA_RUNTIME_CAP_PHYSICAL_CALIBRATION_MOTION | OA_RUNTIME_CAP_PHYSICAL_MOTION |
      OA_RUNTIME_CAP_COLLISION_VALIDATED_MOTION | OA_RUNTIME_CAP_SINGLE_XYZ_IK;
    if (status != OA_RUNTIME_OK || health_.capabilities.backend != OA_RUNTIME_BACKEND_VIRTUAL ||
      health_.capabilities.clock_id != OA_RUNTIME_CLOCK_MONOTONIC ||
      health_.capabilities.units_id != OA_RUNTIME_UNITS_SI_V1 ||
      health_.capabilities.xyz_frame_id != OA_RUNTIME_FRAME_OPENARM_BODY_LINK0 ||
      health_.capabilities.orientation_policy != OA_RUNTIME_ORIENTATION_FREE ||
      health_.capabilities.collision_policy != OA_RUNTIME_COLLISION_VIRTUAL_UNCHECKED ||
      health_.capabilities.collision_checked != 0U ||
      (health_.capabilities.capabilities & required) != required ||
      (health_.capabilities.capabilities & forbidden) != 0U ||
      !valid_digest(health_.capabilities.coordinate_identity_sha256))
    {
      throw std::runtime_error("runtime capability contract mismatch");
    }

    for (std::uint32_t side = 0U; side < OA_RUNTIME_ARMS; ++side) {
      init(health_.model_identity[side]);
      status = oa_runtime_get_model_identity(runtime_, side, &health_.model_identity[side]);
      const auto & identity = health_.model_identity[side];
      if (status != OA_RUNTIME_OK || identity.side != side ||
        identity.model_revision != health_.manifest.model_revision ||
        identity.tcp_revision == 0U || identity.collision_scene_revision != kCollisionSceneRevision ||
        identity.xyz_frame_id != OA_RUNTIME_FRAME_OPENARM_BODY_LINK0 ||
        identity.units_id != OA_RUNTIME_UNITS_SI_V1 ||
        identity.orientation_policy != OA_RUNTIME_ORIENTATION_FREE ||
        identity.collision_policy != OA_RUNTIME_COLLISION_VIRTUAL_UNCHECKED ||
        std::strcmp(identity.coordinate_identity_sha256,
        health_.capabilities.coordinate_identity_sha256) != 0 ||
        !valid_digest(identity.model_data_sha256) || !valid_digest(identity.flattened_urdf_sha256) ||
        identity.model_id[0] == '\0' || identity.tcp_frame[0] == '\0')
      {
        throw std::runtime_error("runtime model/TCP identity mismatch");
      }
    }

    oa_runtime_inventory * inventory = nullptr;
    status = oa_runtime_inventory_query(runtime_, nullptr, &inventory);
    if (status != OA_RUNTIME_OK || inventory == nullptr) {
      throw std::runtime_error("runtime virtual discovery failed");
    }
    init(health_.inventory);
    status = oa_runtime_inventory_get_summary(inventory, &health_.inventory);
    oa_runtime_inventory_destroy(inventory);
    if (status != OA_RUNTIME_OK || health_.inventory.interface_count != 2U ||
      health_.inventory.motor_count != 14U || health_.inventory.unknown_mask != 0U ||
      health_.inventory.ambiguous_mask != 0U || health_.inventory.conflict_mask != 0U ||
      health_.inventory.unresolved_assignment != 0U ||
      !valid_digest(health_.inventory.fingerprint_sha256))
    {
      throw std::runtime_error("runtime virtual inventory is not exact");
    }

    status = oa_runtime_set_interlock(runtime_, 0U, 1U);
    if (status != OA_RUNTIME_OK) {
      throw std::runtime_error("runtime virtual interlock failed: " + std::to_string(status));
    }
    status = oa_runtime_arm_virtual(runtime_);
    if (status != OA_RUNTIME_OK) {
      throw std::runtime_error("runtime virtual arm failed: " + std::to_string(status));
    }
    if (refresh_snapshot(false, true) == RefreshResult::failed) {
      throw std::runtime_error("initial runtime measured snapshot is invalid");
    }
    drain_events();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      health_.adapter_state = AdapterState::idle;
      health_.reason = "ready";
    }
    if (invoke_state_callback(MeasuredState{snapshot_, runtime_now_ns_}) != OA_RUNTIME_OK) {
      throw std::runtime_error("initial runtime measured state publication failed");
    }
  }

  RefreshResult refresh_snapshot(const bool require_new, const bool require_fresh)
  {
    oa_runtime_snapshot next{};
    init(next);
    const oa_runtime_status snapshot_status = oa_runtime_snapshot_get(runtime_, &next);
    if (snapshot_status != OA_RUNTIME_OK) {
      remember_error(snapshot_status);
      return RefreshResult::failed;
    }
    std::uint64_t now = 0U;
    const oa_runtime_status now_status = oa_runtime_now_monotonic_ns(
      runtime_, OA_RUNTIME_CLOCK_MONOTONIC, &now);
    if (now_status != OA_RUNTIME_OK) {
      remember_error(now_status);
      return RefreshResult::failed;
    }
    if (next.clock_id != OA_RUNTIME_CLOCK_MONOTONIC || next.units_id != OA_RUNTIME_UNITS_SI_V1 ||
      next.frame_id != OA_RUNTIME_FRAME_OPENARM_BODY_LINK0 ||
      next.manifest_revision != health_.manifest.manifest_revision ||
      next.model_revision != health_.manifest.model_revision ||
      std::strcmp(next.coordinate_identity_sha256,
      health_.capabilities.coordinate_identity_sha256) != 0 || !allowed_lifecycle(next.lifecycle))
    {
      return RefreshResult::failed;
    }
    bool all_equal = snapshot_.arm[0].feedback_seq != 0U;
    for (std::size_t side = 0U; side < OA_RUNTIME_ARMS; ++side) {
      const auto & arm = next.arm[side];
      all_equal = all_equal && arm.feedback_seq == snapshot_.arm[side].feedback_seq;
      if (arm.feedback_seq == 0U || arm.feedback_seq < snapshot_.arm[side].feedback_seq ||
        arm.expected_mask != kExpectedMask || arm.fault_mask != 0U ||
        arm.measurement_runtime_monotonic_ns == 0U ||
        arm.measurement_runtime_monotonic_ns > now ||
        (require_fresh && arm.fresh_mask != arm.expected_mask))
      {
        return RefreshResult::failed;
      }
    }
    const std::uint64_t left_time = next.arm[0].measurement_runtime_monotonic_ns;
    const std::uint64_t right_time = next.arm[1].measurement_runtime_monotonic_ns;
    const std::uint64_t skew = left_time > right_time ? left_time - right_time : right_time - left_time;
    if (skew > next.maximum_cross_bus_skew_ns) {
      return RefreshResult::failed;
    }
    if (require_new && all_equal) {
      runtime_now_ns_ = now;
      std::lock_guard<std::mutex> lock(mutex_);
      health_.runtime_now_ns = now;
      return RefreshResult::unchanged;
    }
    if (require_new && snapshot_.arm[0].feedback_seq != 0U &&
      (next.arm[0].feedback_seq <= snapshot_.arm[0].feedback_seq ||
      next.arm[1].feedback_seq <= snapshot_.arm[1].feedback_seq))
    {
      return RefreshResult::failed;
    }
    snapshot_ = next;
    runtime_now_ns_ = now;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      health_.snapshot = snapshot_;
      health_.runtime_now_ns = runtime_now_ns_;
    }
    return RefreshResult::updated;
  }

  bool publish_measured()
  {
    const auto callback_status = invoke_state_callback(MeasuredState{snapshot_, runtime_now_ns_});
    if (callback_status != OA_RUNTIME_OK) {
      fault_local(
        callback_status, callback_status == OA_RUNTIME_ETIMEOUT ?
        "state_callback_rejected" : "state_callback_failed");
      return false;
    }
    return publish_feedback();
  }

  void process_pending()
  {
    std::optional<SessionCommand> pending;
    bool canceled_before_submit = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!pending_) {
        return;
      }
      pending = std::move(pending_);
      pending_.reset();
      canceled_before_submit = cancelled_reservation_owner_ == pending->owner;
      if (canceled_before_submit) {
        terminalizing_owner_ = pending->owner;
      }
    }
    if (canceled_before_submit) {
      CommandResult result = make_result(OA_RUNTIME_OK);
      result.outcome = CommandResult::Outcome::canceled;
      copy_terminal_snapshot(result);
      result.event = OA_RUNTIME_EVENT_STOPPED;
      result.reason = "canceled_reserved_disable_stop";
      const bool terminal_ok = invoke_terminal_callback(*pending, result);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (health_.owner == pending->owner) {
          health_.owner.clear();
        }
        cancelled_reservation_owner_.clear();
        terminalizing_owner_.clear();
        health_.terminal_feedback_seq[0] = result.terminal_feedback_seq[0];
        health_.terminal_feedback_seq[1] = result.terminal_feedback_seq[1];
        if (terminal_ok) {
          health_.reason = result.reason;
          notify_health_unlocked();
        }
      }
      if (!terminal_ok) {
        fault_local(OA_RUNTIME_EFAULT, "terminal_callback_failed");
      }
      return;
    }

    oa_runtime_plan * plan = nullptr;
    oa_runtime_status status = OA_RUNTIME_EINVAL;
    for (unsigned int attempt = 0U; attempt < 3U; ++attempt) {
      if (refresh_snapshot(false, true) == RefreshResult::failed) {
        reject_on_owner(std::move(*pending), OA_RUNTIME_ESTALE, "stale_runtime_snapshot");
        return;
      }
      std::uint64_t now = 0U;
      status = oa_runtime_now_monotonic_ns(runtime_, OA_RUNTIME_CLOCK_MONOTONIC, &now);
      std::uint64_t plan_expiry = 0U;
      if (status != OA_RUNTIME_OK || !checked_add(now, kPlanHorizonNs, plan_expiry)) {
        reject_on_owner(std::move(*pending), status == OA_RUNTIME_OK ? OA_RUNTIME_EINVAL : status,
          "plan_deadline_failed");
        return;
      }
      if (pending->kind == SessionCommand::Kind::joint) {
        oa_runtime_joint_move move{};
        init(move);
        move.clock_id = OA_RUNTIME_CLOCK_MONOTONIC;
        move.units_id = OA_RUNTIME_UNITS_SI_V1;
        move.expiry_runtime_monotonic_ns = plan_expiry;
        move.required_feedback_seq = snapshot_.arm[pending->side].feedback_seq;
        move.side = pending->side;
        move.joint = pending->joint;
        move.target_model_rad = pending->target_rad;
        move.velocity_scale = 0.5;
        move.acceleration_scale = 0.5;
        move.jerk_scale = 0.5;
        move.position_tolerance_rad = 5.0e-4;
        move.velocity_tolerance_rad_s = 2.0e-2;
        move.required_model_revision = health_.capabilities.model_revision;
        move.required_tcp_revision = health_.model_identity[pending->side].tcp_revision;
        move.collision_scene_revision = kCollisionSceneRevision;
        move.required_collision_policy = OA_RUNTIME_COLLISION_VIRTUAL_UNCHECKED;
        std::snprintf(move.required_coordinate_identity_sha256,
          sizeof(move.required_coordinate_identity_sha256), "%s",
          health_.capabilities.coordinate_identity_sha256);
        status = oa_runtime_plan_joint(runtime_, &move, &plan);
      } else {
        oa_runtime_paired_tcp_move move{};
        init(move);
        move.clock_id = OA_RUNTIME_CLOCK_MONOTONIC;
        move.units_id = OA_RUNTIME_UNITS_SI_V1;
        move.frame_id = OA_RUNTIME_FRAME_OPENARM_BODY_LINK0;
        move.orientation_policy = OA_RUNTIME_ORIENTATION_FREE;
        move.expiry_runtime_monotonic_ns = plan_expiry;
        move.required_feedback_seq[0] = snapshot_.arm[0].feedback_seq;
        move.required_feedback_seq[1] = snapshot_.arm[1].feedback_seq;
        std::copy(pending->left_tcp_m.begin(), pending->left_tcp_m.end(), move.left_tcp_m);
        std::copy(pending->right_tcp_m.begin(), pending->right_tcp_m.end(), move.right_tcp_m);
        move.velocity_scale = 0.5;
        move.acceleration_scale = 0.5;
        move.jerk_scale = 0.5;
        move.tcp_tolerance_m = 1.0e-3;
        move.collision_scene_revision = kCollisionSceneRevision;
        move.required_model_revision = health_.capabilities.model_revision;
        move.required_tcp_revision[0] = health_.model_identity[0].tcp_revision;
        move.required_tcp_revision[1] = health_.model_identity[1].tcp_revision;
        move.required_collision_policy = OA_RUNTIME_COLLISION_VIRTUAL_UNCHECKED;
        std::snprintf(move.required_coordinate_identity_sha256,
          sizeof(move.required_coordinate_identity_sha256), "%s",
          health_.capabilities.coordinate_identity_sha256);
        move.maximum_branch_step_rad = 2.0;
        move.minimum_singular_value = 0.0;
        status = oa_runtime_plan_paired_tcp_body(runtime_, &move, &plan);
      }
      if (status != OA_RUNTIME_ESTALE) {
        break;
      }
    }
    if (status != OA_RUNTIME_OK || plan == nullptr) {
      reject_on_owner(std::move(*pending), status, "planning_failed");
      return;
    }

    oa_runtime_plan_report report{};
    init(report);
    status = oa_runtime_plan_get_report(plan, &report);
    const bool report_valid = status == OA_RUNTIME_OK && report.duration_ns <= kMaximumDurationNs &&
      report.manifest_revision == health_.manifest.manifest_revision &&
      report.model_revision == health_.manifest.model_revision &&
      report.tcp_revision[0] == health_.model_identity[0].tcp_revision &&
      report.tcp_revision[1] == health_.model_identity[1].tcp_revision &&
      report.collision_scene_revision == kCollisionSceneRevision &&
      report.collision_policy == OA_RUNTIME_COLLISION_VIRTUAL_UNCHECKED &&
      report.frame_id == OA_RUNTIME_FRAME_OPENARM_BODY_LINK0 &&
      report.units_id == OA_RUNTIME_UNITS_SI_V1 &&
      std::strcmp(report.coordinate_identity_sha256,
      health_.capabilities.coordinate_identity_sha256) == 0;
    if (!report_valid) {
      oa_runtime_plan_destroy(plan);
      if (status == OA_RUNTIME_OK && report.duration_ns > kMaximumDurationNs) {
        reject_on_owner(std::move(*pending), OA_RUNTIME_ETIMEOUT, "plan_duration_rejected");
      } else if (status != OA_RUNTIME_OK) {
        reject_on_owner(std::move(*pending), status, "plan_report_failed");
      } else {
        fault_local(OA_RUNTIME_EIDENTITY, "runtime_plan_identity_mismatch");
      }
      return;
    }
    std::uint64_t now = 0U;
    status = oa_runtime_now_monotonic_ns(runtime_, OA_RUNTIME_CLOCK_MONOTONIC, &now);
    std::uint64_t expiry = 0U;
    std::uint64_t producer_deadline = 0U;
    if (status != OA_RUNTIME_OK || !checked_add(now, report.duration_ns, expiry) ||
      !checked_add(expiry, kExecutionMarginNs, expiry) ||
      !checked_add(now, kHeartbeatAheadNs, producer_deadline))
    {
      oa_runtime_plan_destroy(plan);
      reject_on_owner(std::move(*pending), status == OA_RUNTIME_OK ? OA_RUNTIME_EINVAL : status,
        "execution_deadline_failed");
      return;
    }
    oa_runtime_execute_request execute{};
    init(execute);
    execute.clock_id = OA_RUNTIME_CLOCK_MONOTONIC;
    execute.start_runtime_monotonic_ns = 0U;
    execute.expiry_runtime_monotonic_ns = expiry;
    execute.producer_deadline_runtime_monotonic_ns = producer_deadline;
    execute.stop_kind = OA_RUNTIME_STOP_DISABLE;
    std::uint64_t command_id = 0U;
    status = oa_runtime_execute(runtime_, plan, &execute, &command_id);
    oa_runtime_plan_destroy(plan);
    if (status != OA_RUNTIME_OK) {
      reject_on_owner(std::move(*pending), status, "execute_failed");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      Active active;
      active.command = std::move(*pending);
      active.report = report;
      active.command_id = command_id;
      for (std::size_t side = 0U; side < OA_RUNTIME_ARMS; ++side) {
        std::copy_n(snapshot_.arm[side].q_model_rad, OA_RUNTIME_DOF, active.start_q[side]);
      }
      active_ = std::move(active);
      health_.adapter_state = AdapterState::executing;
      health_.command_id = command_id;
      health_.plan_seed_feedback_seq[0] = report.seed_feedback_seq[0];
      health_.plan_seed_feedback_seq[1] = report.seed_feedback_seq[1];
      health_.plan_duration_ns = report.duration_ns;
      health_.reason = "executing";
      notify_health_unlocked();
    }
    drain_events();
  }

  void reject_on_owner(SessionCommand command, const oa_runtime_status status, const std::string & reason)
  {
    CommandResult result = make_result(status);
    result.outcome = CommandResult::Outcome::rejected;
    result.seed_feedback_seq[0] = snapshot_.arm[0].feedback_seq;
    result.seed_feedback_seq[1] = snapshot_.arm[1].feedback_seq;
    copy_terminal_snapshot(result);
    result.reason = reason;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      terminalizing_owner_ = command.owner;
    }
    const bool terminal_ok = invoke_terminal_callback(command, result);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (health_.owner == command.owner) {
        health_.owner.clear();
      }
      health_.terminal_feedback_seq[0] = result.terminal_feedback_seq[0];
      health_.terminal_feedback_seq[1] = result.terminal_feedback_seq[1];
      terminalizing_owner_.clear();
      if (terminal_ok) {
        health_.adapter_state = AdapterState::idle;
        health_.reason = reason;
        notify_health_unlocked();
      }
    }
    if (!terminal_ok) {
      fault_local(OA_RUNTIME_EFAULT, "terminal_callback_failed");
    }
  }

  bool heartbeat()
  {
    std::uint64_t command_id = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_) {
        return true;
      }
      command_id = active_->command_id;
    }
    std::uint64_t now = 0U;
    oa_runtime_status status = oa_runtime_now_monotonic_ns(
      runtime_, OA_RUNTIME_CLOCK_MONOTONIC, &now);
    std::uint64_t deadline = 0U;
    if (status != OA_RUNTIME_OK || !checked_add(now, kHeartbeatAheadNs, deadline)) {
      fault_from_status(status == OA_RUNTIME_OK ? OA_RUNTIME_EINVAL : status,
        "heartbeat_deadline_failed");
      return false;
    }
    status = oa_runtime_heartbeat(runtime_, command_id, OA_RUNTIME_CLOCK_MONOTONIC, deadline);
    if (status != OA_RUNTIME_OK) {
      fault_from_status(status, "heartbeat_failed");
      return false;
    }
    return true;
  }

  void drain_events()
  {
    for (;;) {
      oa_runtime_event event{};
      init(event);
      const oa_runtime_status status = oa_runtime_poll_event(runtime_, 0U, &event);
      if (status == OA_RUNTIME_ETIMEOUT) {
        return;
      }
      if (status != OA_RUNTIME_OK) {
        fault_from_status(status, "runtime_event_poll_failed");
        return;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        health_.last_event = event.kind;
        init(health_.last_error);
        health_.last_error.status = event.status;
        health_.last_error.facility = event.source_facility;
        health_.last_error.lower_code = event.source_status;
        notify_health_unlocked();
      }
      if (event.kind == OA_RUNTIME_EVENT_COMPLETED) {
        complete_active(event);
        if (adapter_faulted()) {
          return;
        }
      } else if (event.kind == OA_RUNTIME_EVENT_FAULTED || event.kind == OA_RUNTIME_EVENT_ESTOP) {
        fault_with_detail(event.status, event.source_facility, event.source_status, 0U,
          "runtime_fault_event");
        return;
      }
    }
  }

  void complete_active(const oa_runtime_event & event)
  {
    CommandResult result;
    SessionCommand command;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || event.command_id != active_->command_id) {
        return;
      }
      command = active_->command;
      result = make_result_unlocked(OA_RUNTIME_OK);
      result.outcome = CommandResult::Outcome::completed;
      result.command_id = event.command_id;
      result.seed_feedback_seq[0] = active_->report.seed_feedback_seq[0];
      result.seed_feedback_seq[1] = active_->report.seed_feedback_seq[1];
      result.plan_duration_ns = active_->report.duration_ns;
      result.terminal_feedback_seq[0] = snapshot_.arm[0].feedback_seq;
      result.terminal_feedback_seq[1] = snapshot_.arm[1].feedback_seq;
      result.lifecycle = event.lifecycle;
      result.event = event.kind;
      result.cause = event.source_status;
      result.collision_checked = active_->report.collision_checked != 0U;
      result.motion_authorized = active_->report.motion_authorized != 0U;
      result.reason = "completed_measured_feedback";
      terminalizing_owner_ = command.owner;
    }
    const bool terminal_ok = invoke_terminal_callback(command, result);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || active_->command_id != event.command_id) {
        return;
      }
      active_.reset();
      terminalizing_owner_.clear();
      health_.command_id = 0U;
      if (health_.owner == command.owner) {
        health_.owner.clear();
      }
      health_.terminal_feedback_seq[0] = result.terminal_feedback_seq[0];
      health_.terminal_feedback_seq[1] = result.terminal_feedback_seq[1];
      if (terminal_ok) {
        health_.adapter_state = AdapterState::idle;
        health_.reason = "completed";
      }
    }
    if (terminal_ok) {
      invoke_health_callback();
    } else {
      fault_local(OA_RUNTIME_EFAULT, "terminal_callback_failed");
    }
  }

  bool publish_feedback()
  {
    std::function<bool(const CommandFeedback &)> callback;
    CommandFeedback feedback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || !active_->command.feedback) {
        return true;
      }
      callback = active_->command.feedback;
      feedback.lifecycle = snapshot_.lifecycle;
      feedback.event = health_.last_event;
      feedback.command_id = active_->command_id;
      feedback.feedback_seq[0] = snapshot_.arm[0].feedback_seq;
      feedback.feedback_seq[1] = snapshot_.arm[1].feedback_seq;
      double start_squared = 0.0;
      double remaining_squared = 0.0;
      for (std::size_t side = 0U; side < OA_RUNTIME_ARMS; ++side) {
        for (std::size_t joint = 0U; joint < OA_RUNTIME_DOF; ++joint) {
          const double start_delta =
            active_->report.target_q_model_rad[side][joint] - active_->start_q[side][joint];
          const double remaining_delta = active_->report.target_q_model_rad[side][joint] -
            snapshot_.arm[side].q_model_rad[joint];
          start_squared += start_delta * start_delta;
          remaining_squared += remaining_delta * remaining_delta;
        }
      }
      feedback.measured_progress = start_squared <= 1.0e-18 ? 1.0 :
        std::clamp(1.0 - std::sqrt(remaining_squared / start_squared), 0.0, 1.0);
    }
    try {
      if (callback(feedback)) {
        return true;
      }
    } catch (...) {
    }
    fault_local(OA_RUNTIME_EFAULT, "feedback_callback_failed");
    return false;
  }

  void process_cancel()
  {
    std::string owner;
    std::optional<SessionCommand> command;
    std::optional<oa_runtime_plan_report> report;
    std::uint64_t command_id = 0U;
    bool active_command = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (cancel_owner_.empty()) {
        return;
      }
      owner = std::move(cancel_owner_);
      cancel_owner_.clear();
      if (health_.owner != owner) {
        return;
      }
      if (active_ && active_->command.owner == owner) {
        command = active_->command;
        report = active_->report;
        command_id = active_->command_id;
        active_command = true;
      } else if (pending_ && pending_->owner == owner) {
        command = *pending_;
      }
      if (command) {
        terminalizing_owner_ = owner;
      }
    }

    oa_runtime_status stop_status = OA_RUNTIME_OK;
    if (snapshot_.lifecycle == kLifecycleArmedIdle || snapshot_.lifecycle == kLifecycleExecuting) {
      stop_status = oa_runtime_stop(runtime_, OA_RUNTIME_STOP_DISABLE);
      if (stop_status != OA_RUNTIME_OK) {
        remember_error(stop_status);
      }
      drain_events();
      if (adapter_faulted()) {
        return;
      }
    }
    if (!refresh_stopped_snapshot()) {
      stop_status = OA_RUNTIME_EFAULT;
    }

    if (command) {
      CommandResult result = make_result(stop_status);
      result.outcome = CommandResult::Outcome::canceled;
      result.command_id = command_id;
      if (report) {
        result.seed_feedback_seq[0] = report->seed_feedback_seq[0];
        result.seed_feedback_seq[1] = report->seed_feedback_seq[1];
        result.plan_duration_ns = report->duration_ns;
        result.collision_checked = report->collision_checked != 0U;
        result.motion_authorized = report->motion_authorized != 0U;
      }
      copy_terminal_snapshot(result);
      result.event = active_command ? OA_RUNTIME_EVENT_ABORTED : OA_RUNTIME_EVENT_STOPPED;
      result.reason = stop_status == OA_RUNTIME_OK ?
        "canceled_disable_stop" : "cancel_disable_stop_failed";
      const bool terminal_ok = invoke_terminal_callback(*command, result);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_ && active_->command.owner == owner) {
          active_.reset();
        }
        if (pending_ && pending_->owner == owner) {
          pending_.reset();
        }
        terminalizing_owner_.clear();
        if (health_.owner == owner) {
          health_.owner.clear();
        }
        health_.command_id = 0U;
        health_.terminal_feedback_seq[0] = result.terminal_feedback_seq[0];
        health_.terminal_feedback_seq[1] = result.terminal_feedback_seq[1];
        if (terminal_ok) {
          health_.adapter_state = AdapterState::stopped_requires_restart;
          health_.reason = result.reason;
          notify_health_unlocked();
        }
      }
      if (!terminal_ok) {
        fault_local(OA_RUNTIME_EFAULT, "terminal_callback_failed");
      }
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      health_.command_id = 0U;
      health_.adapter_state = AdapterState::stopped_requires_restart;
      health_.reason = stop_status == OA_RUNTIME_OK ?
        "canceled_reserved_disable_stop" : "cancel_disable_stop_failed";
      cancelled_reservation_owner_ = owner;
      notify_health_unlocked();
    }
  }

  bool refresh_stopped_snapshot()
  {
    return refresh_snapshot(false, false) != RefreshResult::failed &&
           snapshot_.lifecycle == kLifecycleDisarmed;
  }

  bool adapter_faulted() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return health_.adapter_state == AdapterState::fault;
  }

  void remember_error(const oa_runtime_status status)
  {
    oa_runtime_error_detail detail{};
    init(detail);
    if (runtime_ == nullptr || oa_runtime_get_last_error(runtime_, &detail) != OA_RUNTIME_OK ||
      detail.status != status)
    {
      init(detail);
      detail.status = status;
      detail.facility = OA_RUNTIME_FACILITY_RUNTIME;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    health_.last_error = detail;
  }

  CommandResult make_result(const oa_runtime_status status)
  {
    if (status != OA_RUNTIME_OK) {
      remember_error(status);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return make_result_unlocked(status);
  }

  CommandResult make_result_unlocked(const oa_runtime_status status) const
  {
    CommandResult result;
    result.runtime_status = status;
    if (status == OA_RUNTIME_OK) {
      result.runtime_facility = OA_RUNTIME_FACILITY_RUNTIME;
      result.control_status = 0U;
    } else {
      result.runtime_facility = health_.last_error.facility;
      result.lower_status = health_.last_error.lower_code;
      result.system_error = health_.last_error.system_error;
      result.control_status = result.runtime_facility == OA_RUNTIME_FACILITY_CONTROL ?
        result.lower_status : 0U;
    }
    return result;
  }

  void copy_terminal_snapshot(CommandResult & result) const
  {
    result.terminal_feedback_seq[0] = snapshot_.arm[0].feedback_seq;
    result.terminal_feedback_seq[1] = snapshot_.arm[1].feedback_seq;
    result.lifecycle = snapshot_.lifecycle;
  }

  void fault_from_status(const oa_runtime_status status, const std::string & reason)
  {
    remember_error(status);
    const SessionHealth current = health();
    fault_with_detail(status, current.last_error.facility, current.last_error.lower_code,
      current.last_error.system_error, reason);
  }

  void fault_local(const oa_runtime_status status, const std::string & reason)
  {
    fault_with_detail(status, OA_RUNTIME_FACILITY_RUNTIME, 0U, 0U, reason);
  }

  void fault_with_detail(
    const oa_runtime_status status, const oa_runtime_facility facility,
    const std::uint32_t lower, const std::uint32_t system_error, const std::string & reason)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (health_.adapter_state == AdapterState::fault) {
        return;
      }
    }
    if (runtime_ != nullptr) {
      (void)oa_runtime_stop(runtime_, OA_RUNTIME_STOP_DISABLE);
      (void)refresh_snapshot(false, false);
    }
    std::optional<Active> active;
    std::optional<SessionCommand> pending;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active = std::move(active_);
      active_.reset();
      pending = std::move(pending_);
      pending_.reset();
      if (active) {
        terminalizing_owner_ = active->command.owner;
      } else if (pending) {
        terminalizing_owner_ = pending->owner;
      }
      health_.adapter_state = AdapterState::fault;
      init(health_.last_error);
      health_.last_error.status = status;
      health_.last_error.facility = facility;
      health_.last_error.lower_code = lower;
      health_.last_error.system_error = system_error;
      health_.reason = reason;
      health_.command_id = 0U;
      notify_health_unlocked();
    }
    SessionCommand * command = active ? &active->command : (pending ? &*pending : nullptr);
    if (command != nullptr && command->terminal) {
      CommandResult result;
      result.outcome = CommandResult::Outcome::aborted;
      result.runtime_status = status;
      result.runtime_facility = facility;
      result.lower_status = lower;
      result.system_error = system_error;
      result.control_status = facility == OA_RUNTIME_FACILITY_CONTROL ? lower : 0U;
      result.command_id = active ? active->command_id : 0U;
      if (active) {
        result.seed_feedback_seq[0] = active->report.seed_feedback_seq[0];
        result.seed_feedback_seq[1] = active->report.seed_feedback_seq[1];
        result.plan_duration_ns = active->report.duration_ns;
        result.collision_checked = active->report.collision_checked != 0U;
        result.motion_authorized = active->report.motion_authorized != 0U;
      }
      copy_terminal_snapshot(result);
      result.event = OA_RUNTIME_EVENT_FAULTED;
      result.cause = facility == OA_RUNTIME_FACILITY_CONTROL ? lower : 0U;
      result.reason = reason;
      const bool terminal_ok = invoke_terminal_callback(*command, result);
      std::lock_guard<std::mutex> lock(mutex_);
      health_.terminal_feedback_seq[0] = result.terminal_feedback_seq[0];
      health_.terminal_feedback_seq[1] = result.terminal_feedback_seq[1];
      if (!terminal_ok) {
        health_.reason = "terminal_callback_failed_after_fault";
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    health_.owner.clear();
    terminalizing_owner_.clear();
    notify_health_unlocked();
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
      if (active) {
        terminalizing_owner_ = active->command.owner;
      } else if (pending) {
        terminalizing_owner_ = pending->owner;
      }
    }
    oa_runtime_status stop_status = OA_RUNTIME_OK;
    if (snapshot_.lifecycle == kLifecycleExecuting || snapshot_.lifecycle == kLifecycleArmedIdle) {
      stop_status = oa_runtime_stop(runtime_, OA_RUNTIME_STOP_DISABLE);
      if (stop_status != OA_RUNTIME_OK) {
        remember_error(stop_status);
      }
      drain_events();
      if (!refresh_stopped_snapshot()) {
        stop_status = OA_RUNTIME_EFAULT;
      }
    }
    const SessionHealth terminal_health = health();
    const bool runtime_faulted = terminal_health.adapter_state == AdapterState::fault;
    SessionCommand * command = active ? &active->command : (pending ? &*pending : nullptr);
    bool terminal_ok = true;
    if (command != nullptr) {
      CommandResult result = make_result(runtime_faulted ? terminal_health.last_error.status :
        (stop_status == OA_RUNTIME_OK ? OA_RUNTIME_ESTATE : stop_status));
      result.outcome = CommandResult::Outcome::aborted;
      result.command_id = active ? active->command_id : 0U;
      if (active) {
        result.seed_feedback_seq[0] = active->report.seed_feedback_seq[0];
        result.seed_feedback_seq[1] = active->report.seed_feedback_seq[1];
        result.plan_duration_ns = active->report.duration_ns;
        result.collision_checked = active->report.collision_checked != 0U;
        result.motion_authorized = active->report.motion_authorized != 0U;
      }
      copy_terminal_snapshot(result);
      result.event = runtime_faulted ? OA_RUNTIME_EVENT_FAULTED : OA_RUNTIME_EVENT_ABORTED;
      result.cause = runtime_faulted && result.runtime_facility == OA_RUNTIME_FACILITY_CONTROL ?
        result.lower_status : 0U;
      result.reason = runtime_faulted ? terminal_health.reason :
        (stop_status == OA_RUNTIME_OK ? "shutdown_disable_stop" :
        "shutdown_disable_stop_failed");
      terminal_ok = invoke_terminal_callback(*command, result);
      std::lock_guard<std::mutex> lock(mutex_);
      health_.terminal_feedback_seq[0] = result.terminal_feedback_seq[0];
      health_.terminal_feedback_seq[1] = result.terminal_feedback_seq[1];
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      health_.owner.clear();
      health_.command_id = 0U;
      terminalizing_owner_.clear();
    }
    if (!terminal_ok && !runtime_faulted) {
      fault_local(OA_RUNTIME_EFAULT, "terminal_callback_failed");
    }
  }

  void destroy_handles() noexcept
  {
    oa_runtime_destroy(runtime_);
    runtime_ = nullptr;
    oa_runtime_manifest_destroy(manifest_);
    manifest_ = nullptr;
  }

  void notify_health_unlocked()
  {
    cv_.notify_all();
    invoke_health_callback();
  }

  oa_runtime_status invoke_state_callback(const MeasuredState & state) noexcept
  {
    try {
      if (!state_callback_) {
        return OA_RUNTIME_EFAULT;
      }
      return state_callback_(state) ? OA_RUNTIME_OK : OA_RUNTIME_ETIMEOUT;
    } catch (...) {
      return OA_RUNTIME_EFAULT;
    }
  }

  void invoke_health_callback() noexcept
  {
    try {
      if (health_callback_) {
        health_callback_();
      }
    } catch (...) {
    }
  }

  static bool invoke_terminal_callback(
    const SessionCommand & command, const CommandResult & result) noexcept
  {
    try {
      return !command.terminal || command.terminal(result);
    } catch (...) {
      return false;
    }
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
  std::string cancelled_reservation_owner_;
  std::string terminalizing_owner_;
  std::optional<SessionCommand> pending_;
  std::optional<Active> active_;
  SessionHealth health_;
  oa_runtime_manifest * manifest_{};
  oa_runtime * runtime_{};
  oa_runtime_snapshot snapshot_{};
  std::uint64_t runtime_now_ns_{};
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
  static const std::array<std::string, 14> names = []() {
      std::array<std::string, 14> value{};
      const auto & description = standard_manifest();
      for (std::size_t index = 0U; index < value.size(); ++index) {
        value[index] = description.motor[index].joint_name;
      }
      return value;
    }();
  return names;
}

bool VirtualControlSession::map_joint(
  const std::string & name, std::uint32_t & side, std::uint32_t & joint)
{
  const auto & names = joint_names();
  const auto found = std::find(names.begin(), names.end(), name);
  if (found == names.end()) {
    return false;
  }
  const auto index = static_cast<std::size_t>(std::distance(names.begin(), found));
  side = index < OA_RUNTIME_DOF ? kLeftSide : kRightSide;
  joint = static_cast<std::uint32_t>(index % OA_RUNTIME_DOF);
  return true;
}

bool VirtualControlSession::joint_target_in_limits(
  const std::uint32_t side, const std::uint32_t joint, const double target)
{
  if (side >= OA_RUNTIME_ARMS || joint >= OA_RUNTIME_DOF || !std::isfinite(target)) {
    return false;
  }
  const auto & motor = standard_manifest().motor[side * OA_RUNTIME_DOF + joint];
  return target >= motor.lower_rad && target <= motor.upper_rad;
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
