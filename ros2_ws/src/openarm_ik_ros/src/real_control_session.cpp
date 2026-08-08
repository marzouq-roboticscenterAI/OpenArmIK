// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/real_control_session.hpp"

#include "openarm_ik_ros/display_calibration.hpp"
#include "openarm_ik_ros/gripper_calibration.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

extern "C" {
#define OPENARM_DISABLE_LEGACY_GENERIC_STATUS 1
#include "openarm_can.h"
#include "openarm_collision.h"
#include "openarm_model.h"
#include "openarm_route.h"
#include "openarm_runtime_motion.h"
}

namespace openarm_ik_ros::real
{
namespace
{
using Clock = std::chrono::steady_clock;
using JointArray = std::array<double, OA_DOF>;
using PairQ = std::array<JointArray, 2>;

constexpr auto kCycle = std::chrono::milliseconds(20);
constexpr auto kFeedbackDeadline = std::chrono::milliseconds(18);
constexpr auto kFeedbackWatchdog = std::chrono::milliseconds(100);
constexpr auto kConnectSampleDeadline = std::chrono::milliseconds(50);
constexpr auto kConnectRampDeadline = std::chrono::milliseconds(20);
constexpr std::size_t kPhysicalMotorsPerArm = 8U;
constexpr std::uint32_t kArmMask = 0x7fU;
// Commissioning phase: keep every physical trajectory at the slowest portal
// setting until the complete real-arm demo suite has been accepted by the
// operator.  This is enforced here as well as in the browser so direct ROS
// action clients cannot bypass it.
constexpr double kPhysicalMotionLimitScale = 0.5;
constexpr double kEncoderLimitToleranceRad = 0.002;
constexpr double kPositionToleranceRad = 0.015;
constexpr double kVelocityToleranceRadS = 0.08;
// Preserve the official inner Damiao PD gains while removing their static
// loaded-pose error with a slow encoder-closed integral torque. It is enabled
// only after the smooth nominal segment ends, changes by at most 0.02 Nm per
// 20 ms cycle, remains far inside each joint's verified effort limit, and is
// disabled for intentional-contact commands.
constexpr double kEndpointIntegralGainNmPerRadS = 20.0;
constexpr double kEndpointTorqueStepNm = 0.02;
constexpr std::array<double, OA_DOF> kEndpointTorqueMaximumNm{
  6.0, 6.0, 4.0, 4.0, 1.5, 1.5, 1.5};
constexpr unsigned kSettleCycles = 8U;
constexpr unsigned kContactPersistenceCycles = 5U;
constexpr double kDefaultContactTorqueFraction = 0.10;
constexpr double kClearanceWorseningEpsilon = 1.0e-6;
constexpr double kNeutralJointValidationStepRad = 0.02;
constexpr std::array<double, OA_DOF> kKp{70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0};
constexpr std::array<double, OA_DOF> kKd{2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5};
constexpr double kGripperHoldTorqueNm = 0.25;
constexpr double kGripperDefaultSpeedMS = 0.0044;
constexpr double kGripperMaximumSpeedMS = 0.011;
constexpr double kGripperMinimumTorqueNm = 0.05;
constexpr double kGripperMaximumTorqueNm = 1.5;
constexpr double kGripperPositionToleranceM = 0.00075;
constexpr double kGripperVelocityToleranceMS = 0.002;
constexpr auto kMaximumSettleDuration = std::chrono::seconds(8);
constexpr std::uint32_t kGripperControlModePositionForce = 4U;
// DaMiao register 9 counts 50 us ticks. Keep this volatile (never flash-save
// it): 4000 ticks = 200 ms, ten normal 50 Hz command periods and twice the
// host's complete-feedback watchdog. A drive that becomes unreachable while
// enabled therefore releases itself even when a host disable frame cannot
// reach it.
constexpr std::uint32_t kMotorCanTimeoutTicks = 4000U;
constexpr auto kDisableConfirmationWindow = std::chrono::seconds(3);
constexpr auto kDisableConfirmationPoll = std::chrono::milliseconds(20);

std::uint64_t monotonic_ns() noexcept
{
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now().time_since_epoch()).count());
}

template<typename T>
void init_runtime(T & value)
{
  value = {};
  value.struct_size = static_cast<std::uint32_t>(sizeof(value));
  value.abi_version = OA_RUNTIME_ABI_VERSION;
}

template<typename T>
void init_can(T & value)
{
  value = {};
  value.struct_size = static_cast<std::uint32_t>(sizeof(value));
  value.abi_version = OA_CAN_ABI_VERSION;
}

const oa_model * model_for_side(const std::size_t side)
{
  return side == 0U ? oa_model_left_v10_bimanual() : oa_model_right_v10_bimanual();
}

oa_can_mit_profile profile_for(const std::size_t joint)
{
  oa_can_mit_profile profile;
  init_can(profile);
  profile.target_send_id = static_cast<std::uint16_t>(joint + 1U);
  profile.receive_id = static_cast<std::uint16_t>(joint + 0x11U);
  profile.pmax_rad = 12.5;
  if (joint < 2U) {
    profile.vmax_rad_s = 45.0;
    profile.tmax_nm = 54.0;
  } else if (joint < 4U) {
    profile.vmax_rad_s = 10.0;
    profile.tmax_nm = 28.0;
  } else {
    profile.vmax_rad_s = 30.0;
    profile.tmax_nm = 10.0;
  }
  profile.verified_mask = OA_CAN_PROFILE_ALL_VERIFIED;
  return profile;
}

double smoothstep7(const double input) noexcept
{
  const double x = std::clamp(input, 0.0, 1.0);
  return x * x * x * x * (35.0 + x * (-84.0 + x * (70.0 - 20.0 * x)));
}

double trajectory_duration_s(const PairQ & start, const PairQ & target, const double scale)
{
  double duration = 0.1;
  for (std::size_t side = 0; side < 2U; ++side) {
    for (std::size_t joint = 0; joint < OA_DOF; ++joint) {
      const double distance = std::abs(target[side][joint] - start[side][joint]);
      duration = std::max(duration, distance * 2.2 / (1.0 * scale));
      duration = std::max(duration, std::sqrt(distance * 8.0 / (2.0 * scale)));
      duration = std::max(duration, std::cbrt(distance * 60.0 / (10.0 * scale)));
    }
  }
  return duration;
}

bool finite_q(const PairQ & q)
{
  for (const auto & side : q) {
    for (const double value : side) {
      if (!std::isfinite(value)) {return false;}
    }
  }
  return true;
}

bool q_in_limits(const PairQ & q)
{
  if (!finite_q(q)) {return false;}
  for (std::size_t side = 0; side < 2U; ++side) {
    for (std::size_t joint = 0; joint < OA_DOF; ++joint) {
      double lower = 0.0;
      double upper = 0.0;
      if (oa_model_limits(model_for_side(side), joint, &lower, &upper) != OA_MODEL_OK ||
        q[side][joint] < lower - kEncoderLimitToleranceRad ||
        q[side][joint] > upper + kEncoderLimitToleranceRad)
      {
        return false;
      }
    }
  }
  return true;
}

bool solve_position(
  const std::size_t side, const double target[3], const JointArray & seed,
  JointArray & output)
{
  oa_ik_options options{};
  options.abi_version = OA_MODEL_ABI_VERSION;
  options.struct_size = static_cast<std::uint32_t>(sizeof(options));
  std::copy(seed.begin(), seed.end(), options.seed);
  std::copy(seed.begin(), seed.end(), options.posture);
  std::fill(std::begin(options.posture_weight), std::end(options.posture_weight), 1.0);
  options.position_tolerance_m = 1.0e-6;
  options.max_joint_step_rad = 0.12;
  options.damping_min = 1.0e-5;
  options.damping_max = 0.1;
  options.limit_margin_rad = 1.0e-5;
  options.max_iterations = 500U;
  oa_ik_diagnostics diagnostics{};
  return oa_ik_position_v2(
    model_for_side(side), target, &options, OA_IK_DIAGNOSTICS_VERSION,
    static_cast<std::uint32_t>(sizeof(diagnostics)), &diagnostics) == OA_MODEL_OK &&
    diagnostics.position_error_m <= 1.0e-5 &&
    (std::copy(std::begin(diagnostics.q), std::end(diagnostics.q), output.begin()), true);
}

struct CollisionState
{
  bool evaluated{false};
  bool clear{false};
  double minimum{-std::numeric_limits<double>::infinity()};
  bool terminal_pair{false};
  bool claw_contact{false};
  std::uint32_t violation{OA_COLLISION_VIOLATION_NONFINITE};
};

CollisionState collision_state(
  const PairQ & q, const double threshold, const oa_collision_contact_policy policy)
{
  CollisionState result;
  oa_fk_result fk[2]{};
  if (oa_fk(model_for_side(0U), q[0].data(), &fk[0]) != OA_MODEL_OK ||
    oa_fk(model_for_side(1U), q[1].data(), &fk[1]) != OA_MODEL_OK)
  {
    return result;
  }
  oa_collision_report report{};
  oa_collision_contact_evidence evidence{};
  const auto status = oa_collision_evaluate_scoped_fk_with_threshold(
    &fk[0], &fk[1], threshold, policy, &report, &evidence);
  result.evaluated = status == OA_MODEL_OK;
  result.clear = result.evaluated && report.clear != 0U;
  result.minimum = report.minimum_clearance_m;
  result.terminal_pair = evidence.terminal_pair_active != 0U;
  result.claw_contact = evidence.claw_contact_active != 0U;
  result.violation = report.violation;
  return result;
}

std::string json_escape(const std::string & value)
{
  std::string out;
  for (const char character : value) {
    if (character == '"' || character == '\\') {out.push_back('\\');}
    if (character == '\n') {out += "\\n";} else {out.push_back(character);}
  }
  return out;
}

}  // namespace

class RealControlSession::Impl
{
public:
  Impl(RealControlConfig config, StateCallback state_callback, HealthCallback health_callback)
  : config_(std::move(config)), state_callback_(std::move(state_callback)),
    health_callback_(std::move(health_callback))
  {
    if (config_.active_side_mask < 1U || config_.active_side_mask > 3U) {
      throw std::invalid_argument("active_side_mask must select left, right, or both");
    }
    if (config_.calibration_path.empty()) {
      const char * home = std::getenv("HOME");
      config_.calibration_path = std::string(home == nullptr ? "/tmp" : home) +
        "/.openarm_real_zero";
    }
    if (config_.gripper_calibration_path.empty()) {
      const char * home = std::getenv("HOME");
      config_.gripper_calibration_path = std::string(home == nullptr ? "/tmp" : home) +
        "/.openarm_real_gripper";
    }
    std::string detail;
    calibration_loaded_ = calibration_.load(config_.calibration_path, detail);
    calibration_detail_ = detail;
    gripper_calibration_loaded_ = gripper_calibration_.load(
      config_.gripper_calibration_path, detail);
    gripper_calibration_detail_ = detail;
    init_health();
    worker_ = std::thread([this]() {worker_main();});
  }

  ~Impl() {close();}

  bool connect_and_enable(std::string & detail)
  {
    if (estop_.load(std::memory_order_acquire) || oa_runtime_estop_asserted() != 0U) {
      detail = "E-stop is latched; release it before connecting";
      return false;
    }
    if (!calibration_loaded_) {
      detail = "saved neutral calibration is required before physical control: " +
        calibration_detail_;
      return false;
    }
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    if (connected_.load(std::memory_order_acquire)) {
      detail = armed_.load(std::memory_order_acquire) ?
        "already connected and motor control is enabled" : "already connected";
      return armed_.load(std::memory_order_acquire);
    }
    if (!open_sockets(detail)) {return false;}
    connected_.store(true, std::memory_order_release);
    const auto fail_connect_closed = [this](std::string & failure) noexcept {
        disable_all_immediate();
        const bool confirmed = disable_until_confirmed();
        close_sockets();
        connected_.store(false, std::memory_order_release);
        if (!confirmed) {
          failure += "; one or more disabled states were not confirmed; use the hardware stop";
        }
        set_health(AdapterState::fault, failure, kLifecycleFault);
        return false;
      };

    PairQ measured{};
    bool sampled = false;
    for (unsigned attempt = 0U; attempt < 5U && !sampled; ++attempt) {
      sampled = exchange_frames(false, 0.0, kConnectSampleDeadline, measured);
    }
    if (!sampled || !q_in_limits(measured)) {
      detail = sampled ?
        "fresh encoders do not map inside the commissioned URDF joint limits" :
        "did not receive fresh feedback from every motor on each active arm";
      return fail_connect_closed(detail);
    }
    const CollisionState initial = collision_state(
      measured, oa_collision_intervention_clearance_m(), OA_COLLISION_CONTACT_NONE);
    if (!initial.evaluated) {
      detail = "the measured neutral pose could not be evaluated by the collision model";
      return fail_connect_closed(detail);
    }

    for (std::size_t side = 0U; side < 2U; ++side) {
      if (!side_active(side)) {continue;}
      for (std::size_t motor = 0U; motor < kPhysicalMotorsPerArm; ++motor) {
        if (!configure_motor_timeout(side, motor, detail)) {
          return fail_connect_closed(detail);
        }
      }
      if (!configure_gripper_position_force(side, detail)) {
        return fail_connect_closed(detail);
      }
    }

    measured_q_ = measured;
    target_raw_ = raw_from_model(measured);
    feedforward_model_tau_ = {};
    gripper_target_raw_ = measured_raw_gripper_;
    gripper_exact_raw_hold_ = {true, true};
    for (std::size_t side = 0U; side < 2U; ++side) {
      if (!side_active(side)) {continue;}
      if (gripper_calibration_loaded_) {
        gripper_target_opening_m_[side] = gripper_calibration_.opening_m(
          side, measured_raw_gripper_[side]);
      }
    }
    // Seed zero-gain MIT references while disabled, then enable. This avoids
    // the stock driver's return-to-zero snap.
    if (!send_hold_commands(0.0) || !send_enable_all()) {
      detail = "failed while seeding measured targets or enabling motors";
      return fail_connect_closed(detail);
    }
    armed_.store(true, std::memory_order_release);
    for (unsigned step = 1U; step <= 50U; ++step) {
      if (estop_.load(std::memory_order_acquire) || oa_runtime_estop_asserted() != 0U) {
        detail = "E-stop asserted during gain ramp";
        armed_.store(false, std::memory_order_release);
        return fail_connect_closed(detail);
      }
      PairQ updated{};
      const double gain = static_cast<double>(step) / 50.0;
      bool complete = false;
      for (unsigned attempt = 0U; attempt < 3U && !complete; ++attempt) {
        complete = exchange_frames(true, gain, kConnectRampDeadline, updated);
      }
      if (!complete) {
        detail = "measured-pose gain ramp failed: " + last_exchange_failure_;
        armed_.store(false, std::memory_order_release);
        return fail_connect_closed(detail);
      }
      measured_q_ = updated;
      publish_measured(updated, kLifecycleArmedIdle);
      std::this_thread::sleep_for(kCycle);
    }
    const std::string scope = config_.active_side_mask == 3U ? "all 16 motors" :
      (config_.active_side_mask == 1U ? "left IDs 1..8 only" : "right IDs 1..8 only");
    set_health(AdapterState::idle, "connected; " + scope + " enabled at measured pose",
      kLifecycleArmedIdle);
    detail = "connected to " + scope + "; encoder-seeded hold is active";
    cv_.notify_all();
    return true;
  }

  bool disconnect_and_disable(std::string & detail) noexcept
  {
    const bool had_sockets = sockets_[0].load(std::memory_order_acquire) >= 0 ||
      sockets_[1].load(std::memory_order_acquire) >= 0;
    cancel_requested_.store(true, std::memory_order_release);
    armed_.store(false, std::memory_order_release);
    disable_all_immediate();
    bool confirmed = false;
    {
      std::lock_guard<std::mutex> io_lock(io_mutex_);
      confirmed = had_sockets && disable_until_confirmed();
      close_sockets();
    }
    connected_.store(false, std::memory_order_release);
    finish_active(CommandResult::Outcome::aborted, OA_RUNTIME_EVENT_DISARMED,
      "operator_disconnect_disable");
    const std::string result = !had_sockets ?
      "controller was already passive with no open CAN sockets" :
      (confirmed ?
      "disconnected; every active motor confirmed disabled and sockets closed" :
      "disconnected after repeated disable, but not all motors confirmed disabled; "
      "use the hardware stop");
    set_health(!had_sockets || confirmed ? AdapterState::starting : AdapterState::fault, result,
      !had_sockets || confirmed ? kLifecycleDisarmed : kLifecycleFault);
    detail = result;
    return !had_sockets || confirmed;
  }

  bool stop_motion(std::string & detail) noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_ && !active_) {
      detail = "no physical motion is active";
      return true;
    }
    cancel_requested_.store(true, std::memory_order_release);
    detail = "physical motion stop requested; measured-pose hold remains active";
    cv_.notify_all();
    return true;
  }

  bool emergency_stop(std::string & detail) noexcept
  {
    const bool had_sockets = sockets_[0].load(std::memory_order_acquire) >= 0 ||
      sockets_[1].load(std::memory_order_acquire) >= 0;
    estop_.store(true, std::memory_order_release);
    oa_runtime_estop_assert();
    cancel_requested_.store(true, std::memory_order_release);
    armed_.store(false, std::memory_order_release);
    // This write intentionally does not wait for io_mutex_: the latch wins
    // immediately, and SocketCAN frame writes are atomic.
    for (unsigned repetition = 0U; repetition < 3U; ++repetition) {
      disable_all_immediate();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    bool confirmed = false;
    {
      std::lock_guard<std::mutex> io_lock(io_mutex_);
      confirmed = had_sockets && disable_until_confirmed();
      close_sockets();
    }
    connected_.store(false, std::memory_order_release);
    finish_active(CommandResult::Outcome::aborted, OA_RUNTIME_EVENT_ESTOP,
      "software_estop_disable");
    detail = !had_sockets ?
      "E-stop latched; controller was already passive with no open CAN sockets" :
      (confirmed ?
      "E-stop latched; all 16 motors confirmed disabled and sockets closed" :
      "E-stop latched, but not all motors confirmed disabled; use the hardware stop");
    set_health(AdapterState::stopped_requires_restart, detail, kLifecycleEstop);
    // The latch itself always wins, but callers use this return value for the
    // portal's physical_disabled field. Never turn a transmitted disable into
    // a false confirmation when one or more drives did not report status 0.
    return !had_sockets || confirmed;
  }

  bool clear_emergency_stop(std::string & detail) noexcept
  {
    if (connected_.load(std::memory_order_acquire) || armed_.load(std::memory_order_acquire)) {
      detail = "refusing to clear E-stop while connected";
      return false;
    }
    if (oa_runtime_estop_clear() != OA_RUNTIME_OK) {
      detail = "runtime E-stop latch could not be cleared";
      return false;
    }
    estop_.store(false, std::memory_order_release);
    cancel_requested_.store(false, std::memory_order_release);
    set_health(AdapterState::starting,
      "E-stop released; motors remain disabled until Connect is pressed", kLifecycleDisarmed);
    detail = "E-stop released; motors remain disabled and require an explicit Connect";
    return true;
  }

  bool capture_grippers_closed(std::string & detail)
  {
    if (config_.active_side_mask != 3U) {
      detail = "paired gripper calibration is unavailable in single-arm mode";
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closing_ || pending_ || active_ || !owner_.empty()) {
        detail = "gripper calibration requires an idle controller";
        return false;
      }
    }
    if (!connected_.load(std::memory_order_acquire) ||
      !armed_.load(std::memory_order_acquire))
    {
      detail = "connect first so both J8 encoders are fresh";
      return false;
    }
    std::array<double, 2> closed{};
    {
      std::lock_guard<std::mutex> io_lock(io_mutex_);
      const auto time_now = Clock::now();
      for (std::size_t side = 0U; side < closed.size(); ++side) {
        if (!last_sample_[side][7U].valid ||
          time_now - last_feedback_[side][7U] > kFeedbackWatchdog)
        {
          detail = "both J8 encoders must be fresh before capturing closed";
          return false;
        }
        closed[side] = last_sample_[side][7U].raw_q;
      }
    }
    GripperCalibration captured;
    if (!captured.capture_closed(closed) ||
      !captured.save(config_.gripper_calibration_path, detail))
    {
      return false;
    }
    {
      // The worker reads calibration while holding either mutex_ (feedback
      // interpretation/completion) or io_mutex_ (CAN command encoding).
      // Publish a newly persisted calibration only while holding both so a
      // 100 Hz cycle can never observe a partially replaced pair.
      std::scoped_lock lock(mutex_, io_mutex_);
      gripper_calibration_ = captured;
      gripper_calibration_loaded_ = true;
      gripper_calibration_detail_ = "captured both physically closed J8 encoders";
    }
    detail = "captured both closed J8 positions; official -1.0472 rad travel maps to 44 mm open";
    return true;
  }

  bool reserve(const std::string & owner, std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closing_) {reason = "closing"; return false;}
    if (estop_.load(std::memory_order_acquire)) {reason = "estop_latched"; return false;}
    if (!connected_.load(std::memory_order_acquire) || !armed_.load(std::memory_order_acquire)) {
      reason = "physical_motors_not_connected";
      return false;
    }
    if (!owner_.empty() || pending_.has_value() || active_.has_value()) {
      reason = "busy";
      return false;
    }
    owner_ = owner;
    health_.owner = owner;
    health_.adapter_state = AdapterState::reserved;
    health_.reason = "reserved";
    notify_health();
    return true;
  }

  bool submit(SessionCommand command, std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closing_ || owner_ != command.owner || pending_ || active_) {
      reason = closing_ ? "closing" : "reservation_mismatch";
      return false;
    }
    if (command.kind != SessionCommand::Kind::joint &&
      command.kind != SessionCommand::Kind::gripper &&
      !valid_motion_limit_scale(command.motion_limit_scale))
    {
      reason = "invalid_motion_limit_scale";
      return false;
    }
    if (!command_allowed_for_active_sides(command, reason)) {return false;}
    if (command.kind != SessionCommand::Kind::gripper) {
      command.motion_limit_scale = std::min(
        command.kind == SessionCommand::Kind::joint ? kLegacyMotionLimitScale :
        command.motion_limit_scale,
        kPhysicalMotionLimitScale);
    }
    cancel_requested_.store(false, std::memory_order_release);
    pending_ = std::move(command);
    cv_.notify_all();
    return true;
  }

  bool cancel(const std::string & owner)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (owner_ != owner || (!pending_ && !active_)) {return false;}
    cancel_requested_.store(true, std::memory_order_release);
    cv_.notify_all();
    return true;
  }

  void release(const std::string & owner, const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (owner_ == owner && !pending_ && !active_) {
      owner_.clear();
      health_.owner.clear();
      health_.adapter_state = armed_.load() ? AdapterState::idle : AdapterState::starting;
      health_.reason = reason;
      notify_health();
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
      if (closing_) {return;}
      closing_ = true;
      cancel_requested_.store(true, std::memory_order_release);
      cv_.notify_all();
    }
    armed_.store(false, std::memory_order_release);
    disable_all_immediate();
    if (worker_.joinable() && worker_.get_id() != std::this_thread::get_id()) {
      worker_.join();
    }
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    (void)disable_until_confirmed();
    close_sockets();
    connected_.store(false, std::memory_order_release);
  }

  bool connected() const noexcept {return connected_.load(std::memory_order_acquire);}
  bool armed() const noexcept {return armed_.load(std::memory_order_acquire);}
  bool estop_asserted() const noexcept {return estop_.load(std::memory_order_acquire);}

  bool busy() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_.has_value() || active_.has_value() || !owner_.empty();
  }

  std::string status_json() const
  {
    const SessionHealth current = health();
    std::ostringstream out;
    out << "{\"connected\":" << (connected() ? "true" : "false")
        << ",\"resolved\":true,\"armed\":" << (armed() ? "true" : "false")
        << ",\"estop\":" << (estop_asserted() ? "true" : "false")
        << ",\"busy\":" << (busy() ? "true" : "false")
        << ",\"active_side_mask\":" << config_.active_side_mask
        << ",\"control\":\"physical_encoder_feedback\""
        << ",\"confidence\":\"high\",\"detail\":\""
        << json_escape(current.reason) << "\",\"buses\":[";
    bool separator = false;
    for (std::size_t side = 0U; side < 2U; ++side) {
      if (!side_active(side)) {continue;}
      if (separator) {out << ',';}
      separator = true;
      out << "{\"name\":\"" << json_escape(config_.interface_for_side[side])
          << "\",\"side\":\"" << (side == 0U ? "left" : "right")
          << "\",\"motors\":8}";
    }
    out << "]}";
    return out.str();
  }

private:
  bool side_active(const std::size_t side) const noexcept
  {
    return side < 2U && (config_.active_side_mask & (1U << side)) != 0U;
  }

  bool command_allowed_for_active_sides(
    const SessionCommand & command, std::string & reason) const
  {
    if (config_.active_side_mask == 3U) {return true;}
    const std::size_t active_side = config_.active_side_mask == 1U ? 0U : 1U;
    const int inactive_side = active_side == 0U ? 1 : 0;
    if (command.kind == SessionCommand::Kind::joint && command.side == active_side) {
      return true;
    }
    if (command.kind == SessionCommand::Kind::gripper &&
      command.gripper_side_mask == (1U << active_side))
    {
      return true;
    }
    if (command.kind == SessionCommand::Kind::paired_tcp &&
      command.preserved_side == inactive_side)
    {
      return true;
    }
    reason = "command_involves_inactive_arm";
    return false;
  }

  struct MotorSample
  {
    bool valid{false};
    double raw_q{0.0};
    double raw_dq{0.0};
    double raw_tau{0.0};
    std::uint8_t status{0U};
    std::uint8_t mos{0U};
    std::uint8_t rotor{0U};
  };

  struct Active
  {
    SessionCommand command;
    std::vector<PairQ> waypoints;
    std::size_t waypoint{0U};
    PairQ segment_start{};
    PairQ segment_target{};
    PairQ endpoint_integral_tau{};
    Clock::time_point segment_started{};
    double segment_duration_s{0.1};
    std::uint64_t command_id{0U};
    std::uint64_t seed_seq[2]{};
    std::uint64_t total_duration_ns{0U};
    unsigned settled{0U};
    unsigned contact_streak{0U};
    double completed_duration_s{0.0};
    double previous_clearance{-std::numeric_limits<double>::infinity()};
    bool contact_policy{false};
    bool gripper_motion{false};
    std::array<double, 2> gripper_start_opening_m{};
    std::array<double, 2> gripper_target_opening_m{};
    std::uint32_t gripper_side_mask{};
  };

  void init_health()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    health_ = {};
    health_.adapter_state = AdapterState::starting;
    health_.reason = calibration_loaded_ ?
      "physical controller passive; press Connect to enable" :
      "physical controller passive; neutral calibration is missing";
    init_runtime(health_.snapshot);
    for (auto & arm : health_.snapshot.arm) {init_runtime(arm);}
    health_.snapshot.clock_id = OA_RUNTIME_CLOCK_MONOTONIC;
    health_.snapshot.units_id = OA_RUNTIME_UNITS_SI_V1;
    health_.snapshot.frame_id = OA_RUNTIME_FRAME_OPENARM_BODY_LINK0;
    health_.snapshot.lifecycle = kLifecycleDisarmed;
    health_.snapshot.maximum_cross_bus_skew_ns = 10000000ULL;
    health_.last_error = {};
    init_runtime(health_.last_error);
    health_.last_error.status = OA_RUNTIME_OK;
  }

  void notify_health()
  {
    health_notification_.store(true, std::memory_order_release);
  }

  void dispatch_health()
  {
    if (!health_notification_.exchange(false, std::memory_order_acq_rel)) {return;}
    try {if (health_callback_) {health_callback_();}} catch (...) {}
  }

  void set_health(const AdapterState state, const std::string & reason, const std::uint32_t lifecycle)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      health_.adapter_state = state;
      health_.reason = reason;
      health_.snapshot.lifecycle = lifecycle;
      if (state != AdapterState::executing) {health_.command_id = 0U;}
      notify_health();
    }
    dispatch_health();
  }

  bool open_sockets(std::string & detail)
  {
    for (std::size_t side = 0; side < 2U; ++side) {
      if (!side_active(side)) {continue;}
      const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
      if (fd < 0) {
        detail = "cannot create SocketCAN socket: " + std::string(std::strerror(errno));
        close_sockets();
        return false;
      }
      ifreq request{};
      std::snprintf(request.ifr_name, IFNAMSIZ, "%s", config_.interface_for_side[side].c_str());
      if (::ioctl(fd, SIOCGIFINDEX, &request) < 0) {
        detail = "CAN interface " + config_.interface_for_side[side] + " is unavailable";
        ::close(fd);
        close_sockets();
        return false;
      }
      std::array<can_filter, kPhysicalMotorsPerArm> filters{};
      for (std::size_t joint = 0; joint < filters.size(); ++joint) {
        filters[joint].can_id = static_cast<canid_t>(0x11U + joint);
        filters[joint].can_mask = CAN_SFF_MASK;
      }
      if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
        static_cast<socklen_t>(sizeof(filters))) < 0)
      {
        detail = "cannot install exact motor feedback filters on " +
          config_.interface_for_side[side];
        ::close(fd);
        close_sockets();
        return false;
      }
      sockaddr_can address{};
      address.can_family = AF_CAN;
      address.can_ifindex = request.ifr_ifindex;
      if (::bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
        detail = "cannot bind " + config_.interface_for_side[side] + ": " +
          std::strerror(errno);
        ::close(fd);
        close_sockets();
        return false;
      }
      sockets_[side].store(fd, std::memory_order_release);
    }
    return true;
  }

  void close_sockets() noexcept
  {
    for (auto & slot : sockets_) {
      const int fd = slot.exchange(-1, std::memory_order_acq_rel);
      if (fd >= 0) {(void)::close(fd);}
    }
  }

  static bool write_frame(const int fd, const oa_can_frame & source) noexcept
  {
    if (fd < 0) {return false;}
    can_frame frame{};
    frame.can_id = source.can_id;
    frame.can_dlc = source.dlc;
    std::copy(std::begin(source.data), std::end(source.data), std::begin(frame.data));
    return ::write(fd, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
  }

  bool write_and_confirm_u32_register(
    const std::size_t side, const std::size_t motor, const oa_can_register_id register_id,
    const std::uint32_t requested_value, const std::string & label, std::string & detail)
  {
    const int fd = sockets_[side].load(std::memory_order_acquire);
    oa_can_register_write write;
    init_can(write);
    write.send_id = static_cast<std::uint16_t>(motor + 1U);
    write.register_id = register_id;
    write.value_type = OA_CAN_REGISTER_U32;
    write.value_u32 = requested_value;
    oa_can_frame outgoing;
    init_can(outgoing);
    if (oa_can_make_register_write(&write, &outgoing) != OA_CAN_OK ||
      !write_frame(fd, outgoing))
    {
      detail = "could not request " + label + " for " +
        config_.interface_for_side[side] + " ID " + std::to_string(motor + 1U);
      return false;
    }

    oa_can_register_request expected;
    init_can(expected);
    expected.send_id = static_cast<std::uint16_t>(motor + 1U);
    expected.receive_id = static_cast<std::uint16_t>(0x11U + motor);
    expected.register_id = register_id;
    expected.value_type = OA_CAN_REGISTER_U32;
    const auto deadline = Clock::now() + std::chrono::milliseconds(250);
    while (Clock::now() < deadline) {
      pollfd descriptor{fd, POLLIN, 0};
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - Clock::now());
      const int ready = ::poll(&descriptor, 1, static_cast<int>(std::max<std::int64_t>(
          1, remaining.count())));
      if (ready < 0 && errno == EINTR) {continue;}
      if (ready <= 0) {break;}
      can_frame incoming{};
      if (::read(fd, &incoming, sizeof(incoming)) != static_cast<ssize_t>(sizeof(incoming))) {
        continue;
      }
      oa_can_frame response;
      init_can(response);
      response.can_id = incoming.can_id & CAN_SFF_MASK;
      response.dlc = incoming.can_dlc;
      std::copy(std::begin(incoming.data), std::end(incoming.data), std::begin(response.data));
      oa_can_register_value value;
      init_can(value);
      if (oa_can_decode_register_response(
          &response, &expected, OA_CAN_REGISTER_WRITE, &value) == OA_CAN_OK &&
        value.value_u32 == requested_value)
      {
        return true;
      }
    }
    detail = config_.interface_for_side[side] + " ID " + std::to_string(motor + 1U) +
      " did not confirm " + label;
    return false;
  }

  bool configure_motor_timeout(
    const std::size_t side, const std::size_t motor, std::string & detail)
  {
    return write_and_confirm_u32_register(
      side, motor, OA_CAN_RID_TIMEOUT, kMotorCanTimeoutTicks,
      "volatile 200 ms communications timeout", detail);
  }

  bool configure_gripper_position_force(const std::size_t side, std::string & detail)
  {
    return write_and_confirm_u32_register(
      side, 7U, OA_CAN_RID_CTRL_MODE, kGripperControlModePositionForce,
      "J8 position-force mode", detail);
  }

  void disable_all_immediate() noexcept
  {
    for (std::size_t side = 0; side < 2U; ++side) {
      const int fd = sockets_[side].load(std::memory_order_acquire);
      if (fd < 0) {continue;}
      for (std::size_t motor = 0; motor < kPhysicalMotorsPerArm; ++motor) {
        oa_can_frame frame;
        init_can(frame);
        if (oa_can_make_disable(static_cast<std::uint16_t>(motor + 1U), &frame) == OA_CAN_OK) {
          (void)write_frame(fd, frame);
        }
      }
    }
  }

  bool disable_until_confirmed() noexcept
  {
    if (sockets_[0].load(std::memory_order_acquire) < 0 &&
      sockets_[1].load(std::memory_order_acquire) < 0)
    {
      return true;
    }
    const auto deadline = Clock::now() + kDisableConfirmationWindow;
    do {
      disable_all_immediate();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      if (!send_refresh_all()) {continue;}
      std::array<std::array<MotorSample, kPhysicalMotorsPerArm>, 2> sample{};
      std::array<std::uint32_t, 2> mask{};
      (void)collect_feedback(kDisableConfirmationPoll, sample, mask);
      bool all_disabled = true;
      for (std::size_t side = 0U; side < 2U; ++side) {
        if (!side_active(side)) {continue;}
        all_disabled = all_disabled && mask[side] == 0xffU;
        for (std::size_t motor = 0U; motor < kPhysicalMotorsPerArm; ++motor) {
          all_disabled = all_disabled && sample[side][motor].valid &&
            sample[side][motor].status == OA_CAN_FEEDBACK_DISABLED;
        }
      }
      if (all_disabled) {return true;}
    } while (Clock::now() < deadline);
    return false;
  }

  bool send_enable_all()
  {
    for (std::size_t side = 0; side < 2U; ++side) {
      if (!side_active(side)) {continue;}
      const int fd = sockets_[side].load(std::memory_order_acquire);
      for (std::size_t motor = 0; motor < kPhysicalMotorsPerArm; ++motor) {
        oa_can_frame frame;
        init_can(frame);
        if (oa_can_make_enable(static_cast<std::uint16_t>(motor + 1U), &frame) != OA_CAN_OK ||
          !write_frame(fd, frame)) {return false;}
      }
    }
    return true;
  }

  bool send_refresh_all()
  {
    for (std::size_t side = 0; side < 2U; ++side) {
      if (!side_active(side)) {continue;}
      const int fd = sockets_[side].load(std::memory_order_acquire);
      for (std::size_t motor = 0; motor < kPhysicalMotorsPerArm; ++motor) {
        oa_can_frame frame;
        init_can(frame);
        if (oa_can_make_refresh_status(static_cast<std::uint16_t>(motor + 1U), &frame) != OA_CAN_OK ||
          !write_frame(fd, frame)) {return false;}
      }
    }
    return true;
  }

  bool send_hold_commands(const double gain_scale)
  {
    for (std::size_t side = 0; side < 2U; ++side) {
      if (!side_active(side)) {continue;}
      const int fd = sockets_[side].load(std::memory_order_acquire);
      for (std::size_t joint = 0; joint < OA_DOF; ++joint) {
        oa_can_mit_profile_command command;
        init_can(command);
        command.send_id = static_cast<std::uint16_t>(joint + 1U);
        command.position_rad = target_raw_[side][joint];
        command.velocity_rad_s = 0.0;
        command.kp = gain_scale * kKp[joint];
        command.kd = gain_scale * kKd[joint];
        command.torque_nm = calibration_.signed_value(
          side, joint, feedforward_model_tau_[side][joint]);
        const auto profile = profile_for(joint);
        oa_can_frame frame;
        init_can(frame);
        if (oa_can_encode_mit_profile(&command, &profile, &frame) != OA_CAN_OK ||
          !write_frame(fd, frame)) {return false;}
      }

      const auto profile = profile_for(7U);
      oa_can_frame frame;
      init_can(frame);
      oa_can_status encoded = OA_CAN_ESTATE;
      if (!gripper_exact_raw_hold_[side] && gripper_calibration_loaded_) {
        oa_can_gripper_command command;
        init_can(command);
        command.send_id = 8U;
        command.target_opening_m = gripper_target_opening_m_[side];
        command.maximum_opening_speed_m_s = gripper_speed_m_s_[side];
        command.maximum_motor_torque_nm = gripper_torque_limit_nm_[side];
        encoded = oa_can_encode_gripper_move(
          &command, &gripper_calibration_.side(side), &profile, &frame);
      } else {
        oa_can_pos_force_command command;
        init_can(command);
        command.send_id = 8U;
        command.position_rad = gripper_target_raw_[side];
        command.max_velocity_rad_s = kGripperDefaultSpeedMS *
          std::abs(GripperCalibration::kOfficialMotorTravelRad) /
          GripperCalibration::kMaximumOpeningM;
        command.current_limit_per_unit = kGripperHoldTorqueNm / profile.tmax_nm;
        encoded = oa_can_encode_pos_force(&command, &profile, &frame);
      }
      if (encoded != OA_CAN_OK || !write_frame(fd, frame)) {return false;}
    }
    return true;
  }

  bool collect_feedback(
    const std::chrono::milliseconds deadline,
    std::array<std::array<MotorSample, kPhysicalMotorsPerArm>, 2> & sample,
    std::array<std::uint32_t, 2> & mask)
  {
    mask = {};
    const auto end = Clock::now() + deadline;
    const auto complete = [this, &mask]() {
        return (!side_active(0U) || mask[0] == 0xffU) &&
               (!side_active(1U) || mask[1] == 0xffU);
      };
    while (!complete() && Clock::now() < end) {
      pollfd descriptors[2]{{sockets_[0].load(), POLLIN, 0}, {sockets_[1].load(), POLLIN, 0}};
      const auto remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(
        end - Clock::now());
      const auto timeout_count = std::max<std::int64_t>(
        0, (remaining.count() + 999999LL) / 1000000LL);
      const int timeout = static_cast<int>(std::min<std::int64_t>(timeout_count, 1000));
      const int ready = ::poll(descriptors, 2, timeout);
      if (ready < 0 && errno == EINTR) {continue;}
      if (ready <= 0) {break;}
      for (std::size_t side = 0; side < 2U; ++side) {
        if (!side_active(side)) {continue;}
        if ((descriptors[side].revents & POLLIN) == 0) {continue;}
        can_frame incoming{};
        if (::read(descriptors[side].fd, &incoming, sizeof(incoming)) !=
          static_cast<ssize_t>(sizeof(incoming))) {continue;}
        const std::uint32_t id = incoming.can_id & CAN_SFF_MASK;
        if ((incoming.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0U ||
          incoming.can_dlc != 8U || id < 0x11U || id > 0x18U) {continue;}
        const std::size_t joint = id - 0x11U;
        oa_can_frame frame;
        init_can(frame);
        frame.can_id = id;
        frame.dlc = incoming.can_dlc;
        std::copy(std::begin(incoming.data), std::end(incoming.data), std::begin(frame.data));
        oa_can_feedback feedback;
        init_can(feedback);
        const auto profile = profile_for(joint);
        const oa_can_status decoded = oa_can_decode_feedback_profile(
          &frame, static_cast<std::uint16_t>(id), static_cast<std::uint8_t>(joint + 1U),
          &profile, &feedback);
        // Fault replies still contain authoritative motor identity, encoder,
        // temperature, and status. Preserve them so fail-safe shutdown can
        // distinguish disabled from LOST_COMM instead of treating both as an
        // absent frame.
        if (decoded != OA_CAN_OK && decoded != OA_CAN_EFAULT) {continue;}
        auto & motor = sample[side][joint];
        motor.valid = true;
        motor.raw_q = feedback.position_rad;
        motor.raw_dq = feedback.velocity_rad_s;
        motor.raw_tau = feedback.torque_nm;
        motor.status = feedback.status_nibble;
        motor.mos = feedback.mos_temperature_c;
        motor.rotor = feedback.rotor_temperature_c;
        mask[side] |= 1U << joint;
      }
    }
    return complete();
  }

  bool exchange_frames(
    const bool command, const double gain, const std::chrono::milliseconds deadline,
    PairQ & measured)
  {
    std::array<std::array<MotorSample, kPhysicalMotorsPerArm>, 2> sample{};
    std::array<std::uint32_t, 2> mask{};
    const bool sent = command ? send_hold_commands(gain) : send_refresh_all();
    if (!sent) {
      last_exchange_failure_ = "CAN command transmission failed";
      return false;
    }
    (void)collect_feedback(deadline, sample, mask);
    const auto now = Clock::now();
    for (std::size_t side = 0; side < 2U; ++side) {
      if (!side_active(side)) {continue;}
      for (std::size_t joint = 0; joint < kPhysicalMotorsPerArm; ++joint) {
        if ((mask[side] & (1U << joint)) != 0U) {
          last_sample_[side][joint] = sample[side][joint];
          last_feedback_[side][joint] = now;
        }
      }
    }

    // USB-CAN adapters may deliver adjacent motor replies in separate host
    // scheduler batches.  A reply is therefore accepted by its measured age,
    // not by requiring all 16 frames to land inside one 18 ms poll window.
    // No queued frame is discarded: each motor must independently remain
    // newer than the watchdog limit, which preserves fail-closed semantics
    // without false trips at a batch boundary.
    std::ostringstream failure;
    bool all_fresh = true;
    for (std::size_t side = 0; side < 2U; ++side) {
      if (!side_active(side)) {continue;}
      for (std::size_t joint = 0; joint < kPhysicalMotorsPerArm; ++joint) {
        const auto stamp = last_feedback_[side][joint];
        if (stamp.time_since_epoch().count() == 0 || now - stamp > kFeedbackWatchdog) {
          if (all_fresh) {failure << "stale encoder IDs";}
          failure << ' ' << config_.interface_for_side[side] << ':' << joint + 1U;
          all_fresh = false;
        }
      }
    }
    if (!all_fresh) {
      last_exchange_failure_ = failure.str();
      return false;
    }
    if (command) {
      for (std::size_t side = 0; side < last_sample_.size(); ++side) {
        if (!side_active(side)) {continue;}
        for (std::size_t joint = 0; joint < last_sample_[side].size(); ++joint) {
          if (last_sample_[side][joint].status != OA_CAN_FEEDBACK_ENABLED) {
            last_exchange_failure_ = "motor remained disabled or faulted: " +
              config_.interface_for_side[side] + " ID " + std::to_string(joint + 1U) +
              " status " + std::to_string(last_sample_[side][joint].status);
            return false;
          }
        }
      }
    }
    for (std::size_t side = 0; side < 2U; ++side) {
      if (!side_active(side)) {continue;}
      for (std::size_t joint = 0; joint < OA_DOF; ++joint) {
        measured[side][joint] = calibration_.position(
          side, joint, last_sample_[side][joint].raw_q);
      }
      measured_raw_gripper_[side] = last_sample_[side][7U].raw_q;
    }
    if (!finite_q(measured)) {
      last_exchange_failure_ = "non-finite calibrated encoder position";
      return false;
    }
    last_exchange_failure_.clear();
    return true;
  }

  bool append_validated_joint_edge(
    const PairQ & from, const PairQ & target, std::vector<PairQ> & waypoints,
    std::string & reason) const
  {
    double maximum_delta = 0.0;
    for (std::size_t side = 0; side < 2U; ++side) {
      for (std::size_t joint = 0; joint < OA_DOF; ++joint) {
        maximum_delta = std::max(maximum_delta,
          std::abs(target[side][joint] - from[side][joint]));
      }
    }
    if (maximum_delta <= 1.0e-12) {return true;}
    const std::size_t samples = std::max<std::size_t>(1U,
      static_cast<std::size_t>(std::ceil(maximum_delta / kNeutralJointValidationStepRad)));
    CollisionState previous = collision_state(
      from, oa_collision_required_clearance_m(), OA_COLLISION_CONTACT_NONE);
    bool recovering = previous.evaluated && !previous.clear;
    if (recovering) {
      previous = collision_state(
        from, oa_collision_intervention_clearance_m(), OA_COLLISION_CONTACT_NONE);
      if (!previous.clear) {
        reason = "joint_edge_start_inside_live_intervention_envelope";
        return false;
      }
    }
    for (std::size_t sample = 1U; sample <= samples; ++sample) {
      const double fraction = static_cast<double>(sample) / static_cast<double>(samples);
      PairQ q{};
      for (std::size_t side = 0; side < 2U; ++side) {
        for (std::size_t joint = 0; joint < OA_DOF; ++joint) {
          q[side][joint] = from[side][joint] +
            fraction * (target[side][joint] - from[side][joint]);
        }
      }
      CollisionState checked = collision_state(
        q, oa_collision_required_clearance_m(), OA_COLLISION_CONTACT_NONE);
      if (recovering && checked.evaluated && !checked.clear) {
        checked = collision_state(
          q, oa_collision_intervention_clearance_m(), OA_COLLISION_CONTACT_NONE);
        if (!checked.clear || checked.minimum < previous.minimum - kClearanceWorseningEpsilon) {
          reason = "joint_edge_failed_clearance_recovery";
          return false;
        }
      } else if (!checked.clear) {
        reason = "joint_edge_collision_gate";
        return false;
      } else {
        recovering = false;
      }
      previous = checked;
    }
    if (recovering) {
      reason = "joint_edge_did_not_recover_planning_clearance";
      return false;
    }
    waypoints.push_back(target);
    return true;
  }

  PairQ raw_from_model(const PairQ & q) const
  {
    PairQ raw{};
    for (std::size_t side = 0; side < 2U; ++side) {
      for (std::size_t joint = 0; joint < OA_DOF; ++joint) {
        raw[side][joint] = calibration_.raw_position(side, joint, q[side][joint]);
      }
    }
    return raw;
  }

  void publish_measured(const PairQ & q, const std::uint32_t lifecycle)
  {
    const std::uint64_t now = monotonic_ns();
    MeasuredState measured;
    init_runtime(measured.snapshot);
    measured.snapshot.clock_id = OA_RUNTIME_CLOCK_MONOTONIC;
    measured.snapshot.units_id = OA_RUNTIME_UNITS_SI_V1;
    measured.snapshot.frame_id = OA_RUNTIME_FRAME_OPENARM_BODY_LINK0;
    measured.snapshot.lifecycle = lifecycle;
    measured.snapshot.maximum_cross_bus_skew_ns = 10000000ULL;
    measured.runtime_now_ns = now;
    for (std::size_t side = 0; side < 2U; ++side) {
      auto & arm = measured.snapshot.arm[side];
      init_runtime(arm);
      arm.feedback_seq = side_active(side) ? feedback_sequence_[side].fetch_add(
        1U, std::memory_order_acq_rel) + 1U : 0U;
      arm.measurement_runtime_monotonic_ns = now;
      arm.expected_mask = side_active(side) ? kArmMask : 0U;
      arm.fresh_mask = side_active(side) ? kArmMask : 0U;
      arm.fault_mask = 0U;
      for (std::size_t joint = 0; joint < OA_DOF; ++joint) {
        arm.q_model_rad[joint] = q[side][joint];
        arm.dq_model_rad_s[joint] = calibration_.signed_value(
          side, joint, last_sample_[side][joint].raw_dq);
        arm.tau_model_nm[joint] = calibration_.signed_value(
          side, joint, last_sample_[side][joint].raw_tau);
        arm.q_output_rad[joint] = last_sample_[side][joint].raw_q;
        arm.dq_output_rad_s[joint] = last_sample_[side][joint].raw_dq;
        arm.tau_output_nm[joint] = last_sample_[side][joint].raw_tau;
        arm.status[joint] = last_sample_[side][joint].status;
        arm.mos_temperature_c[joint] = last_sample_[side][joint].mos;
        arm.coil_temperature_c[joint] = last_sample_[side][joint].rotor;
      }
      auto & gripper = measured.gripper[side];
      gripper.calibrated = gripper_calibration_loaded_ && side_active(side);
      gripper.motor_position_rad = last_sample_[side][7U].raw_q;
      gripper.motor_velocity_rad_s = last_sample_[side][7U].raw_dq;
      gripper.motor_torque_nm = last_sample_[side][7U].raw_tau;
      if (gripper_calibration_loaded_) {
        gripper.opening_m = gripper_calibration_.opening_m(side, gripper.motor_position_rad);
        gripper.velocity_m_s = gripper_calibration_.velocity_m_s(
          side, gripper.motor_velocity_rad_s);
      }
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      health_.snapshot = measured.snapshot;
      health_.runtime_now_ns = now;
    }
    try {if (state_callback_) {(void)state_callback_(measured);}} catch (...) {}
  }

  bool plan_command(const SessionCommand & command, const PairQ & start, Active & active,
    std::string & reason)
  {
    active.command = command;
    active.segment_start = start;
    active.seed_seq[0] = feedback_sequence_[0].load(std::memory_order_acquire);
    active.seed_seq[1] = feedback_sequence_[1].load(std::memory_order_acquire);
    active.command_id = next_command_id_++;
    active.contact_policy = command.kind == SessionCommand::Kind::converge_tcp;

    if (command.kind == SessionCommand::Kind::gripper) {
      if (!gripper_calibration_loaded_) {
        reason = "gripper_closed_calibration_required";
        return false;
      }
      if (command.gripper_side_mask == 0U || (command.gripper_side_mask & ~0x3U) != 0U ||
        !std::isfinite(command.gripper_opening_m) || command.gripper_opening_m < 0.0 ||
        command.gripper_opening_m > GripperCalibration::kMaximumOpeningM ||
        !std::isfinite(command.gripper_speed_m_s) || command.gripper_speed_m_s <= 0.0 ||
        command.gripper_speed_m_s > kGripperMaximumSpeedMS ||
        !std::isfinite(command.gripper_torque_limit_nm) ||
        command.gripper_torque_limit_nm < kGripperMinimumTorqueNm ||
        command.gripper_torque_limit_nm > kGripperMaximumTorqueNm)
      {
        reason = "invalid_gripper_command";
        return false;
      }
      active.gripper_motion = true;
      active.gripper_side_mask = command.gripper_side_mask;
      double maximum_delta = 0.0;
      for (std::size_t side = 0U; side < 2U; ++side) {
        const double opening = gripper_calibration_.opening_m(side, measured_raw_gripper_[side]);
        if (!std::isfinite(opening)) {
          reason = "invalid_gripper_encoder_mapping";
          return false;
        }
        active.gripper_start_opening_m[side] = opening;
        active.gripper_target_opening_m[side] = opening;
        if ((command.gripper_side_mask & (1U << side)) != 0U) {
          active.gripper_target_opening_m[side] = command.gripper_opening_m;
          maximum_delta = std::max(maximum_delta, std::abs(command.gripper_opening_m - opening));
        }
      }
      // The seventh-order profile's peak slope is about 2.2 times its average.
      // Scaling duration by 2.2 keeps the requested opening speed a hard cap.
      active.segment_duration_s = std::max(
        0.1, 2.2 * maximum_delta / command.gripper_speed_m_s);
      active.segment_started = Clock::now();
      active.total_duration_ns = static_cast<std::uint64_t>(std::ceil(
          active.segment_duration_s * 1.0e9));
      return true;
    }

    if (command.kind == SessionCommand::Kind::neutral) {
      PairQ target{};
      if (!collision_state(target, oa_collision_required_clearance_m(),
        OA_COLLISION_CONTACT_NONE).clear)
      {
        reason = "neutral_pose_collision_gate";
        return false;
      }
      std::string direct_reason;
      if (!append_validated_joint_edge(start, target, active.waypoints, direct_reason)) {
        // If the direct measured-to-zero edge is obstructed, move through the
        // dynamic paired Cartesian graph, then prove the final exact joint
        // edge.  The direct edge is preferred because position-only IK can
        // choose a different redundant wrist branch even when model zero is
        // only a few encoder counts away.
        active.waypoints.clear();
        oa_route_request request{};
        request.abi_version = OA_ROUTE_ABI_VERSION;
        request.struct_size = static_cast<std::uint32_t>(sizeof(request));
        request.flags = OA_ROUTE_ALLOW_CLEARANCE_RECOVERY;
        request.maximum_branch_step_rad = 0.35;
        for (std::size_t side = 0; side < 2U; ++side) {
          std::copy(start[side].begin(), start[side].end(), request.start_q_rad[side]);
          oa_fk_result fk{};
          if (oa_fk(model_for_side(side), target[side].data(), &fk) != OA_MODEL_OK) {
            reason = "neutral_fk_failed";
            return false;
          }
          request.target_tcp_m[side][0] = fk.hand_tcp.m[3];
          request.target_tcp_m[side][1] = fk.hand_tcp.m[7];
          request.target_tcp_m[side][2] = fk.hand_tcp.m[11];
        }
        oa_route_result route{};
        route.abi_version = OA_ROUTE_ABI_VERSION;
        route.struct_size = static_cast<std::uint32_t>(sizeof(route));
        const oa_route_status status = oa_route_plan_paired(&request, &route);
        if (status != OA_ROUTE_OK || route.waypoint_count == 0U) {
          reason = direct_reason + "; neutral_collision_aware_route_failed:" +
            std::to_string(status);
          return false;
        }
        for (std::size_t index = 0; index < route.waypoint_count; ++index) {
          PairQ q{};
          for (std::size_t side = 0; side < 2U; ++side) {
            std::copy(std::begin(route.waypoint_q_rad[index][side]),
              std::end(route.waypoint_q_rad[index][side]), q[side].begin());
          }
          active.waypoints.push_back(q);
        }
        const PairQ & neutral_edge_start = active.waypoints.back();
        if (!append_validated_joint_edge(
            neutral_edge_start, target, active.waypoints, reason))
        {
          return false;
        }
      }
    } else if (command.kind == SessionCommand::Kind::joint) {
      PairQ target = start;
      if (command.side >= 2U || command.joint >= OA_DOF ||
        !std::isfinite(command.target_rad)) {
        reason = "joint_target_out_of_limits";
        return false;
      }
      target[command.side][command.joint] = command.target_rad;
      if (!q_in_limits(target)) {
        reason = "joint_target_out_of_limits";
        return false;
      }
      if (!append_validated_joint_edge(start, target, active.waypoints, reason)) {return false;}
    } else {
      std::array<std::array<double, 3>, 2> target{command.left_tcp_m, command.right_tcp_m};
      if (command.kind == SessionCommand::Kind::centroid_tcp) {
        std::array<std::array<double, 3>, 2> tcp{};
        for (std::size_t side = 0; side < 2U; ++side) {
          oa_fk_result fk{};
          if (oa_fk(model_for_side(side), start[side].data(), &fk) != OA_MODEL_OK) {
            reason = "centroid_fk_failed"; return false;
          }
          tcp[side] = {fk.hand_tcp.m[3], fk.hand_tcp.m[7], fk.hand_tcp.m[11]};
        }
        for (std::size_t axis = 0; axis < 3U; ++axis) {
          const double delta = command.target_m[axis] - 0.5 * (tcp[0][axis] + tcp[1][axis]);
          target[0][axis] = tcp[0][axis] + delta;
          target[1][axis] = tcp[1][axis] + delta;
        }
      } else if (command.kind == SessionCommand::Kind::mirrored_tcp) {
        const std::size_t lead = command.side;
        if (lead >= 2U) {reason = "invalid_mirror_lead"; return false;}
        target[lead] = command.left_tcp_m;
        target[1U - lead] = {command.left_tcp_m[0], -command.left_tcp_m[1],
          command.left_tcp_m[2]};
      } else if (command.kind == SessionCommand::Kind::converge_tcp) {
        for (std::size_t side = 0; side < 2U; ++side) {
          oa_fk_result fk{};
          if (oa_fk(model_for_side(side), start[side].data(), &fk) != OA_MODEL_OK) {
            reason = "converge_fk_failed"; return false;
          }
          const std::array<double, 3> from{fk.hand_tcp.m[3], fk.hand_tcp.m[7], fk.hand_tcp.m[11]};
          double length = 0.0;
          for (std::size_t axis = 0; axis < 3U; ++axis) {
            const double d = command.target_m[axis] - from[axis]; length += d * d;
          }
          length = std::sqrt(length);
          if (!(length > command.stop_distance_m + command.minimum_progress_m)) {
            reason = "converge_progress_too_small"; return false;
          }
          const double fraction = (length - command.stop_distance_m) / length;
          for (std::size_t axis = 0; axis < 3U; ++axis) {
            target[side][axis] = from[axis] + fraction * (command.target_m[axis] - from[axis]);
          }
        }
      }

      if (command.preserved_side >= 0) {
        const std::size_t held = static_cast<std::size_t>(command.preserved_side);
        oa_fk_result fk{};
        if (held >= 2U || oa_fk(model_for_side(held), start[held].data(), &fk) != OA_MODEL_OK) {
          reason = "invalid_preserved_side";
          return false;
        }
        target[held] = {fk.hand_tcp.m[3], fk.hand_tcp.m[7], fk.hand_tcp.m[11]};
      }

      if (active.contact_policy) {
        PairQ seed = start;
        constexpr std::size_t samples = 32U;
        std::array<std::array<double, 3>, 2> start_tcp{};
        for (std::size_t side = 0; side < 2U; ++side) {
          oa_fk_result fk{};
          if (oa_fk(model_for_side(side), start[side].data(), &fk) != OA_MODEL_OK) {
            reason = "contact_start_fk_failed"; return false;
          }
          start_tcp[side] = {fk.hand_tcp.m[3], fk.hand_tcp.m[7], fk.hand_tcp.m[11]};
        }
        for (std::size_t sample = 1U; sample <= samples; ++sample) {
          PairQ next{};
          const double fraction = static_cast<double>(sample) / static_cast<double>(samples);
          for (std::size_t side = 0; side < 2U; ++side) {
            double point[3]{};
            for (std::size_t axis = 0; axis < 3U; ++axis) {
              point[axis] = start_tcp[side][axis] + fraction *
                (target[side][axis] - start_tcp[side][axis]);
            }
            if (!solve_position(side, point, seed[side], next[side])) {
              reason = "contact_path_ik_failed"; return false;
            }
          }
          const CollisionState checked = collision_state(
            next, oa_collision_required_clearance_m(), OA_COLLISION_CONTACT_TERMINAL_CAPS);
          if (!checked.clear) {reason = "contact_path_collision_gate"; return false;}
          seed = next;
        }
        active.waypoints.push_back(seed);
      } else {
        oa_route_request request{};
        request.abi_version = OA_ROUTE_ABI_VERSION;
        request.struct_size = static_cast<std::uint32_t>(sizeof(request));
        request.flags = OA_ROUTE_ALLOW_CLEARANCE_RECOVERY;
        if (command.preserved_side == 0) {request.flags |= OA_ROUTE_PRESERVE_LEFT;}
        else if (command.preserved_side == 1) {request.flags |= OA_ROUTE_PRESERVE_RIGHT;}
        else if (command.preserved_side != -1) {
          reason = "invalid_preserved_side";
          return false;
        }
        request.maximum_branch_step_rad = 0.35;
        for (std::size_t side = 0; side < 2U; ++side) {
          std::copy(start[side].begin(), start[side].end(), request.start_q_rad[side]);
          std::copy(target[side].begin(), target[side].end(), request.target_tcp_m[side]);
        }
        oa_route_result route{};
        route.abi_version = OA_ROUTE_ABI_VERSION;
        route.struct_size = static_cast<std::uint32_t>(sizeof(route));
        const oa_route_status status = oa_route_plan_paired(&request, &route);
        if (status != OA_ROUTE_OK || route.waypoint_count == 0U) {
          reason = "collision_aware_route_failed:" + std::to_string(status);
          return false;
        }
        for (std::size_t index = 0; index < route.waypoint_count; ++index) {
          PairQ q{};
          for (std::size_t side = 0; side < 2U; ++side) {
            std::copy(std::begin(route.waypoint_q_rad[index][side]),
              std::end(route.waypoint_q_rad[index][side]), q[side].begin());
          }
          active.waypoints.push_back(q);
        }
      }
    }
    if (active.waypoints.empty()) {reason = "empty_route"; return false;}
    active.segment_target = active.waypoints.front();
    active.segment_duration_s = trajectory_duration_s(
      active.segment_start, active.segment_target,
      command.kind == SessionCommand::Kind::joint ? kLegacyMotionLimitScale :
      command.motion_limit_scale);
    active.segment_started = Clock::now();
    for (std::size_t index = 0U; index < active.waypoints.size(); ++index) {
      const PairQ & from = index == 0U ? start : active.waypoints[index - 1U];
      active.total_duration_ns += static_cast<std::uint64_t>(std::ceil(
        trajectory_duration_s(from, active.waypoints[index], command.motion_limit_scale) * 1.0e9));
    }
    const CollisionState initial = collision_state(
      start, oa_collision_intervention_clearance_m(),
      active.contact_policy ? OA_COLLISION_CONTACT_TERMINAL_CAPS : OA_COLLISION_CONTACT_NONE);
    active.previous_clearance = initial.minimum;
    return true;
  }

  void process_pending()
  {
    std::optional<SessionCommand> pending;
    PairQ start{};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!pending_) {return;}
      pending = std::move(pending_);
      pending_.reset();
      start = measured_q_;
    }
    if (cancel_requested_.load(std::memory_order_acquire)) {
      terminal_command(*pending, CommandResult::Outcome::canceled, OA_RUNTIME_EVENT_STOPPED,
        "canceled_before_motion", 0U, 0U, 0U);
      return;
    }
    Active planned;
    std::string reason;
    if (!plan_command(*pending, start, planned, reason)) {
      terminal_command(*pending, CommandResult::Outcome::rejected, OA_RUNTIME_EVENT_ABORTED,
        reason, 0U, feedback_sequence_[0].load(std::memory_order_acquire),
        feedback_sequence_[1].load(std::memory_order_acquire));
      return;
    }
    // Carry the existing idle gravity compensation into a new trajectory and
    // let update_reference() ramp it down. Dropping it in one control cycle
    // would create a load-dependent jerk at motion start.
    planned.endpoint_integral_tau = feedforward_model_tau_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_ = std::move(planned);
      if (active_->gripper_motion) {
        for (std::size_t side = 0U; side < 2U; ++side) {
          if ((active_->gripper_side_mask & (1U << side)) == 0U) {continue;}
          gripper_target_opening_m_[side] = active_->gripper_start_opening_m[side];
          gripper_speed_m_s_[side] = active_->command.gripper_speed_m_s;
          gripper_torque_limit_nm_[side] = active_->command.gripper_torque_limit_nm;
          gripper_exact_raw_hold_[side] = false;
        }
      }
      health_.adapter_state = AdapterState::executing;
      health_.command_id = active_->command_id;
      health_.plan_seed_feedback_seq[0] = active_->seed_seq[0];
      health_.plan_seed_feedback_seq[1] = active_->seed_seq[1];
      health_.plan_duration_ns = active_->total_duration_ns;
      health_.reason = "executing_collision_checked_physical_motion";
      notify_health();
    }
    dispatch_health();
  }

  void update_reference()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {
      // Idle is a fixed measured-feedback hold. Connect, Stop, completion,
      // disconnect, and E-stop establish or clear this reference explicitly;
      // following each new encoder sample here would let gravity walk a
      // completed arm away from its reported Cartesian target.
      return;
    }
    const auto now = Clock::now();
    const double elapsed = std::chrono::duration<double>(now - active_->segment_started).count();
    const double u = elapsed / active_->segment_duration_s;
    const double position_scale = smoothstep7(u);
    if (active_->gripper_motion) {
      for (std::size_t side = 0U; side < 2U; ++side) {
        if ((active_->gripper_side_mask & (1U << side)) == 0U) {continue;}
        gripper_target_opening_m_[side] = active_->gripper_start_opening_m[side] +
          (active_->gripper_target_opening_m[side] -
          active_->gripper_start_opening_m[side]) * position_scale;
      }
      return;
    }
    for (std::size_t side = 0; side < 2U; ++side) {
      for (std::size_t joint = 0; joint < OA_DOF; ++joint) {
        const double model = active_->segment_start[side][joint] +
          (active_->segment_target[side][joint] - active_->segment_start[side][joint]) *
          position_scale;
        if (u >= 1.0 && !active_->contact_policy) {
          const double error = active_->segment_target[side][joint] -
            measured_q_[side][joint];
          const double delta = std::clamp(
            kEndpointIntegralGainNmPerRadS * error *
            std::chrono::duration<double>(kCycle).count(),
            -kEndpointTorqueStepNm, kEndpointTorqueStepNm);
          active_->endpoint_integral_tau[side][joint] = std::clamp(
            active_->endpoint_integral_tau[side][joint] + delta,
            -kEndpointTorqueMaximumNm[joint], kEndpointTorqueMaximumNm[joint]);
        } else {
          active_->endpoint_integral_tau[side][joint] += std::clamp(
            -active_->endpoint_integral_tau[side][joint],
            -kEndpointTorqueStepNm, kEndpointTorqueStepNm);
        }
        feedforward_model_tau_[side][joint] =
          active_->endpoint_integral_tau[side][joint];
        target_raw_[side][joint] = calibration_.raw_position(side, joint, model);
      }
    }
  }

  void publish_command_feedback()
  {
    std::function<bool(const CommandFeedback &)> callback;
    CommandFeedback feedback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || !active_->command.feedback) {return;}
      callback = active_->command.feedback;
      feedback.lifecycle = kLifecycleExecuting;
      feedback.event = OA_RUNTIME_EVENT_STARTED;
      feedback.command_id = active_->command_id;
      feedback.feedback_seq[0] = feedback_sequence_[0].load(std::memory_order_acquire);
      feedback.feedback_seq[1] = feedback_sequence_[1].load(std::memory_order_acquire);
      const double elapsed = std::clamp(
        std::chrono::duration<double>(Clock::now() - active_->segment_started).count(),
        0.0, active_->segment_duration_s);
      const double total = static_cast<double>(active_->total_duration_ns) / 1.0e9;
      feedback.measured_progress = total <= 1.0e-12 ? 1.0 :
        std::clamp((active_->completed_duration_s + elapsed) / total, 0.0, 1.0);
    }
    bool accepted = false;
    try {accepted = callback(feedback);} catch (...) {}
    if (!accepted) {cancel_requested_.store(true, std::memory_order_release);}
  }

  void monitor_and_complete(const PairQ & measured)
  {
    std::optional<SessionCommand> terminal;
    CommandResult::Outcome terminal_outcome{CommandResult::Outcome::aborted};
    std::string terminal_reason;
    std::uint32_t terminal_event{OA_RUNTIME_EVENT_ABORTED};
    std::uint64_t terminal_id{0U};
    bool terminal_fault{false};
    bool preserve_endpoint_hold{false};
    PairQ endpoint_hold_q{};
    PairQ endpoint_hold_tau{};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_) {return;}
      if (cancel_requested_.load(std::memory_order_acquire)) {
        if (active_->gripper_motion) {
          for (std::size_t side = 0U; side < 2U; ++side) {
            if ((active_->gripper_side_mask & (1U << side)) == 0U) {continue;}
            gripper_target_opening_m_[side] = gripper_calibration_.opening_m(
              side, measured_raw_gripper_[side]);
            gripper_torque_limit_nm_[side] = kGripperHoldTorqueNm;
            gripper_exact_raw_hold_[side] = false;
          }
        }
        terminal = active_->command;
        terminal_outcome = CommandResult::Outcome::canceled;
        terminal_reason = "software_stop_holding_measured_pose";
        terminal_event = OA_RUNTIME_EVENT_STOPPED;
        terminal_id = active_->command_id;
        active_.reset();
      } else if (active_->gripper_motion) {
        const auto now = Clock::now();
        const double elapsed = std::chrono::duration<double>(
          now - active_->segment_started).count();
        bool all_at_target = true;
        bool torque_contact = false;
        for (std::size_t side = 0U; side < 2U; ++side) {
          if ((active_->gripper_side_mask & (1U << side)) == 0U) {continue;}
          const double opening = gripper_calibration_.opening_m(
            side, measured_raw_gripper_[side]);
          const double velocity = gripper_calibration_.velocity_m_s(
            side, last_sample_[side][7U].raw_dq);
          all_at_target = all_at_target && std::isfinite(opening) && std::isfinite(velocity) &&
            std::abs(opening - active_->gripper_target_opening_m[side]) <=
            kGripperPositionToleranceM && std::abs(velocity) <= kGripperVelocityToleranceMS;
          torque_contact = torque_contact ||
            std::abs(last_sample_[side][7U].raw_tau) >=
            0.90 * active_->command.gripper_torque_limit_nm;
        }
        active_->contact_streak = active_->command.gripper_stop_on_contact && torque_contact ?
          active_->contact_streak + 1U : 0U;
        active_->settled = elapsed >= active_->segment_duration_s && all_at_target ?
          active_->settled + 1U : 0U;
        if (active_->contact_streak >= kContactPersistenceCycles ||
          active_->settled >= kSettleCycles)
        {
          terminal = active_->command;
          terminal_outcome = CommandResult::Outcome::completed;
          terminal_reason = active_->contact_streak >= kContactPersistenceCycles ?
            "gripper_grasp_holding_on_encoder_torque_contact" :
            "gripper_completed_measured_feedback";
          terminal_event = active_->contact_streak >= kContactPersistenceCycles ?
            OA_RUNTIME_EVENT_STOPPED : OA_RUNTIME_EVENT_COMPLETED;
          terminal_id = active_->command_id;
          active_.reset();
        } else if (now - active_->segment_started >
          std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(active_->segment_duration_s)) +
          kMaximumSettleDuration)
        {
          for (std::size_t side = 0U; side < 2U; ++side) {
            if ((active_->gripper_side_mask & (1U << side)) == 0U) {continue;}
            gripper_target_opening_m_[side] = gripper_calibration_.opening_m(
              side, measured_raw_gripper_[side]);
            gripper_torque_limit_nm_[side] = kGripperHoldTorqueNm;
            gripper_exact_raw_hold_[side] = false;
          }
          terminal = active_->command;
          terminal_reason = "gripper_settle_timeout_holding_measured_position";
          terminal_event = OA_RUNTIME_EVENT_ABORTED;
          terminal_id = active_->command_id;
          active_.reset();
        }
      } else {
        const auto policy = active_->contact_policy ?
          OA_COLLISION_CONTACT_TERMINAL_CAPS : OA_COLLISION_CONTACT_NONE;
        const CollisionState collision = collision_state(
          measured, oa_collision_intervention_clearance_m(), policy);
        if (!collision.evaluated) {
          terminal = active_->command;
          terminal_reason = "live_collision_evaluation_failed";
          terminal_event = OA_RUNTIME_EVENT_FAULTED;
          terminal_id = active_->command_id;
          terminal_fault = true;
          active_.reset();
        } else if (active_->contact_policy && collision.terminal_pair && collision.claw_contact) {
          terminal = active_->command;
          terminal_outcome = CommandResult::Outcome::completed;
          terminal_reason = "converge_halted_on_proved_contact";
          terminal_event = OA_RUNTIME_EVENT_STOPPED;
          terminal_id = active_->command_id;
          active_.reset();
        } else if (!collision.clear && std::isfinite(active_->previous_clearance) &&
          collision.minimum < active_->previous_clearance - kClearanceWorseningEpsilon)
        {
          terminal = active_->command;
          terminal_reason = "live_measured_collision_intervention";
          terminal_event = OA_RUNTIME_EVENT_STOPPED;
          terminal_id = active_->command_id;
          active_.reset();
        } else {
          active_->previous_clearance = collision.minimum;
          if (active_->contact_policy) {
            const double fraction = active_->command.contact_torque_fraction > 0.0 ?
              active_->command.contact_torque_fraction : kDefaultContactTorqueFraction;
            bool torque_contact = false;
            for (std::size_t side = 0; side < 2U; ++side) {
              for (std::size_t joint = 0; joint < OA_DOF; ++joint) {
                const double threshold = profile_for(joint).tmax_nm * fraction;
                torque_contact = torque_contact ||
                  std::abs(last_sample_[side][joint].raw_tau) >= threshold;
              }
            }
            active_->contact_streak = torque_contact ? active_->contact_streak + 1U : 0U;
            if (active_->contact_streak >= kContactPersistenceCycles) {
              terminal = active_->command;
              terminal_outcome = CommandResult::Outcome::completed;
              terminal_reason = "converge_halted_on_encoder_torque_resistance";
              terminal_event = OA_RUNTIME_EVENT_STOPPED;
              terminal_id = active_->command_id;
              active_.reset();
            }
          }
          if (!terminal) {
            const double elapsed = std::chrono::duration<double>(
              Clock::now() - active_->segment_started).count();
            if (elapsed >= active_->segment_duration_s) {
              double maximum_error = 0.0;
              double maximum_velocity = 0.0;
              for (std::size_t side = 0; side < 2U; ++side) {
                for (std::size_t joint = 0; joint < OA_DOF; ++joint) {
                  maximum_error = std::max(maximum_error,
                    std::abs(measured[side][joint] - active_->segment_target[side][joint]));
                  maximum_velocity = std::max(maximum_velocity,
                    std::abs(calibration_.signed_value(
                      side, joint, last_sample_[side][joint].raw_dq)));
                }
              }
              active_->settled = maximum_error <= kPositionToleranceRad &&
                maximum_velocity <= kVelocityToleranceRadS ? active_->settled + 1U : 0U;
              if (active_->settled >= kSettleCycles) {
                ++active_->waypoint;
                if (active_->waypoint >= active_->waypoints.size()) {
                  endpoint_hold_q = active_->segment_target;
                  endpoint_hold_tau = active_->endpoint_integral_tau;
                  preserve_endpoint_hold = true;
                  terminal = active_->command;
                  terminal_outcome = CommandResult::Outcome::completed;
                  terminal_reason = "completed_measured_feedback";
                  terminal_event = OA_RUNTIME_EVENT_COMPLETED;
                  terminal_id = active_->command_id;
                  active_.reset();
                } else {
                  // Re-seed each route leg from the encoder pose actually
                  // reached. This is the dynamic path correction boundary.
                  active_->segment_start = measured;
                  active_->completed_duration_s += active_->segment_duration_s;
                  active_->segment_target = active_->waypoints[active_->waypoint];
                  active_->segment_duration_s = trajectory_duration_s(
                    active_->segment_start, active_->segment_target,
                    active_->command.motion_limit_scale);
                  active_->segment_started = Clock::now();
                  active_->settled = 0U;
                }
              } else if (Clock::now() - active_->segment_started >
                std::chrono::duration_cast<Clock::duration>(
                  std::chrono::duration<double>(active_->segment_duration_s)) +
                kMaximumSettleDuration)
              {
                terminal = active_->command;
                terminal_reason = "segment_settle_timeout_holding_measured_pose;error=" +
                  std::to_string(maximum_error) + ";velocity=" +
                  std::to_string(maximum_velocity);
                terminal_event = OA_RUNTIME_EVENT_ABORTED;
                terminal_id = active_->command_id;
                active_.reset();
              }
            }
          }
        }
      }
      if (terminal) {
        if (preserve_endpoint_hold) {
          target_raw_ = raw_from_model(endpoint_hold_q);
          feedforward_model_tau_ = endpoint_hold_tau;
        } else {
          target_raw_ = raw_from_model(measured);
          feedforward_model_tau_ = {};
        }
        health_.command_id = 0U;
      }
    }
    if (terminal) {
      const bool disable_confirmed = terminal_fault ? disable_and_close_for_fault() : true;
      terminal_command(*terminal, terminal_outcome, terminal_event, terminal_reason,
        terminal_id, feedback_sequence_[0].load(std::memory_order_acquire),
        feedback_sequence_[1].load(std::memory_order_acquire));
      if (terminal_fault) {
        set_health(
          AdapterState::fault,
          terminal_reason + (disable_confirmed ?
          "; all 16 motors confirmed disabled and CAN sockets closed" :
          "; disable unconfirmed for one or more motors; use the hardware stop"),
          kLifecycleFault);
      }
    }
  }

  void terminal_command(
    const SessionCommand & command, const CommandResult::Outcome outcome,
    const std::uint32_t event, const std::string & reason, const std::uint64_t command_id,
    const std::uint64_t left_seq, const std::uint64_t right_seq)
  {
    CommandResult result;
    result.outcome = outcome;
    result.runtime_status = outcome == CommandResult::Outcome::completed ||
      outcome == CommandResult::Outcome::canceled ? OA_RUNTIME_OK : OA_RUNTIME_EFAULT;
    result.command_id = command_id;
    result.seed_feedback_seq[0] = left_seq;
    result.seed_feedback_seq[1] = right_seq;
    result.terminal_feedback_seq[0] = feedback_sequence_[0].load(std::memory_order_acquire);
    result.terminal_feedback_seq[1] = feedback_sequence_[1].load(std::memory_order_acquire);
    result.lifecycle = armed() ? kLifecycleArmedIdle :
      (estop_asserted() ? kLifecycleEstop : kLifecycleDisarmed);
    result.event = event;
    result.cause = event == OA_RUNTIME_EVENT_COMPLETED ? OA_RUNTIME_STOP_CAUSE_PLAN_COMPLETE :
      (event == OA_RUNTIME_EVENT_ESTOP ? OA_RUNTIME_STOP_CAUSE_ESTOP :
      (reason.find("contact") != std::string::npos ||
      reason.find("resistance") != std::string::npos ? OA_RUNTIME_STOP_CAUSE_CONTACT :
      (reason.find("collision") != std::string::npos ? OA_RUNTIME_STOP_CAUSE_KEEPOUT :
      OA_RUNTIME_STOP_CAUSE_NONE)));
    result.collision_checked = true;
    result.motion_authorized = armed();
    result.reason = reason;
    bool terminal_ok = true;
    try {if (command.terminal) {terminal_ok = command.terminal(result);}} catch (...) {
      terminal_ok = false;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (owner_ == command.owner) {owner_.clear();}
      health_.owner.clear();
      health_.command_id = 0U;
      health_.terminal_feedback_seq[0] = feedback_sequence_[0].load(std::memory_order_acquire);
      health_.terminal_feedback_seq[1] = feedback_sequence_[1].load(std::memory_order_acquire);
      health_.adapter_state = armed() ? AdapterState::idle :
        (estop_asserted() ? AdapterState::stopped_requires_restart : AdapterState::starting);
      health_.reason = reason;
      notify_health();
    }
    dispatch_health();
    if (!terminal_ok) {
      const bool disable_confirmed = disable_and_close_for_fault();
      set_health(
        AdapterState::fault,
        std::string("terminal_callback_failed") + (disable_confirmed ?
        "; all 16 motors confirmed disabled and CAN sockets closed" :
        "; disable unconfirmed for one or more motors; use the hardware stop"),
        kLifecycleFault);
    }
  }

  void finish_active(
    const CommandResult::Outcome outcome, const std::uint32_t event, const std::string & reason)
  {
    std::optional<SessionCommand> command;
    std::uint64_t id = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_) {command = active_->command; id = active_->command_id; active_.reset();}
      else if (pending_) {command = *pending_; pending_.reset();}
    }
    if (command) {
      terminal_command(*command, outcome, event, reason, id,
        feedback_sequence_[0].load(std::memory_order_acquire),
        feedback_sequence_[1].load(std::memory_order_acquire));
    }
  }

  bool disable_and_close_for_fault()
  {
    armed_.store(false, std::memory_order_release);
    disable_all_immediate();
    bool confirmed = false;
    {
      std::lock_guard<std::mutex> io_lock(io_mutex_);
      confirmed = disable_until_confirmed();
      close_sockets();
    }
    connected_.store(false, std::memory_order_release);
    return confirmed;
  }

  void fault_and_disable(const std::string & reason)
  {
    const bool confirmed = disable_and_close_for_fault();
    finish_active(CommandResult::Outcome::aborted, OA_RUNTIME_EVENT_FAULTED, reason);
    set_health(
      AdapterState::fault, reason + (confirmed ?
      "; all 16 motors confirmed disabled and CAN sockets closed" :
      "; disable unconfirmed for one or more motors; use the hardware stop"),
      kLifecycleFault);
  }

  void worker_main() noexcept
  {
    auto next_cycle = Clock::now();
    while (true) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_until(lock, next_cycle, [this]() {return closing_ || pending_.has_value();});
        if (closing_) {break;}
      }
      dispatch_health();
      if (!armed_.load(std::memory_order_acquire)) {
        process_pending();
        next_cycle = Clock::now() + kCycle;
        continue;
      }
      if (estop_.load(std::memory_order_acquire) || oa_runtime_estop_asserted() != 0U) {
        (void)emergency_stop(ignored_detail_);
        next_cycle = Clock::now() + kCycle;
        continue;
      }
      process_pending();
      update_reference();
      PairQ measured{};
      bool complete = false;
      {
        std::lock_guard<std::mutex> io_lock(io_mutex_);
        if (armed_.load(std::memory_order_acquire)) {
          complete = exchange_frames(true, 1.0, kFeedbackDeadline, measured);
        }
      }
      if (!complete) {
        // Disconnect and E-stop intentionally clear armed_ before closing the
        // sockets. If either overlaps an in-flight exchange, that exchange is
        // expected to finish incomplete and must not overwrite the requested
        // terminal state with a spurious watchdog fault. A real incomplete
        // exchange while still armed and connected remains fail-closed.
        if (armed_.load(std::memory_order_acquire) &&
          connected_.load(std::memory_order_acquire))
        {
          fault_and_disable(
            "encoder_feedback_watchdog_expired: " + last_exchange_failure_);
        }
      } else {
        measured_q_ = measured;
        bool executing = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          executing = active_.has_value();
        }
        publish_measured(measured, executing ? kLifecycleExecuting : kLifecycleArmedIdle);
        publish_command_feedback();
        monitor_and_complete(measured);
      }
      next_cycle += kCycle;
      if (next_cycle < Clock::now()) {next_cycle = Clock::now() + kCycle;}
    }
    armed_.store(false, std::memory_order_release);
    disable_all_immediate();
  }

  RealControlConfig config_;
  StateCallback state_callback_;
  HealthCallback health_callback_;
  DisplayCalibration calibration_;
  bool calibration_loaded_{false};
  std::string calibration_detail_;
  GripperCalibration gripper_calibration_;
  bool gripper_calibration_loaded_{false};
  std::string gripper_calibration_detail_;
  mutable std::mutex mutex_;
  std::mutex io_mutex_;
  std::condition_variable cv_;
  std::thread worker_;
  bool closing_{false};
  std::array<std::atomic<int>, 2> sockets_{-1, -1};
  std::atomic<bool> connected_{false};
  std::atomic<bool> armed_{false};
  std::atomic<bool> estop_{false};
  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> health_notification_{false};
  std::optional<SessionCommand> pending_;
  std::optional<Active> active_;
  std::string owner_;
  SessionHealth health_{};
  PairQ measured_q_{};
  PairQ target_raw_{};
  PairQ feedforward_model_tau_{};
  std::array<double, 2> measured_raw_gripper_{};
  std::array<double, 2> gripper_target_raw_{};
  std::array<double, 2> gripper_target_opening_m_{};
  std::array<double, 2> gripper_speed_m_s_{kGripperDefaultSpeedMS, kGripperDefaultSpeedMS};
  std::array<double, 2> gripper_torque_limit_nm_{kGripperHoldTorqueNm, kGripperHoldTorqueNm};
  std::array<bool, 2> gripper_exact_raw_hold_{true, true};
  std::array<std::array<MotorSample, kPhysicalMotorsPerArm>, 2> last_sample_{};
  std::array<std::array<Clock::time_point, kPhysicalMotorsPerArm>, 2> last_feedback_{};
  std::array<std::atomic<std::uint64_t>, 2> feedback_sequence_{};
  std::uint64_t next_command_id_{1U};
  std::string ignored_detail_;
  std::string last_exchange_failure_;
};

RealControlSession::RealControlSession(
  RealControlConfig config, StateCallback state_callback, HealthCallback health_callback)
: impl_(std::make_unique<Impl>(
    std::move(config), std::move(state_callback), std::move(health_callback)))
{}

RealControlSession::~RealControlSession() = default;
bool RealControlSession::reserve(const std::string & owner, std::string & reason)
{return impl_->reserve(owner, reason);}
bool RealControlSession::submit(SessionCommand command, std::string & reason)
{return impl_->submit(std::move(command), reason);}
bool RealControlSession::cancel(const std::string & owner) {return impl_->cancel(owner);}
void RealControlSession::release(const std::string & owner, const std::string & reason)
{impl_->release(owner, reason);}
SessionHealth RealControlSession::health() const {return impl_->health();}
void RealControlSession::close() noexcept {if (impl_) {impl_->close();}}
bool RealControlSession::connect_and_enable(std::string & detail)
{return impl_->connect_and_enable(detail);}
bool RealControlSession::disconnect_and_disable(std::string & detail) noexcept
{return impl_->disconnect_and_disable(detail);}
bool RealControlSession::stop_motion(std::string & detail) noexcept
{return impl_->stop_motion(detail);}
bool RealControlSession::emergency_stop(std::string & detail) noexcept
{return impl_->emergency_stop(detail);}
bool RealControlSession::clear_emergency_stop(std::string & detail) noexcept
{return impl_->clear_emergency_stop(detail);}
bool RealControlSession::capture_grippers_closed(std::string & detail)
{return impl_->capture_grippers_closed(detail);}
bool RealControlSession::connected() const noexcept {return impl_->connected();}
bool RealControlSession::armed() const noexcept {return impl_->armed();}
bool RealControlSession::estop_asserted() const noexcept {return impl_->estop_asserted();}
bool RealControlSession::busy() const noexcept {return impl_->busy();}
std::string RealControlSession::status_json() const {return impl_->status_json();}

}  // namespace openarm_ik_ros::real
