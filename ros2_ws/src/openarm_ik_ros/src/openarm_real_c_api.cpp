// SPDX-License-Identifier: Apache-2.0
#include <openarm_real.h>

#include <openarm_control_msgs/action/move_gripper.hpp>
#include <openarm_control_msgs/action/move_joint.hpp>
#include <openarm_control_msgs/action/move_paired_tcp_scaled.hpp>
#include <openarm_model.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <unistd.h>

namespace
{
using namespace std::chrono_literals;
using MoveGripper = openarm_control_msgs::action::MoveGripper;
using MoveJoint = openarm_control_msgs::action::MoveJoint;
using MoveTcp = openarm_control_msgs::action::MovePairedTcpScaled;
using Trigger = std_srvs::srv::Trigger;

constexpr auto kStateMaximumAge = 500ms;
constexpr auto kSpinSlice = 20ms;
constexpr auto kCancellationWait = 3s;

bool result_layout_valid(const oa_real_result * value)
{
  return value != nullptr && value->abi_version == OA_REAL_ABI_VERSION &&
         value->struct_size >= sizeof(*value);
}

bool snapshot_layout_valid(const oa_real_snapshot * value)
{
  return value != nullptr && value->abi_version == OA_REAL_ABI_VERSION &&
         value->struct_size >= sizeof(*value);
}

void copy_reason(char destination[OA_REAL_REASON_CAPACITY], const std::string & value)
{
  std::snprintf(destination, OA_REAL_REASON_CAPACITY, "%s", value.c_str());
}

void reset_result(oa_real_result & value)
{
  const std::uint32_t size = value.struct_size;
  std::memset(&value, 0, sizeof(value));
  value.abi_version = OA_REAL_ABI_VERSION;
  value.struct_size = size;
}

void set_result(oa_real_result & out, const oa_real_status status, const std::string & reason)
{
  reset_result(out);
  out.status = status;
  copy_reason(out.reason, reason);
}

std::uint64_t steady_now_ns()
{
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool json_boolean(const std::string & json, const char * key)
{
  return json.find(std::string("\"") + key + "\":true") != std::string::npos;
}

std::uint32_t json_uint(
  const std::string & json, const char * key, const std::uint32_t fallback)
{
  const std::string prefix = std::string("\"") + key + "\":";
  const auto position = json.find(prefix);
  if (position == std::string::npos) {return fallback;}
  const char * first = json.data() + position + prefix.size();
  const char * last = json.data() + json.size();
  std::uint32_t value = fallback;
  const auto parsed = std::from_chars(first, last, value);
  return parsed.ec == std::errc{} ? value : fallback;
}

const char * joint_name(const oa_real_side side, const std::uint32_t joint)
{
  static constexpr std::array<const char *, 7> left{
    "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
    "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
    "openarm_left_joint7"};
  static constexpr std::array<const char *, 7> right{
    "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
    "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
    "openarm_right_joint7"};
  return side == OA_REAL_SIDE_LEFT ? left[joint - 1U] : right[joint - 1U];
}

template<typename Action>
oa_real_status finish_action_result(
  const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult & wrapped,
  const double progress, oa_real_result & out)
{
  reset_result(out);
  out.measured_progress = progress;
  if (!wrapped.result) {
    out.status = OA_REAL_EINTERNAL;
    copy_reason(out.reason, "controller returned no action result payload");
    return out.status;
  }
  out.outcome = wrapped.result->outcome;
  out.command_id = wrapped.result->command_id;
  out.cause = wrapped.result->cause;
  out.collision_checked = wrapped.result->collision_checked ? 1U : 0U;
  copy_reason(out.reason, wrapped.result->reason);
  if (wrapped.code == rclcpp_action::ResultCode::CANCELED ||
    wrapped.result->outcome == Action::Result::OUTCOME_CANCELED)
  {
    out.status = OA_REAL_ECANCELED;
  } else if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
    wrapped.result->outcome == Action::Result::OUTCOME_COMPLETED)
  {
    out.status = OA_REAL_OK;
  } else if (wrapped.result->outcome == Action::Result::OUTCOME_REJECTED) {
    out.status = OA_REAL_EREJECTED;
  } else {
    out.status = OA_REAL_EABORTED;
  }
  return out.status;
}

class RealClient final
{
public:
  RealClient()
  {
    context_ = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init_options;
    context_->init(0, nullptr, init_options);
    rclcpp::NodeOptions node_options;
    node_options.context(context_);
    const auto sequence = next_node_.fetch_add(1U, std::memory_order_relaxed);
    node_ = std::make_shared<rclcpp::Node>(
      "openarm_real_c_" + std::to_string(static_cast<long long>(::getpid())) + "_" +
      std::to_string(sequence), node_options);
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(node_);

    status_subscription_ = node_->create_subscription<std_msgs::msg::String>(
      "/openarm_real/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        have_status_ = true;
        connected_ = json_boolean(message->data, "connected");
        armed_ = json_boolean(message->data, "armed");
        estop_ = json_boolean(message->data, "estop");
        busy_ = json_boolean(message->data, "busy");
        active_side_mask_ = json_uint(message->data, "active_side_mask", 3U);
      });
    joint_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {on_joint_state(*message);});

    connect_ = node_->create_client<Trigger>("/openarm_real/connect");
    disconnect_ = node_->create_client<Trigger>("/openarm_real/disconnect");
    stop_ = node_->create_client<Trigger>("/openarm_real/stop");
    estop_assert_ = node_->create_client<Trigger>("/openarm_real/estop");
    estop_clear_ = node_->create_client<Trigger>("/openarm_real/estop_clear");
    neutral_ = node_->create_client<Trigger>("/openarm_real/neutral");
    joint_action_ = rclcpp_action::create_client<MoveJoint>(node_, "/openarm_ik/move_joint");
    tcp_action_ = rclcpp_action::create_client<MoveTcp>(
      node_, "/openarm_ik/move_paired_tcp_scaled");
    gripper_action_ = rclcpp_action::create_client<MoveGripper>(
      node_, "/openarm_ik/move_gripper");
  }

  ~RealClient()
  {
    try {
      if (executor_ && node_) {executor_->remove_node(node_);}
      if (context_ && context_->is_valid()) {context_->shutdown("OpenArm real C client destroyed");}
    } catch (...) {}
  }

  RealClient(const RealClient &) = delete;
  RealClient & operator=(const RealClient &) = delete;

  oa_real_status wait_ready(const std::uint32_t timeout_ms, oa_real_result & out)
  {
    std::lock_guard<std::mutex> lock(api_mutex_);
    if (timeout_ms == 0U) {
      set_result(out, OA_REAL_EINVAL, "timeout must be positive");
      return out.status;
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    if (!spin_until([this]() {return have_status_;}, deadline)) {
      set_result(out, OA_REAL_EUNAVAILABLE, "production real-arm controller status is unavailable");
      return out.status;
    }
    if (!wait_service(connect_, deadline) || !wait_service(disconnect_, deadline) ||
      !wait_service(stop_, deadline) || !wait_service(estop_assert_, deadline) ||
      !wait_service(estop_clear_, deadline) || !wait_service(neutral_, deadline) ||
      !wait_action<MoveJoint>(joint_action_, deadline) ||
      !wait_action<MoveTcp>(tcp_action_, deadline) ||
      !wait_action<MoveGripper>(gripper_action_, deadline))
    {
      set_result(out, OA_REAL_EUNAVAILABLE, "production controller service/action set is incomplete");
      return out.status;
    }
    set_result(out, OA_REAL_OK, "production real-arm controller is ready; CAN remains separately managed");
    return out.status;
  }

  oa_real_status read(const std::uint32_t timeout_ms, oa_real_snapshot & out)
  {
    std::lock_guard<std::mutex> lock(api_mutex_);
    if (timeout_ms == 0U) {return OA_REAL_EINVAL;}
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    if (!spin_until([this]() {return have_status_;}, deadline)) {return OA_REAL_EUNAVAILABLE;}
    const bool need_encoder = connected_;
    if (need_encoder && !spin_until([this]() {return state_fresh();}, deadline)) {
      return OA_REAL_ESTALE;
    }
    const std::uint32_t size = out.struct_size;
    std::memset(&out, 0, sizeof(out));
    out.abi_version = OA_REAL_ABI_VERSION;
    out.struct_size = size;
    out.controller_available = 1U;
    out.connected = connected_ ? 1U : 0U;
    out.armed = armed_ ? 1U : 0U;
    out.estop_asserted = estop_ ? 1U : 0U;
    out.busy = busy_ ? 1U : 0U;
    out.active_side_mask = active_side_mask_;
    if (state_fresh()) {
      const std::uint32_t caller_size = out.struct_size;
      out = latest_;
      out.struct_size = caller_size;
      out.controller_available = 1U;
      out.connected = connected_ ? 1U : 0U;
      out.armed = armed_ ? 1U : 0U;
      out.estop_asserted = estop_ ? 1U : 0U;
      out.busy = busy_ ? 1U : 0U;
      out.encoder_state_valid = connected_ ? 1U : 0U;
      out.active_side_mask = active_side_mask_;
    }
    return OA_REAL_OK;
  }

  oa_real_status service(
    const std::shared_ptr<rclcpp::Client<Trigger>> & client, const std::uint32_t timeout_ms,
    oa_real_result & out)
  {
    std::lock_guard<std::mutex> lock(api_mutex_);
    if (timeout_ms == 0U) {
      set_result(out, OA_REAL_EINVAL, "timeout must be positive");
      return out.status;
    }
    const auto timeout = std::chrono::milliseconds(timeout_ms);
    if (!client->wait_for_service(timeout)) {
      set_result(out, OA_REAL_EUNAVAILABLE, "production controller service is unavailable");
      return out.status;
    }
    auto future = client->async_send_request(std::make_shared<Trigger::Request>());
    if (executor_->spin_until_future_complete(future, timeout) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      set_result(out, OA_REAL_ETIMEOUT, "production controller service timed out");
      return out.status;
    }
    const auto response = future.get();
    set_result(out, response->success ? OA_REAL_OK : OA_REAL_EREJECTED, response->message);
    return out.status;
  }

  oa_real_status move_joint(
    const oa_real_side side, const std::uint32_t joint, const double target,
    const std::uint32_t timeout_ms, oa_real_result & out)
  {
    std::lock_guard<std::mutex> lock(api_mutex_);
    if ((side != OA_REAL_SIDE_LEFT && side != OA_REAL_SIDE_RIGHT) || joint < 1U || joint > 7U ||
      !std::isfinite(target) || timeout_ms == 0U)
    {
      set_result(out, OA_REAL_EINVAL, "invalid side, joint, finite target, or timeout");
      return out.status;
    }
    MoveJoint::Goal goal;
    goal.stamp = node_->now();
    goal.joint_name = joint_name(side, joint);
    goal.target_rad = target;
    return run_action<MoveJoint>(joint_action_, std::move(goal), timeout_ms, out);
  }

  oa_real_status move_tcp(
    const oa_real_side side, const oa_vec3d & target, const oa_length_unit unit,
    const double scale, const std::uint32_t timeout_ms, oa_real_result & out)
  {
    std::lock_guard<std::mutex> lock(api_mutex_);
    oa_vec3d metres{};
    if ((side != OA_REAL_SIDE_LEFT && side != OA_REAL_SIDE_RIGHT) ||
      oa_vec3d_convert(&target, unit, OA_LENGTH_UNIT_METRES, &metres) != OA_UNITS_OK ||
      !valid_scale(scale) || timeout_ms == 0U)
    {
      set_result(out, OA_REAL_EINVAL, "invalid side, binary64 coordinate/unit, scale, or timeout");
      return out.status;
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    if (!spin_until([this]() {return have_status_ && connected_ && armed_ && state_fresh();}, deadline)) {
      set_result(out, OA_REAL_ESTALE, "fresh armed encoder state is unavailable");
      return out.status;
    }
    MoveTcp::Goal goal = tcp_goal_from_snapshot(scale);
    auto & point = side == OA_REAL_SIDE_LEFT ? goal.left_tcp_m : goal.right_tcp_m;
    point.x = metres.x;
    point.y = metres.y;
    point.z = metres.z;
    goal.preserve_side = side == OA_REAL_SIDE_LEFT ? MoveTcp::Goal::PRESERVE_RIGHT :
      MoveTcp::Goal::PRESERVE_LEFT;
    return run_action<MoveTcp>(tcp_action_, std::move(goal), remaining_ms(deadline), out);
  }

  oa_real_status move_paired(
    const oa_vec3d & left, const oa_vec3d & right, const oa_length_unit unit,
    const double scale, const std::uint32_t timeout_ms, oa_real_result & out)
  {
    std::lock_guard<std::mutex> lock(api_mutex_);
    oa_vec3d left_m{};
    oa_vec3d right_m{};
    if (oa_vec3d_convert(&left, unit, OA_LENGTH_UNIT_METRES, &left_m) != OA_UNITS_OK ||
      oa_vec3d_convert(&right, unit, OA_LENGTH_UNIT_METRES, &right_m) != OA_UNITS_OK ||
      !valid_scale(scale) || timeout_ms == 0U)
    {
      set_result(out, OA_REAL_EINVAL, "invalid binary64 coordinates/unit, scale, or timeout");
      return out.status;
    }
    MoveTcp::Goal goal;
    goal.header.stamp = node_->now();
    goal.header.frame_id = "openarm_body_link0";
    goal.left_tcp_m.x = left_m.x;
    goal.left_tcp_m.y = left_m.y;
    goal.left_tcp_m.z = left_m.z;
    goal.right_tcp_m.x = right_m.x;
    goal.right_tcp_m.y = right_m.y;
    goal.right_tcp_m.z = right_m.z;
    goal.motion_limit_scale = scale;
    goal.preserve_side = MoveTcp::Goal::PRESERVE_NONE;
    return run_action<MoveTcp>(tcp_action_, std::move(goal), timeout_ms, out);
  }

  oa_real_status move_gripper(
    const oa_real_gripper_mask side_mask, const double opening, const double speed,
    const double torque, const std::uint32_t stop_on_contact, const std::uint32_t timeout_ms,
    oa_real_result & out)
  {
    std::lock_guard<std::mutex> lock(api_mutex_);
    if (side_mask < OA_REAL_GRIPPER_LEFT || side_mask > OA_REAL_GRIPPER_BOTH ||
      !std::isfinite(opening) || opening < 0.0 || opening > 0.044 ||
      !std::isfinite(speed) || speed <= 0.0 || speed > 0.011 ||
      !std::isfinite(torque) || torque < 0.05 || torque > 1.5 ||
      stop_on_contact > 1U || timeout_ms == 0U)
    {
      set_result(out, OA_REAL_EINVAL, "invalid gripper mask, opening, speed, torque, contact flag, or timeout");
      return out.status;
    }
    MoveGripper::Goal goal;
    goal.stamp = node_->now();
    goal.side_mask = static_cast<std::uint8_t>(side_mask);
    goal.target_opening_m = opening;
    goal.maximum_opening_speed_m_s = speed;
    goal.maximum_motor_torque_nm = torque;
    goal.stop_on_contact = stop_on_contact != 0U;
    return run_action<MoveGripper>(gripper_action_, std::move(goal), timeout_ms, out);
  }

  std::shared_ptr<rclcpp::Client<Trigger>> connect() const {return connect_;}
  std::shared_ptr<rclcpp::Client<Trigger>> disconnect() const {return disconnect_;}
  std::shared_ptr<rclcpp::Client<Trigger>> stop() const {return stop_;}
  std::shared_ptr<rclcpp::Client<Trigger>> estop_assert() const {return estop_assert_;}
  std::shared_ptr<rclcpp::Client<Trigger>> estop_clear() const {return estop_clear_;}
  std::shared_ptr<rclcpp::Client<Trigger>> neutral() const {return neutral_;}

private:
  template<typename Predicate>
  bool spin_until(Predicate predicate, const std::chrono::steady_clock::time_point deadline)
  {
    while (context_->is_valid()) {
      if (predicate()) {return true;}
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {break;}
      executor_->spin_once(std::min(
        std::chrono::steady_clock::duration(kSpinSlice), deadline - now));
    }
    return predicate();
  }

  template<typename Service>
  static bool wait_service(
    const std::shared_ptr<rclcpp::Client<Service>> & client,
    const std::chrono::steady_clock::time_point deadline)
  {
    const auto now = std::chrono::steady_clock::now();
    return now < deadline && client->wait_for_service(deadline - now);
  }

  template<typename Action>
  static bool wait_action(
    const typename rclcpp_action::Client<Action>::SharedPtr & client,
    const std::chrono::steady_clock::time_point deadline)
  {
    const auto now = std::chrono::steady_clock::now();
    return now < deadline && client->wait_for_action_server(deadline - now);
  }

  template<typename Action>
  oa_real_status run_action(
    const typename rclcpp_action::Client<Action>::SharedPtr & client,
    typename Action::Goal goal, const std::uint32_t timeout_ms, oa_real_result & out)
  {
    if (timeout_ms == 0U || !client->wait_for_action_server(std::chrono::milliseconds(timeout_ms))) {
      set_result(out, timeout_ms == 0U ? OA_REAL_EINVAL : OA_REAL_EUNAVAILABLE,
        "production controller action server is unavailable");
      return out.status;
    }
    measured_progress_ = 0.0;
    typename rclcpp_action::Client<Action>::SendGoalOptions options;
    options.feedback_callback = [this](
      typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr,
      const std::shared_ptr<const typename Action::Feedback> feedback)
      {
        measured_progress_ = feedback->measured_progress;
      };
    auto goal_future = client->async_send_goal(goal, options);
    const auto timeout = std::chrono::milliseconds(timeout_ms);
    if (executor_->spin_until_future_complete(goal_future, timeout) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      set_result(out, OA_REAL_ETIMEOUT, "controller goal response timed out before acceptance");
      return out.status;
    }
    const auto handle = goal_future.get();
    if (!handle) {
      set_result(out, OA_REAL_EREJECTED, "controller rejected the motion goal");
      return out.status;
    }
    auto result_future = client->async_get_result(handle);
    if (executor_->spin_until_future_complete(result_future, timeout) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return finish_action_result<Action>(result_future.get(), measured_progress_, out);
    }
    auto cancel_future = client->async_cancel_goal(handle);
    (void)executor_->spin_until_future_complete(cancel_future, kCancellationWait);
    if (executor_->spin_until_future_complete(result_future, kCancellationWait) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      (void)finish_action_result<Action>(result_future.get(), measured_progress_, out);
      if (out.status == OA_REAL_ECANCELED) {
        out.status = OA_REAL_ETIMEOUT;
        copy_reason(out.reason, "motion timed out and controller confirmed cancellation");
      }
      return out.status;
    }
    set_result(out, OA_REAL_ETIMEOUT, "motion timed out; cancellation terminal result is unconfirmed");
    return out.status;
  }

  void on_joint_state(const sensor_msgs::msg::JointState & message)
  {
    if (message.name.size() != message.position.size() ||
      (!message.velocity.empty() && message.velocity.size() != message.name.size()) ||
      (!message.effort.empty() && message.effort.size() != message.name.size())) return;
    oa_real_snapshot next{};
    next.abi_version = OA_REAL_ABI_VERSION;
    next.struct_size = sizeof(next);
    std::uint32_t mask = 0U;
    for (std::size_t index = 0U; index < message.name.size(); ++index) {
      bool matched = false;
      for (std::size_t side = 0U; side < 2U && !matched; ++side) {
        for (std::size_t joint = 0U; joint < 7U; ++joint) {
          if (message.name[index] == joint_name(static_cast<oa_real_side>(side), joint + 1U)) {
            const std::uint32_t bit = 1U << (side * 8U + joint);
            if ((mask & bit) != 0U || !std::isfinite(message.position[index])) {return;}
            next.joint_position_rad[side][joint] = message.position[index];
            next.joint_velocity_rad_s[side][joint] = message.velocity.empty() ? 0.0 :
              message.velocity[index];
            next.joint_torque_nm[side][joint] = message.effort.empty() ? 0.0 :
              message.effort[index];
            mask |= bit;
            matched = true;
            break;
          }
        }
      }
      if (matched) {continue;}
      for (std::size_t side = 0U; side < 2U; ++side) {
        const char * name = side == 0U ? "openarm_left_finger_joint1" :
          "openarm_right_finger_joint1";
        if (message.name[index] == name) {
          const std::uint32_t bit = 1U << (side * 8U + 7U);
          if ((mask & bit) != 0U || !std::isfinite(message.position[index])) {return;}
          next.gripper_opening_m[side] = message.position[index];
          next.gripper_velocity_m_s[side] = message.velocity.empty() ? 0.0 :
            message.velocity[index];
          next.gripper_motor_torque_nm[side] = message.effort.empty() ? 0.0 :
            message.effort[index];
          mask |= bit;
          break;
        }
      }
    }
    if (mask != UINT32_C(0xffff)) {return;}
    for (std::size_t side = 0U; side < 2U; ++side) {
      oa_fk_result fk{};
      const oa_model * model = side == 0U ? oa_model_left_v10_bimanual() :
        oa_model_right_v10_bimanual();
      if (oa_fk(model, next.joint_position_rad[side], &fk) != OA_MODEL_OK) {return;}
      next.tcp_m[side][0] = fk.hand_tcp.m[3];
      next.tcp_m[side][1] = fk.hand_tcp.m[7];
      next.tcp_m[side][2] = fk.hand_tcp.m[11];
    }
    const auto stamp = rclcpp::Time(message.header.stamp).nanoseconds();
    if (stamp <= 0) {return;}
    next.ros_stamp_ns = static_cast<std::uint64_t>(stamp);
    next.receipt_steady_ns = steady_now_ns();
    next.encoder_state_valid = 1U;
    next.active_side_mask = active_side_mask_;
    latest_ = next;
    have_state_ = true;
  }

  bool state_fresh() const
  {
    if (!have_state_ || latest_.receipt_steady_ns == 0U) {return false;}
    const auto now = steady_now_ns();
    const auto maximum = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(kStateMaximumAge).count());
    return now >= latest_.receipt_steady_ns && now - latest_.receipt_steady_ns <= maximum;
  }

  MoveTcp::Goal tcp_goal_from_snapshot(const double scale) const
  {
    MoveTcp::Goal goal;
    goal.header.stamp = node_->now();
    goal.header.frame_id = "openarm_body_link0";
    goal.left_tcp_m.x = latest_.tcp_m[0][0];
    goal.left_tcp_m.y = latest_.tcp_m[0][1];
    goal.left_tcp_m.z = latest_.tcp_m[0][2];
    goal.right_tcp_m.x = latest_.tcp_m[1][0];
    goal.right_tcp_m.y = latest_.tcp_m[1][1];
    goal.right_tcp_m.z = latest_.tcp_m[1][2];
    goal.motion_limit_scale = scale;
    return goal;
  }

  static bool valid_scale(const double scale)
  {
    return std::isfinite(scale) && scale >= 0.5 && scale <= 1.0;
  }

  static std::uint32_t remaining_ms(const std::chrono::steady_clock::time_point deadline)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {return 1U;}
    const auto value = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
    return static_cast<std::uint32_t>(std::max<std::int64_t>(1, value));
  }

  static std::atomic<std::uint64_t> next_node_;
  std::mutex api_mutex_;
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  std::shared_ptr<rclcpp::Client<Trigger>> connect_;
  std::shared_ptr<rclcpp::Client<Trigger>> disconnect_;
  std::shared_ptr<rclcpp::Client<Trigger>> stop_;
  std::shared_ptr<rclcpp::Client<Trigger>> estop_assert_;
  std::shared_ptr<rclcpp::Client<Trigger>> estop_clear_;
  std::shared_ptr<rclcpp::Client<Trigger>> neutral_;
  rclcpp_action::Client<MoveJoint>::SharedPtr joint_action_;
  rclcpp_action::Client<MoveTcp>::SharedPtr tcp_action_;
  rclcpp_action::Client<MoveGripper>::SharedPtr gripper_action_;
  bool have_status_{false};
  bool connected_{false};
  bool armed_{false};
  bool estop_{false};
  bool busy_{false};
  std::uint32_t active_side_mask_{3U};
  bool have_state_{false};
  oa_real_snapshot latest_{};
  double measured_progress_{0.0};
};

std::atomic<std::uint64_t> RealClient::next_node_{1U};

RealClient * implementation(oa_real_client * client)
{
  return reinterpret_cast<RealClient *>(client);
}

template<typename Function>
oa_real_status call_with_result(oa_real_client * client, oa_real_result * out, Function function)
{
  if (client == nullptr) {return OA_REAL_EINVAL;}
  if (!result_layout_valid(out)) {return OA_REAL_EABI;}
  try {
    return function(*implementation(client), *out);
  } catch (const std::exception & error) {
    set_result(*out, OA_REAL_EINTERNAL, error.what());
    return out->status;
  } catch (...) {
    set_result(*out, OA_REAL_EINTERNAL, "unknown C ABI implementation exception");
    return out->status;
  }
}
}

extern "C"
{
void oa_real_result_init(oa_real_result * out)
{
  if (out == nullptr) {return;}
  std::memset(out, 0, sizeof(*out));
  out->abi_version = OA_REAL_ABI_VERSION;
  out->struct_size = sizeof(*out);
}

void oa_real_snapshot_init(oa_real_snapshot * out)
{
  if (out == nullptr) {return;}
  std::memset(out, 0, sizeof(*out));
  out->abi_version = OA_REAL_ABI_VERSION;
  out->struct_size = sizeof(*out);
}

const char * oa_real_status_string(const oa_real_status status)
{
  switch (status) {
    case OA_REAL_OK: return "ok";
    case OA_REAL_EINVAL: return "invalid_argument";
    case OA_REAL_EABI: return "abi_mismatch";
    case OA_REAL_EUNAVAILABLE: return "controller_unavailable";
    case OA_REAL_ETIMEOUT: return "timeout";
    case OA_REAL_EREJECTED: return "rejected";
    case OA_REAL_EABORTED: return "aborted";
    case OA_REAL_ECANCELED: return "canceled";
    case OA_REAL_ESTALE: return "stale_encoder_state";
    case OA_REAL_EINTERNAL: return "internal_error";
    default: return "unknown";
  }
}

oa_real_status oa_real_client_create(oa_real_client ** out)
{
  if (out == nullptr) {return OA_REAL_EINVAL;}
  *out = nullptr;
  try {
    *out = reinterpret_cast<oa_real_client *>(new RealClient());
    return OA_REAL_OK;
  } catch (...) {
    return OA_REAL_EINTERNAL;
  }
}

void oa_real_client_destroy(oa_real_client * client)
{
  delete implementation(client);
}

oa_real_status oa_real_client_wait_ready(
  oa_real_client * client, const std::uint32_t timeout_ms, oa_real_result * out)
{
  return call_with_result(client, out, [timeout_ms](RealClient & value, oa_real_result & result) {
    return value.wait_ready(timeout_ms, result);
  });
}

oa_real_status oa_real_client_read(
  oa_real_client * client, const std::uint32_t timeout_ms, oa_real_snapshot * out)
{
  if (client == nullptr) {return OA_REAL_EINVAL;}
  if (!snapshot_layout_valid(out)) {return OA_REAL_EABI;}
  try {return implementation(client)->read(timeout_ms, *out);}
  catch (...) {return OA_REAL_EINTERNAL;}
}

#define OA_REAL_SERVICE_WRAPPER(name, accessor) \
  oa_real_status name(oa_real_client * client, const std::uint32_t timeout_ms, \
    oa_real_result * out) \
  { \
    return call_with_result(client, out, [timeout_ms](RealClient & value, \
      oa_real_result & result) {return value.service(value.accessor(), timeout_ms, result);}); \
  }

OA_REAL_SERVICE_WRAPPER(oa_real_client_connect, connect)
OA_REAL_SERVICE_WRAPPER(oa_real_client_disconnect, disconnect)
OA_REAL_SERVICE_WRAPPER(oa_real_client_stop, stop)
OA_REAL_SERVICE_WRAPPER(oa_real_client_estop, estop_assert)
OA_REAL_SERVICE_WRAPPER(oa_real_client_estop_clear, estop_clear)
OA_REAL_SERVICE_WRAPPER(oa_real_client_neutral, neutral)
#undef OA_REAL_SERVICE_WRAPPER

oa_real_status oa_real_client_move_joint(
  oa_real_client * client, const oa_real_side side, const std::uint32_t joint,
  const double target_rad, const std::uint32_t timeout_ms, oa_real_result * out)
{
  return call_with_result(client, out,
    [=](RealClient & value, oa_real_result & result) {
      return value.move_joint(side, joint, target_rad, timeout_ms, result);
    });
}

oa_real_status oa_real_client_move_tcp(
  oa_real_client * client, const oa_real_side side, const oa_vec3d * target,
  const oa_length_unit unit, const double motion_limit_scale,
  const std::uint32_t timeout_ms, oa_real_result * out)
{
  if (target == nullptr) {return OA_REAL_EINVAL;}
  const oa_vec3d copy = *target;
  return call_with_result(client, out,
    [=](RealClient & value, oa_real_result & result) {
      return value.move_tcp(side, copy, unit, motion_limit_scale, timeout_ms, result);
    });
}

oa_real_status oa_real_client_move_paired_tcp(
  oa_real_client * client, const oa_vec3d * left_target,
  const oa_vec3d * right_target, const oa_length_unit unit,
  const double motion_limit_scale, const std::uint32_t timeout_ms,
  oa_real_result * out)
{
  if (left_target == nullptr || right_target == nullptr) {return OA_REAL_EINVAL;}
  const oa_vec3d left = *left_target;
  const oa_vec3d right = *right_target;
  return call_with_result(client, out,
    [=](RealClient & value, oa_real_result & result) {
      return value.move_paired(left, right, unit, motion_limit_scale, timeout_ms, result);
    });
}

oa_real_status oa_real_client_move_gripper(
  oa_real_client * client, const oa_real_gripper_mask side_mask,
  const double target_opening_m, const double maximum_opening_speed_m_s,
  const double maximum_motor_torque_nm, const std::uint32_t stop_on_contact,
  const std::uint32_t timeout_ms, oa_real_result * out)
{
  return call_with_result(client, out,
    [=](RealClient & value, oa_real_result & result) {
      return value.move_gripper(side_mask, target_opening_m, maximum_opening_speed_m_s,
        maximum_motor_torque_nm, stop_on_contact, timeout_ms, result);
    });
}
}
