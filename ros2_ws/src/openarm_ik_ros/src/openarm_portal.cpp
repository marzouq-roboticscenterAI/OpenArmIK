// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/portal_core.hpp"
#include "openarm_ik_ros/rviz_capture.hpp"

#include <openarm_runtime_motion.h>

#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/thread_pool.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <openarm_control_msgs/action/move_paired_tcp_scaled.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>
#include <sys/random.h>
#include <unistd.h>

namespace openarm_ik_ros::portal
{
namespace
{
namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using tcp = asio::ip::tcp;
using Action = openarm_control_msgs::action::MovePairedTcpScaled;
using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
static_assert(std::is_same_v<Point::value_type, double>);
static_assert(std::is_same_v<decltype(Action::Goal{}.left_tcp_m.x), double>);
static_assert(std::is_same_v<decltype(Action::Goal{}.right_tcp_m.z), double>);
static_assert(std::is_same_v<decltype(Action::Goal{}.motion_limit_scale), double>);
constexpr auto kStateFreshness = std::chrono::milliseconds(500);
constexpr auto kDiagnosticFreshness = std::chrono::milliseconds(1500);
volatile std::sig_atomic_t stop_requested = 0;

void signal_handler(int)
{
  stop_requested = 1;
}

std::int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string token()
{
  std::array<unsigned char, 32> bytes{};
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const ssize_t received = getrandom(bytes.data() + offset, bytes.size() - offset, 0);
    if (received < 0) {
      if (errno == EINTR) {continue;}
      throw std::runtime_error("getrandom failed while creating CSRF token");
    }
    if (received == 0) {
      throw std::runtime_error("getrandom returned no CSRF token bytes");
    }
    offset += static_cast<std::size_t>(received);
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const unsigned char value : bytes) {
    output << std::setw(2) << static_cast<unsigned int>(value);
  }
  return output.str();
}

std::string number(double value)
{
  std::ostringstream output;
  output << std::fixed << std::setprecision(6) << value;
  return output.str();
}

template<typename Body>
void write_response(beast::tcp_stream & stream, http::response<Body> & response)
{
  response.keep_alive(false);
  response.set(http::field::server, "openarm-portal");
  response.set(http::field::cache_control, "no-store");
  response.set(http::field::x_content_type_options, "nosniff");
  response.set(http::field::referrer_policy, "no-referrer");
  response.set(http::field::content_security_policy,
    "default-src 'none'; script-src 'self'; style-src 'self'; img-src 'self'; "
    "connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'");
  response.prepare_payload();
  beast::error_code error;
  http::write(stream, response, error);
}

void write_json(beast::tcp_stream & stream, http::status status, std::string body)
{
  http::response<http::string_body> response{status, 11};
  response.set(http::field::content_type, "application/json; charset=utf-8");
  response.body() = std::move(body);
  write_response(stream, response);
}

std::string_view header_view(beast::string_view value)
{
  return {value.data(), value.size()};
}

bool ascii_iequals(std::string_view left, std::string_view right)
{
  return left.size() == right.size() && std::equal(
    left.begin(), left.end(), right.begin(),
    [](unsigned char a, unsigned char b) {
      return std::tolower(a) == std::tolower(b);
    });
}

template<typename Request>
SafeRequestHeaders safe_headers(const Request & request)
{
  SafeRequestHeaders headers;
  for (const auto & field : request.base()) {
    const std::string_view name = header_view(field.name_string());
    if (ascii_iequals(name, "Host")) {
      ++headers.host_count;
      if (headers.host_count == 1) {headers.host = header_view(field.value());}
    } else if (ascii_iequals(name, "Origin")) {
      ++headers.origin_count;
      if (headers.origin_count == 1) {headers.origin = header_view(field.value());}
    } else if (ascii_iequals(name, "Sec-Fetch-Site")) {
      ++headers.sec_fetch_site_count;
      if (headers.sec_fetch_site_count == 1) {headers.sec_fetch_site = header_view(field.value());}
    }
  }
  return headers;
}

template<typename Request>
bool unique_header(const Request & request, std::string_view expected, std::string_view & value)
{
  std::size_t count = 0;
  for (const auto & field : request.base()) {
    if (ascii_iequals(header_view(field.name_string()), expected)) {
      ++count;
      if (count == 1) {value = header_view(field.value());}
    }
  }
  return count == 1;
}

std::filesystem::path installed_share_directory()
{
  std::array<char, 4096> executable{};
  const ssize_t length = readlink("/proc/self/exe", executable.data(), executable.size() - 1);
  if (length <= 0) {
    throw std::runtime_error("cannot resolve portal executable for installed viewer assets");
  }
  const std::filesystem::path path(std::string(executable.data(), static_cast<std::size_t>(length)));
  const std::filesystem::path prefix = path.parent_path().parent_path().parent_path();
  return prefix / "share" / "openarm_ik_ros";
}

struct StaticAsset
{
  std::string_view route;
  std::string_view relative;
  std::string_view content_type;
  std::uintmax_t maximum_size;
  std::uintmax_t exact_size;
  std::string_view sha256;
  bool served;
};

std::uint32_t rotate_right(std::uint32_t value, unsigned count)
{
  return (value >> count) | (value << (32U - count));
}

std::string sha256(std::string_view input)
{
  static constexpr std::array<std::uint32_t, 64> constants{{
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U,
    0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
    0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U, 0xe49b69c1U, 0xefbe4786U,
    0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
    0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
    0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
    0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U, 0xa2bfe8a1U, 0xa81a664bU,
    0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
    0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU,
    0x5b9cca4fU, 0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U,
  }};
  std::vector<unsigned char> bytes(input.begin(), input.end());
  const std::uint64_t bit_length = static_cast<std::uint64_t>(bytes.size()) * 8U;
  bytes.push_back(0x80U);
  while ((bytes.size() % 64U) != 56U) {bytes.push_back(0U);}
  for (int shift = 56; shift >= 0; shift -= 8) {
    bytes.push_back(static_cast<unsigned char>((bit_length >> shift) & 0xffU));
  }
  std::array<std::uint32_t, 8> state{{
    0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
    0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U,
  }};
  for (std::size_t offset = 0; offset < bytes.size(); offset += 64U) {
    std::array<std::uint32_t, 64> words{};
    for (std::size_t index = 0; index < 16U; ++index) {
      const std::size_t at = offset + index * 4U;
      words[index] = (static_cast<std::uint32_t>(bytes[at]) << 24U) |
        (static_cast<std::uint32_t>(bytes[at + 1U]) << 16U) |
        (static_cast<std::uint32_t>(bytes[at + 2U]) << 8U) |
        static_cast<std::uint32_t>(bytes[at + 3U]);
    }
    for (std::size_t index = 16U; index < words.size(); ++index) {
      const std::uint32_t s0 = rotate_right(words[index - 15U], 7U) ^
        rotate_right(words[index - 15U], 18U) ^ (words[index - 15U] >> 3U);
      const std::uint32_t s1 = rotate_right(words[index - 2U], 17U) ^
        rotate_right(words[index - 2U], 19U) ^ (words[index - 2U] >> 10U);
      words[index] = words[index - 16U] + s0 + words[index - 7U] + s1;
    }
    auto working = state;
    for (std::size_t index = 0; index < words.size(); ++index) {
      const std::uint32_t sum1 = rotate_right(working[4], 6U) ^
        rotate_right(working[4], 11U) ^ rotate_right(working[4], 25U);
      const std::uint32_t choice =
        (working[4] & working[5]) ^ ((~working[4]) & working[6]);
      const std::uint32_t temporary1 =
        working[7] + sum1 + choice + constants[index] + words[index];
      const std::uint32_t sum0 = rotate_right(working[0], 2U) ^
        rotate_right(working[0], 13U) ^ rotate_right(working[0], 22U);
      const std::uint32_t majority = (working[0] & working[1]) ^
        (working[0] & working[2]) ^ (working[1] & working[2]);
      const std::uint32_t temporary2 = sum0 + majority;
      for (std::size_t word = 7U; word > 0U; --word) {working[word] = working[word - 1U];}
      working[4] += temporary1;
      working[0] = temporary1 + temporary2;
    }
    for (std::size_t index = 0; index < state.size(); ++index) {
      state[index] += working[index];
    }
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const std::uint32_t word : state) {output << std::setw(8) << word;}
  return output.str();
}

struct VerifiedAsset
{
  StaticAsset specification;
  std::string bytes;
};

class ViewerAssets
{
public:
  explicit ViewerAssets(std::filesystem::path share_directory)
  : share_directory_(std::move(share_directory))
  {
    validate();
  }

  bool ready() const {return ready_;}
  const std::string & reason() const {return reason_;}

  const VerifiedAsset * find(std::string_view route) const
  {
    for (const VerifiedAsset & asset : loaded_) {
      if (asset.specification.served && asset.specification.route == route) {return &asset;}
    }
    return nullptr;
  }

private:
  static const std::array<StaticAsset, 17> & assets()
  {
    static const std::array<StaticAsset, 17> value =
#include "viewer_asset_integrity.inc"
    ;
    return value;
  }

  void validate()
  {
    try {
      constexpr std::uintmax_t kMaximumResidentBytes = 3U * 1024U * 1024U;
      std::uintmax_t total_size = 0U;
      loaded_.reserve(assets().size());
      for (const StaticAsset & asset : assets()) {
        const std::filesystem::path asset_path = share_directory_ / asset.relative;
        if (!std::filesystem::is_regular_file(asset_path)) {
          reason_ = "required viewer asset is missing";
          return;
        }
        const std::uintmax_t size = std::filesystem::file_size(asset_path);
        if (size == 0 || size > asset.maximum_size || size != asset.exact_size ||
          total_size > kMaximumResidentBytes - size)
        {
          reason_ = "required viewer asset has an invalid size";
          return;
        }
        std::ifstream input(asset_path, std::ios::binary);
        std::string bytes{
          std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>{}};
        if (input.bad() || bytes.size() != asset.exact_size || sha256(bytes) != asset.sha256) {
          reason_ = "required viewer asset failed its pinned SHA-256";
          return;
        }
        total_size += size;
        loaded_.push_back({asset, std::move(bytes)});
      }
      ready_ = true;
      reason_ = "viewer assets ready";
    } catch (const std::filesystem::filesystem_error &) {
      reason_ = "required viewer asset cannot be inspected";
    }
  }

  std::filesystem::path share_directory_;
  bool ready_{false};
  std::string reason_;
  std::vector<VerifiedAsset> loaded_;
};
}  // namespace

class PortalNode : public rclcpp::Node
{
public:
  explicit PortalNode(const bool real_mode = false)
  : Node("openarm_portal"), real_mode_(real_mode)
  {
    action_ = rclcpp_action::create_client<Action>(this, "/openarm_ik/move_paired_tcp_scaled");
    if (real_mode_) {
      // Real mode drives a physical arm, so the portal owns no motion path of
      // its own here: it only relays connect/disconnect to the read-only
      // observer and mirrors what the observer reports.
      real_connect_ = create_client<std_srvs::srv::Trigger>("/openarm_real/connect");
      real_disconnect_ = create_client<std_srvs::srv::Trigger>("/openarm_real/disconnect");
      real_status_subscription_ = create_subscription<std_msgs::msg::String>(
        "/openarm_real/status",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::String::SharedPtr message) {
          std::lock_guard<std::mutex> lock(real_mutex_);
          real_status_ = message->data;
        });
    }
    state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {update_state(*message);});
    diagnostic_subscription_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/openarm_ik/diagnostics", rclcpp::QoS(10).reliable(),
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
        update_diagnostics(*message);
      });
  }

  bool real_mode() const {return real_mode_;}

  /// Latest observer status, or a placeholder before the first publication.
  std::string real_status() const
  {
    std::lock_guard<std::mutex> lock(real_mutex_);
    return real_status_.empty() ?
           std::string("{\"connected\":false,\"detail\":\"observer has not reported yet\"}") :
           real_status_;
  }

  /// Relay to the observer's connect or disconnect service. Blocking, because
  /// a sweep of both buses takes hundreds of milliseconds and the operator is
  /// waiting on the answer; this runs on an HTTP worker, not the ROS thread.
  bool real_command(const bool connect, std::string & out_message)
  {
    const auto client = connect ? real_connect_ : real_disconnect_;
    if (!client) {
      out_message = "portal is not in real mode";
      return false;
    }
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      out_message = "the real-arm observer is not running";
      return false;
    }
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(std::chrono::seconds(20)) != std::future_status::ready) {
      out_message = "the observer did not answer within 20 s";
      return false;
    }
    const auto response = future.get();
    out_message = response->message;
    return response->success;
  }

  bool state(GuardInput & input, std::array<Point, 2> & tcp, std::string & reason) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::int64_t time_now = now().nanoseconds();
    const std::int64_t steady_now = steady_now_ns();
    if (!have_state_ || !fresh_at_use(
        state_freshness_, time_now, steady_now,
        std::chrono::duration_cast<std::chrono::nanoseconds>(kStateFreshness).count()) ||
      !diagnostic_valid_ || !fresh_at_use(
        diagnostic_freshness_, time_now, steady_now,
        std::chrono::duration_cast<std::chrono::nanoseconds>(kDiagnosticFreshness).count()))
    {
      reason = diagnostic_reason_.empty() ?
        "encoder-derived joint state or controller diagnostics are missing or stale" : diagnostic_reason_;
      return false;
    }
    input.measured_q = measured_q_;
    input.state_sequence = state_sequence_;
    input.diagnostic_sequence = diagnostic_sequence_;
    input.state_freshness = state_freshness_;
    input.diagnostic_freshness = diagnostic_freshness_;
    for (std::size_t side = 0; side < 2; ++side) {
      oa_fk_result fk{};
      const oa_model * model = side == 0 ? oa_model_left_v10_bimanual() :
        oa_model_right_v10_bimanual();
      if (oa_fk(model, measured_q_[side].data(), &fk) != OA_MODEL_OK) {
        reason = "public FK rejected the freshest measured state";
        return false;
      }
      tcp[side] = {fk.hand_tcp.m[3], fk.hand_tcp.m[7], fk.hand_tcp.m[11]};
    }
    return true;
  }

  std::string state_json() const
  {
    GuardInput input;
    std::array<Point, 2> tcp{};
    std::string reason;
    const bool fresh = state(input, tcp, reason);
    std::lock_guard<std::mutex> lock(mutex_);
    return portal_state_json(
      fresh, goal_active_ || command_gate_.active(), tcp,
      fresh ? "Fresh encoder-derived virtual state; controller collision_checked=false" : reason,
      command_);
  }

  ViewerSnapshot viewer_snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ViewerSnapshot snapshot;
    snapshot.have_state = have_state_;
    snapshot.sequence = state_sequence_;
    snapshot.producer_time_ns = state_freshness_.producer_time_ns;
    snapshot.receipt_steady_ns = state_freshness_.receipt_steady_ns;
    snapshot.fresh = have_state_ && fresh_at_use(
      state_freshness_, now().nanoseconds(), steady_now_ns(),
      std::chrono::duration_cast<std::chrono::nanoseconds>(kStateFreshness).count());
    if (snapshot.have_state) {
      for (std::size_t side = 0; side < measured_q_.size(); ++side) {
        std::copy(measured_q_[side].begin(), measured_q_[side].end(),
          snapshot.position_rad.begin() + side * OA_DOF);
      }
    }
    return snapshot;
  }

  bool begin_guard(std::uint64_t & token, std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || stop_requested != 0) {
      reason = "portal shutdown is in progress";
      return false;
    }
    if (goal_active_ || command_gate_.active()) {
      reason = "another portal command or guard evaluation is active";
      return false;
    }
    if (!command_gate_.begin(token)) {
      reason = "portal command reservation generation is exhausted";
      return false;
    }
    command_ = "Evaluating sampled nominal guard; a software stop invalidates this request.";
    return true;
  }

  void release_guard(std::uint64_t token)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (command_gate_.release(token)) {
      command_ = "Guard evaluation ended without submitting motion.";
    }
  }

  bool move(
    const MoveRequest & request, const GuardInput & input,
    const GuardResult & guard, std::uint64_t guard_token, std::string & reason)
  {
    if (!action_->wait_for_action_server(std::chrono::milliseconds(0))) {
      reason = "MovePairedTcpScaled action server is unavailable";
      return false;
    }
    std::unique_lock<std::mutex> state_lock(mutex_);
    {
      const std::int64_t time_now = now().nanoseconds();
      const std::int64_t steady_now = steady_now_ns();
      if (!command_gate_.valid(guard_token)) {
        reason = "guard evaluation was canceled before action submission";
        return false;
      }
      if (stopping_ || stop_requested != 0 || !guard_handoff_valid(
          input, handoff_evidence_locked(), time_now, steady_now,
          std::chrono::duration_cast<std::chrono::nanoseconds>(kStateFreshness).count(),
          std::chrono::duration_cast<std::chrono::nanoseconds>(kDiagnosticFreshness).count()))
      {
        reason = "measured state changed or became untrusted during guard evaluation; retry";
        return false;
      }
      if (goal_active_) {
        reason = "another portal goal is active";
        return false;
      }
      if (!command_gate_.consume(guard_token)) {
        reason = "guard reservation changed before action submission";
        return false;
      }
      goal_active_ = true;
      cancel_pending_ = false;
      command_ = guard.target_projected ?
        "Best-effort guard projected the request to " +
        number(guard.achieved_fraction * 100.0) +
        "% of its straight-line ray; waiting for controller goal acceptance. Nominal minimum "
        "clearance " + number(guard.minimum_nominal_clearance_m) +
        " m. Controller collision_checked remains false." :
        "Guard accepted the exact request; waiting for controller goal acceptance. Nominal "
        "minimum clearance " + number(guard.minimum_nominal_clearance_m) +
        " m. Controller collision_checked remains false.";
    }
    Action::Goal goal;
    goal.header.stamp = now();
    goal.header.frame_id = "openarm_body_link0";
    goal.left_tcp_m.x = guard.commanded_tcp[0][0];
    goal.left_tcp_m.y = guard.commanded_tcp[0][1];
    goal.left_tcp_m.z = guard.commanded_tcp[0][2];
    goal.right_tcp_m.x = guard.commanded_tcp[1][0];
    goal.right_tcp_m.y = guard.commanded_tcp[1][1];
    goal.right_tcp_m.z = guard.commanded_tcp[1][2];
    goal.motion_limit_scale = request.motion_limit_scale;
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
        bool cancel = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          active_goal_ = handle;
          if (!handle) {
            goal_active_ = false;
            cancel_pending_ = false;
            command_ = "Controller rejected the guarded goal.";
          } else if (cancel_pending_) {
            cancel = true;
            command_ = "Software cancellation sent after controller acceptance; not a hardwired E-stop.";
          } else {
            command_ = "Controller accepted goal; waiting for measured progress.";
          }
        }
        if (cancel && handle) {
          try {
            action_->async_cancel_goal(handle);
          } catch (const std::exception & error) {
            std::lock_guard<std::mutex> lock(mutex_);
            command_ = "Software cancellation request failed: " + std::string(error.what());
          }
        }
      };
    options.feedback_callback = [this](
      GoalHandle::SharedPtr, const std::shared_ptr<const Action::Feedback> feedback)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        command_ = "Measured progress " + number(feedback->measured_progress * 100.0) +
          "% (command " + std::to_string(feedback->command_id) + ").";
      };
    options.result_callback = [this](const GoalHandle::WrappedResult & wrapped) {
        std::lock_guard<std::mutex> lock(mutex_);
        active_goal_.reset();
        goal_active_ = false;
        cancel_pending_ = false;
        if (!wrapped.result) {
          command_ = "Controller returned no result payload.";
          return;
        }
        command_ = "Measured result: " + wrapped.result->reason +
          "; controller collision_checked=" +
          std::string(wrapped.result->collision_checked ? "true" : "false") +
          "; outcome=" + std::to_string(wrapped.result->outcome) + ".";
      };
    const std::int64_t send_time_now = now().nanoseconds();
    const std::int64_t send_steady_now = steady_now_ns();
    if (stopping_ || stop_requested != 0 || !guard_handoff_valid(
        input, handoff_evidence_locked(), send_time_now, send_steady_now,
        std::chrono::duration_cast<std::chrono::nanoseconds>(kStateFreshness).count(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(kDiagnosticFreshness).count()))
    {
      goal_active_ = false;
      reason = "producer state or diagnostics aged out before action send; retry";
      command_ = reason;
      return false;
    }
    try {
      action_->async_send_goal(goal, options);
    } catch (const std::exception & error) {
      goal_active_ = false;
      cancel_pending_ = false;
      active_goal_.reset();
      reason = error.what();
      command_ = "Goal submission failed: " + reason;
      return false;
    }
    state_lock.unlock();
    if (request.dual) {
      reason = "Both arms commanded together to Left [" +
        number(guard.commanded_tcp[0][0]) + ", " + number(guard.commanded_tcp[0][1]) +
        ", " + number(guard.commanded_tcp[0][2]) + "] m and Right [" +
        number(guard.commanded_tcp[1][0]) + ", " + number(guard.commanded_tcp[1][1]) +
        ", " + number(guard.commanded_tcp[1][2]) +
        "] m as one atomic paired command; neither target was projected";
      return true;
    }
    const std::size_t selected = request.side == MoveRequest::Side::left ? 0U : 1U;
    const std::string side_name = selected == 0U ? "Left" : "Right";
    reason = side_name + (guard.target_projected ?
      " target was outside the guarded straight-line workspace and was projected to [" +
      number(guard.commanded_tcp[selected][0]) + ", " +
      number(guard.commanded_tcp[selected][1]) + ", " +
      number(guard.commanded_tcp[selected][2]) + "] m (" +
      number(guard.achieved_fraction * 100.0) + "% of the request); " +
      guard.limiting_reason :
      " exact target submitted") +
      (selected == 0U ? "; Right target is the guarded measured TCP" :
      "; Left target is the guarded measured TCP");
    return true;
  }

  std::string cancel()
  {
    GoalHandle::SharedPtr goal;
    bool guard_canceled = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal = active_goal_;
      guard_canceled = command_gate_.cancel();
      cancel_pending_ = goal_active_;
      command_ = goal ? "Urgent software cancellation requested; not a hardwired E-stop." :
        (goal_active_ ? "Software cancellation queued until the pending goal response; not a hardwired E-stop." :
        (guard_canceled ?
        "In-flight guard evaluation canceled before motion; not a hardwired E-stop." :
        "No portal goal or guard evaluation to cancel. This software button is not a hardwired E-stop."));
    }
    if (goal) {
      try {
        action_->async_cancel_goal(goal);
      } catch (const std::exception & error) {
        std::lock_guard<std::mutex> lock(mutex_);
        command_ = "Software cancellation request failed: " + std::string(error.what());
      }
    }
    return command();
  }

  // The emergency stop is a process-wide lock-free latch. It takes no handle
  // and no lock, so it is honoured even while a command holds the controller.
  std::string engage_estop()
  {
    oa_runtime_estop_assert();
    (void)cancel();
    return std::string("{\"estop\":\"engaged\",\"asserted\":") +
      (oa_runtime_estop_asserted() != 0U ? "true" : "false") +
      ",\"assertions\":" + std::to_string(oa_runtime_estop_assert_count()) +
      ",\"note\":\"software interlock; not a hardwired safety-rated E-stop\"}";
  }

  std::string release_estop()
  {
    const bool cleared = oa_runtime_estop_clear() == OA_RUNTIME_OK;
    return std::string("{\"estop\":\"") + (cleared ? "released" : "release_failed") +
      "\",\"asserted\":" + (oa_runtime_estop_asserted() != 0U ? "true" : "false") + "}";
  }

  std::string verify_simulation() const
  {
    GuardInput input;
    std::array<Point, 2> tcp{};
    std::string reason;
    if (!state(input, tcp, reason)) {
      return "Simulation verification failed without motion: " + reason;
    }
    return "Simulation verified; no physical calibration was performed. Controller collision_checked=false.";
  }

  std::string command() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return command_;
  }

  bool goal_active() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return goal_active_ || command_gate_.active();
  }

  void begin_shutdown()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    (void)cancel();
  }

private:
  GuardHandoffEvidence handoff_evidence_locked() const
  {
    return {
      measured_q_, state_sequence_, diagnostic_sequence_, state_freshness_,
      diagnostic_freshness_, have_state_, diagnostic_valid_};
  }

  void update_state(const sensor_msgs::msg::JointState & message)
  {
    const FreshnessEvidence freshness{
      rclcpp::Time(message.header.stamp).nanoseconds(), steady_now_ns()};
    if (!fresh_at_use(
        freshness, now().nanoseconds(), freshness.receipt_steady_ns,
        std::chrono::duration_cast<std::chrono::nanoseconds>(kStateFreshness).count()))
    {
      return;
    }
    std::array<JointVector, 2> next{};
    if (!map_canonical_joint_state(message.name, message.position, next)) {return;}
    std::lock_guard<std::mutex> lock(mutex_);
    measured_q_ = next;
    state_freshness_ = freshness;
    have_state_ = true;
    ++state_sequence_;
  }

  void update_diagnostics(const diagnostic_msgs::msg::DiagnosticArray & message)
  {
    const FreshnessEvidence freshness{
      rclcpp::Time(message.header.stamp).nanoseconds(), steady_now_ns()};
    const bool producer_fresh = fresh_at_use(
      freshness, now().nanoseconds(), freshness.receipt_steady_ns,
      std::chrono::duration_cast<std::chrono::nanoseconds>(kDiagnosticFreshness).count());
    for (const auto & status : message.status) {
      if (status.name != "openarm_ik_ros/virtual_control") {continue;}
      auto value = [&status](const std::string & key) {
          for (const auto & item : status.values) {
            if (item.key == key) {return item.value;}
          }
          return std::string{};
        };
      const std::string left_expected = value("left_expected_mask");
      const std::string right_expected = value("right_expected_mask");
      const bool masks = left_expected == "127" && right_expected == "127" &&
        left_expected == value("left_fresh_mask") && right_expected == value("right_fresh_mask") &&
        value("left_fault_mask") == "0" && value("right_fault_mask") == "0";
      const bool valid = producer_fresh &&
        status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN &&
        value("backend") == "virtual" && value("physical_motion_authorized") == "false" &&
        value("collision_checked") == "false" && value("adapter_state") == "idle" &&
        value("lifecycle") == "4" && value("executing") == "false" && masks;
      std::lock_guard<std::mutex> lock(mutex_);
      diagnostic_valid_ = valid;
      diagnostic_reason_ = valid ? std::string{} :
        "controller diagnostics are delayed, replayed, faulted, or inconsistent";
      diagnostic_freshness_ = freshness;
      ++diagnostic_sequence_;
      return;
    }
  }

  mutable std::mutex mutex_;
  std::array<JointVector, 2> measured_q_{};
  FreshnessEvidence state_freshness_{};
  bool have_state_{false};
  std::uint64_t state_sequence_{0};
  std::uint64_t diagnostic_sequence_{0};
  bool diagnostic_valid_{false};
  std::string diagnostic_reason_{"controller diagnostics have not arrived"};
  FreshnessEvidence diagnostic_freshness_{};
  bool goal_active_{false};
  bool stopping_{false};
  bool cancel_pending_{false};
  CommandReservationGate command_gate_;
  std::string command_{"No portal command."};
  GoalHandle::SharedPtr active_goal_;
  rclcpp_action::Client<Action>::SharedPtr action_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  const bool real_mode_{false};
  mutable std::mutex real_mutex_;
  std::string real_status_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr real_connect_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr real_disconnect_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr real_status_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_subscription_;
};

class GuardReservation
{
public:
  GuardReservation(std::shared_ptr<PortalNode> node, std::uint64_t token)
  : node_(std::move(node)), token_(token)
  {
  }

  GuardReservation(const GuardReservation &) = delete;
  GuardReservation & operator=(const GuardReservation &) = delete;
  ~GuardReservation() {node_->release_guard(token_);}

private:
  std::shared_ptr<PortalNode> node_;
  std::uint64_t token_;
};

class PortalServer
{
private:
  using Request = http::request<http::string_body>;
  static constexpr unsigned kMaximumIntake = 16U;
  static constexpr auto kIntakeDeadline = std::chrono::milliseconds(500);
  static constexpr unsigned kStaticWorkers = 2U;
  static constexpr unsigned kStaticPending = 1U;
  static constexpr unsigned kMaximumApi = 8U;
  static constexpr unsigned kMaximumUrgent = 4U;
  // MJPEG viewers are long-lived, so they get their own small lane. Capping
  // them keeps a browser reload storm from starving the API and stop lanes.
  static constexpr unsigned kMaximumStream = 3U;
  static constexpr int kStreamQuality = 68;
  // Headroom above the 30 fps target rather than exactly 30: capture and encode
  // take a few milliseconds that vary with scene complexity, and pacing at
  // exactly 33 ms measured 28.5 fps while the arms were moving. The cap is a
  // ceiling, not a promise; the stream cannot outrun what rviz2 renders, which
  // is why the portal asks for GPU rendering.
  static constexpr auto kStreamInterval = std::chrono::milliseconds(28);

  class IntakeSession : public std::enable_shared_from_this<IntakeSession>
  {
  public:
    IntakeSession(PortalServer & server, tcp::socket socket)
    : server_(server), stream_(std::move(socket))
    {
      parser_.header_limit(8192);
      parser_.body_limit(512);
    }

    void start()
    {
      stream_.expires_after(kIntakeDeadline);
      const auto self = shared_from_this();
      http::async_read(stream_, buffer_, parser_,
        [self](beast::error_code error, std::size_t) {
          self->complete(error);
        });
    }

    void cancel()
    {
      beast::error_code ignored;
      stream_.socket().cancel(ignored);
      stream_.socket().shutdown(tcp::socket::shutdown_both, ignored);
      stream_.socket().close(ignored);
    }

  private:
    void complete(beast::error_code error)
    {
      if (error) {
        server_.finish_intake(shared_from_this());
        return;
      }
      server_.complete_intake(
        shared_from_this(), std::move(stream_), parser_.release());
    }

    PortalServer & server_;
    beast::tcp_stream stream_;
    beast::flat_buffer buffer_;
    http::request_parser<http::string_body> parser_;
  };

public:
  PortalServer(unsigned short port, std::shared_ptr<PortalNode> node)
  : context_(1), acceptor_(context_), node_(std::move(node)), csrf_(token()),
    authority_("127.0.0.1:" + std::to_string(port)), policy_(authority_, csrf_),
    safe_policy_(authority_), assets_(installed_share_directory()), page_(portal_page(csrf_))
  {
    const tcp::endpoint endpoint{asio::ip::make_address_v4("127.0.0.1"), port};
    acceptor_.open(endpoint.protocol());
    acceptor_.set_option(asio::socket_base::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen(16);
    acceptor_.non_blocking(true);
  }

  void run()
  {
    while (stop_requested == 0) {
      context_.restart();
      context_.poll();
      tcp::socket socket(context_);
      beast::error_code error;
      acceptor_.accept(socket, error);
      if (error == asio::error::would_block || error == asio::error::try_again) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      if (error) {
        if (stop_requested == 0) {
          std::fprintf(stderr, "openarm_portal accept: %s\n", error.message().c_str());
        }
        continue;
      }
      start_intake(std::move(socket));
    }
    beast::error_code ignored;
    acceptor_.close(ignored);
    cancel_intakes();
    node_->begin_shutdown();
    // Streams never finish on their own, so they must be told to stop before
    // the pool is joined or shutdown would hang here forever.
    stream_stopping_.store(true);
    static_pool_.join();
    urgent_pool_.join();
    api_pool_.join();
    stream_pool_.join();
  }

private:
  static void close_stream(beast::tcp_stream & stream)
  {
    beast::error_code ignored;
    stream.socket().shutdown(tcp::socket::shutdown_both, ignored);
    stream.socket().close(ignored);
  }

  static void reject_stream(beast::tcp_stream & stream)
  {
    beast::error_code ignored;
    stream.socket().set_option(asio::socket_base::linger(true, 0), ignored);
    stream.socket().close(ignored);
  }

  bool write_asset(
    beast::tcp_stream & stream, const Request & request, const VerifiedAsset & asset)
  {
    http::response<http::string_body> response{http::status::ok, request.version()};
    response.set(http::field::content_type, asset.specification.content_type);
    response.body() = asset.bytes;
    write_response(stream, response);
    return true;
  }

  void dispatch_static(beast::tcp_stream stream, Request request)
  {
    if (static_admitted_.fetch_add(1) >= kStaticWorkers + kStaticPending) {
      static_admitted_.fetch_sub(1);
      reject_stream(stream);
      return;
    }
    try {
      asio::post(static_pool_,
        [this, stream = std::move(stream), request = std::move(request)]() mutable {
          try {
            stream.expires_after(std::chrono::seconds(1));
            beast::error_code ignored;
            stream.socket().set_option(asio::socket_base::send_buffer_size(16384), ignored);
            serve_static(stream, request);
            close_stream(stream);
          } catch (const std::exception & exception) {
            std::fprintf(stderr, "openarm_portal static request: %s\n", exception.what());
          }
          static_admitted_.fetch_sub(1);
        });
    } catch (...) {
      static_admitted_.fetch_sub(1);
      throw;
    }
  }

  void dispatch_api(beast::tcp_stream stream, Request request)
  {
    if (api_admitted_.fetch_add(1) >= kMaximumApi) {
      api_admitted_.fetch_sub(1);
      reject_stream(stream);
      return;
    }
    try {
      asio::post(api_pool_,
        [this, stream = std::move(stream), request = std::move(request)]() mutable {
          try {
            stream.expires_after(std::chrono::seconds(2));
            serve_api(stream, request);
            close_stream(stream);
          } catch (const std::exception & exception) {
            std::fprintf(stderr, "openarm_portal API request: %s\n", exception.what());
          }
          api_admitted_.fetch_sub(1);
        });
    } catch (...) {
      api_admitted_.fetch_sub(1);
      throw;
    }
  }

  void dispatch_urgent(beast::tcp_stream stream, Request request)
  {
    if (urgent_admitted_.fetch_add(1) >= kMaximumUrgent) {
      urgent_admitted_.fetch_sub(1);
      reject_stream(stream);
      return;
    }
    try {
      asio::post(urgent_pool_,
        [this, stream = std::move(stream), request = std::move(request)]() mutable {
          try {
            stream.expires_after(std::chrono::milliseconds(500));
            serve_api(stream, request);
            close_stream(stream);
          } catch (const std::exception & exception) {
            std::fprintf(stderr, "openarm_portal urgent request: %s\n", exception.what());
          }
          urgent_admitted_.fetch_sub(1);
        });
    } catch (...) {
      urgent_admitted_.fetch_sub(1);
      throw;
    }
  }

  // A long-lived multipart/x-mixed-replace response carrying the live rviz2
  // window. Deliberately not routed through dispatch_api: that lane sets a
  // two-second deadline, which is correct for one-shot requests and fatal for
  // a stream.
  void dispatch_stream(beast::tcp_stream stream, Request request)
  {
    if (stream_admitted_.fetch_add(1) >= kMaximumStream) {
      stream_admitted_.fetch_sub(1);
      write_json(stream, http::status::service_unavailable,
        "{\"error\":\"too many viewers\"}");
      close_stream(stream);
      return;
    }
    try {
      asio::post(stream_pool_,
        [this, stream = std::move(stream), request = std::move(request)]() mutable {
          try {
            serve_stream(stream, request);
          } catch (const std::exception & exception) {
            std::fprintf(stderr, "openarm_portal stream: %s\n", exception.what());
          }
          close_stream(stream);
          stream_admitted_.fetch_sub(1);
        });
    } catch (...) {
      stream_admitted_.fetch_sub(1);
      throw;
    }
  }

  void serve_stream(beast::tcp_stream & stream, const Request & request)
  {
    static constexpr const char * kBoundary = "openarmframe";
    // No read/write deadline: the response never completes by design.
    stream.expires_never();
    std::string header =
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=" + std::string(kBoundary) +
      "\r\nCache-Control: no-store, no-cache, must-revalidate\r\n"
      "Pragma: no-cache\r\nConnection: close\r\n\r\n";
    beast::error_code error;
    asio::write(stream.socket(), asio::buffer(header), error);
    if (error) {return;}
    (void)request;

    std::vector<unsigned char> frame;
    auto next = std::chrono::steady_clock::now();
    while (!stream_stopping_.load()) {
      {
        // One capture at a time: the X connection is not shared safely and a
        // single encode serves every viewer.
        std::lock_guard<std::mutex> lock(capture_mutex_);
        if (capture_.connect() && capture_.locate()) {
          std::vector<unsigned char> encoded;
          if (capture_.grab(encoded, kStreamQuality)) {
            frame = std::move(encoded);
          }
        }
      }
      if (!frame.empty()) {
        std::string part = "--" + std::string(kBoundary) +
          "\r\nContent-Type: image/jpeg\r\nContent-Length: " +
          std::to_string(frame.size()) + "\r\n\r\n";
        asio::write(stream.socket(), asio::buffer(part), error);
        if (error) {return;}
        asio::write(stream.socket(), asio::buffer(frame), error);
        if (error) {return;}
        asio::write(stream.socket(), asio::buffer(std::string("\r\n")), error);
        if (error) {return;}
      }
      next += kStreamInterval;
      // If a frame overran its slot, resynchronise instead of trying to catch
      // up: an accumulated deficit would otherwise spin the loop flat out.
      const auto now = std::chrono::steady_clock::now();
      if (next < now) {
        next = now;
      } else {
        std::this_thread::sleep_until(next);
      }
    }
  }

  void start_intake(tcp::socket socket)
  {
    if (active_intakes_.size() >= kMaximumIntake) {
      const auto oldest = active_intakes_.front();
      active_intakes_.pop_front();
      oldest->cancel();
    }
    auto session = std::make_shared<IntakeSession>(*this, std::move(socket));
    active_intakes_.push_back(session);
    session->start();
  }

  void finish_intake(const std::shared_ptr<IntakeSession> & session)
  {
    const auto position = std::find(active_intakes_.begin(), active_intakes_.end(), session);
    if (position != active_intakes_.end()) {
      active_intakes_.erase(position);
    }
  }

  void complete_intake(
    const std::shared_ptr<IntakeSession> & session, beast::tcp_stream stream, Request request)
  {
    finish_intake(session);
    const std::string target(request.target());
    std::string reason;
    const SafeRequestHeaders request_headers = safe_headers(request);
    if (!safe_policy_.validate_read(request_headers, reason)) {
      write_json(stream, http::status::forbidden,
        "{\"error\":\"" + json_escape(reason) + "\"}");
      close_stream(stream);
      return;
    }
    if (request.method() == http::verb::get && !request.body().empty()) {
      write_json(stream, http::status::bad_request, "{\"error\":\"GET body is not allowed\"}");
      close_stream(stream);
      return;
    }
    const bool static_route = request.method() == http::verb::get &&
      (target == "/" || (assets_.ready() && assets_.find(target) != nullptr));
    if (static_route) {
      dispatch_static(std::move(stream), std::move(request));
    } else if (request.method() == http::verb::get && target == "/api/rviz/stream") {
      dispatch_stream(std::move(stream), std::move(request));
    } else if (request.method() == http::verb::post &&
      (target == "/api/estop" || target == "/api/estop/release")) {
      dispatch_urgent(std::move(stream), std::move(request));
    } else if (request.method() == http::verb::post && target == "/api/stop") {
      // Keep authenticated software-stop handling independent of expensive IK
      // work and ordinary API admission.  Validation remains in serve_api.
      dispatch_urgent(std::move(stream), std::move(request));
    } else {
      dispatch_api(std::move(stream), std::move(request));
    }
  }

  void cancel_intakes()
  {
    const auto intakes = std::move(active_intakes_);
    active_intakes_.clear();
    for (const auto & intake : intakes) {
      intake->cancel();
    }
    context_.restart();
    context_.poll();
    context_.stop();
  }

  void serve_static(beast::tcp_stream & stream, const Request & request)
  {
    const std::string target(request.target());
    if (target == "/") {
      http::response<http::string_body> response{http::status::ok, request.version()};
      response.set(http::field::content_type, "text/html; charset=utf-8");
      response.body() = page_;
      write_response(stream, response);
      return;
    }
    if (const VerifiedAsset * asset = assets_.find(target)) {
        (void)write_asset(stream, request, *asset);
        return;
    }
    write_json(stream, http::status::not_found, "{\"error\":\"route not found\"}");
  }

  void serve_api(beast::tcp_stream & stream, const Request & request)
  {
    const std::string target(request.target());
    std::string reason;
    const SafeRequestHeaders request_headers = safe_headers(request);
    if (request.method() == http::verb::get && target == "/api/health") {
      write_json(stream, assets_.ready() ? http::status::ok : http::status::service_unavailable,
        std::string("{\"healthy\":") + (assets_.ready() ? "true" : "false") +
        ",\"bind\":\"127.0.0.1\",\"viewer_assets_ready\":" +
        (assets_.ready() ? "true" : "false") + ",\"reason\":\"" +
        json_escape(assets_.reason()) + "\"}");
      return;
    }
    if (request.method() == http::verb::get && target == "/api/state") {
      write_json(stream, http::status::ok, node_->state_json());
      return;
    }
    if (request.method() == http::verb::get && target == "/api/real/status") {
      // Always answers, even outside real mode, so the page can discover which
      // mode the portal is in with one request and render accordingly.
      write_json(stream, http::status::ok,
        std::string("{\"enabled\":") + (node_->real_mode() ? "true" : "false") +
        ",\"observer\":" + (node_->real_mode() ? node_->real_status() : "null") + "}");
      return;
    }
    if (request.method() == http::verb::get && target == "/api/view-state") {
      write_json(stream, http::status::ok, viewer_state_json(node_->viewer_snapshot(), steady_now_ns()));
      return;
    }
    if (request.method() != http::verb::post ||
      (target != "/api/move" && target != "/api/v2/move" && target != "/api/v3/move" &&
      target != "/api/stop" && target != "/api/verify" && target != "/api/estop" &&
      target != "/api/estop/release" && target != "/api/v3/move-both" &&
      target != "/api/real/connect" && target != "/api/real/disconnect"))
    {
      write_json(stream, http::status::not_found, "{\"error\":\"route not found\"}");
      return;
    }
    if (!safe_policy_.validate_mutation(request_headers, reason)) {
      write_json(stream, http::status::forbidden,
        "{\"error\":\"" + json_escape(reason) + "\"}");
      return;
    }
    std::string_view csrf;
    std::string_view content_type;
    if (!unique_header(request, "X-CSRF-Token", csrf) ||
      !unique_header(request, "Content-Type", content_type))
    {
      write_json(stream, http::status::forbidden,
        "{\"error\":\"exact CSRF and Content-Type headers are required\"}");
      return;
    }
    MutationHeaders headers{
      request_headers.host, request_headers.origin, csrf, content_type, request.body().size()};
    if (stop_requested != 0) {
      write_json(stream, http::status::service_unavailable,
        "{\"error\":\"portal shutdown is in progress\"}");
      return;
    }
    if (!policy_.validate(headers, reason)) {
      write_json(stream, http::status::forbidden,
        "{\"error\":\"" + json_escape(reason) + "\"}");
      return;
    }
    if (target == "/api/real/connect" || target == "/api/real/disconnect") {
      if (!node_->real_mode()) {
        write_json(stream, http::status::not_found,
          "{\"error\":\"portal is not in real-arm mode\"}");
        return;
      }
      std::string message;
      const bool ok = node_->real_command(target == "/api/real/connect", message);
      // On failure the page's shared post() helper reads "error", so emit both
      // rather than leaving it to report a generic "request rejected".
      const std::string escaped = json_escape(message);
      write_json(stream, ok ? http::status::ok : http::status::conflict,
        ok ? "{\"ok\":true,\"message\":\"" + escaped + "\"}" :
        "{\"ok\":false,\"message\":\"" + escaped + "\",\"error\":\"" + escaped + "\"}");
      return;
    }
    if (target == "/api/estop" || target == "/api/estop/release") {
      if (!StrictJson::empty_object(request.body())) {
        write_json(stream, http::status::bad_request,
          "{\"error\":\"body must be an empty JSON object\"}");
        return;
      }
      write_json(stream, http::status::ok,
        target == "/api/estop" ? node_->engage_estop() : node_->release_estop());
      return;
    }
    // Refuse motion at the boundary while the stop is latched. The controller
    // would reject it anyway, but accepting a request that cannot move is
    // misleading, and the operator should be told the stop is the reason.
    if (oa_runtime_estop_asserted() != 0U && target != "/api/stop") {
      write_json(stream, http::status::service_unavailable,
        "{\"error\":\"emergency stop is engaged; release it before commanding motion\"}");
      return;
    }
    if (target == "/api/stop" || target == "/api/verify") {
      if (!StrictJson::empty_object(request.body())) {
        write_json(stream, http::status::bad_request, "{\"error\":\"exact empty JSON object required\"}");
        return;
      }
      const std::string message = target == "/api/stop" ? node_->cancel() : node_->verify_simulation();
      write_json(stream, http::status::ok,
        "{\"message\":\"" + json_escape(message) + "\"}");
      return;
    }
    MoveRequest move;
    if (target == "/api/v3/move-both") {
      // Both arms to their own targets, in one atomic paired command.
      UnitMoveRequest unit_move;
      Point right_target{};
      if (!StrictJson::parse_move_both(request.body(), unit_move, right_target, reason) ||
        !normalise_move_to_metres(unit_move, move, reason))
      {
        write_json(stream, http::status::bad_request,
          "{\"error\":\"" + json_escape(reason) + "\"}");
        return;
      }
      // normalise_move_to_metres converted the left triple; convert the right
      // one through the same path so both share exactly one unit conversion.
      UnitMoveRequest right_unit = unit_move;
      right_unit.target.x = right_target[0];
      right_unit.target.y = right_target[1];
      right_unit.target.z = right_target[2];
      MoveRequest right_move;
      if (!normalise_move_to_metres(right_unit, right_move, reason)) {
        write_json(stream, http::status::bad_request,
          "{\"error\":\"" + json_escape(reason) + "\"}");
        return;
      }
      move.dual = true;
      move.dual_target[0] = move.target;
      move.dual_target[1] = right_move.target;
    } else if (target == "/api/v2/move" || target == "/api/v3/move") {
      UnitMoveRequest unit_move;
      const bool parsed = target == "/api/v3/move" ?
        StrictJson::parse_move_v3(request.body(), unit_move, reason) :
        StrictJson::parse_move_v2(request.body(), unit_move, reason);
      if (!parsed ||
        !normalise_move_to_metres(unit_move, move, reason))
      {
        write_json(stream, http::status::bad_request,
          "{\"error\":\"" + json_escape(reason) + "\"}");
        return;
      }
    } else {
      if (!StrictJson::parse_move(request.body(), move, reason)) {
        write_json(stream, http::status::bad_request,
          "{\"error\":\"" + json_escape(reason) + "\"}");
        return;
      }
    }
    std::uint64_t guard_token = 0U;
    if (!node_->begin_guard(guard_token, reason)) {
      write_json(stream, http::status::conflict,
        "{\"error\":\"" + json_escape(reason) + "\"}");
      return;
    }
    GuardReservation reservation(node_, guard_token);
    GuardInput input;
    std::array<Point, 2> tcp{};
    if (!node_->state(input, tcp, reason)) {
      write_json(stream, http::status::conflict,
        "{\"error\":\"" + json_escape(reason) + "\"}");
      return;
    }
    input.request = move;
    const GuardResult guarded = guard_.validate_or_project(input);
    if (!guarded.accepted) {
      write_json(stream, http::status::unprocessable_entity,
        "{\"error\":\"virtual nominal guard rejected: " + json_escape(guarded.reason) + "\"}");
      return;
    }
    if (!node_->move(move, input, guarded, guard_token, reason)) {
      write_json(stream, http::status::conflict,
        "{\"error\":\"" + json_escape(reason) + "\"}");
      return;
    }
    if (move.dual) {
      write_json(stream, http::status::accepted,
        "{\"message\":\"" + json_escape(reason +
        "; sampled nominal virtual protection only; controller collision_checked=false") +
        "\",\"projected\":false,\"achieved_fraction\":1" +
        ",\"motion_limit_scale\":" + json_number(move.motion_limit_scale) +
        ",\"left_commanded_m\":[" + json_number(guarded.commanded_tcp[0][0]) + "," +
        json_number(guarded.commanded_tcp[0][1]) + "," +
        json_number(guarded.commanded_tcp[0][2]) +
        "],\"right_commanded_m\":[" + json_number(guarded.commanded_tcp[1][0]) + "," +
        json_number(guarded.commanded_tcp[1][1]) + "," +
        json_number(guarded.commanded_tcp[1][2]) + "]}");
      return;
    }
    const std::size_t selected = move.side == MoveRequest::Side::left ? 0U : 1U;
    write_json(stream, http::status::accepted,
      "{\"message\":\"" + json_escape(reason +
      "; sampled nominal virtual protection only; controller collision_checked=false") +
      "\",\"projected\":" + (guarded.target_projected ? "true" : "false") +
      ",\"achieved_fraction\":" + json_number(guarded.achieved_fraction) +
      ",\"motion_limit_scale\":" + json_number(move.motion_limit_scale) +
      ",\"requested_m\":[" + json_number(guarded.requested_tcp[0]) + "," +
      json_number(guarded.requested_tcp[1]) + "," + json_number(guarded.requested_tcp[2]) +
      "],\"commanded_m\":[" + json_number(guarded.commanded_tcp[selected][0]) + "," +
      json_number(guarded.commanded_tcp[selected][1]) + "," +
      json_number(guarded.commanded_tcp[selected][2]) + "]}");
  }

  asio::io_context context_;
  tcp::acceptor acceptor_;
  asio::thread_pool static_pool_{kStaticWorkers};
  asio::thread_pool api_pool_{4};
  asio::thread_pool urgent_pool_{1};
  asio::thread_pool stream_pool_{kMaximumStream};
  std::atomic_uint static_admitted_{0};
  std::atomic_uint api_admitted_{0};
  std::atomic_uint urgent_admitted_{0};
  std::atomic_uint stream_admitted_{0};
  std::atomic_bool stream_stopping_{false};
  RvizCapture capture_;
  std::mutex capture_mutex_;
  std::deque<std::shared_ptr<IntakeSession>> active_intakes_;
  std::shared_ptr<PortalNode> node_;
  std::string csrf_;
  std::string authority_;
  MutationPolicy policy_;
  SafeRequestPolicy safe_policy_;
  ViewerAssets assets_;
  std::string page_;
  NominalPathGuard guard_;
};

}  // namespace openarm_ik_ros::portal

namespace
{
std::uint64_t unsigned_number(const char * value, const char * name)
{
  try {
    std::size_t used = 0;
    const std::string text(value);
    const std::uint64_t parsed = std::stoull(text, &used, 10);
    if (used != text.size() || parsed == 0) {
      throw std::invalid_argument(name);
    }
    return parsed;
  } catch (const std::exception &) {
    throw std::invalid_argument(std::string("invalid ") + name);
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc < 3 || argc > 4 || std::string(argv[1]) != "--port" ||
      (argc == 4 && std::string(argv[3]) != "--real"))
    {
      std::fprintf(stderr, "Usage: openarm_portal --port PORT [--real]\n");
      return 2;
    }
    const bool real_mode = argc == 4;
    const std::uint64_t port_value = unsigned_number(argv[2], "port");
    if (port_value < 1024 || port_value > 65535)
    {
      throw std::invalid_argument("port outside allowed range");
    }
    rclcpp::init(argc, argv);
    std::signal(SIGINT, openarm_ik_ros::portal::signal_handler);
    std::signal(SIGTERM, openarm_ik_ros::portal::signal_handler);
    auto node = std::make_shared<openarm_ik_ros::portal::PortalNode>(real_mode);
    std::thread ros_thread([node]() {rclcpp::spin(node);});
    try {
      openarm_ik_ros::portal::PortalServer server(static_cast<unsigned short>(port_value), node);
      server.run();
      (void)node->cancel();
      const auto cancel_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      while (node->goal_active() && std::chrono::steady_clock::now() < cancel_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      if (node->goal_active()) {
        std::fprintf(stderr, "openarm_portal: software stop state unconfirmed during shutdown\n");
      }
    } catch (...) {
      rclcpp::shutdown();
      ros_thread.join();
      throw;
    }
    rclcpp::shutdown();
    ros_thread.join();
    return 0;
  } catch (const std::exception & error) {
    std::fprintf(stderr, "openarm_portal: %s\n", error.what());
    return 1;
  }
}
