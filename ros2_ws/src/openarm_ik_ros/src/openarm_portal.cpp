// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/portal_core.hpp"

#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/thread_pool.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <openarm_control_msgs/action/move_paired_tcp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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
#include <filesystem>
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
using Action = openarm_control_msgs::action::MovePairedTcp;
using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
static_assert(std::is_same_v<Point::value_type, double>);
static_assert(std::is_same_v<decltype(Action::Goal{}.left_tcp_m.x), double>);
static_assert(std::is_same_v<decltype(Action::Goal{}.right_tcp_m.z), double>);
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

  const StaticAsset * find(std::string_view route) const
  {
    for (const StaticAsset & asset : assets()) {
      if (asset.route == route) {return &asset;}
    }
    return nullptr;
  }

  const std::filesystem::path path(const StaticAsset & asset) const
  {
    return share_directory_ / asset.relative;
  }

private:
  static const std::array<StaticAsset, 16> & assets()
  {
    static const std::array<StaticAsset, 16> value{{
      {"/web/portal.css", "web/portal.css", "text/css; charset=utf-8", 65536, 0},
      {"/web/portal.js", "web/portal.js", "application/javascript; charset=utf-8", 65536, 0},
      {"/web/viewer.js", "web/viewer.js", "application/javascript; charset=utf-8", 131072, 0},
      {"/viewer/manifest.json", "viewer/manifest.json", "application/json; charset=utf-8", 16384, 0},
      {"/viewer/stage_a.urdf", "viewer/stage_a.urdf", "application/xml; charset=utf-8", 131072, 0},
      {"/viewer/mesh/body_link0_symp.stl", "viewer/mesh/body_link0_symp.stl", "model/stl", 293284, 293284},
      {"/viewer/mesh/link0_symp.stl", "viewer/mesh/link0_symp.stl", "model/stl", 40284, 40284},
      {"/viewer/mesh/link1_symp.stl", "viewer/mesh/link1_symp.stl", "model/stl", 17784, 17784},
      {"/viewer/mesh/link2_symp.stl", "viewer/mesh/link2_symp.stl", "model/stl", 13384, 13384},
      {"/viewer/mesh/link3_symp.stl", "viewer/mesh/link3_symp.stl", "model/stl", 156984, 156984},
      {"/viewer/mesh/link4_symp.stl", "viewer/mesh/link4_symp.stl", "model/stl", 1139984, 1139984},
      {"/viewer/mesh/link5_symp.stl", "viewer/mesh/link5_symp.stl", "model/stl", 751484, 751484},
      {"/viewer/mesh/link6_symp.stl", "viewer/mesh/link6_symp.stl", "model/stl", 30084, 30084},
      {"/viewer/mesh/link7_symp.stl", "viewer/mesh/link7_symp.stl", "model/stl", 23884, 23884},
      {"/viewer/mesh/hand.stl", "viewer/mesh/hand.stl", "model/stl", 18284, 18284},
      {"/viewer/mesh/finger.stl", "viewer/mesh/finger.stl", "model/stl", 13284, 13284},
    }};
    return value;
  }

  void validate()
  {
    try {
      for (const StaticAsset & asset : assets()) {
        const std::filesystem::path asset_path = path(asset);
        if (!std::filesystem::is_regular_file(asset_path)) {
          reason_ = "required viewer asset is missing";
          return;
        }
        const std::uintmax_t size = std::filesystem::file_size(asset_path);
        if (size == 0 || size > asset.maximum_size ||
          (asset.exact_size != 0 && size != asset.exact_size))
        {
          reason_ = "required viewer asset has an invalid size";
          return;
        }
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
};
}  // namespace

class PortalNode : public rclcpp::Node
{
public:
  PortalNode()
  : Node("openarm_portal")
  {
    action_ = rclcpp_action::create_client<Action>(this, "/openarm_ik/move_paired_tcp");
    state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {update_state(*message);});
    diagnostic_subscription_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/openarm_ik/diagnostics", rclcpp::QoS(10).reliable(),
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
        update_diagnostics(*message);
      });
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
      fresh, goal_active_, tcp,
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

  bool move(
    const MoveRequest & request, const GuardInput & input,
    const GuardResult & guard, std::string & reason)
  {
    if (!action_->wait_for_action_server(std::chrono::milliseconds(0))) {
      reason = "MovePairedTcp action server is unavailable";
      return false;
    }
    std::unique_lock<std::mutex> state_lock(mutex_);
    {
      const std::int64_t time_now = now().nanoseconds();
      const std::int64_t steady_now = steady_now_ns();
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
      goal_active_ = true;
      cancel_pending_ = false;
      command_ = "Guard accepted; waiting for controller goal acceptance. Nominal minimum clearance " +
        number(guard.minimum_nominal_clearance_m) + " m. Controller collision_checked remains false.";
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
    reason = request.side == MoveRequest::Side::left ?
      "Left target submitted with Right target set to the guarded measured TCP" :
      "Right target submitted with Left target set to the guarded measured TCP";
    return true;
  }

  std::string cancel()
  {
    GoalHandle::SharedPtr goal;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal = active_goal_;
      cancel_pending_ = goal_active_;
      command_ = goal ? "Urgent software cancellation requested; not a hardwired E-stop." :
        (goal_active_ ? "Software cancellation queued until the pending goal response; not a hardwired E-stop." :
        "No portal goal to cancel. This software button is not a hardwired E-stop.");
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
    return goal_active_;
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
    if (message.name.size() != message.position.size()) {
      return;
    }
    const FreshnessEvidence freshness{
      rclcpp::Time(message.header.stamp).nanoseconds(), steady_now_ns()};
    if (!fresh_at_use(
        freshness, now().nanoseconds(), freshness.receipt_steady_ns,
        std::chrono::duration_cast<std::chrono::nanoseconds>(kStateFreshness).count()))
    {
      return;
    }
    std::array<JointVector, 2> next{};
    for (std::size_t side = 0; side < 2; ++side) {
      const oa_model * model = side == 0 ? oa_model_left_v10_bimanual() :
        oa_model_right_v10_bimanual();
      for (std::size_t joint = 0; joint < OA_DOF; ++joint) {
        const std::string expected = oa_model_joint_name(model, joint);
        const auto found = std::find(message.name.begin(), message.name.end(), expected);
        if (found == message.name.end()) {
          return;
        }
        const std::size_t index = static_cast<std::size_t>(found - message.name.begin());
        if (!std::isfinite(message.position[index])) {
          return;
        }
        next[side][joint] = message.position[index];
      }
    }
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
  std::string command_{"No portal command."};
  GoalHandle::SharedPtr active_goal_;
  rclcpp_action::Client<Action>::SharedPtr action_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_subscription_;
};

class PortalServer
{
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
      tcp::socket socket(context_);
      beast::error_code error;
      acceptor_.accept(socket, error);
      if (error == asio::error::would_block || error == asio::error::try_again) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }
      if (error) {
        if (stop_requested == 0) {
          std::fprintf(stderr, "openarm_portal accept: %s\n", error.message().c_str());
        }
        continue;
      }
      if (active_requests_.fetch_add(1) >= 8) {
        active_requests_.fetch_sub(1);
        socket.close(error);
        continue;
      }
      try {
        asio::post(pool_, [this, socket = std::move(socket)]() mutable {
          try {
            beast::tcp_stream stream(std::move(socket));
            stream.expires_after(std::chrono::seconds(2));
            serve(stream);
            beast::error_code ignored;
            stream.socket().shutdown(tcp::socket::shutdown_both, ignored);
          } catch (const std::exception & exception) {
            std::fprintf(stderr, "openarm_portal request: %s\n", exception.what());
          }
          active_requests_.fetch_sub(1);
        });
      } catch (...) {
        active_requests_.fetch_sub(1);
        throw;
      }
    }
    beast::error_code ignored;
    acceptor_.close(ignored);
    node_->begin_shutdown();
    pool_.join();
  }

private:
  bool write_asset(
    beast::tcp_stream & stream, const http::request<http::string_body> & request,
    const StaticAsset & asset)
  {
    beast::error_code error;
    http::file_body::value_type body;
    const std::filesystem::path asset_path = assets_.path(asset);
    body.open(asset_path.string().c_str(), beast::file_mode::scan, error);
    if (error) {
      write_json(stream, http::status::service_unavailable,
        "{\"error\":\"viewer asset is unavailable\"}");
      return false;
    }
    http::response<http::file_body> response{http::status::ok, request.version()};
    response.set(http::field::content_type, asset.content_type);
    response.content_length(body.size());
    response.body() = std::move(body);
    write_response(stream, response);
    return true;
  }

  void serve(beast::tcp_stream & stream)
  {
    beast::flat_buffer buffer;
    http::request_parser<http::string_body> parser;
    parser.header_limit(8192);
    parser.body_limit(512);
    beast::error_code error;
    http::read(stream, buffer, parser, error);
    if (error) {
      return;
    }
    const auto & request = parser.get();
    const std::string target(request.target());
    std::string reason;
    const SafeRequestHeaders request_headers = safe_headers(request);
    if (!safe_policy_.validate_read(request_headers, reason)) {
      write_json(stream, http::status::forbidden,
        "{\"error\":\"" + json_escape(reason) + "\"}");
      return;
    }
    if (request.method() == http::verb::get && !request.body().empty()) {
      write_json(stream, http::status::bad_request, "{\"error\":\"GET body is not allowed\"}");
      return;
    }
    if (request.method() == http::verb::get && target == "/") {
      http::response<http::string_body> response{http::status::ok, request.version()};
      response.set(http::field::content_type, "text/html; charset=utf-8");
      response.body() = page_;
      write_response(stream, response);
      return;
    }
    if (request.method() == http::verb::get && assets_.ready()) {
      if (const StaticAsset * asset = assets_.find(target)) {
        (void)write_asset(stream, request, *asset);
        return;
      }
    }
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
    if (request.method() == http::verb::get && target == "/api/view-state") {
      write_json(stream, http::status::ok, viewer_state_json(node_->viewer_snapshot(), steady_now_ns()));
      return;
    }
    if (request.method() != http::verb::post ||
      (target != "/api/move" && target != "/api/v2/move" &&
      target != "/api/stop" && target != "/api/verify"))
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
    if (target == "/api/v2/move") {
      UnitMoveRequest unit_move;
      if (!StrictJson::parse_move_v2(request.body(), unit_move, reason) ||
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
    GuardInput input;
    std::array<Point, 2> tcp{};
    if (!node_->state(input, tcp, reason)) {
      write_json(stream, http::status::conflict,
        "{\"error\":\"" + json_escape(reason) + "\"}");
      return;
    }
    input.request = move;
    const GuardResult guarded = guard_.validate(input);
    if (!guarded.accepted) {
      write_json(stream, http::status::unprocessable_entity,
        "{\"error\":\"virtual nominal guard rejected: " + json_escape(guarded.reason) + "\"}");
      return;
    }
    if (!node_->move(move, input, guarded, reason)) {
      write_json(stream, http::status::conflict,
        "{\"error\":\"" + json_escape(reason) + "\"}");
      return;
    }
    write_json(stream, http::status::accepted,
      "{\"message\":\"" + json_escape(reason +
      "; sampled nominal virtual protection only; controller collision_checked=false") + "\"}");
  }

  asio::io_context context_;
  tcp::acceptor acceptor_;
  asio::thread_pool pool_{4};
  std::atomic_uint active_requests_{0};
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
    if (argc != 3 || std::string(argv[1]) != "--port")
    {
      std::fprintf(stderr, "Usage: openarm_portal --port PORT\n");
      return 2;
    }
    const std::uint64_t port_value = unsigned_number(argv[2], "port");
    if (port_value < 1024 || port_value > 65535)
    {
      throw std::invalid_argument("port outside allowed range");
    }
    rclcpp::init(argc, argv);
    std::signal(SIGINT, openarm_ik_ros::portal::signal_handler);
    std::signal(SIGTERM, openarm_ik_ros::portal::signal_handler);
    auto node = std::make_shared<openarm_ik_ros::portal::PortalNode>();
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
