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

#include "openarm_ik_ros/rviz_capture.hpp"

#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <memory>
#include <limits>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

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
  std::random_device source;
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < 32; ++index) {
    output << std::setw(2) << (source() & 0xffU);
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
  PortalServer(
    unsigned short port, std::shared_ptr<PortalNode> node,
    std::int64_t rviz_pid, std::uint64_t rviz_start_ticks, std::string rviz_executable)
  : context_(1), acceptor_(context_), node_(std::move(node)),
    capture_(rviz_pid, rviz_start_ticks, std::move(rviz_executable)),
    csrf_(token()), authority_("127.0.0.1:" + std::to_string(port)), policy_(authority_, csrf_),
    page_(portal_page(csrf_))
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
    if (request.method() == http::verb::get && target == "/") {
      http::response<http::string_body> response{http::status::ok, request.version()};
      response.set(http::field::content_type, "text/html; charset=utf-8");
      response.set(http::field::content_security_policy,
        "default-src 'self'; img-src 'self'; style-src 'unsafe-inline'; script-src 'unsafe-inline'; object-src 'none'; frame-ancestors 'none'");
      response.set(http::field::x_content_type_options, "nosniff");
      response.set(http::field::referrer_policy, "no-referrer");
      response.body() = page_;
      write_response(stream, response);
      return;
    }
    if (request.method() == http::verb::get && target == "/api/health") {
      bool valid = false;
      std::string reason;
      {
        std::lock_guard<std::mutex> lock(capture_mutex_);
        valid = capture_.window_ready(reason);
      }
      write_json(stream, valid ? http::status::ok : http::status::service_unavailable,
        std::string("{\"healthy\":") + (valid ? "true" : "false") +
        ",\"bind\":\"127.0.0.1\",\"rviz_process_identity\":" +
        (capture_.identity_valid() ? "true" : "false") +
        ",\"window_ready\":" + (valid ? "true" : "false") +
        ",\"reason\":\"" + json_escape(reason) + "\"}");
      return;
    }
    if (request.method() == http::verb::get && target == "/api/state") {
      write_json(stream, http::status::ok, node_->state_json());
      return;
    }
    if (request.method() == http::verb::get && target.rfind("/api/rviz.jpg", 0) == 0) {
      std::vector<unsigned char> jpeg;
      std::string reason;
      bool captured = false;
      {
        std::lock_guard<std::mutex> lock(capture_mutex_);
        captured = capture_.capture_jpeg(jpeg, reason);
      }
      if (!captured) {
        write_json(stream, http::status::service_unavailable,
          "{\"error\":\"" + json_escape(reason) + "\"}");
        return;
      }
      http::response<http::vector_body<unsigned char>> response{http::status::ok, request.version()};
      response.set(http::field::content_type, "image/jpeg");
      response.body() = std::move(jpeg);
      write_response(stream, response);
      return;
    }
    if (request.method() != http::verb::post ||
      (target != "/api/move" && target != "/api/v2/move" &&
      target != "/api/stop" && target != "/api/verify"))
    {
      write_json(stream, http::status::not_found, "{\"error\":\"route not found\"}");
      return;
    }
    MutationHeaders headers{
      request[http::field::host], request[http::field::origin], request["X-CSRF-Token"],
      request[http::field::content_type], request.body().size()};
    std::string reason;
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
  RvizCapture capture_;
  std::mutex capture_mutex_;
  std::string csrf_;
  std::string authority_;
  MutationPolicy policy_;
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
    if (argc != 9 || std::string(argv[1]) != "--rviz-pid" ||
      std::string(argv[3]) != "--rviz-start-ticks" ||
      std::string(argv[5]) != "--rviz-executable" || std::string(argv[7]) != "--port")
    {
      std::fprintf(stderr,
        "Usage: openarm_portal --rviz-pid PID --rviz-start-ticks TICKS "
        "--rviz-executable ABSOLUTE_PATH --port PORT\n");
      return 2;
    }
    const std::uint64_t pid = unsigned_number(argv[2], "RViz PID");
    const std::uint64_t ticks = unsigned_number(argv[4], "RViz start ticks");
    const std::string rviz_executable(argv[6]);
    const std::uint64_t port_value = unsigned_number(argv[8], "port");
    if (pid > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
      port_value < 1024 || port_value > 65535 || rviz_executable.empty() ||
      rviz_executable.front() != '/')
    {
      throw std::invalid_argument("PID or port outside allowed range");
    }
    rclcpp::init(argc, argv);
    std::signal(SIGINT, openarm_ik_ros::portal::signal_handler);
    std::signal(SIGTERM, openarm_ik_ros::portal::signal_handler);
    auto node = std::make_shared<openarm_ik_ros::portal::PortalNode>();
    std::thread ros_thread([node]() {rclcpp::spin(node);});
    try {
      openarm_ik_ros::portal::PortalServer server(
        static_cast<unsigned short>(port_value), node, static_cast<std::int64_t>(pid), ticks,
        rviz_executable);
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
