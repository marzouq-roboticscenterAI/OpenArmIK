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
constexpr auto kStateFreshness = std::chrono::milliseconds(500);
constexpr auto kDiagnosticFreshness = std::chrono::milliseconds(1500);
volatile std::sig_atomic_t stop_requested = 0;

void signal_handler(int)
{
  stop_requested = 1;
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

std::string page(std::string_view csrf)
{
  std::string html = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>OpenArm virtual portal</title><style>
:root{color-scheme:dark;font-family:Inter,system-ui,sans-serif;background:#0b1018;color:#e9eef6}*{box-sizing:border-box}body{margin:0}main{display:grid;grid-template-columns:minmax(320px,420px) 1fr;min-height:100vh}.controls{padding:24px;background:#121a26;border-right:1px solid #2a3545;overflow:auto}.viewer{display:flex;flex-direction:column;padding:18px;min-width:0}.truth{background:#402713;border:1px solid #b36b27;border-radius:8px;padding:12px;margin:0 0 16px;line-height:1.4}.card{background:#192331;border:1px solid #304055;border-radius:10px;padding:14px;margin-bottom:12px}h1{font-size:1.4rem;margin:0 0 8px}h2{font-size:1rem;margin:0 0 10px}.xyz{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}label{font-size:.75rem;color:#aeb9c8}input{width:100%;margin-top:4px;padding:9px;background:#0e1621;color:#fff;border:1px solid #41516a;border-radius:6px}button{width:100%;padding:10px;margin-top:10px;border:0;border-radius:7px;background:#2d77d0;color:white;font-weight:650;cursor:pointer}button.stop{background:#a13636}button.verify{background:#526275}button:disabled{opacity:.45;cursor:not-allowed}.status{font-family:ui-monospace,monospace;font-size:.82rem;white-space:pre-wrap;line-height:1.5}.frame{flex:1;display:flex;align-items:center;justify-content:center;background:#06090e;border:1px solid #2b3747;border-radius:10px;overflow:hidden;min-height:280px}.frame img{max-width:100%;max-height:calc(100vh - 80px);object-fit:contain}.caption{color:#9eabbc;font-size:.8rem;margin-top:8px}@media(max-width:900px){main{grid-template-columns:1fr}.controls{border-right:0}.viewer{min-height:55vh}.frame img{max-height:70vh}}
</style></head><body><main><section class="controls"><h1>OpenArm virtual portal</h1>
<div class="truth"><strong>Virtual simulation only.</strong><br>Controller collision checked: <strong>NO</strong>.<br>The portal uses sampled nominal capsules and a central keepout. This is not physical collision certification. A path is rejected unless that limited guard can prove its checks.</div>
<div class="card"><h2>Measured TCP / target — metres, openarm_body_link0</h2><div id="age" class="caption">Waiting for encoder-derived joint state…</div></div>
<div class="card"><h2>Left target (orientation unconstrained)</h2><div class="xyz"><label>X<input id="lx" inputmode="decimal"></label><label>Y<input id="ly" inputmode="decimal"></label><label>Z<input id="lz" inputmode="decimal"></label></div><button id="left">Move Left (Right target = freshest measured TCP)</button></div>
<div class="card"><h2>Right target (orientation unconstrained)</h2><div class="xyz"><label>X<input id="rx" inputmode="decimal"></label><label>Y<input id="ry" inputmode="decimal"></label><label>Z<input id="rz" inputmode="decimal"></label></div><button id="right">Move Right (Left target = freshest measured TCP)</button></div>
<div class="card"><button class="verify" id="verify">Auto Calibrate — simulation verification only</button><div class="caption">Nonmoving model/state verification; it performs no physical calibration.</div><button class="stop" id="stop">Request software stop (not a hardwired E-stop)</button><div class="caption">Cancels the active portal goal. It is not safety-rated and cannot replace a hardwired E-stop.</div></div>
<div class="card"><h2>Measured command progress/result</h2><div id="status" class="status">No portal command.</div></div></section>
<section class="viewer"><h2>Actual launcher-owned stock RViz pixels</h2><div class="frame"><img id="rviz" alt="RViz capture unavailable"></div><div class="caption">XComposite snapshot from the exact launcher PID. Image freshness is never used as control feedback.</div></section></main>
<script>const csrf='__CSRF__';let seeded=false;const $=id=>document.getElementById(id);async function state(){try{const r=await fetch('/api/state',{cache:'no-store'}),s=await r.json();$('status').textContent=s.command;const ok=s.state_fresh&&!s.command_active;$('left').disabled=!ok;$('right').disabled=!ok;$('age').textContent=s.summary;if(s.state_fresh&&!seeded){for(const [id,v] of [['lx',s.left[0]],['ly',s.left[1]],['lz',s.left[2]],['rx',s.right[0]],['ry',s.right[1]],['rz',s.right[2]]])$(id).value=Number(v).toFixed(6);seeded=true}}catch(e){$('age').textContent='State unavailable';$('left').disabled=true;$('right').disabled=true}}async function post(path,body={}){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':csrf},body:JSON.stringify(body)});const j=await r.json();if(!r.ok)throw new Error(j.error||'request rejected');$('status').textContent=j.message}function move(side){const p=side==='left'?'l':'r';post('/api/move',{side,x:Number($(p+'x').value),y:Number($(p+'y').value),z:Number($(p+'z').value)}).catch(e=>$('status').textContent=e.message)}$('left').onclick=()=>move('left');$('right').onclick=()=>move('right');$('stop').onclick=()=>post('/api/stop').catch(e=>$('status').textContent=e.message);$('verify').onclick=()=>post('/api/verify').catch(e=>$('status').textContent=e.message);setInterval(state,250);state();const im=$('rviz');setInterval(()=>{im.src='/api/rviz.jpg?t='+Date.now()},350);im.src='/api/rviz.jpg';</script></body></html>)HTML";
  const std::size_t position = html.find("__CSRF__");
  html.replace(position, 8, csrf);
  return html;
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
    const auto steady_now = std::chrono::steady_clock::now();
    if (!have_state_ || steady_now - state_received_ > kStateFreshness ||
      !diagnostic_valid_ || steady_now - diagnostic_received_ > kDiagnosticFreshness)
    {
      reason = diagnostic_reason_.empty() ?
        "encoder-derived joint state or controller diagnostics are missing or stale" : diagnostic_reason_;
      return false;
    }
    input.measured_q = measured_q_;
    input.state_sequence = state_sequence_;
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
    std::ostringstream output;
    output << "{\"state_fresh\":" << (fresh ? "true" : "false") <<
      ",\"command_active\":" << (goal_active_ ? "true" : "false") <<
      ",\"left\":[" << number(tcp[0][0]) << ',' << number(tcp[0][1]) << ',' <<
      number(tcp[0][2]) << "],\"right\":[" << number(tcp[1][0]) << ',' <<
      number(tcp[1][1]) << ',' << number(tcp[1][2]) << "],\"summary\":\"" <<
      json_escape(fresh ? "Fresh encoder-derived virtual state; controller collision_checked=false" : reason) <<
      "\",\"command\":\"" << json_escape(command_) << "\"}";
    return output.str();
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
      const auto steady_now = std::chrono::steady_clock::now();
      if (stopping_ || stop_requested != 0 || input.state_sequence == 0 ||
        input.state_sequence != state_sequence_ ||
        !have_state_ || steady_now - state_received_ > kStateFreshness ||
        !diagnostic_valid_ || steady_now - diagnostic_received_ > kDiagnosticFreshness)
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
      "Left target submitted with Right target set to freshest measured TCP" :
      "Right target submitted with Left target set to freshest measured TCP";
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
  void update_state(const sensor_msgs::msg::JointState & message)
  {
    if (message.name.size() != message.position.size()) {
      return;
    }
    const rclcpp::Time producer_stamp(message.header.stamp);
    const rclcpp::Duration age = now() - producer_stamp;
    if (producer_stamp.nanoseconds() <= 0 || age.nanoseconds() < 0 ||
      age.nanoseconds() > std::chrono::duration_cast<std::chrono::nanoseconds>(kStateFreshness).count())
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
    state_received_ = std::chrono::steady_clock::now();
    have_state_ = true;
    ++state_sequence_;
  }

  void update_diagnostics(const diagnostic_msgs::msg::DiagnosticArray & message)
  {
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
      const bool valid = status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN &&
        value("backend") == "virtual" && value("physical_motion_authorized") == "false" &&
        value("collision_checked") == "false" && masks;
      std::lock_guard<std::mutex> lock(mutex_);
      diagnostic_valid_ = valid;
      diagnostic_reason_ = valid ? std::string{} : "controller diagnostics are stale, faulted, or inconsistent";
      diagnostic_received_ = std::chrono::steady_clock::now();
      return;
    }
  }

  mutable std::mutex mutex_;
  std::array<JointVector, 2> measured_q_{};
  std::chrono::steady_clock::time_point state_received_{};
  bool have_state_{false};
  std::uint64_t state_sequence_{0};
  bool diagnostic_valid_{false};
  std::string diagnostic_reason_{"controller diagnostics have not arrived"};
  std::chrono::steady_clock::time_point diagnostic_received_{};
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
    std::int64_t rviz_pid, std::uint64_t rviz_start_ticks)
  : context_(1), acceptor_(context_), node_(std::move(node)), capture_(rviz_pid, rviz_start_ticks),
    csrf_(token()), authority_("127.0.0.1:" + std::to_string(port)), policy_(authority_, csrf_),
    page_(page(csrf_))
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
      (target != "/api/move" && target != "/api/stop" && target != "/api/verify"))
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
    if (!StrictJson::parse_move(request.body(), move, reason)) {
      write_json(stream, http::status::bad_request,
        "{\"error\":\"" + json_escape(reason) + "\"}");
      return;
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
    if (argc != 7 || std::string(argv[1]) != "--rviz-pid" ||
      std::string(argv[3]) != "--rviz-start-ticks" || std::string(argv[5]) != "--port")
    {
      std::fprintf(stderr,
        "Usage: openarm_portal --rviz-pid PID --rviz-start-ticks TICKS --port PORT\n");
      return 2;
    }
    const std::uint64_t pid = unsigned_number(argv[2], "RViz PID");
    const std::uint64_t ticks = unsigned_number(argv[4], "RViz start ticks");
    const std::uint64_t port_value = unsigned_number(argv[6], "port");
    if (pid > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) ||
      port_value < 1024 || port_value > 65535)
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
        static_cast<unsigned short>(port_value), node, static_cast<std::int64_t>(pid), ticks);
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
