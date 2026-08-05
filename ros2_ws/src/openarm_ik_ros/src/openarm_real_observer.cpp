// SPDX-License-Identifier: Apache-2.0
//
// Read-only observer for a physically connected OpenArm v1.0.
//
// This node exists for one job: show what the real arms are doing. It reads
// DaMiao motor feedback from can0 and can1, works out which arm is on which
// interface, and publishes /joint_states so robot_state_publisher and RViz
// mirror the hardware.
//
// It cannot move the arms, and that is enforced structurally rather than by
// convention. The only frames it will transmit are the two read-only DaMiao
// primitives, refresh-status and register-query; transmit() rejects anything
// else before it reaches the socket. There is no path here to enable a motor,
// write a register, set a zero, or send a motion command.
//
// It also starts passive. Nothing is opened and nothing is transmitted until
// something calls the connect service, so plugging in and launching the stack
// is inert.
//
// What this node does NOT establish, and must not be read as establishing:
//   - that the motor zero positions correspond to the URDF zero pose. Nothing
//     here commissions a zero, a direction, or a gear ratio. Joint angles are
//     published under the identity mapping unless one is supplied, so a pose
//     that looks wrong in RViz means the mapping needs commissioning, not that
//     the reading failed.
//   - that the arm assignment is safe to command from. It is good enough to
//     label a view; it is not a commissioning record.
#include "openarm_ik_ros/real_observer_core.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <fstream>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
using namespace std::chrono_literals;
using openarm_ik_ros::real::ArmAssignment;
using openarm_ik_ros::real::BusReading;
using openarm_ik_ros::real::ObserverConfig;
using openarm_ik_ros::real::RealObserver;

class RealObserverNode : public rclcpp::Node
{
public:
  RealObserverNode()
  : rclcpp::Node("openarm_real_observer")
  {
    ObserverConfig config;
    config.interfaces[0] = declare_parameter<std::string>("interface_a", "can0");
    config.interfaces[1] = declare_parameter<std::string>("interface_b", "can1");
    config.first_send_id =
      static_cast<std::uint16_t>(declare_parameter<int>("first_send_id", 0x01));
    config.last_send_id =
      static_cast<std::uint16_t>(declare_parameter<int>("last_send_id", 0x0E));
    config.receive_id_offset =
      static_cast<std::uint16_t>(declare_parameter<int>("receive_id_offset", 0x10));
    config.reply_timeout_ms = declare_parameter<int>("reply_timeout_ms", 40);
    // "left" or "right" pins the assignment when the automatic methods cannot
    // separate the two buses. Empty leaves it automatic.
    config.forced_side_for_interface_a =
      declare_parameter<std::string>("interface_a_side", "");

    const char * home = std::getenv("HOME");
    zero_path_ = declare_parameter<std::string>(
      "zero_file", std::string(home != nullptr ? home : "/tmp") + "/.openarm_real_zero");
    invert_gripper_ = !declare_parameter<bool>("gripper_opens_with_increasing_angle", false);
    capture_zero_on_connect_ = declare_parameter<bool>("capture_zero_on_connect", true);
    const bool connect_on_start = declare_parameter<bool>("connect_on_start", false);
    load_zero();

    observer_ = std::make_unique<RealObserver>(config);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    joint_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/openarm_real/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    connect_service_ = create_service<std_srvs::srv::Trigger>(
      "/openarm_real/connect",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        std::string detail;
        const bool ok = observer_->connect(detail);
        // Capture the startup pose as the reference automatically. The arms
        // rest in a consistent physical position at power-on, so treating that
        // as neutral makes RViz agree with the robot without a manual step.
        //
        // Worth being clear about what this assumes: whatever pose the arms
        // happen to be in at Connect becomes the definition of neutral. If they
        // are not resting when you connect, the reference is wrong and nothing
        // will say so -- press Clear zero and reconnect with the arms at rest.
        if (ok && capture_zero_on_connect_) {
          std::string zero_message;
          if (capture_zero(zero_message)) {
            detail += " Startup pose captured as neutral.";
          } else {
            detail += " Could not capture a startup zero: " + zero_message;
          }
        }
        response->success = ok;
        response->message = detail;
        publish_status();
        RCLCPP_INFO(get_logger(), "connect requested: %s", detail.c_str());
      });
    disconnect_service_ = create_service<std_srvs::srv::Trigger>(
      "/openarm_real/disconnect",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        observer_->disconnect();
        response->success = true;
        response->message = "disconnected; sockets closed and publishing stopped";
        publish_status();
        RCLCPP_INFO(get_logger(), "disconnected");
      });

    swap_service_ = create_service<std_srvs::srv::Trigger>(
      "/openarm_real/swap_sides",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        observer_->swap_sides();
        response->success = true;
        response->message = observer_->assignment().reason;
        publish_status();
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
      });

    capture_zero_service_ = create_service<std_srvs::srv::Trigger>(
      "/openarm_real/capture_zero",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        std::string message;
        response->success = capture_zero(message);
        response->message = message;
        RCLCPP_INFO(get_logger(), "%s", message.c_str());
      });
    clear_zero_service_ = create_service<std_srvs::srv::Trigger>(
      "/openarm_real/clear_zero",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        clear_zero();
        response->success = true;
        response->message = "cleared the captured zero; showing raw motor angles again";
      });

    for (const auto & entry : {std::pair<const char *, std::size_t>{"left", 0U},
        std::pair<const char *, std::size_t>{"right", 1U}})
    {
      const std::size_t side = entry.second;
      flip_services_.push_back(create_service<std_srvs::srv::Trigger>(
          std::string("/openarm_real/flip_") + entry.first,
          [this, side, name = std::string(entry.first)](
            const std_srvs::srv::Trigger::Request::SharedPtr,
            std_srvs::srv::Trigger::Response::SharedPtr response)
          {
            arm_sign_[side] = -arm_sign_[side];
            save_zero();
            response->success = true;
            response->message = "the " + name + " arm now moves " +
            (arm_sign_[side] < 0.0 ? "inverted" : "normally") +
            " on screen; saved, so it persists across restarts";
            RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
          }));
    }

    timer_ = create_wall_timer(10ms, [this]() {poll();});
    if (connect_on_start) {
      // One shot, slightly delayed. Connecting from the constructor would sweep
      // the bus before the node is fully built and before publishers are
      // discovered, so the first readings would go nowhere.
      startup_timer_ = create_wall_timer(1500ms, [this]() {
          startup_timer_->cancel();
          std::string detail;
          if (observer_->connect(detail) && capture_zero_on_connect_) {
            std::string zero_message;
            (void)capture_zero(zero_message);
            detail += " " + zero_message;
          }
          publish_status();
          RCLCPP_INFO(get_logger(), "auto-connect: %s", detail.c_str());
        });
    }
    publish_status();
    RCLCPP_WARN(
      get_logger(),
      "Real-arm observer is READ ONLY and starts passive. It transmits only "
      "DaMiao refresh-status and register-query frames and has no path to "
      "enable, zero, or move a motor. Joint angles use an uncommissioned "
      "mapping: a pose that looks wrong in RViz means the mapping needs "
      "commissioning.");
  }

private:
  void poll()
  {
    // Being passive means sending nothing on the CAN bus. It does not mean
    // going quiet on ROS: robot_state_publisher derives TF from /joint_states,
    // so if this stops publishing, the robot vanishes from RViz entirely and an
    // idle stack looks like a broken one. Publish the rest pose until there is
    // something measured to show.
    std::array<BusReading, 2> readings{};
    const bool measured = observer_->connected() && observer_->read_once(readings);

    if (observer_->connected() && !measured) {
      ++consecutive_failures_;
      if (consecutive_failures_ == 50U) {
        RCLCPP_WARN(get_logger(), "no complete reading for 500 ms: %s",
          observer_->detail().c_str());
        publish_status();
      }
    } else {
      consecutive_failures_ = 0U;
    }

    publish_joint_state(readings, measured);
    if (++status_divider_ >= 100U) {
      status_divider_ = 0U;
      publish_status();
    }
  }

  void publish_joint_state(const std::array<BusReading, 2> & readings, const bool measured)
  {
    const ArmAssignment assignment = observer_->assignment();
    // An unresolved assignment means we do not know which arm is which, so
    // measured angles would be as likely to land on the wrong side as the
    // right one. Fall back to the rest pose rather than guess.
    const bool usable = measured && assignment.resolved;
    sensor_msgs::msg::JointState state;
    state.header.stamp = now();
    const auto & names = openarm_ik_ros::real::canonical_joint_names();
    state.name.assign(names.begin(), names.end());
    // The gripper motor drives prismatic finger joints that are not part of the
    // 14-joint arm manifest, so they are appended here rather than baked into
    // the manifest, which describes the arm.
    for (const char * finger : {"openarm_left_finger_joint1", "openarm_left_finger_joint2",
        "openarm_right_finger_joint1", "openarm_right_finger_joint2"})
    {
      state.name.emplace_back(finger);
    }
    state.position.assign(state.name.size(), 0.0);
    state.velocity.assign(state.name.size(), 0.0);
    state.effort.assign(state.name.size(), 0.0);
    if (usable) {
      for (std::size_t bus = 0; bus < 2U; ++bus) {
        if (!readings[bus].complete) {
          continue;   // Unpopulated bus: leave that arm at the rest pose.
        }
        const std::size_t side = assignment.side_of_interface[bus];
        for (std::size_t joint = 0; joint < 7U; ++joint) {
          const std::size_t index = side * 7U + joint;
          // Motor space to URDF space: subtract the captured reference, then
          // apply this arm's direction sign. Order matters -- the reference was
          // captured in motor space, so it must come off before the sign.
          //
          // The sign is operator-set, not derived. The virtual manifest's
          // q_scale was tried and did not match this hardware, which is not
          // surprising: that manifest carries a uniform 0.125 q_offset for
          // every joint, so its per-joint values are placeholders rather than
          // measurements. Nothing on the bus reports mounting orientation
          // either. So this is settled by looking at the robot and pressing a
          // button, and the result is persisted.
          state.position[index] = arm_sign_[side] *
            (readings[bus].position_rad[joint] - zero_offset_[side][joint]);
            state.velocity[index] = arm_sign_[side] * readings[bus].velocity_rad_s[joint];
          state.effort[index] = arm_sign_[side] * readings[bus].torque_nm[joint];
        }
        if (readings[bus].has_gripper) {
          const double metres = gripper_metres(bus, readings[bus].gripper_rad);
          const std::size_t base = names.size() + (side == 0U ? 0U : 2U);
          state.position[base] = metres;
          state.position[base + 1U] = metres;
        }
      }
    }
    joint_publisher_->publish(std::move(state));
  }

  /// Map the gripper motor angle onto the finger joints' 0..0.044 m travel.
  ///
  /// The conversion factor is not known: nothing has commissioned the gripper's
  /// zero or its gear ratio, and the motor angle alone does not say how far the
  /// jaws are apart. So this self-calibrates on the range it has actually seen
  /// since connecting -- open and close the claw once and it tracks correctly
  /// from then on.
  ///
  /// The consequence to understand: before a full open-close cycle the reading
  /// is relative, so a half-open claw will render fully open simply because
  /// that is the widest position observed so far. It is a faithful indicator of
  /// movement, not yet a calibrated measurement.
  double gripper_metres(const std::size_t bus, const double angle)
  {
    auto & span = gripper_span_[bus];
    if (!span.seen) {
      span.seen = true;
      span.minimum = angle;
      span.maximum = angle;
    }
    span.minimum = std::min(span.minimum, angle);
    span.maximum = std::max(span.maximum, angle);
    constexpr double kFingerTravelM = 0.044;
    // Until the jaws have actually been moved there is no span to scale
    // against; reporting closed is better than dividing by ~0 and jumping.
    constexpr double kMinimumUsableSpanRad = 0.05;
    const double span_rad = span.maximum - span.minimum;
    if (span_rad < kMinimumUsableSpanRad) {
      return 0.0;
    }
    // The jaws open as the motor angle DECREASES: measured on the hardware,
    // where opening the claw showed as closed on screen and vice versa. This is
    // a fixed property of how the gripper's drive is mounted rather than a
    // per-unit calibration, so the default is inverted and gripper_opens_with
    // exists only for a unit built the other way round.
    const double fraction = invert_gripper_ ?
      (span.maximum - angle) / span_rad : (angle - span.minimum) / span_rad;
    return std::max(0.0, std::min(kFingerTravelM, fraction * kFingerTravelM));
  }

  struct GripperSpan
  {
    bool seen{false};
    double minimum{0.0};
    double maximum{0.0};
  };
  std::array<GripperSpan, 2> gripper_span_{};

  /// Record the pose the arms are in right now as the URDF zero pose.
  ///
  /// This is what makes RViz agree with the real robot. The motors report an
  /// angle measured from wherever their own encoder zero happens to sit, which
  /// has no relationship to the URDF zero: that is why a resting arm renders
  /// lifted, and why joint 4 reads about -0.94 rad when its URDF range starts
  /// at 0. Subtracting a captured reference removes that constant.
  ///
  /// The operator has to put the arms into the URDF zero pose first. Convenient
  /// property of this design: while passive, the observer publishes exactly
  /// that pose, so the shape shown in RViz before connecting IS the shape to
  /// match the hardware to.
  bool capture_zero(std::string & out_message)
  {
    std::array<BusReading, 2> readings{};
    if (!observer_->connected() || !observer_->read_once(readings)) {
      out_message = "cannot capture a zero without a live reading; press Connect first";
      return false;
    }
    const ArmAssignment assignment = observer_->assignment();
    if (!assignment.resolved) {
      out_message = "cannot capture a zero while the arms are unidentified: the offsets "
        "would be stored against the wrong sides";
      return false;
    }
    std::size_t captured = 0;
    for (std::size_t bus = 0; bus < 2U; ++bus) {
      if (!readings[bus].complete) {
        continue;
      }
      const std::size_t side = assignment.side_of_interface[bus];
      for (std::size_t joint = 0; joint < 7U; ++joint) {
        zero_offset_[side][joint] = readings[bus].position_rad[joint];
      }
      ++captured;
    }
    save_zero();
    out_message = "captured the current pose as zero for " + std::to_string(captured) +
      " arm(s); RViz now shows movement relative to it. Saved to " + zero_path_;
    return true;
  }

  void clear_zero()
  {
    zero_offset_ = {};
    save_zero();
  }

  void save_zero() const
  {
    std::ofstream file(zero_path_);
    if (!file) {
      RCLCPP_WARN(get_logger(), "could not write %s; the zero will not survive a restart",
        zero_path_.c_str());
      return;
    }
    for (const auto & side : zero_offset_) {
      for (const double value : side) {
        file << value << ' ';
      }
      file << '\n';
    }
    file << arm_sign_[0] << ' ' << arm_sign_[1] << '\n';
  }

  void load_zero()
  {
    std::ifstream file(zero_path_);
    if (!file) {
      return;
    }
    for (auto & side : zero_offset_) {
      for (double & value : side) {
        if (!(file >> value)) {
          zero_offset_ = {};   // A truncated file is worse than none.
          return;
        }
      }
    }
    // Signs were added after the first release of this file, so a file without
    // them must still load rather than being discarded.
    double left_sign = 1.0;
    double right_sign = 1.0;
    if (file >> left_sign >> right_sign) {
      arm_sign_[0] = left_sign < 0.0 ? -1.0 : 1.0;
      arm_sign_[1] = right_sign < 0.0 ? -1.0 : 1.0;
    }
    RCLCPP_INFO(get_logger(), "loaded a saved zero from %s", zero_path_.c_str());
  }

  std::array<std::array<double, 7>, 2> zero_offset_{};
  /// Direction of each arm, +1 or -1, applied to every joint on that arm.
  std::array<double, 2> arm_sign_{1.0, 1.0};
  std::string zero_path_;
  bool invert_gripper_{true};
  bool capture_zero_on_connect_{true};

  void publish_status()
  {
    std_msgs::msg::String message;
    message.data = observer_->status_json();
    status_publisher_->publish(std::move(message));
  }

  std::unique_ptr<RealObserver> observer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr swap_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_zero_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_zero_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> flip_services_;
  unsigned consecutive_failures_{0};
  unsigned status_divider_{0};
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealObserverNode>());
  rclcpp::shutdown();
  return 0;
}
