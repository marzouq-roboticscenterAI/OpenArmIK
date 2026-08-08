// SPDX-License-Identifier: Apache-2.0
//
// Read-only DaMiao bus observer. See openarm_real_observer.cpp for the node
// that drives it and for the safety argument.
#ifndef OPENARM_IK_ROS__REAL_OBSERVER_CORE_HPP_
#define OPENARM_IK_ROS__REAL_OBSERVER_CORE_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace openarm_ik_ros::real
{

/// URDF revolute joints per arm. The physical arm carries one more motor than
/// this: a v1.0 bus answers on IDs 0x01..0x08, where 0x01..0x07 drive the
/// joints and 0x08 drives the gripper. The arm runtime manifest contains only
/// the seven revolute joints; the observer appends the URDF finger joints.
inline constexpr std::size_t kJointsPerArm = 7U;
inline constexpr std::size_t kBusCount = 2U;
inline constexpr std::size_t kLeftSide = 0U;
inline constexpr std::size_t kRightSide = 1U;

/// The 14 URDF joint names, left arm first, in the order robot_state_publisher
/// expects. Sourced from the runtime's virtual manifest so this cannot drift
/// away from the model.
const std::array<std::string, kJointsPerArm * kBusCount> & canonical_joint_names();

struct ObserverConfig
{
  std::array<std::string, kBusCount> interfaces{"can0", "can1"};
  /// Inclusive send-ID sweep. DaMiao motors ship with distinct IDs per bus, so
  /// the range is deliberately wider than one arm's worth of motors: we
  /// discover what is actually out there rather than assuming a layout.
  std::uint16_t first_send_id{0x01};
  std::uint16_t last_send_id{0x0E};
  /// DaMiao's stock convention is receive_id = send_id + 0x10, but the reply is
  /// matched on whatever ID actually arrives, so a motor commissioned to a
  /// different offset is still found. This only seeds the expectation.
  std::uint16_t receive_id_offset{0x10};
  int reply_timeout_ms{40};
  /// "left" or "right" pins interface_a when automatic identification cannot
  /// separate the buses. Empty leaves it automatic.
  std::string forced_side_for_interface_a;
};

/// One motor as actually found on a bus.
struct MotorRecord
{
  std::uint16_t send_id{0};
  std::uint16_t receive_id{0};
  std::uint8_t motor_id{0};
  std::uint8_t status_nibble{0};
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  double torque_nm{0.0};
  std::uint8_t mos_temperature_c{0};
  std::uint8_t rotor_temperature_c{0};
};

/// A synchronous sample of one bus's seven joints.
struct BusReading
{
  std::array<double, kJointsPerArm> position_rad{};
  std::array<double, kJointsPerArm> velocity_rad_s{};
  std::array<double, kJointsPerArm> torque_nm{};
  /// True only when that exact joint ID answered this sample. This permits a
  /// partial arm to update its responding joints without shifting IDs or
  /// pretending the missing joint was measured.
  std::array<bool, kJointsPerArm> joint_valid{};
  /// Gripper motor, when the bus carries one. No URDF joint corresponds to it.
  bool has_gripper{false};
  double gripper_rad{0.0};
  bool complete{false};
};

/// Map records by their actual send IDs. Present joints are populated and
/// marked in joint_valid; the return value and complete flag are true only when
/// J1..J7 (IDs 1..7) are all present. ID 8 is the optional gripper. This keeps
/// a missing joint from shifting a later joint or the gripper into its slot.
bool map_motor_records_by_id(const std::vector<MotorRecord> & records, BusReading & reading);

/// True only when a received frame is the reply for one of the exact send IDs
/// requested in the current read cycle. This rejects local-loopback command
/// frames (notably arbitration ID 0x7ff) before their payload can be decoded as
/// encoder feedback.
bool reply_matches_expected(
  std::uint16_t receive_id, std::uint8_t payload_motor_id,
  const std::vector<std::uint16_t> & expected_send_ids,
  std::uint16_t receive_id_offset);

struct ArmAssignment
{
  bool resolved{false};
  /// side_of_interface[bus] is kLeftSide or kRightSide.
  std::array<std::size_t, kBusCount> side_of_interface{kLeftSide, kRightSide};
  /// How it was decided: "forced", "motor-id-partition", "joint-limit-signature",
  /// "operator-confirmed", or empty when unresolved.
  std::string method;
  /// "high" only when a human or an explicit parameter settled it. The angle
  /// heuristic is "low" and must be presented as provisional because the motor
  /// zeros are uncommissioned and absolute angles carry no dependable side
  /// identity.
  std::string confidence{"none"};
  std::string reason;
};

/// Owns the two sockets. Not thread safe; the node drives it from one thread.
///
/// The only frames this class can put on the wire are DaMiao refresh-status and
/// register-query. That is structural, not a policy: the private socket write
/// takes a send ID and builds the frame itself, so no caller can hand it an
/// arbitrary payload, and no enable/zero/motion encoder is referenced anywhere
/// in the implementation.
class RealObserver
{
public:
  explicit RealObserver(ObserverConfig config);
  ~RealObserver();

  RealObserver(const RealObserver &) = delete;
  RealObserver & operator=(const RealObserver &) = delete;

  /// Open both interfaces, sweep for motors, and identify the arms. Returns
  /// false and leaves the observer disconnected if either bus is unusable.
  /// Populates out_detail either way.
  bool connect(std::string & out_detail);
  void disconnect();
  bool connected() const {return connected_;}

  /// One synchronous sample of both buses. Every call refreshes IDs 1..8 on
  /// both buses. True when at least one exact J1..J7 record was received.
  /// Missing joints remain explicitly invalid rather than being mis-mapped.
  bool read_once(std::array<BusReading, kBusCount> & out_readings);

  ArmAssignment assignment() const {return assignment_;}

  /// Flip which bus is the left arm and record that a human decided it. This
  /// is the authoritative correction for the angle heuristic guessing wrong.
  void swap_sides();
  const std::string & detail() const {return detail_;}
  const std::vector<MotorRecord> & discovered(std::size_t bus) const {return discovered_[bus];}

  /// Machine-readable snapshot for the portal.
  std::string status_json() const;

private:
  /// The sole socket write. Builds a refresh-status frame for send_id via
  /// oa_can_make_refresh_status; there is no overload taking a frame.
  bool request_status(std::size_t bus, std::uint16_t send_id);
  /// Drains replies until every expected send ID has answered or the deadline
  /// passes. Returns the records that arrived.
  std::vector<MotorRecord> collect_replies(
    std::size_t bus, const std::vector<std::uint16_t> & expected, int timeout_ms);
  std::vector<MotorRecord> sweep(std::size_t bus);
  void identify_arms();

  ObserverConfig config_;
  std::array<int, kBusCount> sockets_{-1, -1};
  std::array<std::vector<MotorRecord>, kBusCount> discovered_{};
  ArmAssignment assignment_{};
  bool connected_{false};
  std::string detail_;
};

/// Scores how well a measured joint vector fits one side's manifest limits:
/// the total radians by which the vector falls outside them, so lower is a
/// better fit and 0.0 means fully inside. Exposed for testing.
double joint_limit_misfit(const std::array<double, kJointsPerArm> & q, std::size_t side);

/// Sign that converts a motor angle into a URDF joint angle for this joint.
///
/// The manifest's q_scale alternates along the chain and is mirrored between
/// the arms, because the two arms are built as reflections of each other.
/// Publishing a motor angle without it makes every negatively-scaled joint
/// travel the wrong way, which shows up as an arm swinging inboard into the
/// central pole when it was physically moved outboard.
double joint_scale(std::size_t side, std::size_t joint);

}  // namespace openarm_ik_ros::real

#endif  // OPENARM_IK_ROS__REAL_OBSERVER_CORE_HPP_
