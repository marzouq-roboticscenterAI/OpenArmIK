// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/real_observer_core.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>

extern "C" {
#include "openarm_can.h"
#include "openarm_runtime.h"
}

namespace openarm_ik_ros::real
{
namespace
{

/// Per-joint metadata lifted from the runtime's virtual manifest: names, motor
/// types and joint limits. The manifest is the model's own record, so nothing
/// here can drift away from the URDF the viewer renders.
struct JointMetadata
{
  std::array<std::string, kJointsPerArm * kBusCount> names{};
  std::array<std::uint32_t, kJointsPerArm * kBusCount> motor_type{};
  std::array<double, kJointsPerArm * kBusCount> lower_rad{};
  std::array<double, kJointsPerArm * kBusCount> upper_rad{};
  std::array<double, kJointsPerArm * kBusCount> q_scale{};
};

const JointMetadata & metadata()
{
  static const JointMetadata value = []() {
      JointMetadata built;
      oa_runtime_manifest * manifest = nullptr;
      if (oa_runtime_manifest_create_virtual(&manifest) != OA_RUNTIME_OK || manifest == nullptr) {
        throw std::runtime_error("runtime virtual manifest creation failed");
      }
      for (std::uint32_t side = 0U; side < kBusCount; ++side) {
        for (std::uint32_t joint = 0U; joint < kJointsPerArm; ++joint) {
          oa_runtime_motor_manifest motor{};
          motor.struct_size = static_cast<std::uint32_t>(sizeof(motor));
          motor.abi_version = OA_RUNTIME_ABI_VERSION;
          if (oa_runtime_manifest_get_motor(manifest, side, joint, &motor) != OA_RUNTIME_OK) {
            oa_runtime_manifest_destroy(manifest);
            throw std::runtime_error("runtime virtual motor manifest read failed");
          }
          const std::size_t index = side * kJointsPerArm + joint;
          built.names[index] = motor.joint_name;
          built.motor_type[index] = motor.motor_type;
          built.lower_rad[index] = motor.lower_rad;
          built.upper_rad[index] = motor.upper_rad;
          // q_scale and direction carry the same sign in this manifest, so
          // applying both would cancel the correction. q_scale alone is used.
          built.q_scale[index] = motor.q_scale != 0.0 ? motor.q_scale : 1.0;
        }
      }
      oa_runtime_manifest_destroy(manifest);
      return built;
    }();
  return value;
}

std::uint64_t monotonic_ms()
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

std::string quote(const std::string & text)
{
  std::string out = "\"";
  for (const char character : text) {
    if (character == '"' || character == '\\') {
      out.push_back('\\');
    }
    out.push_back(character);
  }
  out.push_back('"');
  return out;
}

std::string number(const double value, const int precision = 6)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

}  // namespace

const std::array<std::string, kJointsPerArm * kBusCount> & canonical_joint_names()
{
  return metadata().names;
}

double joint_scale(const std::size_t side, const std::size_t joint)
{
  if (side >= kBusCount || joint >= kJointsPerArm) {
    return 1.0;
  }
  return metadata().q_scale[side * kJointsPerArm + joint];
}

double joint_limit_misfit(const std::array<double, kJointsPerArm> & q, const std::size_t side)
{
  const JointMetadata & meta = metadata();
  double misfit = 0.0;
  for (std::size_t joint = 0; joint < kJointsPerArm; ++joint) {
    const std::size_t index = side * kJointsPerArm + joint;
    if (q[joint] < meta.lower_rad[index]) {
      misfit += meta.lower_rad[index] - q[joint];
    } else if (q[joint] > meta.upper_rad[index]) {
      misfit += q[joint] - meta.upper_rad[index];
    }
  }
  return misfit;
}

RealObserver::RealObserver(ObserverConfig config)
: config_(std::move(config))
{
  detail_ = "passive; not connected";
}

RealObserver::~RealObserver()
{
  disconnect();
}

bool RealObserver::connect(std::string & out_detail)
{
  disconnect();

  for (std::size_t bus = 0; bus < kBusCount; ++bus) {
    const std::string & name = config_.interfaces[bus];
    const int handle = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (handle < 0) {
      detail_ = "cannot create a CAN socket for " + name + ": " + std::strerror(errno);
      out_detail = detail_;
      disconnect();
      return false;
    }
    sockets_[bus] = handle;

    ifreq request{};
    std::snprintf(request.ifr_name, IFNAMSIZ, "%s", name.c_str());
    if (::ioctl(handle, SIOCGIFINDEX, &request) < 0) {
      detail_ = "interface " + name + " does not exist; is the adapter plugged in?";
      out_detail = detail_;
      disconnect();
      return false;
    }

    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (::bind(handle, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      // ENODEV here is the ordinary "you forgot to bring the link up" case.
      detail_ = "cannot bind " + name + " (" + std::strerror(errno) +
        "); run: sudo bash scripts/setup_can_interfaces.sh";
      out_detail = detail_;
      disconnect();
      return false;
    }
  }

  connected_ = true;
  for (std::size_t bus = 0; bus < kBusCount; ++bus) {
    discovered_[bus] = sweep(bus);
  }
  identify_arms();

  const std::size_t populated = (discovered_[0].size() >= kJointsPerArm ? 1U : 0U) +
    (discovered_[1].size() >= kJointsPerArm ? 1U : 0U);

  std::ostringstream summary;
  summary << "found " << discovered_[0].size() << " motors on " << config_.interfaces[0]
          << " and " << discovered_[1].size() << " on " << config_.interfaces[1];
  if (populated == 1U) {
    const std::size_t empty = discovered_[0].size() >= kJointsPerArm ? 1U : 0U;
    summary << " (one arm only: " << config_.interfaces[empty]
            << " is silent, so that arm is unpowered or unplugged)";
  }
  summary << "; " << assignment_.reason;
  detail_ = summary.str();
  out_detail = detail_;

  // Stay connected either way: a partial sweep is exactly the state an operator
  // needs to see in the portal to work out what is unpowered or mis-wired.
  // Success means we can put a correctly-labelled arm on screen, which one
  // identified arm is enough for.
  return populated >= 1U && assignment_.resolved;
}

void RealObserver::disconnect()
{
  for (int & handle : sockets_) {
    if (handle >= 0) {
      ::close(handle);
      handle = -1;
    }
  }
  connected_ = false;
  detail_ = "passive; not connected";
}

bool RealObserver::request_status(const std::size_t bus, const std::uint16_t send_id)
{
  // The frame is built here, from an ID, by the read-only encoder. A caller
  // cannot supply bytes, so no motion, enable, zero or save frame can reach
  // the socket through this class.
  oa_can_frame frame{};
  frame.struct_size = static_cast<std::uint32_t>(sizeof(frame));
  frame.abi_version = OA_CAN_ABI_VERSION;
  if (oa_can_make_refresh_status(send_id, &frame) != OA_CAN_OK) {
    return false;
  }

  can_frame outgoing{};
  outgoing.can_id = frame.can_id;
  outgoing.can_dlc = frame.dlc;
  std::memcpy(outgoing.data, frame.data, sizeof(outgoing.data));
  const ssize_t written = ::write(sockets_[bus], &outgoing, sizeof(outgoing));
  return written == static_cast<ssize_t>(sizeof(outgoing));
}

std::vector<MotorRecord> RealObserver::collect_replies(
  const std::size_t bus, const std::vector<std::uint16_t> & expected, const int timeout_ms)
{
  std::vector<MotorRecord> records;
  const std::uint64_t deadline = monotonic_ms() + static_cast<std::uint64_t>(timeout_ms);

  while (records.size() < expected.size()) {
    const std::uint64_t now = monotonic_ms();
    if (now >= deadline) {
      break;
    }
    pollfd descriptor{sockets_[bus], POLLIN, 0};
    const int ready = ::poll(&descriptor, 1, static_cast<int>(deadline - now));
    if (ready <= 0) {
      break;
    }

    can_frame incoming{};
    const ssize_t read_bytes = ::read(sockets_[bus], &incoming, sizeof(incoming));
    if (read_bytes != static_cast<ssize_t>(sizeof(incoming))) {
      continue;
    }
    if ((incoming.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)) != 0U || incoming.can_dlc != 8U) {
      continue;
    }

    oa_can_frame frame{};
    frame.struct_size = static_cast<std::uint32_t>(sizeof(frame));
    frame.abi_version = OA_CAN_ABI_VERSION;
    frame.can_id = incoming.can_id & CAN_SFF_MASK;
    frame.dlc = 8U;
    std::memcpy(frame.data, incoming.data, sizeof(frame.data));

    // The decoder demands the identity up front, so take it from the frame we
    // are holding: the reply's own CAN ID and the motor ID in its low nibble.
    // The decoder still rejects the frame if those are internally inconsistent
    // or the status nibble is not a known one.
    const auto motor_id = static_cast<std::uint8_t>(frame.data[0] & 0x0FU);
    oa_can_feedback feedback{};
    feedback.struct_size = static_cast<std::uint32_t>(sizeof(feedback));
    feedback.abi_version = OA_CAN_ABI_VERSION;
    // Position is quantized against +/-12.5 rad for every DaMiao type in this
    // family, so the angle is exact regardless of which type answered. Only
    // velocity and torque depend on the type; the sweep uses them for display,
    // and read_once re-decodes with the manifest type once the slot is known.
    if (oa_can_decode_feedback(
        &frame, static_cast<std::uint16_t>(frame.can_id), motor_id,
        OA_CAN_MOTOR_DM8009, &feedback) != OA_CAN_OK)
    {
      continue;
    }

    const auto already = std::find_if(
      records.begin(), records.end(),
      [&](const MotorRecord & record) {return record.receive_id == frame.can_id;});
    if (already != records.end()) {
      continue;
    }

    MotorRecord record;
    record.receive_id = static_cast<std::uint16_t>(frame.can_id);
    // Recover the send ID by inverting the configured offset when that lands
    // on an ID we actually asked about; otherwise report the motor's own ID.
    const auto inverted = static_cast<std::uint16_t>(frame.can_id - config_.receive_id_offset);
    record.send_id =
      std::find(expected.begin(), expected.end(), inverted) != expected.end() ?
      inverted : static_cast<std::uint16_t>(motor_id);
    record.motor_id = feedback.motor_id;
    record.status_nibble = feedback.status_nibble;
    record.position_rad = feedback.position_rad;
    record.velocity_rad_s = feedback.velocity_rad_s;
    record.torque_nm = feedback.torque_nm;
    record.mos_temperature_c = feedback.mos_temperature_c;
    record.rotor_temperature_c = feedback.rotor_temperature_c;
    records.push_back(record);
  }

  std::sort(
    records.begin(), records.end(),
    [](const MotorRecord & a, const MotorRecord & b) {return a.send_id < b.send_id;});
  return records;
}

std::vector<MotorRecord> RealObserver::sweep(const std::size_t bus)
{
  std::vector<MotorRecord> found;
  // One ID at a time. A broadcast sweep would be faster, but a motor
  // commissioned to an unexpected receive ID could then not be attributed to
  // the ID that provoked it.
  for (std::uint16_t send_id = config_.first_send_id; send_id <= config_.last_send_id; ++send_id) {
    if (!request_status(bus, send_id)) {
      continue;
    }
    const std::vector<std::uint16_t> expected{send_id};
    std::vector<MotorRecord> replies = collect_replies(bus, expected, config_.reply_timeout_ms);
    for (MotorRecord & record : replies) {
      record.send_id = send_id;
      found.push_back(record);
    }
  }
  std::sort(
    found.begin(), found.end(),
    [](const MotorRecord & a, const MotorRecord & b) {return a.send_id < b.send_id;});
  return found;
}

void RealObserver::identify_arms()
{
  assignment_ = ArmAssignment{};

  if (!config_.forced_side_for_interface_a.empty()) {
    const bool a_is_left = config_.forced_side_for_interface_a == "left";
    const bool a_is_right = config_.forced_side_for_interface_a == "right";
    if (a_is_left || a_is_right) {
      assignment_.resolved = true;
      assignment_.side_of_interface[0] = a_is_left ? kLeftSide : kRightSide;
      assignment_.side_of_interface[1] = a_is_left ? kRightSide : kLeftSide;
      assignment_.method = "forced";
      assignment_.reason = "assignment pinned by the interface_a_side parameter (" +
        config_.forced_side_for_interface_a + "); no evidence was consulted";
      return;
    }
  }

  const std::array<bool, kBusCount> populated{
    discovered_[0].size() >= kJointsPerArm, discovered_[1].size() >= kJointsPerArm};
  if (!populated[0] && !populated[1]) {
    assignment_.reason =
      "cannot identify arms: no motors answered on either bus. Check that the arms "
      "are powered and that the CAN cables are seated.";
    return;
  }

  // Identification rests on the mirrored joint limits, because that is the only
  // signal that is a property of the arms rather than of how somebody numbered
  // them. Joints 1 and 2 have ranges that are reflections of each other between
  // the sides (left joint 2 spans [-3.316, +0.175], right spans [-0.175,
  // +3.316]), so a real pose almost always sits inside one side's box and
  // outside the other's. Joints 3 to 7 are identical between the sides and
  // contribute nothing either way.
  //
  // Deliberately NOT used as the primary signal: motor IDs. A v1.0 arm answers
  // on 0x01..0x08 and the IDs are unique only within one bus, so a fresh pair of
  // arms puts the same eight IDs on both interfaces and no partition exists. It
  // survives below purely as a tie-break for the unusual case of a pair that has
  // been renumbered into disjoint blocks, and even then it is a convention.
  const auto joints_of = [this](const std::size_t bus) {
      std::array<double, kJointsPerArm> q{};
      for (std::size_t joint = 0; joint < kJointsPerArm; ++joint) {
        q[joint] = discovered_[bus][joint].position_rad;
      }
      return q;
    };

  double a_as_left = 0.0;
  double a_as_right = 0.0;
  std::string coverage;
  if (populated[0] && populated[1]) {
    // Both arms present: score the two whole-pair hypotheses against each other.
    const auto qa = joints_of(0);
    const auto qb = joints_of(1);
    a_as_left = joint_limit_misfit(qa, kLeftSide) + joint_limit_misfit(qb, kRightSide);
    a_as_right = joint_limit_misfit(qa, kRightSide) + joint_limit_misfit(qb, kLeftSide);
    coverage = "both buses";
  } else {
    // One arm present: score that bus on its own. The absent side contributes
    // nothing, so this is the same comparison with one term dropped.
    const std::size_t bus = populated[0] ? 0U : 1U;
    const auto q = joints_of(bus);
    const double as_left = joint_limit_misfit(q, kLeftSide);
    const double as_right = joint_limit_misfit(q, kRightSide);
    a_as_left = bus == 0U ? as_left : as_right;
    a_as_right = bus == 0U ? as_right : as_left;
    coverage = config_.interfaces[bus] + " only";
  }

  // A hair's difference is noise, not evidence. One DaMiao position code is
  // 3.81e-4 rad; require a margin well clear of that before believing it.
  constexpr double kDecisiveMargin = 0.05;
  int limit_vote = 0;   // -1 => interface_a is left, +1 => interface_a is right
  if (std::abs(a_as_left - a_as_right) >= kDecisiveMargin) {
    limit_vote = a_as_left < a_as_right ? -1 : 1;
  }

  bool disjoint = populated[0] && populated[1];
  for (const MotorRecord & left : discovered_[0]) {
    for (const MotorRecord & right : discovered_[1]) {
      if (left.send_id == right.send_id) {
        disjoint = false;
      }
    }
  }
  const int id_vote = disjoint ?
    (discovered_[0].front().send_id < discovered_[1].front().send_id ? -1 : 1) : 0;

  std::ostringstream evidence;
  evidence << std::fixed << std::setprecision(3) << "scored across " << coverage
           << ": joint-limit misfit " << a_as_left << " rad if " << config_.interfaces[0]
           << " is the left arm versus " << a_as_right << " rad if it is the right; motor IDs "
           << (disjoint ? "form disjoint blocks" : "are the same on both buses, as a stock "
    "pair is, so they cannot separate the arms");

  const auto adopt = [&](const int vote, const char * method, const std::string & why) {
      assignment_.resolved = true;
      assignment_.side_of_interface[0] = vote < 0 ? kLeftSide : kRightSide;
      assignment_.side_of_interface[1] = vote < 0 ? kRightSide : kLeftSide;
      assignment_.method = method;
      assignment_.confidence = std::string(method) == "forced" ? "high" : "low";
      assignment_.reason = why;
    };

  if (limit_vote != 0) {
    adopt(limit_vote, "joint-limit-signature",
      "PROVISIONAL GUESS from the mirrored joint-1/joint-2 limits. " + evidence.str() +
      ". Treat this as unverified: it depends on the motor zeros agreeing with the "
      "URDF zeros, and on this hardware they do not, so the guess has been observed "
      "to come out backwards. Confirm it by moving an arm and watching which side "
      "moves on screen, then use Swap arms or set interface_a_side.");
  } else if (id_vote != 0) {
    adopt(id_vote, "motor-id-partition",
      "the measured pose was not decisive, so this falls back to the disjoint motor-ID "
      "blocks, taking the lower block as the left arm. " + evidence.str() +
      ". That is a numbering convention rather than evidence; verify it in RViz and set "
      "interface_a_side if it is backwards.");
  } else {
    assignment_.reason =
      "cannot identify arms: the measured pose fits both sides about equally. " +
      evidence.str() + ". Move the arms into a clearly asymmetric pose and reconnect, "
      "or set the interface_a_side parameter to left or right to pin it.";
  }
}

bool RealObserver::read_once(std::array<BusReading, kBusCount> & out_readings)
{
  if (!connected_) {
    return false;
  }
  const JointMetadata & meta = metadata();
  bool all_complete = true;
  bool any_complete = false;

  for (std::size_t bus = 0; bus < kBusCount; ++bus) {
    BusReading & reading = out_readings[bus];
    reading = BusReading{};
    if (discovered_[bus].size() < kJointsPerArm) {
      // An unpopulated bus is not a failure of the populated one. Leave this
      // side at rest and keep publishing the arm that is actually there.
      continue;
    }

    // Poll everything found, gripper included, so its state is reported even
    // though no URDF joint corresponds to it.
    std::vector<std::uint16_t> expected;
    expected.reserve(discovered_[bus].size());
    for (const MotorRecord & record : discovered_[bus]) {
      expected.push_back(record.send_id);
    }
    // Ask all seven first, then drain. Requesting one at a time would cost a
    // full round trip per joint and skew the arm's joints in time relative to
    // each other, which shows up as a rubbery pose in RViz.
    for (const std::uint16_t send_id : expected) {
      if (!request_status(bus, send_id)) {
        all_complete = false;
      }
    }
    const std::vector<MotorRecord> replies =
      collect_replies(bus, expected, config_.reply_timeout_ms);
    if (replies.size() < kJointsPerArm) {
      all_complete = false;
      detail_ = "bus " + config_.interfaces[bus] + " answered with " +
        std::to_string(replies.size()) + " of " + std::to_string(expected.size()) + " motors";
      continue;
    }
    if (replies.size() > kJointsPerArm) {
      reading.has_gripper = true;
      reading.gripper_rad = replies[kJointsPerArm].position_rad;
    }

    const std::size_t side = assignment_.resolved ?
      assignment_.side_of_interface[bus] : bus;
    for (std::size_t joint = 0; joint < kJointsPerArm; ++joint) {
      reading.position_rad[joint] = replies[joint].position_rad;
      // Velocity and torque scale with the motor type, which the sweep could
      // not know. Re-decode is not possible from the record, so rescale from
      // the DM8009 ranges the sweep assumed to this joint's actual type.
      const std::size_t index = side * kJointsPerArm + joint;
      oa_can_limits assumed{};
      oa_can_limits actual{};
      if (oa_can_motor_limits(OA_CAN_MOTOR_DM8009, &assumed) == OA_CAN_OK &&
        oa_can_motor_limits(meta.motor_type[index], &actual) == OA_CAN_OK)
      {
        reading.velocity_rad_s[joint] = replies[joint].velocity_rad_s *
          (actual.velocity_max_rad_s / assumed.velocity_max_rad_s);
        reading.torque_nm[joint] =
          replies[joint].torque_nm * (actual.torque_max_nm / assumed.torque_max_nm);
      }
    }
    reading.complete = true;
    any_complete = true;
  }

  // A sample is useful when every bus that has motors answered in full. With
  // one arm connected that is one bus; with none it is a failure, not a
  // vacuous success.
  return all_complete && any_complete;
}

void RealObserver::swap_sides()
{
  std::swap(assignment_.side_of_interface[0], assignment_.side_of_interface[1]);
  assignment_.resolved = true;
  assignment_.method = "operator-confirmed";
  assignment_.confidence = "high";
  assignment_.reason = "assignment set by the operator: " + config_.interfaces[0] + " is the " +
    (assignment_.side_of_interface[0] == kLeftSide ? "left" : "right") + " arm and " +
    config_.interfaces[1] + " is the " +
    (assignment_.side_of_interface[1] == kLeftSide ? "left" : "right") + " arm.";
  detail_ = assignment_.reason;
}

std::string RealObserver::status_json() const
{
  std::ostringstream out;
  out << "{\"connected\":" << (connected_ ? "true" : "false")
      << ",\"detail\":" << quote(detail_)
      << ",\"resolved\":" << (assignment_.resolved ? "true" : "false")
      << ",\"method\":" << quote(assignment_.method)
      << ",\"confidence\":" << quote(assignment_.confidence)
      << ",\"reason\":" << quote(assignment_.reason)
      << ",\"buses\":[";
  for (std::size_t bus = 0; bus < kBusCount; ++bus) {
    if (bus != 0U) {
      out << ',';
    }
    const bool resolved = assignment_.resolved;
    out << "{\"interface\":" << quote(config_.interfaces[bus])
        << ",\"side\":"
        << quote(!resolved ? "unknown" :
      (assignment_.side_of_interface[bus] == kLeftSide ? "left" : "right"))
        << ",\"motor_count\":" << discovered_[bus].size()
        << ",\"motors\":[";
    for (std::size_t index = 0; index < discovered_[bus].size(); ++index) {
      const MotorRecord & record = discovered_[bus][index];
      if (index != 0U) {
        out << ',';
      }
      out << "{\"send_id\":" << record.send_id
          << ",\"receive_id\":" << record.receive_id
          << ",\"motor_id\":" << static_cast<unsigned>(record.motor_id)
          << ",\"status\":" << static_cast<unsigned>(record.status_nibble)
          << ",\"position_rad\":" << number(record.position_rad)
          << ",\"mos_c\":" << static_cast<unsigned>(record.mos_temperature_c)
          << ",\"rotor_c\":" << static_cast<unsigned>(record.rotor_temperature_c) << '}';
    }
    out << "]}";
  }
  out << "]}";
  return out.str();
}

}  // namespace openarm_ik_ros::real
