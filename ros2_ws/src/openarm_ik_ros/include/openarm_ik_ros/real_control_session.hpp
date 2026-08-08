// SPDX-License-Identifier: Apache-2.0
#ifndef OPENARM_IK_ROS__REAL_CONTROL_SESSION_HPP_
#define OPENARM_IK_ROS__REAL_CONTROL_SESSION_HPP_

#include "openarm_ik_ros/virtual_control_session.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace openarm_ik_ros::real
{

struct RealControlConfig
{
  // This installed pair was established by physically moving each arm under
  // the passive observer: can0 is robot-right and can1 is robot-left.
  std::array<std::string, 2> interface_for_side{"can1", "can0"};
  // Bit 0 is robot-left and bit 1 is robot-right. Normal bimanual mode is 3.
  std::uint32_t active_side_mask{3U};
  std::string calibration_path;
  std::string gripper_calibration_path;
};

/// Deliberately armed SocketCAN controller for OpenArm v1.0.
///
/// Construction is passive. connect_and_enable() is the only transition that
/// can energise motors; it first proves fresh feedback for IDs 1..8 on both
/// buses and seeds every MIT target from the measured encoder position.
class RealControlSession final : public ControlSession
{
public:
  RealControlSession(
    RealControlConfig config, StateCallback state_callback,
    HealthCallback health_callback);
  ~RealControlSession() override;
  RealControlSession(const RealControlSession &) = delete;
  RealControlSession & operator=(const RealControlSession &) = delete;

  bool reserve(const std::string & owner, std::string & reason) override;
  bool submit(SessionCommand command, std::string & reason) override;
  bool cancel(const std::string & owner) override;
  void release(const std::string & owner, const std::string & reason) override;
  SessionHealth health() const override;
  void close() noexcept override;

  bool connect_and_enable(std::string & detail);
  bool disconnect_and_disable(std::string & detail) noexcept;
  bool stop_motion(std::string & detail) noexcept;
  bool emergency_stop(std::string & detail) noexcept;
  bool clear_emergency_stop(std::string & detail) noexcept;
  bool capture_grippers_closed(std::string & detail);
  bool connected() const noexcept;
  bool armed() const noexcept;
  bool estop_asserted() const noexcept;
  bool busy() const noexcept;
  std::string status_json() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace openarm_ik_ros::real

#endif  // OPENARM_IK_ROS__REAL_CONTROL_SESSION_HPP_
