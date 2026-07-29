// SPDX-License-Identifier: Apache-2.0
#ifndef OPENARM_IK_ROS__RVIZ_CAPTURE_HPP_
#define OPENARM_IK_ROS__RVIZ_CAPTURE_HPP_

#include <X11/Xlib.h>

#include <cstdint>
#include <string>
#include <vector>

namespace openarm_ik_ros::portal
{

class RvizCapture
{
public:
  RvizCapture(std::int64_t pid, std::uint64_t start_ticks);
  ~RvizCapture();
  RvizCapture(const RvizCapture &) = delete;
  RvizCapture & operator=(const RvizCapture &) = delete;

  bool capture_jpeg(std::vector<unsigned char> & jpeg, std::string & reason);
  bool identity_valid() const;
  bool window_ready(std::string & reason) const;

private:
  Window find_window(Window root) const;
  void collect_windows(Window root, Window & candidate, unsigned int & count) const;
  bool window_pid(Window window, unsigned long & pid) const;
  bool is_top_level(Window window) const;
  void release_pixmap();

  std::int64_t pid_;
  std::uint64_t start_ticks_;
  Display * display_{nullptr};
  Atom pid_atom_{None};
  Atom wm_state_atom_{None};
  Window window_{None};
  Window redirected_{None};
  Pixmap pixmap_{None};
};

}  // namespace openarm_ik_ros::portal

#endif  // OPENARM_IK_ROS__RVIZ_CAPTURE_HPP_
