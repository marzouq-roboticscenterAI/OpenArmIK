// SPDX-License-Identifier: Apache-2.0
#ifndef OPENARM_IK_ROS__RVIZ_CAPTURE_HPP_
#define OPENARM_IK_ROS__RVIZ_CAPTURE_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace openarm_ik_ros
{

// Captures the live rviz2 window from the X server and encodes it as JPEG.
//
// This streams the real RViz render, not a re-implementation of it. The Ogre
// 3D content is drawn into the rviz2 top-level window, so the top-level is what
// gets captured; the render widget is a child window whose own drawable reads
// back blank, and capturing that child instead yields an empty image. The
// child's geometry is still useful: cropping the parent capture to it removes
// the Qt menu bar, toolbar and status bar, leaving only the 3D view.
//
// Capture is display-only. Nothing here sends input to RViz.
class RvizCapture
{
public:
  struct Status
  {
    bool connected{false};
    bool window_found{false};
    unsigned long window_id{0};
    int width{0};
    int height{0};
    std::uint64_t frames{0};
    std::uint64_t failures{0};
    std::string detail;
  };

  RvizCapture();
  ~RvizCapture();
  RvizCapture(const RvizCapture &) = delete;
  RvizCapture & operator=(const RvizCapture &) = delete;

  // Connects to the X display named by DISPLAY. Safe to call repeatedly.
  bool connect();

  // Finds the rviz2 top-level window and its render child. Re-runs cheaply, so
  // it can be called each cycle to pick up a window that appears later or is
  // replaced. Returns false when no rviz2 window is currently mapped.
  bool locate();

  // Captures one frame and encodes it. Returns false if capture failed; the
  // caller should keep serving the previous frame rather than tearing down.
  bool grab(std::vector<unsigned char> & jpeg, int quality);

  /// A pointer event to replay into the RViz window, in coordinates normalised
  /// to the cropped render area: (0,0) top-left, (1,1) bottom-right.
  struct PointerEvent
  {
    enum class Kind {move, press, release, wheel};
    Kind kind{Kind::move};
    double x{0.0};
    double y{0.0};
    /// X button number for press/release: 1 left, 2 middle, 3 right.
    int button{1};
    /// Wheel notches; positive scrolls up (button 4), negative down (button 5).
    int notches{0};
  };

  /// Replay a pointer event into the RViz window so the browser view can be
  /// orbited, panned and zoomed.
  ///
  /// This uses the XTEST extension, which drives the real X pointer rather than
  /// delivering a synthetic event to one window. Two consequences the caller
  /// must live with: the on-screen cursor visibly moves during a drag, and
  /// whatever window happens to be topmost at the target point would receive
  /// the click. The second is a hazard, not a cosmetic issue -- a click could
  /// land in an unrelated application -- so this refuses to inject unless the
  /// topmost window at the target point really is the RViz window. Returns
  /// false and sets out_reason when it declines.
  bool inject_pointer(const PointerEvent & event, std::string & out_reason);

  Status status() const;

private:
  class Impl;
  Impl * impl_;
};

}  // namespace openarm_ik_ros

#endif  // OPENARM_IK_ROS__RVIZ_CAPTURE_HPP_
