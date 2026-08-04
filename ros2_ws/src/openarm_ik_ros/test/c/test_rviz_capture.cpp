// SPDX-License-Identifier: Apache-2.0
// A BadWindow must not take the process down.
//
// Walking the X window tree races every other client by construction: an id
// returned by XQueryTree can be destroyed before the very next
// XGetWindowAttributes. Xlib's default error handler calls exit(), so before
// RvizCapture installed its own handler a viewer attaching while rviz2 was
// still creating its startup windows killed the whole portal:
//
//   X Error of failed request:  BadWindow (invalid Window parameter)
//   Major opcode of failed request:  3 (X_GetWindowAttributes)
//
// This exercises that exact request against a deliberately invalid id.
#include "openarm_ik_ros/rviz_capture.hpp"

#include <X11/Xlib.h>

#include <cstdio>
#include <cstdlib>

int main()
{
  openarm_ik_ros::RvizCapture capture;
  if (!capture.connect()) {
    // No X display available here; nothing to assert and nothing to regress.
    std::printf("skipped: %s\n", capture.status().detail.c_str());
    return 0;
  }
  const std::uint64_t before = capture.status().failures;

  Display * const display = XOpenDisplay(nullptr);
  if (display == nullptr) {
    std::printf("skipped: cannot open a second display connection\n");
    return 0;
  }
  // An id that cannot correspond to a live resource. Without the handler
  // installed by connect() above, the XSync below never returns.
  XWindowAttributes attributes;
  const Status queried = XGetWindowAttributes(display, 0x7ffffffeUL, &attributes);
  XSync(display, False);
  XCloseDisplay(display);

  if (queried != 0) {
    std::fprintf(stderr, "expected the invalid window query to fail\n");
    return 1;
  }
  if (capture.status().failures <= before) {
    std::fprintf(stderr, "the swallowed X error was not counted\n");
    return 1;
  }
  // Reaching here at all is the assertion: the default handler would have
  // exited during XSync.
  std::printf("survived a BadWindow; errors counted %llu -> %llu\n",
    static_cast<unsigned long long>(before),
    static_cast<unsigned long long>(capture.status().failures));
  return 0;
}
