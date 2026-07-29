// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/rviz_capture.hpp"

#include "openarm_ik_ros/portal_core.hpp"

#include <X11/extensions/Xcomposite.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <jpeglib.h>

#include <algorithm>
#include <csetjmp>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <utility>

namespace openarm_ik_ros::portal
{
namespace
{
thread_local int x_error_code = 0;

int capture_x_error(Display *, XErrorEvent * event)
{
  x_error_code = event->error_code;
  return 0;
}

class XErrorTrap
{
public:
  explicit XErrorTrap(Display * display)
  : display_(display), previous_(XSetErrorHandler(capture_x_error))
  {
    x_error_code = 0;
  }

  ~XErrorTrap()
  {
    XSync(display_, False);
    XSetErrorHandler(previous_);
  }

  int code()
  {
    XSync(display_, False);
    return x_error_code;
  }

private:
  Display * display_;
  XErrorHandler previous_;
};

struct JpegError
{
  jpeg_error_mgr base;
  std::jmp_buf jump;
};

void jpeg_error_exit(j_common_ptr info)
{
  auto * error = reinterpret_cast<JpegError *>(info->err);
  std::longjmp(error->jump, 1);
}

unsigned char channel(unsigned long pixel, unsigned long mask)
{
  if (mask == 0) {
    return 0;
  }
  unsigned int shift = 0;
  while (((mask >> shift) & 1UL) == 0UL) {
    ++shift;
  }
  const unsigned long maximum = mask >> shift;
  return static_cast<unsigned char>(((pixel & mask) >> shift) * 255UL / maximum);
}
}  // namespace

RvizCapture::RvizCapture(
  std::int64_t pid, std::uint64_t start_ticks, std::string expected_executable)
: pid_(pid), start_ticks_(start_ticks), expected_executable_(std::move(expected_executable))
{
  if (!identity_valid()) {
    throw std::runtime_error("RViz process identity does not match launcher evidence");
  }
  if (XInitThreads() == 0) {
    throw std::runtime_error("X11 thread safety initialization failed");
  }
  display_ = XOpenDisplay(nullptr);
  if (display_ == nullptr) {
    throw std::runtime_error("could not open X11 display");
  }
  int event_base = 0;
  int error_base = 0;
  if (XCompositeQueryExtension(display_, &event_base, &error_base) == 0) {
    XCloseDisplay(display_);
    display_ = nullptr;
    throw std::runtime_error("XComposite is unavailable on this display");
  }
  int composite_major = 0;
  int composite_minor = 0;
  if (XCompositeQueryVersion(display_, &composite_major, &composite_minor) == 0 ||
    !xcomposite_version_supported(composite_major, composite_minor))
  {
    XCloseDisplay(display_);
    display_ = nullptr;
    throw std::runtime_error("XComposite protocol 0.2 or newer is required");
  }
  pid_atom_ = XInternAtom(display_, "_NET_WM_PID", True);
  wm_state_atom_ = XInternAtom(display_, "WM_STATE", True);
  if (pid_atom_ == None || wm_state_atom_ == None) {
    XCloseDisplay(display_);
    display_ = nullptr;
    throw std::runtime_error("window manager does not expose _NET_WM_PID");
  }
}

RvizCapture::~RvizCapture()
{
  release_pixmap();
  if (display_ != nullptr) {
    XCloseDisplay(display_);
  }
}

bool RvizCapture::identity_valid() const
{
  if (!process_identity_matches(pid_, start_ticks_)) {
    return false;
  }
  return process_executable_matches(pid_, expected_executable_);
}

bool RvizCapture::window_ready(std::string & reason) const
{
  if (!identity_valid()) {
    reason = "launcher-owned RViz process identity is invalid";
    return false;
  }
  XErrorTrap trap(display_);
  if (find_window(DefaultRootWindow(display_)) == None || trap.code() != 0) {
    reason = "exactly one mapped top-level launcher-owned RViz window is required";
    return false;
  }
  return true;
}

bool RvizCapture::window_pid(Window window, unsigned long & pid) const
{
  Atom type = None;
  int format = 0;
  unsigned long count = 0;
  unsigned long remaining = 0;
  unsigned char * data = nullptr;
  const int status = XGetWindowProperty(
    display_, window, pid_atom_, 0, 1, False, XA_CARDINAL,
    &type, &format, &count, &remaining, &data);
  if (status != Success || type != XA_CARDINAL || format != 32 || count != 1 || data == nullptr) {
    if (data != nullptr) {XFree(data);}
    return false;
  }
  pid = *reinterpret_cast<unsigned long *>(data);
  XFree(data);
  return true;
}

Window RvizCapture::find_window(Window root) const
{
  Window candidate = None;
  unsigned int count = 0;
  collect_windows(root, candidate, count);
  return count == 1 ? candidate : None;
}

bool RvizCapture::is_top_level(Window window) const
{
  Atom type = None;
  int format = 0;
  unsigned long count = 0;
  unsigned long remaining = 0;
  unsigned char * data = nullptr;
  const int status = XGetWindowProperty(
    display_, window, wm_state_atom_, 0, 2, False, AnyPropertyType,
    &type, &format, &count, &remaining, &data);
  if (data != nullptr) {XFree(data);}
  return status == Success && type != None && format == 32 && count >= 1;
}

void RvizCapture::collect_windows(Window root, Window & candidate, unsigned int & matches) const
{
  unsigned long owner = 0;
  XWindowAttributes attributes{};
  if (window_pid(root, owner) && owner == static_cast<unsigned long>(pid_) &&
    XGetWindowAttributes(display_, root, &attributes) != 0 && attributes.map_state == IsViewable &&
    attributes.width > 100 && attributes.height > 100 && is_top_level(root))
  {
    candidate = root;
    ++matches;
  }
  Window returned_root = None;
  Window parent = None;
  Window * children = nullptr;
  unsigned int count = 0;
  if (XQueryTree(display_, root, &returned_root, &parent, &children, &count) == 0) {
    return;
  }
  for (unsigned int index = 0; index < count; ++index) {
    collect_windows(children[index], candidate, matches);
  }
  if (children != nullptr) {XFree(children);}
}

void RvizCapture::release_pixmap()
{
  if (display_ != nullptr && pixmap_ != None) {
    XErrorTrap trap(display_);
    XFreePixmap(display_, pixmap_);
    (void)trap.code();
    pixmap_ = None;
  }
}

bool RvizCapture::capture_jpeg(std::vector<unsigned char> & jpeg, std::string & reason)
{
  if (!identity_valid()) {
    reason = "launcher-owned RViz process exited or changed identity";
    return false;
  }
  XErrorTrap trap(display_);
  const Window current_window = find_window(DefaultRootWindow(display_));
  if (current_window == None || trap.code() != 0) {
    window_ = None;
    redirected_ = None;
    reason = "exactly one mapped top-level launcher-owned RViz window is required";
    return false;
  }
  if (window_ != current_window) {
    window_ = current_window;
    redirected_ = None;
  }
  XWindowAttributes attributes{};
  unsigned long current_window_pid = 0;
  if (!window_pid(window_, current_window_pid) || current_window_pid != static_cast<unsigned long>(pid_) ||
    !is_top_level(window_) || XGetWindowAttributes(display_, window_, &attributes) == 0 ||
    trap.code() != 0 ||
    attributes.map_state != IsViewable || attributes.width < 1 || attributes.height < 1 ||
    attributes.width > 4096 || attributes.height > 4096)
  {
    window_ = None;
    redirected_ = None;
    reason = "RViz window is unmapped or has invalid dimensions";
    return false;
  }
  release_pixmap();
  if (redirected_ != window_) {
    XCompositeRedirectWindow(display_, window_, CompositeRedirectAutomatic);
    if (trap.code() != 0) {
      window_ = None;
      reason = "XComposite could not redirect the RViz window";
      return false;
    }
    redirected_ = window_;
  }
  pixmap_ = XCompositeNameWindowPixmap(display_, window_);
  if (trap.code() != 0 || pixmap_ == None) {
    release_pixmap();
    window_ = None;
    redirected_ = None;
    reason = "XComposite could not acquire the RViz window pixmap";
    return false;
  }
  XImage * image = XGetImage(
    display_, pixmap_, 0, 0, static_cast<unsigned int>(attributes.width),
    static_cast<unsigned int>(attributes.height), AllPlanes, ZPixmap);
  if (image == nullptr || trap.code() != 0) {
    if (image != nullptr) {XDestroyImage(image);}
    release_pixmap();
    window_ = None;
    redirected_ = None;
    reason = "XComposite RViz pixel readback failed";
    return false;
  }
  std::vector<unsigned char> rgb(
    static_cast<std::size_t>(attributes.width) * static_cast<std::size_t>(attributes.height) * 3U);
  for (int y = 0; y < attributes.height; ++y) {
    for (int x = 0; x < attributes.width; ++x) {
      const unsigned long pixel = XGetPixel(image, x, y);
      const std::size_t offset =
        (static_cast<std::size_t>(y) * static_cast<std::size_t>(attributes.width) +
        static_cast<std::size_t>(x)) * 3U;
      rgb[offset] = channel(pixel, image->red_mask);
      rgb[offset + 1] = channel(pixel, image->green_mask);
      rgb[offset + 2] = channel(pixel, image->blue_mask);
    }
  }
  XDestroyImage(image);
  jpeg_compress_struct compressor{};
  JpegError error{};
  compressor.err = jpeg_std_error(&error.base);
  error.base.error_exit = jpeg_error_exit;
  if (setjmp(error.jump) != 0) {
    jpeg_destroy_compress(&compressor);
    release_pixmap();
    reason = "JPEG encoder rejected the RViz frame";
    return false;
  }
  jpeg_create_compress(&compressor);
  unsigned char * output = nullptr;
  unsigned long output_size = 0;
  jpeg_mem_dest(&compressor, &output, &output_size);
  compressor.image_width = static_cast<JDIMENSION>(attributes.width);
  compressor.image_height = static_cast<JDIMENSION>(attributes.height);
  compressor.input_components = 3;
  compressor.in_color_space = JCS_RGB;
  jpeg_set_defaults(&compressor);
  jpeg_set_quality(&compressor, 75, True);
  jpeg_start_compress(&compressor, True);
  while (compressor.next_scanline < compressor.image_height) {
    JSAMPROW row = rgb.data() +
      static_cast<std::size_t>(compressor.next_scanline) *
      static_cast<std::size_t>(attributes.width) * 3U;
    jpeg_write_scanlines(&compressor, &row, 1);
  }
  jpeg_finish_compress(&compressor);
  jpeg.assign(output, output + output_size);
  std::free(output);
  jpeg_destroy_compress(&compressor);
  release_pixmap();
  return true;
}

}  // namespace openarm_ik_ros::portal
