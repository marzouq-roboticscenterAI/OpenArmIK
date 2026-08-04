// SPDX-License-Identifier: Apache-2.0
#include "openarm_ik_ros/rviz_capture.hpp"

#include <X11/Xatom.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/Xcomposite.h>

// Xlib defines Status as a bare `int` macro, which would rewrite every mention
// of RvizCapture::Status below. Nothing here uses Xlib's Status return type.
#ifdef Status
#undef Status
#endif

#include <cstdio>
#include <cstring>
#include <jpeglib.h>

#include <algorithm>
#include <atomic>

namespace openarm_ik_ros
{
namespace
{
// Matches the WM_CLASS the rviz2 executable sets on both its top-level and its
// Ogre render child.
constexpr const char * kRvizClass = "rviz2";

// Xlib's default error handler calls exit(). Walking the window tree races
// every other X client by construction: a window returned by XQueryTree can be
// destroyed before the very next XGetWindowAttributes, which raises BadWindow
// and would take the whole portal down with it. Observed exactly that while
// RViz was still creating and tearing down its startup windows:
//
//   X Error of failed request:  BadWindow (invalid Window parameter)
//   Major opcode of failed request:  3 (X_GetWindowAttributes)
//
// Counting and continuing is correct here. A vanished window is not an error
// condition for a capture that re-locates its target every cycle anyway, and
// the web server must outlive anything that happens to one X window.
std::atomic<unsigned long> g_x_error_count{0};

int handle_x_error(Display *, XErrorEvent *) {
    g_x_error_count.fetch_add(1U, std::memory_order_relaxed);
    return 0;
}

bool window_class_is(Display * display, Window window, const char * expected)
{
  XClassHint hint;
  std::memset(&hint, 0, sizeof(hint));
  if (XGetClassHint(display, window, &hint) == 0) {return false;}
  const bool matched =
    (hint.res_name != nullptr && std::strcmp(hint.res_name, expected) == 0) ||
    (hint.res_class != nullptr && std::strcmp(hint.res_class, expected) == 0);
  if (hint.res_name != nullptr) {XFree(hint.res_name);}
  if (hint.res_class != nullptr) {XFree(hint.res_class);}
  return matched;
}
}  // namespace

class RvizCapture::Impl
{
public:
  Display * display{nullptr};
  Window window{0};
  Pixmap pixmap{0};
  int width{0};
  int height{0};
  // Crop rectangle inside the top-level, taken from the render child so the Qt
  // chrome is excluded.
  int crop_x{0};
  int crop_y{0};
  int crop_width{0};
  int crop_height{0};
  std::uint64_t frames{0};
  std::uint64_t failures{0};
  std::string detail;
  mutable std::mutex mutex;

  ~Impl() {release();}

  void release()
  {
    if (display != nullptr) {
      if (pixmap != 0) {
        XFreePixmap(display, pixmap);
        pixmap = 0;
      }
      if (window != 0) {
        XCompositeUnredirectWindow(display, window, CompositeRedirectAutomatic);
        window = 0;
      }
      XCloseDisplay(display);
      display = nullptr;
    }
  }

  void drop_window()
  {
    if (display != nullptr && pixmap != 0) {
      XFreePixmap(display, pixmap);
    }
    pixmap = 0;
    if (display != nullptr && window != 0) {
      XCompositeUnredirectWindow(display, window, CompositeRedirectAutomatic);
    }
    window = 0;
    width = 0;
    height = 0;
  }

  // Depth-first search for a mapped window whose WM_CLASS is rviz2 and which
  // has a child of the same class. The window manager reparents rviz2 into a
  // frame, so the top-level cannot be assumed to be a direct child of root.
  Window find_rviz(Window root, int depth = 0)
  {
    if (depth > 6) {return 0;}
    Window returned_root = 0;
    Window parent = 0;
    Window * children = nullptr;
    unsigned count = 0;
    if (XQueryTree(display, root, &returned_root, &parent, &children, &count) == 0) {
      return 0;
    }
    Window found = 0;
    for (unsigned index = 0; index < count && found == 0; ++index) {
      const Window candidate = children[index];
      XWindowAttributes attributes;
      if (XGetWindowAttributes(display, candidate, &attributes) != 0 &&
        attributes.map_state == IsViewable && attributes.width > 64 &&
        attributes.height > 64 && window_class_is(display, candidate, kRvizClass))
      {
        found = candidate;
        break;
      }
      found = find_rviz(candidate, depth + 1);
    }
    if (children != nullptr) {XFree(children);}
    return found;
  }

  // The largest same-class child is the Ogre render widget. Its geometry is the
  // crop rectangle; if there is no such child the whole window is used.
  void compute_crop()
  {
    crop_x = 0;
    crop_y = 0;
    crop_width = width;
    crop_height = height;
    Window returned_root = 0;
    Window parent = 0;
    Window * children = nullptr;
    unsigned count = 0;
    if (XQueryTree(display, window, &returned_root, &parent, &children, &count) == 0) {
      return;
    }
    long best_area = 0;
    for (unsigned index = 0; index < count; ++index) {
      XWindowAttributes attributes;
      if (XGetWindowAttributes(display, children[index], &attributes) == 0) {continue;}
      if (attributes.map_state != IsViewable) {continue;}
      const long area = static_cast<long>(attributes.width) * attributes.height;
      if (area <= best_area) {continue;}
      best_area = area;
      crop_x = attributes.x;
      crop_y = attributes.y;
      crop_width = attributes.width;
      crop_height = attributes.height;
    }
    if (children != nullptr) {XFree(children);}
    // Keep the crop inside the captured image no matter what geometry reports.
    crop_x = std::max(0, std::min(crop_x, width - 1));
    crop_y = std::max(0, std::min(crop_y, height - 1));
    crop_width = std::max(1, std::min(crop_width, width - crop_x));
    crop_height = std::max(1, std::min(crop_height, height - crop_y));
  }
};

RvizCapture::RvizCapture()
: impl_(new Impl())
{
}

RvizCapture::~RvizCapture()
{
  delete impl_;
}

bool RvizCapture::connect()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->display != nullptr) {return true;}
  // Installed before the first request so no window-tree race can abort the
  // process. XSetErrorHandler is process-global by Xlib's design.
  static std::once_flag error_handler_once;
  std::call_once(error_handler_once, []() {XSetErrorHandler(&handle_x_error);});
  impl_->display = XOpenDisplay(nullptr);
  if (impl_->display == nullptr) {
    impl_->detail = "cannot open the X display";
    return false;
  }
  int event_base = 0;
  int error_base = 0;
  if (XCompositeQueryExtension(impl_->display, &event_base, &error_base) == 0) {
    impl_->detail = "the X server has no Composite extension";
    XCloseDisplay(impl_->display);
    impl_->display = nullptr;
    return false;
  }
  impl_->detail = "connected";
  return true;
}

bool RvizCapture::locate()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->display == nullptr) {return false;}

  // A window we already hold stays valid until it is unmapped or resized.
  if (impl_->window != 0) {
    XWindowAttributes attributes;
    if (XGetWindowAttributes(impl_->display, impl_->window, &attributes) != 0 &&
      attributes.map_state == IsViewable)
    {
      if (attributes.width == impl_->width && attributes.height == impl_->height) {
        return true;
      }
      // Resized: the named pixmap refers to the old geometry and must be
      // recreated, otherwise capture silently keeps the stale size.
      if (impl_->pixmap != 0) {
        XFreePixmap(impl_->display, impl_->pixmap);
        impl_->pixmap = 0;
      }
      impl_->width = attributes.width;
      impl_->height = attributes.height;
      impl_->pixmap = XCompositeNameWindowPixmap(impl_->display, impl_->window);
      impl_->compute_crop();
      return impl_->pixmap != 0;
    }
    impl_->drop_window();
  }

  const Window root = DefaultRootWindow(impl_->display);
  const Window found = impl_->find_rviz(root);
  if (found == 0) {
    impl_->detail = "no mapped rviz2 window";
    return false;
  }
  XWindowAttributes attributes;
  if (XGetWindowAttributes(impl_->display, found, &attributes) == 0) {
    impl_->detail = "rviz2 window vanished while being inspected";
    return false;
  }
  // Automatic redirection keeps RViz on screen as usual while also giving the
  // server an offscreen copy, so capture keeps working when the window is
  // obscured or moved off screen.
  XCompositeRedirectWindow(impl_->display, found, CompositeRedirectAutomatic);
  XSync(impl_->display, False);
  impl_->window = found;
  impl_->width = attributes.width;
  impl_->height = attributes.height;
  impl_->pixmap = XCompositeNameWindowPixmap(impl_->display, found);
  impl_->compute_crop();
  impl_->detail = impl_->pixmap != 0 ? "tracking rviz2" : "cannot name the window pixmap";
  return impl_->pixmap != 0;
}

bool RvizCapture::grab(std::vector<unsigned char> & jpeg, const int quality)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->display == nullptr || impl_->window == 0) {return false;}
  const Drawable source = impl_->pixmap != 0 ? impl_->pixmap : impl_->window;
  XImage * const image = XGetImage(
    impl_->display, source, impl_->crop_x, impl_->crop_y,
    static_cast<unsigned>(impl_->crop_width), static_cast<unsigned>(impl_->crop_height),
    AllPlanes, ZPixmap);
  if (image == nullptr) {
    ++impl_->failures;
    impl_->detail = "XGetImage failed";
    return false;
  }

  const int width = image->width;
  const int height = image->height;
  std::vector<unsigned char> rgb(static_cast<std::size_t>(width) * height * 3u);

  // A Pixmap has no associated visual, so XGetImage leaves the RGB masks zero
  // even though the data is ordinary TrueColor. Using them unchecked yields
  // (pixel & 0) for every channel, i.e. a perfectly black frame. Fall back to
  // the standard depth-24 layout when the server reports nothing.
  unsigned long red_mask = image->red_mask != 0 ? image->red_mask : 0x00ff0000ul;
  unsigned long green_mask = image->green_mask != 0 ? image->green_mask : 0x0000ff00ul;
  unsigned long blue_mask = image->blue_mask != 0 ? image->blue_mask : 0x000000fful;
  const auto shift_of = [](unsigned long mask) {
      int shift = 0;
      while (mask != 0 && (mask & 1ul) == 0ul) {
        mask >>= 1;
        ++shift;
      }
      return shift;
    };
  const int red_shift = shift_of(red_mask);
  const int green_shift = shift_of(green_mask);
  const int blue_shift = shift_of(blue_mask);

  // Fast path for the ordinary 24/32-bit little-endian BGRX layout; the generic
  // XGetPixel path is far slower and only needed for unusual visuals.
  const bool packed = (image->bits_per_pixel == 32 || image->bits_per_pixel == 24) &&
    image->byte_order == LSBFirst && red_mask == 0x00ff0000ul &&
    green_mask == 0x0000ff00ul && blue_mask == 0x000000fful;
  const int step = image->bits_per_pixel / 8;
  for (int y = 0; y < height; ++y) {
    unsigned char * out = &rgb[static_cast<std::size_t>(y) * width * 3u];
    if (packed) {
      const unsigned char * in =
        reinterpret_cast<const unsigned char *>(image->data) +
        static_cast<std::size_t>(y) * image->bytes_per_line;
      for (int x = 0; x < width; ++x) {
        out[x * 3 + 0] = in[x * step + 2];
        out[x * 3 + 1] = in[x * step + 1];
        out[x * 3 + 2] = in[x * step + 0];
      }
    } else {
      for (int x = 0; x < width; ++x) {
        const unsigned long pixel = XGetPixel(image, x, y);
        out[x * 3 + 0] = static_cast<unsigned char>((pixel & red_mask) >> red_shift);
        out[x * 3 + 1] = static_cast<unsigned char>((pixel & green_mask) >> green_shift);
        out[x * 3 + 2] = static_cast<unsigned char>((pixel & blue_mask) >> blue_shift);
      }
    }
  }
  XDestroyImage(image);

  jpeg_compress_struct compress;
  jpeg_error_mgr error;
  compress.err = jpeg_std_error(&error);
  jpeg_create_compress(&compress);
  unsigned char * buffer = nullptr;
  unsigned long size = 0;
  jpeg_mem_dest(&compress, &buffer, &size);
  compress.image_width = static_cast<JDIMENSION>(width);
  compress.image_height = static_cast<JDIMENSION>(height);
  compress.input_components = 3;
  compress.in_color_space = JCS_RGB;
  jpeg_set_defaults(&compress);
  jpeg_set_quality(&compress, std::max(1, std::min(quality, 95)), TRUE);
  jpeg_start_compress(&compress, TRUE);
  while (compress.next_scanline < compress.image_height) {
    JSAMPROW row = &rgb[static_cast<std::size_t>(compress.next_scanline) * width * 3u];
    jpeg_write_scanlines(&compress, &row, 1);
  }
  jpeg_finish_compress(&compress);
  jpeg.assign(buffer, buffer + size);
  jpeg_destroy_compress(&compress);
  if (buffer != nullptr) {free(buffer);}

  ++impl_->frames;
  impl_->detail = "streaming";
  return true;
}

RvizCapture::Status RvizCapture::status() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  Status report;
  report.connected = impl_->display != nullptr;
  report.window_found = impl_->window != 0;
  report.window_id = static_cast<unsigned long>(impl_->window);
  report.width = impl_->crop_width;
  report.height = impl_->crop_height;
  report.frames = impl_->frames;
  report.failures = impl_->failures + g_x_error_count.load(std::memory_order_relaxed);
  report.detail = impl_->detail;
  return report;
}

}  // namespace openarm_ik_ros
