// SPDX-License-Identifier: Apache-2.0
#include <X11/Xlib.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/types.h>

namespace {

using Clock = std::chrono::steady_clock;

class ProcessMonitor {
 public:
  static bool is_running(pid_t pid) {
    std::ifstream stat_file("/proc/" + std::to_string(pid) + "/stat");
    std::string stat;
    if (!stat_file || !std::getline(stat_file, stat)) {
      return false;
    }

    const std::size_t command_end = stat.rfind(") ");
    if (command_end == std::string::npos || command_end + 2 >= stat.size()) {
      return false;
    }
    return stat[command_end + 2] != 'Z';
  }
};

class XDisplay {
 public:
  XDisplay() : display_(XOpenDisplay(nullptr)) {
    if (display_ == nullptr) {
      const char* display_name = std::getenv("DISPLAY");
      throw std::runtime_error(
          "cannot open X display " +
          std::string(display_name == nullptr ? "(unset)" : display_name));
    }
  }

  ~XDisplay() { XCloseDisplay(display_); }

  XDisplay(const XDisplay&) = delete;
  XDisplay& operator=(const XDisplay&) = delete;

  Display* get() const { return display_; }

 private:
  Display* display_;
};

class X11WindowCloser {
 public:
  bool request_close(pid_t pid, std::chrono::duration<double> timeout) {
    const Clock::time_point deadline = Clock::now() +
        std::chrono::duration_cast<Clock::duration>(timeout);
    Window window = None;
    while (ProcessMonitor::is_running(pid) && Clock::now() < deadline) {
      window = find_window(pid);
      if (window != None) {
        break;
      }
      sleep_until_poll(deadline);
    }
    if (window == None) {
      return !ProcessMonitor::is_running(pid);
    }

    const Atom wm_protocols = XInternAtom(display_.get(), "WM_PROTOCOLS", False);
    const Atom wm_delete = XInternAtom(display_.get(), "WM_DELETE_WINDOW", False);
    XEvent event{};
    event.xclient.type = ClientMessage;
    event.xclient.serial = 0;
    event.xclient.send_event = True;
    event.xclient.display = display_.get();
    event.xclient.window = window;
    event.xclient.message_type = wm_protocols;
    event.xclient.format = 32;
    event.xclient.data.l[0] = static_cast<long>(wm_delete);
    event.xclient.data.l[1] = CurrentTime;
    if (XSendEvent(display_.get(), window, False, NoEventMask, &event) == 0) {
      return false;
    }
    XFlush(display_.get());

    while (ProcessMonitor::is_running(pid) && Clock::now() < deadline) {
      sleep_until_poll(deadline);
    }
    return !ProcessMonitor::is_running(pid);
  }

 private:
  std::vector<unsigned long> property_values(Window window, Atom property) const {
    Atom actual_type = None;
    int actual_format = 0;
    unsigned long count = 0;
    unsigned long remaining = 0;
    unsigned char* data = nullptr;
    const int status = XGetWindowProperty(
        display_.get(), window, property, 0, 4096, False, AnyPropertyType,
        &actual_type, &actual_format, &count, &remaining, &data);
    if (status != Success || data == nullptr ||
        (actual_format != 8 && actual_format != 16 && actual_format != 32)) {
      if (data != nullptr) {
        XFree(data);
      }
      return {};
    }

    std::vector<unsigned long> values;
    values.reserve(count);
    if (actual_format == 32) {
      const auto* typed_data = reinterpret_cast<unsigned long*>(data);
      values.assign(typed_data, typed_data + count);
    } else if (actual_format == 16) {
      const auto* typed_data = reinterpret_cast<unsigned short*>(data);
      for (unsigned long index = 0; index < count; ++index) {
        values.push_back(typed_data[index]);
      }
    } else {
      for (unsigned long index = 0; index < count; ++index) {
        values.push_back(data[index]);
      }
    }
    XFree(data);
    return values;
  }

  Window find_window(pid_t pid) const {
    const Atom client_list =
        XInternAtom(display_.get(), "_NET_CLIENT_LIST", False);
    const Atom window_pid = XInternAtom(display_.get(), "_NET_WM_PID", False);
    const Window root = DefaultRootWindow(display_.get());
    for (const unsigned long value : property_values(root, client_list)) {
      const Window window = static_cast<Window>(value);
      const std::vector<unsigned long> pids = property_values(window, window_pid);
      if (!pids.empty() && pids.front() == static_cast<unsigned long>(pid)) {
        return window;
      }
    }
    return None;
  }

  static void sleep_until_poll(Clock::time_point deadline) {
    const Clock::time_point now = Clock::now();
    if (now >= deadline) {
      return;
    }
    const Clock::duration poll = std::chrono::milliseconds(50);
    std::this_thread::sleep_for(std::min(deadline - now, poll));
  }

  XDisplay display_;
};

struct Options {
  pid_t pid;
  std::chrono::duration<double> timeout{3.0};
};

void print_usage(std::ostream& output) {
  output << "Usage: close_rviz_window PID [--timeout SECONDS]\n";
}

Options parse_options(int argc, char* argv[]) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    print_usage(std::cout);
    std::exit(0);
  }
  if (argc != 2 && argc != 4) {
    throw std::invalid_argument("expected PID and optional --timeout SECONDS");
  }

  std::size_t parsed = 0;
  const long long raw_pid = std::stoll(argv[1], &parsed, 10);
  if (parsed != std::string(argv[1]).size() || raw_pid <= 0 ||
      raw_pid > std::numeric_limits<pid_t>::max()) {
    throw std::invalid_argument("PID must be a positive integer");
  }

  Options options{static_cast<pid_t>(raw_pid)};
  if (argc == 4) {
    if (std::string(argv[2]) != "--timeout") {
      throw std::invalid_argument("expected --timeout before SECONDS");
    }
    parsed = 0;
    const double seconds = std::stod(argv[3], &parsed);
    if (parsed != std::string(argv[3]).size() || !std::isfinite(seconds) ||
        seconds < 0.0 || seconds > 3600.0) {
      throw std::invalid_argument("timeout must be between 0 and 3600 seconds");
    }
    options.timeout = std::chrono::duration<double>(seconds);
  }
  return options;
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const Options options = parse_options(argc, argv);
    X11WindowCloser closer;
    return closer.request_close(options.pid, options.timeout) ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "close_rviz_window: " << error.what() << '\n';
    return 2;
  }
}
