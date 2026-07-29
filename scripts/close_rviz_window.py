#!/usr/bin/env python3
"""Ask an X11/XWayland RViz window to close through WM_DELETE_WINDOW."""

from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import os
import sys
import time


Atom = ctypes.c_ulong
Bool = ctypes.c_int
Display = ctypes.c_void_p
Window = ctypes.c_ulong


class ClientMessageData(ctypes.Union):
    _fields_ = [
        ("b", ctypes.c_char * 20),
        ("s", ctypes.c_short * 10),
        ("l", ctypes.c_long * 5),
    ]


class XClientMessageEvent(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int),
        ("serial", ctypes.c_ulong),
        ("send_event", Bool),
        ("display", Display),
        ("window", Window),
        ("message_type", Atom),
        ("format", ctypes.c_int),
        ("data", ClientMessageData),
    ]


class XEvent(ctypes.Union):
    _fields_ = [
        ("xclient", XClientMessageEvent),
        ("pad", ctypes.c_long * 24),
    ]


def process_is_running(pid: int) -> bool:
    try:
        with open(f"/proc/{pid}/stat", encoding="ascii") as stat_file:
            fields = stat_file.read().split()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return False
    return len(fields) > 2 and fields[2] != "Z"


def configure_xlib() -> ctypes.CDLL:
    path = ctypes.util.find_library("X11")
    if path is None:
        raise RuntimeError("libX11 is not installed")
    xlib = ctypes.CDLL(path)
    xlib.XOpenDisplay.argtypes = [ctypes.c_char_p]
    xlib.XOpenDisplay.restype = Display
    xlib.XDefaultRootWindow.argtypes = [Display]
    xlib.XDefaultRootWindow.restype = Window
    xlib.XInternAtom.argtypes = [Display, ctypes.c_char_p, Bool]
    xlib.XInternAtom.restype = Atom
    xlib.XGetWindowProperty.argtypes = [
        Display,
        Window,
        Atom,
        ctypes.c_long,
        ctypes.c_long,
        Bool,
        Atom,
        ctypes.POINTER(Atom),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.c_ulong),
        ctypes.POINTER(ctypes.POINTER(ctypes.c_ubyte)),
    ]
    xlib.XGetWindowProperty.restype = ctypes.c_int
    xlib.XSendEvent.argtypes = [Display, Window, Bool, ctypes.c_long, ctypes.POINTER(XEvent)]
    xlib.XSendEvent.restype = ctypes.c_int
    xlib.XFlush.argtypes = [Display]
    xlib.XFlush.restype = ctypes.c_int
    xlib.XFree.argtypes = [ctypes.c_void_p]
    xlib.XFree.restype = ctypes.c_int
    xlib.XCloseDisplay.argtypes = [Display]
    xlib.XCloseDisplay.restype = ctypes.c_int
    return xlib


def property_values(
    xlib: ctypes.CDLL, display: Display, window: Window, atom: Atom
) -> list[int]:
    actual_type = Atom()
    actual_format = ctypes.c_int()
    count = ctypes.c_ulong()
    remaining = ctypes.c_ulong()
    data = ctypes.POINTER(ctypes.c_ubyte)()
    status = xlib.XGetWindowProperty(
        display,
        window,
        atom,
        0,
        4096,
        0,
        0,
        ctypes.byref(actual_type),
        ctypes.byref(actual_format),
        ctypes.byref(count),
        ctypes.byref(remaining),
        ctypes.byref(data),
    )
    if status != 0 or not data or actual_format.value not in (8, 16, 32):
        if data:
            xlib.XFree(data)
        return []
    try:
        if actual_format.value == 32:
            values = ctypes.cast(data, ctypes.POINTER(ctypes.c_ulong))
        elif actual_format.value == 16:
            values = ctypes.cast(data, ctypes.POINTER(ctypes.c_ushort))
        else:
            values = ctypes.cast(data, ctypes.POINTER(ctypes.c_ubyte))
        return [int(values[index]) for index in range(count.value)]
    finally:
        xlib.XFree(data)


def find_window(xlib: ctypes.CDLL, display: Display, pid: int) -> int | None:
    root = xlib.XDefaultRootWindow(display)
    client_list = xlib.XInternAtom(display, b"_NET_CLIENT_LIST", 0)
    window_pid = xlib.XInternAtom(display, b"_NET_WM_PID", 0)
    for window in property_values(xlib, display, root, client_list):
        values = property_values(xlib, display, Window(window), window_pid)
        if values and values[0] == pid:
            return window
    return None


def request_close(pid: int, timeout: float) -> bool:
    xlib = configure_xlib()
    display = xlib.XOpenDisplay(None)
    if not display:
        raise RuntimeError(f"cannot open X display {os.environ.get('DISPLAY', '(unset)')}")
    try:
        deadline = time.monotonic() + timeout
        window = None
        while process_is_running(pid) and time.monotonic() < deadline:
            window = find_window(xlib, display, pid)
            if window is not None:
                break
            time.sleep(0.05)
        if window is None:
            return not process_is_running(pid)

        wm_protocols = xlib.XInternAtom(display, b"WM_PROTOCOLS", 0)
        wm_delete = xlib.XInternAtom(display, b"WM_DELETE_WINDOW", 0)
        event = XEvent()
        event.xclient.type = 33  # ClientMessage
        event.xclient.serial = 0
        event.xclient.send_event = 1
        event.xclient.display = display
        event.xclient.window = window
        event.xclient.message_type = wm_protocols
        event.xclient.format = 32
        event.xclient.data.l[0] = wm_delete
        event.xclient.data.l[1] = 0  # CurrentTime
        if not xlib.XSendEvent(display, window, 0, 0, ctypes.byref(event)):
            return False
        xlib.XFlush(display)
        while process_is_running(pid) and time.monotonic() < deadline:
            time.sleep(0.05)
        return not process_is_running(pid)
    finally:
        xlib.XCloseDisplay(display)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("pid", type=int)
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()
    try:
        return 0 if request_close(args.pid, args.timeout) else 1
    except RuntimeError as error:
        print(f"close_rviz_window.py: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
