#!/usr/bin/env python3
"""Make an X11 window fullscreen by (part of) its title.

Chromium's --kiosk does not go fullscreen with the snap Chromium on
Ubuntu 24.04 (JetPack 7.2): the dashboard opens as a normal window. This asks
the window manager for fullscreen, like wmctrl -b add,fullscreen, but with
libX11 only, so nothing has to be installed.

Usage: x11_fullscreen.py <title substring> [timeout seconds]
Exits 0 when the window was found and asked to go fullscreen, 1 otherwise.
"""
import ctypes
import ctypes.util
import sys
import time

title = sys.argv[1]
timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0

libname = ctypes.util.find_library("X11")
if not libname:
	sys.exit(1)
x = ctypes.CDLL(libname)
x.XOpenDisplay.restype = ctypes.c_void_p
x.XOpenDisplay.argtypes = [ctypes.c_char_p]
x.XDefaultRootWindow.restype = ctypes.c_ulong
x.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
x.XInternAtom.restype = ctypes.c_ulong
x.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
x.XGetWindowProperty.argtypes = [
	ctypes.c_void_p, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_long,
	ctypes.c_long, ctypes.c_int, ctypes.c_ulong,
	ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_int),
	ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong),
	ctypes.POINTER(ctypes.c_void_p),
]
x.XFree.argtypes = [ctypes.c_void_p]
x.XFlush.argtypes = [ctypes.c_void_p]


class ClientMessage(ctypes.Structure):
	_fields_ = [
		("type", ctypes.c_int), ("serial", ctypes.c_ulong),
		("send_event", ctypes.c_int), ("display", ctypes.c_void_p),
		("window", ctypes.c_ulong), ("message_type", ctypes.c_ulong),
		("format", ctypes.c_int), ("data", ctypes.c_long * 5),
	]


class XEvent(ctypes.Union):
	_fields_ = [("xclient", ClientMessage), ("pad", ctypes.c_long * 24)]


x.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int,
                         ctypes.c_long, ctypes.POINTER(XEvent)]

dpy = x.XOpenDisplay(None)
if not dpy:
	sys.exit(1)
root = x.XDefaultRootWindow(dpy)


def atom(name):
	return x.XInternAtom(dpy, name.encode(), 0)


def prop(win, name):
	"""Raw bytes and item count of a window property."""
	actual_type, actual_format = ctypes.c_ulong(), ctypes.c_int()
	nitems, after = ctypes.c_ulong(), ctypes.c_ulong()
	data = ctypes.c_void_p()
	if x.XGetWindowProperty(dpy, win, atom(name), 0, 4096, 0, 0,
	                        ctypes.byref(actual_type), ctypes.byref(actual_format),
	                        ctypes.byref(nitems), ctypes.byref(after),
	                        ctypes.byref(data)) != 0 or not data.value:
		return b"", 0
	# format 32 items are C longs (8 bytes on 64-bit)
	size = nitems.value * (ctypes.sizeof(ctypes.c_long) if actual_format.value == 32
	                       else actual_format.value // 8)
	raw = ctypes.string_at(data.value, size)
	x.XFree(data)
	return raw, nitems.value


def find_window():
	raw, n = prop(root, "_NET_CLIENT_LIST")
	wins = (ctypes.c_ulong * n).from_buffer_copy(raw) if n else []
	for win in wins:
		name, _ = prop(win, "_NET_WM_NAME")
		if title.encode() in name:
			return win
	return None


deadline = time.time() + timeout
win = find_window()
while win is None and time.time() < deadline:
	time.sleep(0.5)
	win = find_window()
if win is None:
	sys.exit(1)

ev = XEvent()
ev.xclient.type = 33  # ClientMessage
ev.xclient.send_event = 1
ev.xclient.window = win
ev.xclient.message_type = atom("_NET_WM_STATE")
ev.xclient.format = 32
ev.xclient.data[0] = 1  # _NET_WM_STATE_ADD
ev.xclient.data[1] = atom("_NET_WM_STATE_FULLSCREEN")
ev.xclient.data[3] = 1  # source: application
# SubstructureRedirectMask | SubstructureNotifyMask
x.XSendEvent(dpy, root, 0, (1 << 20) | (1 << 19), ctypes.byref(ev))
x.XFlush(dpy)
