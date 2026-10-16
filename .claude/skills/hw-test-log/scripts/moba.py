"""Drive MobaXterm on the desktop and take screenshots of it.

  moba.py front                       bring MobaXterm to the front, maximized
  moba.py type "python session.py .." type a line + Enter into the active tab
  moba.py shot OUT.png [x0 y0 x1 y1]  screenshot of the screen (or a region)

The window is always maximized so the terminal area sits at the same pixels
in every screenshot (see TERM_BOX).
"""
import ctypes, subprocess, sys, time
from ctypes import wintypes
from PIL import ImageGrab

try:
    ctypes.windll.shcore.SetProcessDpiAwareness(2)
except Exception:
    pass
u = ctypes.windll.user32

# terminal area of a maximized MobaXterm window on a 1920x1080 screen at 100 %
TERM_BOX = (226, 146, 1896, 994)


def moba_window():
    out = subprocess.run(["tasklist", "/FO", "CSV", "/NH"], capture_output=True, text=True).stdout
    pids = {int(l.split('","')[1]) for l in out.splitlines() if l.lower().startswith('"mobaxterm')}
    best = []

    @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    def cb(h, _):
        pid = wintypes.DWORD()
        u.GetWindowThreadProcessId(h, ctypes.byref(pid))
        if pid.value in pids and u.IsWindowVisible(h):
            r = wintypes.RECT()
            u.GetWindowRect(h, ctypes.byref(r))
            best.append(((r.right - r.left) * (r.bottom - r.top), h))
        return True
    u.EnumWindows(cb, 0)
    if not best:
        sys.exit("MobaXterm is not running")
    return max(best)[1]


def front():
    """Bring MobaXterm to the front; exit (type nothing) if Windows refuses."""
    h = moba_window()
    if u.IsIconic(h):
        u.ShowWindow(h, 9)
    u.ShowWindow(h, 3)                                   # maximized: fixed layout
    k = ctypes.windll.kernel32
    for _ in range(5):
        fg = u.GetForegroundWindow()
        t_fg = u.GetWindowThreadProcessId(fg, None)
        t_me = k.GetCurrentThreadId()
        u.AttachThreadInput(t_me, t_fg, True)
        u.keybd_event(0x12, 0, 0, 0); u.keybd_event(0x12, 0, 2, 0)
        u.BringWindowToTop(h); u.SetForegroundWindow(h); u.SetFocus(h)
        u.AttachThreadInput(t_me, t_fg, False)
        time.sleep(0.4)
        if u.GetForegroundWindow() == h:
            return h
    sys.exit("MobaXterm could not be brought to the front: nothing typed")


class KI(ctypes.Structure):
    _fields_ = [("wVk", wintypes.WORD), ("wScan", wintypes.WORD), ("dwFlags", wintypes.DWORD),
                ("time", wintypes.DWORD), ("dwExtraInfo", ctypes.c_size_t)]


class INP(ctypes.Structure):
    class _U(ctypes.Union):
        _fields_ = [("ki", KI), ("pad", ctypes.c_byte * 32)]
    _anonymous_ = ("u",)
    _fields_ = [("type", wintypes.DWORD), ("u", _U)]


def _key(vk=0, scan=0, flags=0):
    i = INP(type=1)
    i.ki = KI(vk, scan, flags, 0, 0)
    u.SendInput(1, ctypes.byref(i), ctypes.sizeof(i))


def type_line(text, enter=True):
    for ch in text:
        _key(0, ord(ch), 4); _key(0, ord(ch), 6)       # KEYEVENTF_UNICODE, + KEYUP
        time.sleep(0.03)
    if enter:
        _key(0x0D); _key(0x0D, 0, 2)


def shot(path, box=TERM_BOX):
    time.sleep(0.3)
    ImageGrab.grab(bbox=box, all_screens=True).save(path)


if __name__ == "__main__":
    a = sys.argv[1:]
    front()
    if a[0] == "type":
        type_line(a[1])
    elif a[0] == "shot":
        shot(a[1], tuple(int(v) for v in a[2:6]) if len(a) >= 6 else TERM_BOX)
