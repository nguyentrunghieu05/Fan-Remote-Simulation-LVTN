"""Scripted serial session with the board, run inside a MobaXterm tab, with screenshots.

  session.py PORT STEP...
    send:<text>          type a console command
    wait:<s>             keep printing board output for s seconds
    until:<text>[:<s>]   print output until <text> appears (timeout s, default 60)
    reset                hardware reset through ST-LINK (STM32_Programmer_CLI -hardRst)
    upload:<bin>:<ver>   release the port, run tools/fwupdate.py --no-install, reopen
    shot:<png>           screenshot of the MobaXterm terminal area
    hold:<s>             keep the tab open s seconds after the last step (default 5)

Everything the board prints is also appended to %TEMP%/hw_session.log,
so the text of each figure can be quoted exactly in the report.
"""
import os, subprocess, sys, tempfile, time
import serial

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
sys.path.insert(0, HERE)
import moba

PORT = sys.argv[1]
seen = bytearray()
log = open(os.path.join(tempfile.gettempdir(), "hw_session.log"), "ab")


def open_port():
    return serial.Serial(PORT, 115200, timeout=0.05)


def pump(p, secs, text=None):
    end = time.time() + secs
    while time.time() < end:
        b = p.read(4096)
        if b:
            seen.extend(b)
            log.write(b)
            sys.stdout.write(b.decode(errors="replace").replace("\r\n", "\n").replace("\r", ""))
            sys.stdout.flush()
        if text and text.encode() in seen:
            return True
    return False


sys.stdout.write("\033[2J\033[3J\033[H")                 # clean screen, no banner
p = open_port()
hold = 5.0
for step in sys.argv[2:]:
    kind, _, arg = step.partition(":")
    if kind == "send":
        p.write((arg + "\r").encode())
        pump(p, 1.5)
    elif kind == "wait":
        pump(p, float(arg))
    elif kind == "until":
        text, t = arg, 60.0
        head, _, tail = arg.rpartition(":")
        if head and tail.replace(".", "").isdigit():
            text, t = head, float(tail)
        del seen[:]
        if not pump(p, t, text):
            print("\n[timeout waiting for %r]" % text)
    elif kind == "reset":
        subprocess.run(["STM32_Programmer_CLI", "-c", "port=SWD", "-hardRst"], capture_output=True, timeout=30)
    elif kind == "upload":
        img, ver = arg.rsplit(":", 1)
        p.close()
        print("\n$ python tools/fwupdate.py %s %s --version %s --no-install" % (PORT, os.path.basename(img), ver))
        r = subprocess.run([sys.executable, "-u", os.path.join(REPO, "tools", "fwupdate.py"), PORT, img,
                            "--version", ver, "--no-install"], capture_output=True, text=True,
                           encoding="utf-8", errors="replace", cwd=REPO, timeout=300)
        lines = [l for l in r.stdout.replace("\r", "\n").split("\n") if l.strip()]
        prog = [l for l in lines if "%" in l and "bytes" in l]
        for l in lines:
            if l in prog and l != prog[-1]:
                continue                                   # keep the last progress line only
            print(l)
        p = open_port()
        pump(p, 0.5)
    elif kind == "shot":
        moba.front()
        moba.shot(arg)
    elif kind == "hold":
        hold = float(arg)
p.close()
log.close()
print("\n[session done]")
time.sleep(hold)
