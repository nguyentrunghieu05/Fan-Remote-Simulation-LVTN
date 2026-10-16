#!/usr/bin/env python3
"""Send a new firmware to the fan controller over the console port.

    python tools/fwupdate.py COM5                      # build_app/fan_controller.bin
    python tools/fwupdate.py COM5 path/to/app.bin --version 1.0.1 --no-install

The image must be built for the bootloader layout (make BOOT=1). The board
stores it in the NEW slot of the W25Q64, checks the CRC-32, and on
'fw install' resets into the bootloader, which installs it.

Protocol (see cmd_fw in src/services/console.c): after 'fw recv <size>
<crc32> <a.b.c>' the board answers 0x06 when ready; each packet is
[len lo][len hi][data][CRC-16/MODBUS lo][hi], answered 0x06 (ok), 0x15
(send again) or 0x18 (abort). A packet with len 0 ends the transfer.
Needs pyserial: pip install pyserial
"""
import argparse
import re
import sys
import time
import zlib

import serial

ACK, NAK, CAN = 0x06, 0x15, 0x18
PKT = 256


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def app_version(main_c="src/app/main.c"):
    try:
        with open(main_c, encoding="utf-8") as f:
            m = re.search(r'#define\s+APP_VERSION\s+"(\d+\.\d+\.\d+)"', f.read())
            return m.group(1) if m else None
    except OSError:
        return None


def wait_reply(port, timeout):
    """Return ACK/NAK/CAN, echoing any text the board prints meanwhile."""
    end = time.time() + timeout
    text = bytearray()
    while time.time() < end:
        b = port.read(1)
        if not b:
            continue
        if b[0] in (ACK, NAK, CAN):
            if text.strip():
                print(text.decode(errors="replace").strip())
            return b[0]
        text += b
    if text.strip():
        print(text.decode(errors="replace").strip())
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="serial port of the ST-LINK VCP, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("image", nargs="?", default="build_app/fan_controller.bin")
    ap.add_argument("--version", default=app_version(), help="a.b.c (default: APP_VERSION in main.c)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--no-install", action="store_true", help="only store the image, do not install")
    a = ap.parse_args()

    data = open(a.image, "rb").read()
    if not a.version:
        sys.exit("version unknown: pass --version a.b.c")
    crc = zlib.crc32(data) & 0xFFFFFFFF
    print(f"{a.image}: {len(data)} bytes, CRC-32 {crc:08X}, version {a.version}")

    with serial.Serial(a.port, a.baud, timeout=0.1) as port:
        port.reset_input_buffer()
        port.write(b"\r")
        time.sleep(0.2)
        port.reset_input_buffer()
        port.write(f"fw recv {len(data)} {crc:08x} {a.version}\r".encode())
        if wait_reply(port, 5) != ACK:
            sys.exit("board did not answer 'fw recv' (is the firmware new enough?)")

        t0, resent = time.time(), 0
        offsets = list(range(0, len(data), PKT)) + [len(data)]   # last: empty packet = end
        for off in offsets:
            chunk = data[off:off + PKT]
            body = len(chunk).to_bytes(2, "little") + chunk
            pkt = body + crc16_modbus(body).to_bytes(2, "little")
            for attempt in range(6):
                port.write(pkt)
                r = wait_reply(port, 10 if not chunk else 3)
                if r == ACK:
                    break
                if r == CAN:
                    sys.exit(f"board aborted at {off} bytes")
                resent += 1
            else:
                sys.exit(f"no answer at {off} bytes")
            if chunk:
                done = min(off + PKT, len(data))
                print(f"\r{done * 100 // len(data):3d} %  {done}/{len(data)} bytes", end="", flush=True)
        print(f"\nsent in {time.time() - t0:.1f} s, {resent} packet(s) resent")
        time.sleep(0.3)
        print(port.read(port.in_waiting or 1).decode(errors="replace").strip())

        if not a.no_install:
            port.write(b"fw install\r")
            time.sleep(0.3)
            print(port.read(port.in_waiting or 1).decode(errors="replace").strip())
            print("the board resets, the bootloader installs the image; watch the console")


if __name__ == "__main__":
    main()
