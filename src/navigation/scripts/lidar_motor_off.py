#!/usr/bin/python3
"""Best-effort YDLIDAR motor OFF helper.

Used after mapping runtime exits as a belt-and-suspenders fallback.  It sends
YDLIDAR STOP (A5 65) and asserts RTS+DTR, matching the C++ driver's active-low
motor-power OFF state.
"""
import fcntl
import os
import struct
import sys
import termios
import time
from pathlib import Path

DEFAULT_PORTS = [
    '/tmp/agv_devices/lidar',
    '/dev/serial/by-path/platform-3610000.usb-usb-0:2.1.2:1.0-port0',
    '/dev/serial/by-path/platform-3610000.usb-usb-0:2.1.4:1.0-port0',
]


def motor_off(path: str) -> bool:
    try:
        target = os.path.realpath(path)
        if not os.path.exists(target):
            return False
        fd = os.open(target, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC)
    except OSError:
        return False

    try:
        attrs = termios.tcgetattr(fd)
        # HUPCL would toggle modem-control lines again on close.  The helper
        # explicitly asserts RTS+DTR for motor OFF, so preserve that state.
        attrs[2] &= ~termios.HUPCL
        if hasattr(termios, 'B230400'):
            attrs[4] = termios.B230400
            attrs[5] = termios.B230400
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        try:
            os.write(fd, b'\xA5\x65')
            termios.tcdrain(fd)
        except OSError:
            pass
        time.sleep(0.20)
        lines = struct.pack('I', termios.TIOCM_RTS | termios.TIOCM_DTR)
        fcntl.ioctl(fd, termios.TIOCMBIS, lines)
        print(f'[lidar_motor_off] OFF OK: {path} -> {target}', flush=True)
        return True
    except OSError as exc:
        print(f'[lidar_motor_off] OFF failed on {target}: {exc}', file=sys.stderr, flush=True)
        return False
    finally:
        os.close(fd)


def main() -> int:
    ports = sys.argv[1:] or DEFAULT_PORTS
    seen = set()
    for port in ports + DEFAULT_PORTS:
        resolved = os.path.realpath(port)
        if resolved in seen:
            continue
        seen.add(resolved)
        if motor_off(port):
            return 0
    print('[lidar_motor_off] no accessible LiDAR serial port', file=sys.stderr, flush=True)
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
