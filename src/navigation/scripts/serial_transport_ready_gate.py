#!/usr/bin/python3
"""Wait for stable, correctly bound IMU/LiDAR serial topology without opening it.

V17 deliberately avoids the old one-shot open/close probe.  On this AGV the two
CP210x bridges can enumerate normally, pass a nonblocking open test, and then be
left in a bad UART state by the probe's close before the real sensor driver
claims the port.  The gate is therefore non-invasive: it verifies aliases,
canonical tty identity, permissions and cp210x driver binding only.

The actual sensor drivers now open O_NONBLOCK from the first syscall and own the
port continuously until clean shutdown.  This removes the probe->close->reopen
race that was visible in the 2026-08-19 logs.
"""
import argparse
import os
from pathlib import Path
import time


def parse_bool(value):
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def canonical(path: str):
    try:
        if not os.path.lexists(path):
            return None
        target = os.path.realpath(path)
        return target if os.path.exists(target) else None
    except OSError:
        return None


def tty_health(target: str):
    """Return (healthy, detail) without opening the tty."""
    tty = Path(target).name
    sys_tty = Path('/sys/class/tty') / tty
    if not sys_tty.exists():
        return False, 'tty sysfs settling'
    if not os.access(target, os.R_OK | os.W_OK):
        return False, 'tty permissions settling'

    driver_link = sys_tty / 'device' / 'driver'
    try:
        driver = driver_link.resolve(strict=True).name
    except OSError:
        return False, 'serial driver binding settling'
    if driver != 'cp210x':
        return False, f'unexpected serial driver={driver}'

    # The USB interface can exist while its parent USB device is unauthorized.
    # Walk upward and reject an explicitly unauthorized parent without opening
    # the endpoint or changing any USB state.
    try:
        dev_real = (sys_tty / 'device').resolve(strict=True)
        for parent in [dev_real, *dev_real.parents]:
            auth = parent / 'authorized'
            if auth.exists():
                try:
                    if auth.read_text().strip() == '0':
                        return False, 'USB device not authorized'
                except OSError:
                    pass
                break
    except OSError:
        return False, 'USB topology settling'

    return True, f'{tty}/cp210x ready'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--imu', default='/tmp/agv_devices/imu')
    ap.add_argument('--lidar', default='/tmp/agv_devices/lidar')
    ap.add_argument('--stable-cycles', type=int, default=5)
    ap.add_argument('--require-imu', default='true')
    ap.add_argument('--require-lidar', default='true')
    ap.add_argument('--poll-sec', type=float, default=0.30)
    ap.add_argument('--timeout-sec', type=float, default=0.0,
                    help='0 = wait indefinitely; never launch drivers into a partial topology')
    args = ap.parse_args()

    req_imu = parse_bool(args.require_imu)
    req_lidar = parse_bool(args.require_lidar)
    if not req_imu and not req_lidar:
        print('[SERIAL-READY] nothing required; READY', flush=True)
        return 0

    stable_needed = max(3, args.stable_cycles)
    poll = max(0.10, args.poll_sec)
    deadline = None if args.timeout_sec <= 0 else time.monotonic() + args.timeout_sec
    stable = 0
    previous_pair = None
    last_line = None

    requested = ' + '.join(name for name, enabled in [('IMU', req_imu), ('LiDAR', req_lidar)] if enabled)
    print(f'[SERIAL-READY] waiting for stable {requested} cp210x topology', flush=True)
    while True:
        imu_real = canonical(args.imu)
        lidar_real = canonical(args.lidar)
        pair = (imu_real, lidar_real)
        distinct = bool(imu_real and lidar_real and imu_real != lidar_real)

        if req_imu:
            imu_ok, imu_detail = tty_health(imu_real) if imu_real else (False, 'alias settling')
        else:
            imu_ok, imu_detail = True, 'not-required'
        if req_lidar:
            lidar_ok, lidar_detail = tty_health(lidar_real) if lidar_real else (False, 'alias settling')
        else:
            lidar_ok, lidar_detail = True, 'not-required'

        distinct_ok = distinct if (req_imu and req_lidar) else True
        healthy = imu_ok and lidar_ok and distinct_ok

        if healthy and pair == previous_pair:
            stable += 1
        elif healthy:
            stable = 1
        else:
            stable = 0
        previous_pair = pair if healthy else None

        line = (
            f'imu={imu_real or "WAIT"}({imu_detail}) '
            f'lidar={lidar_real or "WAIT"}({lidar_detail}) '
            f'distinct={int(distinct)} required={requested} stable={stable}/{stable_needed}'
        )
        if line != last_line:
            print('[SERIAL-READY] ' + line, flush=True)
            last_line = line

        if healthy and stable >= stable_needed:
            print(
                f'[SERIAL-READY] READY required={requested} imu={imu_real} lidar={lidar_real}; '
                'aliases stable, distinct, writable and bound to cp210x; no probe open/close performed',
                flush=True)
            return 0

        if deadline is not None and time.monotonic() >= deadline:
            print('[SERIAL-READY] waiting continues: topology is not healthy enough to hand off',
                  flush=True)
            # Keep waiting instead of cycling hardware ownership. Autonomous launch
            # now also uses success-only exit handlers, so an unexpected nonzero
            # process exit can never be interpreted as readiness.
            deadline = time.monotonic() + args.timeout_sec
        time.sleep(poll)


if __name__ == '__main__':
    raise SystemExit(main())
