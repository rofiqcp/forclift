#!/usr/bin/env python3
"""V55 safe recovery for the two AGV CP210x sensor bridges.

The recovery path is deliberately conservative and per-role:
  * if the target interface already exists, is bound to cp210x and passes
    an open()+termios probe, it does nothing;
  * a stale ttyUSB node that returns EIO is treated as broken, not healthy;
  * bind missing interfaces before considering any unbind;
  * rebind only the target interface (1-2.1.1:1.0 or 1-2.1.4:1.0);
  * if the rebind does not help, issue USBDEVFS_RESET on ONLY the sensor's
    own USB device — never the parent hub;
  * if still broken, toggle authorized 0→1 on ONLY the target device;
  * LAST RESORT: per-port disable/enable on ONLY the sensor's hub port
    (IMU=port 1, LiDAR=port 4) — NEVER unbind the whole hub 1-2.1.

WHOLE-HUB UNBIND IS PROHIBITED.  It can hang the kernel and block all
sensors, requiring a physical unplug/replug.
"""
from __future__ import annotations
import argparse
import errno
import fcntl
import os
from pathlib import Path
import shutil
import subprocess
import sys
import termios
import time
from typing import Iterable, List, Optional

SYS_USB = Path('/sys/bus/usb/devices')
USB_DRIVER = Path('/sys/bus/usb/drivers/usb')
CP210X_DRIVER = Path('/sys/bus/usb/drivers/cp210x')
SYS_TTY = Path('/sys/class/tty')
CP210X_VID = '10c4'
CP210X_PID = 'ea60'
KNOWN_HUB_IDS = {('05e3', '0608'), ('05e3', '0610')}
HELPER_VERSION = 'AGV-SERIAL-RECOVERY-V55'
LOCK_PATH = '/tmp/agv_sensor_recovery.lock'

# Role -> (USB device port, target interface).  Physical topology is the
# authoritative identity for each CP210x-on-this-AGV sensor bridge.  Recovery
# succeeds only when its OWN target interface is present, bound to cp210x and
# I/O-healthy — never when the other sensor's healthy endpoint merely exists.
ROLE_TARGETS = {
    'imu':   ('1-2.1.1', '1-2.1.1:1.0'),
    'lidar': ('1-2.1.4', '1-2.1.4:1.0'),
}
ROLE_RESCAN_ALIASES = {
    'imu':   '/tmp/agv_devices/imu',
    'lidar': '/tmp/agv_devices/lidar',
}


def role_targets(mode: str):
    """Return [(device, interface)] for the requested recovery mode."""
    if mode == 'both':
        return list(ROLE_TARGETS.values())
    return [ROLE_TARGETS[mode]]



def log(msg: str) -> None:
    print(f'[AGV-SERIAL-RECOVERY] {msg}', flush=True)


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding='ascii', errors='ignore').strip().lower()
    except OSError:
        return ''


def write_sysfs(path: Path, value: str) -> bool:
    try:
        path.write_text(value + '\n', encoding='ascii')
        return True
    except OSError:
        return False


def settle(timeout: int = 8) -> None:
    exe = shutil.which('udevadm')
    if exe:
        subprocess.run([exe, 'settle', f'--timeout={timeout}'], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def modprobe_serial() -> None:
    exe = shutil.which('modprobe') or '/sbin/modprobe'
    for module in ('usbserial', 'cp210x'):
        try:
            subprocess.run([exe, module], check=False, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            pass
    settle(3)


def usb_identity(dev: Path) -> tuple[str, str]:
    return read_text(dev / 'idVendor'), read_text(dev / 'idProduct')


def cp210x_usb_devices() -> List[Path]:
    out: List[Path] = []
    for vendor in SYS_USB.glob('*/idVendor') if SYS_USB.exists() else []:
        dev = vendor.parent
        if ':' in dev.name:
            continue
        if usb_identity(dev) == (CP210X_VID, CP210X_PID):
            out.append(dev)
    return sorted(out, key=lambda p: p.name)


def usb_interfaces_for_device(dev: Path) -> List[str]:
    return sorted(p.name for p in SYS_USB.glob(dev.name + ':*') if p.is_dir())


def all_cp210x_interfaces() -> List[str]:
    return sorted({i for d in cp210x_usb_devices() for i in usb_interfaces_for_device(d)})


def iface_bound_to_cp210x(iface: str) -> bool:
    try:
        return (SYS_USB / iface / 'driver').resolve(strict=True).name == 'cp210x'
    except OSError:
        return False


def ttys_for_iface(iface: str) -> List[str]:
    out: List[str] = []
    for tty in SYS_TTY.glob('ttyUSB*'):
        try:
            resolved = (tty / 'device').resolve(strict=True)
        except OSError:
            continue
        if iface in str(resolved) and (Path('/dev') / tty.name).exists():
            out.append('/dev/' + tty.name)
    return sorted(out)


def cp210x_ttys() -> List[str]:
    out: List[str] = []
    for tty in SYS_TTY.glob('ttyUSB*'):
        try:
            driver = (tty / 'device' / 'driver').resolve(strict=True).name
        except OSError:
            continue
        node = Path('/dev') / tty.name
        if driver == 'cp210x' and node.exists():
            out.append(str(node))
    return sorted(set(out))


def probe_tty(path: str) -> tuple[bool, str]:
    """Check whether a tty is usable at the kernel/termios layer.

    A CP210x can remain visible in /dev and sysfs while open() returns EIO.
    Presence alone therefore is not a health check.  EBUSY is treated as
    healthy/owned because a live sensor node uses TIOCEXCL intentionally.
    """
    flags = os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC
    try:
        fd = os.open(path, flags)
    except OSError as exc:
        if exc.errno == errno.EBUSY:
            return True, 'owned-exclusive'
        return False, f'open:{exc.errno}:{exc.strerror}'
    try:
        try:
            attrs = termios.tcgetattr(fd)
        except OSError as exc:
            return False, f'tcgetattr:{exc.errno}:{exc.strerror}'
        # Avoid a last-close HUPCL edge after this health probe.
        try:
            attrs[2] &= ~termios.HUPCL
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except OSError as exc:
            return False, f'tcsetattr:{exc.errno}:{exc.strerror}'
        return True, 'open+termios-ok'
    finally:
        os.close(fd)


def healthy_cp210x_ttys(verbose: bool = False) -> List[str]:
    healthy: List[str] = []
    for tty in cp210x_ttys():
        ok, detail = probe_tty(tty)
        if ok:
            healthy.append(tty)
        elif verbose:
            log(f'BROKEN tty transport {tty}: {detail}')
    return healthy


def wait_for_ttys(required: int, timeout: float) -> List[str]:
    deadline = time.monotonic() + timeout
    previous: List[str] = []
    stable = 0
    while time.monotonic() < deadline:
        current = healthy_cp210x_ttys()
        if len(current) >= required and current == previous:
            stable += 1
        else:
            stable = 1 if len(current) >= required else 0
        if stable >= 2:
            return current
        previous = current
        time.sleep(0.30)
    return healthy_cp210x_ttys()


def power_lock_device(dev: Path) -> None:
    names = [dev.name]
    name = dev.name
    while '.' in name:
        name = name.rsplit('.', 1)[0]
        names.append(name)
    for n in names:
        candidate = SYS_USB / n
        auth = candidate / 'authorized'
        if auth.exists() and read_text(auth) == '0':
            write_sysfs(auth, '1')
        power = candidate / 'power'
        if (power / 'control').exists():
            write_sysfs(power / 'control', 'on')
        if (power / 'autosuspend_delay_ms').exists():
            write_sysfs(power / 'autosuspend_delay_ms', '-1')


def release_owners(devnodes: Iterable[str]) -> None:
    nodes = [n for n in devnodes if Path(n).exists()]
    fuser = shutil.which('fuser')
    if not nodes or not fuser:
        return
    subprocess.run([fuser, '-k', '-TERM', *nodes], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.2)


def bind_missing_interfaces() -> None:
    for iface in all_cp210x_interfaces():
        dev = SYS_USB / iface.split(':', 1)[0]
        power_lock_device(dev)
        if iface_bound_to_cp210x(iface) and ttys_for_iface(iface):
            continue
        if not iface_bound_to_cp210x(iface):
            write_sysfs(CP210X_DRIVER / 'bind', iface)
    settle(5)


def rebind_only_broken_interfaces() -> None:
    for iface in all_cp210x_interfaces():
        nodes = ttys_for_iface(iface)
        usable = False
        details: List[str] = []
        for node in nodes:
            ok, detail = probe_tty(node)
            details.append(f'{node}={detail}')
            if ok:
                usable = True
        if usable:
            continue

        dev = SYS_USB / iface.split(':', 1)[0]
        power_lock_device(dev)
        if nodes:
            log(f'rebinding I/O-broken interface {iface}: ' + ', '.join(details))
        else:
            log(f'rebinding interface {iface}: tty endpoint missing')
        # NOTE: deliberately no release_owners(nodes) here. The real sensor driver
        # has already closed its own failing descriptor before requesting
        # recovery, so fuser-killing a target tty would only damage a valid
        # current-session owner and recreate the STOP/START race.
        was_bound = iface_bound_to_cp210x(iface)
        try:
            if was_bound:
                write_sysfs(CP210X_DRIVER / 'unbind', iface)
                time.sleep(0.45)
        finally:
            # Always attempt to restore driver ownership after an unbind.
            if (SYS_USB / iface).exists() and not iface_bound_to_cp210x(iface):
                write_sysfs(CP210X_DRIVER / 'bind', iface)
    settle(6)


def usbdevfs_reset(dev: Path) -> bool:
    """Reset only the sensor's own USB device via IOCTL_USBDEVFS_RESET.

    This is the software equivalent of a physical replug of that single
    adapter.  It must NEVER touch the parent hub (1-2.1) or any other port.
    """
    try:
        busnum = int((dev / 'busnum').read_text().strip())
        devnum = int((dev / 'devnum').read_text().strip())
    except (OSError, ValueError):
        log(f'usbdevfs_reset: cannot read busnum/devnum for {dev.name}')
        return False
    if busnum <= 0 or devnum <= 0:
        log(f'usbdevfs_reset: target {dev.name} is not fully enumerated (busnum={busnum}, devnum={devnum}); skipping device reset')
        return False
    node = f'/dev/bus/usb/{busnum:03d}/{devnum:03d}'
    if not os.path.exists(node):
        log(f'usbdevfs_reset: usbfs node missing {node}')
        return False
    # Run the potentially blocking usbfs open+ioctl in a short-lived child.
    # A CP210x that disconnects while devnum is still present can otherwise
    # block this helper forever and prevent GUI START from reaching the drivers.
    code = (
        "import fcntl,os,sys; "
        "fd=os.open(sys.argv[1],os.O_RDWR|os.O_NONBLOCK); "
        "fcntl.ioctl(fd,0x5514); os.close(fd)"
    )
    try:
        result = subprocess.run(
            ['/usr/bin/python3', '-c', code, node],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            text=True, timeout=5.0, check=False)
    except subprocess.TimeoutExpired:
        log(f'usbdevfs_reset: timed out after 5s on {node}')
        return False
    if result.returncode != 0:
        detail = (result.stderr or '').strip().splitlines()
        log(f'usbdevfs_reset: failed {node}: {detail[-1] if detail else "unknown error"}')
        return False
    return True


def authorized_toggle(dev: Path) -> bool:
    """De-authorize then re-authorize ONLY the target device."""
    auth = dev / 'authorized'
    if not auth.exists():
        return False
    write_sysfs(auth, '0')
    time.sleep(0.3)
    write_sysfs(auth, '1')
    settle(4)
    return dev.exists()


def parent_hub(dev: Path | str) -> Optional[Path]:
    """Return the direct parent hub for a target USB device name or Path.

    recover_target() carries role identities as strings (for example
    ``1-2.1.1``).  Accept both representations so the final per-port recovery
    cannot crash before cycling the target sensor port.
    """
    name = dev.name if isinstance(dev, Path) else str(dev)
    if '.' in name:
        parent = SYS_USB / name.rsplit('.', 1)[0]
        return parent if parent.exists() else None
    return None


def role_hub_port(device: str) -> Optional[int]:
    """Per-port disable/enable targets only the sensor port: IMU=1, LiDAR=4."""
    mapping = {'1-2.1.1': 1, '1-2.1.4': 4}
    return mapping.get(device)


def per_port_cycle(hub: Optional[Path], port: int) -> bool:
    """Disable then enable a single hub port via the port's 'disable' sysfs
    attribute.  This is the software equivalent of a physical unplug/replug of
    only that port and must NEVER unbind the whole hub."""
    if hub is None:
        return False
    pdisable = hub / f'port{port}' / 'disable'
    if not pdisable.exists():
        return False
    for val in ('1', '0'):
        if not write_sysfs(pdisable, val):
            return False
        time.sleep(0.5 if val == '1' else 1.2)
    settle(6)
    return True


def recover_target(device: str, iface: str) -> bool:
    """Repair exactly one CP210x sensor bridge by physical topology.

    Returns True only when the target interface exists, is bound to cp210x and
    passes an open()+termios I/O probe.  Sibling sensors are never touched.
    """
    # 1) USB authorized (non-destructive).
    dev = SYS_USB / device
    if not dev.exists():
        log(f'role target {device} not present in USB tree; nothing to repair')
        return False
    power_lock_device(dev)

    # 2) interface must be bound to cp210x.
    if not iface_bound_to_cp210x(iface):
        write_sysfs(CP210X_DRIVER / 'bind', iface)
        settle(3)
        if not iface_bound_to_cp210x(iface):
            log(f'role target {iface} still not bound to cp210x after bind attempt')
            return False

    # 3) If the tty exists and is I/O-healthy, the target is ready.
    nodes = ttys_for_iface(iface)
    if nodes:
        healthy = [n for n in nodes if probe_tty(n)[0]]
        if healthy:
            log(f'role target {iface} already I/O-healthy -> {", ".join(healthy)}')
            return True

    # 4) targeted rebind of the broken interface only.
    details = [f'{n}={probe_tty(n)[1]}' for n in nodes]
    log(f'rebinding role target {iface}: ' + (', '.join(details) if details else 'tty endpoint missing'))
    # The actual sensor driver has already closed its own failing descriptor
    # before requesting recovery.  Do NOT fuser-kill a target tty here: that can
    # terminate a valid current-session sensor owner and recreate the STOP/START
    # race this helper is meant to prevent.
    was_bound = iface_bound_to_cp210x(iface)
    try:
        if was_bound:
            write_sysfs(CP210X_DRIVER / 'unbind', iface)
            time.sleep(0.45)
    finally:
        if (SYS_USB / iface).exists() and not iface_bound_to_cp210x(iface):
            write_sysfs(CP210X_DRIVER / 'bind', iface)
    settle(6)

    nodes = ttys_for_iface(iface)
    healthy = [n for n in nodes if probe_tty(n)[0]]
    if healthy:
        log(f'role target {iface} recovered -> {", ".join(healthy)}')
        return True

    # 4b) USBDEVFS_RESET on ONLY the role's own USB device.
    log(f'role target {iface} still broken; attempting USBDEVFS_RESET on {device}')
    if usbdevfs_reset(dev):
        modprobe_serial()
        power_lock_device(dev)
        if not iface_bound_to_cp210x(iface):
            write_sysfs(CP210X_DRIVER / 'bind', iface)
        settle(6)
        nodes = ttys_for_iface(iface)
        healthy = [n for n in nodes if probe_tty(n)[0]]
        if healthy:
            log(f'role target {iface} recovered via targeted USBDEVFS_RESET -> {", ".join(healthy)}')
            return True

    # 5) De-authorize/re-authorize ONLY the target sensor device.
    log(f'role target {iface} still broken; toggling authorized on {device}')
    if authorized_toggle(dev):
        modprobe_serial()
        dev = SYS_USB / device
        power_lock_device(dev)
        if not iface_bound_to_cp210x(iface):
            write_sysfs(CP210X_DRIVER / 'bind', iface)
        settle(6)
        nodes = ttys_for_iface(iface)
        healthy = [n for n in nodes if probe_tty(n)[0]]
        if healthy:
            log(f'role target {iface} recovered via targeted authorized toggle -> {", ".join(healthy)}')
            return True

    # 6) FINAL RESORT (per-port only): disable/enable JUST the sensor port.
    # IMU = parent hub 1-2.1 port 1; LiDAR = port 4. Never unbind 1-2.1.
    port = role_hub_port(device)
    if port:
        log(f'role target {iface} still broken; attempting per-port cycle on 1-2.1 port {port}')
        if per_port_cycle(parent_hub(device), port):
            modprobe_serial()
            power_lock_device(dev)
            if not iface_bound_to_cp210x(iface):
                write_sysfs(CP210X_DRIVER / 'bind', iface)
            settle(6)
            nodes = ttys_for_iface(iface)
            healthy = [n for n in nodes if probe_tty(n)[0]]
            if healthy:
                log(f'role target {iface} recovered via per-port cycle -> {", ".join(healthy)}')
                return True
    return False

def recover_for_mode(mode: str, required: int) -> List[str]:
    """Repair only the role(s) requested, by physical topology.

    Returns the list of healthy cp210x tty endpoints that belong to the
    targeted role(s).  Never reports success based on the sibling sensor's
    healthy endpoint.
    """
    modprobe_serial()
    targets = role_targets(mode)
    ready: List[str] = []
    for device, iface in targets:
        if recover_target(device, iface):
            ready.extend(ttys_for_iface(iface))
    # De-duplicate while preserving order.
    seen = set()
    ready = [t for t in ready if not (t in seen or seen.add(t))]
    if len(ready) >= required:
        log('READY: targeted role(s) recovered -> ' + ', '.join(ready))
    else:
        log(f'PENDING: targeted role recovery incomplete; ready={ready or "none"}')
    return ready



def main() -> int:
    ap = argparse.ArgumentParser(description='Recover AGV CP210x sensor transports safely.')
    ap.add_argument('mode', nargs='?', default='both', choices=['imu', 'lidar', 'both'])
    ap.add_argument('--version', action='store_true')
    args = ap.parse_args()
    if args.version:
        print(HELPER_VERSION)
        return 0
    if os.geteuid() != 0:
        print('[AGV-SERIAL-RECOVERY] root privileges required', file=sys.stderr, flush=True)
        return 77

    lock_fd = os.open(LOCK_PATH, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC, 0o666)
    try:
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            log('another recovery is already running; leaving this request to it')
            # LOCK BUSY != SUCCESS.  Return non-zero so the caller knows
            # its targeted role was NOT verified by this invocation.
            return 2

        required = 2 if args.mode == 'both' else 1
        ttys = recover_for_mode(args.mode, required)
        return 0 if len(ttys) >= required else 1
    finally:
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_UN)
        except OSError:
            pass
        os.close(lock_fd)


if __name__ == '__main__':
    raise SystemExit(main())
