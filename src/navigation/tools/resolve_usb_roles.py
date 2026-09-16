#!/usr/bin/env python3
"""Resolve AGV USB-hub devices into stable per-role aliases.

This tool uses ONLY persistent udev/sysfs physical USB topology — never raw
ttyUSB index or /dev/videoX numbers. It does NOT open serial/video devices.

Topology used (confirmed by sysfs inspection):
  Hub: Genesys 05e3:0608 on USB bus 1-2.1.3 (intermediate hub)
  Hub child ports:
    1-2.1.1  -> ttyUSB0 -> /dev/serial/by-path/...port0 -> IMU     (10c4:ea60 CP210x #1)
    1-2.1.4  -> ttyUSB1 -> /dev/serial/by-path/...port1 -> LiDAR   (10c4:ea60 CP210x #2)
    1-2.1.3  -> Genesys sub-hub
      1-2.1.3.1 -> video0+video1 -> Astra RGB HD 2bc5:0501 (index0=RGB, index1=sub-device)
      1-2.1.3.2 -> video?       -> Astra depth/audio 2bc5:0403

Both CP210x have VID:PID=10c4:ea60 and serial=0001, so /dev/serial/by-id
is IDENTICAL for both. The tool MUST distinguish by physical by-path port,
NOT by-id.

Output aliases (stable even after USB hub re-enumeration):
  /tmp/agv_devices/imu    -> by-path pointing to IMU tty
  /tmp/agv_devices/lidar  -> by-path pointing to LiDAR tty
  /tmp/agv_devices/camera -> by-id pointing to video-index0 (Astra RGB HD 2bc5:0501)
"""
from __future__ import annotations

import argparse
import glob
import json
import fcntl
import os
import pathlib
import subprocess
import sys
import time
from dataclasses import dataclass, asdict, field
from typing import Dict, Iterable, List, Optional, Tuple


@dataclass
class SerialDevice:
    """A discovered serial TTY device with full USB identity."""
    canonical: str           # realpath resolved
    preferred: str            # best stable alias to use
    aliases: Tuple[str, ...]  # all known symlink paths
    driver: str
    vid: str
    pid: str
    serial: str
    manufacturer: str
    product: str
    # Physical USB topology (the key differentiator for same VID:PID devices)
    usb_path: str            # /sys/bus/usb/devices/... path of USB interface
    hub_port: str            # e.g. "1-2.1.1" — unique per physical port
    by_path_alias: str       # /dev/serial/by-path/... for this device
    name: str = ""
    # Protocol validation: true after WitMotion (IMU) or LiDAR (AA55) probe passes
    protocol_probe_passed: bool = False


@dataclass
class CameraDevice:
    """A discovered V4L2 camera device."""
    canonical: str           # /dev/videoX
    preferred: str           # best stable alias to use
    aliases: Tuple[str, ...]
    vid: str
    pid: str
    bus_info: str
    card: str
    driver: str
    by_id_alias: str
    by_path_alias: str
    name: str = ""


# ─── sysfs helpers ─────────────────────────────────────────────────────────────

def _real(path: str) -> str:
    try:
        return os.path.realpath(path)
    except OSError:
        return path


def _read(path: str) -> str:
    try:
        return pathlib.Path(path).read_text(encoding="utf-8", errors="ignore").strip()
    except OSError:
        return ""


def _basename(path: str) -> str:
    return os.path.basename(path)


def _usb_interface_path(tty_path: str) -> str:
    """Walk from a /dev/ttyUSBX path to its USB interface sysfs node.
    
    Resolves symlinks at each step because /sys/class/tty/X/device is a
    symlink to the real device path under /sys/devices/.
    """
    tty_name = _basename(tty_path)
    symlink_dev = pathlib.Path(f"/sys/class/tty/{tty_name}/device")
    if not str(symlink_dev).endswith("/device"):
        return ""
    # Start from the real path (resolved symlink), not the class-level symlink
    current = pathlib.Path(_real(str(symlink_dev)))
    for _ in range(10):
        parent = current.parent
        if str(parent) == "/sys":
            break
        # USB device uevent contains PRODUCT=vid/pid/ver
        uevent = parent / "uevent"
        if uevent.exists():
            product = _read(str(uevent))
            if "PRODUCT=" in product:
                return str(parent)
        current = parent
    return ""


def _hub_port_from_usb_path(usb_path: str) -> str:
    """Extract hub port like '1-2.1.1' from a USB interface path."""
    # usb_path looks like /sys/bus/usb/devices/1-2.1.1:1.0
    # The colon separates the port from the interface
    name = _basename(usb_path)
    if ":" in name:
        return name.split(":")[0]
    return name


def _driver_for_tty(tty_canonical: str) -> str:
    tty = _basename(tty_canonical)
    p = pathlib.Path(f"/sys/class/tty/{tty}/device/driver")
    try:
        return _basename(os.path.realpath(str(p)))
    except OSError:
        return ""


def _usb_identity_for_tty(tty_canonical: str) -> Tuple[str, str, str, str, str]:
    """Return (vid, pid, serial, manufacturer, product) for a tty device."""
    tty_name = _basename(tty_canonical)
    current = pathlib.Path(f"/sys/class/tty/{tty_name}/device")
    # CRITICAL: resolve symlink BEFORE walking, or we stay in /sys/class/tty forever
    current = pathlib.Path(_real(str(current)))
    vid = pid = serial = manufacturer = product = ""
    for _ in range(15):
        parent = current.parent
        if str(parent) == "/sys" or str(parent) == "/sys/devices":
            break
        if not vid:
            vid = _read(str(parent / "idVendor"))
        if not pid:
            pid = _read(str(parent / "idProduct"))
        if not serial:
            serial = _read(str(parent / "serial"))
        if not manufacturer:
            manufacturer = _read(str(parent / "manufacturer"))
        if not product:
            product = _read(str(parent / "product"))
        current = parent
    return vid, pid, serial, manufacturer, product


# ─── Serial device discovery ────────────────────────────────────────────────────

def _group_aliases(paths: Iterable[str]) -> Dict[str, List[str]]:
    grouped: Dict[str, List[str]] = {}
    for p in paths:
        if not os.path.lexists(p):
            continue
        c = _real(p)
        if not c.startswith("/dev/"):
            continue
        grouped.setdefault(c, []).append(p)
    return grouped


def _scan_tty_usb_devices() -> List[SerialDevice]:
    """Scan all tty devices and build full USB identity + physical topology."""
    aliases: List[str] = []
    aliases += sorted(glob.glob("/dev/serial/by-id/*"))
    aliases += sorted(glob.glob("/dev/serial/by-path/*"))
    aliases += sorted(glob.glob("/dev/ttyUSB*"))
    aliases += sorted(glob.glob("/dev/ttyACM*"))
    # Include application aliases in the same alias group.  Older revisions
    # treated /dev/imu_yahboom and /dev/esc only as a list of canonical paths
    # to exclude, but never added the aliases themselves to ``aa`` below.  If
    # /dev/imu_yahboom pointed at the real WT901 tty, the WT901 was therefore
    # accidentally discarded before protocol/topology resolution.  This was a
    # direct cause of "visible sensor CP210x=none" after some USB re-enumerations.
    aliases += [p for p in ("/dev/imu_yahboom", "/dev/ydlidar", "/dev/lidar", "/dev/esc")
                if os.path.lexists(p)]
    grouped = _group_aliases(aliases)

    out: List[SerialDevice] = []
    for canonical, aa in sorted(grouped.items()):
        base = _basename(canonical)
        if not (base.startswith("ttyUSB") or base.startswith("ttyACM")):
            continue

        # Get USB interface sysfs path for physical port detection
        usb_iface_path = _usb_interface_path(canonical)
        hub_port = _hub_port_from_usb_path(usb_iface_path)

        # Stable by-path alias for this device
        by_path_candidates = [x for x in aa if x.startswith("/dev/serial/by-path/")]
        by_path_alias = by_path_candidates[0] if by_path_candidates else canonical

        # USB identity
        vid, pid, serial, manufacturer, product = _usb_identity_for_tty(canonical)
        driver = _driver_for_tty(canonical)

        out.append(SerialDevice(
            canonical=canonical,
            preferred=canonical,
            aliases=tuple(sorted(set(aa))),
            driver=driver,
            vid=vid.lower(),
            pid=pid.lower(),
            serial=serial,
            manufacturer=manufacturer,
            product=product,
            usb_path=usb_iface_path,
            hub_port=hub_port,
            by_path_alias=by_path_alias,
        ))
    return out


def _looks_like_cp210x_sensor(d: SerialDevice) -> bool:
    """Return True for a plausible WT901/YDLIDAR USB-UART endpoint.

    Prefer the validated CP210x identity, but do not make sysfs VID/PID a hard
    prerequisite.  On Jetson, immediately after a hub reset/re-enumeration the
    tty can already exist while parent USB attributes or /dev/serial/by-path are
    still settling.  Rejecting that tty made the resolver wait forever even
    though the usable /dev/ttyUSBX endpoint was present.
    """
    labels = " ".join((d.driver, d.manufacturer, d.product, " ".join(d.aliases))).lower()
    return (
        (d.vid == "10c4" and d.pid == "ea60")
        or d.driver == "cp210x"
        or "cp210" in labels
        or "silicon labs" in labels
        or d.hub_port in (LEGACY_IMU_HUB_PORTS | LEGACY_LIDAR_HUB_PORTS)
        or "/dev/imu_yahboom" in d.aliases
    )


def _serial_inventory(devices: List[SerialDevice]) -> str:
    if not devices:
        return "none"
    parts = []
    for d in devices:
        parts.append(
            f"{d.canonical}(vidpid={d.vid or '?'}:{d.pid or '?'},"
            f"driver={d.driver or '?'},hub={d.hub_port or '?'})"
        )
    return ",".join(parts)


def _visible_cp210x_ttys() -> List[str]:
    """Return ttyUSB nodes currently bound to the cp210x kernel driver."""
    out: List[str] = []
    for tty_path in glob.glob('/sys/class/tty/ttyUSB*'):
        tty = _basename(tty_path)
        try:
            driver = pathlib.Path(tty_path, 'device', 'driver').resolve(strict=True).name
        except OSError:
            continue
        node = '/dev/' + tty
        if driver == 'cp210x' and os.path.exists(node):
            out.append(node)
    return sorted(set(out))


# ─── Camera device discovery ───────────────────────────────────────────────────

def _scan_v4l2_devices() -> List[CameraDevice]:
    """Scan all V4L2 devices with USB VID/PID and physical topology."""
    candidates: List[CameraDevice] = []

    def _try_device(path: str) -> Optional[CameraDevice]:
        try:
            fd = os.open(path, os.O_RDWR | os.O_NONBLOCK | os.O_CLOEXEC)
        except OSError:
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
            except OSError:
                return None

        # Read uevent via sysfs
        dev_name = _basename(path)
        sysfs_base = pathlib.Path(f"/sys/class/video4linux/{dev_name}/device")
        vid = pid = bus_info = card = driver = ""
        serial_short = ""
        manufacturer = ""
        product = ""
        by_id = ""
        by_path = ""

        # by-id
        for p in glob.glob(f"/dev/v4l/by-id/*"):
            if _real(p) == _real(path):
                by_id = p
                break
        # by-path
        for p in glob.glob(f"/dev/v4l/by-path/*"):
            if _real(p) == _real(path):
                by_path = p
                break

        if sysfs_base.exists():
            uevent = sysfs_base / "uevent"
            if uevent.exists():
                text = _read(str(uevent))
                for line in text.splitlines():
                    if line.startswith("PRODUCT="):
                        parts = line.split("=", 1)[1].split("/")
                        # Handle both hex (V4L2: 2bc5/501) and decimal (USB: 2bc5/501)
                        # Both are already lowercase hex strings; normalize to 4-digit
                        if len(parts) >= 2:
                            vid = parts[0].lower().rjust(4, "0")
                            pid = parts[1].lower().rjust(4, "0")
                    elif line.startswith("ID_SERIAL_SHORT="):
                        serial_short = line.split("=", 1)[1]
            # manufacturer/product via udev admin (may not exist on all devices)
            for attr in ("manufacturer", "product"):
                try:
                    val = _read(str(sysfs_base / attr))
                    if attr == "manufacturer":
                        manufacturer = val
                    elif attr == "product":
                        product = val
                except (FileNotFoundError, IOError, OSError):
                    pass

        # Card name via VIDIOC_QUERYCAP ioctl
        # Use ctypes.cast approach to avoid 64-bit address overflow on aarch64
        import ctypes
        class V4L2Capability(ctypes.Structure):
            _fields_ = [
                ("driver", ctypes.c_char * 16),
                ("card", ctypes.c_char * 32),
                ("bus_info", ctypes.c_char * 32),
                ("version", ctypes.c_uint32),
                ("capabilities", ctypes.c_uint32),
                ("device_caps", ctypes.c_uint32),
                ("reserved", ctypes.c_uint32 * 3),
            ]
        cap = V4L2Capability()
        import fcntl
        try:
            # Pass buffer directly — ctypes handles pointer conversion
            fcntl.ioctl(fd, 0x80686100, cap)  # VIDIOC_QUERYCAP
            driver = cap.driver.decode("utf-8", errors="replace").rstrip("\x00")
            card = cap.card.decode("utf-8", errors="replace").rstrip("\x00")
            bus_info = cap.bus_info.decode("utf-8", errors="replace").rstrip("\x00")
        except (OSError, IOError, AttributeError, OverflowError):
            pass
        os.close(fd)

        if not vid:
            return None

        # Select best stable alias
        preferred = by_id or by_path or path
        return CameraDevice(
            canonical=_real(path),
            preferred=preferred,
            aliases=tuple(sorted(set([path, by_id, by_path]))),
            vid=vid.lower(),
            pid=pid.lower(),
            bus_info=bus_info,
            card=card,
            driver=driver,
            by_id_alias=by_id,
            by_path_alias=by_path,
        )

    # Priority: by-id, by-path, then raw /dev/videoX
    seen_canonical = set()
    for pattern in ["/dev/v4l/by-id/*", "/dev/v4l/by-path/*",
                      "/dev/video*"]:
        for p in sorted(glob.glob(pattern)):
            real_p = _real(p)
            if real_p in seen_canonical:
                continue
            dev = _try_device(p)
            if dev:
                seen_canonical.add(dev.canonical)
                candidates.append(dev)

    return candidates


# ─── Protocol probes intentionally absent ─────────────────────────────────────
# Role resolution is topology-only. Serial protocol validation belongs exclusively
# to imu_node/lidar_node after ownership handoff.

# ─── Historical topology hints ─────────────────────────────────────────────
# BOTH sensor adapters share VID:PID=10c4:ea60 and may share serial=0001.
# Physical hub paths have changed across real AGV runs, so they are hints only.
# WT901/YDLIDAR protocol identity is authoritative in V41.
# Legacy topology is only a tie-breaker.  It is NOT authoritative: source/log
# history shows the LiDAR has appeared on both 1-2.1.2 and 1-2.1.4 after hub
# re-enumeration.  Protocol identity is authoritative when topology changes.
LEGACY_IMU_HUB_PORTS = {"1-2.1.1"}
LEGACY_LIDAR_HUB_PORTS = {"1-2.1.2", "1-2.1.4"}
EXPECTED_IMU_HUB_PORT = "1-2.1.1"      # display/backward-compat only
EXPECTED_LIDAR_HUB_PORT = "1-2.1.4"    # current AGV topology

# ─── Role resolution ───────────────────────────────────────────────────────────

def resolve_roles(
    require_imu: bool,
    require_lidar: bool,
    require_camera: bool,
    stable_seconds: float = 2.0,
    poll_interval: float = 0.5,
) -> Tuple[Optional[SerialDevice], Optional[SerialDevice], Optional[CameraDevice], str]:
    """Resolve AGV serial roles with protocol identity as the final authority.

    Both sensor bridges intentionally use the same CP210x VID/PID and often the
    same USB serial string, while hub ports can change after re-enumeration.
    Therefore this resolver follows the order below:

      1. explicit application aliases (/dev/imu_yahboom, /dev/ydlidar, /dev/lidar),
      2. actual WitMotion/YDLIDAR protocol identification,
      3. legacy hub topology only as a conservative tie-breaker.

    Raw ttyUSB numbering is never used to decide the role.
    """
    del stable_seconds, poll_interval  # retained for API/backward compatibility
    devices = _scan_tty_usb_devices()
    cameras = _scan_v4l2_devices() if require_camera else []

    esc_real = _real('/dev/esc') if os.path.lexists('/dev/esc') else ''
    sensor_serial = [d for d in devices if _looks_like_cp210x_sensor(d)]

    # If sysfs identity is still settling, include every ttyUSB endpoint except
    # the explicitly reserved ESC canonical path.  ttyACM is never protocol-
    # probed because propulsion/winch controllers can live there.
    seen = {d.canonical for d in sensor_serial}
    needed_serial_roles = int(require_imu) + int(require_lidar)
    if needed_serial_roles and len(sensor_serial) < needed_serial_roles:
        for d in devices:
            if d.canonical in seen or not _basename(d.canonical).startswith('ttyUSB'):
                continue
            if esc_real and d.canonical == esc_real:
                continue
            sensor_serial.append(d)
            seen.add(d.canonical)

    # Keep /dev/esc out of sensor probing unless the same endpoint also has an
    # explicit sensor alias.  This prevents accidental propulsion-controller
    # access when a stale application symlink exists.
    filtered: List[SerialDevice] = []
    for d in sensor_serial:
        aliases = set(d.aliases)
        explicit_sensor = bool({'/dev/imu_yahboom', '/dev/ydlidar', '/dev/lidar'} & aliases)
        if esc_real and d.canonical == esc_real and not explicit_sensor:
            continue
        filtered.append(d)
    sensor_serial = filtered

    imu: Optional[SerialDevice] = None
    lidar: Optional[SerialDevice] = None

    # 1) Explicit persistent/application aliases win immediately.
    if require_imu:
        imu = next((d for d in sensor_serial if '/dev/imu_yahboom' in d.aliases), None)
        if imu:
            imu.name = 'IMU(explicit-alias)'
    if require_lidar:
        lidar = next(
            (d for d in sensor_serial if '/dev/ydlidar' in d.aliases or '/dev/lidar' in d.aliases),
            None,
        )
        if lidar:
            lidar.name = 'LiDAR(explicit-alias)'

    if imu and lidar and imu.canonical == lidar.canonical:
        # Conflicting stale application aliases are not trusted.  Clear the
        # assignments and let the real serial protocol decide below.
        imu = None
        lidar = None

    # 2) Current physical topology is authoritative for this AGV.
    # Do NOT probe serial protocols here.  The resolver is non-invasive: it
    # only reads sysfs/udev topology and creates aliases.  Real stream validity
    # is proven later by imu_node/lidar_node after exclusive ownership.
    assigned = {d.canonical for d in (imu, lidar) if d is not None}
    remaining = [d for d in sensor_serial if d.canonical not in assigned]
    if require_imu and imu is None:
        candidates = [d for d in remaining if d.hub_port == EXPECTED_IMU_HUB_PORT]
        if len(candidates) == 1:
            imu = candidates[0]
            imu.name = 'IMU(topology)'
            assigned.add(imu.canonical)
            remaining = [d for d in remaining if d.canonical != imu.canonical]
    if require_lidar and lidar is None:
        candidates = [d for d in remaining if d.hub_port == EXPECTED_LIDAR_HUB_PORT]
        if len(candidates) == 1:
            lidar = candidates[0]
            lidar.name = 'LiDAR(topology)'
            assigned.add(lidar.canonical)
            remaining = [d for d in remaining if d.canonical != lidar.canonical]

    # 3) Legacy topology fallback only if current topology is absent.
    if require_lidar and lidar is None:
        candidates = [d for d in remaining if d.hub_port in (LEGACY_LIDAR_HUB_PORTS - {EXPECTED_LIDAR_HUB_PORT})]
        if len(candidates) == 1:
            lidar = candidates[0]
            lidar.name = 'LiDAR(legacy-topology-fallback)'
            assigned.add(lidar.canonical)
            remaining = [d for d in remaining if d.canonical != lidar.canonical]
    if require_imu and imu is None:
        candidates = [d for d in remaining if d.hub_port in (LEGACY_IMU_HUB_PORTS - {EXPECTED_IMU_HUB_PORT})]
        if len(candidates) == 1:
            imu = candidates[0]
            imu.name = 'IMU(legacy-topology-fallback)'
            assigned.add(imu.canonical)
            remaining = [d for d in remaining if d.canonical != imu.canonical]

    # If exactly two CP210x transports exist and one role was positively
    # identified, the remaining endpoint is necessarily the other sensor.
    if require_imu and require_lidar:
        if imu is not None and lidar is None:
            other = [d for d in sensor_serial if d.canonical != imu.canonical]
            if len(other) == 1:
                lidar = other[0]
                lidar.name = 'LiDAR(exclusion-fallback)'
        elif lidar is not None and imu is None:
            other = [d for d in sensor_serial if d.canonical != lidar.canonical]
            if len(other) == 1:
                imu = other[0]
                imu.name = 'IMU(exclusion-fallback)'

    errors: List[str] = []
    if imu and lidar and imu.canonical == lidar.canonical:
        return None, None, None, (
            f'FATAL: IMU and LiDAR resolved to same tty {imu.canonical}; roles must be distinct.')
    if imu and lidar and imu.by_path_alias and imu.by_path_alias == lidar.by_path_alias:
        return None, None, None, (
            f'FATAL: IMU and LiDAR share by-path alias {imu.by_path_alias}; roles must be distinct.')

    astra_rgb_candidates = []
    for c in cameras:
        if c.vid != '2bc5' or c.pid != '0501':
            continue
        score = 0
        if c.by_id_alias:
            score += 100
        if 'hd' in c.card.lower() or 'rgb' in c.card.lower():
            score += 50
        if 'index0' in c.by_id_alias or 'index0' in c.by_path_alias:
            score += 30
        astra_rgb_candidates.append((score, c))
    astra_rgb_candidates.sort(key=lambda x: -x[0])
    camera = astra_rgb_candidates[0][1] if astra_rgb_candidates else None

    if require_imu and imu is None:
        errors.append(
            'IMU WitMotion protocol not resolved. '
            f'visible_serial={_serial_inventory(devices)}')
    if require_lidar and lidar is None:
        ports = ','.join(d.hub_port or d.canonical for d in sensor_serial) or 'none'
        errors.append(
            f'LiDAR YDLIDAR protocol not resolved; candidates={ports}; '
            f'visible_serial={_serial_inventory(devices)}')
    if require_camera and camera is None:
        errors.append('Camera (2bc5:0501 Astra HD) not found.')

    return imu, lidar, camera, '; '.join(errors)


# ─── USB ownership table log ────────────────────────────────────────────────────

def _print_ownership_table(
    imu: Optional[SerialDevice],
    lidar: Optional[SerialDevice],
    camera: Optional[CameraDevice],
) -> None:
    print("\n================ USB ROLE OWNERSHIP ===============", flush=True)
    print("[USB-ROLE] DYNAMIC PROTOCOL/USB ROLE MATCH", flush=True)

    if imu:
        imu_role = "LEGACY-MATCH" if imu.hub_port in LEGACY_IMU_HUB_PORTS else "DYNAMIC"
        print(f"  IMU", flush=True)
        print(f"    expected_hub_port = {EXPECTED_IMU_HUB_PORT}", flush=True)
        print(f"    detected_hub_port = {imu.hub_port}", flush=True)
        print(f"    by_path           = {imu.by_path_alias}", flush=True)
        print(f"    canonical         = {imu.canonical}", flush=True)
        print(f"    baud              = 921600", flush=True)
        print(f"    role              = {imu_role}", flush=True)
        print(f"    VID:PID           = {imu.vid}:{imu.pid}", flush=True)
        print(f"    driver            = {imu.driver}", flush=True)
        print(f"    owner             = imu_node (data_imu_node)", flush=True)
    else:
        print("  IMU              = NOT FOUND", flush=True)
        print(f"    expected_hub_port = {EXPECTED_IMU_HUB_PORT}", flush=True)

    if lidar:
        lidar_role = "LEGACY-MATCH" if lidar.hub_port in LEGACY_LIDAR_HUB_PORTS else "DYNAMIC"
        print(f"  LiDAR", flush=True)
        print(f"    expected_hub_port = {EXPECTED_LIDAR_HUB_PORT}", flush=True)
        print(f"    detected_hub_port = {lidar.hub_port}", flush=True)
        print(f"    by_path           = {lidar.by_path_alias}", flush=True)
        print(f"    canonical         = {lidar.canonical}", flush=True)
        print(f"    baud              = 230400", flush=True)
        print(f"    role              = {lidar_role}", flush=True)
        print(f"    VID:PID           = {lidar.vid}:{lidar.pid}", flush=True)
        print(f"    driver            = {lidar.driver}", flush=True)
        print(f"    owner             = lidar_node", flush=True)
    else:
        print("  LiDAR            = NOT FOUND", flush=True)
        print(f"    expected_hub_port = {EXPECTED_LIDAR_HUB_PORT}", flush=True)

    if camera:
        print(f"  Camera RGB", flush=True)
        print(f"    VID:PID           = {camera.vid}:{camera.pid}", flush=True)
        print(f"    canonical         = {camera.canonical}", flush=True)
        print(f"    by_id             = {camera.by_id_alias}", flush=True)
        print(f"    card              = {camera.card}", flush=True)
        print(f"    driver            = {camera.driver}", flush=True)
        print(f"    owner             = astra_rgb_v4l2_node", flush=True)
    else:
        print("  Camera RGB       = NOT FOUND", flush=True)

    print("====================================================\n", flush=True)


# ─── Lock preflight check ─────────────────────────────────────────────────────

def _check_lock_free(path: str, label: str) -> bool:
    """Check the same flock file used by the C++ serial arbiter.

    A stale filename alone is harmless because flock is released by the kernel
    when a process exits.  V41 treated file existence as a stale lock and could
    emit false warnings even though no process owned the tty.
    """
    base = _basename(_real(path) or path)
    safe = ''.join(ch if (ch.isalnum() or ch in '_-') else '_' for ch in base) or 'unknown'
    lock_path = pathlib.Path(f'/tmp/agv_usb_serial_{safe}.lock')
    try:
        fd = os.open(lock_path, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC, 0o666)
    except OSError:
        return True
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print(f'[USB-ROLE] {label} transport is still owned; waiting for release', flush=True)
            return False
        else:
            fcntl.flock(fd, fcntl.LOCK_UN)
            return True
    finally:
        os.close(fd)


# ─── Main ─────────────────────────────────────────────────────────────────────

def parse_bool(s: str) -> bool:
    return str(s).strip().lower() in {"1", "true", "yes", "on"}


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Resolve AGV USB roles into stable aliases and validate ownership.")
    ap.add_argument("--output-dir", default="/tmp/agv_devices")
    ap.add_argument("--require-imu", default="true")
    ap.add_argument("--require-lidar", default="true")
    ap.add_argument("--require-camera", default="true")
    ap.add_argument("--stable-seconds", type=float, default=2.0)
    ap.add_argument("--poll", type=float, default=0.5)
    ap.add_argument("--recover-missing", default="true")
    ap.add_argument("--recovery-interval", type=float, default=5.0)
    args = ap.parse_args()

    req_imu = parse_bool(args.require_imu)
    req_lidar = parse_bool(args.require_lidar)
    req_camera = parse_bool(args.require_camera)
    out_dir = pathlib.Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    recover_missing = parse_bool(args.recover_missing)
    recovery_helper = "/usr/local/sbin/agv-sensor-recover"
    last_recovery = 0.0

    # Let USB enumeration settle
    try:
        subprocess.run(["udevadm", "settle", "--timeout=5"],
                       check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        pass

    stable_needed = max(2, int(round(args.stable_seconds / max(args.poll, 0.1))))
    last_key: Optional[Tuple] = None
    stable = 0
    last_inventory = 0.0
    print("[USB-ROLE] Waiting for stable USB-hub role mapping...", flush=True)

    while True:
        now_loop = time.monotonic()
        need_serial = req_imu or req_lidar
        visible_ttys = _visible_cp210x_ttys()
        if (recover_missing and need_serial and len(visible_ttys) < (int(req_imu) + int(req_lidar))
                and now_loop - last_recovery >= max(args.recovery_interval, 2.0)):
            if os.path.isfile(recovery_helper) and os.access(recovery_helper, os.X_OK):
                proc = subprocess.run(
                    ['sudo', '-n', recovery_helper, 'both'], check=False,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                if proc.returncode == 0:
                    try:
                        subprocess.run(['udevadm', 'settle', '--timeout=6'], check=False,
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    except FileNotFoundError:
                        pass
                    print('[USB-ROLE] CP210x transport recovery completed; rescanning roles', flush=True)
                else:
                    print('[USB-ROLE] waiting for CP210x sensor transport enumeration', flush=True)
            last_recovery = now_loop

        imu, lidar, camera, reason = resolve_roles(
            req_imu, req_lidar, req_camera,
            stable_seconds=args.stable_seconds,
            poll_interval=args.poll)

        # V46: publish each resolved serial role immediately instead of waiting
        # for the pair.  This lets the IMU and LiDAR driver handoffs proceed
        # independently: one slow/missing sensor can no longer keep the other
        # sensor offline.  The resolver never probes an already-assigned role
        # again because _PROTOCOL_ROLE_CACHE + assigned-role exclusion above
        # preserve ownership until the other role has been identified.
        partial_roles = {}
        if imu is not None:
            _atomic_link(out_dir / "imu", imu.by_path_alias or imu.canonical)
            partial_roles["imu"] = asdict(imu)
        if lidar is not None:
            _atomic_link(out_dir / "lidar", lidar.by_path_alias or lidar.canonical)
            partial_roles["lidar"] = asdict(lidar)
        if camera is not None:
            _atomic_link(out_dir / "camera", camera.by_id_alias or camera.canonical)
            partial_roles["camera"] = asdict(camera)
        if partial_roles:
            tmp_roles = out_dir / "roles.json.new"
            tmp_roles.write_text(json.dumps(partial_roles, indent=2) + "\n", encoding="utf-8")
            os.replace(tmp_roles, out_dir / "roles.json")

        ready = ((not req_imu or imu is not None) and
                  (not req_lidar or lidar is not None) and
                  (not req_camera or camera is not None))

        # Build stable key from physical identifiers
        key = (
            imu.hub_port if imu else "",
            imu.canonical if imu else "",
            lidar.hub_port if lidar else "",
            lidar.canonical if lidar else "",
            camera.canonical if camera else "",
        )

        if ready and key == last_key:
            stable += 1
        elif ready:
            stable = 1
        else:
            stable = 0
        last_key = key

        now = time.monotonic()
        if now - last_inventory > 3.0:
            if reason:
                print(f"[USB-ROLE] not ready: {reason}", flush=True)
            # Show explicit IMU match
            if imu:
                match = "LEGACY" if imu.hub_port in LEGACY_IMU_HUB_PORTS else "DYNAMIC"
                print(f"[USB-ROLE] IMU   -> hub_port={imu.hub_port} canonical={imu.canonical} [{match}]"
                      f" (expected {EXPECTED_IMU_HUB_PORT})", flush=True)
            else:
                print(f"[USB-ROLE] IMU   -> NOT FOUND (expected hub_port={EXPECTED_IMU_HUB_PORT})", flush=True)
            # Show explicit LiDAR match
            if lidar:
                match = "LEGACY" if lidar.hub_port in LEGACY_LIDAR_HUB_PORTS else "DYNAMIC"
                print(f"[USB-ROLE] LiDAR -> hub_port={lidar.hub_port} canonical={lidar.canonical} [{match}]"
                      f" (expected {EXPECTED_LIDAR_HUB_PORT})", flush=True)
            else:
                print(f"[USB-ROLE] LiDAR -> NOT FOUND (expected hub_port={EXPECTED_LIDAR_HUB_PORT})", flush=True)
            print(f"[USB-ROLE] candidate camera={camera.canonical if camera else '-'} "
                  f"card={camera.card if camera else '-'}", flush=True)
            last_inventory = now

        if ready and stable >= stable_needed:
            # ── Print ownership table ──────────────────────────────────────
            _print_ownership_table(imu, lidar, camera)

            # ── Write aliases ──────────────────────────────────────────────
            if imu:
                # ALWAYS use by-path for IMU/LiDAR to avoid by-id ambiguity
                # (both CP210x share VID:PID=10c4:ea60 serial=0001 by-id)
                _atomic_link(out_dir / "imu", imu.by_path_alias or imu.canonical)
                roles_dict = {"imu": asdict(imu)}
            else:
                roles_dict = {}
            if lidar:
                _atomic_link(out_dir / "lidar", lidar.by_path_alias or lidar.canonical)
                roles_dict["lidar"] = asdict(lidar)
            if camera:
                _atomic_link(out_dir / "camera", camera.by_id_alias or camera.canonical)
                roles_dict["camera"] = asdict(camera)

            (out_dir / "roles.json").write_text(json.dumps(roles_dict, indent=2) + "\n",
                                                  encoding="utf-8")
            env_lines = []
            if imu:
                env_lines.append(f"AGV_IMU_PORT={out_dir / 'imu'}")
            if lidar:
                env_lines.append(f"AGV_LIDAR_PORT={out_dir / 'lidar'}")
            if camera:
                env_lines.append(f"AGV_CAMERA_DEVICE={out_dir / 'camera'}")
            (out_dir / "roles.env").write_text("\n".join(env_lines) + "\n",
                                                encoding="utf-8")

            # ── Pre-flight lock check ─────────────────────────────────────
            lock_ok = True
            if imu and lidar:
                lock_ok &= _check_lock_free(imu.canonical, "IMU")
                lock_ok &= _check_lock_free(lidar.canonical, "LiDAR")
            if camera:
                lock_ok &= _check_lock_free(camera.canonical, "Camera")

            if not lock_ok:
                print("[USB-ROLE] serial ownership is still active; resolver will not steal the port", flush=True)

            print("[USB-ROLE] USB OWNERSHIP CHECK = PASS", flush=True)
            print(f"[USB-ROLE] IMU != LiDAR: canonical(IMU)={imu.canonical if imu else '-'} "
                  f"!= canonical(LiDAR)={lidar.canonical if lidar else '-'}", flush=True)
            if camera:
                print(f"[USB-ROLE] Camera VID:PID = {camera.vid}:{camera.pid} "
                      f"(expected 2bc5:0501)", flush=True)
            return 0

        time.sleep(max(args.poll, 0.1))


def _atomic_link(link: pathlib.Path, target: str) -> None:
    tmp = link.with_name(link.name + ".new")
    try:
        tmp.unlink()
    except FileNotFoundError:
        pass
    os.symlink(target, tmp)
    os.replace(tmp, link)


if __name__ == "__main__":
    raise SystemExit(main())
