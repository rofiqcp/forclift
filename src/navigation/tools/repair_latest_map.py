#!/usr/bin/env python3
"""Repair /home/otomasi2/forclift/maps/latest_map.txt from an existing valid map.

Priority:
  1. Valid map_*.yaml under /home/otomasi2/forclift/maps
  2. Valid current Map 1/2/3 under src/navigation/maps
  3. Valid maps under src_backup*/navigation/maps, src_*/navigation/maps,
     and install/navigation/share/navigation/maps

A valid map requires a YAML/YML file with positive `resolution:` and an
existing non-empty image referenced by `image:`.
"""

from pathlib import Path
import glob
import os
import re
import sys
import tempfile

WS = Path('/home/otomasi2/forclift')
PERSISTENT = WS / 'maps'
CURRENT = WS / 'src' / 'navigation' / 'maps'
POINTER = PERSISTENT / 'latest_map.txt'


def valid_pair(path: Path):
    if not path.is_file() or path.suffix.lower() not in ('.yaml', '.yml'):
        return None
    try:
        text = path.read_text(encoding='utf-8')
    except OSError:
        return None
    im = re.search(r'^\s*image\s*:\s*(.+?)\s*$', text, flags=re.M)
    rs = re.search(r'^\s*resolution\s*:\s*([0-9eE+\-.]+)', text, flags=re.M)
    if not im or not rs:
        return None
    try:
        if float(rs.group(1)) <= 0.0:
            return None
    except ValueError:
        return None
    image = Path(im.group(1).strip().strip('"\''))
    if not image.is_absolute():
        image = path.parent / image
    try:
        image = image.resolve()
    except OSError:
        return None
    if not image.is_file() or image.stat().st_size <= 0:
        return None
    return image


def newest(paths):
    valid = []
    for p in paths:
        p = Path(p)
        image = valid_pair(p)
        if image is not None:
            try:
                mt = p.stat().st_mtime_ns
            except OSError:
                mt = 0
            valid.append((mt, p.resolve(), image))
    return max(valid, key=lambda x: x[0]) if valid else None


def atomic_write_pointer(target: Path):
    PERSISTENT.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.latest_map.', suffix='.tmp', dir=str(PERSISTENT), text=True)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(str(target.resolve()) + '\n')
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, POINTER)
    except Exception:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


# Existing valid pointer wins.
if POINTER.is_file():
    try:
        target = Path(POINTER.read_text(encoding='utf-8').strip()).expanduser()
    except OSError:
        target = Path('')
    if str(target) and valid_pair(target) is not None:
        print(f'[PASS] latest_map.txt already valid -> {target.resolve()}')
        raise SystemExit(0)

# 1) Persistent runtime maps.
choice = newest(list(PERSISTENT.glob('map_*.yaml')) + list(PERSISTENT.glob('map_*.yml')))

# 2) Current Map 1/2/3.
if choice is None:
    slot_paths = []
    for slot in (1, 2, 3):
        slot_paths.extend([CURRENT / f'map_{slot}.yaml', CURRENT / f'map_{slot}.yml'])
    choice = newest(slot_paths)

# 3) Legacy/backup/install recovery.
if choice is None:
    candidates = []
    patterns = [
        str(WS / 'src_backup*' / 'navigation' / 'maps' / '*.yaml'),
        str(WS / 'src_backup*' / 'navigation' / 'maps' / '*.yml'),
        str(WS / 'src_*' / 'navigation' / 'maps' / '*.yaml'),
        str(WS / 'src_*' / 'navigation' / 'maps' / '*.yml'),
        str(WS / 'install' / 'navigation' / 'share' / 'navigation' / 'maps' / '*.yaml'),
        str(WS / 'install' / 'navigation' / 'share' / 'navigation' / 'maps' / '*.yml'),
    ]
    for pattern in patterns:
        candidates.extend(glob.glob(pattern))
    choice = newest(candidates)

if choice is None:
    print('[FAIL] No valid saved map pair found anywhere.', file=sys.stderr)
    print('Run: ros2 launch navigation map.launch.py, then START -> STOP + SAVE MAP.', file=sys.stderr)
    raise SystemExit(2)

_, yaml_path, image_path = choice
atomic_write_pointer(yaml_path)
print('[PASS] latest map pointer repaired')
print(f'  pointer: {POINTER}')
print(f'  yaml   : {yaml_path}')
print(f'  image  : {image_path}')
