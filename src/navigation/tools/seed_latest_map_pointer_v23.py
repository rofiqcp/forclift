#!/usr/bin/python3
from pathlib import Path
from datetime import datetime, timezone
import os, re, sys

MAP_DIR = Path('/home/otomasi2/forclift/maps')
POINTER = MAP_DIR / 'latest_map.txt'


def valid_pair(yaml_path: Path):
    if not yaml_path.is_file() or yaml_path.stat().st_size <= 0:
        return None
    try:
        text = yaml_path.read_text(encoding='utf-8')
    except OSError:
        return None
    m = re.search(r'^\s*image\s*:\s*(.+?)\s*$', text, flags=re.MULTILINE)
    r = re.search(r'^\s*resolution\s*:\s*([0-9eE+\-.]+)', text, flags=re.MULTILINE)
    if not m or not r:
        return None
    try:
        if float(r.group(1)) <= 0:
            return None
    except ValueError:
        return None
    image = Path(m.group(1).strip().strip('"\''))
    if not image.is_absolute():
        image = yaml_path.parent / image
    try:
        image = image.resolve()
    except OSError:
        return None
    return image if image.is_file() and image.stat().st_size > 0 else None


def rank(path: Path):
    m = re.fullmatch(r'map_(\d{8})_(\d{6})\.(?:yaml|yml)', path.name, flags=re.I)
    if m:
        try:
            dt = datetime.strptime(m.group(1) + m.group(2), '%Y%m%d%H%M%S')
            return dt.replace(tzinfo=timezone.utc).timestamp()
        except ValueError:
            pass
    return path.stat().st_mtime


def current_pointer():
    try:
        p = Path(POINTER.read_text(encoding='utf-8').strip()).resolve()
    except Exception:
        return None
    if p.parent != MAP_DIR.resolve():
        return None
    return p if valid_pair(p) else None


def atomic_write(path: Path):
    tmp = MAP_DIR / '.latest_map.txt.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(str(path.resolve()) + '\n')
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, POINTER)


def main():
    MAP_DIR.mkdir(parents=True, exist_ok=True)
    current = current_pointer()
    if current:
        print(f'[V23-MAP-POINTER] existing valid pointer={current}')
        return 0
    candidates = []
    for ext in ('*.yaml', '*.yml'):
        for p in MAP_DIR.glob(ext):
            if p.name.startswith('map_') and valid_pair(p):
                candidates.append(p)
    if not candidates:
        print('[V23-MAP-POINTER] no valid persistent map pair to seed; create one with map.launch.py STOP + SAVE MAP', file=sys.stderr)
        return 2
    selected = max(candidates, key=rank)
    atomic_write(selected)
    print(f'[V23-MAP-POINTER] seeded={selected.resolve()} image={valid_pair(selected)}')
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
