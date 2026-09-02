#!/usr/bin/python3
"""Migrate valid saved-map pairs into /home/otomasi2/ros/maps.

V21 keeps mapping output outside the source tree.  This helper also recovers
maps from src_backup* directories created by previous clean source installs.
"""
from pathlib import Path
from datetime import datetime, timezone
import re
import shutil

WS = Path('/home/otomasi2/ros')
DEST = WS / 'maps'


def image_for_yaml(yaml_path: Path):
    try:
        text = yaml_path.read_text(encoding='utf-8')
    except OSError:
        return None
    m = re.search(r'^\s*image\s*:\s*(.+?)\s*$', text, re.MULTILINE)
    r = re.search(r'^\s*resolution\s*:\s*([0-9eE+\-.]+)', text, re.MULTILINE)
    if not m or not r:
        return None
    try:
        if float(r.group(1)) <= 0.0:
            return None
    except ValueError:
        return None
    value = m.group(1).strip().strip('"\'')
    image = Path(value) if Path(value).is_absolute() else yaml_path.parent / value
    return image.resolve() if image.exists() else None


def epoch(path: Path):
    m = re.search(r'map_(\d{8})_(\d{6})', path.name)
    if m:
        try:
            dt = datetime.strptime(m.group(1) + m.group(2), '%Y%m%d%H%M%S')
            return dt.replace(tzinfo=timezone.utc).timestamp()
        except ValueError:
            pass
    return path.stat().st_mtime


def main():
    DEST.mkdir(parents=True, exist_ok=True)
    dirs = [WS / 'src' / 'navigation' / 'maps']
    dirs += sorted(WS.glob('src_backup*/navigation/maps'))
    dirs += sorted(WS.glob('src_*/navigation/maps'))
    copied = 0
    valid = []
    seen = set()
    for directory in dirs:
        if not directory.is_dir():
            continue
        for yaml_path in list(directory.glob('*.yaml')) + list(directory.glob('*.yml')):
            try:
                key = str(yaml_path.resolve())
            except OSError:
                continue
            if key in seen:
                continue
            seen.add(key)
            image = image_for_yaml(yaml_path)
            if image is None:
                continue
            dst_yaml = DEST / yaml_path.name
            dst_image = DEST / image.name
            if not dst_yaml.exists():
                shutil.copy2(yaml_path, dst_yaml)
                copied += 1
            if not dst_image.exists():
                shutil.copy2(image, dst_image)
            if image_for_yaml(dst_yaml):
                valid.append(dst_yaml)

    for yaml_path in list(DEST.glob('*.yaml')) + list(DEST.glob('*.yml')):
        if image_for_yaml(yaml_path):
            valid.append(yaml_path)
    valid = list({str(p.resolve()): p for p in valid}.values())
    if not valid:
        print('[V21-MAPS] no valid maps found yet')
        return 0
    latest = max(valid, key=lambda p: (epoch(p), p.stat().st_mtime))
    print(f'[V21-MAPS] migrated={copied} persistent_dir={DEST}')
    print(f'[V21-MAPS] newest={latest}')
    print(f'[V21-MAPS] image={image_for_yaml(latest)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
