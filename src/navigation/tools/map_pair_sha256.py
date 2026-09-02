#!/usr/bin/python3
"""Print the exact YAML+image SHA256 used to bind a KNOWN_POSE to a map."""
from __future__ import annotations
import argparse
import hashlib
from pathlib import Path
import sys
import yaml


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('map_yaml')
    args=ap.parse_args()
    y=Path(args.map_yaml).expanduser().resolve()
    if not y.is_file():
        print(f'ERROR map YAML not found: {y}',file=sys.stderr); return 2
    doc=yaml.safe_load(y.read_text(encoding='utf-8'))
    if not isinstance(doc,dict) or not doc.get('image'):
        print('ERROR map YAML has no image field',file=sys.stderr); return 2
    image=Path(str(doc['image'])).expanduser()
    if not image.is_absolute(): image=(y.parent/image).resolve()
    if not image.is_file():
        print(f'ERROR map image not found: {image}',file=sys.stderr); return 2
    h=hashlib.sha256()
    for path in (y,image):
        with path.open('rb') as fh:
            for chunk in iter(lambda:fh.read(1024*1024),b''): h.update(chunk)
    print(h.hexdigest())
    print(f'YAML={y}')
    print(f'IMAGE={image}')
    return 0

if __name__=='__main__': sys.exit(main())
