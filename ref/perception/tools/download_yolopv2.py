#!/usr/bin/env python3
"""Unduh dan verifikasi bobot resmi YOLOPv2 untuk backend CPU Mini-PC.

Default target sengaja berada di source tree agar workspace dapat dipindah tanpa
path /home/otomasi/ros yang hard-coded:
    src/perception/models/yolopv2.pt
"""
from __future__ import annotations

import argparse
import hashlib
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

OFFICIAL_URL = "https://github.com/CAIC-AD/YOLOPv2/releases/download/V0.0.1/yolopv2.pt"
EXPECTED_SIZE = 156_380_200
EXPECTED_SHA256 = "f2a8c8374203ae3e67ff9c184e931f763957de92a993b23269e4e721627f1f8c"


def default_target() -> Path:
    return Path(__file__).resolve().parents[1] / "models" / "yolopv2.pt"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate(path: Path, quiet: bool = False) -> bool:
    if not path.is_file():
        if not quiet:
            print(f"[MODEL] belum ada: {path}")
        return False
    size = path.stat().st_size
    if size != EXPECTED_SIZE:
        if not quiet:
            print(f"[MODEL] ukuran salah: {size} byte; expected {EXPECTED_SIZE}")
        return False
    actual_hash = sha256_file(path)
    if actual_hash.lower() != EXPECTED_SHA256:
        if not quiet:
            print(f"[MODEL] SHA-256 salah: {actual_hash}")
        return False
    if not quiet:
        print(f"[MODEL] VALID: {path}")
        print(f"[MODEL] size={size} sha256={actual_hash}")
    return True


def download(target: Path, force: bool = False) -> None:
    target = target.expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)

    if target.exists() and not force and validate(target):
        print("[MODEL] tidak perlu download ulang.")
        return

    if target.exists():
        stamp = time.strftime("%Y%m%d_%H%M%S")
        bad = target.with_name(target.name + f".invalid_{stamp}")
        target.replace(bad)
        print(f"[MODEL] file lama tidak valid dipindah ke: {bad}")

    partial = target.with_suffix(target.suffix + ".part")
    partial.unlink(missing_ok=True)
    request = urllib.request.Request(
        OFFICIAL_URL,
        headers={"User-Agent": "ros2-minipc-yolopv2-model-installer/1.0"},
    )

    print(f"[MODEL] download resmi: {OFFICIAL_URL}")
    print(f"[MODEL] target         : {target}")
    try:
        with urllib.request.urlopen(request, timeout=30) as response, partial.open("wb") as out:
            total = response.headers.get("Content-Length")
            total = int(total) if total and total.isdigit() else EXPECTED_SIZE
            received = 0
            last_bucket = -1
            while True:
                chunk = response.read(4 * 1024 * 1024)
                if not chunk:
                    break
                out.write(chunk)
                received += len(chunk)
                bucket = int((received * 20) / max(1, total))
                if bucket != last_bucket:
                    last_bucket = bucket
                    pct = min(100.0, received * 100.0 / max(1, total))
                    print(f"[MODEL] {received / 1024 / 1024:7.1f} MiB / {total / 1024 / 1024:7.1f} MiB ({pct:5.1f}%)")
            out.flush()
            os.fsync(out.fileno())
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        partial.unlink(missing_ok=True)
        raise RuntimeError(f"download YOLOPv2 gagal: {exc}") from exc

    if not validate(partial):
        partial.unlink(missing_ok=True)
        raise RuntimeError("file hasil download gagal verifikasi size/SHA-256")

    partial.replace(target)
    print("[MODEL] instalasi selesai dan terverifikasi.")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", type=Path, default=default_target())
    parser.add_argument("--check", action="store_true", help="hanya validasi tanpa download")
    parser.add_argument("--force", action="store_true", help="download ulang meskipun file valid")
    args = parser.parse_args()
    try:
        if args.check:
            return 0 if validate(args.target.expanduser().resolve()) else 2
        download(args.target, args.force)
        return 0
    except Exception as exc:  # user-facing setup helper
        print(f"[MODEL] ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
