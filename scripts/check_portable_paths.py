#!/usr/bin/python3
"""Fail jika workspace glue portable kembali memakai path HOME absolut.

Package runtime lama dapat tetap memakai AGV_WS/deployment paths sampai dimigrasikan
secara terpisah; installer, environment helper, dan dokumentasi root harus portable.
"""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FORBIDDEN = re.compile("/" + "home" + r"/[A-Za-z0-9._-]+/")
SKIP_DIRS = {".git", "build", "install", "log", "backups", "backup", ".backup", ".preview_v3", "docs", "doc", "__pycache__", ".vscode"}
SKIP_SUFFIXES = (".pyc", ".o", ".so", ".a", ".png", ".jpg", ".pt", ".engine")
SCAN_ROOTS = [
    ROOT / "scripts",
    ROOT / "install.sh",
    ROOT / "README.md",
]


def candidates():
    for base in SCAN_ROOTS:
        items = [base] if base.is_file() else base.rglob("*") if base.exists() else []
        for path in items:
            if not path.is_file() or any(part in SKIP_DIRS for part in path.parts):
                continue
            if path.name == Path(__file__).name or ".bak" in path.name or path.name.endswith((".bak", ".baseline")):
                continue
            if path.suffix.lower() in SKIP_SUFFIXES:
                continue
            yield path


def main() -> int:
    failures = []
    for path in candidates():
        try:
            text = path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        for lineno, line in enumerate(text.splitlines(), 1):
            if FORBIDDEN.search(line):
                failures.append(f"{path.relative_to(ROOT)}:{lineno}: {line.strip()}")
    if failures:
        print("FAIL portable path contract")
        for item in failures:
            print("  -", item)
        return 2
    print("PASS portable path contract: AGV_ROOT -> fallback $HOME/forclift")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
