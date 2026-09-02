#!/usr/bin/env python3
"""Synchronize geometry-owned runtime parameters from vehicle_geometry.yaml."""
import argparse
import json
import os
import sys
from pathlib import Path

# Source-tree convenience; installed package resolves navigation_runtime normally.
_here = Path(__file__).resolve()
_source_python = _here.parents[1] / "python"
if _source_python.is_dir():
    sys.path.insert(0, str(_source_python))

from navigation_runtime.vehicle_geometry import GeometryError, sync_derived_configs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--navigation-share", default=str(_here.parents[1]))
    ap.add_argument("--esc-share", default=str(_here.parents[2] / "esc"))
    ap.add_argument("--dry-run", action="store_true")
    ns = ap.parse_args()
    try:
        result = sync_derived_configs(ns.navigation_share, ns.esc_share, dry_run=ns.dry_run)
    except GeometryError as exc:
        print(f"[GEOMETRY-SYNC] FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
