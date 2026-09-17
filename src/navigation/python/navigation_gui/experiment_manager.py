#!/usr/bin/env python3
"""Experiment/session logging for repeatable AGV tests."""
from __future__ import annotations

import csv
import os
import shutil
import subprocess
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional

from .yaml_store import YamlStore


@dataclass
class ExperimentSession:
    root: Path
    test_id: str
    test_name: str
    subsystem: str
    map_id: str = ""
    map_name: str = ""
    started_at: float = field(default_factory=time.time)
    rows: List[Dict[str, object]] = field(default_factory=list)

    @property
    def raw_dir(self) -> Path:
        return self.root / "raw"

    @property
    def csv_dir(self) -> Path:
        return self.root / "csv"

    @property
    def plots_dir(self) -> Path:
        return self.root / "plots"

    @property
    def screenshots_dir(self) -> Path:
        return self.root / "screenshots"

    @property
    def config_dir(self) -> Path:
        return self.root / "config_snapshot"


class ExperimentManager:
    def __init__(self, workspace: str | None = None):
        self.workspace = Path(workspace or os.environ.get('AGV_ROOT') or os.environ.get('AGV_WS') or (Path.home() / 'forclift'))
        self.base_dir = self.workspace / "log" / "agv_gui"
        self.current: Optional[ExperimentSession] = None

    def start(self, test_name: str, subsystem: str, map_id: str = "", map_name: str = "",
              operator_note: str = "", config_files: Iterable[Path] = ()) -> ExperimentSession:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_sub = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in subsystem)[:32]
        test_id = f"{stamp}_{safe_sub}"
        root = self.base_dir / test_id
        session = ExperimentSession(root=root, test_id=test_id, test_name=test_name,
                                    subsystem=subsystem, map_id=map_id, map_name=map_name)
        for directory in (session.raw_dir, session.csv_dir, session.plots_dir,
                          session.screenshots_dir, session.config_dir):
            directory.mkdir(parents=True, exist_ok=True)
        for path in config_files:
            try:
                p = Path(path)
                if p.is_file():
                    shutil.copy2(p, session.config_dir / p.name)
            except OSError:
                pass
        metadata = {
            "test_id": test_id,
            "test_name": test_name,
            "subsystem": subsystem,
            "map_id": map_id,
            "map_name": map_name,
            "operator_note": operator_note,
            "started_at": datetime.now().isoformat(),
            "workspace": str(self.workspace),
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
            "hostname": os.uname().nodename,
            "git_commit": self._git_commit(),
            "result": "RUNNING",
        }
        store = YamlStore(root / "metadata.yaml")
        store.data = metadata
        store.save(make_backup=False)
        self.current = session
        return session

    def append(self, row: Dict[str, object]):
        if self.current is None:
            return
        item = {"timestamp": time.time()}
        item.update(row)
        self.current.rows.append(item)

    def mark(self, label: str, note: str = ""):
        self.append({"event": label, "note": note})

    def stop(self, result: str = "COMPLETED") -> Optional[ExperimentSession]:
        session = self.current
        if session is None:
            return None
        if session.rows:
            keys = []
            seen = set()
            for row in session.rows:
                for key in row:
                    if key not in seen:
                        seen.add(key)
                        keys.append(key)
            with (session.csv_dir / "raw_data.csv").open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=keys)
                writer.writeheader()
                writer.writerows(session.rows)
        meta_store = YamlStore(session.root / "metadata.yaml")
        try:
            meta = meta_store.load()
        except Exception:
            meta = {}
        meta["finished_at"] = datetime.now().isoformat()
        meta["duration_s"] = max(0.0, time.time() - session.started_at)
        meta["result"] = result
        meta_store.data = meta
        meta_store.save(make_backup=False)
        self.current = None
        return session

    def list_sessions(self) -> List[Path]:
        if not self.base_dir.is_dir():
            return []
        return sorted((p for p in self.base_dir.iterdir() if p.is_dir()), reverse=True)

    def _git_commit(self) -> str:
        try:
            return subprocess.check_output(
                ["git", "-C", str(self.workspace), "rev-parse", "--short", "HEAD"],
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=1.0,
            ).strip()
        except Exception:
            return ""
