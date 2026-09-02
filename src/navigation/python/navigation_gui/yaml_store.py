#!/usr/bin/env python3
"""Safe YAML loading/editing helpers for the AGV GUI.

The GUI reads production YAML files directly from disk.  Writes are atomic and
backed up.  ruamel.yaml is used when available so comments/ordering survive;
PyYAML is a fallback for systems where ruamel is not installed.
"""
from __future__ import annotations

import copy
import os
import shutil
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Sequence, Tuple

try:
    from ruamel.yaml import YAML  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    YAML = None

try:
    import yaml as pyyaml  # type: ignore
except Exception:  # pragma: no cover
    pyyaml = None


class YamlError(RuntimeError):
    pass


class YamlStore:
    """Round-trip-ish YAML store with atomic writes and backup snapshots."""

    def __init__(self, path: str | os.PathLike[str]):
        self.path = Path(path)
        self.data: Any = None
        self.backend = "ruamel" if YAML is not None else "pyyaml"
        self.last_loaded_mtime_ns: int | None = None
        self.last_error: str = ""

    def load(self) -> Any:
        if not self.path.exists():
            self.data = {}
            self.last_loaded_mtime_ns = None
            return self.data
        try:
            text = self.path.read_text(encoding="utf-8")
            if YAML is not None:
                y = YAML()
                y.preserve_quotes = True
                self.data = y.load(text) or {}
            elif pyyaml is not None:
                self.data = pyyaml.safe_load(text) or {}
            else:
                raise YamlError("Neither ruamel.yaml nor PyYAML is installed")
            self.last_loaded_mtime_ns = self.path.stat().st_mtime_ns
            self.last_error = ""
            return self.data
        except Exception as exc:
            self.last_error = str(exc)
            raise YamlError(f"Failed to read YAML {self.path}: {exc}") from exc

    def reload_if_changed(self) -> bool:
        try:
            mtime = self.path.stat().st_mtime_ns
        except OSError:
            mtime = None
        if mtime != self.last_loaded_mtime_ns:
            self.load()
            return True
        return False

    def get(self, key_path: Sequence[str | int], default: Any = None) -> Any:
        node = self.data
        try:
            for key in key_path:
                node = node[key]
            return node
        except Exception:
            return default

    def set(self, key_path: Sequence[str | int], value: Any) -> None:
        if self.data is None:
            self.load()
        if not key_path:
            self.data = value
            return
        node = self.data
        for key in key_path[:-1]:
            if isinstance(key, int):
                while len(node) <= key:
                    node.append({})
                node = node[key]
            else:
                if key not in node or node[key] is None:
                    node[key] = {}
                node = node[key]
        node[key_path[-1]] = value

    def backup(self) -> Path | None:
        if not self.path.exists():
            return None
        stamp = time.strftime("%Y%m%d_%H%M%S")
        backup_dir = self.path.parent / ".agv_gui_backups"
        backup_dir.mkdir(parents=True, exist_ok=True)
        backup = backup_dir / f"{self.path.name}.{stamp}.bak"
        try:
            shutil.copy2(self.path, backup)
            return backup
        except OSError:
            return None

    def save(self, make_backup: bool = True) -> Path:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if make_backup:
            self.backup()
        fd, tmp_name = tempfile.mkstemp(prefix=f".{self.path.name}.", suffix=".tmp", dir=str(self.path.parent))
        os.close(fd)
        tmp = Path(tmp_name)
        try:
            if YAML is not None:
                y = YAML()
                y.preserve_quotes = True
                y.indent(mapping=2, sequence=4, offset=2)
                with tmp.open("w", encoding="utf-8") as handle:
                    y.dump(self.data if self.data is not None else {}, handle)
                    handle.flush()
                    os.fsync(handle.fileno())
            elif pyyaml is not None:
                with tmp.open("w", encoding="utf-8") as handle:
                    pyyaml.safe_dump(
                        self.data if self.data is not None else {},
                        handle,
                        sort_keys=False,
                        default_flow_style=False,
                        allow_unicode=True,
                    )
                    handle.flush()
                    os.fsync(handle.fileno())
            else:
                raise YamlError("Neither ruamel.yaml nor PyYAML is installed")
            os.replace(tmp, self.path)
            try:
                dir_fd = os.open(str(self.path.parent), os.O_DIRECTORY)
                try:
                    os.fsync(dir_fd)
                finally:
                    os.close(dir_fd)
            except OSError:
                pass
            self.last_loaded_mtime_ns = self.path.stat().st_mtime_ns
            self.last_error = ""
            return self.path
        except Exception as exc:
            try:
                tmp.unlink()
            except OSError:
                pass
            self.last_error = str(exc)
            raise YamlError(f"Failed to save YAML {self.path}: {exc}") from exc


def iter_scalar_items(data: Any, prefix: Tuple[str | int, ...] = ()) -> Iterator[Tuple[Tuple[str | int, ...], Any]]:
    """Yield scalar leaves while keeping list-valued ROS parameter arrays intact."""
    if isinstance(data, dict):
        for key, value in data.items():
            yield from iter_scalar_items(value, prefix + (key,))
        return
    if isinstance(data, list):
        # ROS parameter vectors are edited as one value instead of one widget per cell.
        if all(not isinstance(v, (dict, list)) for v in data):
            yield prefix, list(data)
        else:
            for index, value in enumerate(data):
                yield from iter_scalar_items(value, prefix + (index,))
        return
    yield prefix, data


def clone_plain(data: Any) -> Any:
    """Return a plain-Python deep copy suitable for metadata snapshots."""
    if isinstance(data, dict):
        return {str(k): clone_plain(v) for k, v in data.items()}
    if isinstance(data, list):
        return [clone_plain(v) for v in data]
    if isinstance(data, tuple):
        return [clone_plain(v) for v in data]
    return copy.deepcopy(data)
