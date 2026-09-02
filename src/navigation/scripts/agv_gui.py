#!/usr/bin/python3
"""Python 3.10 entry point for the Autonomous Vehicle Interface."""
from __future__ import annotations

import os
import sys
import traceback
from datetime import datetime
from pathlib import Path

LOG_PATH = Path("/home/otomasi2/ros/log/agv_gui_startup.log")


def _log(message: str) -> None:
    text = f"{datetime.now().isoformat(timespec='seconds')} [AGV-GUI-PY] {message}"
    print(text, flush=True)
    # When started through agv_gui_launcher.sh stdout/stderr are already
    # tee'd into the same startup log. Avoid writing the identical line twice.
    if os.environ.get("AGV_GUI_STDIO_TEE") == "1":
        return
    try:
        LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        with LOG_PATH.open("a", encoding="utf-8") as handle:
            handle.write(text + "\n")
    except Exception:
        pass


def _ensure_python_310() -> None:
    if sys.version_info[:2] != (3, 10):
        raise SystemExit(
            "[AGV-GUI][ERROR] wajib Python 3.10 "
            f"(current={sys.version.split()[0]} executable={sys.executable})"
        )


def _ensure_local_module_path() -> None:
    script_dir = str(Path(__file__).resolve().parent)
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)


def _diagnostic_window(error_text: str) -> int:
    try:
        from PyQt5.QtCore import Qt
        from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QTextEdit, QVBoxLayout, QWidget
        app = QApplication.instance() or QApplication(sys.argv)
        w = QWidget()
        w.setWindowTitle("Autonomous Vehicle Interface — STARTUP DIAGNOSTIC")
        layout = QVBoxLayout(w)
        title = QLabel("GUI core failed to initialize")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size:18px;font-weight:600;padding:10px")
        layout.addWidget(title)
        text = QTextEdit(); text.setReadOnly(True); text.setPlainText(error_text)
        layout.addWidget(text, 1)
        close = QPushButton("Close"); close.clicked.connect(w.close); layout.addWidget(close)
        w.resize(900, 560); w.show(); w.raise_(); w.activateWindow()
        return app.exec_()
    except Exception:
        return 2


_ensure_python_310()
_ensure_local_module_path()
_log(f"entry started python={sys.version.split()[0]} executable={sys.executable} DISPLAY={os.environ.get('DISPLAY','')}")

try:
    from navigation_gui.main_window import main
except BaseException:
    tb = traceback.format_exc()
    _log("main_window import failed:\n" + tb)
    raise SystemExit(_diagnostic_window(tb))

if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SystemExit:
        raise
    except BaseException:
        tb = traceback.format_exc()
        _log("main() failed:\n" + tb)
        raise SystemExit(_diagnostic_window(tb))
