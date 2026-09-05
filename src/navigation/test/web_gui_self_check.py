#!/usr/bin/env python3
"""Dependency-light contract for the native C++ localhost web HMI."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]

def fail(msg):
    print("FAIL:", msg)
    raise SystemExit(1)

cpp = (ROOT / "web/web_server.cpp").read_text(encoding="utf-8")
html = (ROOT / "web/static/index.html").read_text(encoding="utf-8")
css = (ROOT / "web/static/styles.css").read_text(encoding="utf-8")
js = (ROOT / "web/static/app.js").read_text(encoding="utf-8")
cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
auto = (ROOT / "launch/autonomous.launch.py").read_text(encoding="utf-8")
gui = (ROOT / "launch/gui.launch.py").read_text(encoding="utf-8")

for path in (ROOT/"web/static/index.html", ROOT/"web/static/styles.css", ROOT/"web/static/app.js"):
    if not path.is_file() or path.stat().st_size < 1000:
        fail(f"web asset missing/too small: {path}")
for token in ("add_executable(agv_web_gui", "Qt5::Network", "web/static", "agv_web_gui"):
    if token not in cmake: fail(f"CMake web integration missing {token}")
for token in ("start_web_gui", "web_bind_address", "web_port", "127.0.0.1", "agv_web_gui"):
    if token not in auto: fail(f"autonomous launch web contract missing {token}")
for token in ("start_web_gui", "web_bind_address", "web_port"):
    if token not in gui: fail(f"gui launch does not forward {token}")
for endpoint in ("/api/events", "/api/state", "/api/health", "/api/camera.jpg", "/api/map.png",
                 "/api/navigation/goal", "/api/navigation/cancel",
                 "/api/localization/initial-pose", "/api/steering/calibration-mode",
                 "/api/config/set", "/api/experiment/record/start",
                 "/api/experiment/record/stop", "/api/experiment/record/status"):
    if endpoint not in cpp: fail(f"web endpoint missing {endpoint}")
for page in ("overview", "navigation", "perception", "sensors", "esc", "calibration", "tuning", "experiments", "reports", "diagnostics", "configuration"):
    if f'id="page-{page}"' not in html: fail(f"frontend page missing {page}")
for bad in ("https://", "http://cdn", "unpkg.com", "cdnjs", "jsdelivr"):
    if bad in html or bad in js or bad in css: fail(f"web GUI must remain offline/self-contained: {bad}")
if "EventSource('/api/events')" not in js:
    fail("frontend realtime SSE connection missing")
if '<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">' not in html:
    fail("responsive viewport meta missing")
if '<link rel="icon" href="data:,">' not in html:
    fail("inline favicon guard missing; browser must not emit favicon 404")
for token in ("Mobile containment", ".workbench-main>*", ".domain-tab>div", ".table-scroll{max-width:100%", ".sidebar-backdrop", ".sidebar.open + .sidebar-backdrop"):
    if token not in css: fail(f"BAB IV mobile containment missing {token}")
if 'id="sidebarBackdrop"' not in html or "$('sidebarBackdrop').onclick" not in js:
    fail("mobile sidebar backdrop behavior missing")

# BAB IV web workbench must expose the three source domains and real tuning/evidence surfaces.
for token in ("BAB IV • Pengujian, Tuning & Evidence", "data-exp=\"navigation\"",
              "data-exp=\"perception\"", "data-exp=\"steering\"",
              "id=\"tuningFields\"", "id=\"experimentGraphs\"", "id=\"experimentTable\"",
              "id=\"expMapCanvas\"", "id=\"expCameraImage\"", "id=\"expEscCanvas\"",
              "id=\"startCsvBtn\"", "id=\"stopCsvBtn\""):
    if token not in html: fail(f"BAB IV web workbench missing {token}")
for token in ("WEB_TUNING", "resolveMetricPath", "drawExperimentChart", "drawExperimentMap",
              "drawExperimentEsc", "saveTuningField", "startWebRecording", "stopWebRecording"):
    if token not in js: fail(f"BAB IV web behavior missing {token}")
for token in ("setYamlValueAtomic", "patchExistingYamlScalar", "captureRecordingSample",
              "saveRecordingFiles", "agv_web_reports", "foc_thesis", "bbox_calibration",
              "localization_cpp.yaml", "mppi_closed_loop.yaml"):
    if token not in cpp: fail(f"web tuning backend missing {token}")
if "Content-Security-Policy" not in cpp:
    fail("HTTP response security headers missing")

for token in ("cameraEncodeMutex_", "rosShutdownGuard", "Request body too large",
              "canonicalRoot", "declare_parameter<std::int64_t>(\"port\"",
              "configuredPort > 65535"):
    if token not in cpp: fail(f"web runtime hardening missing {token}")
print("PASS web_gui_self_check")
