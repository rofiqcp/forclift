#!/usr/bin/env bash
set -Eeuo pipefail
SRC="${AGV_WS:-$HOME/ros}/src/navigation"
fail(){ echo "[V57-VERIFY] FAIL: $*" >&2; exit 2; }
pass(){ echo "[V57-VERIFY] PASS: $*"; }

PAGES="$SRC/python/navigation_gui/pages.py"
MAIN="$SRC/python/navigation_gui/main_window.py"
BRIDGE="$SRC/python/navigation_gui/ros_bridge.py"
WIDGETS="$SRC/python/navigation_gui/widgets.py"

for f in "$PAGES" "$MAIN" "$BRIDGE" "$WIDGETS"; do [[ -f "$f" ]] || fail "missing $f"; done

grep -q 'self.local_goal: Optional' "$PAGES" || fail 'per-map local_goal state missing'
grep -q 'GOAL M{self.source.slot}' "$PAGES" || fail 'map-owned goal label missing'
grep -q 'Not published to Nav2 to prevent cross-map goal leakage' "$MAIN" || fail 'inactive-map goal publish gate missing'
grep -q 'self.override_box.setChecked(False)' "$PAGES" || fail 'inactive-map live overlay isolation missing'
grep -q 'TOPICS = \["/scan_nav"' "$PAGES" || fail 'TF/Timing still using legacy /scan'
grep -q 'Sparse/event-driven signals' "$WIDGETS" || fail 'single-sample plot marker fix missing'
grep -q '"/amcl_pose", self._amcl_cb, best_effort' "$BRIDGE" || fail 'AMCL adaptive QoS fix missing'
grep -q 'fallback_topics=\["/obstacle_detection/visualization"\]' "$MAIN" || fail 'camera processed fallback missing'
grep -q 'image_interest = ("/camera/color/image_raw", "/obstacle_detection/visualization")' "$MAIN" || fail 'camera image-interest fallback missing'
grep -q 'page.update_health(payload)' "$MAIN" || fail 'image health routing missing'
grep -q 'GUI image conversion failed' "$BRIDGE" || fail 'camera conversion diagnostic missing'

python3 -m py_compile "$PAGES" "$MAIN" "$BRIDGE" "$WIDGETS"
pass 'goal isolation, /scan_nav timing, AMCL graph/QoS, camera preview/fallback and Python syntax'
