# Autonomous + GUI Isolation Fix — 2026-08-23

## Root cause
Runtime vehicle geometry is still physically unvalidated. Older autonomous.launch.py
raised GeometryError during launch generation. When gui.launch.py included autonomous
inside the same launch service, this exception/shutdown propagated SIGINT into PyQt and
looked like a GUI force-close.

## Fix
- autonomous.launch.py no longer aborts bring-up solely because calibration flags are false.
- autonomy_health_manager receives geometry_validated and keeps
  /system/autonomy_motion_allowed=false with reason geometry:unvalidated until physical
  calibration is validated (or explicit bench override is set).
- gui.launch.py runs autonomous.launch.py as an isolated child ros2-launch process after
  the GUI first-paint delay. Autonomous fail-closed shutdown can no longer kill the GUI.
- scan_self_filter.py suppresses only the known Humble executor teardown RuntimeError on
  shutdown, removing a false traceback while preserving genuine runtime exceptions.
