# V27 — Map Overlay, Hole-Block Preview, and Winch Integration

## Map 1 / Map 2

Map snapshots now show live ROS overlays by default, exactly like Map 3. The GUI still stores live data for all map slots and paints only the visible map, so this does not reintroduce hidden-tab redraw load. Goal and Initial Pose publication remains blocked when the selected map is not the active navigation map.

## Hole-Block Alignment

The autonomous launch now starts YOLO and `hole_block_alignment_node.py` independently of the camera-ready gate. The gate remains diagnostic only. The Hole-Block GUI requests both `/fork_alignment/image` and `/camera/color/image_raw`: processed alignment image has priority; raw camera is shown as an explicitly labelled fallback if the processed stream is temporarily absent.

`perception_all.launch.py` now starts camera + YOLO + hole alignment by default and `package.xml` declares the Python runtime dependencies used by the alignment node.

## Electric Winch

`esc` now owns a ROS 2 serial bridge (`winch_serial_node`) and launches it by default. The bridge auto-discovers `/dev/winch`, compatible STM/CDC by-id ports, or ttyACM devices; it never auto-selects ttyUSB to avoid claiming the ESC adapter. A candidate port is considered connected only after valid `STATE:` firmware telemetry is received. Every reconnect sends `STOP` before `STATUS`.

GUI commands use `/winch/command`. Structured status is published on `/winch/connected`, `/winch/port`, `/winch/state`, `/winch/top_limit`, `/winch/bottom_limit`, `/winch/pwm_pct`, `/winch/direction`, and `/winch/servo_deg`.

The firmware source was cleaned of generated PlatformIO/cache files. Duplicate Servo initialization/update calls were removed, pin banner text was corrected, direction tracking/dead-time was fixed, and the full 0..195 degree servo range is handled consistently.

## Verification

Clean build:

```bash
bash /home/otomasi2/ros/src/navigation/tools/build_map_hole_winch_clean.sh
```

After starting autonomous mode:

```bash
bash /home/otomasi2/ros/src/navigation/tools/verify_map_hole_winch_runtime.sh
```
