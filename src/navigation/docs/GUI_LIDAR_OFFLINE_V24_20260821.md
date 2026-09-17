# V24 — GUI OFFLINE while LiDAR motor still spins

## Evidence from the supplied 2026-08-21 logs

`starburst_fix_20260821_030204.txt` stops at:

- `AUTONOMOUS-SENSOR-PREFLIGHT waiting for previous hardware owners to release`

The stage never reaches the serial resolver / transport-ready gate, therefore the
current autonomous launch has not started its new IMU/LiDAR/localization/Nav2 chain.
A spinning LiDAR motor in this state can belong to a stale/respawned old lidar_node
and does not prove that `/scan_raw` or `/scan` is publishing in the current run.

The Connection screenshot also contains a second clue: `/map` and `/nav_map` have
already been seen, while the GUI card later says `ROS OFFLINE`. This means the GUI
ROS bridge was alive and then stopped, not that Qt itself failed to start.

## Fixes

1. `autonomous_sensor_preflight.sh`
   - kills OLD `gui.launch.py`, `allsystem.launch.py`, standalone LiDAR/IMU launch
     parents as well as the previous mapping/autonomous parents;
   - never kills the current launch ancestor;
   - ignores zombie PIDs because they cannot own serial FDs and cannot be removed
     with SIGKILL;
   - prints the exact blocker PID/command if a live owner still remains.

2. `navigation_gui/ros_bridge.py`
   - initializes rclpy in its worker thread without installing signal handlers;
   - contains callback exceptions so one malformed/stale custom message cannot
     take the whole GUI bridge offline;
   - makes custom YOLO/docking/alignment callbacks tolerant of older generated
     message layouts.

3. `navigation_gui/main_window.py`
   - mirrors runtime bridge diagnostics to stdout so they are captured in
     `/home/otomasi2/forclift/log/agv_gui_startup.log`.

4. `navigation_gui/pages.py`
   - Smac/MPPI event topics now show `IDLE` instead of misleading `NO DATA` before
     a goal is sent. `/smac_plan` is expected to publish only after Goal Pose.

## Post-build verification

Run the normal GUI launch, then in another terminal verify:

```bash
source /opt/ros/humble/setup.bash
source /home/otomasi2/forclift/install/setup.bash

pgrep -af 'ros2.*launch.*navigation'
ros2 node list
ros2 topic info -v /scan_raw
ros2 topic info -v /scan
ros2 topic echo /lidar/status --once
ros2 topic hz /scan_raw
ros2 topic hz /scan
```

Expected before sending a navigation goal:

- `/scan_raw` and `/scan` continuously publish (normally around the configured 10 Hz),
- LiDAR, LiDAR Odometry, IMU and EKF cards transition away from OFFLINE as their
  respective nodes/topics start,
- Smac may remain `IDLE — waiting for Goal Pose` until a goal is sent,
- after a valid goal, `/smac_plan` publishes and the Smac statistics table fills.
