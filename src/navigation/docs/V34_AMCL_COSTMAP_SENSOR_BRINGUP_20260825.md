# V34 AMCL, costmap, LiDAR, and IMU bring-up fix

## Root cause found

The supplied build log is clean and contains no runtime Nav2 trace. The supplied
GUI screenshot shows that the real sensor streams are already present
(`/scan_nav` at about 6 Hz and IMU data at about 50 Hz). The blocking condition
was localization initialization:

1. the selected `map_3` has no valid map-bound `.autopose.json` sidecar;
2. AMCL therefore remained in `OPERATOR` mode waiting for a manually published
   initial pose;
3. without AMCL, `map -> odom` did not exist;
4. PlannerServer/global costmap correctly remained inactive;
5. ControllerServer/local costmap was unnecessarily chained behind the global
   costmap even though it uses `odom` as its global frame.

## Changes

- If the exact map has a valid SHA256-bound pose sidecar, AMCL still uses it.
- Otherwise AMCL now calls `/reinitialize_global_localization` and performs
  scan-based initialization from the real `/map` and `/scan_nav` data.
- No `(0, 0, 0)` pose is invented. AMCL must publish a stable, bounded-covariance
  pose and both `map -> odom` and `map -> base_footprint` must exist before the
  global Nav2 chain can activate.
- The local costmap now activates immediately after `/imu/data`, `/scan_nav`,
  `/esc/odom`, `/odometry/filtered`, LiDAR health, and local TF pass the pre-AMCL
  gate.
- The global costmap remains correctly gated on converged AMCL.
- A final transient-local join gate verifies that both costmaps published before
  BT Navigator, behaviors, smoother, and collision monitor activate.
- `gui.launch.py` forwards the new `auto_global_localization` option.

## Install and static verification on the Jetson/mini PC

From the workspace containing this replacement `src` directory:

```bash
cd /home/otomasi2/ros
bash src/navigation/tools/rebuild_navigation_clean.sh /home/otomasi2/ros
source /opt/ros/humble/setup.bash
source /home/otomasi2/ros/install/setup.bash
python3 src/navigation/tools/verify_v34_amcl_costmap_sensors.py
```

Expected static result:

```text
V34 STATIC RESULT: 35 PASS / 0 FAIL
```

## Real-hardware run

For the requested navigation-only test without camera/YOLO load:

```bash
ros2 launch navigation autonomous.launch.py \
  map:=auto \
  enable_camera:=false \
  enable_yolo:=false \
  enable_hole_alignment:=false \
  auto_global_localization:=true
```

Keep the AGV stationary during initial global localization. If the environment
is geometrically ambiguous, rotate or move it slowly by manual control while all
safety interlocks are healthy; do not bypass the covariance gate.

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/otomasi2/ros/install/setup.bash
AUTONOMOUS_CHECK_SECONDS=20 \
AUTONOMOUS_CHECK_REQUIRE_PERCEPTION=0 \
ros2 run navigation autonomous_runtime_check.py
```

The runtime checker is the acceptance test. A source-level check cannot prove
USB hardware, TF timing, AMCL convergence, or live costmap publication.

## Validation performed before packaging

- V34 AMCL/costmap/sensor verifier: 35 pass, 0 fail.
- PART-5 localization/timing verifier: 80 pass, 0 fail.
- PART-4 LiDAR safety verifier: 50 pass, 0 fail, 0 warning.
- PART-4 autonomy interlock verifier: 53 pass, 0 fail.
- Pure Python unit checks: 8 pass, 0 fail.
- Parse checks: 101 Python files, 33 YAML files, 3 package manifests, and all
  shell scripts passed.
- The supplied previous build log contained no compiler/CMake warning or error.

ROS 2 Humble and the physical AGV are not present in the packaging environment,
so `colcon build` and live topic/TF/lifecycle acceptance must be run on the
Jetson/mini PC using the commands above.

To force the previous manual behavior:

```bash
ros2 launch navigation autonomous.launch.py auto_global_localization:=false
```

Then publish the initial pose from the GUI or RViz `2D Pose Estimate` tool.
