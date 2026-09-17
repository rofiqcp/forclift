# V39 - Goal -> Hybrid A* -> MPPI + AMCL startup fix

Date: 2026-08-25

## Scope

This revision addresses the reported condition where a Goal Pose is clicked but
no visible Smac Hybrid-A* path (`/smac_plan`) and no MPPI trajectories
(`/trajectories`) appear.

## Root causes addressed in code

1. **Initial pose startup race**
   - The previous relay could forward `/initialpose_safe` before AMCL was fully
     lifecycle-active.
   - `initialpose_stamp_relay.py` now caches the newest manual pose, polls
     `/amcl/get_state`, and only publishes `/initialpose` when AMCL reports
     `ACTIVE`.
   - This also prevents the normal AMCL "received while inactive" startup
     warning.

2. **Manual pose vs automatic localization bootstrap race**
   - `autonomous_amcl_ready_gate.py` now listens to `/initialpose_safe` before
     AMCL activation.
   - A manual operator estimate suppresses automatic KNOWN_POSE /
     GLOBAL_LOCALIZATION bootstrap so it cannot overwrite/race the operator
     estimate.
   - Old AMCL covariance samples are cleared and readiness is evaluated again
     from the new estimate.

3. **Goal lost during Nav2 startup**
   - `goal_pose_nav2_bridge.py` now retains a Goal Pose if the canonical `/map`
     has not latched yet.
   - Once `/map` arrives, the retained goal is validated and then remains queued
     until `/navigate_to_pose` is available.
   - Invalid goals publish an explicit `REJECTED` event with a reason.
   - PlannerServer remains the single planning authority; actual `/plan` is
     relayed to `/smac_plan`.

4. **Inflation corrected for Nav2 Humble without falsifying vehicle geometry**
   - A smaller radius was evaluated but rejected because Humble InflationLayer logs an ERROR when `inflation_radius` is below the footprint inscribed radius (0.40 m).
   - MPPI CostCritic non-circular collision checking also needs the inflation field to cover the footprint circumscribed radius (~0.763 m) with a nonzero cost.
   - Local and global inflation radius is therefore **0.77 m** and cost scaling is **12.0**.
   - Physical footprint remains **1.30 x 0.80 m** with zero padding.
   - Smac and MPPI minimum turning radius remains **2.0 m**.
   - `allow_unknown` remains false and global unknown space stays protected.

## Important diagnostic from the supplied log/map

The reported Map 3 is 113 x 75 cells at 0.05 m/cell with origin
`[-2.93, -1.42]`, giving bounds approximately:

- x: -2.93 .. 2.72 m
- y: -1.42 .. 2.33 m

The supplied GUI log contains:

- Goal: x=1.762, y=-1.142, yaw=0
- Initial pose: x=2.371, y=2.073

Their **center cells are free** on the trinary map. However, at yaw=0 the full
vehicle footprint extends +/-0.65 m longitudinally and +/-0.40 m laterally:

- Goal lower footprint edge ~= -1.542 m, about 0.122 m outside Map 3.
- Initial-pose right footprint edge ~= 3.021 m, about 0.301 m outside Map 3.
- Initial-pose upper footprint edge ~= 2.473 m, about 0.143 m outside Map 3.

Therefore reducing inflation alone cannot make these exact poses safe/valid for
full-footprint collision planning. Do **not** shrink the physical footprint or
fake unknown/out-of-map space as free. If the robot is genuinely located this
close to the saved map edge, regenerate/extend the map. Otherwise set the
initial pose and goal farther inside mapped free space.

## Static validation

Run from the package source:

```bash
cd /home/otomasi2/forclift/src/navigation
python3 tools/verify_v39_goal_amcl_path.py
```

A valid source tree must finish with `0 FAIL`.

## Live ROS acceptance test

After deploying and rebuilding:

```bash
cd /home/otomasi2/forclift
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select navigation
source install/setup.bash
ros2 launch navigation autonomous.launch.py map:=auto
```

In another terminal:

```bash
cd /home/otomasi2/forclift
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run navigation verify_smac_goal_path.py --ros-args -p timeout_sec:=90.0
```

Then set a valid 2D Pose Estimate and a Goal Pose with the **whole robot
footprint inside mapped free space**. PASS requires all of the following in the
same live runtime:

- `/amcl/get_state` reports ACTIVE
- `/amcl_pose` received
- `/map` received
- global and local costmaps received
- PlannerServer action available
- NavigateToPose action available
- `/smac_plan` contains at least 2 poses
- `/trajectories` contains MPPI markers

This live test is the acceptance proof that cannot be replaced by static source
inspection on a machine without ROS/hardware.
