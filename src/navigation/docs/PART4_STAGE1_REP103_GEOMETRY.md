# PART 4 — Stage 1: REP-103 Frame and Canonical Vehicle Geometry

## Scope

This stage changes geometry/frame ownership only. LiDAR safety splitting,
runtime autonomy health gating, AMCL policy, EKF LiDAR fusion policy and map
mutation are intentionally left for later stages so each change can be tested
independently.

## New frame tree

```text
base_footprint          navigation planar reference
  └─ base_link          REP-103: +X forward, +Y left, +Z up
       └─ cad_link      unmodified legacy CAD/joint coordinates
            ├─ body_link
            ├─ mast_link
            ├─ fork_link
            ├─ steering_link
            ├─ wheel joints
            └─ sensor tree
```

`cad_alignment_joint` is the only CAD-to-navigation yaw adapter. The default
candidate maps CAD `-Y` to REP-103 `+X` using +pi/2 yaw. This candidate is based
on the existing Astra CAD comment and fork/mast layout, but **physical validation
is mandatory** before autonomous motion.

## Canonical source of truth

`config/vehicle_geometry.yaml` schema 2 now owns:

- CAD-to-REP-103 yaw
- wheelbase
- loaded/effective wheel radius
- body/footprint dimensions
- max steering angle
- minimum turning radius
- actuator and Nav2 speed limits
- maximum yaw-rate limit

Geometry-owned fields are synchronized into runtime Nav2 and ESC YAML while all
controller/SLAM/EKF tuning values are preserved.

## CAD reference detected from the supplied URDF

- steering-axle Y center:  +0.560003211 m
- fixed-axle Y center:     -0.410003687 m
- CAD axle separation:      0.970006898 m
- CAD steering-axle track:  0.557867756 m
- CAD fixed-axle track:     0.539999970 m

The operational wheelbase remains 0.70 m because CAD dimensions are not allowed
to overwrite an unverified real-world calibration. This mismatch is now an
explicit warning.

## Upgrade behavior

An old PART-3 runtime `vehicle_geometry.yaml` is migrated automatically to
schema 2. Numeric operational values are preserved, a backup is written under
`.geometry_backups/`, and all physical validation flags are reset to false.

## Fail-closed autonomous behavior

Autonomous launch refuses to start while any of these are false:

- `frame_convention.rep103_alignment_validated`
- `vehicle.wheelbase_validated`
- `vehicle.wheel_radius_validated`
- `vehicle.max_steering_validated`

For stationary bench diagnostics only, the explicit environment override
`AGV_ALLOW_UNVALIDATED_GEOMETRY=1` can bypass this gate. It must not be used for
normal autonomous driving.

## Physical validation sequence

1. Build and source the workspace.
2. Run `verify_part4_stage1_geometry.py`.
3. Start `urdf_rviz.launch.py` or mapping mode, not autonomous.
4. Set RViz fixed frame to `base_footprint`.
5. Put a clearly identifiable obstacle physically in front of the AGV.
6. Confirm the obstacle appears in `+X` of `base_footprint`.
7. Confirm physical left is `+Y`.
8. If the CAD-forward candidate is wrong, set either:
   - CAD `-Y` forward -> `cad_to_base_yaw_rad: +1.570796326795`, or
   - CAD `+Y` forward -> `cad_to_base_yaw_rad: -1.570796326795`.
9. Measure physical wheelbase axle-center to axle-center and enter `wheelbase_m`.
10. Measure effective loaded wheel radius and enter `wheel_radius_m`.
11. Measure maximum safe steering angle and enter `max_steering_angle_rad`.
12. Set each validation flag true only after its physical test passes.
13. Save in GUI; geometry-derived runtime files synchronize automatically.
14. Run:

```bash
python3 src/navigation/tools/sync_vehicle_geometry.py
python3 src/navigation/tools/verify_part4_stage1_geometry.py --require-physical-validation
```

Only after the strict verifier passes should autonomous launch be enabled.

## Expected static state before field validation

The source archive should report `0 FAIL` and warnings for the four physical
validation flags plus the current 0.70 m versus 0.970 m CAD wheelbase mismatch.
Those warnings are intentional and cannot be resolved correctly without the
real AGV.
