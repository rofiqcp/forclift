# Mapping GUI START/STOP — 3-Map SLAM Dataset

Top-level `map.launch.py` starts only:

- RViz (`mapping.rviz`)
- `mapping_gui.py`

LiDAR and IMU remain closed until **START** is pressed.

The Mapping GUI now manages exactly three SLAM experiment slots under:

`/home/otomasi2/forclift/src/navigation/maps`

Managed map pairs:

- `map_1.yaml` + `map_1.pgm`
- `map_2.yaml` + `map_2.pgm`
- `map_3.yaml` + `map_3.pgm`

The GUI exposes three tabs: **Map 1**, **Map 2**, and **Map 3**.

## Acquisition sequence

1. First START creates a fresh SLAM session. STOP + SAVE commits Map 1.
2. Second START creates another fresh SLAM session. STOP + SAVE commits Map 2.
3. Third START creates another fresh SLAM session. STOP + SAVE commits Map 3.
4. After all three valid map pairs exist, START is disabled. The GUI never automatically overwrites an experimental map.
5. `RESET 3 MAP` deletes only the three managed map pairs after explicit confirmation and allows a new 3-map experiment cycle.

A map slot is counted only if both the YAML and the image referenced by that YAML exist, are non-empty, and pass the existing stable-file verification.

On **STOP + SAVE MAP**:

1. `/map` is saved first into the active slot.
2. The YAML + image pair is verified and allowed to stabilize.
3. `/home/otomasi2/forclift/maps/latest_map.txt` is atomically updated to point to the newly committed slot YAML.
4. Only after commit is SIGINT sent to the runtime process group.
5. `lidar_node` stops the LiDAR motor and closes the serial port.
6. IMU exits and closes its serial port.
7. RViz and GUI remain open for the next experiment session.

GUI telemetry:

- LiDAR X/Y/theta: `/lidar/odom`
- Accelerometer + gyroscope: `/imu/data`
- Magnetometer: `/imu/mag` (displayed in microtesla)
- EKF X/Y/theta: `/odometry/filtered`
- live map preview: `/map`

If map saving fails, the runtime deliberately remains active so the current map data is not lost. A failed/incomplete map pair does not consume a dataset slot.
