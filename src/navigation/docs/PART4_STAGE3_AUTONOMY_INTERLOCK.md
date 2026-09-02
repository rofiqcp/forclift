# PART 4 Stage 3 — Continuous Autonomy Interlock

## Tujuan
Stage ini menutup celah startup/runtime sehingga command autonomous hanya dapat mencapai ESC ketika seluruh jalur keselamatan dan navigasi tetap sehat secara kontinu.

## Jalur command final

```text
Planner/Controller
      ↓
Velocity Smoother
      ↓
Collision Monitor
      ↓
Sensor CMD Guard
      ↓ /cmd_vel
ESC Command Mux  ← /system/autonomy_motion_allowed
      ↓
/cmd_vel/actuator
      ↓
ESC Driver
```

`/system/autonomy_motion_allowed` dipublish oleh `autonomy_health_manager.py` dengan fail-closed default FALSE.

## Syarat autonomy_motion_allowed=true
- `/lidar/safety_healthy` fresh dan TRUE.
- `/sensor_guard/healthy` fresh dan TRUE.
- `/esc/ready` fresh dan TRUE.
- `/odometry/filtered` fresh.
- AMCL pose pernah diterima dan covariance berada di bawah batas.
- `map -> base_footprint` dan `odom -> base_footprint` tersedia dengan timestamp valid.
- Tidak ada jump `map -> base_footprint` melebihi batas selama hold period.
- `/global_costmap/costmap` dan `/local_costmap/costmap` fresh.
- Lifecycle ACTIVE: AMCL, PlannerServer, ControllerServer, BehaviorServer, BT Navigator, Velocity Smoother, Collision Monitor.
- Semua syarat stabil selama `stable_recovery_sec`.

## Fail-closed layers
1. Health manager publish FALSE saat startup dan shutdown.
2. Jika health manager crash setelah pernah TRUE, ESC mux menganggap gate stale setelah `autonomy_gate_timeout_sec` dan memblok Nav2.
3. Sensor guard tetap independen dan mengeluarkan zero command bila LiDAR/IMU/ESC command freshness gagal.
4. E-stop tetap mempunyai prioritas tertinggi di ESC mux.

## Startup gate
Semua transition penting di `autonomous.launch.py` memakai success-only OnProcessExit. Exit code nonzero menghasilkan Shutdown dan downstream stage tidak dijalankan.

## Uji Jetson

```bash
cd /home/otomasi2/ros
colcon build --symlink-install --packages-select navigation esc
source install/setup.bash
python3 src/navigation/tools/verify_part4_stage3_autonomy_interlock.py
ros2 launch navigation autonomous.launch.py
```

Saat stack sudah ACTIVE:

```bash
ros2 topic echo /system/autonomy_motion_allowed
ros2 topic echo /system/autonomy_health
ros2 topic echo /esc/mux/status
python3 src/navigation/scripts/autonomous_runtime_check.py
```

## Fault injection wajib
Lakukan pada roda terangkat / kendaraan diam terlebih dahulu.

1. **LiDAR health fault**: cabut LiDAR atau blok stream → gate harus FALSE dan `/cmd_vel/actuator` zero.
2. **IMU fault**: hentikan IMU → sensor guard FALSE → autonomy gate FALSE.
3. **ESC feedback fault**: cabut telemetry ESC → `/esc/ready=false` → gate FALSE.
4. **AMCL lifecycle fault**: deactivate AMCL → gate FALSE.
5. **Planner/controller lifecycle fault**: deactivate salah satu → gate FALSE.
6. **Collision monitor fault**: hentikan node → lifecycle tidak ACTIVE dan gate FALSE.
7. **Health manager crash**: kill `autonomy_health_manager`; dalam <= gate timeout mux harus `nav2_blocked`.
8. **TF fault**: hentikan EKF/AMCL → TF stale → gate FALSE.
9. **Costmap fault**: hentikan PlannerServer/ControllerServer → gate FALSE.
10. **Recovery**: setelah fault pulih, gate tidak boleh TRUE sebelum semua kondisi stabil selama `stable_recovery_sec`.

## Catatan AMCL
`/amcl_pose` hanya diwajibkan pernah terlihat dan covariance valid. Pose topic tidak dipaksa terus-menerus fresh karena saat robot diam AMCL dapat tidak menerbitkan pose baru. Liveness global localization dijaga oleh lifecycle dan freshness TF.

AMCL dapat menstamp transform ke masa depan sesuai `transform_tolerance`; `tf_future_tolerance_sec` mengakomodasi perilaku itu secara terbatas.

## Batas validasi host
Static verifier tidak menggantikan build ROS 2 Humble, DDS runtime, lifecycle transition, TF timing, serial hardware, atau stopping test di AGV fisik.
