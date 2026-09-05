# ESC Integrated Runtime

Package `esc` sekarang hanya memiliki dua runtime C++:

1. `motor_teleop` — keyboard/gamepad global -> `/cmd_vel/teleop`
2. `esc_ackermann` (`ackermann_controller_server.cpp`) — mux + Ackermann + UART STM

## Priority

1. E-STOP / safety
2. TELEOP active
3. TELEOP release hold (zero command, default 0.50 s)
4. NAV2 final command (`/cmd_vel` by default)
5. IDLE zero

Teleop yang idle terus publish zero **tidak** mengambil alih Nav2 karena ownership manual ditentukan juga oleh `/teleop/active_source`.

## Command chain

```text
keyboard/gamepad -> motor_teleop -> /cmd_vel/teleop ----+
                                                       |
Nav2 controller -> /cmd_vel_nav_raw                    |
 -> velocity_smoother -> /cmd_vel_nav_smoothed         |
 -> NavigationCore/perception/collision -> /cmd_vel ---+-> esc_ackermann
                                                           | mux
                                                           | Ackermann conversion
                                                           | robust UART
                                                           v
                                                          STM32
```

Untuk memakai smoother langsung (bypass final NavigationCore/CollisionMonitor), ubah `nav2_topic` di `config/ackermann.yaml` menjadi `/cmd_vel_nav_smoothed`. Default `/cmd_vel` lebih aman karena merupakan command final setelah safety/collision pipeline.

## Single source of truth limits

`config/teleop.yaml`:

- `speed_max: 1.00` m/s
- `yaw_max_deg_s: 80.0`
- `serial_left_max_deg: 80.0` logical/protocol steering limit (physical wheel angle must be calibrated separately)
- `serial_right_max_rpm: 300.0`

Mapping drive: `1.0 m/s -> 300 RPM`, `0.5 -> 150 RPM`, `-1.0 -> -300 RPM`.

Manual steering preserves the previous teleop behavior: full A/D at the current yaw limit maps to the same real steering degree. Nav2 uses true Ackermann conversion `delta = atan(L * omega / v)` and is then clamped to the same real steering limit.

## STM serial contract

This ROS package expects the robust hoverboard protocol already integrated previously:

- 115200 8N1
- fixed 14-byte frame
- SOF `A5 5A`
- sequence number
- CRC16-CCITT
- command: LEFT real centi-degree + RIGHT deci-RPM
- ACK: measured LEFT real angle + measured RIGHT RPM + status
- firmware watchdog 250 ms

Only `esc_ackermann` opens the serial port. `motor_teleop` contains no UART sender.

## Build

```bash
cd /home/otomasi/ros
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select esc navigation
source install/setup.bash
```

Standalone ESC runtime:

```bash
# Default Mini-PC: ESC dipilih fail-closed berdasarkan physical USB topology.
# ESC CH340 berada pada USB path ...usb-0:1.1:1.0 sehingga perubahan ttyUSB tidak mengubah identitas.
ros2 launch esc esc.launch.py

# Override hanya untuk commissioning/debug yang disengaja:
# ros2 launch esc esc.launch.py serial_device:=/dev/serial/by-path/<ESC_PATH>
```

Full autonomous runtime remains through `navigation/launch/autonomous.launch.py`.


## Steering physical calibration (V10 Part 1)

Nilai protocol STM `-90/0/+90` **bukan** sudut roda fisik. Gunakan halaman **Kalibrasi Steering Fisik** pada GUI navigation untuk menangkap LEFT/CENTER/RIGHT protocol+ACK dan memasukkan hasil ukur sudut roda dalam kiri/kanan. Setelah aktif, `/esc/steering_actual` dan odometri Ackermann menggunakan sudut roda nyata. `serial_left_max_deg` dipertahankan hanya sebagai fallback legacy sebelum commissioning fisik selesai.
