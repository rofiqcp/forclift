# V17 — Deterministic LiDAR + IMU CP210x Recovery

## Failure reproduced from 2026-08-19 logs

The USB role resolver correctly mapped:

- IMU physical port `1-2.1.1` -> `/dev/ttyUSB0`
- LiDAR physical port `1-2.1.4` -> `/dev/ttyUSB1`

The old serial-ready gate then reported both ports openable, but the real IMU and
LiDAR nodes subsequently returned `Input/output error`. Mapping therefore stayed
at zero `/imu/data` and zero `/scan` messages. A later mapping run reproduced the
same EIO on both bridges.

## Root causes fixed

1. The old readiness gate opened and closed both CP210x tty devices before the
   real drivers. That extra open/close cycle is now removed; the gate is topology
   only and never touches sensor bytes or tty state.
2. IMU and LiDAR drivers previously performed their first `open()` without
   `O_NONBLOCK`. Both now claim the tty non-blocking from the first syscall and
   then configure termios while keeping exclusive ownership.
3. `HUPCL` was not actually disabled in the current source. It is now cleared for
   both serial drivers so last-close does not produce an uncontrolled modem-line
   edge. LiDAR RTS/DTR motor power is controlled explicitly.
4. Mapping GUI previously reopened the LiDAR port after the mapping runtime had
   already exited, via `lidar_motor_off.py`. That redundant post-shutdown
   open/close has been removed. Motor STOP is issued while `lidar_node` still owns
   the port; the node destructor then closes it once.
5. The existing `cp210x_recover.py` contained broken parent-hub reset code and
   assumed raw ttyUSB numbers. It has been replaced by a physical-port based
   recovery helper.
6. Mapping runtime now uses the same staged preflight -> resolver -> topology gate
   -> IMU -> LiDAR flow as autonomous mode.
7. `reset_cp210x.sh` sudo redirection was corrected (`sudo tee`), so manual
   fallback writes actually reach sysfs.
8. The mapping log also contained an unrelated `esc_keyboard_teleop` exit `-11`
   during SIGINT shutdown.  Its destructor no longer publishes after the ROS
   context has started shutting down, removing that false launch ERROR path.

## One-time installation required for zero physical replug

Run once from the workspace source after applying V17:

```bash
cd /home/otomasi2/forclift
bash src/navigation/tools/install_sensor_recovery.sh
```

This installs only:

```text
/usr/local/sbin/agv-sensor-recover
/etc/sudoers.d/agv-sensor-recover
```

The sudoers entry grants passwordless root execution only to that dedicated
recovery helper. ROS launch itself never asks for a sudo password.

After installation, every mapping/autonomous sensor session performs a software
rebind of the two physical CP210x interfaces before the resolver starts. If the
interface-level rebind is insufficient, the helper resets the parent hub and
waits for both physical serial interfaces to return. This replaces the previous
manual unplug/replug workflow.

## Build

```bash
cd /home/otomasi2/forclift
source /opt/ros/humble/setup.bash
colcon build --packages-select navigation esc --symlink-install
source install/setup.bash
```

## Runtime validation

Start either:

```bash
ros2 launch navigation map.launch.py
```

or:

```bash
ros2 launch navigation autonomous.launch.py
```

Expected startup sequence:

```text
[AUTONOMOUS-SENSOR-PREFLIGHT] CP210x software replug starting
[AGV-SERIAL-RECOVERY] IMU: recovered on physical 1-2.1.1 -> /dev/ttyUSBX
[AGV-SERIAL-RECOVERY] LIDAR: recovered on physical 1-2.1.4 -> /dev/ttyUSBY
[AGV-SERIAL-RECOVERY] READY: requested serial interfaces are driver-bound; physical replug not required
[USB-ROLE] USB OWNERSHIP CHECK = PASS
[SERIAL-READY] READY ... no probe open/close performed
[IMU-CONNECT] Opening IMU ...
[LIDAR ...] Opening serial port ...
```

Then verify:

```bash
timeout 8 ros2 topic hz /imu/data
timeout 8 ros2 topic hz /scan
```

Targets are approximately IMU 50 Hz and LiDAR 10 Hz according to the current
launch/configuration.

Most important regression test: STOP the session, start it again without touching
USB, and repeat at least 5 times. No physical cable replug should be required.
