# USB Port Role Fix V9 — IMU + LiDAR + Astra behind one USB hub

## Evidence from log 20260814-123348

The log contains a known-good run at 19:14:59:

- IMU persistent CP2102 alias resolved to `/dev/ttyUSB0`, opened at 115200, and `/imu/data` became valid.
- LiDAR excluded that same canonical IMU port, selected `/dev/ttyUSB1` at 230400, verified AA55 packets, started the motor, and produced scans.

Later runs (19:16:30 and 19:29:20) still resolved the same logical roles, but both serial opens returned `Input/output error (EIO)`. The camera process also remained `rx=0 pub=0`. Therefore V9 separates two failure classes:

1. **role collision / ambiguous enumeration** — solved in software by a single resolver;
2. **kernel/hub EIO after the role is already correct** — visible as a separate hardware/USB health issue instead of being confused with wrong-port probing.

## V9 policy

No sensor is allowed to independently guess raw kernel indices during `map.launch.py` or `autonomous.launch.py`.

A single preflight tool `resolve_usb_roles.py` waits for udev topology to be stable and creates:

```text
/tmp/agv_devices/imu
  -> /dev/serial/by-id/...CP2102...
  -> current /dev/ttyUSBX

/tmp/agv_devices/lidar
  -> /dev/serial/by-path/...
  -> current /dev/ttyUSBY

/tmp/agv_devices/camera
  -> /dev/v4l/by-id or /dev/v4l/by-path/...video-index0
  -> current /dev/videoZ
```

The alias points to the persistent udev alias, not directly to `ttyUSBX` or `videoZ`. Thus a hub re-enumeration can change the final kernel number while the ROS launch argument remains unchanged.

## Important behavior

- IMU launch receives `/tmp/agv_devices/imu`; its own generic probe is bypassed.
- LiDAR launch receives `/tmp/agv_devices/lidar` and explicitly sets `auto_detect_port=false`; it cannot probe the IMU.
- Astra receives `/tmp/agv_devices/camera`; when a fixed device is supplied, the camera driver no longer scans/opens every `/dev/video*` candidate.
- Camera is started only after at least 3 real `/imu/data` and 3 real `/scan` messages. This prevents the high-bandwidth V4L2 device from being opened during serial hub settlement.

## Mapping run

```bash
mkdir -p /home/otomasi2/forclift/log
ros2 launch navigation map.launch.py 2>&1 | \
  tee "/home/otomasi2/forclift/log/$(date +%Y%m%d_%H%M%S).txt"
```

`map.launch.py` now defaults to LiDAR + IMU + Astra camera, with YOLO disabled for port validation.

## Full autonomous run

The current autonomous launch still uses a saved map argument for Nav2 localization. Use the actual saved YAML path:

```bash
ros2 launch navigation autonomous.launch.py \
  map:=/ABSOLUTE/PATH/TO/map.yaml 2>&1 | \
  tee "/home/otomasi2/forclift/log/$(date +%Y%m%d_%H%M%S).txt"
```

The missing-map exception seen in the uploaded log is independent of USB port arbitration.

## Expected resolver log

```text
[USB-ROLE] READY - stable mapping locked by persistent aliases:
  IMU    /tmp/agv_devices/imu -> /dev/serial/by-id/... -> /dev/ttyUSBX
  LiDAR  /tmp/agv_devices/lidar -> /dev/serial/by-path/... -> /dev/ttyUSBY
  Camera /tmp/agv_devices/camera -> /dev/v4l/by-path/...video-index0 -> /dev/videoZ
```

Only after this message are serial sensor nodes started.

## If EIO remains after READY

If the resolver prints the correct three roles and then both `/tmp/agv_devices/imu` and `/tmp/agv_devices/lidar` still return `EIO`, the port-role problem is already eliminated. Check the kernel/hub layer next (`dmesg -w`, hub power, cable, USB reset/over-current). V9 intentionally does not hide this with unsafe cross-probing.
