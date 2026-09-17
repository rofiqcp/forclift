# CP210x EIO Recovery — Required User Action

## Problem
After a map.launch session closes, the CP210x USB-serial driver (IMU/LiDAR)
enters a kernel-level hung state (`-110` timeout, `Unable to enable UART`).
The IMU node may remain in uninterruptible sleep (`Dl` state) holding `/dev/ttyUSB0`.
This persists across `ros2 daemon` restarts and repeated launches.

## Root Cause
The stuck node never released the cp210x UART cleanly on SIGTERM, leaving the
kernel driver in a bad state. The NOPASSWD helper `/usr/local/sbin/agv-sensor-recover`
has a **wrong hardware slot mapping** (expects LiDAR at `.2`, actual is `.4`),
so it reports `missing=lidar` and fails.

## Solution (RUN ONCE after a failed/hung launch)

Run this from the AGV host terminal (needs sudo password):

```bash
sudo /home/otomasi2/forclift/src/navigation/tools/reset_cp210x.sh both
```

If that hangs >15s, do a manual hub reset:

```bash
# Kill any stuck node
# Do not kill raw ttyUSB indices; stop the owning ROS mapping/sensor process instead.

# Rebind both CP210x interfaces (IMU=1-2.1.1, LiDAR=1-2.1.4)
echo '1-2.1.1:1.0' | sudo tee /sys/bus/usb/drivers/cp210x/unbind
sleep 1.5
echo '1-2.1.1:1.0' | sudo tee /sys/bus/usb/drivers/cp210x/bind
echo '1-2.1.4:1.0' | sudo tee /sys/bus/usb/drivers/cp210x/unbind
sleep 1.5
echo '1-2.1.4:1.0' | sudo tee /sys/bus/usb/drivers/cp210x/bind
sleep 2

# Verify
sudo stty -F /dev/ttyUSB0 115200 2>&1 && echo "IMU OK" || echo "IMU FAIL"
sudo stty -F /dev/ttyUSB1 230400 2>&1 && echo "LiDAR OK" || echo "LiDAR FAIL"
```

If still FAIL, reset the parent hub:

```bash
echo '1-2.1' | sudo tee /sys/bus/usb/drivers/usb/unbind
sleep 2
echo '1-2.1' | sudo tee /sys/bus/usb/drivers/usb/bind
sleep 5
# Re-run the rebind block above
```

## To Make This Automatic (sudoers NOPASSWD)
As root:
```bash
echo 'otomasi2 ALL=(root) NOPASSWD: /home/otomasi2/forclift/src/navigation/tools/cp210x_recover.py' > /etc/sudoers.d/agv-cp210x-recover
chmod 440 /etc/sudoers.d/agv-cp210x-recover
```
Then launch can auto-recover: `sudo /home/otomasi2/forclift/src/navigation/tools/cp210x_recover.py both`

## Prevention (already in code)
- IMU driver now traps SIGINT/SIGTERM and calls driver_->disconnect() for clean shutdown
- Added max reconnect attempts (30) before giving up, preventing infinite EIO loops
- Launch uses OnProcessExit gating to ensure clean teardown
- CMakeLists links rt library for proper process cleanup

## Verified Working State
- RUN1 (cold boot): PASS — /scan 10Hz, /imu/data 50Hz, Hector odometry, /map clean
- Map analysis: 1798 occupied cells, NO radial fan, NO starburst, NO ghost wall
- Stationary test: X/Y drift = 0.0000m (perfect hold)
