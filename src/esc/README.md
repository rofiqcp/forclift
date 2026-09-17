# ESC ROS 2 integration

Production transport is native VESC 6.00 at 921600 baud. The package expects two explicit board roles:

- `/dev/vesc_drive`: `DRIVE_DUAL_HALL`, LEFT and RIGHT Hall feedback are the authoritative traction velocity source.
- `/dev/vesc_steer`: `STEER_LEFT_ENCODER`, LEFT ABI encoder is the authoritative steering-angle source.

`esc_driver` verifies both firmware roles before opening the motion-ready gate. Dual-Hall ERPM is converted to mechanical RPM and m/s from explicit pole-pair, gear-ratio and canonical wheel-radius parameters. Steering readiness requires encoder configured, synchronized, homed and calibrated. Offline-zero odometry remains available with very large covariance and is never marked as valid feedback.

The previous fixed 64-byte Protocol-v4 ESC firmware and legacy Winch projects were removed from the active package after migration to `hoverboard-vesc` and `F4gateway` respectively.
