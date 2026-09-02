# Electric Winch integration

The maintained firmware project is `ELECTRIC WINCH/` (Black Pill STM32F411 + BTS7960 + limit switches + servo).
Generated PlatformIO `.pio` directories and Python bytecode are intentionally not stored in the ROS source tree.

ROS 2 integration is provided by `esc/winch_serial_node` and `config/winch.yaml`.
The bridge uses the firmware's USB CDC text protocol at 115200 baud and publishes structured status under `/winch/*`.

## Safe startup behavior

On every serial connection or reconnection the bridge sends `STOP` before requesting `STATUS`.
Auto discovery prioritizes `/dev/winch`, then STM/CDC `/dev/serial/by-id/*`, then `/dev/ttyACM*`.
It intentionally does not auto-select `/dev/ttyUSB*` so an ESC serial adapter cannot be claimed accidentally.

## Commands

Publish `std_msgs/msg/String` to `/winch/command` with one of:
`UP`, `DOWN`, `STOP`, `STATUS`, `LIMITS`, `SERVOTEST`, or `SERVO <0..195>`.
The Autonomous Vehicle Interface exposes these commands in its **Winch** tab.
