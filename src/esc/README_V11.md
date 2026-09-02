# ESC V11

Paket `esc` adalah **single owner** untuk actuator Ackermann:

- `esc_driver`
- `esc_command_mux`
- `esc_keyboard_teleop` (global evdev, tanpa fokus window)
- `joy_node` melalui `esc.launch.py`
- `ackermann_controller_server`
- `esc/EscAwareProgressChecker`

## Standalone

```bash
ros2 launch esc esc.launch.py
```

Default: driver + mux + keyboard + joystick aktif, Nav2 input mati.

Global keyboard primary mode memakai evdev. Sekali saja:

```bash
bash src/esc/tools/setup_global_keyboard_access.sh
```

Lalu logout/login.

Kontrol default:

- RIGHT CTRL + W/S: maju/mundur
- RIGHT CTRL + A/D: kiri/kanan
- RIGHT CTRL + E/Q: speed +/-
- RIGHT CTRL + R/F: steering +/-
- SPACE: E-stop
- RIGHT CTRL + X: clear E-stop

`steering_calibrated=false` adalah default fail-closed. Teleop/mux dapat diuji, tetapi physical motion tidak boleh di-arm sebelum zero/sign/ticks-per-rad steering benar-benar dikalibrasi.

## Autonomous

`navigation/autonomous.launch.py` meng-include `esc.launch.py` dengan `enable_nav2=true`, `standalone_mode=false`, dan `require_autonomy_gate=true`.

Input autonomous final ke ESC mux adalah `/cmd_vel`; output actuator tunggal adalah `/cmd_vel/actuator`.
