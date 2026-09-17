#!/usr/bin/env python3
from pathlib import Path
import re, sys, yaml
R=Path(__file__).resolve().parents[3]
esc=R/'src/esc'
driver=(esc/'src/esc_driver_node.cpp').read_text()
transport=(esc/'src/vesc_board.cpp').read_text()
launch=(esc/'launch/esc.launch.py').read_text()
winch=(esc/'src/winch_serial_node.cpp').read_text()
cfg=yaml.safe_load((esc/'config/ackermann_dual_vesc.yaml').read_text())['esc_driver']['ros__parameters']
assert 'src/vesc_board.cpp' in (esc/'CMakeLists.txt').read_text()
assert 'serial_board.cpp' not in (esc/'CMakeLists.txt').read_text()
assert not (esc/'src/serial_board.cpp').exists()
assert cfg['vehicle_mode']=='ackermann_dual_vesc'
assert cfg['board0_port']=='/dev/vesc_drive' and cfg['board1_port']=='/dev/vesc_steer'
assert cfg['baud']==921600
assert cfg['drive_left_pole_pairs']==15.0 and cfg['drive_right_pole_pairs']==15.0
assert cfg['require_steering_calibration_for_motion'] is True
for token in ('DRIVE_DUAL_HALL','STEER_LEFT_ENCODER','/esc/drive/left/erpm','/esc/drive/right/erpm','/esc/steer/encoder_ready','DUAL_HALL'):
    assert token in driver, token
assert 'B921600' in transport and 'COMM_GET_VALUES_SELECTIVE' in transport and 'COMM_FORWARD_CAN' in transport
assert 'ackermann_dual_vesc.yaml' in launch
assert '/esc/steer/encoder_ready' in winch
assert 'VESC_DRIVE:' in winch and 'VESC_STEER:' in winch
assert 'servo_deg' not in winch and 'SERVOTEST' not in winch
print('NATIVE_VESC_INTEGRATION_SELF_CHECK_PASS')
