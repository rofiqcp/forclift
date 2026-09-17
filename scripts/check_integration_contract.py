#!/usr/bin/env python3
from pathlib import Path
import yaml, sys
R=Path(__file__).resolve().parents[1]
m=yaml.safe_load((R/'config/integration_manifest.yaml').read_text())
assert Path(m['canonical_root'])==R
assert m['runtime_config_root']==str(R/'config/runtime')
esc=m['esc']; assert esc['transport']=='vesc_6_native' and esc['baud']==921600
assert esc['drive']['firmware_role']=='DRIVE_DUAL_HALL' and esc['drive']['firmware_role_id']==1
assert esc['steering']['firmware_role']=='STEER_LEFT_ENCODER' and esc['steering']['firmware_role_id']==2
cfg=yaml.safe_load((R/'config/runtime/esc/ackermann_dual_vesc.yaml').read_text())['esc_driver']['ros__parameters']
for k,v in [('board0_port','/dev/vesc_drive'),('board1_port','/dev/vesc_steer'),('baud',921600)]: assert cfg[k]==v,(k,cfg[k])
assert cfg['drive_left_pole_pairs']==15.0 and cfg['drive_right_pole_pairs']==15.0
geo=yaml.safe_load((R/'config/runtime/navigation/vehicle_geometry.yaml').read_text())['vehicle']
assert abs(float(geo['wheel_radius_m'])-float(m['geometry']['wheel_radius_m']))<1e-9
assert abs(float(geo['wheelbase_m'])-float(m['geometry']['wheelbase_m']))<1e-9
assert geo['wheel_radius_validated'] is False and geo['wheelbase_validated'] is False
f4=(R/'F4gateway/src/main.cpp').read_text(); winch=(R/'src/esc/src/winch_serial_node.cpp').read_text()
for x in ('VESC_LINK:','VESC_DRIVE:','VESC_STEER:','ENC:'): assert x in f4 and x in winch,x
print('AGV_INTEGRATION_CONTRACT_PASS')
