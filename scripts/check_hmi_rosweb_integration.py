#!/usr/bin/env python3
from pathlib import Path
import sys
R=Path(__file__).resolve().parents[1]
checks=[]
def need(ok,msg): checks.append((bool(ok),msg))
web=(R/'src/navigation/web/web_server.cpp').read_text()
launch=(R/'src/navigation/launch/autonomous.launch.py').read_text()
schema=(R/'src/navigation/config/runtime_schema.yaml').read_text()
winch=(R/'src/esc/src/winch_serial_node.cpp').read_text()
f4=(R/'F4gateway/src/main.cpp').read_text()
hdr=(R/'F4gateway/src/HmiDisplay.h').read_text()
touch=(R/'F4gateway/src/HmiTouchService.cpp').read_text()
need('/home/otomasi2/ros' not in launch,'autonomous launch has no retired workspace runtime path')
need('mission_fsm.yaml: 1' in schema and 'gui/interface.yaml: 1' in schema,'mission + GUI are runtime-schema managed')
need('_runtime_config(nav_share, "navigation", "mission_fsm.yaml")' in launch,'mission FSM consumes canonical runtime YAML')
need('ackermann_dual_vesc.yaml' in web,'rosweb ESC catalog points to active dual profile')
need('geometry_sync_next_start' in web and 'FAILED_ROLLED_BACK' in web,'rosweb geometry + generic config are transactional')
need('integration_ok' in web and 'components' in web,'rosweb health exposes integration state')
for token in ['last_esc_ready_rx_','last_drive_connected_rx_','last_nav2_ready_rx_','last_pose_rx_']:
    need(token in winch,f'HMI bridge freshness: {token}')
for wire in ['SPD:','DRIVE_TGT:','DRIVE_ACT:','STEER_TARGET:','STEER_ACTUAL:','GOAL_DIST:','LOCSTATE:','TARGET:','NAV:','OBS:']:
    need(wire in winch and wire in f4,f'HMI wire contract {wire}')
need('touch_x0_{580U}' in hdr and 'touch_x1_{3440U}' in hdr and 'touch_y0_{330U}' in hdr and 'touch_y1_{3310U}' in hdr,'F4 v1 empirical touch range retained')
need('touch_rotate_{false}' in hdr and 'touch_invert_y_{true}' in hdr,'F4 v1 touch axes: no XY swap, Y inverted')
need('const bool rotate =' in touch and 'invertY' in touch,'field touch calibration infers orientation')
failed=[m for ok,m in checks if not ok]
for ok,msg in checks: print(('PASS' if ok else 'FAIL')+': '+msg)
print(f'HMI_ROSWEB_INTEGRATION_CHECK count={len(checks)} fail={len(failed)}')
sys.exit(1 if failed else 0)
