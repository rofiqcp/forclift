#!/usr/bin/python3
import argparse
import json
import os
import signal
import time
from pathlib import Path

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters

PLANNER_EXE = '/opt/ros/humble/lib/nav2_planner/planner_server'

def emit(**kw):
    print(json.dumps(kw, separators=(',', ':')), flush=True)

def find_pid(exe):
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='ignore')
            if raw.startswith(exe + ' '):
                return int(entry.name)
        except Exception:
            pass
    return 0

def call(node, srv_type, name, request, timeout=8.0):
    client = node.create_client(srv_type, name)
    if not client.wait_for_service(timeout_sec=min(4.0, timeout)):
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result() if future.done() else None

def get_state(node):
    res = call(node, GetState, '/planner_server/get_state', GetState.Request(), 5.0)
    return '' if res is None else str(res.current_state.label).strip().lower()

def wait_state(node, wanted, timeout):
    deadline = time.monotonic() + timeout
    last = ''
    while time.monotonic() < deadline:
        last = get_state(node)
        if last == wanted:
            return True, last
        time.sleep(0.35)
    return False, last

def transition(node, transition_id):
    req = ChangeState.Request()
    req.transition.id = int(transition_id)
    client = node.create_client(ChangeState, '/planner_server/change_state')
    if not client.wait_for_service(timeout_sec=4.0):
        return False
    client.call_async(req)
    return True

def param_to_python(value):
    t = int(value.type)
    if t == ParameterType.PARAMETER_BOOL:
        return bool(value.bool_value)
    if t == ParameterType.PARAMETER_INTEGER:
        return int(value.integer_value)
    if t == ParameterType.PARAMETER_DOUBLE:
        return float(value.double_value)
    if t == ParameterType.PARAMETER_STRING:
        return str(value.string_value)
    if t == ParameterType.PARAMETER_BYTE_ARRAY:
        return list(value.byte_array_value)
    if t == ParameterType.PARAMETER_BOOL_ARRAY:
        return list(value.bool_array_value)
    if t == ParameterType.PARAMETER_INTEGER_ARRAY:
        return list(value.integer_array_value)
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(value.double_array_value)
    if t == ParameterType.PARAMETER_STRING_ARRAY:
        return list(value.string_array_value)
    return None

def value_matches(expected, actual):
    if isinstance(expected, float) and isinstance(actual, (int, float)):
        return abs(float(expected) - float(actual)) <= 1e-9 * max(1.0, abs(float(expected)))
    return expected == actual

def read_parameter(node, target_node, parameter):
    req = GetParameters.Request()
    req.names = [parameter]
    res = call(node, GetParameters, f'{target_node}/get_parameters', req, 12.0)
    if res is None or not res.values:
        return None
    return param_to_python(res.values[0])

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--target-node', required=True)
    ap.add_argument('--parameter', required=True)
    ap.add_argument('--expected-json', required=True)
    args = ap.parse_args()
    expected = json.loads(args.expected_json)

    rclpy.init(args=None)
    node = rclpy.create_node(f'nav2_param_reload_{os.getpid()}', use_global_arguments=False)
    try:
        old_state = get_state(node)
        old_pid = find_pid(PLANNER_EXE)
        if not old_pid:
            emit(ok=False, verified=False, state='OWNER_MISSING', message='planner_server process tidak ditemukan')
            return 2
        if old_state == 'active':
            emit(ok=False, verified=False, state='BUSY_ACTIVE_NAV', old_pid=old_pid,
                 message='planner_server ACTIVE; hentikan navigation sebelum tuning parameter restart-required')
            return 3

        os.kill(old_pid, signal.SIGKILL)
        deadline = time.monotonic() + 12.0
        new_pid = 0
        while time.monotonic() < deadline:
            pid = find_pid(PLANNER_EXE)
            if pid and pid != old_pid:
                new_pid = pid
                break
            time.sleep(0.25)
        if not new_pid:
            emit(ok=False, verified=False, state='RESPAWN_FAILED', old_pid=old_pid,
                 message='planner_server tidak respawn setelah restart terisolasi')
            return 4

        deadline = time.monotonic() + 12.0
        state = ''
        while time.monotonic() < deadline:
            state = get_state(node)
            if state:
                break
            time.sleep(0.35)
        if not state:
            emit(ok=False, verified=False, state='LIFECYCLE_UNAVAILABLE', old_pid=old_pid, new_pid=new_pid,
                 message='planner_server respawn tetapi lifecycle service belum siap')
            return 5

        if state == 'unconfigured':
            transition(node, Transition.TRANSITION_CONFIGURE)
            ok_state, state = wait_state(node, 'inactive', 45.0)
            if not ok_state:
                emit(ok=False, verified=False, state='CONFIGURE_FAILED', old_pid=old_pid, new_pid=new_pid,
                     lifecycle_state=state, message='planner_server tidak mencapai INACTIVE setelah CONFIGURE')
                return 6
        elif state != 'inactive':
            emit(ok=False, verified=False, state='UNEXPECTED_LIFECYCLE', old_pid=old_pid, new_pid=new_pid,
                 lifecycle_state=state, message='planner_server bukan UNCONFIGURED/INACTIVE')
            return 7

        actual = read_parameter(node, args.target_node, args.parameter)
        verified = actual is not None and value_matches(expected, actual)
        restored = state
        if old_state in ('', 'unconfigured'):
            transition(node, Transition.TRANSITION_CLEANUP)
            _, restored = wait_state(node, 'unconfigured', 30.0)

        emit(ok=verified, verified=verified,
             state='APPLIED_RESTART' if verified else 'VERIFY_FAILED',
             strategy='planner_restart_configure_verify', old_pid=old_pid, new_pid=new_pid,
             original_lifecycle=old_state, restored_lifecycle=restored,
             target_node=args.target_node, parameter=args.parameter,
             expected=expected, actual_value=actual,
             message=('YAML dimuat oleh planner_server baru, parameter dideklarasikan saat CONFIGURE, '
                      'read-back cocok, lifecycle dikembalikan aman') if verified else
                     'planner_server baru aktif untuk validasi tetapi read-back parameter tidak cocok')
        return 0 if verified else 8
    finally:
        pass

if __name__ == '__main__':
    code = main()
    try:
        import sys
        sys.stdout.flush()
        sys.stderr.flush()
    finally:
        os._exit(int(code))
