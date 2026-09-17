#!/usr/bin/python3
import json
import math
import os
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from action_msgs.srv import CancelGoal
from yolo_obstacle_detection_ros2.msg import AlignmentState


class MissionFSM(Node):
    STATES = ('IDLE','NAV_TO_B','CAMERA_CHECK','FORWARD_5S','LIFT_UP_5S',
              'NAV_TO_C','LIFT_DOWN_5S','REVERSE_5S','NAV_TO_A',
              'MISSION_COMPLETE','ABORTED','FAULT')
    def __init__(self):
        super().__init__('mission_fsm')
        for name, default in {
            'waypoints_yaml':'', 'allow_timed_motion':False,
            'forward_speed_mps':0.0, 'reverse_speed_mps':0.0,
            'forward_duration_sec':5.0, 'lift_up_duration_sec':5.0,
            'lift_down_duration_sec':5.0, 'reverse_duration_sec':5.0,
            'camera_timeout_sec':20.0, 'alignment_hold_sec':0.5,
            'goal_timeout_sec':120.0, 'publish_rate_hz':20.0,
        }.items(): self.declare_parameter(name, default)
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        live_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', qos)
        self.cmd_pub = self.create_publisher(Twist, '/mission/cmd_vel_raw', qos)
        self.override_pub = self.create_publisher(Bool, '/mission/velocity_override', state_qos)
        self.winch_pub = self.create_publisher(String, '/winch/command', qos)
        self.status_pub = self.create_publisher(String, '/mission/status', state_qos)
        self.create_subscription(String, '/navigation/goal_event', self._goal_event_cb, qos)
        self.create_subscription(AlignmentState, '/fork_alignment/state', self._align_cb, state_qos)
        self.create_subscription(Bool, '/winch/connected', lambda m:setattr(self,'winch_connected',bool(m.data)), state_qos)
        self.create_subscription(Bool, '/winch/top_limit', lambda m:setattr(self,'top_limit',bool(m.data)), state_qos)
        self.create_subscription(Bool, '/winch/bottom_limit', lambda m:setattr(self,'bottom_limit',bool(m.data)), state_qos)
        # Readiness outputs are transient-local state topics. Using the same QoS
        # prevents a late-starting FSM from missing the last proven readiness.
        self.create_subscription(Bool, '/system/nav2_ready', lambda m:setattr(self,'nav2_ready',bool(m.data)), state_qos)
        self.create_subscription(Bool, '/system/autonomy_motion_allowed', lambda m:setattr(self,'autonomy_allowed',bool(m.data)), state_qos)
        self.create_subscription(Bool, '/safety/estop', self._estop_cb, live_qos)
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.create_service(Trigger, '/mission/start', self._srv_start)
        self.create_service(Trigger, '/mission/abort', self._srv_abort)
        self.create_service(Trigger, '/mission/reset', self._srv_reset)
        self.state='IDLE'; self.detail='ready'; self.enter_t=time.monotonic(); self.goal_sent_t=0.0
        self.waiting_goal=False; self.goal_id=''; self.goal_result=''
        self.alignment_ready=False; self.alignment_seen_t=0.0; self.alignment_ready_since=0.0
        self.winch_connected=False; self.top_limit=False; self.bottom_limit=False
        self.nav2_ready=False; self.autonomy_allowed=False; self.estop=False
        self.waypoints={}; self.override=False; self.last_status=0.0
        hz=max(5.0,float(self.get_parameter('publish_rate_hz').value))
        self.timer=self.create_timer(1.0/hz,self._tick)
        self._publish_safe_stop(); self._publish_status(force=True)
        self.get_logger().info('Mission FSM READY / IDLE; no motion until /mission/start validation passes')

    def _param(self,n): return self.get_parameter(n).value
    def _elapsed(self): return time.monotonic()-self.enter_t
    def _publish_override(self, value):
        self.override=bool(value); m=Bool(); m.data=self.override; self.override_pub.publish(m)
    def _publish_cmd(self, speed=0.0):
        m=Twist(); m.linear.x=float(speed); self.cmd_pub.publish(m)
    def _winch(self, cmd):
        m=String(); m.data=cmd; self.winch_pub.publish(m)
    def _publish_safe_stop(self):
        self._publish_cmd(0.0); self._publish_override(False); self._winch('STOP')
    def _cancel_nav(self):
        if self.cancel_client.service_is_ready(): self.cancel_client.call_async(CancelGoal.Request())
        self.waiting_goal=False

    def _load_waypoints(self):
        path=str(self._param('waypoints_yaml') or '')
        if not path or not os.path.isfile(path): return False, f'waypoints YAML tidak ditemukan: {path}'
        try:
            data=yaml.safe_load(open(path,'r',encoding='utf-8')) or {}
            wp=(data.get('navigation_waypoints') or ((data.get('navigation') or {}).get('waypoints') or {}))
            out={}
            for k in ('A','B','C'):
                v=wp.get(k) or {}
                if not bool(v.get('enabled',False)): return False, f'Waypoint {k} belum disimpan/enabled'
                out[k]={'x':float(v['x']),'y':float(v['y']),'yaw_deg':float(v['yaw_deg'])}
            self.waypoints=out; return True,'OK'
        except Exception as e: return False,f'waypoint parse error: {e}'

    def _validate_start(self):
        ok,msg=self._load_waypoints()
        if not ok:return False,msg
        if self.estop:return False,'E-STOP aktif'
        if not self.nav2_ready:return False,'Nav2 belum READY'
        if not self.autonomy_allowed:return False,'autonomy motion gate belum READY'
        if not self.winch_connected:return False,'winch belum connected'
        if not bool(self._param('allow_timed_motion')):return False,'Timed motion masih LOCKED (allow_timed_motion=false)'
        f=float(self._param('forward_speed_mps')); r=float(self._param('reverse_speed_mps'))
        if f<=0.0 or r<=0.0:return False,'forward/reverse speed harus > 0 sebelum mission dapat dijalankan'
        return True,'READY'

    def _srv_start(self,req,res):
        if self.state not in ('IDLE','MISSION_COMPLETE','ABORTED','FAULT'):
            res.success=False; res.message=f'Mission sedang aktif: {self.state}'; return res
        ok,msg=self._validate_start()
        if not ok: res.success=False; res.message=msg; self.detail=msg; self._publish_status(force=True); return res
        self._enter('NAV_TO_B','Mission start: menuju B'); res.success=True; res.message='FSM STARTED: A/HOME -> B'; return res
    def _srv_abort(self,req,res):
        self._abort('operator abort'); res.success=True; res.message='Mission ABORTED; velocity=0, winch STOP, Nav2 cancel'; return res
    def _srv_reset(self,req,res):
        self._cancel_nav(); self._publish_safe_stop(); self._enter('IDLE','reset'); res.success=True; res.message='Mission reset ke IDLE'; return res

    def _estop_cb(self,msg):
        self.estop=bool(msg.data)
        if self.estop and self.state not in ('IDLE','ABORTED','FAULT'): self._fault('E-STOP aktif')
    def _align_cb(self,msg):
        now=time.monotonic(); self.alignment_seen_t=now; self.alignment_ready=bool(msg.ready_for_insertion)
        if self.alignment_ready:
            if self.alignment_ready_since<=0.0:self.alignment_ready_since=now
        else:self.alignment_ready_since=0.0
    def _goal_event_cb(self,msg):
        if not self.waiting_goal:return
        try:d=json.loads(msg.data)
        except Exception:return
        if str(d.get('event','')).upper()!='RESULT':return
        reason=str(d.get('reason','')).upper(); self.goal_result=reason; self.waiting_goal=False
        if reason=='SUCCEEDED':
            if self.state=='NAV_TO_B':self._enter('CAMERA_CHECK','B reached; menunggu kamera/alignment')
            elif self.state=='NAV_TO_C':self._enter('LIFT_DOWN_5S','C reached; lift turun')
            elif self.state=='NAV_TO_A':self._enter('MISSION_COMPLETE','A/HOME reached; mission complete')
        elif self.state not in ('ABORTED','FAULT'):
            self._fault(f'Nav2 {self.goal_id} result={reason or "UNKNOWN"}')

    def _send_goal(self,key):
        w=self.waypoints[key]; m=PoseStamped(); m.header.frame_id='map'; m.header.stamp=self.get_clock().now().to_msg()
        m.pose.position.x=w['x']; m.pose.position.y=w['y']; yaw=math.radians(w['yaw_deg'])
        m.pose.orientation.z=math.sin(yaw/2.0); m.pose.orientation.w=math.cos(yaw/2.0)
        self.goal_id=key; self.goal_result=''; self.waiting_goal=True; self.goal_sent_t=time.monotonic(); self.goal_pub.publish(m)

    def _enter(self,state,detail=''):
        if state not in self.STATES: raise ValueError(state)
        self.state=state; self.detail=detail; self.enter_t=time.monotonic(); self.alignment_ready_since=0.0
        self._publish_cmd(0.0); self._publish_override(False)
        if state=='NAV_TO_B': self._send_goal('B')
        elif state=='NAV_TO_C': self._send_goal('C')
        elif state=='NAV_TO_A': self._send_goal('A')
        elif state=='LIFT_UP_5S': self._winch('UP')
        elif state=='LIFT_DOWN_5S': self._winch('DOWN')
        elif state in ('MISSION_COMPLETE','ABORTED','FAULT','IDLE'): self._winch('STOP')
        self._publish_status(force=True)

    def _fault(self,reason):
        self._cancel_nav(); self._publish_safe_stop(); self._enter('FAULT',reason)
    def _abort(self,reason):
        self._cancel_nav(); self._publish_safe_stop(); self._enter('ABORTED',reason)

    def _tick(self):
        if self.estop and self.state not in ('IDLE','ABORTED','FAULT'): self._fault('E-STOP aktif'); return
        if self.state in ('NAV_TO_B','NAV_TO_C','NAV_TO_A'):
            self._publish_override(False)
            if self.waiting_goal and time.monotonic()-self.goal_sent_t>float(self._param('goal_timeout_sec')):
                self._fault(f'{self.state} timeout')
        elif self.state=='CAMERA_CHECK':
            self._publish_override(False)
            hold=float(self._param('alignment_hold_sec'))
            if self.alignment_ready and self.alignment_ready_since>0 and time.monotonic()-self.alignment_ready_since>=hold:
                self._enter('FORWARD_5S','alignment valid; maju ke pallet')
            elif self._elapsed()>float(self._param('camera_timeout_sec')): self._fault('camera/alignment timeout')
        elif self.state=='FORWARD_5S':
            self._publish_override(True); self._publish_cmd(float(self._param('forward_speed_mps')))
            if self._elapsed()>=float(self._param('forward_duration_sec')):
                self._publish_cmd(0.0); self._publish_override(False); self._enter('LIFT_UP_5S','forward complete; lift UP')
        elif self.state=='LIFT_UP_5S':
            self._publish_override(False)
            if self.top_limit or self._elapsed()>=float(self._param('lift_up_duration_sec')):
                self._winch('STOP'); self._enter('NAV_TO_C','lift up complete; menuju C')
        elif self.state=='LIFT_DOWN_5S':
            self._publish_override(False)
            if self.bottom_limit or self._elapsed()>=float(self._param('lift_down_duration_sec')):
                self._winch('STOP'); self._enter('REVERSE_5S','lift down complete; mundur')
        elif self.state=='REVERSE_5S':
            self._publish_override(True); self._publish_cmd(-float(self._param('reverse_speed_mps')))
            if self._elapsed()>=float(self._param('reverse_duration_sec')):
                self._publish_cmd(0.0); self._publish_override(False); self._enter('NAV_TO_A','reverse complete; kembali HOME A')
        else:
            self._publish_override(False); self._publish_cmd(0.0)
        self._publish_status()

    def _publish_status(self,force=False):
        now=time.monotonic()
        if not force and now-self.last_status<0.25:return
        timer_states={'FORWARD_5S':'forward_duration_sec','LIFT_UP_5S':'lift_up_duration_sec','LIFT_DOWN_5S':'lift_down_duration_sec','REVERSE_5S':'reverse_duration_sec'}
        total=float(self._param(timer_states[self.state])) if self.state in timer_states else 0.0
        remaining=max(0.0,total-self._elapsed()) if total else 0.0
        m=String(); m.data=json.dumps({
            'state':self.state,'detail':self.detail,'active':self.state not in ('IDLE','MISSION_COMPLETE','ABORTED','FAULT'),
            'target':self.goal_id if self.state.startswith('NAV_TO_') else '', 'timer_remaining_sec':round(remaining,2),
            'alignment_ready':self.alignment_ready,'winch_connected':self.winch_connected,
            'top_limit':self.top_limit,'bottom_limit':self.bottom_limit,'nav2_ready':self.nav2_ready,
            'autonomy_allowed':self.autonomy_allowed,'estop':self.estop,'velocity_override':self.override,
            'timed_motion_unlocked':bool(self._param('allow_timed_motion')),
            'forward_speed_mps':float(self._param('forward_speed_mps')),'reverse_speed_mps':float(self._param('reverse_speed_mps')),
        },separators=(',',':'))
        self.status_pub.publish(m); self.last_status=now


def main(args=None):
    rclpy.init(args=args); node=MissionFSM()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._publish_safe_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':main()
