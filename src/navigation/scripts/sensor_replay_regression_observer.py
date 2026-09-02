#!/usr/bin/python3
"""Observe recomputed stack outputs during a sensor-only rosbag replay."""
import argparse, json, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool

class Observer(Node):
    def __init__(self, duration):
        super().__init__('sensor_replay_regression_observer')
        self.deadline=time.monotonic()+duration
        self.counts={'ekf':0,'amcl':0,'plan':0,'gate':0}; self.gate_true=0
        self.create_subscription(Odometry,'/odometry/filtered',lambda m:self.hit('ekf'),10)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:self.hit('amcl'),10)
        self.create_subscription(Path,'/smac_plan',lambda m:self.hit('plan'),10)
        self.create_subscription(Bool,'/system/autonomy_motion_allowed',self.gate,10)
    def hit(self,k): self.counts[k]+=1
    def gate(self,m):
        self.counts['gate']+=1; self.gate_true+=int(bool(m.data))

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--duration',type=float,default=30.0); ap.add_argument('--require-amcl',action='store_true'); ap.add_argument('--require-plan',action='store_true'); a=ap.parse_args()
    rclpy.init(); n=Observer(a.duration)
    while rclpy.ok() and time.monotonic()<n.deadline: rclpy.spin_once(n,timeout_sec=0.1)
    result={'counts':n.counts,'gate_true_samples':n.gate_true,'pass':n.counts['ekf']>3}
    if a.require_amcl: result['pass']=result['pass'] and n.counts['amcl']>0
    if a.require_plan: result['pass']=result['pass'] and n.counts['plan']>0
    print(json.dumps(result,sort_keys=True)); code=0 if result['pass'] else 2
    n.destroy_node(); rclpy.shutdown(); raise SystemExit(code)
if __name__=='__main__': main()
