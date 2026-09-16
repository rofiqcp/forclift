import math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
class N(Node):
    def __init__(self):
        super().__init__('check_scan_full_once')
        self.create_subscription(LaserScan, '/scan_nav', self.cb, qos_profile_sensor_data)
    def cb(self, msg):
        vals=list(msg.ranges); finite=[v for v in vals if math.isfinite(v)]
        print('frame',msg.header.frame_id,'n',len(vals),'finite',len(finite),'invalid',len(vals)-len(finite),flush=True)
        if finite:
            print('range_minmax',min(finite),max(finite),'mean',sum(finite)/len(finite),flush=True)
            print('lt3',sum(v<3 for v in finite),'lt4',sum(v<4 for v in finite),'lt5',sum(v<5 for v in finite),'lt5.5',sum(v<5.5 for v in finite),'lt5.9',sum(v<5.9 for v in finite),flush=True)
        rclpy.shutdown()
rclpy.init(); n=N(); rclpy.spin(n)
