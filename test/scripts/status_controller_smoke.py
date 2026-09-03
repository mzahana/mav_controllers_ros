"""Drive the controller with odom + setpoints, then cut the setpoint stream,
and print the status topic's key fields at each stage."""
import rclpy, time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticStatus
from mav_controllers_ros.msg import TargetCommand

class D(Node):
    def __init__(self):
        super().__init__('status_smoke')
        self.od = self.create_publisher(Odometry, 'geometric_controller/odom', qos_profile_sensor_data)
        self.sp = self.create_publisher(TargetCommand, 'geometric_controller/setpoint', 10)
        self.mo = self.create_publisher(Bool, 'geometric_controller/enable_motors', 10)
        self.create_subscription(DiagnosticStatus, 'geometric_controller/status', self.cb, 10)
        self.send_sp = True
        self.last = None
        self.create_timer(0.02, self.odom_tick)
        self.create_timer(0.05, self.sp_tick)
    def odom_tick(self):
        m = Odometry(); m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.pose.position.z = 3.0
        m.pose.pose.orientation.w = 1.0
        self.od.publish(m)
        self.mo.publish(Bool(data=True))
    def sp_tick(self):
        if not self.send_sp: return
        c = TargetCommand(); c.header.stamp = self.get_clock().now().to_msg()
        c.position.z = 3.5   # 0.5 m step -> nonzero pos error
        self.sp.publish(c)
    def cb(self, msg):
        self.last = {kv.key: kv.value for kv in msg.values}
        self.last['_level'] = int.from_bytes(msg.level, 'little') if isinstance(msg.level, bytes) else msg.level
        self.last['_msg'] = msg.message

def show(n, tag):
    d = n.last or {}
    keys = ('_level','_msg','odom_rate_hz','setpoint_rate_hz','control_rate_hz','control_dt_s',
            'setpoint_age_s','hold_active','motors_enabled','pos_err_z_m','pos_int_z','saturated','kx','tilt_cmd_deg')
    print(f'--- {tag}: ' + ', '.join(f'{k.lstrip("_")}={d.get(k)}' for k in keys), flush=True)

rclpy.init(); n = D()
t0 = time.time()
while time.time() - t0 < 6: rclpy.spin_once(n, timeout_sec=0.05)
show(n, 'streaming')
n.send_sp = False
t0 = time.time()
while time.time() - t0 < 4: rclpy.spin_once(n, timeout_sec=0.05)
show(n, 'setpoints cut (expect hold_active=true)')
n.send_sp = True
t0 = time.time()
while time.time() - t0 < 3: rclpy.spin_once(n, timeout_sec=0.05)
show(n, 'setpoints resumed')
rclpy.shutdown()
