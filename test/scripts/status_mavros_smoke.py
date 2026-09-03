"""Feed geometric_mavros_node the inputs it needs, then cut the SE3 command
stream and print the mavros-side status at each stage."""
import rclpy, time, math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State
from diagnostic_msgs.msg import DiagnosticStatus
from mav_controllers_ros.msg import SE3Command

MASS, G = 1.5, 9.81

class D(Node):
    def __init__(self):
        super().__init__('mavros_status_smoke')
        self.pose = self.create_publisher(PoseStamped, 'geometric_mavros/pose', qos_profile_sensor_data)
        self.tw = self.create_publisher(TwistStamped, 'geometric_mavros/twist', qos_profile_sensor_data)
        self.imu = self.create_publisher(Imu, 'geometric_mavros/imu', qos_profile_sensor_data)
        self.st = self.create_publisher(State, 'mavros/state', 10)
        self.cmd = self.create_publisher(SE3Command, 'geometric_controller/cmd', 10)
        self.create_subscription(DiagnosticStatus, 'geometric_mavros/status', self.cb, 10)
        self.send = True; self.last = None
        self.create_timer(0.02, self.tick)
    def tick(self):
        now = self.get_clock().now().to_msg()
        p = PoseStamped(); p.header.stamp = now; p.header.frame_id='map'
        p.pose.position.z = 3.0; p.pose.orientation.w = 1.0; self.pose.publish(p)
        t = TwistStamped(); t.header.stamp = now; t.header.frame_id='map'; self.tw.publish(t)
        i = Imu(); i.header.stamp = now; i.orientation.w = 1.0
        i.linear_acceleration.z = G; self.imu.publish(i)
        s = State(); s.header.stamp = now; s.armed = True; s.mode='OFFBOARD'; self.st.publish(s)
        if self.send:
            c = SE3Command(); c.header.stamp = now; c.header.frame_id='map'
            c.force.z = MASS*G          # hover thrust, straight up
            c.orientation.w = 1.0
            self.cmd.publish(c)
    def cb(self, m):
        self.last = {kv.key: kv.value for kv in m.values}
        self.last['_level'] = int.from_bytes(m.level,'little') if isinstance(m.level, bytes) else m.level
        self.last['_msg'] = m.message

def show(n, tag):
    d = n.last or {}
    keys = ('_level','_msg','armed','se3_cmd_age_s','cmd_timeout_active','setpoints_stopped',
            'throttle','throttle_raw','thrust_cmd_n','max_thrust_n','thrust_scale_est','hover_throttle_pred')
    print(f'--- {tag}: ' + ', '.join(f'{k.lstrip("_")}={d.get(k)}' for k in keys), flush=True)

rclpy.init(); n = D()
def spin(sec):
    t0=time.time()
    while time.time()-t0 < sec: rclpy.spin_once(n, timeout_sec=0.02)
spin(6);  show(n,'commanding')
n.send=False
spin(1.5); show(n,'cmd cut 1.5 s (expect level-hold failsafe)')
spin(3.0); show(n,'cmd cut 4.5 s (expect setpoints stopped)')
n.send=True
spin(2);  show(n,'resumed')
rclpy.shutdown()
