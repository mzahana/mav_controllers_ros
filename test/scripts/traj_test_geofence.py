"""Reproduce the user's case: absolute setpoint (0,0,0) with the vehicle
hovering at 3 m, and print the node's rejection message."""
import rclpy, time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class H(Node):
    def __init__(self):
        super().__init__('fence_check')
        self.od = self.create_publisher(Odometry, 'mavros/local_position/odom', qos_profile_sensor_data)
        self.st = self.create_publisher(State, 'mavros/state', 10)
        self.mo = self.create_publisher(Bool, 'geometric_controller/enable_motors', 10)
        self.create_timer(0.02, self.tick)
    def tick(self):
        now = self.get_clock().now().to_msg()
        o = Odometry(); o.header.stamp = now; o.header.frame_id='map'
        o.pose.pose.position.x = 5.0; o.pose.pose.position.y = 0.0
        o.pose.pose.position.z = 3.0; o.pose.pose.orientation.w = 1.0
        self.od.publish(o)
        s = State(); s.header.stamp = now; s.armed=True; s.mode='OFFBOARD'; self.st.publish(s)
        self.mo.publish(Bool(data=True))

rclpy.init(); n = H()
t0=time.time()
while time.time()-t0 < 2: rclpy.spin_once(n, timeout_sec=0.02)

def setp(items):
    cli = n.create_client(SetParameters, '/trajectory_test_node/set_parameters')
    cli.wait_for_service(timeout_sec=5.0)
    req = SetParameters.Request()
    for name, val in items:
        p = Parameter(name=name)
        if isinstance(val, bool):
            p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=val)
        elif isinstance(val, str):
            p.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=val)
        else:
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(val))
        req.parameters.append(p)
    f = cli.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)

def start():
    cli = n.create_client(Trigger, '/trajectory_test/start')
    cli.wait_for_service(timeout_sec=5.0)
    f = cli.call_async(Trigger.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)
    return f.result()

for tag, params in [
    ("absolute (0,0,0) -- the reported case",
     [("trajectory_type","setpoint"),("relative_to_start",False),
      ("setpoint.x",0.0),("setpoint.y",0.0),("setpoint.z",0.0)]),
    ("absolute (0,0,3) -- above min_z",
     [("trajectory_type","setpoint"),("relative_to_start",False),
      ("setpoint.x",0.0),("setpoint.y",0.0),("setpoint.z",3.0)]),
    ("absolute (0,0,50) -- above max_z",
     [("setpoint.z",50.0)]),
    ("absolute (100,0,3) -- beyond max_radius_xy",
     [("setpoint.x",100.0),("setpoint.z",3.0)]),
]:
    setp(params)
    t0=time.time()
    while time.time()-t0 < 0.5: rclpy.spin_once(n, timeout_sec=0.02)
    r = start()
    print(f"\n{tag}\n  success={r.success}\n  {r.message}", flush=True)
    # back to hold for the next case
    cli = n.create_client(Trigger, '/trajectory_test/stop'); cli.wait_for_service(timeout_sec=5.0)
    f = cli.call_async(Trigger.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)
    t0=time.time()
    while time.time()-t0 < 1.0: rclpy.spin_once(n, timeout_sec=0.02)
rclpy.shutdown()
