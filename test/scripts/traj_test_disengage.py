#!/usr/bin/env python3
"""Verify OFFBOARD dropout mid-trajectory aborts to HOLD and stays held."""
import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from mavros_msgs.msg import State
from mav_controllers_ros.msg import TargetCommand


class H(Node):
    def __init__(self):
        super().__init__('traj_disengage_harness')
        self.mode = 'OFFBOARD'
        self.odom_pub = self.create_publisher(Odometry, 'mavros/local_position/odom',
                                              qos_profile_sensor_data)
        self.state_pub = self.create_publisher(State, 'mavros/state', 10)
        self.motors_pub = self.create_publisher(Bool, 'geometric_controller/enable_motors', 10)
        self.create_subscription(TargetCommand, 'geometric_controller/setpoint', self.sp, 50)
        self.create_subscription(String, 'trajectory_test/status', self.st, 10)
        self.start_cli = self.create_client(Trigger, 'trajectory_test/start')
        self.phases = []
        self.last_sp = None
        self.create_timer(0.02, self.tick)

    def tick(self):
        now = self.get_clock().now().to_msg()
        od = Odometry()
        od.header.stamp = now
        od.pose.pose.position.z = 5.0
        od.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(od)
        s = State()
        s.header.stamp = now
        s.mode = self.mode
        s.armed = True
        self.state_pub.publish(s)
        b = Bool()
        b.data = True
        self.motors_pub.publish(b)

    def sp(self, m):
        self.last_sp = m

    def st(self, m):
        self.phases.append(m.data)
        self.get_logger().info(f'phase: {m.data}')


def spin_for(node, sec):
    t0 = time.monotonic()
    while time.monotonic() - t0 < sec:
        rclpy.spin_once(node, timeout_sec=0.05)


rclpy.init()
h = H()
spin_for(h, 2.0)
h.start_cli.wait_for_service(timeout_sec=5.0)
fut = h.start_cli.call_async(Trigger.Request())
rclpy.spin_until_future_complete(h, fut, timeout_sec=5.0)
print('START:', fut.result().success, fut.result().message, flush=True)
spin_for(h, 8.0)   # into TRACK
print('>>> dropping OFFBOARD', flush=True)
h.mode = 'POSCTL'
spin_for(h, 2.0)
in_hold_after_drop = h.phases and h.phases[-1] == 'HOLD'
h.mode = 'OFFBOARD'
spin_for(h, 3.0)
no_restart = h.phases[-1] == 'HOLD'  # must NOT auto-restart without a new trigger
sp = h.last_sp
hold_at_odom = abs(sp.position.z - 5.0) < 0.01 and abs(sp.velocity.x) < 1e-9
print(f'phases: {h.phases}')
print(f'HOLD after dropout: {in_hold_after_drop}, no auto-restart: {no_restart}, '
      f'hold tracks vehicle pose: {hold_at_odom}')
ok = in_hold_after_drop and no_restart and hold_at_odom and 'TRACK' in h.phases
print('RESULT:', 'PASS' if ok else 'FAIL')
sys.exit(0 if ok else 1)
