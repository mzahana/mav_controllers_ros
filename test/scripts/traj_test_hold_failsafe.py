#!/usr/bin/env python3
"""Verify the controller's hold-on-setpoint-loss failsafe: stream setpoints,
stop the stream (simulating a planner crash/shutdown), and check the
controller keeps publishing commands that hold the current position."""
import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from mav_controllers_ros.msg import TargetCommand, SE3Command

POS = (2.0, -1.0, 5.0)  # "vehicle" position fed to the controller


class H(Node):
    def __init__(self):
        super().__init__('hold_failsafe_harness')
        self.odom_pub = self.create_publisher(Odometry, 'geometric_controller/odom',
                                              qos_profile_sensor_data)
        self.motors_pub = self.create_publisher(Bool, 'geometric_controller/enable_motors', 10)
        self.sp_pub = self.create_publisher(TargetCommand, 'geometric_controller/setpoint', 10)
        self.cmds = []
        self.cmd_poses = []
        self.create_subscription(SE3Command, 'geometric_controller/cmd',
                                 lambda m: self.cmds.append(time.monotonic()), 10)
        self.create_subscription(PoseStamped, 'geometric_controller/cmd_pose',
                                 lambda m: self.cmd_poses.append(m), 10)
        self.stream_setpoints = True
        self.create_timer(0.02, self.tick)

    def tick(self):
        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = 'map'
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = POS
        od.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(od)
        b = Bool()
        b.data = True
        self.motors_pub.publish(b)
        if self.stream_setpoints:
            sp = TargetCommand()
            sp.header.stamp = od.header.stamp
            sp.position.x, sp.position.y, sp.position.z = POS
            self.sp_pub.publish(sp)


def spin_for(node, sec):
    t0 = time.monotonic()
    while time.monotonic() - t0 < sec:
        rclpy.spin_once(node, timeout_sec=0.05)


rclpy.init()
h = H()
spin_for(h, 3.0)
n_streaming = len(h.cmds)
print(f'commands while streaming: {n_streaming}')

print('>>> stopping setpoint stream (simulated planner shutdown)', flush=True)
h.stream_setpoints = False
spin_for(h, 1.2)          # let setpoint_timeout (1.0 s) expire
h.cmds.clear()
h.cmd_poses.clear()
spin_for(h, 3.0)          # hold failsafe should now be commanding

rate = len(h.cmds) / 3.0
last = h.cmd_poses[-1].pose.position if h.cmd_poses else None
hold_ok = last is not None and abs(last.x - POS[0]) < 1e-3 and \
    abs(last.y - POS[1]) < 1e-3 and abs(last.z - POS[2]) < 1e-3
print(f'commands during hold: {len(h.cmds)} ({rate:.0f} Hz), '
      f'hold target = {(last.x, last.y, last.z) if last else None}')

print('>>> resuming setpoint stream', flush=True)
h.stream_setpoints = True
spin_for(h, 1.0)
resumed = len(h.cmds) > 0

ok = n_streaming > 15 and rate > 30 and hold_ok and resumed
print(f'streaming ok: {n_streaming > 15}, hold rate ok: {rate > 30}, '
      f'hold position ok: {hold_ok}, resume ok: {resumed}')
print('RESULT:', 'PASS' if ok else 'FAIL')
sys.exit(0 if ok else 1)
