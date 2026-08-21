#!/usr/bin/env python3
"""Smoke test for trajectory_test_node: fake vehicle state, trigger start/stop,
verify published references are continuous and within limits."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from mavros_msgs.msg import State
from mav_controllers_ros.msg import TargetCommand

V_MAX, A_MAX, J_MAX, YAW_RATE = 8.0, 5.0, 40.0, 1.0
DT = 0.01


class Harness(Node):
    def __init__(self):
        super().__init__('traj_harness')
        self.odom_pub = self.create_publisher(Odometry, 'mavros/local_position/odom',
                                              qos_profile_sensor_data)
        self.state_pub = self.create_publisher(State, 'mavros/state', 10)
        self.motors_pub = self.create_publisher(Bool, 'geometric_controller/enable_motors', 10)
        self.sp_sub = self.create_subscription(TargetCommand, 'geometric_controller/setpoint',
                                               self.sp_cb, 50)
        self.status_sub = self.create_subscription(String, 'trajectory_test/status',
                                                   self.st_cb, 10)
        self.ref_path = self.actual_path = None
        self.sp_pose_count = 0
        self.create_subscription(Path, 'trajectory_test/reference_path',
                                 lambda m: setattr(self, 'ref_path', m), 10)
        self.create_subscription(Path, 'trajectory_test/actual_path',
                                 lambda m: setattr(self, 'actual_path', m), 10)
        self.create_subscription(PoseStamped, 'trajectory_test/setpoint_pose',
                                 self.sp_pose_cb, 10)
        self.start_cli = self.create_client(Trigger, 'trajectory_test/start')
        self.stop_cli = self.create_client(Trigger, 'trajectory_test/stop')
        self.samples = []
        self.phases = []
        self.timer = self.create_timer(0.02, self.tick)

    def sp_pose_cb(self, _):
        self.sp_pose_count += 1

    def tick(self):
        now = self.get_clock().now().to_msg()
        od = Odometry()
        od.header.stamp = now
        od.pose.pose.position.x = 0.0
        od.pose.pose.position.y = 0.0
        od.pose.pose.position.z = 5.0
        od.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(od)
        st = State()
        st.header.stamp = now
        st.mode = 'OFFBOARD'
        st.armed = True
        self.state_pub.publish(st)
        b = Bool()
        b.data = True
        self.motors_pub.publish(b)

    def sp_cb(self, m):
        t = time.monotonic()
        self.samples.append((t,
                             (m.position.x, m.position.y, m.position.z),
                             (m.velocity.x, m.velocity.y, m.velocity.z),
                             (m.acceleration.x, m.acceleration.y, m.acceleration.z),
                             m.yaw, m.yaw_dot))

    def st_cb(self, m):
        self.phases.append(m.data)
        self.get_logger().info(f'phase: {m.data}')


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def call(node, cli):
    if not cli.wait_for_service(timeout_sec=5.0):
        return None
    fut = cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    return fut.result()


def main():
    rclpy.init()
    h = Harness()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        rclpy.spin_once(h, timeout_sec=0.05)

    res = call(h, h.start_cli)
    print(f'START: success={getattr(res, "success", None)} msg={getattr(res, "message", None)}',
          flush=True)
    if not res or not res.success:
        sys.exit(2)

    t0 = time.monotonic()
    while time.monotonic() - t0 < 18.0:
        rclpy.spin_once(h, timeout_sec=0.05)

    res = call(h, h.stop_cli)
    print(f'STOP: success={getattr(res, "success", None)} msg={getattr(res, "message", None)}',
          flush=True)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 7.0:
        rclpy.spin_once(h, timeout_sec=0.05)

    s = h.samples
    print(f'samples: {len(s)}  phases seen: {h.phases}')
    vmax = max(norm(v) for _, _, v, _, _, _ in s)
    amax = max(norm(a) for _, _, _, a, _, _ in s)
    print(f'peak |v| = {vmax:.3f} (limit {V_MAX}), peak |a| = {amax:.3f} (limit {A_MAX})')

    max_dp = max_dv = max_dyaw = 0.0
    for i in range(1, len(s)):
        dt = s[i][0] - s[i - 1][0]
        if dt <= 0 or dt > 0.1:
            continue
        dp = norm([s[i][1][k] - s[i - 1][1][k] for k in range(3)])
        dv = norm([s[i][2][k] - s[i - 1][2][k] for k in range(3)])
        dy = abs(math.remainder(s[i][4] - s[i - 1][4], 2 * math.pi))
        max_dp = max(max_dp, dp / dt)
        max_dv = max(max_dv, dv / dt)
        max_dyaw = max(max_dyaw, dy / dt)
    print(f'worst inter-sample rates: |dp/dt| = {max_dp:.2f} m/s, '
          f'|dv/dt| = {max_dv:.2f} m/s^2, |dyaw/dt| = {max_dyaw:.3f} rad/s')

    end_v = norm(s[-1][2])
    end_a = norm(s[-1][3])
    print(f'final sample: |v| = {end_v:.4f}, |a| = {end_a:.4f}, p = {s[-1][1]}')

    viz_ok = (h.ref_path is not None and len(h.ref_path.poses) > 20 and
              h.actual_path is not None and h.sp_pose_count > 100)
    print(f'viz: ref_path poses = {len(h.ref_path.poses) if h.ref_path else 0}, '
          f'actual_path msgs = {h.actual_path is not None}, '
          f'setpoint_pose count = {h.sp_pose_count}, ok = {viz_ok}')

    ok = (viz_ok and vmax <= V_MAX + 0.01 and amax <= A_MAX + 0.01 and
          max_dp <= V_MAX * 1.6 and max_dv <= A_MAX * 3.0 and
          max_dyaw <= YAW_RATE * 1.6 and end_v < 1e-6 and
          'GOTO' in h.phases and 'HOLD' in h.phases)
    print('RESULT:', 'PASS' if ok else 'FAIL')
    h.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
