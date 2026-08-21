# mav_controllers_ros

ROS 2 implementations of micro aerial vehicle (MAV) controllers: a **geometric
attitude controller** (recommended) and an **SE3 controller**.

The controller libraries are interface-agnostic; a separate node adapts them to
a specific autopilot interface. A MAVROS interface for PX4 is provided.

Tested with ROS 2 **Humble**. Dependencies are listed in [package.xml](package.xml).

> This branch contains production-hardening work (dt-correct integrator,
> anti-windup, altitude-priority saturation, rate feedforward, watchdogs,
> thrust-scale estimator). See [CHANGES.md](CHANGES.md) for the full list.

## Which controller should I use?

| | node | core library | status |
|---|---|---|---|
| **Geometric attitude** | `geometric_controller_node` | [GeometricAttitudeControl.cpp](src/GeometricAttitudeControl.cpp) | Recommended. Commands body rates + thrust. Flies well on PX4. |
| SE3 | `se3controller_node` | [SE3Controller.cpp](src/SE3Controller.cpp) | Legacy. Works with attitude + thrust setpoints; rate setpoints are not stable. |

Both follow the same structure, so the sections below describe the geometric
controller; the SE3 equivalents use `se3controller` in place of
`geometric_controller` in node, topic, launch and config names.

## Quick start (PX4 + MAVROS)

1. Set your vehicle mass and gains in [config/geometric_controller.yaml](config/geometric_controller.yaml).
2. Set `max_thrust` (total thrust of all motors, in Newtons) in
   [config/geometric_mavros.yaml](config/geometric_mavros.yaml). **Note:** this
   scales the effective gains — see [CHANGES.md](CHANGES.md).
3. Build and run:

```bash
colcon build --packages-select mav_controllers_ros
source install/setup.bash
ros2 launch mav_controllers_ros geometric_mavros_interface.launch.py
```

That launch file starts both the controller node and the MAVROS interface node.
To run them individually, use `geometric_controller.launch.py` and
`geometric_to_mavros.launch.py`.

## Nodes

### `geometric_controller_node`
The main controller. Consumes a reference trajectory and odometry, produces a
control command for an interface node. Source:
[src/geometric_attitude_control_node.cpp](src/geometric_attitude_control_node.cpp).

**Subscribes**
* `geometric_controller/setpoint` — [TargetCommand](msg/TargetCommand.msg):
  position, velocity, acceleration, jerk, yaw, yaw_dot (+ optional per-message gains).
* `geometric_controller/multi_dof_setpoint` — `trajectory_msgs/MultiDOFJointTrajectoryPoint`, alternative setpoint input.
* `geometric_controller/odom` — `nav_msgs/Odometry`, position/velocity feedback.
* `geometric_controller/enable_motors` — `std_msgs/Bool`, engages the integrators (published by the interface node).

**Publishes**
* `geometric_controller/cmd` — [SE3Command](msg/SE3Command.msg), consumed by an interface node.
* `geometric_controller/cmd_pose`, `geometric_controller/odom_pose` — `geometry_msgs/PoseStamped` for RViz.
* `geometric_controller/control_errors` — tracking errors for tuning/logging.

Watchdogs revert to a safe hold if the setpoint or odometry stream stalls
(`setpoint_timeout`, `odom_timeout`, `hold_on_setpoint_timeout`).

### `geometric_mavros_node` (interface)
Bridges the controller to MAVROS. Source:
[src/geometric_mavros_node.cpp](src/geometric_mavros_node.cpp).

**Subscribes**
* `geometric_mavros/odom` and `geometric_mavros/imu` — the odometry and autopilot
  orientations, which may differ (e.g. external VIO or motion capture); used to
  reconcile the two frames. `geometric_mavros/pose` + `geometric_mavros/twist`
  are an alternative input, republished as `geometric_mavros/combined_odometry`.
* `geometric_controller/cmd` — the controller output.
* `mavros/state` — arming state.

**Publishes**
* `mavros/attitude_target` — `mavros_msgs/AttitudeTarget` (remap to `/mavros/setpoint_raw/attitude`), after thrust scaling.
* `geometric_controller/enable_motors` — arming state, back to the controller node.
* `geometric_mavros/thrust_scale_estimate` — online hover-thrust estimate (`enable_thrust_estimator`).

All subscriptions are remapped in the launch files.

## Testing

Flight-test the controller with `trajectory_test_node` (setpoint, circle and
figure-8 trajectories, with feasibility limiting, geofence and safe abort):

```bash
ros2 launch mav_controllers_ros trajectory_test.launch.py
```

* [docs/TRAJECTORY_TESTING.md](docs/TRAJECTORY_TESTING.md) — step-by-step guide for SITL and field tests.
* [docs/TRAJECTORY_MATH.md](docs/TRAJECTORY_MATH.md) — the method and math behind the node.

Simpler test nodes: `static_setpoint_test_node` (hold one point) and
`circular_trajectory_node`, with matching `static_setpoint.launch.py` and
`circular_trajectory.launch.py`.

## Creating a custom interface node

To support an autopilot interface other than MAVROS:

1. Add a node modelled on [src/geometric_mavros_node.cpp](src/geometric_mavros_node.cpp) and register it in [CMakeLists.txt](CMakeLists.txt).
2. Add a config file modelled on [config/geometric_mavros.yaml](config/geometric_mavros.yaml).
3. Add a launch file modelled on [launch/geometric_to_mavros.launch.py](launch/geometric_to_mavros.launch.py), remapping the odometry and IMU topics of your interface node and the odometry topic of the controller node.

## History

* 2025: Production hardening of the geometric controller — see [CHANGES.md](CHANGES.md). Added `trajectory_test_node` (setpoint / circle / lemniscate).
* Sept 2023: Added `GeometricAttitudeControl` and its MAVROS interface, based on `mavros_controllers` with modifications. Tested in PX4 SITL with the `x500` model; works better than `SE3Controller`.
* Sept 2023: Added static setpoint and circular trajectory test nodes.
* Sept 2023: `SE3Controller` tested in PX4 SITL with `x500` using attitude + thrust setpoints.
