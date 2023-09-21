# px4_geometric_controller
A ROS2 package that provides implementation of a geometric controller, SE(3).
The implementation of the SE(3) controller is general in the sense it is not implemented for a particular interface such as MAVROS. 
For particular interface, a separate ROS 2 node is used. A MAVROS insterface is currently supported to be used with a PX4 autopilot.

# Compatability and dependencies
This package is tested with ROS 2 `humbel`.

Depedencies can be found the in the [package.xml](package.xml) file.

# Nodes

## se3controller_node
This node is the main ros 2 interface. The core SE3Controller library is implemented in [src/SE3Controller.cpp](src/SE3Controller.cpp). This node (se3controller_node) uses this library and interface the controller to ROS 2 system. This node is implemented in [src/se3_controller_node.cpp](src/se3_controller_node.cpp).

### Subscriptions
* `se3controller/setpoint` This is the setpoint that that the controller needs. Should be published by another node. This topic uses a custom message of type `px4_geometric_controller::msg::TargetCommand` This custom message is defined in [msg/TargetCommand.msg](msg/TargetCommand.msg).
* `se3controller/odom` Odometry topic. Uses message `nav_msgs::msg:Odometry`. This is used to get the position/velocity feedback.

* `se3controller/enable_motors`: This topic uses `std_msgs::msg::Bool` to get the state of motors (armed or not). This is used to engage the integrators coefiicients accordingly.

**NOTE** All the above subscriptions can be remapped to the right topics in the launch file [launch/se3controller.launch.py](launch/se3controller.launch.py)

### Publishers
* `se3controller/cmd`: This uses a custom message of type [SE3Command](msg/SE3Command.msg). It publishes the output of the SE3 controller to be consumed by a particular interface (e.g. MAVROS)

* `se3controller/cmd_pose`: This uses `geometry_msgs::msg::PoseStamped`. It published the position part of the command for visualizatino in RViz2.

### Launch file
* [launch/se3controller.launch.py](launch/se3controller.launch.py)

## se3controller_mavros_node
This nodes is an interface between the main node `se3controller_node` and MAVROS.

### Subscriptions
* `se3controller_mavros/odom`: Uses message type `nav_msgs::msg::Odometry`. This is basically used to get the quaternion from the odmoetry system, which could be not the same as the odmoetry from the autopilot. Examples, are like, external VIO, or motion capture systems.

* `se3controller_mavros/imu`: Uses message type `sensor_msgs::msg::Imu`. This is basically used to get the quaternion from the autopilot IMU (published by MAVROS), which could be not the same as the odmoetry system that is used. 

The two above subscrioptions are used to handle the difference in orientation that is provided by the odometry and the autopilot system, which could be different.

* `mavros/state`: Uses `mavros_msg::msg::State`. This is used to get the arming state (motors are started or not). This is passed to the main node (se3controller_node) using the `se3controller/enable_motors` topic of the main node.

**NOTE** All the above subscriptions can be remapped to the right topics in the launch file [launch/se3controller_to_mavros.launch.py](launch/se3controller_to_mavros.launch.py)

### Publishers

* `mavros/attitude_target`: This uses `mavros_msgs::msg::AttitudeTarget`. It publishes the SE3 control command `se3controller/cmd` from the main node (after doing the neccessarty scaling to thrust) to the appropriate MAVROS topic 
(usually`/mavros/setpoint_raw/target_attitude`)

* `se3controller/enable_motors`: Uses `std_msgs::msg:Bool`. It published the motors state (armed or not) which is consumed by the main node `se3controller_node` to engage the integrators coeeficients in the controller computations. 

### Launch file
* [launch/se3controller_to_mavros.launch.py](launch/se3controller_to_mavros.launch.py)

# How to use this controller?
To be done.

# Notes
 **This controller has not been tested yet**
