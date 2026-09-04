#include <mav_controllers_ros/GeometricAttitudeControl.h>
#include <cmath>
#include <limits>
#include <cstdio>
#include "mav_controllers_ros/msg/se3_command.hpp"
#include "mav_controllers_ros/msg/target_command.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include "mavros_msgs/msg/attitude_target.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/bool.hpp>
#include "mav_controllers_ros/msg/control_errors.hpp"
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <sensor_msgs/msg/imu.hpp>


using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * Implementation guidlines
 * Need an Odomcallback to get current measurements (from mavros)
 * Need target feedback to get reference points
 * Need a publisher for the se3 command
 * Need a publisher to mavros px4 setpoints topic
 * We should not publish commands if there is no odom
*/

class GeometricControlNode: public rclcpp::Node
{
public:
  GeometricControlNode();
  ~GeometricControlNode();

  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  // Need this since we have SO3Control which needs aligned pointer

private:
  // void corrections_callback(const kr_mav_msgs::Corrections::ConstPtr &msg); // Maybe we don't need it!! 
  // void cfg_callback(kr_mav_controllers::SO3Config &config, uint32_t level); // @todo Need to adapt to ros2
  
 
  /*
  @brief Publish SE3 command to the flight controller
  */
  void publishSE3Command();
  void holdWatchdog();
  /*
  @brief Publish a DiagnosticStatus summarising controller health for ground
  monitoring (stream ages/rates, hold failsafe, saturation, errors, gains).
  Observation only: nothing here feeds back into the control path.
  */
  void publishStatus();
  /*
  @brief Update an EWMA rate estimate [Hz] from successive event times.
  */
  static void noteRate(double & rate_hz, rclcpp::Time & prev, bool & have_prev,
                       const rclcpp::Time & now);
  /*
  @brief ROS callback to motor state
  @param msg mav_controllers_ros::msg::TargetCommand
  */
  void motorStateCallback(const std_msgs::msg::Bool & msg);

  /*
  @brief ROS callback to receive setpoints of the SE2 controller
  @param msg mav_controllers_ros::msg::TargetCommand
  */
  void targetCmdCallback(const mav_controllers_ros::msg::TargetCommand & msg);

  /*
  @brief Defines controller setpoints from trajectory_msgs::msg::MultiDOFJointTrajectory
  @param msg trajectory_msgs::msg::MultiDOFJointTrajectory
  */
  void multiDofTrajCallback(const trajectory_msgs::msg::MultiDOFJointTrajectory& msg);

  /*
  @brief Odometry ROS callback to receive linear and rotational measurements
  @param msg nav_msgs::msg::Odometry
  */
  void odomCallback(const nav_msgs::msg::Odometry & msg);

  rcl_interfaces::msg::SetParametersResult  param_callback(const std::vector<rclcpp::Parameter> & parameters);

  /*Publishers */
  rclcpp::Publisher<mav_controllers_ros::msg::SE3Command>::SharedPtr se3_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  odom_pose_pub_;  // For sending PoseStamped to firmware ??
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  command_viz_pub_; // cmd visulaization in RViz2
  rclcpp::Publisher<mav_controllers_ros::msg::ControlErrors>::SharedPtr  cont_err_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr  status_pub_;

  /* Subscribers */
  rclcpp::Subscription<mav_controllers_ros::msg::TargetCommand>::SharedPtr target_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_motor_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr multi_dof_traj_sub_;

  OnSetParametersCallbackHandle::SharedPtr callback_handler_;

  
  GeometricAttitudeControl controller_; /* Geometric Controller object */

  Eigen::Vector3f des_pos_, des_vel_, des_acc_, des_jrk_, config_kx_, config_kv_, config_ki_, config_kd_, kx_, kv_, kd_;
  float attctrl_tau_, config_attctrl_tau_;
  // Watchdogs: reject control on stale odometry; reset integrals after a
  // gap in the setpoint stream.
  double odom_timeout_{0.3};
  double setpoint_timeout_{1.0};
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_control_time_;
  bool have_control_time_{false};
  // Setpoint-loss hold failsafe: if the setpoint stream dies mid-flight
  // (planner node shut down or crashed), latch a closed-loop position hold
  // at the current pose and keep commanding, instead of going silent and
  // depending on the PX4 offboard-loss failsafe configuration.
  bool hold_on_setpoint_timeout_{true};
  bool in_hold_failsafe_{false};
  Eigen::Vector3f current_position_{Eigen::Vector3f::Zero()};
  rclcpp::TimerBase::SharedPtr hold_watchdog_timer_;
  // Ground-monitoring telemetry (observation only). Stream health is
  // otherwise invisible from outside the node, which makes a silent
  // degradation (stale odom, dead planner, saturated controller) look
  // identical to normal flight on the ground station.
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Time last_setpoint_time_;
  bool have_setpoint_time_{false};
  rclcpp::Time prev_odom_time_, prev_setpoint_time_, prev_control_time_;
  bool have_prev_odom_{false}, have_prev_setpoint_{false}, have_prev_control_{false};
  double odom_rate_hz_{0.0}, setpoint_rate_hz_{0.0}, control_rate_hz_{0.0};
  float control_dt_{0.0f};
  float des_yaw_, des_yaw_dot_;
  float current_yaw_;
  Eigen::Quaternionf current_orientation_;
  
  /* flags */
  bool   position_cmd_updated_, position_cmd_init_; 
  bool enable_motors_, use_external_yaw_, have_odom_;

  Eigen::Quaterniond odom_q_, imu_q_;
  double kf_, lin_cof_a_, lin_int_b_;
  int num_props_;  

  // double so3_cmd_timeout_;
  // ros::Time last_so3_cmd_time_;
  // kr_mav_msgs::SO3Command last_so3_cmd_; // Need to change to suitable ros 2 msg

  
  std::string frame_id_; /* of the command */

  
  
  /* drone-specific params */
  float mass_;
  const float g_;
  float max_acc_;

  float yaw_gain_;
};

//////////////// Class definitions ////////////////
GeometricControlNode::GeometricControlNode(): Node("geometric_control_node"),
        des_yaw_(0),
        des_yaw_dot_(0),
        current_yaw_(0),
        current_orientation_(Eigen::Quaternionf::Identity()),
        position_cmd_updated_(false),
        position_cmd_init_(false),
        enable_motors_(false),
        use_external_yaw_(false),
        have_odom_(false),
        g_(9.81),
        max_acc_(10.0)
{
  /* Get params */
  this->declare_parameter("mass", 0.5);
  mass_ = this->get_parameter("mass").get_parameter_value().get<float>();
  RCLCPP_INFO(this->get_logger(), "Mass = %0.2f Kg", mass_);
  controller_.setMass(mass_);
  controller_.setGravity(g_);

  this->declare_parameter("max_accel", 10.0);
  max_acc_ = this->get_parameter("max_accel").get_parameter_value().get<float>();
  controller_.setMaxAcceleration(max_acc_);

  this->declare_parameter("use_external_yaw", true);
  use_external_yaw_ = this->get_parameter("use_external_yaw").get_parameter_value().get<bool>();
  controller_.setVelocityYaw(!use_external_yaw_);

  this->declare_parameter("gains.pos.x", 7.4f);
  config_kx_[0] = this->get_parameter("gains.pos.x").get_parameter_value().get<float>();

  this->declare_parameter("gains.pos.y", 7.4f);
  config_kx_[1] = this->get_parameter("gains.pos.y").get_parameter_value().get<float>();

  this->declare_parameter("gains.pos.z", 10.4f);
  config_kx_[2] = this->get_parameter("gains.pos.z").get_parameter_value().get<float>();

  kx_[0] = config_kx_[0];
  kx_[1] = config_kx_[1];
  kx_[2] = config_kx_[2];

  this->declare_parameter("gains.vel.x", 4.8f);
  config_kv_[0] = this->get_parameter("gains.vel.x").get_parameter_value().get<float>();

  this->declare_parameter("gains.vel.y", 4.8f);
  config_kv_[1] = this->get_parameter("gains.vel.y").get_parameter_value().get<float>();

  this->declare_parameter("gains.vel.z", 6.0f);
  config_kv_[2] = this->get_parameter("gains.vel.z").get_parameter_value().get<float>();

  kv_[0] = config_kv_[0];
  kv_[1] = config_kv_[1];
  kv_[2] = config_kv_[2];

  this->declare_parameter("gains.ki.x", 0.0f);
  config_ki_[0] = this->get_parameter("gains.ki.x").get_parameter_value().get<float>();

  this->declare_parameter("gains.ki.y", 0.0f);
  config_ki_[1] = this->get_parameter("gains.ki.y").get_parameter_value().get<float>();

  this->declare_parameter("gains.ki.z", 0.0f);
  config_ki_[2] = this->get_parameter("gains.ki.z").get_parameter_value().get<float>();

  // gains.kib.* are deprecated: the body-frame integral was never
  // implemented (the accumulator was dead code). Accept the params so old
  // configs still load, but warn if someone tries to use them.
  this->declare_parameter("gains.kib.x", 0.0f);
  this->declare_parameter("gains.kib.y", 0.0f);
  this->declare_parameter("gains.kib.z", 0.0f);
  {
    const float kib_x = this->get_parameter("gains.kib.x").get_parameter_value().get<float>();
    const float kib_y = this->get_parameter("gains.kib.y").get_parameter_value().get<float>();
    const float kib_z = this->get_parameter("gains.kib.z").get_parameter_value().get<float>();
    if(kib_x != 0.0f || kib_y != 0.0f || kib_z != 0.0f)
      RCLCPP_WARN(this->get_logger(),
                  "gains.kib.* are deprecated and have NO effect (the body-frame "
                  "integral was never implemented). Use gains.ki.* instead.");
  }

  this->declare_parameter("drag.kd.x", 0.0f);
  config_kd_[0] = this->get_parameter("drag.kd.x").get_parameter_value().get<float>();

  this->declare_parameter("drag.kd.y", 0.0f);
  config_kd_[1] = this->get_parameter("drag.kd.y").get_parameter_value().get<float>();

  this->declare_parameter("drag.kd.z", 0.0f);
  config_kd_[2] = this->get_parameter("drag.kd.z").get_parameter_value().get<float>();
  
  this->declare_parameter("attctrl_tau", 0.3f);
  config_attctrl_tau_ = this->get_parameter("attctrl_tau").get_parameter_value().get<float>();

  float max_pos_int;
  this->declare_parameter("max_pos_int", 0.5f);
  max_pos_int = this->get_parameter("max_pos_int").get_parameter_value().get<float>();
  this->declare_parameter("mas_pos_int_b", 0.5f);  // deprecated, kept so old configs load

  controller_.setMaxIntegral(max_pos_int);

  this->declare_parameter("odom_timeout", 0.3);
  odom_timeout_ = this->get_parameter("odom_timeout").get_parameter_value().get<double>();

  this->declare_parameter("setpoint_timeout", 1.0);
  setpoint_timeout_ = this->get_parameter("setpoint_timeout").get_parameter_value().get<double>();

  this->declare_parameter("hold_on_setpoint_timeout", true);
  hold_on_setpoint_timeout_ =
      this->get_parameter("hold_on_setpoint_timeout").get_parameter_value().get<bool>();
  if(hold_on_setpoint_timeout_)
    hold_watchdog_timer_ =
        this->create_wall_timer(20ms, std::bind(&GeometricControlNode::holdWatchdog, this));

  this->declare_parameter("enable_rate_feedforward", true);
  controller_.setRateFeedforward(
      this->get_parameter("enable_rate_feedforward").get_parameter_value().get<bool>());

  // Yaw attitude time constant; 0 (default) keeps yaw on attctrl_tau.
  // Yaw authority is drag-torque only, so a slower value (e.g. 0.4-0.5)
  // is typical once tuned.
  this->declare_parameter("yawctrl_tau", 0.0f);
  controller_.setYawCtrlTau(
      this->get_parameter("yawctrl_tau").get_parameter_value().get<float>());

  float max_tilt_angle;
  this->declare_parameter("max_tilt_angle", static_cast<float>(M_PI));
  max_tilt_angle = this->get_parameter("max_tilt_angle").get_parameter_value().get<float>();
  controller_.setMaxTiltAngle(max_tilt_angle);

  this->declare_parameter("yaw_gain", 0.3);
  yaw_gain_ = this->get_parameter("yaw_gain").get_parameter_value().get<float>();
  controller_.setYawGain(yaw_gain_);


  controller_.resetIntegrals();

  // The active gains are normally refreshed by each incoming setpoint (which
  // may carry its own), but before the first setpoint they were left unset --
  // so a ground station reading them saw stale values, and attctrl_tau read
  // as 0. Start them at the configured values.
  kx_ = config_kx_;
  kv_ = config_kv_;
  kd_ = config_kd_;
  attctrl_tau_ = config_attctrl_tau_;

  /* Define subscribers and publishers */

  se3_command_pub_ = this->create_publisher<mav_controllers_ros::msg::SE3Command>("geometric_controller/cmd", 10);
  odom_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("geometric_controller/odom_pose", 10);
  command_viz_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("geometric_controller/cmd_pose", 10);
  cont_err_pub_ = this->create_publisher<mav_controllers_ros::msg::ControlErrors>("geometric_controller/control_errors", 10);

  // Health status for ground monitoring. Deliberately low rate: it is meant
  // to survive a field datalink, so panels never have to subscribe to the
  // 50-100 Hz command streams. status_rate <= 0 disables it entirely.
  this->declare_parameter("status_rate", 5.0);
  const double status_rate = this->get_parameter("status_rate").get_parameter_value().get<double>();
  if(status_rate > 0.0)
  {
    status_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
        "geometric_controller/status", 10);
    status_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / status_rate),
        std::bind(&GeometricControlNode::publishStatus, this));
  }

  target_cmd_sub_ = this->create_subscription<mav_controllers_ros::msg::TargetCommand>(
      "geometric_controller/setpoint", 10, std::bind(&GeometricControlNode::targetCmdCallback, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "geometric_controller/odom", rclcpp::SensorDataQoS(), std::bind(&GeometricControlNode::odomCallback, this, _1));

  enable_motor_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "geometric_controller/enable_motors", 10, std::bind(&GeometricControlNode::motorStateCallback, this, _1));

  multi_dof_traj_sub_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      "geometric_controller/multi_dof_setpoint", 10, std::bind(&GeometricControlNode::multiDofTrajCallback, this, _1));

  callback_handler_ = this->add_on_set_parameters_callback(std::bind(&GeometricControlNode::param_callback, this, std::placeholders::_1));


}

GeometricControlNode::~GeometricControlNode()
{
  /* Destructor */
}

void GeometricControlNode::motorStateCallback(const std_msgs::msg::Bool & msg)
{
  if (msg.data)
    enable_motors_ = true;
  else
  {
    enable_motors_ = false;
    in_hold_failsafe_ = false;  // a stale hold pose must not survive re-engage
  }
}

// Runs at 50 Hz when hold_on_setpoint_timeout is enabled. If we were flying
// setpoints and the stream stops while odometry is healthy and motors are
// enabled, latch a position hold at the current pose and keep commanding.
// A new setpoint releases the hold (see targetCmdCallback).
void GeometricControlNode::holdWatchdog()
{
  if(!have_control_time_ || !have_odom_ || !enable_motors_)
    return;

  const rclcpp::Time now = this->now();
  if((now - last_odom_time_).seconds() > odom_timeout_)
  {
    // Can't hold without state; publishSE3Command's odom watchdog goes
    // silent and PX4's failsafe takes over. Drop the hold so that if odom
    // returns we re-latch at the vehicle's NEW position, not a stale one.
    in_hold_failsafe_ = false;
    return;
  }

  if(!in_hold_failsafe_)
  {
    if((now - last_control_time_).seconds() <= setpoint_timeout_)
      return;
    in_hold_failsafe_ = true;
    des_pos_ = current_position_;
    des_vel_.setZero();
    des_acc_.setZero();
    des_jrk_.setZero();
    des_yaw_ = current_yaw_;
    des_yaw_dot_ = 0.0f;
    kx_ = config_kx_;
    kv_ = config_kv_;
    kd_ = config_kd_;
    attctrl_tau_ = config_attctrl_tau_;
    RCLCPP_ERROR(this->get_logger(),
                 "[holdWatchdog]: setpoint stream lost. HOLDING position at "
                 "(%.2f, %.2f, %.2f), yaw %.2f. New setpoints release the hold.",
                 des_pos_(0), des_pos_(1), des_pos_(2), des_yaw_);
  }
  publishSE3Command();
}


void
GeometricControlNode::odomCallback(const nav_msgs::msg::Odometry & msg)
{
  frame_id_ = msg.header.frame_id;

  const Eigen::Vector3f position(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
  const Eigen::Vector3f velocity(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);

  tf2::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    );
  
  double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  current_yaw_ = yaw;

  current_orientation_ = Eigen::Quaternionf(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);

  current_position_ = position;
  controller_.setPosition(position);
  controller_.setVelocity(velocity);
  controller_.setCurrentOrientation(current_orientation_);

  const rclcpp::Time odom_now = this->now();
  noteRate(odom_rate_hz_, prev_odom_time_, have_prev_odom_, odom_now);
  last_odom_time_ = odom_now;
  have_odom_ = true;

  // if(position_cmd_init_)
  // {
  //   // We set position_cmd_updated_ = false and expect that the
  //   // position_cmd_callback would set it to true since typically a position_cmd
  //   // message would follow an odom message. If not, the position_cmd_callback
  //   // hasn't been called and we publish the so3 command ourselves
  //   // TODO: Fallback to hover if position_cmd hasn't been received for some time
  //   if(!position_cmd_updated_)
  //     publishSO3Command();
  //   position_cmd_updated_ = false;
  // }
}


void
GeometricControlNode::targetCmdCallback(const mav_controllers_ros::msg::TargetCommand & msg)
{
  {
    const rclcpp::Time sp_now = this->now();
    noteRate(setpoint_rate_hz_, prev_setpoint_time_, have_prev_setpoint_, sp_now);
    last_setpoint_time_ = sp_now;
    have_setpoint_time_ = true;
  }
  if(in_hold_failsafe_)
  {
    in_hold_failsafe_ = false;
    RCLCPP_WARN(this->get_logger(),
                "[targetCmdCallback]: setpoint stream resumed; releasing hold.");
  }
  des_pos_ = Eigen::Vector3f(msg.position.x, msg.position.y, msg.position.z);
  des_vel_ = Eigen::Vector3f(msg.velocity.x, msg.velocity.y, msg.velocity.z);
  des_acc_ = Eigen::Vector3f(msg.acceleration.x, msg.acceleration.y, msg.acceleration.z);
  des_jrk_ = Eigen::Vector3f(msg.jerk.x, msg.jerk.y, msg.jerk.z);

  // Check use_msg_gains_flag to decide whether to use gains from the msg or config
  kx_[0] = (msg.use_msg_gains_flags & msg.USE_MSG_GAINS_POSITION_X) ? msg.kx[0] : config_kx_[0];
  kx_[1] = (msg.use_msg_gains_flags & msg.USE_MSG_GAINS_POSITION_Y) ? msg.kx[1] : config_kx_[1];
  kx_[2] = (msg.use_msg_gains_flags & msg.USE_MSG_GAINS_POSITION_Z) ? msg.kx[2] : config_kx_[2];
  kv_[0] = (msg.use_msg_gains_flags & msg.USE_MSG_GAINS_VELOCITY_X) ? msg.kv[0] : config_kv_[0];
  kv_[1] = (msg.use_msg_gains_flags & msg.USE_MSG_GAINS_VELOCITY_Y) ? msg.kv[1] : config_kv_[1];
  kv_[2] = (msg.use_msg_gains_flags & msg.USE_MSG_GAINS_VELOCITY_Z) ? msg.kv[2] : config_kv_[2];

  kd_[0] = config_kd_[0];
  kd_[1] = config_kd_[1];
  kd_[2] = config_kd_[2];

  attctrl_tau_ = config_attctrl_tau_;

  des_yaw_ = msg.yaw;
  des_yaw_dot_ = msg.yaw_dot;
  position_cmd_updated_ = true;
  // position_cmd_init_ = true;

  publishSE3Command();
}

void
GeometricControlNode::multiDofTrajCallback(const trajectory_msgs::msg::MultiDOFJointTrajectory& msg)
{
  {
    const rclcpp::Time sp_now = this->now();
    noteRate(setpoint_rate_hz_, prev_setpoint_time_, have_prev_setpoint_, sp_now);
    last_setpoint_time_ = sp_now;
    have_setpoint_time_ = true;
  }
  if(in_hold_failsafe_)
  {
    in_hold_failsafe_ = false;
    RCLCPP_WARN(this->get_logger(),
                "[multiDofTrajCallback]: setpoint stream resumed; releasing hold.");
  }
  des_pos_ = Eigen::Vector3f(msg.points[0].transforms[0].translation.x,
                              msg.points[0].transforms[0].translation.y,
                              msg.points[0].transforms[0].translation.z);
  des_vel_ = Eigen::Vector3f(msg.points[0].velocities[0].linear.x,
                              msg.points[0].velocities[0].linear.y,
                              msg.points[0].velocities[0].linear.z);

  des_acc_ = Eigen::Vector3f(msg.points[0].accelerations[0].linear.x,
                              msg.points[0].accelerations[0].linear.y,
                              msg.points[0].accelerations[0].linear.z);

  des_jrk_ = Eigen::Vector3f(0.0, 0.0, 0.0);

  des_yaw_dot_ = msg.points[0].velocities[0].angular.z;

  // Yaw straight from the quaternion (eulerAngles(0,1,2) can return the
  // flipped representation (pi, pi, yaw-pi) and corrupt the reference)
  const auto &rot = msg.points[0].transforms[0].rotation;
  des_yaw_ = std::atan2(2.0f * static_cast<float>(rot.w * rot.z + rot.x * rot.y),
                        1.0f - 2.0f * static_cast<float>(rot.y * rot.y + rot.z * rot.z));

  // Check use_msg_gains_flag to decide whether to use gains from the msg or config
  kx_[0] = config_kx_[0];
  kx_[1] = config_kx_[1];
  kx_[2] = config_kx_[2];
  kv_[0] = config_kv_[0];
  kv_[1] = config_kv_[1];
  kv_[2] = config_kv_[2];

  kd_[0] = config_kd_[0];
  kd_[1] = config_kd_[1];
  kd_[2] = config_kd_[2];

  attctrl_tau_ = config_attctrl_tau_;

  position_cmd_updated_ = true;

  publishSE3Command();

  }


void
GeometricControlNode::publishSE3Command()
{
  if(!have_odom_)
  {
    RCLCPP_WARN(this->get_logger(), "[publishSE3Command]: No odometry! Not publishing SE3Command.");
    return;
  }

  const rclcpp::Time now = this->now();

  // Never control on stale state: if odometry stopped, stop commanding.
  // The downstream mavros/PX4 offboard-loss failsafe then takes over.
  const double odom_age = (now - last_odom_time_).seconds();
  if(odom_age > odom_timeout_)
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "[publishSE3Command]: odometry is stale (%.2f s > %.2f s). "
                          "Not publishing SE3Command.", odom_age, odom_timeout_);
    controller_.resetIntegrals();
    have_control_time_ = false;
    return;
  }

  // dt for the position integrator, from the actual control invocation
  // times. After a gap in the setpoint stream the integrals are reset and
  // integration restarts cleanly (dt = 0 on the first cycle back).
  float dt = 0.0f;
  if(have_control_time_)
  {
    const double gap = (now - last_control_time_).seconds();
    if(gap > setpoint_timeout_)
    {
      RCLCPP_WARN(this->get_logger(),
                  "[publishSE3Command]: %.2f s gap in setpoint stream; resetting integrals.", gap);
      controller_.resetIntegrals();
    }
    else
    {
      dt = static_cast<float>(gap);
    }
  }
  noteRate(control_rate_hz_, prev_control_time_, have_prev_control_, now);
  control_dt_ = dt;
  last_control_time_ = now;
  have_control_time_ = true;

  Eigen::Vector3f ki = Eigen::Vector3f::Zero();
  if(enable_motors_)
    ki = config_ki_;
  else
    controller_.resetIntegrals();  // never accumulate while disarmed

  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_jrk_, des_yaw_, des_yaw_dot_, kx_, kv_, ki, kd_, attctrl_tau_, dt);

  const Eigen::Vector3f &force = controller_.getComputedForce();
  const Eigen::Quaternionf &orientation = controller_.getComputedOrientation();
  const Eigen::Vector3f &ang_vel = controller_.getComputedAngularVelocity();

  mav_controllers_ros::msg::ControlErrors err_msg;
  err_msg.header.stamp = this->now();
  err_msg.header.frame_id = frame_id_;
  err_msg.pos_error.x = controller_.getPosError()[0];
  err_msg.pos_error.y = controller_.getPosError()[1];
  err_msg.pos_error.z = controller_.getPosError()[2];
  err_msg.vel_error.x = controller_.getVelError()[0];
  err_msg.vel_error.y = controller_.getVelError()[1];
  err_msg.vel_error.z = controller_.getVelError()[2];
  err_msg.att_error.x = controller_.getAttitudeError()[0];
  err_msg.att_error.y = controller_.getAttitudeError()[1];
  err_msg.att_error.z = controller_.getAttitudeError()[2];
  cont_err_pub_->publish(err_msg);

  // kr_mav_msgs::SO3Command::Ptr so3_command = boost::make_shared<kr_mav_msgs::SO3Command>();
  mav_controllers_ros::msg::SE3Command cmd_msg;
  cmd_msg.header.stamp = this->now();
  cmd_msg.header.frame_id = frame_id_;
  cmd_msg.force.x = force(0);
  cmd_msg.force.y = force(1);
  cmd_msg.force.z = force(2);
  cmd_msg.orientation.x = orientation.x();
  cmd_msg.orientation.y = orientation.y();
  cmd_msg.orientation.z = orientation.z();
  cmd_msg.orientation.w = orientation.w();
  cmd_msg.angular_velocity.x = ang_vel(0);
  cmd_msg.angular_velocity.y = ang_vel(1);
  cmd_msg.angular_velocity.z = ang_vel(2);

  // so3_command->aux.current_yaw = current_yaw_;
  // so3_command->aux.kf_correction = corrections_[0];
  // so3_command->aux.angle_corrections[0] = corrections_[1];
  // so3_command->aux.angle_corrections[1] = corrections_[2];
  // so3_command->aux.enable_motors = enable_motors_;
  // so3_command->aux.use_external_yaw = use_external_yaw_;
  se3_command_pub_->publish(cmd_msg);

  geometry_msgs::msg::PoseStamped cmd_viz_msg;


  cmd_viz_msg.header = cmd_msg.header;
  cmd_viz_msg.pose.position.x = des_pos_(0);
  cmd_viz_msg.pose.position.y = des_pos_(1);
  cmd_viz_msg.pose.position.z = des_pos_(2);
  cmd_viz_msg.pose.orientation.x = orientation.x();
  cmd_viz_msg.pose.orientation.y = orientation.y();
  cmd_viz_msg.pose.orientation.z = orientation.z();
  cmd_viz_msg.pose.orientation.w = orientation.w();
  command_viz_pub_->publish(cmd_viz_msg);
}

// EWMA of the instantaneous rate between successive events. A gap longer
// than 2 s restarts the estimate rather than averaging across the outage,
// so a stream that stops and resumes reports its true rate immediately.
void
GeometricControlNode::noteRate(double & rate_hz, rclcpp::Time & prev, bool & have_prev,
                               const rclcpp::Time & now)
{
  if(have_prev)
  {
    const double gap = (now - prev).seconds();
    if(gap > 2.0 || gap <= 0.0)
    {
      rate_hz = 0.0;
    }
    else
    {
      const double inst = 1.0 / gap;
      rate_hz = (rate_hz > 0.0) ? (0.8 * rate_hz + 0.2 * inst) : inst;
    }
  }
  prev = now;
  have_prev = true;
}

// Health snapshot for the ground station. Everything reported here is state
// the node already maintains; nothing is computed for the control path and
// nothing here can influence it.
void
GeometricControlNode::publishStatus()
{
  if(!status_pub_)
    return;

  const rclcpp::Time now = this->now();
  const double odom_age = have_odom_ ? (now - last_odom_time_).seconds()
                                     : std::numeric_limits<double>::infinity();
  const double setpoint_age = have_setpoint_time_ ? (now - last_setpoint_time_).seconds()
                                                  : std::numeric_limits<double>::infinity();
  const bool odom_stale = !have_odom_ || odom_age > odom_timeout_;
  const bool setpoint_stale = !have_setpoint_time_ || setpoint_age > setpoint_timeout_;
  const bool saturated = controller_.isSaturated();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "geometric_controller";
  st.hardware_id = this->get_name();

  if(odom_stale)
  {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    st.message = have_odom_ ? "odometry stale - not commanding" : "no odometry";
  }
  else if(in_hold_failsafe_)
  {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = "HOLD failsafe: setpoint stream lost";
  }
  else if(enable_motors_ && setpoint_stale)
  {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = "no setpoints";
  }
  else if(saturated)
  {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = "command saturated (accel/tilt limit)";
  }
  else if(!enable_motors_)
  {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = "idle (motors disabled)";
  }
  else
  {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = "tracking";
  }

  auto add = [&st](const char * key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    st.values.push_back(kv);
  };
  auto num = [](double v, int prec = 3) {
    if(!std::isfinite(v))
      return std::string("inf");
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.*f", prec, v);
    return std::string(buf);
  };
  auto bl = [](bool v) { return std::string(v ? "true" : "false"); };

  // A rate estimate only updates when an event arrives, so a stream that
  // has died keeps reporting its last healthy rate. Report zero once the
  // stream is stale, so a panel cannot show "50 Hz" for a dead publisher.
  const double control_age = have_control_time_ ? (now - last_control_time_).seconds()
                                                : std::numeric_limits<double>::infinity();
  auto live = [](double rate, double age) { return (age > 2.0) ? 0.0 : rate; };

  // Stream health
  add("have_odom", bl(have_odom_));
  add("odom_age_s", num(odom_age));
  add("odom_rate_hz", num(live(odom_rate_hz_, odom_age), 1));
  add("odom_timeout_s", num(odom_timeout_));
  add("setpoint_age_s", num(setpoint_age));
  add("setpoint_rate_hz", num(live(setpoint_rate_hz_, setpoint_age), 1));
  add("setpoint_timeout_s", num(setpoint_timeout_));
  add("control_rate_hz", num(live(control_rate_hz_, control_age), 1));
  add("control_dt_s", num(control_dt_, 4));

  // Mode / failsafe
  add("motors_enabled", bl(enable_motors_));
  add("hold_active", bl(in_hold_failsafe_));
  add("hold_enabled", bl(hold_on_setpoint_timeout_));
  add("saturated", bl(saturated));

  // Tracking errors (norms plus per-axis position error, which is what a
  // pilot reads to judge a gain change)
  const Eigen::Vector3f pos_err = controller_.getPosError();
  const Eigen::Vector3f vel_err = controller_.getVelError();
  const Eigen::Vector3f att_err = controller_.getAttitudeError();
  add("pos_err_norm_m", num(pos_err.norm()));
  add("pos_err_x_m", num(pos_err(0)));
  add("pos_err_y_m", num(pos_err(1)));
  add("pos_err_z_m", num(pos_err(2)));
  add("vel_err_norm_mps", num(vel_err.norm()));
  add("att_err_norm", num(att_err.norm()));

  // Attitude: actual tilt from odometry, commanded tilt from the last solve
  const Eigen::Matrix3f R = current_orientation_.toRotationMatrix();
  const float tilt = std::acos(std::max(-1.0f, std::min(1.0f, R(2, 2))));
  const Eigen::Matrix3f Rc = controller_.getComputedOrientation().toRotationMatrix();
  const float tilt_cmd = std::acos(std::max(-1.0f, std::min(1.0f, Rc(2, 2))));
  add("tilt_deg", num(tilt * 180.0 / M_PI, 2));
  add("tilt_cmd_deg", num(tilt_cmd * 180.0 / M_PI, 2));
  add("yaw_deg", num(current_yaw_ * 180.0 / M_PI, 2));
  add("yaw_err_deg", num(att_err(2) * 180.0 / M_PI, 2));

  // Integrator state (windup is invisible otherwise)
  const Eigen::Vector3f pos_int = controller_.getPosIntegral();
  add("pos_int_x", num(pos_int(0)));
  add("pos_int_y", num(pos_int(1)));
  add("pos_int_z", num(pos_int(2)));

  // Commanded force -> what the mavros node will turn into throttle
  const Eigen::Vector3f force = controller_.getComputedForce();
  add("cmd_force_norm_n", num(force.norm(), 2));
  add("mass_kg", num(mass_, 3));

  // ACTIVE gains: these can differ from the node parameters when a
  // TargetCommand carries per-message gains, so a gain panel that only
  // reads parameters would show the wrong numbers.
  add("kx", num(kx_(0), 2) + "," + num(kx_(1), 2) + "," + num(kx_(2), 2));
  add("kv", num(kv_(0), 2) + "," + num(kv_(1), 2) + "," + num(kv_(2), 2));
  add("ki", num(config_ki_(0), 3) + "," + num(config_ki_(1), 3) + "," + num(config_ki_(2), 3));
  add("attctrl_tau", num(attctrl_tau_, 3));
  // The configured values, for comparison: they differ from the active ones
  // only while a setpoint stream is supplying per-message gains.
  add("kx_config", num(config_kx_(0), 2) + "," + num(config_kx_(1), 2) + "," + num(config_kx_(2), 2));
  add("kv_config", num(config_kv_(0), 2) + "," + num(config_kv_(1), 2) + "," + num(config_kv_(2), 2));
  add("attctrl_tau_config", num(config_attctrl_tau_, 3));

  // Setpoint being tracked
  add("des_pos", num(des_pos_(0), 2) + "," + num(des_pos_(1), 2) + "," + num(des_pos_(2), 2));
  add("des_yaw_deg", num(des_yaw_ * 180.0 / M_PI, 1));

  status_pub_->publish(st);
}

rcl_interfaces::msg::SetParametersResult  GeometricControlNode::param_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;
  // Validate: all gain-like parameters must be non-negative finite numbers.
  for (const auto & parameter : parameters)
  {
    const auto & name = parameter.get_name();
    const bool gain_like = name.rfind("gains.", 0) == 0 || name.rfind("drag.", 0) == 0 ||
                           name == "attctrl_tau" || name == "max_accel" ||
                           name == "max_tilt_angle" || name == "mass" ||
                           name == "max_pos_int" || name == "yaw_gain" ||
                           name == "yawctrl_tau";
    if(gain_like && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      const double v = parameter.as_double();
      if(!std::isfinite(v) || v < 0.0 ||
         ((name == "attctrl_tau" || name == "mass") && v <= 0.0))
      {
        result.successful = false;
        result.reason = name + " must be a non-negative finite number";
        RCLCPP_ERROR(this->get_logger(), "Rejected parameter %s = %f", name.c_str(), v);
        return result;
      }
    }
  }
  for (auto parameter : parameters)
  {
    if(parameter.get_name() == "gains.pos.x")
    {
      config_kx_[0] = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "Got gains.pos.x  = %0.2f", config_kx_[0]);
    }
    if(parameter.get_name() == "gains.pos.y")
    {
      config_kx_[1] = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "Got gains.pos.y  = %0.2f", config_kx_[1]);
    }
    if(parameter.get_name() == "gains.pos.z")
    {
      config_kx_[2] = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "Got gains.pos.z  = %0.2f", config_kx_[2]);
    }
    if(parameter.get_name() == "gains.vel.x")
    {
      config_kv_[0] = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "Got gains.vel.x  = %0.2f", config_kv_[0]);
    }
    if(parameter.get_name() == "gains.vel.y")
    {
      config_kv_[1] = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "Got gains.vel.y  = %0.2f", config_kv_[1]);
    }
    if(parameter.get_name() == "gains.vel.z")
    {
      config_kv_[2] = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "Got gains.vel.z  = %0.2f", config_kv_[2]);
    }
    if(parameter.get_name() == "attctrl_tau")
    {
      config_attctrl_tau_ = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "attctrl_tau  = %0.2f", config_attctrl_tau_);
    }
    if(parameter.get_name() == "max_accel")
    {
      max_acc_ = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "max_accel  = %0.2f", max_acc_);
      controller_.setMaxAcceleration(max_acc_);
    }
    if(parameter.get_name() == "max_tilt_angle")
    {
      float max_tilt_ang = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "max_tilt_angle  = %0.2f", max_tilt_ang);
      controller_.setMaxTiltAngle(max_tilt_ang);
    }
    if(parameter.get_name() == "mass")
    {
      mass_ = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "mass  = %0.2f", mass_);
      controller_.setMass(mass_);
    }
    if(parameter.get_name() == "use_external_yaw")
    {
      use_external_yaw_ = parameter.as_bool();
      RCLCPP_INFO(this->get_logger(), "use_external_yaw_  = %d", use_external_yaw_);
      controller_.setVelocityYaw(!use_external_yaw_);
    }
    if(parameter.get_name() == "yaw_gain")
    {
      yaw_gain_ = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "yaw_gain_  = %0.2f", yaw_gain_);
      controller_.setYawGain(yaw_gain_);
    }
    if(parameter.get_name() == "gains.ki.x")
    {
      config_ki_[0] = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "Got gains.ki.x  = %0.3f", config_ki_[0]);
    }
    if(parameter.get_name() == "gains.ki.y")
    {
      config_ki_[1] = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "Got gains.ki.y  = %0.3f", config_ki_[1]);
    }
    if(parameter.get_name() == "gains.ki.z")
    {
      config_ki_[2] = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "Got gains.ki.z  = %0.3f", config_ki_[2]);
    }
    if(parameter.get_name() == "drag.kd.x")
    {
      config_kd_[0] = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "Got drag.kd.x  = %0.3f", config_kd_[0]);
    }
    if(parameter.get_name() == "drag.kd.y")
    {
      config_kd_[1] = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "Got drag.kd.y  = %0.3f", config_kd_[1]);
    }
    if(parameter.get_name() == "drag.kd.z")
    {
      config_kd_[2] = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "Got drag.kd.z  = %0.3f", config_kd_[2]);
    }
    if(parameter.get_name() == "max_pos_int")
    {
      const float v = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "max_pos_int  = %0.2f", v);
      controller_.setMaxIntegral(v);
    }
    if(parameter.get_name() == "enable_rate_feedforward")
    {
      const bool en = parameter.as_bool();
      RCLCPP_INFO(this->get_logger(), "enable_rate_feedforward = %d", en);
      controller_.setRateFeedforward(en);
    }
    if(parameter.get_name() == "yawctrl_tau")
    {
      const float v = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "yawctrl_tau  = %0.3f", v);
      controller_.setYawCtrlTau(v);
    }
  }

  // Mirror the new configuration into the active gains as well. Without
  // this a parameter change is invisible -- to the controller AND to a
  // ground station reading the status topic -- until the next setpoint
  // arrives, which on the ground may be never. A setpoint carrying its own
  // gains still wins on the very next message, as before.
  kx_ = config_kx_;
  kv_ = config_kv_;
  kd_ = config_kd_;
  attctrl_tau_ = config_attctrl_tau_;

  return result;
}

/**
 * Main function
*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeometricControlNode>());
  rclcpp::shutdown();
  return 0;
}