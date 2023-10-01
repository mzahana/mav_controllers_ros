#include <mav_controllers_ros/GeometricAttitudeControl.h>
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
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>


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

  // void imu_callback(const sensor_msgs::Imu::ConstPtr &pose); /* No need ?!*/

  rcl_interfaces::msg::SetParametersResult  param_callback(const std::vector<rclcpp::Parameter> & parameters);

  /*Publishers */
  rclcpp::Publisher<mav_controllers_ros::msg::SE3Command>::SharedPtr se3_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  odom_pose_pub_;  // For sending PoseStamped to firmware ??
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  command_viz_pub_; // cmd visulaization in RViz2
  rclcpp::Publisher<mav_controllers_ros::msg::ControlErrors>::SharedPtr  cont_err_pub_;

  /* Subscribers */
  rclcpp::Subscription<mav_controllers_ros::msg::TargetCommand>::SharedPtr target_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_motor_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr multi_dof_traj_sub_;

  OnSetParametersCallbackHandle::SharedPtr callback_handler_;

  
  GeometricAttitudeControl controller_; /* Geometric Controller object */

  Eigen::Vector3f des_pos_, des_vel_, des_acc_, des_jrk_, config_kx_, config_kv_, config_ki_, config_kib_, config_kd_, kx_, kv_, kd_;
  float attctrl_tau_, config_attctrl_tau_;
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

  this->declare_parameter("gains.kib.x", 0.0f);
  config_kib_[0] = this->get_parameter("gains.kib.x").get_parameter_value().get<float>();

  this->declare_parameter("gains.kib.y", 0.0f);
  config_kib_[1] = this->get_parameter("gains.kib.y").get_parameter_value().get<float>();
  
  this->declare_parameter("gains.kib.z", 0.0f);
  config_kib_[2] = this->get_parameter("gains.kib.z").get_parameter_value().get<float>();

  this->declare_parameter("drag.kd.x", 0.0f);
  config_kd_[0] = this->get_parameter("drag.kd.x").get_parameter_value().get<float>();

  this->declare_parameter("drag.kd.y", 0.0f);
  config_kd_[1] = this->get_parameter("drag.kd.y").get_parameter_value().get<float>();

  this->declare_parameter("drag.kd.z", 0.0f);
  config_kd_[2] = this->get_parameter("drag.kd.z").get_parameter_value().get<float>();
  
  this->declare_parameter("attctrl_tau", 0.3f);
  config_attctrl_tau_ = this->get_parameter("attctrl_tau").get_parameter_value().get<float>();

  float max_pos_int, max_pos_int_b;
  this->declare_parameter("max_pos_int", 0.5f);
  max_pos_int = this->get_parameter("max_pos_int").get_parameter_value().get<float>();
  this->declare_parameter("mas_pos_int_b", 0.5f);
  max_pos_int_b = this->get_parameter("mas_pos_int_b").get_parameter_value().get<float>();

  controller_.setMaxIntegral(max_pos_int);
  controller_.setMaxIntegralBody(max_pos_int_b);

  float max_tilt_angle;
  this->declare_parameter("max_tilt_angle", static_cast<float>(M_PI));
  max_tilt_angle = this->get_parameter("max_tilt_angle").get_parameter_value().get<float>();
  controller_.setMaxTiltAngle(max_tilt_angle);


  controller_.resetIntegrals();

  /* Define subscribers and publishers */

  se3_command_pub_ = this->create_publisher<mav_controllers_ros::msg::SE3Command>("geometric_controller/cmd", 10);
  odom_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("geometric_controller/odom_pose", 10);
  command_viz_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("geometric_controller/cmd_pose", 10);
  cont_err_pub_ = this->create_publisher<mav_controllers_ros::msg::ControlErrors>("geometric_controller/control_errors", 10);

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
    enable_motors_ = false;
}

void
GeometricControlNode::odomCallback(const nav_msgs::msg::Odometry & msg)
{
  have_odom_ = true;

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

  controller_.setPosition(position);
  controller_.setVelocity(velocity);
  controller_.setCurrentOrientation(current_orientation_);

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

  tf2::Quaternion quat;
  tf2::Matrix3x3 m(quat);
  // tf2::convert(msg.points[0].transforms[0].rotation, quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  des_yaw_ = yaw;

  position_cmd_updated_ = true;

  publishSE3Command();

  }


void
GeometricControlNode::publishSE3Command()
{
  if(!have_odom_)
  {
    RCLCPP_WARN(this->get_logger(), "[publishSO3Command::publishSO3Command]: No odometry! Not publishing SO3Command.");
    return;
  }

  Eigen::Vector3f ki = Eigen::Vector3f::Zero();
  Eigen::Vector3f kib = Eigen::Vector3f::Zero();
  if(enable_motors_)
  {
    ki = config_ki_; // @todo implement dynamic config
    kib = config_kib_;
  }
  else
  {
    ki = Eigen::Vector3f::Zero();
    kib = Eigen::Vector3f::Zero();
  }

  // std::cout << "kx_:" << std::endl << kx_;
  // std::cout << "kv_: " << std::endl << kv_;
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_jrk_, des_yaw_, des_yaw_dot_, kx_, kv_, ki, kib, kd_, attctrl_tau_);

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

rcl_interfaces::msg::SetParametersResult  GeometricControlNode::param_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;
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
      float max_tolt_ang = static_cast<float>(parameter.as_double());
      RCLCPP_INFO(this->get_logger(), "max_tilt_angle  = %0.2f", max_tolt_ang);
      controller_.setMaxTiltAngle(max_tolt_ang);
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
      RCLCPP_INFO(this->get_logger(), "mass  = %d", use_external_yaw_);
      controller_.setVelocityYaw(!use_external_yaw_);
    }
    
    
    
  }

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