#include <px4_geometric_controller/SE3Controller.h>
#include "px4_geometric_controller/msg/se3_command.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include "mavros_msgs/msg/attitude_target.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/LinearMath/Quaternion.h"
#include <sensor_msgs/msg/imu.hpp>
#include "mavros_msgs/msg/state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SE3ControllerToMavros: public rclcpp::Node
{

public:
  SE3ControllerToMavros();
  ~SE3ControllerToMavros();

private:
  /*
  @brief ROS callback to receive SE3 commands and publish it as a mavros setpoint
  @param msg px4_geometric_controller::msg::SE3Command
  */
  void se3CmdCallback(const px4_geometric_controller::msg::SE3Command & msg);

  /*
  @brief ROS callback to receive odometry
  @param msg nav_msgs::msg::Odometry
  */
  void odomCallback(const nav_msgs::msg::Odometry & msg);

  /*
  @brief ROS callback to receive IMU measurements
  @param msg sensor_msgs::msg::Imu
  */
  void imuCallback(const sensor_msgs::msg::Imu & msg);

  /*
  @brief Mavros state msg to get arming state.
  @param msg mavros_msgs::msg::State
  */
 void mavrosStateCallback(const mavros_msgs::msg::State & msg);

  /*
  @brief Publish SE3 command to mavros topic.
  */
  void publishMavrosSetpoint();

  rclcpp::Subscription<px4_geometric_controller::msg::SE3Command>::SharedPtr se3_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_raw_pub_; // publisher for mavros setpoints

  bool odom_set_, imu_set_, so3_cmd_set_;
  Eigen::Quaterniond odom_q_, imu_q_;
  double kf_, lin_cof_a_, lin_int_b_; // need to check if they are needed
  int num_props_;

  double se3_cmd_timeout_;
  rclcpp::Time last_se3_cmd_time_;
  px4_geometric_controller::msg::SE3Command last_se3_cmd_;

  rclcpp::Clock::SharedPtr clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  
  // from MAVROS state msg
  bool motors_armed_;


};

/////////////////////////////////////////////////////
// Class definition //
/////////////////////////////////////////////////////
SE3ControllerToMavros::SE3ControllerToMavros(): Node("se3controller_to_mavros_node")
{
  // num_props
  if (!this->has_parameter("num_props"))
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'num_props' was not set!");
    // You can also throw an exception or set a default value here
    // throw std::runtime_error("Parameter 'my_param' was not set!");
  }
  else
  {
    // Get the parameter value if it exists
    this->get_parameter("num_props", num_props_);
    RCLCPP_INFO(this->get_logger(), "Received parameter num_props: %d", num_props_);
  }
  if(num_props_ <= 0)
  {
    RCLCPP_ERROR(this->get_logger(), "num_props must be > 0");
    return;
  }

  // kf
  if (!this->has_parameter("kf"))
  {
    RCLCPP_ERROR(this->get_logger(), "Must set kf param for thrust scaling. Motor speed = sqrt(thrust / num_props / kf)");
  }
  else
  {
    // Get the parameter value if it exists
    this->get_parameter("kf", kf_);
    RCLCPP_INFO(this->get_logger(), "Using kf=%g so that prop speed = sqrt(f / num_props / kf) to scale force to speed.", kf_);
  }
  if(kf_ <= 0)
  {
    RCLCPP_ERROR(this->get_logger(), "kf must be > 0");
    return;
  }

  // lin_cof_a AND lin_int_b
  if (this->has_parameter("lin_cof_a") && this->has_parameter("lin_int_b"))
  {
    RCLCPP_INFO(this->get_logger(), "Using %g*x + %g to scale prop speed to att_throttle.", lin_cof_a_, lin_int_b_);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Must set coefficients for thrust scaling (scaling from rotor "
              "velocity (rad/s) to att_throttle for pixhawk)");
    
      
  }



  // get param for so3 command timeout duration
  this->declare_parameter("se3_cmd_timeout", 0.25);
  se3_cmd_timeout_ = this->get_parameter("se3_cmd_timeout").get_parameter_value().get<double>();

  odom_set_ = false;
  imu_set_ = false;
  so3_cmd_set_ = false;
  
  motors_armed_ = false;

    
  attitude_raw_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("mavros/attitude_target", 10);

  se3_cmd_sub_ = this->create_subscription<px4_geometric_controller::msg::SE3Command>(
    "se3controller/cmd", 10, std::bind(&SE3ControllerToMavros::se3CmdCallback, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&SE3ControllerToMavros::odomCallback, this, _1));
  
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10, std::bind(&SE3ControllerToMavros::imuCallback, this, _1));
}

SE3ControllerToMavros::~SE3ControllerToMavros()
{
  /* Destructor */
}

void
SE3ControllerToMavros::mavrosStateCallback(const mavros_msgs::msg::State & msg)
{
  motors_armed_ = msg.armed;
}

void
SE3ControllerToMavros::se3CmdCallback(const px4_geometric_controller::msg::SE3Command & msg)
{

  // both imu_q_ and odom_q_ would be uninitialized if not set
  std::string topic_name;
  if(!imu_set_)
  {
    topic_name = imu_sub_->get_topic_name();
    RCLCPP_WARN(this->get_logger(), "Did not receive any imu messages from %s", topic_name.c_str());
    return;
  }

  if(!odom_set_)
  {
    // ROS_WARN("Did not receive any odom messages from %s", odom_sub_.getTopic().c_str());
    topic_name = odom_sub_->get_topic_name();
    RCLCPP_WARN(this->get_logger(), "Did not receive any odom messages from %s", topic_name.c_str());
    return;
  }

  // transform to take into consideration the different yaw of the flight
  // controller imu and the odom
  // grab desired forces and rotation from so3
  const Eigen::Vector3d f_des(msg.force.x, msg.force.y, msg.force.z);

  const Eigen::Quaterniond q_des(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);

  // convert to tf::Quaternion
  tf2::Quaternion imu_tf(imu_q_.x(), imu_q_.y(), imu_q_.z(), imu_q_.w());
  tf2::Quaternion odom_tf(odom_q_.x(), odom_q_.y(), odom_q_.z(), odom_q_.w());

  // extract RPY's
  double imu_roll, imu_pitch, imu_yaw;
  double odom_roll, odom_pitch, odom_yaw;
  tf2::Matrix3x3(imu_tf).getRPY(imu_roll, imu_pitch, imu_yaw);
  tf2::Matrix3x3(odom_tf).getRPY(odom_roll, odom_pitch, odom_yaw);

  // create only yaw tf:Quaternions
  tf2::Quaternion imu_tf_yaw;
  tf2::Quaternion odom_tf_yaw;
  imu_tf_yaw.setRPY(0.0, 0.0, imu_yaw);
  odom_tf_yaw.setRPY(0.0, 0.0, odom_yaw);
  const tf2::Quaternion tf_imu_odom_yaw = imu_tf_yaw * odom_tf_yaw.inverse();

  // transform!
  const Eigen::Quaterniond q_des_transformed =
      Eigen::Quaterniond(tf_imu_odom_yaw.w(), tf_imu_odom_yaw.x(), tf_imu_odom_yaw.y(), tf_imu_odom_yaw.z()) * q_des;

  // check psi for stability
  const Eigen::Matrix3d R_des(q_des);
  const Eigen::Matrix3d R_cur(odom_q_);

  const float Psi = 0.5f * (3.0f - (R_des(0, 0) * R_cur(0, 0) + R_des(1, 0) * R_cur(1, 0) + R_des(2, 0) * R_cur(2, 0) +
                                    R_des(0, 1) * R_cur(0, 1) + R_des(1, 1) * R_cur(1, 1) + R_des(2, 1) * R_cur(2, 1) +
                                    R_des(0, 2) * R_cur(0, 2) + R_des(1, 2) * R_cur(1, 2) + R_des(2, 2) * R_cur(2, 2)));

  if(Psi > 1.0f)  // Position control stability guaranteed only when Psi < 1
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *(clock_.get()), 1000, "Psi > 1.0, orientation error is too large!");
  }

  double throttle = f_des(0) * R_cur(0, 2) + f_des(1) * R_cur(1, 2) + f_des(2) * R_cur(2, 2);

  // Scale force to individual rotor velocities (rad/s).
  throttle = std::sqrt(throttle / num_props_ / kf_);

  // Scaling from rotor velocity (rad/s) to att_throttle for pixhawk
  throttle = lin_cof_a_ * throttle + lin_int_b_;

  // failsafe for the error in traj_gen that can lead to nan values
  //prevents throttle from being sent to 1 if it is nan.
  if (isnan(throttle))
  {
    throttle = 0.0;
  }

  // clamp from 0.0 to 1.0
  throttle = std::min(1.0, throttle);
  throttle = std::max(0.0, throttle);

  if(motors_armed_)
    throttle = 0;

  // publish messages
  mavros_msgs::msg::AttitudeTarget setpoint_msg;
  setpoint_msg.header = msg.header;
  setpoint_msg.type_mask = 0; // @todo !!!!!! Set the correct type_mask !!!!!!
  setpoint_msg.orientation.w = q_des_transformed.w();
  setpoint_msg.orientation.x = q_des_transformed.x();
  setpoint_msg.orientation.y = q_des_transformed.y();
  setpoint_msg.orientation.z = q_des_transformed.z();
  setpoint_msg.body_rate.x = msg.angular_velocity.x;
  setpoint_msg.body_rate.y = msg.angular_velocity.y;
  setpoint_msg.body_rate.z = msg.angular_velocity.z;
  setpoint_msg.thrust = throttle;

  attitude_raw_pub_->publish(setpoint_msg);

  // save last so3_cmd
  last_se3_cmd_ = msg;
  last_se3_cmd_time_ = this->now();
  so3_cmd_set_ = true;

}

void
SE3ControllerToMavros::odomCallback(const nav_msgs::msg::Odometry &msg)
{
  odom_q_ = Eigen::Quaterniond(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                               msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
  odom_set_ = true;
}

void
SE3ControllerToMavros::imuCallback(const sensor_msgs::msg::Imu &msg)
{
  imu_q_ = Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  imu_set_ = true;
  
  double dt = this->now().seconds() - last_se3_cmd_time_.seconds();
  if(so3_cmd_set_ && ( dt >= se3_cmd_timeout_))
  {
    RCLCPP_INFO(this->get_logger(), "so3_cmd timeout. %f seconds since last command", dt);
    const auto last_se3_cmd_ptr = boost::make_shared<px4_geometric_controller::msg::SE3Command>(last_se3_cmd_);

    // so3_cmd_callback(last_se3_cmd_ptr);
    se3CmdCallback(last_se3_cmd_);
  }
}

void
SE3ControllerToMavros::publishMavrosSetpoint()
{

}