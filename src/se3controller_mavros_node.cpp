#include <px4_geometric_controller/SE3Controller.h>
#include "px4_geometric_controller/msg/se3_command.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include "mavros_msgs/msg/attitude_target.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
  @brief Publish SE3 command to mavros topic.
  */
  void publishMavrosSetpoint();

  rclcpp::Subscription<px4_geometric_controller::msg::SE3Command>::SharedPtr se3_cmd_sub_;

  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_raw_pub_; // publisher for mavros setpoints

  Eigen::Quaterniond odom_q_, imu_q_;
  double kf_, lin_cof_a_, lin_int_b_; // need to check if they are needed
  int num_props_;

  double so3_cmd_timeout_;
  // ros::Time last_so3_cmd_time_;
  rclcpp::Time last_so3_cmd_time_;
  px4_geometric_controller::msg::SE3Command last_so3_cmd_;

};

/////////////////////////////////////////////////////
// Class definition //
/////////////////////////////////////////////////////
SE3ControllerToMavros::SE3ControllerToMavros(): Node("se3controller_to_mavros_node")
{
    attitude_raw_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("mavros/attitude_target", 10);

    se3_cmd_sub_ = this->create_subscription<px4_geometric_controller::msg::SE3Command>(
      "se3controller/cmd", 10, std::bind(&SE3ControllerToMavros::se3CmdCallback, this, _1));
}

SE3ControllerToMavros::~SE3ControllerToMavros()
{
  /* Destructor */
}

void
SE3ControllerToMavros::se3CmdCallback(const px4_geometric_controller::msg::SE3Command & msg)
{

  // grab desired forces and rotation from se3
  const Eigen::Vector3d f_des(msg.force.x, msg.force.y, msg.force.z);

  const Eigen::Quaterniond q_des(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);

}

void
SE3ControllerToMavros::publishMavrosSetpoint()
{

}