#include <px4_geometric_controller/SE3Controller.h>
#include "px4_geometric_controller/msg/se3_command.hpp"
#include "px4_geometric_controller/msg/target_command.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include "mavros_msgs/msg/attitude_target.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

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

class SE3ControllerToMAVROS: public rclcpp::Node
{
public:
  SE3ControllerToMAVROS();
  ~SE3ControllerToMAVROS();

  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  // Need this since we have SO3Control which needs aligned pointer

private:
  void publishSO3Command();
  void odom_callback(const nav_msgs::msg::Odometry & msg);
  // void enable_motors_callback(const std_msgs::Bool::ConstPtr &msg); // @todo Make it a service!1
  // void corrections_callback(const kr_mav_msgs::Corrections::ConstPtr &msg); // Maybe we don't need it!! 
  // void cfg_callback(kr_mav_controllers::SO3Config &config, uint32_t level); // @todo Need to adapt to ros2
  
  // Callbacks
  void target_cmd_callback(const px4_geometric_controller::msg::TargetCommand & msg);// @todo change to suitable ros2 message
  void odom_callback(const nav_msgs::msg::Odometry & msg);
  // void imu_callback(const sensor_msgs::Imu::ConstPtr &pose); /* No need ?!*/

  /*Publishers */
  rclcpp::Publisher<px4_geometric_controller::msg::SE3Command>::SharedPtr se3_command_pub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_raw_pub_; // publisher to mavros setpoints
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  odom_pose_pub_;  // For sending PoseStamped to firmware ??
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  command_viz_pub_; // cmd visulaization in RViz2

  /* Subscribers */
  rclcpp::Subscription<px4_geometric_controller::msg::TargetCommand>::SharedPtr target_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // ros::Subscriber imu_sub_; /* Probably odom is sufficient ! */
  // ros::Subscriber enable_motors_sub_, corrections_sub_;

  
  SE3Controller controller_; /* SE3 controller object */

  /* flags */
  bool odom_set_, imu_set_, so3_cmd_set_, position_cmd_updated_, position_cmd_init_; 
  bool enable_motors_, use_external_yaw_, have_odom_;

  Eigen::Quaterniond odom_q_, imu_q_;
  double kf_, lin_cof_a_, lin_int_b_;
  int num_props_;  

  // double so3_cmd_timeout_;
  // ros::Time last_so3_cmd_time_;
  // kr_mav_msgs::SO3Command last_so3_cmd_; // Need to change to suitable ros 2 msg

  
  std::string frame_id_; /* of the command */

  Eigen::Vector3f des_pos_, des_vel_, des_acc_, des_jrk_, config_kx_, config_kv_, config_ki_, config_kib_, kx_, kv_;
  float des_yaw_, des_yaw_dot_;
  float current_yaw_;
  Eigen::Quaternionf current_orientation_;
  
  /* drone-specific params */
  float kR_[3], kOm_[3], corrections_[3];
  float mass_;
  const float g_;
};

//////////////// Class definitions ////////////////
SE3ControllerToMAVROS::SE3ControllerToMAVROS(): Node("se3_controller_to_mavros_node"),
position_cmd_updated_(false),
        position_cmd_init_(false),
        des_yaw_(0),
        des_yaw_dot_(0),
        current_yaw_(0),
        enable_motors_(false),
        use_external_yaw_(false),
        have_odom_(false),
        g_(9.81),
        current_orientation_(Eigen::Quaternionf::Identity())
{
  /* Get params */

  /* Define subscribers and publishers */

  controller_.resetIntegrals();
}

SE3ControllerToMAVROS::~SE3ControllerToMAVROS()
{

}

void
SE3ControllerToMAVROS::publishSO3Command()
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

  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_jrk_, des_yaw_, des_yaw_dot_, kx_, kv_, ki, kib);

  const Eigen::Vector3f &force = controller_.getComputedForce();
  const Eigen::Quaternionf &orientation = controller_.getComputedOrientation();
  const Eigen::Vector3f &ang_vel = controller_.getComputedAngularVelocity();

  // kr_mav_msgs::SO3Command::Ptr so3_command = boost::make_shared<kr_mav_msgs::SO3Command>();
  px4_geometric_controller::msg::SE3Command se3_cmd;
  se3_cmd.header.stamp = this->now();
  se3_cmd.header.frame_id = frame_id_;
  se3_cmd.force.x = force(0);
  se3_cmd.force.y = force(1);
  se3_cmd.force.z = force(2);
  se3_cmd.orientation.x = orientation.x();
  se3_cmd.orientation.y = orientation.y();
  se3_cmd.orientation.z = orientation.z();
  se3_cmd.orientation.w = orientation.w();
  se3_cmd.angular_velocity.x = ang_vel(0);
  se3_cmd.angular_velocity.y = ang_vel(1);
  se3_cmd.angular_velocity.z = ang_vel(2);

  for(int i = 0; i < 3; i++)
  {
    se3_cmd.kr[i] = kR_[i];
    se3_cmd.kom[i] = kOm_[i];
  }

  // so3_command->aux.current_yaw = current_yaw_;
  // so3_command->aux.kf_correction = corrections_[0];
  // so3_command->aux.angle_corrections[0] = corrections_[1];
  // so3_command->aux.angle_corrections[1] = corrections_[2];
  // so3_command->aux.enable_motors = enable_motors_;
  // so3_command->aux.use_external_yaw = use_external_yaw_;
  se3_command_pub_->publish(se3_cmd);

  geometry_msgs::msg::PoseStamped cmd_viz_msg;


  cmd_viz_msg.header = se3_cmd.header;
  cmd_viz_msg.pose.position.x = des_pos_(0);
  cmd_viz_msg.pose.position.y = des_pos_(1);
  cmd_viz_msg.pose.position.z = des_pos_(2);
  cmd_viz_msg.pose.orientation.x = orientation.x();
  cmd_viz_msg.pose.orientation.y = orientation.y();
  cmd_viz_msg.pose.orientation.z = orientation.z();
  cmd_viz_msg.pose.orientation.w = orientation.w();
  command_viz_pub_->publish(cmd_viz_msg);
  
  // @todo publish to mavros
}

/**
 * Main function
*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SE3ControllerToMAVROS>());
  rclcpp::shutdown();
  return 0;
}