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
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

class SE3ControllerNode: public rclcpp::Node
{
public:
  SE3ControllerNode();
  ~SE3ControllerNode();

  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  // Need this since we have SO3Control which needs aligned pointer

private:
  // void enable_motors_callback(const std_msgs::Bool::ConstPtr &msg); // @todo Make it a service!
  // void corrections_callback(const kr_mav_msgs::Corrections::ConstPtr &msg); // Maybe we don't need it!! 
  // void cfg_callback(kr_mav_controllers::SO3Config &config, uint32_t level); // @todo Need to adapt to ros2
  
 
  /*
  @brief Publish SE3 command to the flight controller
  */
  void publishSE3Command();
  /*
  @brief ROS callback to receive setpoints of the SE2 controller
  @param msg px4_geometric_controller::msg::TargetCommand
  */
  void targetCmdCallback(const px4_geometric_controller::msg::TargetCommand & msg);

  /*
  @brief Odometry ROS callback to receive linear and rotational measurements
  @param msg nav_msgs::msg::Odometry
  */
  void odomCallback(const nav_msgs::msg::Odometry & msg);

  // void imu_callback(const sensor_msgs::Imu::ConstPtr &pose); /* No need ?!*/

  /*Publishers */
  rclcpp::Publisher<px4_geometric_controller::msg::SE3Command>::SharedPtr se3_command_pub_;
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
SE3ControllerNode::SE3ControllerNode(): Node("se3controller_node"),
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
  this->declare_parameter("mass", 0.5);
  mass_ = this->get_parameter("mass").get_parameter_value().get<float>();
  controller_.setMass(mass_);
  controller_.setGravity(g_);

  this->declare_parameter("use_external_yaw", true);
  use_external_yaw_ = this->get_parameter("use_external_yaw").get_parameter_value().get<bool>();

  this->declare_parameter("gains/pos/x", 7.4f);
  config_kx_[0] = this->get_parameter("gains/pos/x").get_parameter_value().get<float>();

  this->declare_parameter("gains/pos/y", 7.4f);
  config_kx_[1] = this->get_parameter("gains/pos/y").get_parameter_value().get<float>();

  this->declare_parameter("gains/pos/z", 10.4f);
  config_kx_[2] = this->get_parameter("gains/pos/z").get_parameter_value().get<float>();

  kx_[0] = config_kx_[0];
  kx_[1] = config_kx_[1];
  kx_[2] = config_kx_[2];

  this->declare_parameter("gains/vel/x", 4.8f);
  config_kv_[0] = this->get_parameter("gains/vel/x").get_parameter_value().get<float>();

  this->declare_parameter("gains/vel/y", 4.8f);
  config_kv_[1] = this->get_parameter("gains/vel/y").get_parameter_value().get<float>();

  this->declare_parameter("gains/vel/z", 6.0f);
  config_kv_[2] = this->get_parameter("gains/vel/z").get_parameter_value().get<float>();

  kv_[0] = config_kv_[0];
  kv_[1] = config_kv_[1];
  kv_[2] = config_kv_[2];

  this->declare_parameter("gains/ki/x", 0.0f);
  config_ki_[0] = this->get_parameter("gains/ki/x").get_parameter_value().get<float>();

  this->declare_parameter("gains/ki/y", 0.0f);
  config_ki_[1] = this->get_parameter("gains/ki/y").get_parameter_value().get<float>();

  this->declare_parameter("gains/ki/z", 0.0f);
  config_ki_[2] = this->get_parameter("gains/ki/z").get_parameter_value().get<float>();

  this->declare_parameter("gains/kib/x", 0.0f);
  config_kib_[0] = this->get_parameter("gains/kib/x").get_parameter_value().get<float>();

  this->declare_parameter("gains/kib/y", 0.0f);
  config_kib_[1] = this->get_parameter("gains/kib/y").get_parameter_value().get<float>();
  
  this->declare_parameter("gains/kib/z", 0.0f);
  config_kib_[2] = this->get_parameter("gains/kib/z").get_parameter_value().get<float>();

  this->declare_parameter("gains/rot/x", 1.5f);
  kR_[0] = this->get_parameter("gains/rot/x").get_parameter_value().get<float>();

  this->declare_parameter("gains/rot/y", 1.5f);
  kR_[1] = this->get_parameter("gains/rot/y").get_parameter_value().get<float>();

  this->declare_parameter("gains/rot/z", 1.0f);
  kR_[2] = this->get_parameter("gains/rot/z").get_parameter_value().get<float>();

  this->declare_parameter("gains/ang/x", 0.13f);
  kOm_[0] = this->get_parameter("gains/ang/x").get_parameter_value().get<float>();

  this->declare_parameter("gains/ang/y", 0.13f);
  kOm_[1] = this->get_parameter("gains/ang/y").get_parameter_value().get<float>();

  this->declare_parameter("gains/ang/z", 0.1f);
  kOm_[2] = this->get_parameter("gains/ang/z").get_parameter_value().get<float>();

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

  se3_command_pub_ = this->create_publisher<px4_geometric_controller::msg::SE3Command>("se3controller/cmd", 10);
  odom_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("se3controller/odom_pose", 10);
  command_viz_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("se3controller/cmd_pose", 10);

  target_cmd_sub_ = this->create_subscription<px4_geometric_controller::msg::TargetCommand>(
      "se3controller/setpoint", 10, std::bind(&SE3ControllerNode::targetCmdCallback, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "se3controller/odom", 10, std::bind(&SE3ControllerNode::odomCallback, this, _1));
}

SE3ControllerNode::~SE3ControllerNode()
{
  /* Destructor */
}

void
SE3ControllerNode::odomCallback(const nav_msgs::msg::Odometry & msg)
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
SE3ControllerNode::targetCmdCallback(const px4_geometric_controller::msg::TargetCommand & msg)
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

  des_yaw_ = msg.yaw;
  des_yaw_dot_ = msg.yaw_dot;
  position_cmd_updated_ = true;
  // position_cmd_init_ = true;

  publishSE3Command();
}


void
SE3ControllerNode::publishSE3Command()
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
}

/**
 * Main function
*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SE3ControllerNode>());
  rclcpp::shutdown();
  return 0;
}