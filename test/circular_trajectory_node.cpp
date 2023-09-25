#include <rclcpp/rclcpp.hpp>
#include "geometric_controller_ros/msg/target_command.hpp"
#include <cmath>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class CircularTrajectoryPublisherNode : public rclcpp::Node
{
public:
    CircularTrajectoryPublisherNode() : Node("circular_trajectory_publisher_node")
    {
        this->declare_parameter("x_center", 0.0f);
        x_center_ = this->get_parameter("x_center").get_parameter_value().get<float>();

        this->declare_parameter("y_center", 0.0f);
        y_center_ = this->get_parameter("y_center").get_parameter_value().get<float>();

        this->declare_parameter("z", 2.0f);
        z_ = this->get_parameter("z").get_parameter_value().get<float>();

        this->declare_parameter("radius", 1.0f);
        radius_ = this->get_parameter("radius").get_parameter_value().get<float>();

        this->declare_parameter("speed", 2.0f);
        speed_ = this->get_parameter("speed").get_parameter_value().get<float>();

        publisher_ = this->create_publisher<geometric_controller_ros::msg::TargetCommand>("se3controller/setpoint", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&CircularTrajectoryPublisherNode::publishMessage, this));

        start_time_ = this->now();

        enable_motor_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "se3controller/enable_motors", 10, std::bind(&CircularTrajectoryPublisherNode::motorStateCallback, this, _1));
    }

private:
    void motorStateCallback(const std_msgs::msg::Bool & msg)
    {
      if (msg.data)
      {
        enable_motors_ = true;
        if(!start_time_is_set_)
        {
          start_time_ = this->now();
          start_time_is_set_ = true;
        }
      }
        
      else
      {
        start_time_is_set_ = false;
        enable_motors_ = false;
      }
              
    }

    void publishMessage()
    {
        // Publish only if the motors are engaged
        if(!enable_motors_)
          return;

        auto msg = std::make_unique<geometric_controller_ros::msg::TargetCommand>();

        double elapsed_time = (this->now() - start_time_).seconds();
        double omega = speed_ / radius_;

        // Position
        msg->position.x = x_center_ + radius_ * cos(omega * elapsed_time);
        msg->position.y = y_center_ + radius_ * sin(omega * elapsed_time);
        msg->position.z = z_;

        // Velocity
        msg->velocity.x = -speed_ * sin(omega * elapsed_time);
        msg->velocity.y = speed_ * cos(omega * elapsed_time);
        msg->velocity.z = 0.0;

        // Acceleration
        msg->acceleration.x = -radius_ * omega * omega * cos(omega * elapsed_time);
        msg->acceleration.y = -radius_ * omega * omega * sin(omega * elapsed_time);
        msg->acceleration.z = 0.0;

        // Jerk
        msg->jerk.x = radius_ * omega * omega * omega * sin(omega * elapsed_time);
        msg->jerk.y = -radius_ * omega * omega * omega * cos(omega * elapsed_time);
        msg->jerk.z = 0.0;

        // Yaw and Yaw Rate
        msg->yaw = 0.0 ;//atan2(msg->velocity.y, msg->velocity.x);
        msg->yaw_dot = 0.0; //omega;

        msg->kx = {0.0, 0.0, 0.0};
        msg->kv = {0.0, 0.0, 0.0};
        msg->use_msg_gains_flags = msg->USE_MSG_GAINS_NONE;

        publisher_->publish(*msg);
    }

    float x_center_, y_center_, z_, radius_, speed_;
    rclcpp::Publisher<geometric_controller_ros::msg::TargetCommand>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_motor_sub_;

    /*
    @brief ROS callback to motor state
    @param msg geometric_controller_ros::msg::TargetCommand
    */
    // void motorStateCallback(const std_msgs::msg::Bool & msg);


    rclcpp::Time start_time_;
    bool start_time_is_set_;

    bool enable_motors_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircularTrajectoryPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
