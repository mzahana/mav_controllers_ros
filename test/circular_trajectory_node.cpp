#include <rclcpp/rclcpp.hpp>
#include "mav_controllers_ros/msg/target_command.hpp"
#include <cmath>
#include <std_msgs/msg/bool.hpp>
#include "mavros_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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

        this->declare_parameter("publish_rate_ms", 10);
        int rate_ms = this->get_parameter("publish_rate_ms").get_parameter_value().get<int>();
        publish_duration_ = std::chrono::milliseconds(rate_ms);

        this->declare_parameter("max_yaw_rate", 0.05f);
        max_yaw_rate_ = this->get_parameter("max_yaw_rate").get_parameter_value().get<float>();

        publisher_ = this->create_publisher<mav_controllers_ros::msg::TargetCommand>("se3controller/setpoint", 10);
        timer_ = this->create_wall_timer(publish_duration_, std::bind(&CircularTrajectoryPublisherNode::publishMessage, this));

        start_time_ = this->now();

        mavros_state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&CircularTrajectoryPublisherNode::mavrosStateCallback, this, _1));

        enable_motor_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "se3controller/enable_motors", 10, std::bind(&CircularTrajectoryPublisherNode::motorStateCallback, this, _1));

        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "mavros/local_position/odom", rclcpp::SensorDataQoS(), std::bind(&CircularTrajectoryPublisherNode::odometryCallback, this, _1));
    }

private:
    void motorStateCallback(const std_msgs::msg::Bool & msg)
    {
        if (msg.data)
        {
            enable_motors_ = true;
        }
        else
        {
            start_time_is_set_ = false;
            enable_motors_ = false;
        }
    }

    void mavrosStateCallback(const mavros_msgs::msg::State & msg)
    {
        if (msg.mode == msg.MODE_PX4_OFFBOARD)
            offboard_mode_ = true;      
        else
        {
            offboard_mode_ = false;
            start_time_is_set_ = false;
        }
    }

    void odometryCallback(const nav_msgs::msg::Odometry & msg)
    {
        double roll, pitch, yaw;
        tf2::Quaternion q(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        actual_yaw_ = yaw;
        // Update the current drone position    
        current_drone_position_x_ = msg.pose.pose.position.x;
        current_drone_position_y_ = msg.pose.pose.position.y;
    }

    void publishMessage()
    {
        auto msg = std::make_unique<mav_controllers_ros::msg::TargetCommand>();
        // if(!enable_motors_ or !offboard_mode_)
        // {
        //     RCLCPP_WARN(this->get_logger(), "Not armed, not OFFBOARD MODE. Not publishing setpoints");
        //     publisher_->publish(*msg);
        //     return;
        // }
        if(!start_time_is_set_)
        {
            start_time_ = this->now();
            start_time_is_set_ = true;
        }

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
        // double desired_yaw = atan2(msg->velocity.y, msg->velocity.x);
        // Compute the direction vector
        double dir_x = msg->position.x - current_drone_position_x_;
        double dir_y = msg->position.y - current_drone_position_y_;

        // Compute the desired yaw based on the direction vector
        double desired_yaw = atan2(dir_y, dir_x);

        double yaw_mod = fmod(
        desired_yaw - actual_yaw_,
        2 * M_PI);
        if (yaw_mod < -M_PI) {
            yaw_mod += 2 * M_PI;
        } else if (yaw_mod > M_PI) {
            yaw_mod -= 2 * M_PI;
        }
        desired_yaw = actual_yaw_ + yaw_mod;
        // if((desired_yaw - actual_yaw_) > 0.2) desired_yaw = actual_yaw_+0.2;
        RCLCPP_INFO(this->get_logger(), "des_yaw = %f0.2, actual_yaw = %0.2f", desired_yaw, actual_yaw_);
        

        double yaw_diff = desired_yaw - actual_yaw_;
        double seconds_duration = static_cast<double>(publish_duration_.count()) / 1000.0;
        double limited_yaw_rate = std::clamp(yaw_diff / seconds_duration, static_cast<double>(-max_yaw_rate_), static_cast<double>(max_yaw_rate_));
        double new_des_yaw = actual_yaw_+  limited_yaw_rate * seconds_duration;

        msg->yaw = new_des_yaw;
        msg->yaw_dot = 0.0; //limited_yaw_rate;

        msg->kx = {0.0, 0.0, 0.0};
        msg->kv = {0.0, 0.0, 0.0};
        msg->use_msg_gains_flags = msg->USE_MSG_GAINS_NONE;

        publisher_->publish(*msg);
    }

    // Declarations
    
    float x_center_, y_center_, z_, radius_, speed_, max_yaw_rate_;
    rclcpp::Publisher<mav_controllers_ros::msg::TargetCommand>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds publish_duration_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_motor_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    rclcpp::Time start_time_;
    bool start_time_is_set_ = false;
    bool enable_motors_ = false;
    bool offboard_mode_ = false;
    double actual_yaw_ = 0.0;
    double current_drone_position_x_ = 0.0;
    double current_drone_position_y_ = 0.0;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircularTrajectoryPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
