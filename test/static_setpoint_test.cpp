#include <rclcpp/rclcpp.hpp>
#include "mav_controllers_ros/msg/target_command.hpp"
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class TargetCommandPublisherNode : public rclcpp::Node
{
public:
    TargetCommandPublisherNode() : Node("static_setpoint_publisher_node")
    {
        this->declare_parameter("x", 0.0f);
        x_ = this->get_parameter("x").get_parameter_value().get<float>();
        
        this->declare_parameter("y", 0.0f);
        y_ = this->get_parameter("y").get_parameter_value().get<float>();

        this->declare_parameter("z", 2.0f);
        z_ = this->get_parameter("z").get_parameter_value().get<float>();

        this->declare_parameter("yaw", 0.0f);
        yaw_ = this->get_parameter("yaw").get_parameter_value().get<float>();

        this->declare_parameter("use_multi_dof_msg", true);
        use_multi_dof_msg_ = this->get_parameter("use_multi_dof_msg").get_parameter_value().get<bool>();

        this->declare_parameter("publish_rate_ms", 10);
        int rate_ms = this->get_parameter("publish_rate_ms").get_parameter_value().get<int>();
        publish_duration_ = std::chrono::milliseconds(rate_ms);

        this->declare_parameter("max_yaw_rate", 0.05f);
        max_yaw_rate_ = this->get_parameter("max_yaw_rate").get_parameter_value().get<float>();

        

        publisher_ = this->create_publisher<mav_controllers_ros::msg::TargetCommand>("se3controller/setpoint", 10);
        multi_dof_publisher_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>("se3controller/multi_dof_setpoint", 10);
        timer_ = this->create_wall_timer(publish_duration_, std::bind(&TargetCommandPublisherNode::publishMessage, this));
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "mavros/local_position/odom", rclcpp::SensorDataQoS(), std::bind(&TargetCommandPublisherNode::odomCallback, this, _1));
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry& msg)
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
    }
    void publishMessage()
    {
        // Compute the difference between desired yaw and actual yaw
        float yaw_diff = yaw_ - actual_yaw_;
        // Ensure yaw_diff is in the range [-pi, pi]
        if (yaw_diff > M_PI) {
            yaw_diff -= 2 * M_PI;
        } else if (yaw_diff < -M_PI) {
            yaw_diff += 2 * M_PI;
        }

        // Limit the yaw_diff based on max_yaw_rate_ and publish duration
        float max_yaw_change = max_yaw_rate_ * publish_duration_.count() / 1000.0; // Assuming publish_duration_ is in milliseconds
        if (yaw_diff > max_yaw_change) {
            yaw_diff = max_yaw_change;
        } else if (yaw_diff < -max_yaw_change) {
            yaw_diff = -max_yaw_change;
        }

        // Compute the new limited yaw setpoint
        float limited_yaw = actual_yaw_ + yaw_diff;

        // Ensure limited_yaw is in the range [-pi, pi]
        if (limited_yaw > M_PI) {
            limited_yaw -= 2 * M_PI;
        } else if (limited_yaw < -M_PI) {
            limited_yaw += 2 * M_PI;
        }
        if (use_multi_dof_msg_)
        {
            auto multi_dof_msg = std::make_unique<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>();

            multi_dof_msg->transforms.resize(1);
            multi_dof_msg->transforms[0].translation.x = x_;
            multi_dof_msg->transforms[0].translation.y = y_;
            multi_dof_msg->transforms[0].translation.z = z_;
            multi_dof_msg->velocities.resize(1);
            multi_dof_msg->velocities[0].linear.x = 0.0;
            multi_dof_msg->velocities[0].linear.y = 0.0;
            multi_dof_msg->velocities[0].linear.z = 0.0;
            multi_dof_msg->accelerations.resize(1);
            multi_dof_msg->accelerations[0].linear.x = 0.0;
            multi_dof_msg->accelerations[0].linear.y = 0.0;
            multi_dof_msg->accelerations[0].linear.z = 0.0;
            tf2::Quaternion quat;
            // quat.setEuler(yaw_, 0.0, 0.0);
            quat.setEuler(limited_yaw, 0.0, 0.0);
            multi_dof_msg->transforms[0].rotation.w = quat.getW();
            multi_dof_msg->transforms[0].rotation.x = quat.getX();
            multi_dof_msg->transforms[0].rotation.y = quat.getY();
            multi_dof_msg->transforms[0].rotation.z = quat.getZ();
            multi_dof_msg->velocities[0].angular.x = 0.0;
            multi_dof_msg->velocities[0].angular.y = 0.0;
            multi_dof_msg->velocities[0].angular.z = 0.0;

            multi_dof_publisher_->publish(*multi_dof_msg);
        }
        else
        {
            auto msg = std::make_unique<mav_controllers_ros::msg::TargetCommand>();

            // Fill in your fixed values here
            msg->position.x = x_;
            msg->position.y = y_;
            msg->position.z = z_;
            msg->velocity.x = 0.0;
            msg->velocity.y = 0.0;
            msg->velocity.z = 0.0;
            msg->acceleration.x = 0.0;
            msg->acceleration.y = 0.0;
            msg->acceleration.z = 0.0;
            msg->jerk.x = 0.0;
            msg->jerk.y = 0.0;
            msg->jerk.z = 0.0;

            RCLCPP_INFO(this->get_logger(), "Current yaw = %0.2f", actual_yaw_);
            msg->yaw = limited_yaw;
            msg->yaw_dot = 0.0;
            msg->kx = {0.0, 0.0, 0.0};
            msg->kv = {0.0, 0.0, 0.0};
            msg->use_msg_gains_flags = msg->USE_MSG_GAINS_NONE;

            publisher_->publish(*msg);
        }
        
    }

    // target positions
    float x_, y_, z_, yaw_, actual_yaw_;

    bool use_multi_dof_msg_;

    std::chrono::milliseconds publish_duration_;
    float max_yaw_rate_;

    

    rclcpp::Publisher<mav_controllers_ros::msg::TargetCommand>::SharedPtr publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr multi_dof_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetCommandPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
