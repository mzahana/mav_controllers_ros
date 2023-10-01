#include <rclcpp/rclcpp.hpp>
#include "mav_controllers_ros/msg/target_command.hpp"
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <tf2/LinearMath/Quaternion.h>

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

        

        publisher_ = this->create_publisher<mav_controllers_ros::msg::TargetCommand>("se3controller/setpoint", 10);
        multi_dof_publisher_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>("se3controller/multi_dof_setpoint", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&TargetCommandPublisherNode::publishMessage, this));
    }

private:
    void publishMessage()
    {
        if (use_multi_dof_msg_)
        {
            auto multi_dof_msg = std::make_unique<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>();

            multi_dof_msg->transforms.resize(1);
            multi_dof_msg->transforms[0].translation.x = x_;
            multi_dof_msg->transforms[0].translation.y = y_;
            multi_dof_msg->transforms[0].translation.z = z_;
            multi_dof_msg->velocities.resize(1);
            multi_dof_msg->velocities[0].linear.x = 10.0;
            multi_dof_msg->velocities[0].linear.y = 10.0;
            multi_dof_msg->velocities[0].linear.z = 10.0;
            multi_dof_msg->accelerations.resize(1);
            multi_dof_msg->accelerations[0].linear.x = 3.0;
            multi_dof_msg->accelerations[0].linear.y = 3.0;
            multi_dof_msg->accelerations[0].linear.z = 3.0;
            tf2::Quaternion quat;
            quat.setEuler(yaw_, 0.0, 0.0);
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
            msg->yaw = yaw_;
            msg->yaw_dot = 0.0;
            msg->kx = {0.0, 0.0, 0.0};
            msg->kv = {0.0, 0.0, 0.0};
            msg->use_msg_gains_flags = msg->USE_MSG_GAINS_NONE;

            publisher_->publish(*msg);
        }
        
    }

    // target positions
    float x_, y_, z_, yaw_;

    bool use_multi_dof_msg_;

    rclcpp::Publisher<mav_controllers_ros::msg::TargetCommand>::SharedPtr publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr multi_dof_publisher_;
    
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetCommandPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
