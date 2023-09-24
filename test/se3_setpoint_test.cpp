#include <rclcpp/rclcpp.hpp>
#include "geometric_controller_ros/msg/target_command.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class TargetCommandPublisherNode : public rclcpp::Node
{
public:
    TargetCommandPublisherNode() : Node("se3_setpoint_publisher_node")
    {
        this->declare_parameter("x", 0.0f);
        x_ = this->get_parameter("x").get_parameter_value().get<float>();
        
        this->declare_parameter("y", 0.0f);
        y_ = this->get_parameter("y").get_parameter_value().get<float>();

        this->declare_parameter("z", 0.0f);
        z_ = this->get_parameter("z").get_parameter_value().get<float>();

        this->declare_parameter("yaw", 0.0f);
        yaw_ = this->get_parameter("yaw").get_parameter_value().get<float>();

        publisher_ = this->create_publisher<geometric_controller_ros::msg::TargetCommand>("se3controller/setpoint", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&TargetCommandPublisherNode::publishMessage, this));
    }

private:
    void publishMessage()
    {
        auto msg = std::make_unique<geometric_controller_ros::msg::TargetCommand>();

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

    // target positions
    float x_, y_, z_, yaw_;

    rclcpp::Publisher<geometric_controller_ros::msg::TargetCommand>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetCommandPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
