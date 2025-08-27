// motor_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "motor_controller.hpp"
#include "steering_controller.hpp"

using jetracer_control::SteeringController;

class MotorNode : public rclcpp::Node {
public:
    MotorNode() : Node("motor_node") {
        motor_controller_ = std::make_shared<MotorController>();
        steering_controller_ = std::make_shared<SteeringController>();
        
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MotorNode::cmdVelCallback, this, std::placeholders::_1)
        );
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        float throttle = msg->linear.x;     // forward/backward
        float steering = msg->angular.z;    // left/right

        motor_controller_->setThrottle(throttle);
        steering_controller_->setSteering(steering);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    std::shared_ptr<MotorController> motor_controller_;
    std::shared_ptr<SteeringController> steering_controller_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorNode>());
    rclcpp::shutdown();
    return 0;
}
