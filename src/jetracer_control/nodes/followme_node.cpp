#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class FollowMeNode : public rclcpp::Node {
public:
    FollowMeNode() : Node("followme_node"), angle_(0.0), distance_(0.0) {
        // Subscribers
        aoa_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "aoa_angle", 10, std::bind(&FollowMeNode::aoaCallback, this, std::placeholders::_1));

        lidar_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "lidar_distance", 10, std::bind(&FollowMeNode::lidarCallback, this, std::placeholders::_1));

        // Publisher
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer for control loop
        timer_ = this->create_wall_timer(100ms, std::bind(&FollowMeNode::controlLoop, this));
    }

private:
    void aoaCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        angle_ = msg->data;  // Angle in degrees (or radians if you prefer)
    }

    void lidarCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        distance_ = msg->data;  // Distance in mm
    }

    void controlLoop() {
        geometry_msgs::msg::Twist cmd;

        if (distance_ <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid LiDAR data, stopping.");
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        } else {
            // Control gains
            float desired_distance = 1000.0; // in mm
            float kp_linear = 0.001;         // proportional gain for forward speed
            float kp_angular = 0.02;         // proportional gain for turning

            // Compute errors
            float distance_error = distance_ - desired_distance;

            // Linear speed control
            cmd.linear.x = kp_linear * distance_error;

            // Angular speed control
            cmd.angular.z = -kp_angular * angle_; // Negative if angle > 0 means turn right

            // Safety clamps
            if (cmd.linear.x > 0.5) cmd.linear.x = 0.5;
            if (cmd.linear.x < -0.5) cmd.linear.x = -0.5;
            if (cmd.angular.z > 1.0) cmd.angular.z = 1.0;
            if (cmd.angular.z < -1.0) cmd.angular.z = -1.0;
        }

        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr aoa_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lidar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float angle_;
    float distance_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowMeNode>());
    rclcpp::shutdown();
    return 0;
}
