#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "aoa_reader.hpp"
#include "lidar_reader.hpp"

class FollowMeNode : public rclcpp::Node {
public:
    FollowMeNode()
        : Node("follow_me_node"),
          aoa_("/dev/ttyUSB0", 115200),
          lidar_("/dev/ttyUSB1", 115200)
    {
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        if (!aoa_.connect()) {
            RCLCPP_ERROR(get_logger(), "Failed to connect to AOA sensor");
        }
        if (!lidar_.connect()) {
            RCLCPP_ERROR(get_logger(), "Failed to connect to LiDAR");
        }

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FollowMeNode::controlLoop, this)
        );
    }

private:
    void controlLoop() {
        float angle = aoa_.getAngle();
        int distance = lidar_.readDistance();

        geometry_msgs::msg::Twist cmd;

        if (!aoa_.isValid() || distance <= 0) {
            RCLCPP_WARN(get_logger(), "Target lost or invalid data");
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub_->publish(cmd);
            return;
        }

        float desired_distance = 1000; // in mm
        float kp_linear = 0.001; // proportional gain for forward speed
        float kp_angular = 0.02; // proportional gain for turning

        float distance_error = distance - desired_distance;

        // Move forward/back
        cmd.linear.x = kp_linear * distance_error;

        // Turn toward target
        cmd.angular.z = -kp_angular * angle; // Negative if positive AOA means turn right

        // Safety clamp
        if (cmd.linear.x > 0.5) cmd.linear.x = 0.5;
        if (cmd.linear.x < -0.5) cmd.linear.x = -0.5;
        if (cmd.angular.z > 1.0) cmd.angular.z = 1.0;
        if (cmd.angular.z < -1.0) cmd.angular.z = -1.0;

        cmd_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    AOAReader aoa_;
    LiDARReader lidar_;
};
