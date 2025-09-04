#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class FollowMeNode : public rclcpp::Node {
public:
  FollowMeNode()
  : Node("followme_node"),
    angle_deg_(0.0f),
    distance_mm_filtered_(0.0f),
    filter_initialized_(false),
    last_angular_(0.0f)
  {
    // Parameters
    stop_distance_mm_      = declare_parameter<double>("stop_distance_mm", 100.0); // mm
    constant_speed_        = declare_parameter<double>("constant_speed",  0.2);   // m/s
    kp_angular_            = declare_parameter<double>("kp_angular",      0.02);  // angular gain
    max_angular_           = declare_parameter<double>("max_angular",     1.0);   // rad/s
    angle_deadband_deg_    = declare_parameter<double>("angle_deadband_deg", 3.0);// deadband
    distance_alpha_        = declare_parameter<double>("distance_lpf_alpha", 0.4);// smoothing

    // Subscriptions
    aoa_sub_ = create_subscription<std_msgs::msg::Float32>(
      "aoa_angle", 10, std::bind(&FollowMeNode::aoaCallback, this, std::placeholders::_1));

    lidar_sub_ = create_subscription<std_msgs::msg::Float32>(
      "lidar_distance", 10, std::bind(&FollowMeNode::lidarCallback, this, std::placeholders::_1));

    // Publisher
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Control loop timer (10 Hz)
    timer_ = create_wall_timer(100ms, std::bind(&FollowMeNode::controlLoop, this));
  }

private:
  void aoaCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    angle_deg_ = msg->data;
  }

  void lidarCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    float d = msg->data;  // mm
    if (d <= 0.0f || !std::isfinite(d)) return;

    if (!filter_initialized_) {
      distance_mm_filtered_ = d;
      filter_initialized_ = true;
    } else {
      distance_mm_filtered_ = static_cast<float>(
        distance_alpha_ * d + (1.0 - distance_alpha_) * distance_mm_filtered_);
    }
  }

  void controlLoop() {
    geometry_msgs::msg::Twist cmd;

    if (!filter_initialized_) {
      RCLCPP_WARN(this->get_logger(), "No LiDAR data yet, stopping.");
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);
      return;
    }

    // Forward speed logic
    RCLCPP_INFO(this->get_logger(), "LiDAR: %.2f mm | AOA: %.2f deg", distance_mm_filtered, angle_deg_);
    if (distance_mm_filtered_ > stop_distance_mm_) {
      cmd.linear.x = constant_speed_;
    } else {
      cmd.linear.x = 0.0; // Stop if too close
    }

    // Angular control with deadband
    if (std::fabs(angle_deg_) > angle_deadband_deg_) {
      cmd.angular.z = -kp_angular_ * angle_deg_;
      last_angular_ = cmd.angular.z; // Store last turn
    } else {
      cmd.angular.z = 0.0; // aligned
    }

    // Clamp angular speed
    if (cmd.angular.z > max_angular_) cmd.angular.z = max_angular_;
    if (cmd.angular.z < -max_angular_) cmd.angular.z = -max_angular_;

    // If we stopped (distance too close), still allow angular correction to face target
    if (cmd.linear.x == 0.0 && std::fabs(angle_deg_) > angle_deadband_deg_) {
      cmd.angular.z = last_angular_; // keep last direction to face object
    }

    cmd_pub_->publish(cmd);
  }

  // ROS members
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr aoa_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lidar_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  float angle_deg_;
  float distance_mm_filtered_;
  bool filter_initialized_;
  float last_angular_;

  // Parameters
  double stop_distance_mm_;
  double constant_speed_;
  double kp_angular_;
  double max_angular_;
  double angle_deadband_deg_;
  double distance_alpha_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowMeNode>());
  rclcpp::shutdown();
  return 0;
}
