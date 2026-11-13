// Copyright 2026 Motor Control Sim Authors
// Licensed under Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <string>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class CmdVelPublisherNode : public rclcpp::Node
{
public:
  CmdVelPublisherNode()
  : Node("cmd_vel_publisher")
  {
    // Declare parameters
    this->declare_parameter<double>("publish_rate", 10.0);
    this->declare_parameter<std::string>("pattern", "sinusoidal");
    this->declare_parameter<double>("amplitude", 1.0);
    this->declare_parameter<double>("frequency", 0.2);

    // Read parameters
    double publish_rate = this->get_parameter("publish_rate").as_double();
    pattern_ = this->get_parameter("pattern").as_string();
    amplitude_ = this->get_parameter("amplitude").as_double();
    frequency_ = this->get_parameter("frequency").as_double();

    // Publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Record start time
    start_time_ = this->now();

    // Timer
    double period = 1.0 / publish_rate;
    auto timer_period = std::chrono::duration<double>(period);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&CmdVelPublisherNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "cmd_vel_publisher started: pattern=%s, amplitude=%.2f, frequency=%.2f Hz, rate=%.1f Hz",
      pattern_.c_str(), amplitude_, frequency_, publish_rate);
  }

private:
  void timer_callback()
  {
    double t = (this->now() - start_time_).seconds();

    auto msg = geometry_msgs::msg::Twist();

    if (pattern_ == "sinusoidal") {
      msg.linear.x = amplitude_ * std::sin(2.0 * M_PI * frequency_ * t);
      msg.linear.y = 0.0;
      msg.linear.z = amplitude_ * 0.3 * std::sin(M_PI * frequency_ * t);
      msg.angular.z = 0.0;
    } else if (pattern_ == "step") {
      double s = std::sin(2.0 * M_PI * frequency_ * t);
      msg.linear.x = (s >= 0.0) ? amplitude_ : -amplitude_;
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;
      msg.angular.z = 0.0;
    } else if (pattern_ == "circle") {
      msg.linear.x = 0.5 * amplitude_;
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;
      msg.angular.z = amplitude_ * std::sin(2.0 * M_PI * frequency_ * t);
    } else {
      RCLCPP_WARN_ONCE(this->get_logger(), "Unknown pattern '%s', defaulting to zero", pattern_.c_str());
    }

    cmd_vel_pub_->publish(msg);
  }

  // Members
  std::string pattern_;
  double amplitude_;
  double frequency_;
  rclcpp::Time start_time_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
