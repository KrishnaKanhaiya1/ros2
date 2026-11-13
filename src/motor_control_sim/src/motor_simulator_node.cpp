// Copyright 2026 Motor Control Sim Authors
// Licensed under Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "motor_control_msgs/msg/motor_rpm.hpp"
#include "motor_control_msgs/msg/motor_telemetry.hpp"
#include "motor_control_sim/motor_model.hpp"
#include "motor_control_sim/thruster_allocator.hpp"

#include <vector>
#include <memory>
#include <chrono>
#include <string>
#include <cmath>

using namespace std::chrono_literals;

class MotorSimulatorNode : public rclcpp::Node
{
public:
  MotorSimulatorNode()
  : Node("motor_simulator")
  {
    // Declare parameters
    this->declare_parameter<int>("num_thrusters", 6);
    this->declare_parameter<double>("max_rpm", 3000.0);
    this->declare_parameter<double>("time_constant", 0.3);
    this->declare_parameter<double>("thrust_coefficient", 0.0005);
    this->declare_parameter<double>("dead_band_rpm", 50.0);
    this->declare_parameter<double>("velocity_to_rpm_scale", 1000.0);
    this->declare_parameter<double>("simulation_rate", 50.0);
    this->declare_parameter<double>("nominal_voltage", 16.0);

    // Read parameters
    num_thrusters_ = this->get_parameter("num_thrusters").as_int();
    double max_rpm = this->get_parameter("max_rpm").as_double();
    double time_constant = this->get_parameter("time_constant").as_double();
    double thrust_coefficient = this->get_parameter("thrust_coefficient").as_double();
    double dead_band_rpm = this->get_parameter("dead_band_rpm").as_double();
    double velocity_to_rpm_scale = this->get_parameter("velocity_to_rpm_scale").as_double();
    double simulation_rate = this->get_parameter("simulation_rate").as_double();
    double nominal_voltage = this->get_parameter("nominal_voltage").as_double();

    // Build motor params struct
    motor_control_sim::MotorParams motor_params;
    motor_params.max_rpm = max_rpm;
    motor_params.min_rpm = -max_rpm;
    motor_params.time_constant = time_constant;
    motor_params.thrust_coefficient = thrust_coefficient;
    motor_params.dead_band_rpm = dead_band_rpm;
    motor_params.nominal_voltage = nominal_voltage;

    // Create thruster allocator
    allocator_ = std::make_unique<motor_control_sim::ThrusterAllocator>(max_rpm, velocity_to_rpm_scale);

    // Create motor instances
    for (int i = 0; i < num_thrusters_; ++i) {
      std::string name = (i < motor_control_sim::ThrusterAllocator::NUM_THRUSTERS)
                           ? motor_control_sim::ThrusterAllocator::THRUSTER_NAMES[i]
                           : "motor_" + std::to_string(i);
      motors_.emplace_back(i, name, motor_params);
    }

    // Subscriber to /cmd_vel
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&MotorSimulatorNode::cmd_vel_callback, this, std::placeholders::_1));

    // Publisher for aggregate MotorRPM
    rpm_pub_ = this->create_publisher<motor_control_msgs::msg::MotorRPM>("/motor/rpm", 10);

    // Per-motor telemetry publishers
    for (int i = 0; i < num_thrusters_; ++i) {
      std::string topic = "/motor/" + motors_[i].get_state().name + "/telemetry";
      telemetry_pubs_.push_back(
        this->create_publisher<motor_control_msgs::msg::MotorTelemetry>(topic, 10));
    }

    // Simulation timer
    dt_ = 1.0 / simulation_rate;
    auto timer_period = std::chrono::duration<double>(dt_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&MotorSimulatorNode::simulation_callback, this));

    // Initialise cmd state
    last_cmd_time_ = this->now();
    cmd_received_ = false;

    RCLCPP_INFO(this->get_logger(),
      "Motor simulator started: %d thrusters, %.1f Hz, max RPM %.0f",
      num_thrusters_, simulation_rate, max_rpm);
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_ = *msg;
    last_cmd_time_ = this->now();
    cmd_received_ = true;
  }

  void simulation_callback()
  {
    // Determine velocity inputs with timeout check
    double surge = 0.0, sway = 0.0, heave = 0.0, yaw = 0.0;

    if (cmd_received_) {
      double dt_since_cmd = (this->now() - last_cmd_time_).seconds();
      if (dt_since_cmd < 1.0) {
        surge = latest_cmd_.linear.x;
        sway = latest_cmd_.linear.y;
        heave = latest_cmd_.linear.z;
        yaw = latest_cmd_.angular.z;
      } else {
        // Timeout: zero velocity
        RCLCPP_DEBUG(this->get_logger(), "cmd_vel timeout (%.2f s), zeroing velocity", dt_since_cmd);
      }
    }

    // Allocate to thrusters
    std::vector<double> target_rpms = allocator_->allocate(surge, sway, heave, yaw);

    // Update each motor
    for (int i = 0; i < num_thrusters_; ++i) {
      motors_[i].set_target_rpm(target_rpms[i]);
      motors_[i].update(dt_);
    }

    // Publish aggregate MotorRPM message
    auto rpm_msg = motor_control_msgs::msg::MotorRPM();
    rpm_msg.header.stamp = this->now();
    rpm_msg.header.frame_id = "base_link";
    rpm_msg.target_rpm.resize(num_thrusters_);
    rpm_msg.current_rpm.resize(num_thrusters_);
    rpm_msg.thrust_newtons.resize(num_thrusters_);
    rpm_msg.motor_names.resize(num_thrusters_);

    for (int i = 0; i < num_thrusters_; ++i) {
      const auto & state = motors_[i].get_state();
      rpm_msg.target_rpm[i] = state.target_rpm;
      rpm_msg.current_rpm[i] = state.current_rpm;
      rpm_msg.thrust_newtons[i] = state.thrust_newtons;
      rpm_msg.motor_names[i] = state.name;
    }
    rpm_pub_->publish(rpm_msg);

    // Publish per-motor telemetry
    for (int i = 0; i < num_thrusters_; ++i) {
      const auto & state = motors_[i].get_state();

      auto tel_msg = motor_control_msgs::msg::MotorTelemetry();
      tel_msg.header.stamp = rpm_msg.header.stamp;
      tel_msg.header.frame_id = state.name;
      tel_msg.motor_id = static_cast<uint8_t>(state.motor_id);
      tel_msg.motor_name = state.name;
      tel_msg.target_rpm = state.target_rpm;
      tel_msg.current_rpm = state.current_rpm;
      tel_msg.thrust_newtons = state.thrust_newtons;
      tel_msg.voltage = state.voltage;
      tel_msg.current_amps = state.current_amps;
      tel_msg.temperature_celsius = state.temperature;
      tel_msg.is_saturated = state.is_saturated;

      telemetry_pubs_[i]->publish(tel_msg);
    }
  }

  // Members
  int num_thrusters_;
  double dt_;
  bool cmd_received_;

  std::unique_ptr<motor_control_sim::ThrusterAllocator> allocator_;
  std::vector<motor_control_sim::ThrusterMotor> motors_;

  geometry_msgs::msg::Twist latest_cmd_;
  rclcpp::Time last_cmd_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<motor_control_msgs::msg::MotorRPM>::SharedPtr rpm_pub_;
  std::vector<rclcpp::Publisher<motor_control_msgs::msg::MotorTelemetry>::SharedPtr> telemetry_pubs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
