// Copyright 2026 Motor Control Sim Authors
// Licensed under Apache-2.0
//
// Standalone C++ verification test for Underwater ROV Motor Control Simulator

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <cassert>
#include <string>

#include "motor_control_sim/motor_model.hpp"
#include "motor_control_sim/thruster_allocator.hpp"

using namespace motor_control_sim;

// ANSI color helpers
#define GREEN "\033[32m"
#define BLUE  "\033[34m"
#define CYAN  "\033[36m"
#define RED   "\033[31m"
#define BOLD  "\033[1m"
#define RESET "\033[0m"

void test_motor_params_defaults()
{
  std::cout << BLUE << "[TEST 1] Testing Motor Parameters Defaults..." << RESET << std::endl;
  MotorParams p;
  assert(p.max_rpm == 3000.0);
  assert(p.min_rpm == -3000.0);
  assert(p.time_constant == 0.3);
  assert(p.dead_band_rpm == 50.0);
  assert(p.thrust_coefficient == 0.0005);
  assert(p.nominal_voltage == 16.0);
  std::cout << GREEN << "  --> PASS: All physical parameters correctly initialized." << RESET << std::endl;
}

void test_thruster_motor_dynamics()
{
  std::cout << BLUE << "[TEST 2] Testing Motor Response Behavior Simulation..." << RESET << std::endl;
  ThrusterMotor motor(0, "thruster_FL");

  // Step 1: Check initial idle state
  assert(motor.get_state().current_rpm == 0.0);
  assert(motor.get_state().target_rpm == 0.0);
  assert(motor.get_state().thrust_newtons == 0.0);
  assert(motor.get_state().temperature == 25.0);

  // Step 2: Test deadband rejection
  motor.set_target_rpm(35.0); // Below 50 RPM deadband
  assert(motor.get_state().target_rpm == 0.0);
  assert(!motor.get_state().is_saturated);

  // Step 3: Test saturation clamping
  motor.set_target_rpm(4200.0); // Exceeds 3000 RPM max
  assert(motor.get_state().target_rpm == 3000.0);
  assert(motor.get_state().is_saturated == true);

  // Step 4: Test first-order transient response (simulating 50 Hz discrete updates, dt = 0.02s)
  motor.set_target_rpm(1500.0);
  double dt = 0.02;

  // After 1 time constant (tau = 0.3s -> 15 steps of 0.02s), RPM should reach approx 63.2% of target (~948 RPM)
  for (int step = 0; step < 15; ++step) {
    motor.update(dt);
  }
  double rpm_1tau = motor.get_state().current_rpm;
  std::cout << "  At 1*tau (0.3s): RPM = " << std::fixed << std::setprecision(1) << rpm_1tau 
            << " (Expected ~948 RPM, ~63.2%)" << std::endl;
  assert(std::abs(rpm_1tau - 948.0) < 50.0);

  // After 5 time constants (1.5s -> total 75 steps), motor should settle within 1% of 1500 RPM
  for (int step = 15; step < 75; ++step) {
    motor.update(dt);
  }
  double rpm_settled = motor.get_state().current_rpm;
  std::cout << "  At 5*tau (1.5s): Settled RPM = " << rpm_settled << " (Target: 1500 RPM)" << std::endl;
  assert(std::abs(rpm_settled - 1500.0) < 15.0);

  // Step 5: Test thrust and electrical telemetry calculations
  double thrust = motor.get_state().thrust_newtons;
  double expected_thrust = 0.0005 * 1500.0 * 1500.0; // 1125 N
  std::cout << "  Calculated Thrust: " << thrust << " N (Expected: " << expected_thrust << " N)" << std::endl;
  assert(std::abs(thrust - expected_thrust) < 25.0);

  assert(motor.get_state().voltage > 0.0);
  assert(motor.get_state().current_amps >= 0.0);
  assert(motor.get_state().temperature >= 25.0);

  std::cout << GREEN << "  --> PASS: First-order response, deadband, saturation, thrust, and electrical models verified." << RESET << std::endl;
}

void test_thruster_allocator_kinematics()
{
  std::cout << BLUE << "[TEST 3] Testing 6-DOF to 6-Thruster Allocation Matrix..." << RESET << std::endl;
  ThrusterAllocator allocator(3000.0, 1000.0);

  // Case A: Pure Surge (Forward, vx = 1.0 m/s)
  auto rpms_surge = allocator.allocate(1.0, 0.0, 0.0, 0.0);
  assert(rpms_surge.size() == 6);
  std::cout << "  [Surge 1.0 m/s] FL:" << rpms_surge[0] << " FR:" << rpms_surge[1]
            << " RL:" << rpms_surge[2] << " RR:" << rpms_surge[3]
            << " VL:" << rpms_surge[4] << " VR:" << rpms_surge[5] << std::endl;
  assert(rpms_surge[0] > 500.0);  // FL forward
  assert(rpms_surge[1] > 500.0);  // FR forward
  assert(rpms_surge[2] < -500.0); // RL reverse
  assert(rpms_surge[3] < -500.0); // RR reverse
  assert(rpms_surge[4] == 0.0);   // VL zero
  assert(rpms_surge[5] == 0.0);   // VR zero

  // Case B: Pure Heave (Ascend/Descend, vz = 1.2 m/s)
  auto rpms_heave = allocator.allocate(0.0, 0.0, 1.2, 0.0);
  std::cout << "  [Heave 1.2 m/s] FL:" << rpms_heave[0] << " FR:" << rpms_heave[1]
            << " RL:" << rpms_heave[2] << " RR:" << rpms_heave[3]
            << " VL:" << rpms_heave[4] << " VR:" << rpms_heave[5] << std::endl;
  assert(rpms_heave[0] == 0.0 && rpms_heave[1] == 0.0 && rpms_heave[2] == 0.0 && rpms_heave[3] == 0.0);
  assert(rpms_heave[4] == 1200.0 && rpms_heave[5] == 1200.0);

  // Case C: Extreme input with proportional normalization
  auto rpms_extreme = allocator.allocate(10.0, 5.0, 8.0, 4.0);
  for (double r : rpms_extreme) {
    assert(std::abs(r) <= 3000.001);
  }
  std::cout << GREEN << "  --> PASS: Thruster kinematics and RPM normalizer verified." << RESET << std::endl;
}

void test_full_pipeline_telemetry_stream()
{
  std::cout << BLUE << "[TEST 4] Simulating Full End-to-End ROS 2 Telemetry Pipeline..." << RESET << std::endl;

  ThrusterAllocator allocator(3000.0, 1000.0);
  MotorParams params;
  std::vector<ThrusterMotor> motors;
  for (int i = 0; i < 6; ++i) {
    motors.emplace_back(i, ThrusterAllocator::THRUSTER_NAMES[i], params);
  }

  // Simulated flight trajectory:
  // t = 0.0 to 1.0s: Surge forward at 1.0 m/s
  // t = 1.0 to 2.0s: Add heave 0.8 m/s (climbing)
  // t = 2.0 to 3.0s: Add yaw turn 0.5 rad/s
  // t = 3.0 to 4.0s: Velocity command timeout (auto-zero)

  double dt = 0.02; // 50 Hz simulation loop
  int total_steps = 200; // 4.0 seconds

  std::cout << CYAN << std::string(80, '-') << RESET << std::endl;
  std::cout << BOLD << std::left 
            << std::setw(7) << "Time[s]" 
            << std::setw(18) << "cmd_vel(x,z,yaw)" 
            << std::setw(12) << "T1_FL [RPM]" 
            << std::setw(12) << "T2_FR [RPM]" 
            << std::setw(12) << "T5_VL [RPM]" 
            << std::setw(12) << "T1_Thrust[N]" 
            << std::setw(9)  << "T1_Temp[C]" << RESET << std::endl;
  std::cout << CYAN << std::string(80, '-') << RESET << std::endl;

  for (int step = 0; step <= total_steps; ++step) {
    double t = step * dt;
    double vx = 0.0, vy = 0.0, vz = 0.0, wz = 0.0;

    if (t < 1.0) {
      vx = 1.0;
    } else if (t < 2.0) {
      vx = 1.0; vz = 0.8;
    } else if (t < 3.0) {
      vx = 1.0; vz = 0.8; wz = 0.5;
    } else {
      // Timeout zeroing
      vx = 0.0; vy = 0.0; vz = 0.0; wz = 0.0;
    }

    // Allocation (cmd_vel -> target RPMs)
    auto target_rpms = allocator.allocate(vx, vy, vz, wz);

    // Motor response update
    for (int i = 0; i < 6; ++i) {
      motors[i].set_target_rpm(target_rpms[i]);
      motors[i].update(dt);
    }

    // Print telemetry sample every 0.5 seconds
    if (step % 25 == 0) {
      std::string cmd_str = "(" + std::to_string(vx).substr(0,3) + "," +
                            std::to_string(vz).substr(0,3) + "," +
                            std::to_string(wz).substr(0,3) + ")";
      std::cout << std::left 
                << std::setw(7) << std::fixed << std::setprecision(2) << t
                << std::setw(18) << cmd_str
                << std::setw(12) << std::setprecision(1) << motors[0].get_state().current_rpm
                << std::setw(12) << motors[1].get_state().current_rpm
                << std::setw(12) << motors[4].get_state().current_rpm
                << std::setw(12) << motors[0].get_state().thrust_newtons
                << std::setw(9)  << motors[0].get_state().temperature
                << std::endl;
    }
  }
  std::cout << CYAN << std::string(80, '-') << RESET << std::endl;

  // Final verification: motors settled back to 0 RPM after timeout
  assert(std::abs(motors[0].get_state().current_rpm) < 1.0);
  assert(std::abs(motors[4].get_state().current_rpm) < 1.0);
  std::cout << GREEN << "  --> PASS: Full telemetry stream generation and auto-zero settling confirmed." << RESET << std::endl;
}

int main()
{
  std::cout << BOLD << "============================================================" << std::endl;
  std::cout << " Underwater ROV Motor Control Simulator: Test & Verification" << std::endl;
  std::cout << "============================================================" << RESET << std::endl;

  test_motor_params_defaults();
  test_thruster_motor_dynamics();
  test_thruster_allocator_kinematics();
  test_full_pipeline_telemetry_stream();

  std::cout << BOLD << GREEN << "\nALL 4 COMPREHENSIVE TESTS PASSED SUCCESSFULLY!" << RESET << std::endl;
  return 0;
}
