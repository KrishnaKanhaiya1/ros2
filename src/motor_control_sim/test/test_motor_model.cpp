// Copyright 2026 Motor Control Sim Authors
// Licensed under Apache-2.0

#include <gtest/gtest.h>
#include "motor_control_sim/motor_model.hpp"
#include "motor_control_sim/thruster_allocator.hpp"

using namespace motor_control_sim;

TEST(MotorParamsTest, DefaultValues)
{
  MotorParams p;
  EXPECT_DOUBLE_EQ(p.max_rpm, 3000.0);
  EXPECT_DOUBLE_EQ(p.min_rpm, -3000.0);
  EXPECT_DOUBLE_EQ(p.time_constant, 0.3);
  EXPECT_DOUBLE_EQ(p.dead_band_rpm, 50.0);
  EXPECT_DOUBLE_EQ(p.thrust_coefficient, 0.0005);
  EXPECT_DOUBLE_EQ(p.nominal_voltage, 16.0);
}

TEST(ThrusterMotorTest, InitialState)
{
  ThrusterMotor motor(1, "thruster_FL");
  const auto & state = motor.get_state();

  EXPECT_EQ(state.motor_id, 1);
  EXPECT_EQ(state.name, "thruster_FL");
  EXPECT_DOUBLE_EQ(state.target_rpm, 0.0);
  EXPECT_DOUBLE_EQ(state.current_rpm, 0.0);
  EXPECT_DOUBLE_EQ(state.thrust_newtons, 0.0);
  EXPECT_DOUBLE_EQ(state.temperature, 25.0);
  EXPECT_FALSE(state.is_saturated);
}

TEST(ThrusterMotorTest, SetTargetRpmNormal)
{
  ThrusterMotor motor(0, "test_motor");
  motor.set_target_rpm(1500.0);

  EXPECT_DOUBLE_EQ(motor.get_state().target_rpm, 1500.0);
  EXPECT_FALSE(motor.get_state().is_saturated);
}

TEST(ThrusterMotorTest, SaturationClamping)
{
  ThrusterMotor motor(0, "test_motor");
  motor.set_target_rpm(4500.0);

  EXPECT_DOUBLE_EQ(motor.get_state().target_rpm, 3000.0);
  EXPECT_TRUE(motor.get_state().is_saturated);

  motor.set_target_rpm(-4000.0);
  EXPECT_DOUBLE_EQ(motor.get_state().target_rpm, -3000.0);
  EXPECT_TRUE(motor.get_state().is_saturated);
}

TEST(ThrusterMotorTest, DeadBandFiltering)
{
  ThrusterMotor motor(0, "test_motor");
  motor.set_target_rpm(30.0); // Below 50.0 RPM threshold

  EXPECT_DOUBLE_EQ(motor.get_state().target_rpm, 0.0);
  EXPECT_FALSE(motor.get_state().is_saturated);
}

TEST(ThrusterMotorTest, DynamicsConvergence)
{
  ThrusterMotor motor(0, "test_motor");
  motor.set_target_rpm(2000.0);

  // Run 100 steps of 20ms = 2.0s (over 6x time constant tau=0.3s)
  for (int i = 0; i < 100; ++i) {
    motor.update(0.02);
  }

  EXPECT_NEAR(motor.get_state().current_rpm, 2000.0, 5.0);
  EXPECT_GT(motor.get_state().thrust_newtons, 0.0);
  EXPECT_GT(motor.get_state().voltage, 0.0);
}

TEST(ThrusterMotorTest, ThrustDirectionAndQuadraticLaw)
{
  ThrusterMotor m_fwd(0, "fwd");
  ThrusterMotor m_rev(1, "rev");

  m_fwd.set_target_rpm(1000.0);
  m_rev.set_target_rpm(-1000.0);

  for (int i = 0; i < 50; ++i) {
    m_fwd.update(0.02);
    m_rev.update(0.02);
  }

  EXPECT_GT(m_fwd.get_state().thrust_newtons, 0.0);
  EXPECT_LT(m_rev.get_state().thrust_newtons, 0.0);
  EXPECT_NEAR(m_fwd.get_state().thrust_newtons, -m_rev.get_state().thrust_newtons, 1e-4);
}

TEST(ThrusterAllocatorTest, PureSurgeCommand)
{
  ThrusterAllocator allocator(3000.0, 1000.0);
  // surge = 1.0, sway = 0, heave = 0, yaw = 0
  auto rpms = allocator.allocate(1.0, 0.0, 0.0, 0.0);

  ASSERT_EQ(rpms.size(), 6u);
  // Front thrusters (FL, FR) should drive forward (positive)
  EXPECT_GT(rpms[0], 0.0);
  EXPECT_GT(rpms[1], 0.0);
  // Rear thrusters (RL, RR) have negative coefficients in allocation matrix
  EXPECT_LT(rpms[2], 0.0);
  EXPECT_LT(rpms[3], 0.0);
  // Vertical thrusters (VL, VR) must be zero
  EXPECT_DOUBLE_EQ(rpms[4], 0.0);
  EXPECT_DOUBLE_EQ(rpms[5], 0.0);
}

TEST(ThrusterAllocatorTest, PureHeaveCommand)
{
  ThrusterAllocator allocator(3000.0, 1000.0);
  // surge = 0, sway = 0, heave = 1.5, yaw = 0
  auto rpms = allocator.allocate(0.0, 0.0, 1.5, 0.0);

  ASSERT_EQ(rpms.size(), 6u);
  // Horizontal thrusters must be zero
  EXPECT_DOUBLE_EQ(rpms[0], 0.0);
  EXPECT_DOUBLE_EQ(rpms[1], 0.0);
  EXPECT_DOUBLE_EQ(rpms[2], 0.0);
  EXPECT_DOUBLE_EQ(rpms[3], 0.0);
  // Vertical thrusters equal positive output
  EXPECT_NEAR(rpms[4], 1500.0, 1e-3);
  EXPECT_NEAR(rpms[5], 1500.0, 1e-3);
}

TEST(ThrusterAllocatorTest, RPMNormalization)
{
  ThrusterAllocator allocator(3000.0, 1000.0);
  // Very large velocity inputs that would exceed 3000 RPM
  auto rpms = allocator.allocate(10.0, 5.0, 10.0, 5.0);

  for (double rpm : rpms) {
    EXPECT_LE(std::abs(rpm), 3000.0001);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
