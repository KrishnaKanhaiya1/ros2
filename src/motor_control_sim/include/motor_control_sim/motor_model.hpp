#ifndef MOTOR_CONTROL_SIM__MOTOR_MODEL_HPP_
#define MOTOR_CONTROL_SIM__MOTOR_MODEL_HPP_

#include <string>
#include <cmath>
#include <algorithm>

namespace motor_control_sim
{

/// Physical parameters for a brushless DC thruster motor.
struct MotorParams
{
  double max_rpm = 3000.0;
  double min_rpm = -3000.0;
  double time_constant = 0.3;        // First-order response (seconds)
  double dead_band_rpm = 50.0;
  double thrust_coefficient = 0.0005; // F = Kt * |rpm| * rpm
  double voltage_constant = 0.004;    // V per RPM (back-EMF)
  double resistance = 0.5;            // Winding resistance (ohms)
  double nominal_voltage = 16.0;
  double torque_constant = 0.01;      // Nm per amp
  double thermal_resistance = 0.5;    // deg-C per watt
  double ambient_temp = 25.0;
  double max_temp = 85.0;
};

/// Runtime state of a single motor.
struct MotorState
{
  int motor_id = 0;
  std::string name = "motor";
  double target_rpm = 0.0;
  double current_rpm = 0.0;
  double thrust_newtons = 0.0;
  double voltage = 0.0;
  double current_amps = 0.0;
  double temperature = 25.0;
  bool is_saturated = false;
};

/// Simulates a single brushless thruster motor with first-order dynamics.
class ThrusterMotor
{
public:
  ThrusterMotor(int motor_id, const std::string & name, const MotorParams & params = MotorParams())
  : params_(params)
  {
    state_.motor_id = motor_id;
    state_.name = name;
    state_.temperature = params_.ambient_temp;
  }

  /// Set desired RPM, applying dead band and saturation.
  void set_target_rpm(double rpm)
  {
    // Dead band
    if (std::abs(rpm) < params_.dead_band_rpm) {
      rpm = 0.0;
    }
    // Clamp to limits
    double clamped = (rpm < params_.min_rpm) ? params_.min_rpm : ((rpm > params_.max_rpm) ? params_.max_rpm : rpm);
    state_.is_saturated = (clamped != rpm);
    state_.target_rpm = clamped;
  }

  /// Advance simulation by dt seconds.
  /// First-order: rpm[k+1] = rpm[k] + (dt/tau) * (target - rpm[k])
  void update(double dt)
  {
    double tau = params_.time_constant;
    if (tau > 0.0 && dt > 0.0) {
      double alpha = std::min(dt / tau, 1.0);  // clamp for stability
      state_.current_rpm += alpha * (state_.target_rpm - state_.current_rpm);
    } else {
      state_.current_rpm = state_.target_rpm;
    }

    // Dead band on output
    if (std::abs(state_.current_rpm) < params_.dead_band_rpm * 0.5) {
      state_.current_rpm = 0.0;
    }

    // Thrust: F = Kt * |rpm| * rpm  (quadratic, direction-preserving)
    double rpm = state_.current_rpm;
    state_.thrust_newtons = params_.thrust_coefficient * std::abs(rpm) * rpm;

    // Electrical model
    double back_emf = std::abs(rpm) * params_.voltage_constant;
    state_.voltage = std::min(back_emf + 2.0, params_.nominal_voltage);
    if (params_.resistance > 0.0) {
      state_.current_amps = (state_.voltage - back_emf) / params_.resistance;
    } else {
      state_.current_amps = 0.0;
    }
    state_.current_amps = std::max(0.0, state_.current_amps);

    // Thermal model
    double power = state_.current_amps * state_.current_amps * params_.resistance;
    state_.temperature = params_.ambient_temp + params_.thermal_resistance * power;
  }

  const MotorState & get_state() const { return state_; }
  const MotorParams & get_params() const { return params_; }

private:
  MotorParams params_;
  MotorState state_;
};

}  // namespace motor_control_sim

#endif  // MOTOR_CONTROL_SIM__MOTOR_MODEL_HPP_
