#ifndef MOTOR_CONTROL_SIM__THRUSTER_ALLOCATOR_HPP_
#define MOTOR_CONTROL_SIM__THRUSTER_ALLOCATOR_HPP_

#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

namespace motor_control_sim
{

/// Maps velocity commands [surge, sway, heave, yaw] to 6 thruster RPM commands.
///
/// 6-thruster underwater ROV configuration:
///   T1 (front-left, 45deg)   T2 (front-right, 45deg)  - horizontal
///   T3 (rear-left, 45deg)    T4 (rear-right, 45deg)   - horizontal
///   T5 (vertical-left)       T6 (vertical-right)      - vertical
class ThrusterAllocator
{
public:
  static constexpr int NUM_THRUSTERS = 6;
  static constexpr int NUM_DOF = 4;  // surge, sway, heave, yaw

  // Thruster names accessor
  struct ThrusterNamesAccessor
  {
    const char * operator[](std::size_t i) const
    {
      static const char * names[NUM_THRUSTERS] = {
        "thruster_FL", "thruster_FR",
        "thruster_RL", "thruster_RR",
        "thruster_VL", "thruster_VR"
      };
      return (i < NUM_THRUSTERS) ? names[i] : "unknown";
    }
  };
  static constexpr ThrusterNamesAccessor THRUSTER_NAMES{};

  // Allocation matrix accessor [thruster][DOF] — maps [surge, sway, heave, yaw] to thrust per motor
  struct AllocationMatrixAccessor
  {
    struct RowAccessor
    {
      int row;
      double operator[](int col) const
      {
        static constexpr double matrix[NUM_THRUSTERS][NUM_DOF] = {
          //  surge    sway    heave   yaw
          {  0.707,  0.707,  0.0,  -0.5},   // T1 front-left
          {  0.707, -0.707,  0.0,   0.5},   // T2 front-right
          { -0.707, -0.707,  0.0,  -0.5},   // T3 rear-left
          { -0.707,  0.707,  0.0,   0.5},   // T4 rear-right
          {  0.0,    0.0,    1.0,   0.0},   // T5 vertical-left
          {  0.0,    0.0,    1.0,   0.0},   // T6 vertical-right
        };
        return matrix[row][col];
      }
    };
    RowAccessor operator[](int row) const { return RowAccessor{row}; }
  };
  static constexpr AllocationMatrixAccessor ALLOCATION_MATRIX{};

  explicit ThrusterAllocator(double max_rpm = 3000.0, double velocity_to_rpm_scale = 1000.0)
  : max_rpm_(max_rpm), scale_(velocity_to_rpm_scale)
  {}

  /// Convert velocity commands to per-thruster RPM commands.
  std::vector<double> allocate(double surge, double sway, double heave, double yaw) const
  {
    std::array<double, NUM_DOF> velocity = {surge, sway, heave, yaw};
    std::vector<double> rpms(NUM_THRUSTERS, 0.0);

    for (int i = 0; i < NUM_THRUSTERS; ++i) {
      double rpm = 0.0;
      for (int j = 0; j < NUM_DOF; ++j) {
        rpm += ALLOCATION_MATRIX[i][j] * velocity[j];
      }
      rpms[i] = rpm * scale_;
    }

    // Normalize if any thruster exceeds max RPM
    double max_abs = 0.0;
    for (const auto & r : rpms) {
      max_abs = std::max(max_abs, std::abs(r));
    }
    if (max_abs > max_rpm_) {
      double factor = max_rpm_ / max_abs;
      for (auto & r : rpms) {
        r *= factor;
      }
    }

    return rpms;
  }

private:
  double max_rpm_;
  double scale_;
};

}  // namespace motor_control_sim

#endif  // MOTOR_CONTROL_SIM__THRUSTER_ALLOCATOR_HPP_
