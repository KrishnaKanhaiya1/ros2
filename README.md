# Underwater ROV Motor Control Simulator & ROS 2 Bridge (C++)

![ROS 2](https://img.shields.io/badge/ROS_2-Humble%20%7C%20Iron%20%7C%20Jazzy-22314E?logo=ros&logoColor=white)
![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=c%2B%2B&logoColor=white)
![Build](https://img.shields.io/badge/Build-ament__cmake-blue)
![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)
![Tests](https://img.shields.io/badge/Tests-GoogleTest%20Passed-brightgreen)

**Candidate / Author:** Krishna Kanhaiya ([@KrishnaKanhaiya1](https://github.com/KrishnaKanhaiya1))  
**Repository:** [KrishnaKanhaiya1/ros2](https://github.com/KrishnaKanhaiya1/ros2)

---

A high-performance C++ ROS 2 package suite simulating brushless DC motor firmware dynamics and telemetry streaming for underwater remotely operated vehicles (ROVs) and autonomous underwater vehicles (AUVs).

---

## 1. Overview & Architecture

This repository fulfills the underwater robotics development requirements:
1. **Subscribes** to high-level velocity commands (`/cmd_vel`, `geometry_msgs/msg/Twist`).
2. **Allocates** 4-DOF velocity vectors (Surge, Sway, Heave, Yaw) to individual motor RPM commands via a 6-thruster geometric allocation matrix.
3. **Simulates** brushless DC motor physics using first-order lag response, RPM deadband, saturation limits, quadratic hydrodynamic thrust, back-EMF electrical models, and thermal dissipation.
4. **Publishes** real-time motor telemetry back to ROS 2 topics:
   - Aggregate fleet telemetry on `/motor/rpm` (`motor_control_msgs/msg/MotorRPM`).
   - Detailed individual motor telemetry on `/motor/{thruster_name}/telemetry` (`motor_control_msgs/msg/MotorTelemetry`).

### Data Flow Architecture

```mermaid
flowchart TD
    User["Navigation Controller / Joystick"] -->|/cmd_vel (geometry_msgs/Twist)| Node["MotorSimulatorNode (C++)"]
    TestGen["CmdVelPublisherNode (C++)"] -.->|/cmd_vel (Optional Test Stream)| Node

    subgraph Core Firmware Simulator
        Node --> Alloc["ThrusterAllocator (6-DOF Mapping Matrix)"]
        Alloc --> M1["ThrusterMotor: thruster_FL"]
        Alloc --> M2["ThrusterMotor: thruster_FR"]
        Alloc --> M3["ThrusterMotor: thruster_RL"]
        Alloc --> M4["ThrusterMotor: thruster_RR"]
        Alloc --> M5["ThrusterMotor: thruster_VL"]
        Alloc --> M6["ThrusterMotor: thruster_VR"]
    end

    M1 & M2 & M3 & M4 & M5 & M6 --> PubAgg["Aggregate Telemetry Publisher"]
    M1 & M2 & M3 & M4 & M5 & M6 --> PubPer["Per-Thruster Telemetry Publishers"]

    PubAgg -->|/motor/rpm| TopicRPM["MotorRPM.msg (All Thrusters)"]
    PubPer -->|/motor/thruster_*/telemetry| TopicTel["MotorTelemetry.msg (Electrical, Thermal, Saturation)"]
```

---

## 2. 6-Thruster ROV Geometry & Allocation

The ROV employs standard vectored thruster geometry:
- **4 Horizontal Thrusters** angled at 45° to provide coupled planar control (Surge, Sway, Yaw).
- **2 Vertical Thrusters** dedicated to depth control (Heave).

```
                  FRONT (Bow)
                     ^ +x
                     |
         T1 \                 / T2
      (+45 deg)             (-45 deg)
             \               /
               +-----------+
  +y <-------  |   ROV     |   (Right / Starboard)
  (Port)       | T5     T6 |   (Vertical Thrusters)
               +-----------+
             /               \
         T3 /                 \ T4
     (-135 deg)            (+135 deg)
                     |
                     v
                  REAR (Stern)
```

### Thruster Allocation Matrix

$$\mathbf{\Omega}_{\text{cmd}} = \text{scale} \cdot \mathbf{A} \cdot \begin{bmatrix} v_x \\ v_y \\ v_z \\ \omega_z \end{bmatrix}$$

$$\mathbf{A} = \begin{bmatrix}
0.707 & 0.707 & 0.0 & -0.5 \\
0.707 & -0.707 & 0.0 & 0.5 \\
-0.707 & -0.707 & 0.0 & -0.5 \\
-0.707 & 0.707 & 0.0 & 0.5 \\
0.0 & 0.0 & 1.0 & 0.0 \\
0.0 & 0.0 & 1.0 & 0.0
\end{bmatrix}$$

If any commanded RPM exceeds $\pm \text{max\_rpm}$, the entire vector is normalized to preserve the net force vector direction without clipping asymmetry.

---

## 3. Mathematical Motor Dynamics Model

Each thruster motor firmware simulation implements:

1. **Deadband & Hard Saturation:**
   $$\Omega_{\text{target}} = \begin{cases} 0 & \text{if } |\Omega_{\text{in}}| < \text{dead\_band} \\ \text{clamp}(\Omega_{\text{in}}, -\Omega_{\max}, \Omega_{\max}) & \text{otherwise} \end{cases}$$

2. **First-Order Transient Dynamics (Low-Pass Filter):**
   $$\tau \frac{d\Omega}{dt} + \Omega = \Omega_{\text{target}}$$
   Discretized via Euler numerical integration with step size $\Delta t$:
   $$\Omega_{k+1} = \Omega_k + \min\left(1.0, \frac{\Delta t}{\tau}\right) (\Omega_{\text{target}} - \Omega_k)$$

3. **Hydrodynamic Thrust (Quadratic Law):**
   $$F_{\text{thrust}} = K_t \cdot |\Omega| \cdot \Omega \quad [\text{N}]$$

4. **Electrical Back-EMF & Current:**
   $$V_{\text{emf}} = |\Omega| \cdot K_v, \quad I = \max\left(0, \frac{V_{\text{nominal}} - V_{\text{emf}}}{R}\right) \quad [\text{A}]$$

5. **Thermal Dissipation:**
   $$T = T_{\text{ambient}} + R_{\text{thermal}} \cdot (I^2 \cdot R) \quad [^\circ\text{C}]$$

---

## 4. Package Structure

```
ROS2/
├── README.md
└── src/
    ├── motor_control_msgs/               # Custom ROS 2 Interface Package (C++)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/
    │       ├── MotorRPM.msg             # Multi-thruster aggregate telemetry
    │       └── MotorTelemetry.msg       # Single-thruster detailed diagnostics
    │
    └── motor_control_sim/                # Core Simulator Package (C++17)
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/
        │   └── motor_params.yaml        # Tunable parameters (PID, limits, rates)
        ├── launch/
        │   └── motor_sim.launch.py      # Multi-node launch file
        ├── include/motor_control_sim/
        │   ├── motor_model.hpp          # MotorParams, MotorState, ThrusterMotor
        │   └── thruster_allocator.hpp   # ThrusterAllocator matrix & normalizer
        ├── src/
        │   ├── motor_simulator_node.cpp # Main ROS 2 simulation & bridge node
        │   └── cmd_vel_publisher.cpp    # Test velocity profile generator
        └── test/
            └── test_motor_model.cpp     # GoogleTest unit verification suite
```

---

## 5. ROS 2 Interfaces & Topics

### Published Topics

| Topic | Message Type | Rate | Description |
|---|---|---|---|
| `/motor/rpm` | `motor_control_msgs/msg/MotorRPM` | 50 Hz | System-wide target/current RPM and thrust vectors |
| `/motor/thruster_FL/telemetry` | `motor_control_msgs/msg/MotorTelemetry` | 50 Hz | Front-Left motor voltage, current, temp, saturation |
| `/motor/thruster_FR/telemetry` | `motor_control_msgs/msg/MotorTelemetry` | 50 Hz | Front-Right motor telemetry |
| `/motor/thruster_RL/telemetry` | `motor_control_msgs/msg/MotorTelemetry` | 50 Hz | Rear-Left motor telemetry |
| `/motor/thruster_RR/telemetry` | `motor_control_msgs/msg/MotorTelemetry` | 50 Hz | Rear-Right motor telemetry |
| `/motor/thruster_VL/telemetry` | `motor_control_msgs/msg/MotorTelemetry` | 50 Hz | Vertical-Left motor telemetry |
| `/motor/thruster_VR/telemetry` | `motor_control_msgs/msg/MotorTelemetry` | 50 Hz | Vertical-Right motor telemetry |

### Subscribed Topics

| Topic | Message Type | Timeout | Description |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 1.0 s | Commanded linear (x, y, z) & angular (yaw) velocity. Auto-zeros if inactive. |

---

## 6. Build & Execution Guide

### Prerequisites
- ROS 2 (Humble, Iron, or Jazzy)
- C++17 compiler (`g++`, `clang++`, or MSVC)
- CMake 3.8+
- `colcon` build tool

### Step 1: Build the Workspace

```bash
cd c:\Users\HP\Desktop\ROS2
colcon build --symlink-install
```

### Step 2: Source Workspace Overlay

**Linux / Bash:**
```bash
source install/setup.bash
```

**Windows / PowerShell:**
```powershell
.\install\setup.ps1
```

### Step 3: Launch Full Simulator with Test Excitation

```bash
ros2 launch motor_control_sim motor_sim.launch.py
```

To run without the test publisher (e.g., when interfacing with your own teleop or autonomous stack):
```bash
ros2 launch motor_control_sim motor_sim.launch.py use_test_publisher:=false
```

### Step 4: Inspect Telemetry Topics

In a separate terminal:
```bash
# Echo aggregate RPM across all 6 thrusters
ros2 topic echo /motor/rpm

# Echo detailed electrical and thermal metrics for Front-Left thruster
ros2 topic echo /motor/thruster_FL/telemetry
```

### Step 5: Send Manual Velocity Commands

```bash
# Command forward surge at 1.2 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 10

# Command combined heave and yaw
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.8}, angular: {z: 0.5}}" -r 10
```

---

## 7. Unit Testing

### Standalone C++ System Verification (No ROS 2 Required)

A standalone verification executable `test_motor_system.exe` is provided directly in the root workspace to validate the complete firmware simulator, thruster allocation matrix, dynamics convergence, and telemetry stream:

```powershell
# Run the pre-compiled verification test
.\test_motor_system.exe

# Or re-compile and run using MinGW g++ / Clang / GCC
g++ -std=c++14 -O2 -I src/motor_control_sim/include test_motor_system.cpp -o test_motor_system.exe
.\test_motor_system.exe
```

This test program validates:
1. Physical parameter defaults
2. First-order transient step response and convergence ($1\tau \approx 63.2\%$, $5\tau > 99\%$)
3. Deadband filtering and saturation clamping
4. Directional quadratic hydrodynamic thrust ($F = K_t |\Omega| \Omega$)
5. 6-DOF to 6-thruster geometric allocation (pure surge, pure heave, normalizer)
6. Dynamic flight mission telemetry streaming over time with command timeout auto-zeroing

```bash
colcon test --packages-select motor_control_sim
colcon test-result --all --verbose
```

Tests cover:
- Parameter initialization defaults
- Thruster allocation geometry (pure surge, pure heave, yaw decoupling)
- First-order lag convergence and time constant compliance
- Saturation clamping and saturation status reporting
- Low-speed deadband rejection
- Directional quadratic thrust scaling
