# Submarine Driver Framework

A modular **C++ / ROS2 driver framework** for underwater vehicle actuators and sensors, designed for autonomous submarine development and software-in-the-loop (SIL) testing.

## Overview

This framework provides hardware abstraction drivers for underwater vehicle components with configurable noise injection, enabling realistic simulation and testing of autonomy algorithms without physical hardware.

### Key Features

- **Thruster Driver** — PWM control with quadratic thrust curves, deadband compensation, force saturation, and thermal protection modeling (Blue Robotics T200/T500 compatible)
- **IMU Driver** — 6-axis accelerometer + gyroscope simulation with per-axis Gaussian noise, bias drift, calibration offsets, and scale factor corrections
- **DVL Driver** — Doppler Velocity Log stub with bottom-track velocity, altitude measurement, and bottom-lock loss simulation based on altitude range
- **Depth Sensor Driver** — Pressure-based depth measurement with temperature compensation for seawater density variations (hydrostatic equation)
- **6-DOF State Estimator** — Simplified EKF fusing IMU (dead-reckoning), DVL (velocity correction), and depth sensor (z-correction) into a unified pose estimate (x, y, z, roll, pitch, yaw)
- **Noise Model** — Configurable noise injection (Gaussian white noise, bias drift random walk, ADC quantization) for SIL testing of autonomy algorithms against realistic sensor degradation

### Architecture

```
┌─────────────────────────────────────────────────────┐
│                    Core Library                      │
│              (Pure C++17, no ROS2 dependency)        │
│                                                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐   │
│  │ Thruster │  │  Sensors │  │  State Estimator │   │
│  │  Driver  │  │ IMU/DVL/ │  │   6-DOF EKF      │   │
│  │  (PWM,   │  │  Depth   │  │  (IMU predict +  │   │
│  │  curves, │  │  (noise  │  │   DVL/depth      │   │
│  │  deadband│  │  inject) │  │   update)        │   │
│  └──────────┘  └──────────┘  └──────────────────┘   │
│       ▲              ▲               ▲               │
│       └──────────────┴───────────────┘               │
│              Abstract Base Classes                   │
│         DriverBase / SensorBase / ActuatorBase        │
└─────────────────────────────────────────────────────┘
                        │
              ┌─────────┴──────────┐
              │  ROS2 Node Wrappers │ (optional)
              │  sensor_driver_node │
              │  state_estimator_   │
              │       node          │
              └────────────────────┘
```

The core library compiles as **pure C++17** with no external dependencies. ROS2 node wrappers are optional and only built when `-DBUILD_ROS2_NODES=ON` is set.

## Build

### Standalone (no ROS2 required)

```bash
mkdir build && cd build
cmake .. -DBUILD_TESTING=ON
cmake --build .
```

### Run Tests

```bash
cd build
ctest --output-on-failure
# or directly:
./submarine_drivers_tests
```

### With ROS2 (Humble/Iron/Jazzy)

```bash
# From your ROS2 workspace src/
cd ~/ros2_ws/src
ln -s /path/to/submarine-driver-framework .
cd ~/ros2_ws
colcon build --packages-select submarine_drivers --cmake-args -DBUILD_ROS2_NODES=ON
source install/setup.bash
ros2 launch submarine_drivers drivers.launch.py
```

## Project Structure

```
submarine-driver-framework/
├── CMakeLists.txt                          # Dual-mode: standalone + ROS2
├── package.xml                             # ROS2 package manifest
├── config/
│   └── vehicle_params.yaml                 # Vehicle + sensor configuration
├── include/submarine_drivers/
│   ├── core/
│   │   ├── types.hpp                       # Vector3, Pose6DOF, sensor readings
│   │   ├── driver_base.hpp                 # Abstract driver lifecycle
│   │   ├── sensor_base.hpp                 # Abstract sensor interface
│   │   ├── actuator_base.hpp               # Abstract actuator interface
│   │   └── noise_model.hpp                 # Gaussian + bias drift + quantization
│   ├── sensors/
│   │   ├── imu_driver.hpp                  # 6-axis IMU with calibration
│   │   ├── dvl_driver.hpp                  # DVL with bottom-lock simulation
│   │   └── depth_sensor_driver.hpp         # Pressure → depth + temp compensation
│   ├── actuators/
│   │   └── thruster_driver.hpp             # PWM, thrust curves, deadband, thermal
│   └── estimation/
│       └── state_estimator_6dof.hpp        # Simplified EKF (12-state)
├── src/
│   ├── core/noise_model.cpp
│   ├── sensors/{imu,dvl,depth_sensor}_driver.cpp
│   ├── actuators/thruster_driver.cpp
│   ├── estimation/state_estimator_6dof.cpp
│   └── nodes/                              # ROS2 wrappers (conditional)
│       ├── sensor_driver_node.cpp
│       └── state_estimator_node.cpp
├── test/
│   ├── test_noise_model.cpp                # 8 tests
│   ├── test_thruster_driver.cpp            # 12 tests
│   ├── test_imu_driver.cpp                 # 6 tests
│   ├── test_dvl_driver.cpp                 # 6 tests
│   ├── test_depth_sensor.cpp               # 7 tests
│   └── test_state_estimator.cpp            # 11 tests
└── launch/
    └── drivers.launch.py                   # ROS2 launch file
```

## Design Principles

### Abstract Base Classes
All drivers inherit from `DriverBase` which defines the lifecycle: `initialize()` → `update(dt)` → `shutdown()`. Sensors extend `SensorBase` (adds reading/timestamp/validity). Actuators extend `ActuatorBase` (adds command/feedback/safety).

### RAII & Modern C++
- `std::unique_ptr` / `std::shared_ptr` for resource management
- No raw `new`/`delete` anywhere
- `std::chrono` for timestamps
- `enum class` for type-safe status codes
- Rule of Five (move semantics, deleted copy constructors)

### Noise Injection for SIL Testing
The `NoiseModel` class enables testing autonomy algorithms against realistic sensor degradation:
- **Gaussian white noise** — models sensor measurement noise
- **Bias drift** — random walk simulating thermal drift and sensor aging
- **Quantization** — simulates ADC resolution limits

Each sensor driver accepts a `NoiseModel::Config` for independent per-axis noise configuration.

### 6-DOF State Estimation
The state estimator maintains a 12-element state vector `[x,y,z, roll,pitch,yaw, u,v,w, p,q,r]` using a simplified scalar Kalman filter:
- **Prediction**: IMU-driven dead reckoning with Euler kinematic equations and gravity compensation
- **DVL Update**: Body-frame velocity correction
- **Depth Update**: Z-position correction
- **Covariance**: Diagonal approximation (grows on predict, shrinks on update)

## Testing

50 GTest unit tests covering all components:

| Test File | Tests | Coverage |
|-----------|-------|----------|
| `test_noise_model.cpp` | 8 | Passthrough, Gaussian stats, quantization, bias drift, vector noise |
| `test_thruster_driver.cpp` | 12 | Safe state, arming, deadband, quadratic curve, PWM round-trip |
| `test_imu_driver.cpp` | 6 | Initialization, gravity, angular velocity, noise, calibration |
| `test_dvl_driver.cpp` | 6 | Bottom lock, altitude limits, lock recovery |
| `test_depth_sensor.cpp` | 7 | Surface/deep depth, pressure, temperature compensation |
| `test_state_estimator.cpp` | 11 | Init, stationary, yaw integration, corrections, covariance |

## Configuration

Vehicle parameters are configured via YAML (`config/vehicle_params.yaml`):
- Thruster layout (position, direction, PWM range, thrust coefficient)
- Sensor noise parameters (Gaussian stddev, bias drift rate/max)
- State estimator process/measurement noise tuning

## License

MIT License — see [LICENSE](LICENSE).

## Author

**Om Pratap Tilwar** — [GitHub](https://github.com/omtilwar) | [LinkedIn](https://linkedin.com/in/omtilwar)
