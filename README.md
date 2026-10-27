# Nonlinear Control Lab: MyCobot & IMU Sensor Fusion API

## Chapter 0.0: Introduction
This API was written and designed by Elias Ludviksson for the **Nonlinear Control Lab at Columbia University** under the direction of **Professor Homayoon Beigi**. 

The primary aim of this library is to make the hardware control loop as simple and accessible as possible for researchers and students. By abstracting away the complex multi-threading, serial communications, and Unscented Kalman Filter (UKF) math, the API provides a clean interface: it returns the highly accurate, fused state of the robotic arm and accepts setpoints from the user. 

This allows users to focus entirely on designing and implementing their own top-level control algorithms (such as PID, Model Predictive Control (MPC), Secant methods, etc.) without needing to worry about the underlying hardware synchronization. You can utilize the included `Controllers.hpp` header for pre-built algorithms, or easily write your own.

---

## Chapter 0.1: File Structure & Overview
To prevent overwhelming new users, the repository is split into specific, purpose-built header files. Here is a breakdown of the core files you will interact with or rely on:

*   **`RobotInterface.hpp`**: The main API wrapper. This is the only file you usually need to `#include` in your main program. It manages the background threads, the UKF filter, and safely shares the robot's state with your control algorithms.
*   **`Controllers.hpp`**: Contains the top-level control algorithms designed for the lab. Currently includes a Multi-Dimensional PID controller and an Eigen-based Secant controller.
*   **`MyCobotDirect.hpp`**: The low-level hardware driver. It handles the raw hexadecimal serial communication directly with the Elephant Robotics MyCobot m280 arm.
*   **`ImuSensor.hpp`**: Manages the IMU sensors. It contains the `ImuMultiplexer` for reading the ESP32 serial stream and the `ImuState` class for handling quaternion math and taring.
*   **`port_discovery.hpp`**: A utility script that automatically scans your Linux `/dev/` directory to find and assign the correct USB serial ports for the arm and the IMU multiplexer.

---

## Chapter 0.2: Setup and Requirements
To compile and run code using this API, your system must meet the following requirements:

*   **Operating System**: Linux (Ubuntu/Debian recommended). The API relies on POSIX `<termios.h>` and `<fcntl.h>` for low-level, non-blocking serial communication over `/dev/ttyUSB*` or `/dev/ttyACM*`.
*   **C++ Standard**: C++17 or higher (required for `<filesystem>` and atomic operations).
*   **Eigen Library**: The codebase heavily relies on the **Eigen3** C++ template library for linear algebra, matrix operations, and quaternions. **You must have Eigen installed and linked in your compiler/CMake setup** to use this API.
    *   *Ubuntu install*: `sudo apt-get install libeigen-dev`
    *   *Include path*: Ensure your compiler flags include `-I/usr/include/eigen3`.

---

## Chapter 1: Usage Guide

This chapter walks through how to use the API, starting from a minimal script to a full trajectory test.

### 1.1 The Basics: Start, Move, and Stop
Here is a bare-minimum example. This script initializes the robot, commands it to a specific angle, reads the state, and shuts down safely.

```cpp
#include <iostream>
#include <thread>
#include <chrono>
#include "RobotInterface.hpp"

int main() {
    // 1. Instantiate the API
    RobotInterface robot_api;

    // 2. Start the API (This auto-discovers ports, connects, and spins up background threads)
    if (robot_api.Start() != 0) {
        std::cerr << "Failed to start robot." << std::endl;
        return 1;
    }

    // 3. Command the robot to a setpoint (Angles in degrees, Speed 1-100)
    mycobot::Angles target = {45.0, -20.0, 0.0, 0.0, 0.0, 0.0};
    robot_api.SendAngleCommand(target, 30); 

    // 4. Wait a moment for the arm to move
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // 5. Read the fully fused state
    RobotState state = robot_api.GetState();
    std::cout << "Current X Position: " << state.cartesian_pos.x() << " meters\n";

    // 6. Safely stop the threads and hardware
    robot_api.Stop();
    return 0;
}
```

### 1.2 Advanced Example: Trajectory Execution
Here is a more comprehensive example (based on `260813_ri_test.cpp`) that homes the robot, runs through a pre-defined grid of pan/tilt angles, and prints the UKF-filtered data.

```cpp
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <iomanip>
#include "RobotInterface.hpp"

int main() {
    RobotInterface robot_api;
    std::cout << "Starting Robot Interface...\n";

    if (robot_api.Start() != 0) {
        std::cerr << "Failed to start RobotInterface. Exiting.\n";
        return 1;
    }

    std::cout << "Homing robot...\n";
    mycobot::Angles home_target = {0, 0, 0, 0, 0, 0};
    robot_api.SendAngleCommand(home_target, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));

    // Define a trajectory path
    std::vector<double> pan_angles = {0.0, 30.0, 60.0, 90.0, 120.0};
    std::vector<double> tilt_angles = {0.0, 30.0, 60.0};

    // Execute Trajectory
    for (double pan : pan_angles) {
        for (double tilt : tilt_angles) {
            mycobot::Angles target = {pan, tilt, 0, 0, 0, 0};
            
            // Queue the command asynchronously
            robot_api.SendAngleCommand(target, 20);
            
            // Wait for movement to finish and the UKF filter to settle
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            
            // Fetch the fully synchronized state from the interface
            RobotState state = robot_api.GetState();
            
            // Print the output format for verification
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "\nTarget [J1, J2]: [" << std::setw(6) << pan << ", " << std::setw(6) << tilt << "]\n";
            std::cout << "  Raw Encoders: [" << state.raw_encoders[0] << ", " << state.raw_encoders[1] << "]\n";
            std::cout << "  UKF Angles:   [" << state.ukf_angles[0] << ", " << state.ukf_angles[1] << "]\n";
            std::cout << "  Cartesian Z:  " << state.cartesian_pos.z() << " meters\n";
        }
    }

    std::cout << "\nTest complete. Shutting down...\n";
    robot_api.Stop();
    return 0;
}
```

### 1.3 Using Top-Level Controllers
To use a custom controller from `Controllers.hpp` (like the PID controller), you simply instantiate it, initialize it with your system's dimensions, and call it in a loop:

```cpp
#include "Controllers.hpp"

// ... inside your control loop ...
// Create a 6-DOF PID controller (K, Ti, Td, N, b, Tt, dt)
pid_controller my_pid(1.5, 1.0, 0.1, 10.0, 1.0, 0.5, 0.01);

// Initialize with current measurements, setpoints, and joint limits
std::vector<double> current_meas = {state.ukf_angles[0], /* ... other joints */};
std::vector<double> setpoints = {90.0, /* ... other joints */};
std::vector<controller::min_max> limits = {{-160, 160}, /* ... */};

my_pid.init_system(current_meas, setpoints, limits);

// In your loop: Update measurements, compute, and send outputs to robot
my_pid.update_system(current_meas);
my_pid.compute();
std::vector<double> outputs = my_pid.getOutputs();
```

---

## Chapter 2: System Architecture & Threading Design

![System Architecture](Untitled Diagram-Page-3.drawio (1)_2.png)

Because serial reading is inherently blocking, and UKF math requires precise timing, the system is designed around a multi-threaded architecture. Calling `RobotInterface::Start()` kicks off a carefully orchestrated sequence:

1.  **Hardware Auto-Discovery**: `autoDiscoverDevices()` scans `/dev/` to automatically assign the arm and IMU ports.
2.  **IMU Thread (Thread 2)**: `imuMultiplexer::Start()` spawns a dedicated `ThreadLoop` running continuously. It opens the serial port, reads incoming byte chunks, buffers them, and hunts for the `0xAA, 0xBB` header. Once found, it validates the checksum and atomically updates the IMU quaternion data in memory.
3.  **IO Thread (Thread 3 - 20Hz)**: `EncoderCommandThread()` operates on a 50ms loop. It is responsible for safe communication with the physical arm. It checks if you have queued a new `MotorCommand` and sends it. It then reads the physical joint encoders from the arm and locks a mutex to safely share this raw data.
4.  **Sensor Fusion Thread (Thread 4 - 100Hz)**: `ImuUkfThread()` is the brain of the API. It runs on a strict 10ms loop (100Hz):
    *   **Data Fetching**: Safely locks and copies the latest encoder data and IMU quaternions.
    *   **Velocity Estimation**: If a command is active, it estimates the required joint velocities to reach the target smoothly.
    *   **UKF Prediction & Update**: It uses the Unscented Kalman Filter (`ukf_filter_.Predict()` and `UpdateIMU() / UpdateEncoders()`) to fuse the noisy physical encoders with the fast IMU data, resulting in a highly accurate state estimation.
    *   **Kinematics & State Publishing**: It runs Forward Kinematics on the filtered angles to determine 3D Cartesian position, then locks the global `state_mutex_` to publish `current_state_` for the user's main thread to read via `GetState()`.

By isolating the slow serial IO and the fast math into separate background threads, your main program (Thread 1) remains completely unblocked, allowing you to run complex control algorithms at whatever speed you desire.

---

## Chapter 3: Future Work & Roadmap

While the current implementation provides a robust, dual-IMU sensor fusion setup strictly for the stationary Elephant Robotics arm, active development is underway to expand the scope of the project. 

Currently in work is the addition of a **3rd IMU** to the state vector. The overarching goal is to mount the robotic arm onto a mobile platform (such as an AGV or rover). The third IMU will track the orientation and disturbances of the mobile base itself, allowing the nonlinear control algorithms to dynamically compensate for the base's movement and maintain perfect end-effector stability in global Cartesian space.
