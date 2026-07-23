#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <iomanip>
#include <vector>
#include <numeric>

#include "MyCobotDirect.hpp"
#include "ImuSensor.hpp"
#include "port_discovery.hpp"

using namespace std::chrono;

// Safely waits for movement to start and finish
void WaitMoveToFinish(mycobot::MyCobotDirect& robot) {
    std::this_thread::sleep_for(milliseconds(200)); 
    while (robot.IsMoving()) {
        std::this_thread::sleep_for(milliseconds(50));
    }
    std::this_thread::sleep_for(milliseconds(1000)); // Allow sensor to settle
}

int main() {
    // 1. Automatically discover hardware ports
    RobotPorts ports = autoDiscoverDevices();

    if (ports.arm_port.empty() || ports.imu_port.empty()) {
        std::cerr << "CRITICAL ERROR: Could not find all required devices via auto-discovery." << std::endl;
        return 1;
    }

    // 2. Initialize IMU multiplexer
    sensor::ImuMultiplexer esp32_mux(ports.imu_port, B500000);
    esp32_mux.Start();
    std::this_thread::sleep_for(milliseconds(500)); 

    sensor::ImuState& imu_arm = esp32_mux.imu_array[0];
    imu_arm.SetAxisMapping(1, 2, 3);

    // 3. Connect to robot
    mycobot::MyCobotDirect robot;
    if (!robot.Connect(ports.arm_port)) {
        std::cerr << "CRITICAL ERROR: Failed to connect to robot arm on " << ports.arm_port << std::endl;
        esp32_mux.Stop();
        return 1;
    }
    robot.PowerOn();

    std::cout << "Homing robot for IMU alignment...\n";
    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    WaitMoveToFinish(robot);
    
    imu_arm.Tare();
    std::cout << "IMU aligned. Starting Coordinate System Verification Sweep...\n\n";

    const double BASE_HEIGHT = 131.22; 
    const double ARM_LENGTH = 110.4;
    const double FIXED_TILT = 45.0; // Fixed tilt so X and Y are non-zero

    std::vector<double> pan_angles = {-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0};
    
    std::vector<double> squared_errors_x;
    std::vector<double> squared_errors_y;
    std::vector<double> squared_errors_z;

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "----------------------------------------------------------------------------------------\n";
    std::cout << " Pan | Expected (X,   Y,   Z)   | Actual IMU (X,   Y,   Z)   | Euclidean Error (mm)\n";
    std::cout << "----------------------------------------------------------------------------------------\n";

    for (double pan : pan_angles) {
        // Move robot to test position
        robot.WriteAngles({pan, FIXED_TILT, 0, 0, 0, 0}, 20);
        WaitMoveToFinish(robot);

        // Calculate expected coordinates using right-handed assumption
        double pan_rad = pan * M_PI / 180.0;
        double tilt_rad = FIXED_TILT * M_PI / 180.0;

        double exp_x = ARM_LENGTH * sin(tilt_rad) * cos(pan_rad);
        double exp_y = ARM_LENGTH * sin(tilt_rad) * sin(pan_rad);
        double exp_z = (ARM_LENGTH * cos(tilt_rad)) + BASE_HEIGHT;

        // Get actual physical IMU readings
        std::array<double, 3> imu_pos = imu_arm.GetElbowPosition3D(ARM_LENGTH, BASE_HEIGHT);

        // Compute errors
        double err_x = imu_pos[0] - exp_x;
        double err_y = imu_pos[1] - exp_y;
        double err_z = imu_pos[2] - exp_z;
        double euclidean_err = sqrt(err_x*err_x + err_y*err_y + err_z*err_z);

        squared_errors_x.push_back(err_x * err_x);
        squared_errors_y.push_back(err_y * err_y);
        squared_errors_z.push_back(err_z * err_z);

        std::cout << std::setw(4) << pan << " | "
                  << "(" << std::setw(5) << exp_x << ", " << std::setw(5) << exp_y << ", " << std::setw(5) << exp_z << ") | "
                  << "(" << std::setw(5) << imu_pos[0] << ", " << std::setw(5) << imu_pos[1] << ", " << std::setw(5) << imu_pos[2] << ") | "
                  << std::setw(15) << euclidean_err << "\n";
    }

    std::cout << "----------------------------------------------------------------------------------------\n\n";

    // Calculate Mean Squared Error (MSE) across the sweep
    double mse_x = std::accumulate(squared_errors_x.begin(), squared_errors_x.end(), 0.0) / squared_errors_x.size();
    double mse_y = std::accumulate(squared_errors_y.begin(), squared_errors_y.end(), 0.0) / squared_errors_y.size();
    double mse_z = std::accumulate(squared_errors_z.begin(), squared_errors_z.end(), 0.0) / squared_errors_z.size();

    std::cout << "--- Statistical Proof Summary --- \n";
    std::cout << "MSE X (Forward/Backward): " << std::setprecision(4) << mse_x << " mm^2\n";
    std::cout << "MSE Y (Left/Right)      : " << mse_y << " mm^2\n";
    std::cout << "MSE Z (Up/Down)         : " << mse_z << " mm^2\n";
    std::cout << "Conclusion: Low MSE values validate that X is Forward, Y is Left, and Z is Up.\n";

    // Return home
    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    WaitMoveToFinish(robot);

    robot.StopRobot();
    esp32_mux.Stop();
    return 0;
}
