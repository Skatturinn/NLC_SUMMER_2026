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

// Absolute lock to ensure the arm has physically reached the target before reading
void WaitUntilTargetReached(mycobot::MyCobotDirect& robot, const mycobot::Angles& target) {
    for (int timeout = 0; timeout < 60; ++timeout) { // Max 6 seconds
        mycobot::Angles current = robot.GetAngles();
        bool reached = true;
        for (int j = 0; j < 6; j++) {
            if (std::abs(current[j] - target[j]) > 2.5) { // 2.5 degree tolerance
                reached = false;
                break;
            }
        }
        if (reached) break;
        std::this_thread::sleep_for(milliseconds(100));
    }
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
    
    // Testing the Y-Z axis swap to fix the flat Y coordinate
    imu_arm.SetAxisMapping(1, -3, 2);

    // 3. Connect to robot
    mycobot::MyCobotDirect robot;
    if (!robot.Connect(ports.arm_port)) {
        std::cerr << "CRITICAL ERROR: Failed to connect to robot arm on " << ports.arm_port << std::endl;
        esp32_mux.Stop();
        return 1;
    }
    robot.PowerOn();

    std::cout << "Homing robot for IMU alignment...\n";
    mycobot::Angles home_angles = {0, 0, 0, 0, 0, 0};
    robot.WriteAngles(home_angles, 30);
    
    // Explicit encoder wait for home
    WaitUntilTargetReached(robot, home_angles);
    std::this_thread::sleep_for(milliseconds(1500)); // Hold and settle
    
    imu_arm.Tare();
    std::cout << "IMU aligned. Starting Explicit Step Coordinate Verification (-180 to 180)...\n\n";

    const double BASE_HEIGHT = 131.22; 
    const double ARM_LENGTH = 110.4;
    const double FIXED_TILT = 45.0; 

    // Full circle pan sweep in 30-degree increments
    std::vector<double> pan_angles = {
        -180.0, -150.0, -120.0, -90.0, -60.0, -30.0, 
           0.0,   30.0,   60.0,  90.0, 120.0, 150.0, 180.0
    };
    
    std::vector<double> squared_errors_x;
    std::vector<double> squared_errors_y;
    std::vector<double> squared_errors_z;

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "----------------------------------------------------------------------------------------\n";
    std::cout << " Pan | Expected (X,   Y,   Z)   | Actual IMU (X,   Y,   Z)   | Euclidean Error (mm)\n";
    std::cout << "----------------------------------------------------------------------------------------\n";

    for (double pan : pan_angles) {
        mycobot::Angles target = {pan, FIXED_TILT, 0, 0, 0, 0};
        
        // 1. Move
        robot.WriteAngles(target, 20);
        
        // 2. Wait explicitly until encoders confirm we are physically at target
        WaitUntilTargetReached(robot, target);
        
        // 3. Hold position so IMU settles completely
        std::this_thread::sleep_for(milliseconds(1500)); 

        double pan_rad = pan * M_PI / 180.0;
        double tilt_rad = FIXED_TILT * M_PI / 180.0;

        // Kinematics adjusted to your physical layout:
        // Pan 0 points Backwards (-X). Pan +90 points Right (+Y).
        double exp_x = -ARM_LENGTH * sin(tilt_rad) * cos(pan_rad);
        double exp_y = ARM_LENGTH * sin(tilt_rad) * sin(pan_rad);
        double exp_z = (ARM_LENGTH * cos(tilt_rad)) + BASE_HEIGHT;

        // 4. Read IMU
        std::array<double, 3> imu_pos = imu_arm.GetElbowPosition3D(ARM_LENGTH, BASE_HEIGHT);

        double err_x = imu_pos[0] - exp_x;
        double err_y = imu_pos[1] - exp_y;
        double err_z = imu_pos[2] - exp_z;
        double euclidean_err = sqrt(err_x*err_x + err_y*err_y + err_z*err_z);

        squared_errors_x.push_back(err_x * err_x);
        squared_errors_y.push_back(err_y * err_y);
        squared_errors_z.push_back(err_z * err_z);

        // 5. Print
        std::cout << std::setw(6) << pan << " | "
                  << "(" << std::setw(5) << exp_x << ", " << std::setw(5) << exp_y << ", " << std::setw(5) << exp_z << ") | "
                  << "(" << std::setw(5) << imu_pos[0] << ", " << std::setw(5) << imu_pos[1] << ", " << std::setw(5) << imu_pos[2] << ") | "
                  << std::setw(15) << euclidean_err << std::flush << "\n";
                  
        // 6. Brief pause for readability before next move
        std::this_thread::sleep_for(milliseconds(500)); 
    }

    std::cout << "----------------------------------------------------------------------------------------\n\n";

    double mse_x = std::accumulate(squared_errors_x.begin(), squared_errors_x.end(), 0.0) / squared_errors_x.size();
    double mse_y = std::accumulate(squared_errors_y.begin(), squared_errors_y.end(), 0.0) / squared_errors_y.size();
    double mse_z = std::accumulate(squared_errors_z.begin(), squared_errors_z.end(), 0.0) / squared_errors_z.size();

    std::cout << "--- Statistical Proof Summary --- \n";
    std::cout << "MSE X: " << std::setprecision(4) << mse_x << " mm^2\n";
    std::cout << "MSE Y: " << mse_y << " mm^2\n";
    std::cout << "MSE Z: " << mse_z << " mm^2\n";

    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    WaitUntilTargetReached(robot, {0, 0, 0, 0, 0, 0});

    robot.StopRobot();
    esp32_mux.Stop();
    return 0;
}
