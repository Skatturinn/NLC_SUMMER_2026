#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>

#include "MyCobotDirect.hpp"
#include "ImuSensor.hpp"

using namespace std::chrono;

// Helper to check for orientation changes
void SweepJoint(mycobot::MyCobotDirect& robot, sensor::ImuState& imu, int joint_idx, const std::string& name) {
    std::cout << "\n--- SWEEP: " << name << " (Joint " << joint_idx + 1 << ") ---\n";
    
    // Get initial orientation
    auto q_start = imu.GetRawQuaternion();
    
    // Move joint 
    mycobot::Angles target = {0, 0, 0, 0, 0, 0};
    target[joint_idx] = 45.0;
    
    robot.WriteAngles(target, 30);
    std::this_thread::sleep_for(milliseconds(1500)); // Allow time to move and settle
    
    auto q_end = imu.GetRawQuaternion();
    
    // Simple check: did the quaternion change significantly?
    double delta = std::abs(q_end[0] - q_start[0]) + std::abs(q_end[1] - q_start[1]);
    
    if (delta > 0.05) {
        std::cout << "[RESULT] Wrist IMU orientation CHANGED. Sensor is AFTER Joint " << joint_idx + 1 << ".\n";
    } else {
        std::cout << "[RESULT] Wrist IMU orientation did NOT change. Sensor is BEFORE Joint " << joint_idx + 1 << ".\n";
    }

    // Return to zero
    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    std::this_thread::sleep_for(milliseconds(1500));
}

int main() {
    sensor::ImuMultiplexer esp32_mux("/dev/ttyUSB1", B500000);
    esp32_mux.Start();
    std::this_thread::sleep_for(milliseconds(500)); 

    // Wrist IMU is index 1 based on your ESP32 array
    sensor::ImuState& imu_wrist = esp32_mux.imu_array[1];

    mycobot::MyCobotDirect robot;
    if (!robot.Connect("/dev/ttyUSB0")) return 1;

    robot.PowerOn();
    robot.StopRobot();
    std::this_thread::sleep_for(milliseconds(1000));

    // Reset all to Zero
    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    std::this_thread::sleep_for(milliseconds(2000));

    // Sweep through joints 2 to 5 to isolate the wrist IMU location
    // We skip Joint 0 and 1 because we know the base/arm IMU handles those
    SweepJoint(robot, imu_wrist, 2, "Joint 3");
    SweepJoint(robot, imu_wrist, 3, "Joint 4");
    SweepJoint(robot, imu_wrist, 4, "Joint 5");

    robot.StopRobot();
    esp32_mux.Stop();
    
    return 0;
}
