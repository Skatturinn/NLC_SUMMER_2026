#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <iomanip>

#include "MyCobotDirect.hpp"
#include "ImuSensor.hpp"

using namespace std::chrono;

void WaitMoveToFinish(mycobot::MyCobotDirect& robot) {
    std::this_thread::sleep_for(milliseconds(200)); 
    while (robot.IsMoving()) std::this_thread::sleep_for(milliseconds(50));
    std::this_thread::sleep_for(milliseconds(2000)); // 2s settle time
}

// Logs the state clearly for "Before/After" comparison
void LogState(const std::string& label, mycobot::MyCobotDirect& robot, sensor::ImuState& imu, int joint_idx) {
    std::cout << "[" << label << "]\n";
    std::cout << "  Robot Encoder (J" << joint_idx+1 << "): " << robot.GetAngles()[joint_idx] << " deg\n";
    std::cout << "  IMU Yaw:   " << std::fixed << std::setprecision(2) << imu.GetYaw() << " deg\n";
    std::cout << "  IMU Pitch: " << imu.GetPitch() << " deg\n";
    std::cout << "  IMU Roll:  " << imu.GetRoll() << " deg\n\n";
}

int main() {
    sensor::ImuMultiplexer esp32_mux("/dev/ttyUSB1", B500000);
    esp32_mux.Start();
    std::this_thread::sleep_for(milliseconds(1000)); 

    // Target the Wrist IMU (Index 1)
    sensor::ImuState& imu_wrist = esp32_mux.imu_array[1];

    mycobot::MyCobotDirect robot;
    if (!robot.Connect("/dev/ttyUSB0")) return 1;

    robot.PowerOn();
    robot.StopRobot();
    WaitMoveToFinish(robot);

    // 1. Move to Home
    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    WaitMoveToFinish(robot);
    imu_wrist.Tare(); 
    
    std::cout << "Calibration: Home position tared.\n";

    // 2. Perform calibration on a specific joint (e.g., Joint 4 - Wrist Twist)
    int joint_to_test = 3; // Index 3 is Joint 4
    double test_angle = 45.0;

    LogState("BEFORE MOVEMENT", robot, imu_wrist, joint_to_test);

    std::cout << "Moving Joint " << joint_to_test + 1 << " to " << test_angle << " deg...\n";
    mycobot::Angles target = {0, 0, 0, 0, 0, 0};
    target[joint_to_test] = test_angle;
    robot.WriteAngles(target, 20);
    
    WaitMoveToFinish(robot);

    LogState("AFTER MOVEMENT", robot, imu_wrist, joint_to_test);

    // 3. Simple Offset Calculation
    double encoder_diff = robot.GetAngles()[joint_to_test];
    double imu_diff = imu_wrist.GetYaw(); // Assuming Yaw is the axis for this joint
    
    std::cout << "Calculated Skew Offset: " << (encoder_diff - imu_diff) << " degrees.\n";
    std::cout << "Hardcode this offset into your SetOrientationOffsets() call.\n";

    robot.StopRobot();
    esp32_mux.Stop();
    return 0;
}
