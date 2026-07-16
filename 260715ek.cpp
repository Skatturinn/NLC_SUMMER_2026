#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <iomanip>
#include <vector>

#include "MyCobotDirect.hpp"
#include "ImuSensor.hpp"

using namespace std::chrono;

// Safely waits for movement to start and finish
void WaitMoveToFinish(mycobot::MyCobotDirect& robot) {
    std::this_thread::sleep_for(milliseconds(200)); 
    while (robot.IsMoving()) {
        std::this_thread::sleep_for(milliseconds(50));
    }
    std::this_thread::sleep_for(milliseconds(1000)); // Allow sensor to settle
}

// Function to calculate and print the state
void PrintKinematics(const std::string& label, mycobot::MyCobotDirect& robot, sensor::ImuState& imu) {
    // The precise measurements from the robot's engineering diagrams
    const double BASE_HEIGHT = 131.22; 
    const double ARM_LENGTH = 110.4;

    mycobot::Angles encoders = robot.GetAngles();
    double pan_rad = encoders[0] * M_PI / 180.0;
    double tilt_rad = encoders[1] * M_PI / 180.0;

    // Theoretical Kinematics (Encoders)
    double enc_x = ARM_LENGTH * cos(tilt_rad) * cos(pan_rad);
    double enc_y = ARM_LENGTH * cos(tilt_rad) * sin(pan_rad);
    double enc_z = (ARM_LENGTH * sin(tilt_rad)) + BASE_HEIGHT;

    // True Physical Kinematics (IMU)
    std::array<double, 3> imu_elbow = imu.GetElbowPosition3D(ARM_LENGTH, BASE_HEIGHT);

    std::cout << "  [" << label << "]\n";
    std::cout << "    ENCODER -> X: " << std::setw(6) << std::fixed << std::setprecision(1) << enc_x 
              << " | Y: " << std::setw(6) << enc_y 
              << " | Z: " << std::setw(6) << enc_z << "\n";
    std::cout << "    IMU     -> X: " << std::setw(6) << imu_elbow[0] 
              << " | Y: " << std::setw(6) << imu_elbow[1]
              << " | Z: " << std::setw(6) << imu_elbow[2] << "\n\n";
}

// Helper to automate the testing of a waypoint
void TestPosition(mycobot::MyCobotDirect& robot, sensor::ImuState& imu, mycobot::Angles target_angles, const std::string& pos_name) {
    std::cout << "--- Testing Position: " << pos_name << " ---\n";
    
    PrintKinematics("BEFORE MOVEMENT", robot, imu);

    std::cout << "  Moving robot...\n";
    robot.WriteAngles(target_angles, 20);
    WaitMoveToFinish(robot);

    PrintKinematics("AFTER MOVEMENT", robot, imu);
}

int main() {
    sensor::ImuMultiplexer esp32_mux("/dev/ttyUSB0", B500000);
    esp32_mux.Start();
    std::this_thread::sleep_for(milliseconds(500)); 

    sensor::ImuState& imu_arm = esp32_mux.imu_array[0];
    
    // HARDCODED AXIS MAPPING
    // Adjust these depending on how your specific sensor is physically mounted
    imu_arm.SetAxisMapping(1, 2, 3);

    mycobot::MyCobotDirect robot;
    if (!robot.Connect("/dev/ttyUSB1")) return 1;
    robot.PowerOn();

    std::cout << "Homing robot for IMU alignment...\n";
    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    WaitMoveToFinish(robot);
    
    imu_arm.Tare();
    std::cout << "IMU aligned to Robot Space.\n\n";

    // Test 4 distinct positions to compare math
    TestPosition(robot, imu_arm, {45, 45, 0, 0, 0, 0}, "Pan 45, Tilt 45");
    TestPosition(robot, imu_arm, {-45, 30, 0, 0, 0, 0}, "Pan -45, Tilt 30");
    TestPosition(robot, imu_arm, {90, 0, 0, 0, 0, 0}, "Pan 90, Tilt 0");
    TestPosition(robot, imu_arm, {0, 0, 0, 0, 0, 0}, "Return to Home");

    std::cout << "Sweep complete.\n";

    robot.StopRobot();
    esp32_mux.Stop();
    return 0;
}
