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
    std::this_thread::sleep_for(milliseconds(1000)); 
}

int main() {
    sensor::ImuMultiplexer esp32_mux("/dev/ttyUSB0", B500000);
    esp32_mux.Start();
    std::this_thread::sleep_for(milliseconds(500)); 

    sensor::ImuState& imu_arm = esp32_mux.imu_array[0];

    // HARDCODED AXIS MAPPING
    // Based on your previous logs: Robot Y maps to IMU Z (inverted). Robot Z maps to IMU Y (inverted).
    imu_arm.SetAxisMapping(1, -3, -2);

    mycobot::MyCobotDirect robot;
    if (!robot.Connect("/dev/ttyUSB1")) return 1;
    robot.PowerOn();

    // 1. Move to Home and Tare
    std::cout << "Homing robot for IMU alignment...\n";
    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    WaitMoveToFinish(robot);
    
    imu_arm.Tare();
    std::cout << "IMU aligned to Robot Space.\n\n";

    const double BASE_HEIGHT = 131.22; 
    const double ARM_LENGTH = 110.4;  

    std::cout << "Starting Sweep... Watch the XYZ coordinates match.\n\n";

    // 2. Command a slow diagonal sweep (Pan +45 degrees, Tilt +45 degrees)
    robot.WriteAngles({45, 45, 0, 0, 0, 0}, 10); // Speed 10 for a slow, observable sweep

    // Loop continuously while the robot is executing the movement
    while (robot.IsMoving()) {
        // --- THEORETICAL KINEMATICS (Encoders) ---
        mycobot::Angles encoders = robot.GetAngles();
        double pan_rad = encoders[0] * M_PI / 180.0;
        double tilt_rad = encoders[1] * M_PI / 180.0;

        double enc_x = ARM_LENGTH * cos(tilt_rad) * cos(pan_rad);
        double enc_y = ARM_LENGTH * cos(tilt_rad) * sin(pan_rad);
        double enc_z = (ARM_LENGTH * sin(tilt_rad)) + BASE_HEIGHT;

        // --- TRUE PHYSICAL KINEMATICS (IMU) ---
        std::array<double, 3> imu_elbow = imu_arm.GetElbowPosition3D(ARM_LENGTH, BASE_HEIGHT);

        // --- OUTPUT ---
        std::cout << "\r[ENCODER] X: " << std::setw(6) << std::fixed << std::setprecision(1) << enc_x 
                  << " | Y: " << std::setw(6) << enc_y 
                  << " | Z: " << std::setw(6) << enc_z 
                  << "      [IMU] X: " << std::setw(6) << imu_elbow[0] 
                  << " | Y: " << std::setw(6) << imu_elbow[1]
                  << " | Z: " << std::setw(6) << imu_elbow[2]
                  << std::flush;

        std::this_thread::sleep_for(milliseconds(50)); 
    }

    std::cout << "\n\nSweep complete." << std::endl;

    robot.StopRobot();
    esp32_mux.Stop();
    return 0;
}
