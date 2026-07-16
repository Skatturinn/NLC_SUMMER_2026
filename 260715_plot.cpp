#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <iomanip>
#include <vector>

#include "MyCobotDirect.hpp"
#include "ImuSensor.hpp"

using namespace std::chrono;

int main() {
    sensor::ImuMultiplexer esp32_mux("/dev/ttyUSB0", B500000);
    esp32_mux.Start();
    std::this_thread::sleep_for(milliseconds(500)); 

    sensor::ImuState& imu_arm = esp32_mux.imu_array[0];
    
    // Perfect 1-to-1 physical alignment map
    imu_arm.SetAxisMapping(1, 2, 3);
    
    // Replace with your physical offset values
    imu_arm.SetHardcodedTare(1.0, 0.0, 0.0, 0.0);

    mycobot::MyCobotDirect robot;
    if (!robot.Connect("/dev/ttyUSB1")) return 1;
    robot.PowerOn();

    const double BASE_HEIGHT = 131.22; 
    const double ARM_LENGTH = 110.4;  

    // 1. Move to starting position
    mycobot::Angles start_pos = {0, 0, 0, 0, 0, 0};
    robot.WriteAngles(start_pos, 30);
    robot.WaitMoveToAngles(start_pos);

    // 2. Command the sweep
    mycobot::Angles target_pos = {45, 45, 0, 0, 0, 0};
    robot.WriteAngles(target_pos, 15); 
    
    // Allow the trajectory to begin before starting the while loop
    std::this_thread::sleep_for(milliseconds(200));

    std::cout << "Timestamp_ms,Enc_X,Enc_Y,Enc_Z,IMU_X,IMU_Y,IMU_Z\n";

    // 3. HIGH SPEED LOGGING LOOP
    // Exits immediately when the robot motors physically stop
    while (robot.IsMoving()) {
        mycobot::Angles encoders = robot.GetAngles();

        double pan_rad = encoders[0] * M_PI / 180.0;
        double tilt_rad = encoders[1] * M_PI / 180.0;

        double enc_x = ARM_LENGTH * cos(tilt_rad) * cos(pan_rad);
        double enc_y = ARM_LENGTH * cos(tilt_rad) * sin(pan_rad);
        double enc_z = (ARM_LENGTH * sin(tilt_rad)) + BASE_HEIGHT;

        std::array<double, 3> imu_elbow = imu_arm.GetElbowPosition3D(ARM_LENGTH, BASE_HEIGHT);

        std::cout << imu_arm.GetTimestamp() << "," 
                  << std::fixed << std::setprecision(3)
                  << enc_x << "," << enc_y << "," << enc_z << ","
                  << imu_elbow[0] << "," << imu_elbow[1] << "," << imu_elbow[2] << "\n";

        std::this_thread::sleep_for(milliseconds(20)); 
    }

    robot.StopRobot();
    esp32_mux.Stop();
    return 0;
}
