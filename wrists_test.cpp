#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>

#include "MyCobotDirect.hpp"
#include "ImuSensor.hpp"

using namespace std::chrono;

void WaitMoveToFinish(mycobot::MyCobotDirect& robot) {
    std::this_thread::sleep_for(milliseconds(200)); 
    while (robot.IsMoving()) std::this_thread::sleep_for(milliseconds(50));
    // Long settle time to ensure sensor is perfectly static
    std::this_thread::sleep_for(milliseconds(2000)); 
}

struct Orientation {
    double roll, pitch, yaw;
};

void PrintOrientation(const std::string& label, const Orientation& o) {
    std::cout << "  " << label << " -> ";
    std::cout << "Roll: " << std::setw(8) << o.roll << " | ";
    std::cout << "Pitch: " << std::setw(8) << o.pitch << " | ";
    std::cout << "Yaw: " << std::setw(8) << o.yaw << "\n";
}

int main() {
    sensor::ImuMultiplexer esp32_mux("/dev/ttyUSB1", B500000);
    esp32_mux.Start();
    std::this_thread::sleep_for(milliseconds(500)); 
    sensor::ImuState& imu = esp32_mux.imu_array[1]; // Wrist IMU

    mycobot::MyCobotDirect robot;
    if (!robot.Connect("/dev/ttyUSB0")) return 1;
    robot.PowerOn();

    // Home to Zero
    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    WaitMoveToFinish(robot);
    imu.Tare();

    for (int i = 0; i < 6; i++) {
        std::cout << "\n========================================\n";
        std::cout << "TESTING JOINT " << i + 1 << "\n";
        
        Orientation before = {imu.GetRoll(), imu.GetPitch(), imu.GetYaw()};
        PrintOrientation("BEFORE", before);

        mycobot::Angles target = {0, 0, 0, 0, 0, 0};
        target[i] = 45.0; // Move 45 degrees
        
        robot.WriteAngles(target, 20);
        WaitMoveToFinish(robot);

        Orientation after = {imu.GetRoll(), imu.GetPitch(), imu.GetYaw()};
        PrintOrientation("AFTER ", after);

        std::cout << "DELTA   -> ";
        std::cout << "Roll: " << std::setw(8) << (after.roll - before.roll) << " | ";
        std::cout << "Pitch: " << std::setw(8) << (after.pitch - before.pitch) << " | ";
        std::cout << "Yaw: " << std::setw(8) << (after.yaw - before.yaw) << "\n";

        // Return to zero
        robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
        WaitMoveToFinish(robot);
        imu.Tare();
    }

    robot.StopRobot();
    esp32_mux.Stop();
    return 0;
}
