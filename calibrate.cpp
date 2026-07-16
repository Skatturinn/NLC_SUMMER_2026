#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

#include "MyCobotDirect.hpp"
#include "ImuSensor.hpp"

using namespace std::chrono;

// Increased settle time to 2 seconds for perfect stability
void WaitMoveToFinish(mycobot::MyCobotDirect& robot) {
    std::this_thread::sleep_for(milliseconds(200)); 
    while (robot.IsMoving()) {
        std::this_thread::sleep_for(milliseconds(50));
    }
    std::this_thread::sleep_for(milliseconds(2000)); // 2 full seconds of mechanical settling
}

// Helper to print state
void PrintState(const std::string& phase, mycobot::MyCobotDirect& robot, sensor::ImuState& imu, int joint_idx) {
    std::cout << phase << ":\n";
    std::cout << "  Robot Encoder: " << robot.GetAngles()[joint_idx] << " deg\n";
    std::cout << "  IMU Roll:      " << imu.GetRoll() << " deg\n";
    std::cout << "  IMU Pitch:     " << imu.GetPitch() << " deg\n";
    std::cout << "  IMU Yaw:       " << imu.GetYaw() << " deg\n\n";
}

int main() {
    sensor::ImuMultiplexer esp32_mux("/dev/ttyUSB0", B500000);
    esp32_mux.Start();
    std::this_thread::sleep_for(milliseconds(500)); 

    sensor::ImuState& imu_arm = esp32_mux.imu_array[0];

    // Your tape measure physical offsets
    imu_arm.SetAxisInversion(false, false, true);
    imu_arm.SetKinematicOffsets(75.0, 75.0);

    mycobot::MyCobotDirect robot;
    if (!robot.Connect("/dev/ttyUSB1")) return 1;

    robot.PowerOn();
    robot.StopRobot();
    WaitMoveToFinish(robot);

    // ==========================================
    // PHASE 1: ALIGNMENT & TARE
    // ==========================================
    std::cout << "Moving to Home position...\n";
    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    WaitMoveToFinish(robot);
    
    imu_arm.Tare();
    std::cout << "IMU Tared. Robot (0,0,0) is now IMU (0,0,0).\n\n";

    // ==========================================
    // PHASE 2: PAN (JOINT 1) MAPPING
    // ==========================================
    std::cout << "--- SWEEP 1: Base Pan (Checking Z-Axis Rotation) ---\n";
    
    PrintState("BEFORE MOVEMENT", robot, imu_arm, 0);
    
    std::cout << "Moving slowly to +90 degrees...\n";
    robot.WriteAngles({90, 0, 0, 0, 0, 0}, 20);
    WaitMoveToFinish(robot);

    PrintState("AFTER MOVEMENT", robot, imu_arm, 0);

    // Return to zero
    std::cout << "Returning to zero...\n";
    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    WaitMoveToFinish(robot);
    imu_arm.Tare(); // Re-tare to kill any drift

    // ==========================================
    // PHASE 3: TILT (JOINT 2) MAPPING
    // ==========================================
    std::cout << "\n--- SWEEP 2: Arm Tilt (Checking Y-Axis Rotation) ---\n";
    
    PrintState("BEFORE MOVEMENT", robot, imu_arm, 1);
    
    std::cout << "Moving slowly to +90 degrees...\n";
    robot.WriteAngles({0, 90, 0, 0, 0, 0}, 20);
    WaitMoveToFinish(robot);

    PrintState("AFTER MOVEMENT (Note: Expect Roll/Yaw Gimbal Lock here)", robot, imu_arm, 1);

    // Cleanup
    std::cout << "Returning to zero and shutting down...\n";
    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    WaitMoveToFinish(robot);
    
    robot.StopRobot();
    esp32_mux.Stop();
    
    return 0;
}
