#include <iostream>
#include <chrono>
#include <thread>
#include "MyCobotDirect.hpp"

using namespace std::chrono_literals;

// Helper to print arrays
template<typename T>
void PrintMsg(const std::string& str, const T& data) {
    std::cout << str << " [";
    for (int i = 0; i < 6; ++i) {
        std::cout << data[i] << (i < 5 ? ", " : "");
    }
    std::cout << "]\n";
}

int main() {
    mycobot::MyCobotDirect robot;

    std::cout << "Connecting to MyCobot...\n";
    if (!robot.Connect("/dev/ttyUSB0")) {
        std::cerr << "Failed to connect to /dev/ttyUSB0\n";
        return 1;
    }

    // 1. Smart Power On (No sleep(1) required - it verifies via serial)
    std::cout << "Powering up servos...\n";
    robot.PowerOn();
    std::cout << "Robot servos fully powered and ready.\n";

    robot.StopRobot();
    std::this_thread::sleep_for(200ms);

    // Read current state
    mycobot::Angles initial_angles = robot.GetAngles();
    PrintMsg("Initial Angles", initial_angles);
    
    mycobot::Coords initial_coords = robot.GetCoords();
    PrintMsg("Initial Coords", initial_coords);

    // 2. Move Joint 3 to 45 Degrees
    std::cout << "\nMoving Joint 3 to 45 degrees...\n";
    mycobot::Angles target_angles_45 = initial_angles;
    target_angles_45[2] = 45.0; // Joint 3 (index 2)
    robot.WriteAngles(target_angles_45, 50);
    
    std::this_thread::sleep_for(100ms); // Give firmware a moment to register movement
    
    // Polling movement state (No arbitrary sleep!)
    while (robot.IsMoving()) {
        PrintMsg("Moving", robot.GetAngles());
        std::this_thread::sleep_for(250ms);
    }
    std::cout << "Reached 45 degrees.\n";

    // 3. Move Joint 3 to 90 Degrees
    std::cout << "\nMoving Joint 3 to 90 degrees...\n";
    mycobot::Angles target_angles_90 = target_angles_45;
    target_angles_90[2] = 90.0;
    robot.WriteAngles(target_angles_90, 50);

    std::this_thread::sleep_for(100ms);
    
    while (robot.IsMoving()) {
        PrintMsg("Moving", robot.GetAngles());
        std::this_thread::sleep_for(250ms);
    }
    std::cout << "Reached 90 degrees.\n";

    std::cout << "\nStopping Robot...\n";
    robot.StopRobot();
    
    return 0;
}
