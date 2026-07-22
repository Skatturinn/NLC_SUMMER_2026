#include <iostream>
#include "port_discovery.hpp"
#include "ImuSensor.hpp"
#include "MyCobotDirect.hpp"

int main() {
    // 1. Run the discovery utility
    RobotPorts ports = autoDiscoverDevices();

    // 2. Check if it found everything
    if (ports.arm_port.empty() || ports.imu_port.empty()) {
        std::cerr << "CRITICAL ERROR: Could not find all devices!" << std::endl;
        return 1;
    }

    // 3. Pass the discovered ports securely to your actual connection logic
    mycobot::MyCobotDirect arm;
    arm.Connect(ports.arm_port);
    
    sensor::ImuMultiplexer imuStream(ports.imu_port);
    imuStream.Start();

    // ... continue with your robotics logic ...
    
    return 0;
}
