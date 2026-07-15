#include <iostream>
#include <chrono>
#include <thread>
#include "ImuSensor.hpp"

int main() {
    sensor::ImuMultiplexer esp32_mux("/dev/ttyUSB1", B500000);
    esp32_mux.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Starting Raw IMU Sniffer..." << std::endl;
    std::cout << "If values don't change when you move the arm, the ESP32/I2C is the issue.\n\n";

    while (true) {
        auto q0 = esp32_mux.imu_array[0].GetRawQuaternion(); // Slot 7 (Arm)
        auto q1 = esp32_mux.imu_array[1].GetRawQuaternion(); // Slot 4 (Wrist)

        std::cout << "\rIMU_0 (Arm): [" 
                  << q0[0] << ", " << q0[1] << ", " << q0[2] << ", " << q0[3] << "] | "
                  << "IMU_1 (Wrist): [" 
                  << q1[0] << ", " << q1[1] << ", " << q1[2] << ", " << q1[3] << "]" 
                  << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
