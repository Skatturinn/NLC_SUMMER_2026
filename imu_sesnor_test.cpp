#include "ImuSensor.hpp"

int main() {
    // 1. Open the serial port and start sorting the packets
    sensor::ImuMultiplexer esp32_mux("/dev/ttyUSB1", B500000);
    esp32_mux.Start();

    // 2. Access your independent IMUs
    // They are updated automatically in the background
    sensor::ImuState& imu_wrist = esp32_mux.imu_array[0];
    sensor::ImuState& imu_shoulder = esp32_mux.imu_array[1];

    // You can calibrate them completely independently!
    imu_wrist.ZeroCurrentOrientation();
    imu_shoulder.ZeroCurrentOrientation();

    // Loop and read
    while(true) {
        std::cout << "Wrist Yaw: " << imu_wrist.GetYaw() 
                  << " | Shoulder Yaw: " << imu_shoulder.GetYaw() << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
