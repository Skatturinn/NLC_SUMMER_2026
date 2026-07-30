#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <cmath>
#include <vector>

#include "MyCobotDirect.hpp"
#include "ImuSensor.hpp"
#include "port_discovery.hpp"
#include "ArmEKF.hpp"

using namespace std::chrono;

int main() {
    RobotPorts ports = autoDiscoverDevices();
    if (ports.arm_port.empty() || ports.imu_port.empty()) return 1;

    sensor::ImuMultiplexer esp32_mux(ports.imu_port, B500000);
    esp32_mux.Start();
    std::this_thread::sleep_for(milliseconds(500)); 

    sensor::ImuState& imu_arm = esp32_mux.imu_array[0];
    imu_arm.SetMountingRotation(0.0, 1.0, 0.0, 0.0);

    mycobot::MyCobotDirect robot;
    if (!robot.Connect(ports.arm_port)) return 1;
    robot.PowerOn();

    std::cout << "Homing robot to calibrate IMU...\n";
    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    robot.WaitMoveToAngles({0, 0, 0, 0, 0, 0});
    std::this_thread::sleep_for(milliseconds(1000));
    
    imu_arm.Tare();

    ArmEKF ekf;

    std::vector<double> pan_angles = {30.0, 60.0, 90.0, 0.0};
    const double FIXED_TILT = 45.0;

    auto last_time = steady_clock::now();
    auto last_encoder_poll = steady_clock::now();
    const int ENCODER_POLL_INTERVAL_MS = 250; // 4 Hz encoder polling

    mycobot::Angles last_valid_encoders = {0, 0, 0, 0, 0, 0};
    int consecutive_drops = 0;

    for (double pan : pan_angles) {
        mycobot::Angles target = {pan, FIXED_TILT, 0, 0, 0, 0};
        robot.WriteAngles(target, 20);
        
        std::cout << "\nCommanding Pan to " << pan << " degrees...\n";
        bool reached_target = false;
        int loop_counter = 0;

        while (!reached_target && loop_counter < 200) {
            loop_counter++;
            
            auto current_time = steady_clock::now();
            double dt = duration<double>(current_time - last_time).count();
            last_time = current_time;
            if (dt < 0.001) dt = 0.001; 

            // 1. ALWAYS Predict
            ekf.Predict(dt);

            // 2. ALWAYS Update IMU
            std::array<double, 4> q_imu = imu_arm.GetAlignedQuaternion();
            ekf.UpdateIMU(q_imu);

            // 3. THROTTLED ENCODER READ (Every 250ms)
            bool polled_this_loop = false;
            bool serial_valid = false;
            auto time_since_poll = duration_cast<milliseconds>(current_time - last_encoder_poll).count();
            
            if (time_since_poll >= ENCODER_POLL_INTERVAL_MS) {
                last_encoder_poll = current_time;
                polled_this_loop = true;
                
                mycobot::Angles current_encoders;
                serial_valid = robot.GetAngles(current_encoders);
                
                if (serial_valid) {
                    consecutive_drops = 0;
                    last_valid_encoders = current_encoders;
                    double q1_rad = current_encoders[0] * M_PI / 180.0;
                    double q2_rad = current_encoders[1] * M_PI / 180.0;
                    ekf.UpdateEncoders(q1_rad, q2_rad);
                } else {
                    consecutive_drops++;
                    if (consecutive_drops >= 3) {
                        std::cout << "\n [CRITICAL WARNING: Serial Dead. Executing Hard Reconnect...]\n";
//                        std::cout << " [SERIAL RECOVERY: Pausing to clear bus contention]\n";
                        robot.HardReconnect();
//                        std::this_thread::sleep_for(milliseconds(50));
                        consecutive_drops = 0;
                    }
                }
            }

            // 4. Multi-Line Console Logging
            std::array<double, 4> q_ekf = ekf.GetExpectedQuaternion();

            const char* status_tag = "";
            if (polled_this_loop) {
                status_tag = serial_valid ? " [POLL OK]" : " [TIMEOUT]";
            }

            std::cout << std::fixed << std::setprecision(2);
            std::cout << "[LOOP " << std::setw(3) << loop_counter << "] "
                      << "J1(Enc: " << std::setw(6) << last_valid_encoders[0] 
                      << " True: "  << std::setw(6) << (ekf.GetQ1() * 180.0 / M_PI) << ") | "
                      << "J2(Enc: " << std::setw(6) << last_valid_encoders[1] 
                      << " True: "  << std::setw(6) << (ekf.GetQ2() * 180.0 / M_PI) << ")"
                      << status_tag << "\n";

            std::cout << "    IMU Quat: [" << std::setw(6) << q_imu[0] << ", " 
                      << std::setw(6) << q_imu[1] << ", " << std::setw(6) << q_imu[2] 
                      << ", " << std::setw(6) << q_imu[3] << "]\n";

            std::cout << "    EKF Quat: [" << std::setw(6) << q_ekf[0] << ", " 
                      << std::setw(6) << q_ekf[1] << ", " << std::setw(6) << q_ekf[2] 
                      << ", " << std::setw(6) << q_ekf[3] << "]\n";

            // Check if reached goal using our latest valid encoder angles
            if (std::abs(last_valid_encoders[0] - target[0]) <= 3.0 &&
                std::abs(last_valid_encoders[1] - target[1]) <= 3.0) {
                reached_target = true;
            }

            std::this_thread::sleep_for(milliseconds(50)); 
        }
        std::cout << "Target Reached!\n";
    }

    robot.StopRobot();
    esp32_mux.Stop();
    return 0;
}
