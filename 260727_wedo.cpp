#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <iomanip>
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "MyCobotDirect.hpp"
#include "ImuSensor.hpp"
#include "port_discovery.hpp"

using namespace std::chrono;


// Builds the 4x4 Transformation Matrix for a single robotic joint
Eigen::Matrix4d GetDHMatrix(double theta, double d, double a, double alpha) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    
    T(0, 0) = std::cos(theta);
    T(0, 1) = -std::sin(theta) * std::cos(alpha);
    T(0, 2) = std::sin(theta) * std::sin(alpha);
    T(0, 3) = a * std::cos(theta);

    T(1, 0) = std::sin(theta);
    T(1, 1) = std::cos(theta) * std::cos(alpha);
    T(1, 2) = -std::cos(theta) * std::sin(alpha);
    T(1, 3) = a * std::sin(theta);

    T(2, 0) = 0.0;
    T(2, 1) = std::sin(alpha);
    T(2, 2) = std::cos(alpha);
    T(2, 3) = d;

    return T;
}


// Translates the 4x1 state vector into a 6x1 expected measurement vector
Eigen::VectorXd h(const Eigen::VectorXd& x) {
    Eigen::VectorXd z_expected = Eigen::VectorXd::Zero(6); 

    double q1 = x(0);
    double q2 = x(1);
    z_expected(0) = q1; 
    z_expected(1) = q2;

    // 1. Calculate the Absolute DH Quaternion at HOME (0, 0)
    double dh_home[2][4] = {
        {M_PI / 2.0,  0.0,      0.13122, 0.0},
        {0.0,        -0.1104,   0.0,     -M_PI / 2.0}
    };
    Eigen::Matrix4d T_home = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 2; i++) {
        T_home = T_home * GetDHMatrix(dh_home[i][3], dh_home[i][2], dh_home[i][1], dh_home[i][0]);
    }
    Eigen::Quaterniond quat_home(T_home.block<3, 3>(0, 0));

    // 2. Calculate the Absolute DH Quaternion at CURRENT State
    double dh_current[2][4] = {
        {M_PI / 2.0,  0.0,      0.13122, q1},
        {0.0,        -0.1104,   0.0,     q2 - M_PI / 2.0}
    };
    Eigen::Matrix4d T_current = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 2; i++) {
        T_current = T_current * GetDHMatrix(dh_current[i][3], dh_current[i][2], dh_current[i][1], dh_current[i][0]);
    }
    Eigen::Quaterniond quat_current(T_current.block<3, 3>(0, 0));

    // 3. Find the RELATIVE rotation (matches the IMU Tare behavior)
    Eigen::Quaterniond quat_expected = quat_home.conjugate() * quat_current;
    
    // 4. Fill the measurement vector
    z_expected(2) = quat_expected.w();
    z_expected(3) = quat_expected.x();
    z_expected(4) = quat_expected.y();
    z_expected(5) = quat_expected.z();
    
    return z_expected;
}


// Computes the 6x4 Jacobian Matrix numerically
Eigen::MatrixXd ComputeMeasurementJacobian(const Eigen::VectorXd& x) {
    double delta = 1e-4;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, 4); 
    Eigen::VectorXd z0 = h(x); // Baseline measurement

    for (int i = 0; i < 4; i++) {
        Eigen::VectorXd x_temp = x;
        x_temp(i) += delta; // Nudge one state variable
        Eigen::VectorXd z_temp = h(x_temp);
        
        // Calculate the slope (Rise over Run)
        H.col(i) = (z_temp - z0) / delta;
    }

    return H;
}


// A simple blocking function to ensure the robot has physically stopped
void WaitUntilTargetReached(mycobot::MyCobotDirect& robot, const mycobot::Angles& target) {
    for (int timeout = 0; timeout < 60; ++timeout) { 
        // mycobot::Angles current = robot.GetAngles();
        mycobot::Angles current;
        bool serial_valid = robot.GetAngles(current);
        bool reached = true;
        for (int j = 0; j < 2; j++) { // Only checking first two joints for this test
            if (std::abs(current[j] - target[j]) > 3.0) { 
                reached = false;
                break;
            }
        }
        if (reached) break;
        std::this_thread::sleep_for(milliseconds(100));
    }
}




int main() {
    // 1. Hardware Initialization
    RobotPorts ports = autoDiscoverDevices();
    if (ports.arm_port.empty() || ports.imu_port.empty()) return 1;

    sensor::ImuMultiplexer esp32_mux(ports.imu_port, B500000);
    esp32_mux.Start();
    std::this_thread::sleep_for(milliseconds(500)); 

    sensor::ImuState& imu_arm = esp32_mux.imu_array[0];
// this is nonsense and needs to be deleted in future    imu_arm.SetAxisMapping(1, -2, -3); // Adjust to your physical mounting
    imu_arm.SetMountingRotation(0.0, 1.0, 0.0, 0.0); // We rotate the imu 180 around x
    mycobot::MyCobotDirect robot;
    if (!robot.Connect(ports.arm_port)) return 1;
    robot.PowerOn();

    std::cout << "Homing robot to calibrate IMU...\n";
    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    WaitUntilTargetReached(robot, {0, 0, 0, 0, 0, 0});
    std::this_thread::sleep_for(milliseconds(2000)); 
    imu_arm.Tare();

    // 2. EKF Matrix Initialization
    Eigen::VectorXd x = Eigen::VectorXd::Zero(4); // [J1, J2, Vel1, Vel2]
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(4, 4) * 1.0; 
    
    // Process Noise (Q) - How much we trust the F matrix math
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4, 4) * 0.05; 
    
    // Measurement Noise (R) - 6x6 Matrix. 
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(6, 6);
    R(0, 0) = 0.01; R(1, 1) = 0.01; // High trust in encoders
    for (int i = 2; i < 6; i++) R(i, i) = 0.5; // Lower trust in IMU quaternion
    // 3. Command a trajectory sequence
    std::vector<double> pan_angles = {30.0, 60.0, 90.0, 0.0};
    const double FIXED_TILT = 45.0;

    auto last_time = std::chrono::steady_clock::now();

    for (double pan : pan_angles) {
        mycobot::Angles target = {pan, FIXED_TILT, 0, 0, 0, 0};
        robot.WriteAngles(target, 20);
        
        std::cout << "\nCommanding Pan to " << pan << " degrees...\n";
        bool reached_target = false;
        int timeout_counter = 0;

        // Active State Estimation Loop (~20Hz)
        while (!reached_target && timeout_counter < 120) {
            timeout_counter++;
            
            auto current_time = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(current_time - last_time).count();
            last_time = current_time;
            if (dt < 0.001) dt = 0.001; 

            // --- A. PREDICT STEP ---
            // F Matrix links velocity to position via dt
            Eigen::MatrixXd F = Eigen::MatrixXd::Identity(4, 4);
            F(0, 2) = dt; 
            F(1, 3) = dt;
            
            x = F * x; 
            P = F * P * F.transpose() + Q;

            // --- B. READ SENSORS ---
            // old // mycobot::Angles current_encoders = robot.GetAngles();
            mycobot::Angles current_encoders;
            bool serial_valid = robot.GetAngles(current_encoders);
            if (!serial_valid) {
                std::cout << "[EKF " << std::setw(3) << timeout_counter << "] SERIAL TIMEOUT! Robot unresponsive. Coasting...\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue; 
            }
            std::array<double, 4> q1 = imu_arm.GetAlignedQuaternion();

            // Populate Actual Measurement Vector (z)
            Eigen::VectorXd z_actual(6);
            z_actual(0) = current_encoders[0] * M_PI / 180.0;
            z_actual(1) = current_encoders[1] * M_PI / 180.0;
            z_actual(2) = q1[0]; z_actual(3) = q1[1]; z_actual(4) = q1[2]; z_actual(5) = q1[3];

            // --- C. UPDATE STEP ---
            Eigen::MatrixXd H = ComputeMeasurementJacobian(x);
            Eigen::VectorXd z_expected = h(x);
            
            Eigen::VectorXd y = z_actual - z_expected; // Residual (Error)
            Eigen::MatrixXd S = H * P * H.transpose() + R;
            Eigen::MatrixXd K = P * H.transpose() * S.inverse(); // Kalman Gain
            
            x = x + K * y; // Correct the State Vector
            P = (Eigen::MatrixXd::Identity(4, 4) - K * H) * P;

            // --- D. LOGGING ---
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "[EKF " << std::setw(3) << timeout_counter << "] "
                      << "J1(Enc: " << std::setw(6) << current_encoders[0] << " True: " << std::setw(6) << (x(0) * 180.0 / M_PI) << ") | "
                      << "J2(Enc: " << std::setw(6) << current_encoders[1] << " True: " << std::setw(6) << (x(1) * 180.0 / M_PI) << ")\n";



           std::cout << "     IMU Quat: [" << std::setw(6) << q1[0] << ", " << std::setw(6) << q1[1] << ", " << std::setw(6) << q1[2] << ", " << std::setw(6) << q1[3] << "]\n";
           std::cout << "     EKF Quat: [" << std::setw(6) << z_expected(2) << ", " << std::setw(6) << z_expected(3) << ", " << std::setw(6) << z_expected(4) << ", " << std::setw(6) << z_expected(5) << "]\n";
            
            // Check if physically reached target
            if (std::abs(current_encoders[0] - target[0]) <= 3.0 && std::abs(current_encoders[1] - target[1]) <= 3.0) {
                reached_target = true;
            }

            std::this_thread::sleep_for(milliseconds(50)); 
        }
        
        std::this_thread::sleep_for(milliseconds(500)); 
    }

    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    WaitUntilTargetReached(robot, {0, 0, 0, 0, 0, 0});
    robot.StopRobot();
    esp32_mux.Stop();
    return 0;
}
