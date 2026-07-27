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

// ---------------------------------------------------------
// 1. EKF Mathematical Definitions
// ---------------------------------------------------------

// Builds the standard Denavit-Hartenberg Transformation Matrix
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

// The Observation Function h(x): Translates the 12x1 state vector into a 14x1 expected measurement vector
Eigen::VectorXd h(const Eigen::VectorXd& x) {
    // 14 Measurements: 6 Encoders (rads), 4 IMU1 Quats (W,X,Y,Z), 4 IMU2 Quats (W,X,Y,Z)
    Eigen::VectorXd z_expected = Eigen::VectorXd::Zero(14); 

    // The expected encoder readings are simply the predicted joint angles
    double q[6] = {x(0), x(1), x(2), x(3), x(4), x(5)};
    for (int i = 0; i < 6; i++) {
        z_expected(i) = q[i]; 
    }

    // myCobot 280 DH parameters (converted to meters)[cite: 1]
    // Order: {alpha, a, d, theta}
    double dh_params[6][4] = {
        {M_PI / 2.0,  0.0,      0.13122, q[0]},
        {0.0,        -0.1104,   0.0,     q[1] - M_PI / 2.0},
        {0.0,        -0.096,    0.0,     q[2]},
        {M_PI / 2.0,  0.0,      0.0634,  q[3] - M_PI / 2.0},
        {-M_PI / 2.0, 0.0,      0.07505, q[4] + M_PI / 2.0},
        {0.0,         0.0,      0.0456,  q[5]}
    };

    Eigen::Matrix4d T_current = Eigen::Matrix4d::Identity();
    
    for (int i = 0; i < 6; i++) {
        Eigen::Matrix4d A_i = GetDHMatrix(dh_params[i][3], dh_params[i][2], dh_params[i][1], dh_params[i][0]);
        T_current = T_current * A_i;

        // Extract IMU 1 (Assuming mounted on Upper Arm, after Joint 2)
        if (i == 1) {
            Eigen::Quaterniond quat1(T_current.block<3, 3>(0, 0));
            z_expected(6) = quat1.w();
            z_expected(7) = quat1.x();
            z_expected(8) = quat1.y();
            z_expected(9) = quat1.z();
        }
        
        // Extract IMU 2 (Assuming mounted on Forearm, after Joint 5)
        if (i == 4) {
            Eigen::Quaterniond quat2(T_current.block<3, 3>(0, 0));
            z_expected(10) = quat2.w();
            z_expected(11) = quat2.x();
            z_expected(12) = quat2.y();
            z_expected(13) = quat2.z();
        }
    }
    return z_expected;
}

// Computes the 14x12 Jacobian Matrix numerically
Eigen::MatrixXd ComputeMeasurementJacobian(const Eigen::VectorXd& x) {
    double delta = 1e-4;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(14, 12); 
    Eigen::VectorXd z0 = h(x); 

    for (int i = 0; i < 12; i++) {
        Eigen::VectorXd x_temp = x;
        x_temp(i) += delta;
        Eigen::VectorXd z_temp = h(x_temp);
        H.col(i) = (z_temp - z0) / delta;
    }

    return H;
}

// ---------------------------------------------------------
// 2. Hardware Synchronization 
// ---------------------------------------------------------

void WaitUntilTargetReached(mycobot::MyCobotDirect& robot, const mycobot::Angles& target) {
    for (int timeout = 0; timeout < 60; ++timeout) { 
        mycobot::Angles current = robot.GetAngles();
        bool reached = true;
        for (int j = 0; j < 6; j++) {
            if (std::abs(current[j] - target[j]) > 2.5) { 
                reached = false;
                break;
            }
        }
        if (reached) break;
        std::this_thread::sleep_for(milliseconds(100));
    }
}

// ---------------------------------------------------------
// 3. Main Execution Loop
// ---------------------------------------------------------

int main() {
    // Hardware Discovery[cite: 2]
    RobotPorts ports = autoDiscoverDevices();
    if (ports.arm_port.empty() || ports.imu_port.empty()) {
        std::cerr << "CRITICAL ERROR: Devices missing." << std::endl;
        return 1;
    }

    sensor::ImuMultiplexer esp32_mux(ports.imu_port, B500000);
    esp32_mux.Start();
    std::this_thread::sleep_for(milliseconds(500)); 

    sensor::ImuState& imu_arm = esp32_mux.imu_array[0];
    sensor::ImuState& imu_wrist = esp32_mux.imu_array[1];
    
    // Set appropriate mappings based on physical installation
    imu_arm.SetAxisMapping(1, 2, 3);
    imu_wrist.SetAxisMapping(1, 2, 3);

    mycobot::MyCobotDirect robot;
    if (!robot.Connect(ports.arm_port)) {
        std::cerr << "CRITICAL ERROR: Failed to connect." << std::endl;
        esp32_mux.Stop();
        return 1;
    }
    robot.PowerOn();

    // Homing and Zeroing 
    std::cout << "Homing robot to calibrate IMUs...\n";
    mycobot::Angles home_angles = {0, 0, 0, 0, 0, 0};
    robot.WriteAngles(home_angles, 30);
    WaitUntilTargetReached(robot, home_angles);
    std::this_thread::sleep_for(milliseconds(2000)); 
    
    imu_arm.Tare();
    imu_wrist.Tare();
    std::cout << "Sensors Aligned. Initializing EKF...\n\n";

    // Initialize EKF Variables
    Eigen::VectorXd x = Eigen::VectorXd::Zero(12); // State: 6 Angles, 6 Velocities
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(12, 12) * 1.0; 
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12, 12) * 0.05; // Process Noise
    
    // Measurement Noise (R): 14x14
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(14, 14);
    for (int i = 0; i < 6; i++) R(i, i) = 0.01;       // High trust in encoders
    for (int i = 6; i < 14; i++) R(i, i) = 0.5;       // Lower trust in IMUs

    // Begin Active Trajectory with real-time EKF
    std::vector<double> pan_angles = {30.0, 60.0, 90.0, 60.0, 30.0, 0.0};
    const double FIXED_TILT = 45.0;

    auto last_time = std::chrono::steady_clock::now();

    for (double pan : pan_angles) {
        mycobot::Angles target = {pan, FIXED_TILT, 0, 0, 0, 0};
        robot.WriteAngles(target, 20);
        
        std::cout << "\nCommanding Pan to " << pan << " degrees...\n";

        // Active State Estimation Loop while moving
        while (robot.IsMoving()) {
            auto current_time = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(current_time - last_time).count();
            last_time = current_time;

            if (dt < 0.001) dt = 0.001; // Safety threshold

            // EKF Predict Step
            Eigen::MatrixXd F = Eigen::MatrixXd::Identity(12, 12);
            for (int i = 0; i < 6; i++) F(i, i + 6) = dt;
            x = F * x;
            P = F * P * F.transpose() + Q;

            // Gather Real-Time Sensor Data
            mycobot::Angles current_encoders = robot.GetAngles();
            std::array<double, 4> q1 = imu_arm.GetAlignedQuaternion();
            std::array<double, 4> q2 = imu_wrist.GetAlignedQuaternion();

            // Populate Actual Measurement Vector (z)
            Eigen::VectorXd z_actual(14);
            for(int i = 0; i < 6; i++) z_actual(i) = current_encoders[i] * M_PI / 180.0;
            z_actual(6) = q1[0]; z_actual(7) = q1[1]; z_actual(8) = q1[2]; z_actual(9) = q1[3];
            z_actual(10) = q2[0]; z_actual(11) = q2[1]; z_actual(12) = q2[2]; z_actual(13) = q2[3];

            // EKF Measurement Update Step
            Eigen::MatrixXd H = ComputeMeasurementJacobian(x);
            Eigen::VectorXd z_expected = h(x);
            Eigen::VectorXd y = z_actual - z_expected; // Residual

            Eigen::MatrixXd S = H * P * H.transpose() + R;
            Eigen::MatrixXd K = P * H.transpose() * S.inverse();
            
            x = x + K * y; // Apply corrections
            P = (Eigen::MatrixXd::Identity(12, 12) - K * H) * P;

            // Output the fusion results for J1 and J2
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Raw J1: " << current_encoders[0] << " | EKF J1: " << x(0) * 180.0 / M_PI << " || ";
            std::cout << "Raw J2: " << current_encoders[1] << " | EKF J2: " << x(1) * 180.0 / M_PI << "\r" << std::flush;

            std::this_thread::sleep_for(milliseconds(20)); // ~50Hz estimation loop
        }
        
        WaitUntilTargetReached(robot, target);
        std::cout << "\nReached waypoint.\n";
    }

    // Safely Return to Home
    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    WaitUntilTargetReached(robot, {0, 0, 0, 0, 0, 0});

    robot.StopRobot();
    esp32_mux.Stop();
    std::cout << "\nEKF State Estimation Shutdown Successfully.\n";
    
    return 0;
}
