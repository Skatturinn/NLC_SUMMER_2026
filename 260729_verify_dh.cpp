// Having issues with reading (while turning)/(after sending command)

#include <iostream> // To print out in console
#include <chrono> // For time measurement
#include <thread> // For sleep_for // dont like using thing
#include <cmath> // For math functions
#include <iomanip> // For std::setw 
#include <vector> // For std::vector
#include <array> // For std::array
#include <Eigen/Dense> // For Eigen::Matrix4d, Matrix3d
#include <Eigen/Geometry> // For Eigen::Quaterniond

#include "MyCobotDirect.hpp" // For MyCobotDirect class
#include "ImuSensor.hpp" // For ImuSensor class
#include "port_discovery.hpp" // For port discovery

using namespace std::chrono; // For time measurement

// ============================================================================
// 1. PURE DENAVIT-HARTENBERG FORWARD KINEMATICS (JOINT 1 & JOINT 2 ONLY)
// ============================================================================

// Builds the standard 4x4 DH transformation matrix for a single link
Eigen::Matrix4d BuildDHMatrix(double theta, double d, double a, double alpha) {
    Eigen::Matrix4d T;
    double ct = std::cos(theta), st = std::sin(theta);
    double ca = std::cos(alpha), sa = std::sin(alpha);
    T << ct, -st * ca,  st * sa, a * ct,
         st,  ct * ca, -ct * sa, a * st,
         0,   sa,       ca,      d,
         0,   0,        0,       1;
    return T;
}

// Calculates the expected relative quaternion [w, x, y, z] from Home (0, 0)
// using ONLY Denavit-Hartenberg forward kinematics of the first two joints.
std::array<double, 4> ComputeDhQuaternion(double q1_deg, double q2_deg) {
    double q1 = q1_deg * M_PI / 180.0;
    double q2 = q2_deg * M_PI / 180.0;

    // A. Forward Kinematics at HOME (q1 = 0, q2 = 0)
    Eigen::Matrix4d T1_home = BuildDHMatrix(0.0,         0.13122,  0.0,     M_PI / 2.0);
    Eigen::Matrix4d T2_home = BuildDHMatrix(-M_PI / 2.0, 0.0,     -0.1104,  0.0);
    Eigen::Matrix3d R_home  = (T1_home * T2_home).block<3, 3>(0, 0);
    Eigen::Quaterniond q_home(R_home);

    // B. Forward Kinematics at CURRENT ENCODERS (q1, q2)
    Eigen::Matrix4d T1_curr = BuildDHMatrix(q1,              0.13122,  0.0,     M_PI / 2.0);
    Eigen::Matrix4d T2_curr = BuildDHMatrix(q2 - M_PI / 2.0, 0.0,     -0.1104,  0.0);
    Eigen::Matrix3d R_curr  = (T1_curr * T2_curr).block<3, 3>(0, 0);
    Eigen::Quaterniond q_curr(R_curr);

    // C. Relative rotation in Base World Frame: Q_rel = Q_curr * Q_home^-1
    Eigen::Quaterniond q_rel = q_curr * q_home.conjugate();

    // D. Keep scalar w positive so console printouts are easy to compare
    if (q_rel.w() < 0.0) {
        q_rel.coeffs() *= -1.0; // Eigen stores coeffs as [x, y, z, w]
    }

    return { q_rel.w(), q_rel.x(), q_rel.y(), q_rel.z() };
}
// ============================================================================


int main() {
    RobotPorts ports = autoDiscoverDevices();
    if (ports.arm_port.empty() || ports.imu_port.empty()) return 1;

    sensor::ImuMultiplexer esp32_mux(ports.imu_port, B500000);
    esp32_mux.Start();

    std::this_thread::sleep_for(milliseconds(500)); // Wait for the IMU to start streaming

    sensor::ImuState& imu_arm = esp32_mux.imu_array[0];
    imu_arm.SetMountingRotation(0.0, 1.0, 0.0, 0.0); // 180 deg around Y

    mycobot::MyCobotDirect robot;
    if (!robot.Connect(ports.arm_port)) return 1;
    robot.PowerOn();

    std::cout << "Homing robot to calibrate IMU...\n";
    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    std::this_thread::sleep_for(milliseconds(2000));
    imu_arm.Tare(); // Tare the IMU to set current orientation as reference

    std::vector<double> pan_angles  = {0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 168.0};
    std::vector<double> tilt_angles = {0.0, 30.0, 60.0, 90.0};

    for (double pan : pan_angles) {
        for (double tilt : tilt_angles) {
            mycobot::Angles target = {pan, tilt, 0, 0, 0, 0};
            robot.WriteAngles(target, 20);
            std::this_thread::sleep_for(milliseconds(2000)); // Wait for movement to finish

            std::array<double, 4> q_imu = imu_arm.GetAlignedQuaternion();
            
            mycobot::Angles current_encoders;
            bool serial_valid = robot.GetAngles(current_encoders);

            // Use true encoder readings (fallback to target angles if serial dropped)
            double q1_deg = serial_valid ? current_encoders[0] : pan;
            double q2_deg = serial_valid ? current_encoders[1] : tilt;

            // Compute pure DH Ground Truth from the first two joints
            std::array<double, 4> q_dh = ComputeDhQuaternion(q1_deg, q2_deg);

            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Pan: " << std::setw(6) << pan << " Tilt: " << std::setw(6) << tilt
                      << " | IMU Quat: [" << std::setw(5) << q_imu[0] << ", " << std::setw(5) << q_imu[1]
                      << ", " << std::setw(5) << q_imu[2] << ", " << std::setw(5) << q_imu[3] << "]"
                      << " | DH Quat: ["  << std::setw(5) << q_dh[0]  << ", " << std::setw(5) << q_dh[1]
                      << ", " << std::setw(5) << q_dh[2]  << ", " << std::setw(5) << q_dh[3]  << "]"
                      << " | Encoders: [" << std::setw(6) << q1_deg   << ", " << std::setw(6) << q2_deg << "]\n";
        }
    }

    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
    std::this_thread::sleep_for(milliseconds(2000));
    robot.StopRobot();
    esp32_mux.Stop();
    return 0;
}
