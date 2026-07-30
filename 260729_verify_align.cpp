// Having issues with reading (while turning)/(after sending command)


#include <iostream> // To print out in console
#include <chrono> // For time measurement
#include <thread> // For sleep_for // dont like using thing
#include <cmath> // For math functions
#include <iomanip> // For std::setw	
#include <vector> // For std::vector
#include <array> // For std::array
// #include <Eigen/Dense> // For Eigen::MatrixXd
// #include <Eigen/Geometry> // For Eigen::Quaterniond
// maby we should be using pinocchio or KDL?
#include "MyCobotDirect.hpp" // For MyCobotDirect class
#include "ImuSensor.hpp" // For ImuSensor class
#include "port_discovery.hpp" // For port discovery

using namespace std::chrono; // For time measurement


// 1. Call this ONLY ONCE during initialization for each link
// Eigen::Matrix4d GetDHConstantMatrix(double d, double a, double alpha) {
//     double ca = std::cos(alpha);
//     double sa = std::sin(alpha);

//     Eigen::Matrix4d T_const;
//     T_const << 1.0,  0.0,  0.0,  a,
//                0.0,   ca,  -sa,  0.0,
//                0.0,   sa,   ca,  d,
//                0.0,  0.0,  0.0,  1.0;
//     return T_const;
// }

// // 2. Call this IN YOUR REAL-TIME LOOP
// // Extremely fast: Only 2 trig calls + 1 SIMD matrix multiplication
// Eigen::Matrix4d GetJointTransformFast(double theta, const Eigen::Matrix4d& T_const) {
//     double ct = std::cos(theta);
//     double st = std::sin(theta);

//     Eigen::Matrix4d Rot_z;
//     Rot_z <<  ct, -st, 0.0, 0.0,
//               st,  ct, 0.0, 0.0,
//              0.0, 0.0, 1.0, 0.0,
//              0.0, 0.0, 0.0, 1.0;

//     return Rot_z * T_const;
// }




int main() {
	RobotPorts ports = autoDiscoverDevices();
	if (ports.arm_port.empty() || ports.imu_port.empty()) return 1;

	sensor::ImuMultiplexer esp32_mux(ports.imu_port, B500000);
	esp32_mux.Start();

	std::this_thread::sleep_for(milliseconds(500)); // Wait for the IMU to start streaming


	// // We fetch the imu for the arm // Should set it up totally
	sensor::ImuState& imu_arm = esp32_mux.imu_array[0];
	imu_arm.SetMountingRotation(0.0, 0.0, 1.0, 0.0); // We rotate the imu 180 around x
	// We should add the DH constant into the imu
	// SO that we call the static once and the just call the simple func of theta.
    // 


	mycobot::MyCobotDirect robot;
	if (!robot.Connect(ports.arm_port)) return 1;
	robot.PowerOn();

	std::cout << "Homing robot to calibrate IMU...\n";
	robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
	// WaitUntilTargetReached(robot, {0, 0, 0, 0, 0, 0});
	// Dont like this sleep_for
	std::this_thread::sleep_for(milliseconds(2000)); // Wait for the IMU to
	imu_arm.Tare(); // Tare the IMU to set the current orientation as the reference


	std::array<double, 4> q1 = imu_arm.GetAlignedQuaternion();

	std::vector<double> pan_angles = {0.0 , 30.0, 60.0, 90.0, 120.0, 150.0, 168.0};
        std::vector<double> tilt_angles = {0.0, 30.0, 60.0, 90.0};
	for (double pan : pan_angles) {
		for (double tilt : tilt_angles) {
			mycobot::Angles target = {pan, tilt, 0, 0, 0, 0};
			robot.WriteAngles(target, 20);
			std::this_thread::sleep_for(milliseconds(2000)); // Wait for the robot to reach the target
		    std::array<double, 4> q1 = imu_arm.GetAlignedQuaternion();
            mycobot::Angles current_encoders;
            bool serial_valid = robot.GetAngles(current_encoders);
			std::cout << std::fixed << std::setprecision(2);
			std::cout << "Pan: " << std::setw(6) << pan << " Tilt: " << std::setw(6) << tilt
					  << " | IMU Quat: [" << std::setw(6) << q1[0] << ", " << std::setw(6) << q1[1]
					  << ", " << std::setw(6) << q1[2] << ", " << std::setw(6) << q1[3] << "]"
					  << " | Encoders: [" << std::setw(6) << current_encoders[0] << ", "
					  << std::setw(6) << current_encoders[1] << "]\n";
		}
	}
	// Where should the EKF be, here in line or?
	// We make a seperate header file?
    robot.WriteAngles({0, 0, 0, 0, 0, 0}, 30);
	std::this_thread::sleep_for(milliseconds(2000)); // Wait for the IMU to
    // WaitUntilTargetReached(robot, {0, 0, 0, 0, 0, 0});
    robot.StopRobot();
    esp32_mux.Stop();
    return 0;


}
