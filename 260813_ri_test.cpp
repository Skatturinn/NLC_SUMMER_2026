#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <iomanip>

// We include the RobotInterface header to connect, control, and read state from the MyCobot robot and IMUs
#include "RobotInterface.hpp"

int main() {
    // We create a instance of the RobotInterface class // from the header imported above
    RobotInterface robot_api;

    std::cout << "Starting Robot Interface...\n";

	// We call .Start() // Line 495
	if (robot_api.Start() != 0) {
        std::cerr << "Failed to start RobotInterface. Exiting.\n";
        return 1;
    }

    // 3. Homing Sequence
    std::cout << "Homing robot...\n";
    mycobot::Angles home_target = {0, 0, 0, 0, 0, 0};
    robot_api.SendAngleCommand(home_target, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(4000));

    // 4. Trajectory Loop Data
    std::vector<double> pan_angles = {0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 168.0};
    std::vector<double> tilt_angles = {0.0, 30.0, 60.0};

    // 5. Execute Test Trajectory
    for (double pan : pan_angles) {
        for (double tilt : tilt_angles) {
            mycobot::Angles target = {pan, tilt, 0, 0, 0, 0};
            
            // Queue the command asynchronously
            robot_api.SendAngleCommand(target, 20);
            
            // Wait for movement to finish and the UKF filter to settle
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            
            // Fetch the fully synchronized state from the interface
            RobotState state = robot_api.GetState();
            
            // Print the output format for verification
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "\nTarget [J1, J2]: [" << std::setw(6) << pan << ", " << std::setw(6) << tilt << "]\n";
            
            std::cout << "  Raw Encoders: [" << std::setw(6) << state.raw_encoders[0] << ", " 
                      << std::setw(6) << state.raw_encoders[1] << ", " << std::setw(6) << state.raw_encoders[2] << ", " 
                      << std::setw(6) << state.raw_encoders[3] << ", " << std::setw(6) << state.raw_encoders[4] << ", " 
                      << std::setw(6) << state.raw_encoders[5] << "]\n";

            std::cout << "  UKF Angles:   [" << std::setw(6) << state.ukf_angles[0] << ", " 
                      << std::setw(6) << state.ukf_angles[1] << ", " << std::setw(6) << state.ukf_angles[2] << ", " 
                      << std::setw(6) << state.ukf_angles[3] << ", " << std::setw(6) << state.ukf_angles[4] << ", " 
                      << std::setw(6) << state.ukf_angles[5] << "]\n";
                      
            std::cout << "  UKF Velocity: [" << std::setw(6) << state.joint_velocities[0] << ", " 
                      << std::setw(6) << state.joint_velocities[1] << ", " << std::setw(6) << state.joint_velocities[2] << ", " 
                      << std::setw(6) << state.joint_velocities[3] << ", " << std::setw(6) << state.joint_velocities[4] << ", " 
                      << std::setw(6) << state.joint_velocities[5] << "]\n";
                      
            std::cout << "  Cartesian X:   " << std::setw(6) << state.cartesian_pos.x() << " meters\n";
            std::cout << "  Cartesian Y:   " << std::setw(6) << state.cartesian_pos.y() << " meters\n";
            std::cout << "  Cartesian Z:   " << std::setw(6) << state.cartesian_pos.z() << " meters\n";
        }
    }

    // 6. Cleanup
    std::cout << "\nTest complete. Homing and shutting down...\n";
    robot_api.SendAngleCommand(home_target, 30);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    
    robot_api.Stop();

    return 0;
}
