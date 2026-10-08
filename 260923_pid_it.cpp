// lets do a remote demo and test

#include <iostream>
#include <chrono>

// Our system, U input -> Y output.
#include "RobotInterface.hpp"
// Our controll logic
#include "Controllers.hpp"

int main() {
	// we get the robot interface class
	RobotInterface robot_api;
	// we try to start.
	if (robot_api.Start() != 0) {
		std::cerr << "Failed to start RobotInterface. Exiting.\n";
		return 1;
	}


	// Lets remake the Node from ROS2 logic

	pid_controller pid_cpp{0.5, 0.0, 0.075, 10.0, 1.0, 0.5, 0.1}; // K, Ti, Td, N, b, Tt, Ts




	// i believe this should just be a 100hz loop
	// it gets the state, compute and sends command.
	// this might be too fast for our robot, also this raises the issue of reading and writing at same time.
	




	// it started. now we can use
	// robot_api.SendAngleCommand(mycobot::Angles, 20);


}