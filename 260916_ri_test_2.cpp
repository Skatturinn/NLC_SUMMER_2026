#include <iostream>
#include <chrono>
#include <thread>
// #include <vector>
// #include <iomanip>


#include "RobotInterface.hpp"

void printState(const RobotInterface& robot_api) {
	const auto interval = std::chrono::milliseconds(10); // 100hz might be too fast to see

	while (true) {
		
	}
	RobotState state = robot_api.GetState()
}

int main() {

	// RobotInterface robot_api;

	// if (robot_api.Start() != 0) {
	// 	std::cerr << "Failed to start RobotInterface. Exiting.\n";
	// 	return 1;
	// }
	// std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	// std::chrono::milliseconds(1000);
	auto last = std::chrono::steady_clock::now();

	for (int i = 0; i < 100; ++i) {
		auto now = std::chrono::steady_clock::now();
		double dt = std::chrono::duration<double>(now - last).count();
		std::cout << "\rCurrent i: " << i << "; " << "Elapsed time: " << dt << "s;" << std::flush;
		last = now;

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	std::cout << "\n Lok." << std::endl;

	return 0;
}