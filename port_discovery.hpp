#ifndef PORT_DISCOVERY_HPP
#define PORT_DISCOVERY_HPP

#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <chrono>

#include "MyCobotDirect.hpp"

namespace fs = std::filesystem;

struct RobotPorts {
    std::string arm_port;
    std::string imu_port;
};

inline RobotPorts autoDiscoverDevices() {
    RobotPorts found_ports;
    std::vector<std::string> candidate_ports;

    // 1. Scan /dev for USB serial devices
    try {
        for (const auto& entry : fs::directory_iterator("/dev")) {
            std::string filename = entry.path().filename().string();
            if (filename.find("ttyUSB") == 0 || filename.find("ttyACM") == 0) {
                candidate_ports.push_back(entry.path().string());
            }
        }
    } catch (...) {
        std::cerr << "Filesystem scanning error." << std::endl;
    }

    std::cout << "Scanning " << candidate_ports.size() << " candidate serial ports..." << std::endl;

    for (const std::string& port : candidate_ports) {
        std::cout << "  Testing port: " << port << "..." << std::endl;
        
        int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd == -1) continue;

        struct termios tty;
        tcgetattr(fd, &tty);
        cfsetospeed(&tty, B500000);
        cfsetispeed(&tty, B500000);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 2; // 200ms timeout
        tty.c_cflag |= (CLOCAL | CREAD);
        tcsetattr(fd, TCSANOW, &tty);
        tcflush(fd, TCIFLUSH);

        bool is_imu = false;
        auto start = std::chrono::steady_clock::now();
        uint8_t prev_byte = 0x00;

        // FAST SCAN: Just look for the raw 0xAA 0xBB header stream (Max 800ms wait)
        while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(800)) {
            uint8_t buf[128];
            int n = read(fd, buf, sizeof(buf));
            for(int i = 0; i < n; ++i) {
                if (prev_byte == 0xAA && buf[i] == 0xBB) {
                    is_imu = true;
                    break;
                }
                prev_byte = buf[i];
            }
            if (is_imu) break;
        }
        close(fd);

        if (is_imu) {
            std::cout << "--> Identified IMU ESP32 on " << port << std::endl;
            found_ports.imu_port = port;
        } else {
            // Confirm it's the arm
            mycobot::MyCobotDirect test_arm;
            if (test_arm.Connect(port, B1000000)) {
                std::cout << "--> Identified myCobot Arm on " << port << std::endl;
                found_ports.arm_port = port;
                test_arm.Disconnect();
            }
        }
    }

    return found_ports;
}

#endif
