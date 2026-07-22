#ifndef PORT_DISCOVERY_HPP
#define PORT_DISCOVERY_HPP

#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "MyCobotDirect.hpp"
#include "ImuSensor.hpp"

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

    // 2. PASS 1: Identify the IMU ESP32 by testing for valid incoming 119-byte packets at 500000 baud
    for (const std::string& port : candidate_ports) {
        std::cout << "Testing port for IMU stream (500000 baud): " << port << "..." << std::endl;

        int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd != -1) {
            struct termios tty;
            tcgetattr(fd, &tty);
            cfsetospeed(&tty, B500000);
            cfsetispeed(&tty, B500000);
            tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
            tty.c_iflag &= ~IGNBRK;
            tty.c_lflag = 0;
            tty.c_oflag = 0;
            tty.c_cc[VMIN]  = 0;
            tty.c_cc[VTIME] = 2; // 200ms timeout per read check
            tcsetattr(fd, TCSANOW, &tty);
            tcflush(fd, TCIFLUSH);

            // Give the ESP32 a brief window to stream data frames
            int valid_packets_seen = 0;
            uint8_t buffer[256];
            size_t buffer_len = 0;
            auto start_time = std::chrono::steady_clock::now();

            while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(1500)) {
                uint8_t chunk[64];
                int n = read(fd, chunk, sizeof(chunk));
                if (n > 0) {
                    for (int i = 0; i < n; i++) {
                        if (buffer_len < sizeof(buffer)) buffer[buffer_len++] = chunk[i];
                    }

                    // Check if we have accumulated enough bytes for a full packet (119 bytes)
                    while (buffer_len >= sizeof(sensor::ImuDataPacket)) {
                        // Look for the 0xAA 0xBB header inside the accumulated buffer
                        size_t start_idx = 0;
                        bool found = false;
                        for (size_t i = 0; i <= buffer_len - sizeof(sensor::ImuDataPacket); i++) {
                            if (buffer[i] == 0xAA && buffer[i+1] == 0xBB) {
                                start_idx = i;
                                found = true;
                                break;
                            }
                        }

                        if (!found) {
                            buffer_len = 0;
                            break;
                        }

                        if (start_idx > 0) {
                            memmove(buffer, buffer + start_idx, buffer_len - start_idx);
                            buffer_len -= start_idx;
                        }

                        if (buffer_len >= sizeof(sensor::ImuDataPacket)) {
                            sensor::ImuDataPacket* pkt = reinterpret_cast<sensor::ImuDataPacket*>(buffer);
                            
                            // Simple checksum validation
                            uint8_t crc = 0;
                            const uint8_t* ptr = reinterpret_cast<const uint8_t*>(buffer);
                            for (size_t j = 2; j < sizeof(sensor::ImuDataPacket) - 1; ++j) {
                                crc ^= ptr[j];
                            }

                            if (crc == pkt->checksum) {
                                valid_packets_seen++;
                                memmove(buffer, buffer + sizeof(sensor::ImuDataPacket), buffer_len - sizeof(sensor::ImuDataPacket));
                                buffer_len -= sizeof(sensor::ImuDataPacket);
                            } else {
                                memmove(buffer, buffer + 1, buffer_len - 1);
                                buffer_len -= 1;
                            }
                        }
                    }
                }
                if (valid_packets_seen >= 2) break; // Confirmed active stream!
            }
            close(fd);

            if (valid_packets_seen >= 2) {
                std::cout << "--> Identified IMU ESP32 on " << port << std::endl;
                found_ports.imu_port = port;
                break;
            }
        }
    }

    // 3. PASS 2: Assign the remaining port to the MyCobot Arm (tested at 1000000 baud)
    for (const std::string& port : candidate_ports) {
        if (port == found_ports.imu_port) {
            continue;
        }

        std::cout << "Testing remaining port for MyCobot Arm (1000000 baud): " << port << "..." << std::endl;
        mycobot::MyCobotDirect test_arm;
        if (test_arm.Connect(port, B1000000)) {
            mycobot::Angles test_angles = test_arm.GetAngles();
            test_arm.Disconnect();

            // If it responds without crashing, it's the arm
            std::cout << "--> Identified myCobot Arm on " << port << std::endl;
            found_ports.arm_port = port;
            break;
        }
    }

    return found_ports;
}

#endif
