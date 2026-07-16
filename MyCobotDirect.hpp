#ifndef MYCOBOT_DIRECT_HPP
#define MYCOBOT_DIRECT_HPP

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <array>
#include <chrono>
#include <thread>
#include <cmath>

namespace mycobot {

enum Axis : int { X = 1, Y, Z, RX, RY, RZ };
enum Joint : int { J1 = 1, J2, J3, J4, J5, J6 };

constexpr const int Axes = 6;
constexpr const int Joints = 6;
constexpr const int DefaultSpeed = 30;

using Coords = std::array<double, Axes>;
using Angles = std::array<double, Joints>;

class MyCobotDirect {
private:
    int fd;
    
    // Flushes the serial buffer so we only read the freshest responses
    void FlushBuffer() {
        if (fd >= 0) tcflush(fd, TCIFLUSH);
    }

    void WriteCommand(uint8_t cmd, const std::vector<uint8_t>& data = {}) {
        std::vector<uint8_t> packet;
        packet.push_back(0xFE);
        packet.push_back(0xFE);
        packet.push_back(data.size() + 2); // Length = data length + cmd byte + footer
        packet.push_back(cmd);
        packet.insert(packet.end(), data.begin(), data.end());
        packet.push_back(0xFA);
        
        write(fd, packet.data(), packet.size());
    }

    std::vector<uint8_t> ReadPacket(uint8_t expected_cmd, int timeout_ms = 500) {
        auto start = std::chrono::steady_clock::now();
        std::vector<uint8_t> buffer;
        uint8_t chunk[128];

        while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeout_ms)) {
            int n = read(fd, chunk, sizeof(chunk));
            if (n > 0) {
                buffer.insert(buffer.end(), chunk, chunk + n);
                
                for (size_t i = 0; i + 4 < buffer.size(); ++i) {
                    if (buffer[i] == 0xFE && buffer[i+1] == 0xFE) {
                        uint8_t len = buffer[i+2];
                        if (i + 2 + len < buffer.size()) {
                            uint8_t cmd = buffer[i+3];
                            uint8_t footer = buffer[i+2+len];
                            
                            if (footer == 0xFA && cmd == expected_cmd) {
                                std::vector<uint8_t> data;
                                for (size_t j = i + 4; j < i + 2 + len; ++j) {
                                    data.push_back(buffer[j]);
                                }
                                return data;
                            }
                        }
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return {}; // Timeout
    }

public:
    MyCobotDirect() : fd(-1) {}
    ~MyCobotDirect() { Disconnect(); }

    bool Connect(const std::string& port, int baud_rate = B1000000) {
        fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) return false;

        struct termios tty;
        tcgetattr(fd, &tty);
        cfsetospeed(&tty, baud_rate);
        cfsetispeed(&tty, baud_rate);
        
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_oflag &= ~OPOST;
        
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1; 
        
        tcsetattr(fd, TCSANOW, &tty);
        FlushBuffer();
        return true;
    }

    void Disconnect() {
        if (fd >= 0) {
            close(fd);
            fd = -1;
        }
    }

    // --- OVERALL RUNNING STATUS ---

    void PowerOn() {
        if (fd < 0) return;
        FlushBuffer();
        WriteCommand(0x10); // Power Up Command

        // Polling: Wait dynamically instead of sleep(1)
        for (int i = 0; i < 20; ++i) { 
            FlushBuffer();
            WriteCommand(0x51); // Check Power State
            std::vector<uint8_t> response = ReadPacket(0x51, 100);
            if (!response.empty() && response[0] == 0x01) {
                return; // Hardware confirmed it is on!
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void PowerOff() {
        if (fd >= 0) WriteCommand(0x11);
    }

    void StopRobot() {
        if (fd >= 0) WriteCommand(0x34);
    }

    // --- MDI PROGRAM CONTROL MODE ---

    bool IsMoving() {
        if (fd < 0) return false;
        FlushBuffer();
        WriteCommand(0x2B); // Movement Check Command
        std::vector<uint8_t> response = ReadPacket(0x2B, 100);
        if (!response.empty()) {
            return response[0] == 0x01; // 0x01 = Moving, 0x00 = Stopped
        }
        return false;
    }

    // Waits in time, this is sloppy and ideally should not be used
    void WaitMoveToFinish(int settle_time_ms = 1000) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); 
        while (IsMoving()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(settle_time_ms)); 
    }
    // This waits
    // Actively polls the hardware trajectory state and the encoders
    bool WaitMoveToAngles(const Angles& target, double tolerance = 2.0, int timeout_ms = 8000) {
        auto start_time = std::chrono::steady_clock::now();
        
        // Wait for the serial command to process and motors to begin moving
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); 
        
        while (true) {
            // 1. Hardware Check: Did the servo controller finish its trajectory?
            if (!IsMoving()) {
                break; // Robot stopped moving due to steady-state error
            }
            
            // 2. Mathematical Check: Did we reach the target within tolerance?
            Angles current = GetAngles();
            bool reached_goal = true;
            for (int i = 0; i < 6; i++) {
                if (std::abs(current[i] - target[i]) > tolerance) {
                    reached_goal = false;
                    break;
                }
            }
            if (reached_goal) {
                break;
            }
            
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() > timeout_ms) {
                return false; // Timeout triggered
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        
        // Brief mechanical settle once the physical movement stops
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
        return true; 
    }

    Angles GetAngles() {
        Angles angles = {0};
        if (fd < 0) return angles;

        FlushBuffer();
        WriteCommand(0x20); // Read Angles
        std::vector<uint8_t> data = ReadPacket(0x20);
        
        if (data.size() >= 12) {
            for (int i = 0; i < 6; ++i) {
                int16_t val = (data[i*2] << 8) | data[i*2 + 1];
                angles[i] = static_cast<double>(val) / 100.0;
            }
        }
        return angles;
    }

    void WriteAngles(const Angles& angles, int speed = DefaultSpeed) {
        if (fd < 0) return;
        std::vector<uint8_t> data;
        for (int i = 0; i < 6; ++i) {
            int16_t val = static_cast<int16_t>(angles[i] * 100);
            data.push_back((val >> 8) & 0xFF);
            data.push_back(val & 0xFF);
        }
        data.push_back(static_cast<uint8_t>(speed));
        WriteCommand(0x22, data); // Send Entire Angles
    }

    Coords GetCoords() {
        Coords coords = {0};
        if (fd < 0) return coords;

        FlushBuffer();
        WriteCommand(0x23); // Read Entire Coordinates
        std::vector<uint8_t> data = ReadPacket(0x23);
        
        if (data.size() >= 12) {
            for (int i = 0; i < 3; ++i) { // X, Y, Z (Divide by 10)
                int16_t val = (data[i*2] << 8) | data[i*2 + 1];
                coords[i] = static_cast<double>(val) / 10.0;
            }
            for (int i = 3; i < 6; ++i) { // RX, RY, RZ (Divide by 100)
                int16_t val = (data[i*2] << 8) | data[i*2 + 1];
                coords[i] = static_cast<double>(val) / 100.0;
            }
        }
        return coords;
    }

    void WriteCoords(const Coords& coords, int speed = DefaultSpeed) {
        if (fd < 0) return;
        std::vector<uint8_t> data;
        
        for (int i = 0; i < 3; ++i) { // X, Y, Z (Multiply by 10)
            int16_t val = static_cast<int16_t>(coords[i] * 10.0);
            data.push_back((val >> 8) & 0xFF);
            data.push_back(val & 0xFF);
        }
        for (int i = 3; i < 6; ++i) { // RX, RY, RZ (Multiply by 100)
            int16_t val = static_cast<int16_t>(coords[i] * 100.0);
            data.push_back((val >> 8) & 0xFF);
            data.push_back(val & 0xFF);
        }
        
        data.push_back(static_cast<uint8_t>(speed));
        data.push_back(0x01); // Mode: 0x01 per documentation
        WriteCommand(0x25, data); // Send Entire Coordinates
    }

    // --- IO CONTROL ---

    void SetBasicOut(int pin_number, int pin_signal) {
        if (fd < 0) return;
        WriteCommand(0xA0, {static_cast<uint8_t>(pin_number), static_cast<uint8_t>(pin_signal)});
    }

    void SetDigitalOut(int pin_number, int pin_signal) {
        if (fd < 0) return;
        WriteCommand(0x61, {static_cast<uint8_t>(pin_number), static_cast<uint8_t>(pin_signal)});
    }
};

} // namespace mycobot
#endif
