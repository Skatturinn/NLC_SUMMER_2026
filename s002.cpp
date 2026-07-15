#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <thread>
#include <iomanip>

#define ROBOT_PORT "/dev/ttyUSB1"

// Robot Command IDs
const uint8_t CMD_GET_ANGLES = 0x22;
const uint8_t CMD_WRITE_ANGLES = 0x21;

void write_cmd(int fd, uint8_t cmd_id, const std::vector<uint8_t>& params) {
    std::vector<uint8_t> packet = {0xFE, 0xFE, (uint8_t)(params.size() + 2), cmd_id};
    packet.insert(packet.end(), params.begin(), params.end());
    packet.push_back(0xFA);
    write(fd, packet.data(), packet.size());
}

// Helper to convert float angles to the robot's integer format
void send_angles(int fd, float a1, float a2, int speed) {
    std::vector<uint8_t> params;
    // MyCobot expects angles scaled by 100 as 2-byte integers
    auto add_angle = [&](float angle) {
        int16_t val = (int16_t)(angle * 100);
        params.push_back((val >> 8) & 0xFF);
        params.push_back(val & 0xFF);
    };
    
    add_angle(a1); add_angle(a2); 
    // Fill remaining joints with 0 if needed
    for(int i=0; i<4; i++) { params.push_back(0); params.push_back(0); }
    params.push_back(speed);
    
    write_cmd(fd, CMD_WRITE_ANGLES, params);
}

int main() {
    int robot_fd = open(ROBOT_PORT, O_RDWR | O_NOCTTY);
    
    // Command to Get Angles
    std::vector<uint8_t> empty;
    
    while (true) {
        // Request angles
        write_cmd(robot_fd, CMD_GET_ANGLES, empty);
        
        // Read response (MyCobot responses are usually ~15-20 bytes)
        uint8_t response[30];
        int n = read(robot_fd, response, sizeof(response));
        
        if (n > 5) {
            std::cout << "Robot responded with " << n << " bytes." << std::endl;
        }

        // Example: Jitter the arm
        send_angles(robot_fd, 0.0, 15.0, 50); 
        std::this_thread::sleep_for(std::chrono::seconds(2));
        send_angles(robot_fd, 0.0, -15.0, 50);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    return 0;
}
