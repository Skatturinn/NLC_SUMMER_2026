#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <chrono>
#include <thread>

// Change this to B115200 if B1000000 doesn't work
#define BAUD_RATE B1000000 
#define ROBOT_PORT "/dev/ttyUSB0"

int open_serial(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) return -1;

    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);
    
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    
    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

int main() {
    int fd = open_serial(ROBOT_PORT);
    if (fd < 0) {
        std::cerr << "Failed to open robot port." << std::endl;
        return 1;
    }

    // Protocol: Header(0xFE, 0xFE), Len(0x02), Cmd(0x22), Footer(0xFA)
    uint8_t get_angles_cmd[] = {0xFE, 0xFE, 0x02, 0x22, 0xFA};

    while (true) {
        write(fd, get_angles_cmd, sizeof(get_angles_cmd));
        
        uint8_t response[32];
        int n = read(fd, response, sizeof(response));
        
        if (n > 0) {
            std::cout << "Received " << n << " bytes: ";
            for(int i = 0; i < n; i++) printf("%02X ", response[i]);
            std::cout << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return 0;
}
