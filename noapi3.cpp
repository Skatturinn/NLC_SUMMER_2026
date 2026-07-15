#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <chrono>
#include <thread>
#include <iomanip>

#define BAUD_RATE B1000000 
#define ROBOT_PORT "/dev/ttyUSB1"
#define IMU_PORT "/dev/ttyUSB0"

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
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    
    // Non-blocking read so we can flush the servo noise easily
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1; 
    
    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

void send_cmd(int fd, const std::vector<uint8_t>& cmd, const std::string& name) {
    std::cout << "Sending " << name << "...\n";
    write(fd, cmd.data(), cmd.size());
}

int main() {
    int fd = open_serial(ROBOT_PORT);
    if (fd < 0) {
        std::cerr << "Failed to open robot port on " << ROBOT_PORT << "\n";
        return 1;
    }

    std::cout << "Serial port open at 1000000 baud.\n";

    // 1. POWER ON
    // API: mycobot::MyCobot::I().PowerOn();
    send_cmd(fd, {0xFE, 0xFE, 0x02, 0x10, 0xFA}, "Power On");

    // API: mycobot::MyCobot::I().SleepSecond(1);
    std::cout << "Waiting 1 second for servos to boot...\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Clear buffer of any startup Dynamixel FF FF noise
    uint8_t junk[256];
    read(fd, junk, sizeof(junk));

    // 2. STOP ROBOT
    // API: mycobot::MyCobot::I().StopRobot();
    send_cmd(fd, {0xFE, 0xFE, 0x02, 0x34, 0xFA}, "Stop Robot");
    
    // API: std::this_thread::sleep_for(200ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 3. MOVE ARM (Joint 3 to 90 degrees)
    // 90 degrees * 100 = 9000. In Hex, 9000 is 0x2328.
    // Speed: 50 (0x32)
    std::vector<uint8_t> move_cmd = {
        0xFE, 0xFE, 0x0F, 0x22, 
        0x00, 0x00, // Joint 1: 0 deg
        0x00, 0x00, // Joint 2: 0 deg
        0x23, 0x28, // Joint 3: 90 deg (0x2328)
        0x00, 0x00, // Joint 4: 0 deg
        0x00, 0x00, // Joint 5: 0 deg
        0x00, 0x00, // Joint 6: 0 deg
        0x32,       // Speed: 50
        0xFA
    };
    send_cmd(fd, move_cmd, "Move Joint 3 to 90 degrees");

    // Wait to let the movement finish
    for(int i=0; i<5; i++) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Moving... (" << i+1 << "s)\n";
    }

    close(fd);
    std::cout << "Done.\n";
    return 0;
}
