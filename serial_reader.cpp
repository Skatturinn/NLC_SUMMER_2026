#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <cstring>

// --- CONFIGURATION ---
// In Linux, COM ports are usually /dev/ttyUSB0, /dev/ttyACM0, etc.
const char* SERIAL_PORT = "/dev/ttyUSB1"; 
const int BAUD_RATE = B500000;

// Force the compiler to pack the struct exactly as it sits in memory (no padding)
#pragma pack(push, 1)
struct SensorData {
    double quat[4];
    float linAcc[3];
    float grav[3];
    uint8_t calib;
};

struct Payload {
    uint32_t timestamp;
    SensorData arm;
    SensorData wrist;
    uint8_t checksum;
};
#pragma pack(pop)

const int PAYLOAD_SIZE = sizeof(Payload); // Should be 119 bytes

void print_sensor_data(const std::string& name, const SensorData& sensor) {
    uint8_t sys = (sensor.calib >> 6) & 0x03;
    uint8_t gyro = (sensor.calib >> 4) & 0x03;
    uint8_t accel = (sensor.calib >> 2) & 0x03;
    uint8_t mag = sensor.calib & 0x03;

    std::cout << "[" << name << "]\n";
    
    std::cout << std::fixed;
    std::cout << "  Quat: W:" << std::setprecision(3) << sensor.quat[0]
              << " X:" << sensor.quat[1] 
              << " Y:" << sensor.quat[2] 
              << " Z:" << sensor.quat[3] << "\n";
              
    std::cout << "  LAcc: X:" << std::setprecision(2) << sensor.linAcc[0]
              << " Y:" << sensor.linAcc[1] 
              << " Z:" << sensor.linAcc[2] << "\n";
              
    std::cout << "  Grav: X:" << std::setprecision(2) << sensor.grav[0]
              << " Y:" << sensor.grav[1] 
              << " Z:" << sensor.grav[2] << "\n";
              
    std::cout << "  Cal:  Sys:" << (int)sys 
              << " Gyro:" << (int)gyro 
              << " Accel:" << (int)accel 
              << " Mag:" << (int)mag << "\n";
}

int main() {
    std::cout << "Opening " << SERIAL_PORT << " at 500000 baud...\n";

    int fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Failed to open port: " << SERIAL_PORT << "\n";
        return 1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr\n";
        return 1;
    }

    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 5;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr\n";
        return 1;
    }

    tcflush(fd, TCIFLUSH);

    std::cout << "Dumping raw hex data. Press CTRL+C to stop...\n\n";

    uint8_t byte_read;
    int count = 0;

    while (true) {
        int n = read(fd, &byte_read, 1);
        if (n > 0) {
            // Print byte in 2-character hex format
            printf("%02X ", byte_read);
            
            // Print a new line every 16 bytes for readability
            count++;
            if (count % 16 == 0) {
                printf("\n");
            }
            fflush(stdout);
        }
    }

    close(fd);
    return 0;
}
