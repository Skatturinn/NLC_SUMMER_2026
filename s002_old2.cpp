#include <QCoreApplication>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>

// Include the myCobot API
#include "mycobot/MyCobot.hpp"

#define SERIAL_PORT "/dev/ttyUSB1"
#define BAUD_RATE B500000

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Global atomics for thread-safe data sharing between the Robot and the IMU
std::atomic<double> current_pan_deg{0.0};
std::atomic<double> current_tilt_deg{0.0};

#pragma pack(push, 1)
struct SensorData {
    double quat[4];       
    float linAcc[3];      
    float gravity[3];     
    uint8_t calibration;  
};

struct ImuDataPacket {
    uint8_t header[2];       
    uint32_t timestamp;      
    SensorData sensors[2];   
    uint8_t checksum;        
};
#pragma pack(pop)

uint8_t calculateChecksum(const uint8_t* data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
    }
    return crc;
}

int main(int argc, char** argv) {
    // 1. THIS MUST BE THE VERY FIRST LINE
    QCoreApplication app(argc, argv);

    // 2. INITIALIZE ROBOT FIRST (Exactly as it was in your working jog script)
    std::cout << "Initializing MyCobot on ttyUSB0...\n";
    mycobot::MyCobot mc = mycobot::MyCobot::I();
    mc.PowerOn();
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    if (!mc.IsControllerConnected()) {
        std::cerr << "WARNING: MyCobot is NOT connected!\n";
    } else {
        std::cout << "MyCobot connected successfully!\n";
    }

    // 3. SPAWN A DEDICATED ROBOT THREAD
    // This runs in the background and safely updates the angles without blocking the IMU
    std::thread robot_thread([&mc]() {
        while (true) {
            mycobot::Angles angles = mc.GetAngles();
            current_pan_deg.store(angles[0]);
            current_tilt_deg.store(angles[1]);
            // Rest for 200ms (5Hz) to prevent overwhelming the serial buffer
            std::this_thread::sleep_for(std::chrono::milliseconds(200)); 
        }
    });
    robot_thread.detach(); // Let it run independently

    // 4. INITIALIZE THE IMU
    std::cout << "Opening " << SERIAL_PORT << " for IMU...\n";
    int fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Failed to open port: " << SERIAL_PORT << "\n";
        return 1;
    }

    struct termios tty;
    tcgetattr(fd, &tty);
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
    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIFLUSH);

    std::cout << "Reading and parsing IMU data. Press CTRL+C to stop...\n\n";

    const size_t PACKET_SIZE = sizeof(ImuDataPacket);
    uint8_t buffer[1024];
    size_t buffer_len = 0;

    uint32_t last_time = 0;
    double last_tilt_rad = 0.0;

    while (true) {
        // Keep the Qt event loop happy so the robot thread doesn't crash
        QCoreApplication::processEvents();

        uint8_t chunk[256];
        int n = read(fd, chunk, sizeof(chunk));
        
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                if (buffer_len < sizeof(buffer)) {
                    buffer[buffer_len++] = chunk[i];
                }
            }

            while (buffer_len >= PACKET_SIZE) {
                size_t start_idx = 0;
                bool found_header = false;
                
                for (size_t i = 0; i <= buffer_len - PACKET_SIZE; i++) {
                    if (buffer[i] == 0xAA && buffer[i+1] == 0xBB) {
                        start_idx = i;
                        found_header = true;
                        break;
                    }
                }

                if (!found_header) {
                    buffer[0] = buffer[buffer_len - 1];
                    buffer_len = 1;
                    break;
                }

                if (start_idx > 0) {
                    memmove(buffer, buffer + start_idx, buffer_len - start_idx);
                    buffer_len -= start_idx;
                }

                if (buffer_len >= PACKET_SIZE) {
                    ImuDataPacket* packet = reinterpret_cast<ImuDataPacket*>(buffer);
                    uint8_t expected_crc = calculateChecksum(buffer + 2, PACKET_SIZE - 3);
                    
                    if (expected_crc == packet->checksum) {
                        
                        // FETCH THE LATEST ANGLE FROM THE BACKGROUND THREAD
                        double tiltDeg = current_tilt_deg.load();
                        double tiltRad = tiltDeg * (M_PI / 180.0);

                        if (last_time != 0 && packet->timestamp > last_time) {
                            double dt_sec = (packet->timestamp - last_time) / 1000.0;
                            double delta_tilt = tiltRad - last_tilt_rad;
                            double omega = std::fabs(delta_tilt / dt_sec);

                            float ax = packet->sensors[0].linAcc[0];
                            float ay = packet->sensors[0].linAcc[1];
                            float az = packet->sensors[0].linAcc[2];
                            double lin_accel_mag = std::sqrt(ax*ax + ay*ay + az*az);

                            if (omega > 0.15) { 
                                double estimated_length_mm = (lin_accel_mag / (omega * omega)) * 1000.0;
                                printf("Time: %u | Arm W: %.2f | Tilt: %.1f deg | Omega: %.2f rad/s | Accel: %.2f m/s^2 | CALC LENGTH: %.1f mm\n",
                                       packet->timestamp,
                                       packet->sensors[0].quat[0],
                                       tiltDeg,
                                       omega,
                                       lin_accel_mag,
                                       estimated_length_mm);
                            } else {
                                printf("Time: %u | Arm W: %.2f | Tilt: %.1f deg | (Move tilt joint faster to tune length)\n",
                                       packet->timestamp,
                                       packet->sensors[0].quat[0],
                                       tiltDeg);
                            }
                        }

                        last_time = packet->timestamp;
                        last_tilt_rad = tiltRad;

                        memmove(buffer, buffer + PACKET_SIZE, buffer_len - PACKET_SIZE);
                        buffer_len -= PACKET_SIZE;
                    } else {
                        memmove(buffer, buffer + 1, buffer_len - 1);
                        buffer_len -= 1;
                    }
                }
            }
        }
    }

    close(fd);
    mc.StopRobot();
    return 0;
}
