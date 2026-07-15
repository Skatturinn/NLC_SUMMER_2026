#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <cstdio>

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUD_RATE B500000

// Force 1-byte alignment to perfectly match the ESP32 __packed__ attribute
#pragma pack(push, 1)
struct SensorData {
    double quat[4];       // w, x, y, z
    float linAcc[3];      // x, y, z
    float gravity[3];     // x, y, z
    uint8_t calibration;  // System, Gyro, Accel, Mag (2 bits each)
};

struct ImuDataPacket {
    uint8_t header[2];       // Sync bytes: 0xAA, 0xBB
    uint32_t timestamp;      // ESP32 system uptime
    SensorData sensors[2];   // Index 0: Arm, Index 1: Wrist
    uint8_t checksum;        
};
#pragma pack(pop)

// Calculate checksum exactly as the ESP32 does
uint8_t calculateChecksum(const uint8_t* data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
    }
    return crc;
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

    std::cout << "Reading and parsing IMU data. Press CTRL+C to stop...\n\n";

    const size_t PACKET_SIZE = sizeof(ImuDataPacket);
    uint8_t buffer[1024];
    size_t buffer_len = 0;

    while (true) {
        uint8_t chunk[256];
        int n = read(fd, chunk, sizeof(chunk));
        
        if (n > 0) {
            // Append incoming bytes to our buffer
            for (int i = 0; i < n; i++) {
                if (buffer_len < sizeof(buffer)) {
                    buffer[buffer_len++] = chunk[i];
                }
            }

            // Keep processing as long as we have enough bytes for a full packet
            while (buffer_len >= PACKET_SIZE) {
                // Search for the sync header (0xAA, 0xBB)
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
                    // Header not found. Keep the very last byte just in case it's 
                    // the first half of a header (0xAA) crossing the chunk boundary.
                    buffer[0] = buffer[buffer_len - 1];
                    buffer_len = 1;
                    break;
                }

                // If the packet didn't start at index 0, shift it to the front
                if (start_idx > 0) {
                    memmove(buffer, buffer + start_idx, buffer_len - start_idx);
                    buffer_len -= start_idx;
                }

                // Make sure we still have a full packet after shifting
                if (buffer_len >= PACKET_SIZE) {
                    ImuDataPacket* packet = reinterpret_cast<ImuDataPacket*>(buffer);
                    
                    // The checksum skips the 2 header bytes and the 1 checksum byte at the end
                    uint8_t expected_crc = calculateChecksum(buffer + 2, PACKET_SIZE - 3);
                    
                    if (expected_crc == packet->checksum) {
                        
                        // SUCCESS: We have a valid packet and can access real values
                        printf("Time: %u ms | Arm Quat W: %.3f, X: %.3f, Y: %.3f, Z: %.3f \n",
                               packet->timestamp,
                               packet->sensors[0].quat[0],
                               packet->sensors[0].quat[1],
                               packet->sensors[0].quat[2],
                               packet->sensors[0].quat[3]);

                        // ==========================================
                        // DO YOUR MATH AND KINEMATICS WORK HERE
                        // ==========================================

                        // Consume the valid packet from the buffer
                        memmove(buffer, buffer + PACKET_SIZE, buffer_len - PACKET_SIZE);
                        buffer_len -= PACKET_SIZE;
                    } else {
                        // Bad checksum. Consume only the 0xAA byte so the loop searches for the next header
                        memmove(buffer, buffer + 1, buffer_len - 1);
                        buffer_len -= 1;
                    }
                }
            }
        }
    }

    close(fd);
    return 0;
}
