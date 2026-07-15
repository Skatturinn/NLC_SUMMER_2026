#ifndef IMU_SENSOR_HPP
#define IMU_SENSOR_HPP

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include <cstring>
#include <array>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sensor {

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

class ImuState {
private:
    std::atomic<double> quat_w{1.0};
    std::atomic<double> quat_x{0.0};
    std::atomic<double> quat_y{0.0};
    std::atomic<double> quat_z{0.0};
    
    std::atomic<double> accel_x{0.0};
    std::atomic<double> accel_y{0.0};
    std::atomic<double> accel_z{0.0};

    double offset_roll = 0.0;
    double offset_pitch = 0.0;
    double offset_yaw = 0.0;

    double length_L = 0.0; 
    double width_W = 0.0;  

public:
    void UpdateData(const SensorData& data) {
        quat_w.store(data.quat[0]);
        quat_x.store(data.quat[1]);
        quat_y.store(data.quat[2]);
        quat_z.store(data.quat[3]);

        accel_x.store(data.linAcc[0]);
        accel_y.store(data.linAcc[1]);
        accel_z.store(data.linAcc[2]);
    }

    void SetKinematicOffsets(double L, double W) {
        length_L = L;
        width_W = W;
    }

    void SetOrientationOffsets(double r, double p, double y) {
        offset_roll = r;
        offset_pitch = p;
        offset_yaw = y;
    }

    // Lazy evaluation logic - only do the math when asked
    double GetRoll() const { 
        double w = quat_w.load(), x = quat_x.load(), y = quat_y.load(), z = quat_z.load();
        double roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * 180.0 / M_PI;
        return roll - offset_roll; 
    }
    
    double GetPitch() const { 
        double w = quat_w.load(), x = quat_x.load(), y = quat_y.load(), z = quat_z.load();
        double pitch = asin(2.0 * (w * y - z * x)) * 180.0 / M_PI;
        return pitch - offset_pitch; 
    }
    
    double GetYaw() const { 
        double w = quat_w.load(), x = quat_x.load(), y = quat_y.load(), z = quat_z.load();
        double yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * 180.0 / M_PI;
        return yaw - offset_yaw; 
    }
    
    // Quaternion Getters
    std::array<double, 4> GetQuaternion() const {
        return {quat_w.load(), quat_x.load(), quat_y.load(), quat_z.load()};
    }

    // Acceleration Getters
    double GetAccelX() const { return accel_x.load(); }
    double GetAccelY() const { return accel_y.load(); }
    double GetAccelZ() const { return accel_z.load(); }
    
    double GetLengthL() const { return length_L; }
    double GetWidthW() const { return width_W; }
};

class ImuMultiplexer {
private:
    std::string port;
    int baud_rate;
    int fd;
    std::atomic<bool> keep_running{false};
    std::thread serial_thread;

    uint8_t CalculateChecksum(const uint8_t* data, size_t length) {
        uint8_t crc = 0;
        for (size_t i = 0; i < length; i++) crc ^= data[i];
        return crc;
    }

    void ThreadLoop() {
        fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) return;

        struct termios tty;
        tcgetattr(fd, &tty);
        cfsetospeed(&tty, baud_rate);
        cfsetispeed(&tty, baud_rate);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN]  = 1;
        tty.c_cc[VTIME] = 5;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tcsetattr(fd, TCSANOW, &tty);
        tcflush(fd, TCIFLUSH);

        const size_t PACKET_SIZE = sizeof(ImuDataPacket);
        uint8_t buffer[1024];
        size_t buffer_len = 0;

        while (keep_running) {
            uint8_t chunk[256];
            int n = read(fd, chunk, sizeof(chunk));
            
            if (n > 0) {
                for (int i = 0; i < n; i++) {
                    if (buffer_len < sizeof(buffer)) buffer[buffer_len++] = chunk[i];
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
                        uint8_t expected_crc = CalculateChecksum(buffer + 2, PACKET_SIZE - 3);
                        
                        if (expected_crc == packet->checksum) {
                            imu_array[0].UpdateData(packet->sensors[0]);
                            imu_array[1].UpdateData(packet->sensors[1]);

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
    }

public:
    std::array<ImuState, 2> imu_array;

    ImuMultiplexer(std::string port_name, int baud = B500000) 
        : port(port_name), baud_rate(baud), fd(-1) {}

    ~ImuMultiplexer() { Stop(); }

    void Start() {
        if (!keep_running) {
            keep_running = true;
            serial_thread = std::thread(&ImuMultiplexer::ThreadLoop, this);
        }
    }

    void Stop() {
        if (keep_running) {
            keep_running = false;
            if (serial_thread.joinable()) serial_thread.join();
        }
    }
};

} 
#endif
