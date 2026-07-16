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

    // The reference quaternion used to "Tare" or "Re-orient" the IMU
    double tare_w = 1.0;
    double tare_x = 0.0;
    double tare_y = 0.0;
    double tare_z = 0.0;

    double length_L = 0.0; 
    double width_W = 0.0;

    double inv_roll = 1.0;
    double inv_pitch = 1.0;
    double inv_yaw = 1.0;

    // Axis mapping configuration (Default: 1=X, 2=Y, 3=Z)
    int map_x = 1;
    int map_y = 2;
    int map_z = 3;


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

    void SetAxisInversion(bool invert_roll, bool invert_pitch, bool invert_yaw) {
        inv_roll = invert_roll ? -1.0 : 1.0;
        inv_pitch = invert_pitch ? -1.0 : 1.0;
        inv_yaw = invert_yaw ? -1.0 : 1.0;
    }
    // Configures how the IMU's physical axes map to the Robot's axes
    void SetAxisMapping(int x, int y, int z) {
        map_x = x;
        map_y = y;
        map_z = z;
    }


    // Snapshots the current absolute orientation and sets it as the "Zero" frame
    void Tare() {
        tare_w = quat_w.load();
        tare_x = quat_x.load();
        tare_y = quat_y.load();
        tare_z = quat_z.load();
    }

    // Calculates the mathematically corrected Quaternion relative to the Tare position
    std::array<double, 4> GetAlignedQuaternion() const {
        // Q_aligned = Q_tare^(-1) * Q_current
        // The inverse of a unit quaternion Q[w,x,y,z] is [w,-x,-y,-z]
        double w1 = tare_w, x1 = -tare_x, y1 = -tare_y, z1 = -tare_z;
        double w2 = quat_w.load(), x2 = quat_x.load(), y2 = quat_y.load(), z2 = quat_z.load();

        // Quaternion Multiplication
        double w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
        double x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
        double y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
        double z = w1*z2 + x1*y2 - y1*x2 + z1*w2;

        return {w, x, y, z};
    }

    // --- LAZY EULER EVALUATION ---
    // Converts the ALIGNED quaternion to Euler angles only when called
    double GetRoll() const { 
        auto q = GetAlignedQuaternion();
        double r = atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])) * 180.0 / M_PI;
        return r * inv_roll; 
    }
    
    double GetPitch() const { 
        auto q = GetAlignedQuaternion();
        double p = asin(2.0 * (q[0] * q[2] - q[3] * q[1])) * 180.0 / M_PI;
        return p * inv_pitch; 
    }
    
    double GetYaw() const { 
        auto q = GetAlignedQuaternion();
        double y = atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])) * 180.0 / M_PI;
        return y * inv_yaw; 
    }



    // Endpoint
    // Calculates the 3D endpoint using pure quaternion vector rotation + base offset
    // Calculates the 3D endpoint using pure quaternion vector rotation + base offset
    std::array<double, 3> GetElbowPosition3D(double link_length_mm, double base_height_mm) const {
        // double link_length_mm = 110.4;
        // double base_height_mm = 131.56;
        auto q = GetAlignedQuaternion();
        double w = q[0], x = q[1], y = q[2], z = q[3];

        // 1. Rotate the arm length vector (L, 0, 0) by the IMU quaternion
        double end_x = link_length_mm * (1.0 - 2.0*y*y - 2.0*z*z);
        double end_y = link_length_mm * (2.0*x*y + 2.0*z*w);
        double end_z = link_length_mm * (2.0*x*z - 2.0*y*w);

        // 2. Add the physical base height to the Z-axis - invert y and z
        return {end_x, -end_y, -end_z + base_height_mm};
    }
    // --- RAW GETTERS ---
    std::array<double, 4> GetRawQuaternion() const {
        return {quat_w.load(), quat_x.load(), quat_y.load(), quat_z.load()};
    }

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

} // namespace sensor
#endif
