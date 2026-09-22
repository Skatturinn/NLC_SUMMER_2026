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

struct __attribute__((packed)) ImuDataPacket {
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

    std::atomic<uint32_t> last_timestamp{0};
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

    std::array<double, 4> q_mount_offset = {1.0, 0.0, 0.0, 0.0};
public:
    void UpdateData(const SensorData& data, uint32_t ts) {
        last_timestamp.store(ts); // atomic store to ensure thread safety
        quat_w.store(data.quat[0]);
        quat_x.store(data.quat[1]);
        quat_y.store(data.quat[2]);
        quat_z.store(data.quat[3]);

        accel_x.store(data.linAcc[0]);
        accel_y.store(data.linAcc[1]);
        accel_z.store(data.linAcc[2]);
    }

    uint32_t GetTimestamp() const { return last_timestamp.load(); }


    void SetMountingRotation(double w, double x, double y, double z) {
        double mag = std::sqrt(w*w + x*x + y*y + z*z); // length
        if (mag < 1e-6) {
            q_mount_offset = {1.0, 0.0, 0.0, 0.0};
        } else {
            q_mount_offset = {w/mag, x/mag, y/mag, z/mag}; // save normalized quaternion offset
        }
//        q_mount_offset = {w, x, y, z};
    }
    std::array<double, 4> GetNormalizedQuaternion() const {
        double w = quat_w.load(), x = quat_x.load(), y = quat_y.load(), z = quat_z.load();
        // make sure that magnitude is close to 1 in order to keep math stable
		double mag_check = w*w + x*x + y*y + z*z;
		if (std::abs(mag_check - 1) > 1e-6) {
			if (mag_check < 1e-6) return {1.0, 0.0, 0.0, 0.0};
			double mag = std::sqrt(mag_check);
        	return {w/mag, x/mag, y/mag, z/mag}; 
		}
        return {w, x, y, z}; 
    }

    std::array<double, 4> GetMountedQuaternion() const {
        std::array<double, 4> q_raw = GetNormalizedQuaternion();
        return MultiplyQuat(q_mount_offset, q_raw);
    }


    void SetAxisMapping(int x, int y, int z) {
        map_x = x;
        map_y = y;
        map_z = z;
    }


    std::array<double, 4> GetMappedQuaternion() const {
        double raw[4] = {quat_w.load(), quat_x.load(), quat_y.load(), quat_z.load()};
        
        auto apply_map = [&](int map_val) {
            return (map_val < 0 ? -1.0 : 1.0) * raw[std::abs(map_val)];
        };

        double w = raw[0];
        double x = apply_map(map_x);
        double y = apply_map(map_y);
        double z = apply_map(map_z);

        // double mag = std::sqrt(w*w + x*x + y*y + z*z);
        // if (mag < 1e-6) return {1.0, 0.0, 0.0, 0.0};
        
        // return {w/mag, x/mag, y/mag, z/mag}; 
		return {w, x, y, z};
    }
    // nonsense --- void SetKinematicOffsets(double L, double W) {
    //    length_L = L;
  //      width_W = W;
//    }

    void SetAxisInversion(bool invert_roll, bool invert_pitch, bool invert_yaw) {
        inv_roll = invert_roll ? -1.0 : 1.0;
        inv_pitch = invert_pitch ? -1.0 : 1.0;
        inv_yaw = invert_yaw ? -1.0 : 1.0;
    }
    // Configures how the IMU's physical axes map to the Robot's axes
   

    // Snapshots the current absolute orientation and sets it as the "Zero" frame
    void Tare() {
        auto nq = GetNormalizedQuaternion();
        tare_w = nq[0];
        tare_x = nq[1];
        tare_y = nq[2];
        tare_z = nq[3];
    }

    void TareQuat(const std::array<double, 4>& target_q) {
        // 1. Calculate the inverse of the target orientation
        std::array<double, 4> qExpInv = {target_q[0], -target_q[1], -target_q[2], -target_q[3]};

        // 2. Get the current raw IMU reading
        std::array<double, 4> qRaw = GetNormalizedQuaternion();

        // 3. Calculate the perfect offset (Q_tare = Q_raw * Q_enc^-1)
        // This is the exact same math you used beautifully in DynamicTare()
        std::array<double, 4> new_tare = MultiplyQuat(qRaw, qExpInv);

        // 4. Apply the new tare to the class variables
        tare_w = new_tare[0];
        tare_x = new_tare[1];
        tare_y = new_tare[2];
        tare_z = new_tare[3];
    }


    std::array<double, 4> GetAlignedQuaternion() const {
        //double w1 = tare_w, x1 = -tare_x, y1 = -tare_y, z1 = -tare_z;
        //auto mq = GetMappedQuaternion();
        //double w2 = mq[0], x2 = mq[1], y2 = mq[2], z2 = mq[3];

        //double w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
        //double x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
        //double y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
        //double z = w1*z2 + x1*y2 - y1*x2 + z1*w2;
        ////std::array<double, 4> q_tare_inv = {tare_w, -tare_x, -tare_y, -tare_z};
        ////std::array<double, 4> q_curr = GetMountedQuaternion();
        ////return MultiplyQuat(q_tare_inv, q_curr);
        std::array<double, 4> q_raw = GetNormalizedQuaternion();
        
        // 1. Relative rotation from home position in sensor space: Q_rel = Q_tare^-1 * Q_raw
        std::array<double, 4> q_tare_inv = {tare_w, -tare_x, -tare_y, -tare_z};
        std::array<double, 4> q_rel = MultiplyQuat(q_tare_inv, q_raw);

        // 2. Inverse of mounting rotation: Q_mount^-1 = [w, -x, -y, -z]
        std::array<double, 4> q_mount_inv = {
            q_mount_offset[0], 
           -q_mount_offset[1], 
           -q_mount_offset[2], 
           -q_mount_offset[3]
        };

        // 3. Conjugate by mounting offset to transform coordinate axes: Q_mount * Q_rel * Q_mount^-1
        std::array<double, 4> temp = MultiplyQuat(q_mount_offset, q_rel);
        return MultiplyQuat(temp, q_mount_inv);
    }

    void SetHardcodedTare(double w, double x, double y, double z) {
        tare_w = w;
        tare_x = x;
        tare_y = y;
        tare_z = z;
    }
    // Helper function for standard Quaternion multiplication
    std::array<double, 4> MultiplyQuat(const std::array<double, 4>& q1, const std::array<double, 4>& q2) const {
        // Returns hamilton product of two quaternions q1 and q2
		// https://en.wikipedia.org/wiki/Quaternion#Hamilton_product
		return {{
            q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3], 
            q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2],
            q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1],
            q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
        }};
    }

    // Tares the IMU at any position by trusting the current encoder angles 100%
    void DynamicTare(double pan_deg, double tilt_deg) { // currently is only for imu that takes in two encoder angles
        double pan_rad = pan_deg * M_PI / 180.0;
        double tilt_rad = tilt_deg * M_PI / 180.0;

        // 1. Convert encoder Pan (Z-axis) to a quaternion
        std::array<double, 4> qZ = {cos(pan_rad / 2.0), 0.0, 0.0, sin(pan_rad / 2.0)};
        
        // 2. Convert encoder Tilt (Y-axis) to a quaternion
        std::array<double, 4> qY = {cos(tilt_rad / 2.0), 0.0, sin(tilt_rad / 2.0), 0.0};

        // 3. Combine them to find the expected orientation (Q_enc = Q_Z * Q_Y)
        std::array<double, 4> qExpected = MultiplyQuat(qZ, qY);

        // 4. Calculate the inverse of the expected orientation
        std::array<double, 4> qExpInv = {qExpected[0], -qExpected[1], -qExpected[2], -qExpected[3]};

        // 5. Get the current raw IMU reading
        std::array<double, 4> qRaw = GetNormalizedQuaternion();

        // 6. Calculate the perfect offset (Q_tare = Q_raw * Q_enc^-1)
        std::array<double, 4> new_tare = MultiplyQuat(qRaw, qExpInv);

        // 7. Apply the new tare
        tare_w = new_tare[0];
        tare_x = new_tare[1];
        tare_y = new_tare[2];
        tare_z = new_tare[3];
    }


    // Calculates the mathematically corrected Quaternion relative to the Tare position
//    std::array<double, 4> GetAlignedQuaternion() const {
        // Q_aligned = Q_tare^(-1) * Q_current
        // The inverse of a unit quaternion Q[w,x,y,z] is [w,-x,-y,-z]
    //    double w1 = tare_w, x1 = -tare_x, y1 = -tare_y, z1 = -tare_z;
  //      double w2 = quat_w.load(), x2 = quat_x.load(), y2 = quat_y.load(), z2 = quat_z.load();
//
  //      // Quaternion Multiplication
//        double w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
      //  double x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    //    double y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
  //      double z = w1*z2 + x1*y2 - y1*x2 + z1*w2;
//
  //      return {w, x, y, z};
//    }

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
    // Restored exact user math for Y-axis
    std::array<double, 3> GetElbowPosition3D(double link_length_mm, double base_height_mm) const {
        auto q = GetAlignedQuaternion();
        
        // Final normalization to guarantee mathematically strict arm length
        // double mag = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        // double w = q[0]/mag, x = q[1]/mag, y = q[2]/mag, z = q[3]/mag;

        double w = q[0], x = q[1], y = q[2], z = q[3];


        double end_x = link_length_mm * (2.0*x*z + 2.0*y*w);
        double end_y = link_length_mm * (2.0*y*z - 2.0*x*w); //
        double end_z = link_length_mm * (1.0 - 2.0*x*x - 2.0*y*y);

        // 2. Add the physical base height to the Z-axis - invert y and z
        return {-end_x, end_y, end_z + base_height_mm};
    }
    // Normalized
    // Safely reads and enforces mathematical normalization to prevent vector shrinking
    //std::array<double, 4> GetNormalizedQuaternion() const {
        //double w = quat_w.load(), x = quat_x.load(), y = quat_y.load(), z = quat_z.load();
      //  double mag = std::sqrt(w*w + x*x + y*y + z*z);
    //
      //  if (mag < 1e-6) return {1.0, 0.0, 0.0, 0.0}; // Prevent divide by zero
    //
       // return {w/mag, x/mag, y/mag, z/mag}; 
     //   }
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
        for (size_t i = 0; i < length; i++) crc ^= data[i]; // bitwise XOR
        return crc;
    }

    void ThreadLoop() {
        fd = open(port.c_str(), // Open the port to communicate with esp32 
		O_RDWR // Read and writing
		| O_NOCTTY // No controlling terminal, 
		| O_SYNC // Synchronous communicatio, we finish each action at exectuion instead of chaching
	); 
        if (fd < 0) return;

        struct termios tty;
        tcgetattr(fd, &tty);
        cfsetospeed(&tty, baud_rate); // output speed set
        cfsetispeed(&tty, baud_rate); // input speed set
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // clear CSIZE and set it to 8 bits per byte
        tty.c_iflag &= ~IGNBRK; // tanslate to (dont) Ignore Break
        tty.c_lflag = 0; // turn of local flags
        tty.c_oflag = 0; // turn of output flags ( no processing on output)
        tty.c_cc[VMIN]  = 1; // Wait for atleast 1 byte
        tty.c_cc[VTIME] = 5; // give up after 0.5 seconds90ö
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tcsetattr(fd, TCSANOW, &tty); // commit all rules from tty to hardware
        tcflush(fd, TCIFLUSH); // purges input buffer

        const size_t PACKET_SIZE = sizeof(ImuDataPacket); // 121 bytes
        uint8_t buffer[1024];
        size_t buffer_len = 0;

        while (keep_running) {
            uint8_t chunk[256];
            int n = read(fd, // read from fd
				 chunk, // put into chunk array
				 sizeof(chunk)); // up to 256*bytes
            
            if (n > 0) {
                // for (int i = 0; i < n; i++) {
                //     if (buffer_len < sizeof(buffer)) buffer[buffer_len++] = chunk[i];
                // }
				if (buffer_len + n <= sizeof(buffer)) {
					memcpy(buffer + buffer_len, chunk, n);
					buffer_len += n;
				} else {
					// Buffer is completely full and out of sync. 
					// Flush it, but keep the fresh chunk we just read.					memcpy(buffer, chunk, n);
					memcpy(buffer, chunk, n);
					buffer_len = n;
					// continue; 
				}
                while (buffer_len >= PACKET_SIZE) { // buffer >= 121 bytes ( one full imu data packet)
                    size_t start_idx = 0;
                    bool found_header = false;
                    for (size_t i = 0; i <= buffer_len - PACKET_SIZE; i++) {
                        if (buffer[i] == 0xAA && buffer[i+1] == 0xBB) {
                            start_idx = i;
                            found_header = true;
                            break;
                        }
                    }

                    if (!found_header) { // Did not find header, leave last byte and re loop
                        buffer[0] = buffer[buffer_len - 1]; // first item is last byte
                        buffer_len = 1; // 1 byte left in buffer
                        break;
                    }

                    if (start_idx > 0) { // found header not at start, so we update buffer
                        memmove(buffer, // destination
							buffer + start_idx, // source
							buffer_len - start_idx // number of bytes
						);
                        buffer_len -= start_idx; // we update the number of bytes in buffer
                    }

                    if (buffer_len >= PACKET_SIZE) { 
                        ImuDataPacket* packet = reinterpret_cast<ImuDataPacket*>(buffer); // reinterpet buffer (byte array) as if it was ImuDataPacket, pointer to reference and not copy
                        uint8_t expected_crc = CalculateChecksum(buffer + 2, // buffer is a pointer to first byte, add 2 to skip the first 2 bytes (header)
							 PACKET_SIZE - 3 // skip the first two bytes (header) and the last byte (checksum)
							); 
                        
                        if (expected_crc == packet->checksum) {
                            imu_array[0].UpdateData(packet->sensors[0], packet->timestamp);
                            imu_array[1].UpdateData(packet->sensors[1], packet->timestamp);

                            memmove(buffer, buffer + PACKET_SIZE, buffer_len - PACKET_SIZE);
                            buffer_len -= PACKET_SIZE;
                        } else {
                            memmove(buffer, 
								buffer + 2, // We skip the first two bytes (header) since we had a error
								buffer_len - 2);
                            buffer_len -= 2;
                        }
                    }
					// Else the buffer is missing bytes for a packet, we reloop
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
            serial_thread = std::thread(&ImuMultiplexer::ThreadLoop, this); // making a new thread, & address of threadloop, this is the object we are calling it on (ImuMultiplexer)
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
