#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <iomanip>
#include <vector>
#include <array>
#include <queue>
#include <mutex>
#include <atomic>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "MyCobotDirect.hpp"
#include "ImuSensor.hpp"
#include "port_discovery.hpp"

// Structure to hold a queued movement command
struct MotorCommand {
    mycobot::Angles angles;
    int speed;
};

// Structure to hold the latest physical state, fully initialized
struct RobotState {
    mycobot::Angles encoders = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> dh_quaternion_j2 = {1.0, 0.0, 0.0, 0.0};
    std::array<double, 4> dh_quaternion_j5 = {1.0, 0.0, 0.0, 0.0};
    bool is_moving = false;
};

class RobotIOThread {
private:
    mycobot::MyCobotDirect& robot_;
    std::atomic<bool> keep_running_{false};
    std::thread thread_;

    // Command Queue Data
    std::queue<MotorCommand> command_queue_;
    std::mutex queue_mutex_;

    // Current State Data
    RobotState current_state_;
    std::mutex state_mutex_;

    Eigen::Matrix4d BuildDHMatrix(double theta, double d, double a, double alpha) {
        Eigen::Matrix4d T;
        double ct = std::cos(theta), st = std::sin(theta);
        double ca = std::cos(alpha), sa = std::sin(alpha);
        T << ct, -st * ca,  st * sa, a * ct,
             st,  ct * ca, -ct * sa, a * st,
             0,   sa,       ca,      d,
             0,   0,        0,       1;
        return T;
    }

    std::array<double, 4> ComputeDhQuaternion2(double q1_deg, double q2_deg) {
        double q1 = q1_deg * M_PI / 180.0;
        double q2 = q2_deg * M_PI / 180.0;
        
        Eigen::Matrix4d T1_home = BuildDHMatrix(0.0, 0.13122, 0.0, M_PI / 2.0);
        Eigen::Matrix4d T2_home = BuildDHMatrix(-M_PI / 2.0, 0.0, -0.1104, 0.0);
        Eigen::Quaterniond q_home((T1_home * T2_home).block<3, 3>(0, 0));
        q_home.normalize(); // Ensure unit magnitude

        Eigen::Matrix4d T1_curr = BuildDHMatrix(q1, 0.13122, 0.0, M_PI / 2.0);
        Eigen::Matrix4d T2_curr = BuildDHMatrix(q2 - M_PI / 2.0, 0.0, -0.1104, 0.0);
        Eigen::Quaterniond q_curr((T1_curr * T2_curr).block<3, 3>(0, 0));
        q_curr.normalize();

        Eigen::Quaterniond q_rel = q_curr * q_home.conjugate();
        q_rel.normalize();
        if (q_rel.w() < 0.0) q_rel.coeffs() *= -1.0; 
        
        return { q_rel.w(), q_rel.x(), q_rel.y(), q_rel.z() };
    }

    std::array<double, 4> ComputeDhQuaternion5(const mycobot::Angles& enc) {
        double q1 = enc[0] * M_PI / 180.0;
        double q2 = enc[1] * M_PI / 180.0;
        double q3 = enc[2] * M_PI / 180.0;
        double q4 = enc[3] * M_PI / 180.0;
        double q5 = enc[4] * M_PI / 180.0;

        // Multiply progressively to ensure stable float math
        Eigen::Matrix4d T_home = Eigen::Matrix4d::Identity();
        T_home *= BuildDHMatrix(0.0, 0.13122, 0.0, M_PI / 2.0);
        T_home *= BuildDHMatrix(-M_PI / 2.0, 0.0, -0.1104, 0.0);
        T_home *= BuildDHMatrix(0.0, 0.0, -0.096, 0.0);
        T_home *= BuildDHMatrix(-M_PI / 2.0, 0.0634, 0.0, M_PI / 2.0);
        T_home *= BuildDHMatrix(M_PI / 2.0, 0.07505, 0.0, -M_PI / 2.0);

        Eigen::Matrix4d T_curr = Eigen::Matrix4d::Identity();
        T_curr *= BuildDHMatrix(q1, 0.13122, 0.0, M_PI / 2.0);
        T_curr *= BuildDHMatrix(q2 - M_PI / 2.0, 0.0, -0.1104, 0.0);
        T_curr *= BuildDHMatrix(q3, 0.0, -0.096, 0.0);
        T_curr *= BuildDHMatrix(q4 - M_PI / 2.0, 0.0634, 0.0, M_PI / 2.0);
        T_curr *= BuildDHMatrix(q5 + M_PI / 2.0, 0.07505, 0.0, -M_PI / 2.0);

        Eigen::Quaterniond q_home(T_home.block<3, 3>(0, 0));
        q_home.normalize(); // Force vector normalization

        Eigen::Quaterniond q_curr(T_curr.block<3, 3>(0, 0));
        q_curr.normalize(); // Force vector normalization

        Eigen::Quaterniond q_rel = q_curr * q_home.conjugate();
        q_rel.normalize(); // Force vector normalization
        
        if (q_rel.w() < 0.0) q_rel.coeffs() *= -1.0; 
        
        return { q_rel.w(), q_rel.x(), q_rel.y(), q_rel.z() };
    }

    void ThreadLoop() {
        const auto interval = std::chrono::milliseconds(50); // 20Hz loop

        while (keep_running_) {
            auto start_time = std::chrono::steady_clock::now();

            // 1. Process any pending commands in the queue
            bool has_command = false;
            MotorCommand next_command;
            
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (!command_queue_.empty()) {
                    next_command = command_queue_.front();
                    command_queue_.pop();
                    has_command = true;
                }
            }

            // If we found a command, send it to the hardware
            if (has_command) {
                robot_.WriteAngles(next_command.angles, next_command.speed);
                std::this_thread::sleep_for(std::chrono::milliseconds(5)); // Brief bus rest
            }

            // 2. Read the current hardware state
            mycobot::Angles encoders;
            bool valid = robot_.GetAngles(encoders);
            
            if (valid) {
                std::array<double, 4> dh_quat_j2 = ComputeDhQuaternion2(encoders[0], encoders[1]);
                std::array<double, 4> dh_quat_j5 = ComputeDhQuaternion5(encoders);
                bool moving = robot_.IsMoving();
                
                // Safely update the shared state
                std::lock_guard<std::mutex> lock(state_mutex_);
                current_state_.encoders = encoders;
                current_state_.dh_quaternion_j2 = dh_quat_j2;
                current_state_.dh_quaternion_j5 = dh_quat_j5;
                current_state_.is_moving = moving;
            }

            // 3. Sleep for remainder of the 50ms interval to maintain 20Hz
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed < interval) {
                std::this_thread::sleep_for(interval - elapsed);
            }
        }
    }

public:
    RobotIOThread(mycobot::MyCobotDirect& robot) : robot_(robot) {}
    ~RobotIOThread() { Stop(); }

    void Start() {
        if (!keep_running_) {
            keep_running_ = true;
            thread_ = std::thread(&RobotIOThread::ThreadLoop, this);
        }
    }

    void Stop() {
        if (keep_running_) {
            keep_running_ = false;
            if (thread_.joinable()) thread_.join();
        }
    }

    void QueueMovement(const mycobot::Angles& target, int speed) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        command_queue_.push({target, speed});
    }

    RobotState GetState() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return current_state_;
    }
};

int main() {
    RobotPorts ports = autoDiscoverDevices();
    if (ports.arm_port.empty() || ports.imu_port.empty()) return 1;

    // 1. Init IMU multiplexer
    sensor::ImuMultiplexer esp32_mux(ports.imu_port, B500000);
    esp32_mux.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
    
    // Assign mapped IMUs
    sensor::ImuState& imu_j2 = esp32_mux.imu_array[0];
    sensor::ImuState& imu_j5 = esp32_mux.imu_array[1]; 
    imu_j2.SetMountingRotation(0.0, 1.0, 0.0, 0.0);
    imu_j5.SetMountingRotation(0.0, 1.0, 0.0, 0.0);

    // 2. Init Robot and the new RobotIOThread
    mycobot::MyCobotDirect robot;
    if (!robot.Connect(ports.arm_port)) return 1;
    robot.PowerOn();

    RobotIOThread robot_thread(robot);
    robot_thread.Start();

    // 3. Homing Sequence 
    std::cout << "Homing robot to calibrate IMUs...\n";
    robot_thread.QueueMovement({0, 0, 0, 0, 0, 0}, 30);
    
    // Wait until home is reached
    std::this_thread::sleep_for(std::chrono::milliseconds(3000)); 
    imu_j2.Tare();
    imu_j5.Tare();

    // 4. Trajectory Loop (Restored full lengths for Pan and Tilt)
    std::vector<double> pan_angles = {0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 168.0};
    std::vector<double> tilt_angles = {0.0, 30.0, 60.0, 90.0};
    std::vector<double> j5_angles = {0.0, 45.0, 90.0};

    for (double pan : pan_angles) {
        for (double tilt : tilt_angles) {
            for (double j5 : j5_angles) {
                mycobot::Angles target = {pan, tilt, 0, 0, j5, 0};
                
                // Push command to the background thread
                robot_thread.QueueMovement(target, 20);
                
                // Allow movement to finish executing
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                
                // Fetch states directly
                std::array<double, 4> q_imu_j5 = imu_j5.GetAlignedQuaternion();
                RobotState final_state = robot_thread.GetState();
                std::array<double, 4> q_dh_j5 = final_state.dh_quaternion_j5;
                
                // Print the output format for Joint 5 offset calculations
                std::cout << std::fixed << std::setprecision(2);
                std::cout << "\nTarget [J1, J2, J5]: [" << std::setw(6) << pan << ", " << std::setw(6) << tilt << ", " << std::setw(6) << j5 << "]\n";
                
                std::cout << "  IMU5 Quat: [" << std::setw(6) << q_imu_j5[0] << ", " << std::setw(6) << q_imu_j5[1]
                          << ", " << std::setw(6) << q_imu_j5[2] << ", " << std::setw(6) << q_imu_j5[3] << "]\n";
                          
                std::cout << "  DH5  Quat: [" << std::setw(6) << q_dh_j5[0]  << ", " << std::setw(6) << q_dh_j5[1]
                          << ", " << std::setw(6) << q_dh_j5[2]  << ", " << std::setw(6) << q_dh_j5[3]  << "]\n";
            }
        }
    }

    // 5. Cleanup
    robot_thread.QueueMovement({0, 0, 0, 0, 0, 0}, 30);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    
    robot_thread.Stop();
    esp32_mux.Stop();
    robot.StopRobot();

    return 0;
}
