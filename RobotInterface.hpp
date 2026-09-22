#ifndef ROBOT_INTERFACE_HPP
#define ROBOT_INTERFACE_HPP

#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <array>
#include <queue>
#include <mutex>
#include <atomic>
#include <memory>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "MyCobotDirect.hpp"
#include "ImuSensor.hpp"
#include "port_discovery.hpp"

// ============================================================================
// SHARED KINEMATICS MATH (HIGHLY OPTIMIZED)
// ============================================================================

// DH parameters taken from mycobot m280 elephant robotics documentation
// https://docs.elephantrobotics.com/docs/mycobot-pi-en/2-serialproduct/2.1-280/2.1.2.1%20Introduction%20of%20product%20parameters.html
// https://docs.elephantrobotics.com/docs/mycobot-pi-en/resourse/2-serialproduct/2.1-280/M5/2.1.1.1%E4%BA%A7%E5%93%81%E5%8F%82%E6%95%B0%E4%BB%8B%E7%BB%8D/SDH%E5%8F%82%E6%95%B0%E8%A1%A8.png
// Theta is the only variable, we pre compute the result from the other paramteres and create a function of theta for real time
struct DHLink {
    double ca, // cos(alpha)
	 sa, // sin(alpha) 
	 a, // length of the common normal. Assuming a revolute joint, this is the radius about previous z. (Description taken from Wiki)
	 d, // offset along previous z to the common normal
	 theta_offset;
};
// Dh parameters Wiki: https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters

static constexpr DHLink LINKS[6] = {
    { 0.0,  1.0,  0.0,      0.13122,  0.0 },            // J1: alpha = 90
    { 1.0,  0.0, -0.1104,   0.0,     -M_PI / 2.0 },     // J2: alpha = 0
    { 1.0,  0.0, -0.096,    0.0,      0.0 },            // J3: alpha = 0
    { 0.0,  1.0,  0.0,      0.0634,  -M_PI / 2.0 },     // J4: alpha = 90
    { 0.0, -1.0,  0.0,      0.07505,  M_PI / 2.0 },     // J5: alpha = -90
    { 1.0,  0.0,  0.0,      0.0456,   0.0 }             // J6: alpha = 0
};

inline Eigen::Matrix4d GetTransform(double q_deg, const DHLink& link) {
	// Takes in the angle from the servo in degrees and the respective parameters of that link
    double theta = (q_deg * M_PI / 180.0) + link.theta_offset;
    double ct = std::cos(theta);
    double st = std::sin(theta);
    
    Eigen::Matrix4d T;
    T << ct, -st * link.ca,  st * link.sa, link.a * ct,
         st,  ct * link.ca, -ct * link.sa, link.a * st,
         0,   link.sa,       link.ca,      link.d,
         0,   0,             0,            1;
    return T;
}

inline std::array<double, 4> ComputeDhQuaternion2(double q1_deg, double q2_deg) {
    static const Eigen::Quaterniond q_home = [] {
        Eigen::Matrix4d T = GetTransform(0.0, LINKS[0]) * GetTransform(0.0, LINKS[1]);
        Eigen::Quaterniond q(T.block<3, 3>(0, 0));
        q.normalize();
        return q;
    }();

    Eigen::Matrix4d T_curr = GetTransform(q1_deg, LINKS[0]) * GetTransform(q2_deg, LINKS[1]);
    Eigen::Quaterniond q_curr(T_curr.block<3, 3>(0, 0));
    q_curr.normalize();

    Eigen::Quaterniond q_rel = q_curr * q_home.conjugate();
    q_rel.normalize(); 
    if (q_rel.w() < 0.0) q_rel.coeffs() *= -1.0; 
    
    return { q_rel.w(), q_rel.x(), q_rel.y(), q_rel.z() };
}

inline std::array<double, 4> ComputeDhQuaternion5(const Eigen::Matrix<double, 6, 1>& enc_deg) {
    static const Eigen::Quaterniond q_home = [] {
        Eigen::Matrix4d T = GetTransform(0.0, LINKS[0]) * GetTransform(0.0, LINKS[1]) *
                            GetTransform(0.0, LINKS[2]) * GetTransform(0.0, LINKS[3]) *
                            GetTransform(0.0, LINKS[4]);
        Eigen::Quaterniond q(T.block<3, 3>(0, 0));
        q.normalize();
        return q;
    }();

    Eigen::Matrix4d T_curr = GetTransform(enc_deg(0), LINKS[0]) * GetTransform(enc_deg(1), LINKS[1]) *
                             GetTransform(enc_deg(2), LINKS[2]) * GetTransform(enc_deg(3), LINKS[3]) *
                             GetTransform(enc_deg(4), LINKS[4]);

    Eigen::Quaterniond q_curr(T_curr.block<3, 3>(0, 0));
    q_curr.normalize(); 

    Eigen::Quaterniond q_rel = q_curr * q_home.conjugate();
    q_rel.normalize(); 
    if (q_rel.w() < 0.0) q_rel.coeffs() *= -1.0; 
    
    return { q_rel.w(), q_rel.x(), q_rel.y(), q_rel.z() };
}

inline Eigen::Matrix4d ForwardKinematics(const Eigen::Matrix<double, 6, 1>& q_deg) {
    return GetTransform(q_deg(0), LINKS[0]) * GetTransform(q_deg(1), LINKS[1]) *
           GetTransform(q_deg(2), LINKS[2]) * GetTransform(q_deg(3), LINKS[3]) *
           GetTransform(q_deg(4), LINKS[4]) * GetTransform(q_deg(5), LINKS[5]);
}

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct MotorCommand {
    mycobot::Angles angles;
    int speed;
};

struct RobotState {
    mycobot::Angles raw_encoders = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    mycobot::Angles ukf_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    mycobot::Angles joint_velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    std::array<double, 4> raw_imu_q2 = {1.0, 0.0, 0.0, 0.0};
    std::array<double, 4> raw_imu_q5 = {1.0, 0.0, 0.0, 0.0};

    Eigen::Vector3d cartesian_pos = Eigen::Vector3d::Zero(); 
    Eigen::Quaterniond cartesian_orientation = Eigen::Quaterniond::Identity(); 
};
// ============================================================================
// 12-STATE UKF FILTER CLASS DEFINITION
// ============================================================================
class ArmUKF {
public:
    ArmUKF() {
        x_.setZero(); // [theta_1..6, omega_1..6]
        P_ = Eigen::Matrix<double, 12, 12>::Identity() * 0.1; 
    }

    void InitState(const mycobot::Angles& initial_encoders) {
        for (int i = 0; i < 6; i++) {
            x_(i) = initial_encoders[i];
            x_(i + 6) = 0.0;
        }
    }

    void Predict(double dt, const Eigen::Matrix<double, 6, 1>& u_cmd) {
        Eigen::Matrix<double, 12, 12> F = Eigen::Matrix<double, 12, 12>::Identity(); 
        for (int i = 0; i < 6; i++) {
            F(i, i + 6) = dt; // pos = pos + vel * dt 
			// pos is x_(0 to 5), vel is x_(6 to 11); 
			// we are adding dt to the F matrix in row 0 to 5 (pos) and column 6 to 11 (elocity)
        }

        for (int i = 0; i < 6; i++) {
            x_(i + 6) = (0.2 * x_(i + 6)) + (0.8 * u_cmd(i)); 
			// velocity = 0.2 * current_velocity + 0.8 * commanded_velocity
        }

        x_ = F * x_;
		// position = velocity * dt + position ()
        Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Identity() * 0.05;
		// Q is a diagonal matrix of 0.05
        P_ = F * P_ * F.transpose() + Q;
		// P is a diagonal matrix of 0.1 at init, we update
		// P = F * 0.1 * F transpoed, now the dt is in the 6-11 row and column 0-5 + 0.05
    }


    void UpdateIMU(const std::array<double, 4>& imu2_quat, const std::array<double, 4>& imu5_quat) {
        const int L = 12; 
        const int M = 8; 
        
        const double alpha = 1e-3;
        const double beta = 2.0;
        const double kappa = 0.0;
        const double lambda = alpha * alpha * (L + kappa) - L;

        Eigen::VectorXd Wm(2 * L + 1), Wc(2 * L + 1);
        Wm(0) = lambda / (L + lambda);
        Wc(0) = (lambda / (L + lambda)) + (1.0 - alpha * alpha + beta);
        for (int i = 1; i <= 2 * L; i++) {
            Wm(i) = 1.0 / (2.0 * (L + lambda));
            Wc(i) = Wm(i);
        }

        Eigen::MatrixXd X(L, 2 * L + 1);
        X.col(0) = x_;

        P_ += Eigen::Matrix<double, 12, 12>::Identity() * 1e-6; 
        Eigen::MatrixXd S = ((L + lambda) * P_).llt().matrixL();

        for (int i = 0; i < L; i++) {
            X.col(i + 1) = x_ + S.col(i);
            X.col(i + L + 1) = x_ - S.col(i);
        }

        Eigen::MatrixXd Z(M, 2 * L + 1);
        for (int i = 0; i < 2 * L + 1; i++) {
            Z.col(i) = MeasurementModelIMU(X.col(i));
            if (i > 0) {
                if (Z.col(i).head<4>().dot(Z.col(0).head<4>()) < 0) Z.col(i).head<4>() *= -1.0;
                if (Z.col(i).tail<4>().dot(Z.col(0).tail<4>()) < 0) Z.col(i).tail<4>() *= -1.0;
            }
        }

        Eigen::Matrix<double, 8, 1> z_pred = Eigen::Matrix<double, 8, 1>::Zero();
        for (int i = 0; i < 2 * L + 1; i++) z_pred += Wm(i) * Z.col(i);
        z_pred.head<4>().normalize();
        z_pred.tail<4>().normalize();

        Eigen::Matrix<double, 8, 8> Pzz = Eigen::Matrix<double, 8, 8>::Identity() * 0.05; 
        Eigen::Matrix<double, 12, 8> Pxz = Eigen::Matrix<double, 12, 8>::Zero();

        for (int i = 0; i < 2 * L + 1; i++) {
            Eigen::Matrix<double, 8, 1> z_diff = Z.col(i) - z_pred;
            Eigen::Matrix<double, 12, 1> x_diff = X.col(i) - x_;
            Pzz += Wc(i) * (z_diff * z_diff.transpose());
            Pxz += Wc(i) * (x_diff * z_diff.transpose());
        }

        Eigen::Matrix<double, 12, 8> K = Pxz * Pzz.inverse();

        Eigen::Matrix<double, 8, 1> z_meas;
        z_meas << imu2_quat[0], imu2_quat[1], imu2_quat[2], imu2_quat[3],
                  imu5_quat[0], imu5_quat[1], imu5_quat[2], imu5_quat[3];

        if (z_meas.head<4>().dot(z_pred.head<4>()) < 0) z_meas.head<4>() *= -1.0;
        if (z_meas.tail<4>().dot(z_pred.tail<4>()) < 0) z_meas.tail<4>() *= -1.0;

        x_ = x_ + K * (z_meas - z_pred);
        P_ = P_ - K * Pzz * K.transpose();
    }

    void UpdateEncoders(const mycobot::Angles& encoders) {
        const int L = 12; 
        const int M = 6; 
        
        const double alpha = 1e-3;
        const double beta = 2.0;
        const double kappa = 0.0;
        const double lambda = alpha * alpha * (L + kappa) - L;

        Eigen::VectorXd Wm(2 * L + 1), Wc(2 * L + 1);
        Wm(0) = lambda / (L + lambda);
        Wc(0) = (lambda / (L + lambda)) + (1.0 - alpha * alpha + beta);
        for (int i = 1; i <= 2 * L; i++) {
            Wm(i) = 1.0 / (2.0 * (L + lambda));
            Wc(i) = Wm(i);
        }

        Eigen::MatrixXd X(L, 2 * L + 1);
        X.col(0) = x_;

        P_ += Eigen::Matrix<double, 12, 12>::Identity() * 1e-6; 
        Eigen::MatrixXd S = ((L + lambda) * P_).llt().matrixL();

        for (int i = 0; i < L; i++) {
            X.col(i + 1) = x_ + S.col(i);
            X.col(i + L + 1) = x_ - S.col(i);
        }

        Eigen::MatrixXd Z(M, 2 * L + 1);
        for (int i = 0; i < 2 * L + 1; i++) {
            Z.col(i) = X.col(i).head<6>(); 
        }

        Eigen::Matrix<double, 6, 1> z_pred = Eigen::Matrix<double, 6, 1>::Zero();
        for (int i = 0; i < 2 * L + 1; i++) z_pred += Wm(i) * Z.col(i);

        Eigen::Matrix<double, 6, 6> Pzz = Eigen::Matrix<double, 6, 6>::Identity() * 0.01; 
        Eigen::Matrix<double, 12, 6> Pxz = Eigen::Matrix<double, 12, 6>::Zero();

        for (int i = 0; i < 2 * L + 1; i++) {
            Eigen::Matrix<double, 6, 1> z_diff = Z.col(i) - z_pred;
            Eigen::Matrix<double, 12, 1> x_diff = X.col(i) - x_;
            Pzz += Wc(i) * (z_diff * z_diff.transpose());
            Pxz += Wc(i) * (x_diff * z_diff.transpose());
        }

        Eigen::Matrix<double, 12, 6> K = Pxz * Pzz.inverse();

        Eigen::Matrix<double, 6, 1> z_meas;
        z_meas << encoders[0], encoders[1], encoders[2], encoders[3], encoders[4], encoders[5];

        x_ = x_ + K * (z_meas - z_pred);
        P_ = P_ - K * Pzz * K.transpose();
    }

    mycobot::Angles GetFilteredAngles() {
        return {x_(0), x_(1), x_(2), x_(3), x_(4), x_(5)};
    }
    
    mycobot::Angles GetFilteredVelocities() {
        return {x_(6), x_(7), x_(8), x_(9), x_(10), x_(11)};
    }

private:
    Eigen::Matrix<double, 12, 1> x_; 
    Eigen::Matrix<double, 12, 12> P_; 
    
    Eigen::Matrix<double, 8, 1> MeasurementModelIMU(const Eigen::Matrix<double, 12, 1>& state) {
        Eigen::Matrix<double, 6, 1> pos = state.head<6>();
        std::array<double, 4> q2 = ComputeDhQuaternion2(pos(0), pos(1));
        std::array<double, 4> q5 = ComputeDhQuaternion5(pos);
        
        Eigen::Matrix<double, 8, 1> z;
        z << q2[0], q2[1], q2[2], q2[3], q5[0], q5[1], q5[2], q5[3];
        return z;
    }
};

// ============================================================================
// HARDWARE INTERFACE
// ============================================================================
class RobotInterface {
private:
    mycobot::MyCobotDirect robot_;
    std::unique_ptr<sensor::ImuMultiplexer> imu_mux_;
    ArmUKF ukf_filter_;

    // Dual Thread Variables
    std::thread io_thread_;
    std::thread imu_thread_;
    std::atomic<bool> keep_running_{false};

    // Command State
    std::queue<MotorCommand> command_queue_;
    std::mutex queue_mutex_;
    std::atomic<bool> active_command_{false};
    mycobot::Angles target_angles_;
    int target_speed_ = 0;
    std::mutex target_state_mutex_;

    // Cross-Thread Encoder Data
    std::mutex encoder_mutex_;
    mycobot::Angles latest_encoders_;
	bool new_encoder_data_ready_ = false;
    std::atomic<bool> is_initialized_{false};

    // Final API State
    RobotState current_state_;
    std::mutex state_mutex_;

    // Slow Thread (20Hz): Writes commands and reads blocking serial bus
    void EncoderCommandThread() {
		// we create the interval for the thread loop to be 20hz
        const auto interval = std::chrono::milliseconds(50); 
		// we clock run time start
		auto next_time = std::chrono::steady_clock::now();
        while (keep_running_) {
			// we iterate next time
			next_time += interval;
			// Computation
			bool sent_command = false;

            {
                std::lock_guard<std::mutex> lock(queue_mutex_); // lock queue mutex
                if (!command_queue_.empty()) { // read command queue
                    MotorCommand next_cmd = command_queue_.front(); // take first command
                    command_queue_.pop(); // remove command
                    robot_.WriteAngles(next_cmd.angles, next_cmd.speed); // send command to robot by API
                    
                    std::lock_guard<std::mutex> target_lock(target_state_mutex_); // lock target
                    target_angles_ = next_cmd.angles; // write angles
                    target_speed_ = next_cmd.speed; // write speed
                    active_command_ = true; // what is this for?
                    sent_command = true; // local scope, we skip
                }
            }

            if (!sent_command) {
                mycobot::Angles encoders;
                bool valid = robot_.GetAngles(encoders);
                if (valid) {
                    std::lock_guard<std::mutex> lock(encoder_mutex_);
                    latest_encoders_ = encoders;
                    new_encoder_data_ready_ = true;
                }
            }


			// sleep until close to next time
			 
			auto sleep_time = next_time - std::chrono::milliseconds(2);
			// we sleep to safe resources, let the hardware rest.
            if (std::chrono::steady_clock::now() < sleep_time) {
				std::this_thread::sleep_until(sleep_time);
			}
			// we boot up and start running until next time.
			while (std::chrono::steady_clock::now() < next_time) {}
    }

    // Fast Thread (100Hz): Reads IMUs, calculates u, and runs UKF math
    void ImuUkfThread() {

		// make interval for 100hz loop:
        const auto interval = std::chrono::milliseconds(10);  // 100hz
        const double dt = std::chrono::duration<double>(interval).count(); // 0.01 seconds
		// mark next time to run ( now );
		auto next_time = std::chrono::steady_clock::now();
		// auto last_time = std::chrono::steady_clock::now();

        while (keep_running_) {

			next_time += interval; // iterate next time

			// Computation
            if (imu_mux_) { // imu_mux_ is the pointer to the IMU multiplexer, which is running the thread loop getting readings from IMU
                bool has_new_enc = false;
                mycobot::Angles enc_copy;
                {
					// lock_guard so no one else can access encoder data while this block runs
                    std::lock_guard<std::mutex> lock(encoder_mutex_);
                    if (new_encoder_data_ready_) { // Has the encoder thread read new data?
                        enc_copy = latest_encoders_;
                        has_new_enc = true;
                        new_encoder_data_ready_ = false;
                    }
                }

                // if (has_new_enc && !is_initialized_) {
                //     ukf_filter_.InitState(enc_copy); // we populate the X vector with 6 angles from encoders
                //     // Immediately populate the state so Start() can read i	qt!
                //     {
                //         std::lock_guard<std::mutex> state_lock(state_mutex_);
                //         current_state_.raw_encoders = enc_copy;
                //     }
                //     is_initialized_ = true; // we have populated X and dont need to again
                //     last_time = std::chrono::steady_clock::now();
                //     continue; 
                // }

                if (is_initialized_) { // Base case, everything is setup, see next case for setup
                    mycobot::Angles local_target;
                    int local_speed = 0;
                    bool is_active = active_command_; // we access the atmoic bool from encoder thread, true if command sent
                    

                    Eigen::Matrix<double, 6, 1> u_cmd_vel = Eigen::Matrix<double, 6, 1>::Zero();
                    mycobot::Angles current_ukf = ukf_filter_.GetFilteredAngles();
                    
                    if (is_active) {
						{
							// we fetch angles sent
							std::lock_guard<std::mutex> target_lock(target_state_mutex_); // target state mutex locked in block
							local_target = target_angles_; 
							local_speed = target_speed_;
						}
                        double max_err = 0.0;
                        std::array<double, 6> errors;
                        
                        for (int i = 0; i < 6; i++) { 
                            errors[i] = local_target[i] - current_ukf[i];
                            if (std::abs(errors[i]) > max_err) {
                                max_err = std::abs(errors[i]);
                            }
                        }

                        if (max_err > 1.0) { 
                            // Ensure speed is at least 1 to prevent division by zero
                            double safe_speed = std::max(1.0, static_cast<double>(local_speed));
                            double max_vel = (safe_speed / 100.0) * 160.0; // Assuming percentage is off max joint speed // https://www.elephantrobotics.com/en/mycobot-280-for-jetson-nano-specifications-en/ 
                            double estimated_time = max_err / max_vel; 
                            
                            for (int i = 0; i < 6; i++) {
                                u_cmd_vel(i) = errors[i] / estimated_time; // estimate velocity given error and time est
                            }
                        } else {
                            active_command_ = false; 
                        }
                    }

					// Read quaterions
                    std::array<double, 4> q_imu2 = imu_mux_->imu_array[0].GetAlignedQuaternion();
                    std::array<double, 4> q_imu5 = imu_mux_->imu_array[1].GetAlignedQuaternion();

                    ukf_filter_.Predict(dt, u_cmd_vel); // predict 
                    ukf_filter_.UpdateIMU(q_imu2, q_imu5); // then update imu?
                    
                    if (has_new_enc) {
                        ukf_filter_.UpdateEncoders(enc_copy);
                    }
                    
                    mycobot::Angles filtered_angles = ukf_filter_.GetFilteredAngles();
                    mycobot::Angles filtered_vels = ukf_filter_.GetFilteredVelocities();
                    
                    Eigen::Matrix<double, 6, 1> filtered_deg_vec;
                    for (int i = 0; i < 6; i++) {
                        filtered_deg_vec(i) = filtered_angles[i];
                    }

                    Eigen::Matrix4d fk_matrix = ForwardKinematics(filtered_deg_vec);
                    Eigen::Vector3d pos = fk_matrix.block<3, 1>(0, 3);
                    Eigen::Quaterniond rot(fk_matrix.block<3, 3>(0, 0));
                    rot.normalize();

                    std::lock_guard<std::mutex> lock(state_mutex_);
                    if (has_new_enc) current_state_.raw_encoders = enc_copy;
                    current_state_.ukf_angles = filtered_angles;
                    current_state_.joint_velocities = filtered_vels;
                    current_state_.raw_imu_q2 = q_imu2;
                    current_state_.raw_imu_q5 = q_imu5;
                    current_state_.cartesian_pos = pos;
                    current_state_.cartesian_orientation = rot;
                } else if (has_new_enc) {
                    ukf_filter_.InitState(enc_copy); // we populate the X vector with 6 angles from encoders
                    // Immediately populate the state so Start() can read i	qt!
                    {
                        std::lock_guard<std::mutex> state_lock(state_mutex_);
                        current_state_.raw_encoders = enc_copy;
                    }
                    is_initialized_ = true; // we have populated X and dont need to again
                    last_time = std::chrono::steady_clock::now();
                    // continue; 
				};
            }

			// sleep until next time
			auto sleep_time = next_time - std::chrono::milliseconds(1);
			if (std::chrono::steady_clock::now() < sleep_time) {
				std::this_thread::sleep_until(sleep_time);
			}
            // auto elapsed = std::chrono::steady_clock::now() - now;
            // if (elapsed < interval) {
            //     std::this_thread::sleep_for(interval - elapsed);
            // }
			while (std::chrono::steady_clock::now() < next_time) {}
        }
    }

public:
    RobotInterface() = default;
    ~RobotInterface() { Stop(); }

    int Start() {
        RobotPorts ports = autoDiscoverDevices(); // Dont know if this should be it's own header
        if (ports.arm_port.empty() || ports.imu_port.empty()) {
            std::cerr << "Failed to discover necessary ports." << std::endl;
            return 1;
        }

        imu_mux_ = std::make_unique<sensor::ImuMultiplexer>(ports.imu_port, B500000); // we make a pointer on the heap for our IMU class
        imu_mux_->Start(); // starts the IMU thread loop serial read
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Wait IMU thread to spin up properly
        
        imu_mux_->imu_array[0].SetMountingRotation(0.0, 1.0, 0.0, 0.0); // 180 degree rotation about x axis
        imu_mux_->imu_array[1].SetMountingRotation(0.7071, -0.7071, 0.0, 0.0); // -90 degree rotatio  about x axis, then 90 degree rotation about y axis

        if (!robot_.Connect(ports.arm_port, B1000000)) { // We connect through our MyCobotDirect api which mimcs allows us to set our own serial port
            std::cerr << "Failed to connect to MyCobot on port " << ports.arm_port << std::endl;
            return 1;
        }
        robot_.PowerOn(); 

        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // wait for arm to boot up

        // We start the background threads after the robot is connected and IMUs are initialized
        if (!keep_running_) {
            keep_running_ = true;
            io_thread_ = std::thread(&RobotInterface::EncoderCommandThread, this); // We start the arm thread
            imu_thread_ = std::thread(&RobotInterface::ImuUkfThread, this); // we start the Unscented Kalman Filter thread
        }

		std::cout << "Waiting for IO thread to fetch initial encoder data..." << std::flush;
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));

        // 2. WAIT FOR STATE TO POPULATE
        bool ready = false;
        RobotState initial_state;


		// while (!is_initialized_.load()) {
        //     std::cout << "." << std::flush;
        //     std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
        // }

        for(int i = 0; i < 30; ++i) { // 30 tries * 100ms = 3 seconds
            if (is_initialized_.load()) {
                initial_state = GetState();
                ready = true;
                std::cout << " Success!" << std::endl;
                break;
            }
            std::cout << "." << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Fixed from 1000ms
        }
        //for(int i = 0; i < 50; ++i) { // Give it up to 5 seconds
            // is_initialized_ is set to true by ImuUkfThread once it gets the first encoders
            //if (is_initialized_.load()) {
        //std::cout << "waiting" << std::endl;
        //while (!is_initialized_.load()) {
        //    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        //    std::cout << "." << std::flush;
        //}
        //initial_state = GetState();
        //ready = true;
          //      std::cout << " Success!" << std::endl;
                //break;
            //}
           // std::cout << "." << std::flush;
            //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        //}

        // 3. PERFORM DYNAMIC TARE
        if (ready) {
            std::array<double, 4> expected_q2 = ComputeDhQuaternion2(initial_state.raw_encoders[0], initial_state.raw_encoders[1]);
            
            Eigen::Matrix<double, 6, 1> enc_vec;
            for(int i = 0; i < 6; ++i) enc_vec(i) = initial_state.raw_encoders[i];
            std::array<double, 4> expected_q5 = ComputeDhQuaternion5(enc_vec);

            imu_mux_->imu_array[0].TareQuat(expected_q2);
            imu_mux_->imu_array[1].TareQuat(expected_q5);
            std::cout << "Dynamically tared IMUs to current encoder positions." << std::endl;
        } else {
            std::cerr << "\nFailed to read initial encoders after 5 seconds. Falling back to zero tare." << std::endl;
            imu_mux_->imu_array[0].Tare();
            imu_mux_->imu_array[1].Tare();
        }

        std::cout << "RobotInterface successfully started dual-thread execution." << std::endl;
        return 0;
    }
    
	void Home() {
        if (!keep_running_) {
            std::cerr << "Robot is not running. Call Start() first." << std::endl;
            return;
        }

        std::cout << "Sending robot to home position (0,0,0,0,0,0)..." << std::endl;
        mycobot::Angles zero_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        
        // Use thread-safe queue instead of direct serial writes
        SendAngleCommand(zero_angles, 30);

        auto start_time = std::chrono::steady_clock::now();
        bool reached_home = false;

        // Monitor state until the robot reaches zero
        while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(15)) {
            RobotState state = GetState();
            
            bool all_zero = true;
            for (int i = 0; i < 6; ++i) {
                if (std::abs(state.raw_encoders[i]) > 1.5) { // 1.5 degree tolerance
                    all_zero = false;
                    break;
                }
            }

            if (all_zero && !active_command_) {
                reached_home = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        if (reached_home) {
            std::cout << "Robot reached home. Taring IMUs to zero." << std::endl;
            // Use standard tare to zero out at the 0,0,0,0,0,0 position
            imu_mux_->imu_array[0].Tare();
            imu_mux_->imu_array[1].Tare();
        } else {
            std::cerr << "Timeout waiting for robot to reach home position." << std::endl;
        }
    }

    void Stop() {
        if (keep_running_) {
            keep_running_ = false;
            if (io_thread_.joinable()) io_thread_.join();
            if (imu_thread_.joinable()) imu_thread_.join();
        }
        
        if (imu_mux_) imu_mux_->Stop();
        
        robot_.StopRobot();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        robot_.Disconnect();
    }

    void SendAngleCommand(const mycobot::Angles& target, int speed = 30) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        command_queue_.push({target, speed});
    }

    RobotState GetState() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return current_state_;
    }
};
}

#endif
