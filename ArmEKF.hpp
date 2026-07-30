#ifndef ARM_EKF_HPP
#define ARM_EKF_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <array>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class ArmEKF {
private:
    Eigen::VectorXd x; // State: [q1, q2, v1, v2] (radians, rad/s)
    Eigen::MatrixXd P; // State Covariance (4x4)
    Eigen::MatrixXd Q; // Process Noise (4x4)
    
    Eigen::MatrixXd R_enc; // Encoder Noise (2x2)
    Eigen::MatrixXd R_imu; // IMU Quaternion Noise (4x4)

    Eigen::Matrix4d T_const_1;
    Eigen::Matrix4d T_const_2;
    Eigen::Quaterniond quat_home;

    static Eigen::Matrix4d GetDHConstantMatrix(double d, double a, double alpha) {
        double ca = std::cos(alpha);
        double sa = std::sin(alpha);
        Eigen::Matrix4d T_const;
        T_const << 1.0,  0.0,  0.0,  a,
                   0.0,   ca,  -sa,  0.0,
                   0.0,   sa,   ca,  d,
                   0.0,  0.0,  0.0,  1.0;
        return T_const;
    }

    static Eigen::Matrix4d GetJointTransformFast(double theta, const Eigen::Matrix4d& T_const) {
        double ct = std::cos(theta);
        double st = std::sin(theta);
        Eigen::Matrix4d Rot_z;
        Rot_z <<  ct, -st, 0.0, 0.0,
                  st,  ct, 0.0, 0.0,
                 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 1.0;
        return Rot_z * T_const;
    }

    Eigen::VectorXd h_imu(const Eigen::VectorXd& state) const {
        Eigen::Matrix4d T1 = GetJointTransformFast(state(0), T_const_1);
        Eigen::Matrix4d T2 = GetJointTransformFast(state(1) - M_PI / 2.0, T_const_2);
        Eigen::Matrix4d T_curr = T1 * T2;
        
        Eigen::Quaterniond quat_curr(T_curr.block<3, 3>(0, 0));
        Eigen::Quaterniond quat_expected = quat_home.conjugate() * quat_curr;

        Eigen::VectorXd z(4);
        z << quat_expected.w(), quat_expected.x(), quat_expected.y(), quat_expected.z();
        return z;
    }

    Eigen::MatrixXd ComputeIMUJacobian(const Eigen::VectorXd& state) const {
        double delta = 1e-4;
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 4);
        Eigen::VectorXd z0 = h_imu(state);

        for (int i = 0; i < 4; i++) {
            Eigen::VectorXd x_temp = state;
            x_temp(i) += delta;
            H.col(i) = (h_imu(x_temp) - z0) / delta;
        }
        return H;
    }

public:
    ArmEKF() {
        x = Eigen::VectorXd::Zero(4);
        P = Eigen::MatrixXd::Identity(4, 4) * 1.0;
        Q = Eigen::MatrixXd::Identity(4, 4) * 0.05;

        R_enc = Eigen::MatrixXd::Identity(2, 2) * 0.01;
        R_imu = Eigen::MatrixXd::Identity(4, 4) * 0.5;

        T_const_1 = GetDHConstantMatrix(0.13122, 0.0, M_PI / 2.0);
        T_const_2 = GetDHConstantMatrix(0.0, -0.1104, 0.0);

        Eigen::Matrix4d T_home_1 = GetJointTransformFast(0.0, T_const_1);
        Eigen::Matrix4d T_home_2 = GetJointTransformFast(-M_PI / 2.0, T_const_2);
        quat_home = Eigen::Quaterniond((T_home_1 * T_home_2).block<3, 3>(0, 0));
    }

    // --- A. PREDICT STEP ---
    void Predict(double dt) {
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(4, 4);
        F(0, 2) = dt;
        F(1, 3) = dt;
        
        // VELOCITY DAMPING: Prevents runaway drift when encoder packets drop
        F(2, 2) = 0.85; 
        F(3, 3) = 0.85;

        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    // --- B. ENCODER UPDATE ---
    void UpdateEncoders(double q1_rad, double q2_rad) {
        Eigen::VectorXd z_enc(2);
        z_enc << q1_rad, q2_rad;

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 4);
        H(0, 0) = 1.0;
        H(1, 1) = 1.0;

        Eigen::VectorXd y = z_enc - (H * x);
        Eigen::MatrixXd S = H * P * H.transpose() + R_enc;
        Eigen::MatrixXd K = P * H.transpose() * S.inverse();

        x = x + K * y;
        P = (Eigen::MatrixXd::Identity(4, 4) - K * H) * P;
    }

    // --- C. IMU UPDATE ---
    void UpdateIMU(const std::array<double, 4>& q_align) {
        Eigen::VectorXd z_imu(4);
        z_imu << q_align[0], q_align[1], q_align[2], q_align[3];

        Eigen::MatrixXd H = ComputeIMUJacobian(x);
        Eigen::VectorXd z_expected = h_imu(x);

        // QUATERNION DOUBLE-COVER FIX:
        // If dot product is negative, q and -q are on opposite hemispheres.
        // Flip sign so we take the shortest path on the sphere.
        if (z_imu.dot(z_expected) < 0.0) {
            z_imu = -z_imu;
        }

        Eigen::VectorXd y = z_imu - z_expected;
        Eigen::MatrixXd S = H * P * H.transpose() + R_imu;
        Eigen::MatrixXd K = P * H.transpose() * S.inverse();

        x = x + K * y;
        P = (Eigen::MatrixXd::Identity(4, 4) - K * H) * P;
    }
    // --- GETTERS ---
    double GetQ1() const { return x(0); }
    double GetQ2() const { return x(1); }
    Eigen::VectorXd GetState() const { return x; }

    // Returns [w, x, y, z] expected by EKF math
    std::array<double, 4> GetExpectedQuaternion() const {
        Eigen::VectorXd z = h_imu(x);
        return {z(0), z(1), z(2), z(3)};
    }
};

#endif // ARM_EKF_HPP
