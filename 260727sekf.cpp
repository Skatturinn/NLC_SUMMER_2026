#include <iostream>
#include <random>
#include <Eigen/Dense>
#include <iomanip>

int main() {
    // 1. Initial State [angle, velocity]
    // We start completely blind, assuming 0 angle and 0 velocity.
    Eigen::Vector2d x(0.0, 0.0);
    
    // 2. Initial Uncertainty (P)
    // We tell the filter we are highly uncertain about our starting state.
    Eigen::Matrix2d P = Eigen::Matrix2d::Identity() * 1000.0;
    
    // 3. Process Noise (Q)
    // How much we trust our physics model. We keep this low.
    Eigen::Matrix2d Q = Eigen::Matrix2d::Identity() * 0.01;
    
    // 4. Measurement Noise (R)
    // How much we trust our sensor. We set this high because our sensor is terrible.
    Eigen::Matrix<double, 1, 1> R;
    R << 10.0; 
    
    // 5. Measurement mapping (H)
    // We only have a sensor for the angle (the 1st variable in our state).
    Eigen::Matrix<double, 1, 2> H;
    H << 1.0, 0.0;
    
    // --- SIMULATION SETUP ---
    double dt = 0.1; // 100ms loop time
    double true_velocity = 10.0; // The motor is actually moving 10 deg/sec
    double true_angle = 0.0;
    
    // Random noise generator to simulate a bad sensor
    std::default_random_engine generator;
    std::normal_distribution<double> noise(0.0, 3.0); // +/- 3 degrees of random jitter
    
    std::cout << "Time | True Ang | Noisy Sensor | EKF Filtered Ang | EKF Est Vel\n";
    std::cout << "-----------------------------------------------------------------\n";
    
    for (int i = 1; i <= 20; i++) {
        // --- A. SIMULATE THE REAL WORLD ---
        true_angle += true_velocity * dt;
        double noisy_measurement = true_angle + noise(generator);
        
        // --- B. EKF PREDICT STEP ---
        Eigen::Matrix2d F;
        F << 1.0, dt,
             0.0, 1.0;
             
        x = F * x; // Predict where we are
        P = F * P * F.transpose() + Q; // Predict our new uncertainty
        
        // --- C. EKF UPDATE STEP ---
        Eigen::Matrix<double, 1, 1> z;
        z << noisy_measurement;
        
        Eigen::Matrix<double, 1, 1> y = z - (H * x); // The Residual (Reality vs Prediction)
        Eigen::Matrix<double, 1, 1> S = H * P * H.transpose() + R;
        Eigen::Matrix<double, 2, 1> K = P * H.transpose() * S.inverse(); // The Kalman Gain
        
        x = x + K * y; // Final correction
        P = (Eigen::Matrix2d::Identity() - K * H) * P;
        
        // --- D. PRINT RESULTS ---
        std::cout << std::fixed << std::setprecision(2);
        std::cout << std::setw(4) << i * dt << " | "
                  << std::setw(8) << true_angle << " | "
                  << std::setw(12) << noisy_measurement << " | "
                  << std::setw(16) << x(0) << " | "
                  << std::setw(11) << x(1) << "\n";
    }
    return 0;
}
