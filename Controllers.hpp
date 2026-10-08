#ifndef NLC_CPP_LIB__CONTROLLERS_HPP_
#define NLC_CPP_LIB__CONTROLLERS_HPP_

#include <vector>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>

// ---------------------------------------------------------
// BASE CONTROLLER CLASS
// ---------------------------------------------------------
class controller {
public: 
    struct min_max {
        double min;
        double max;
    };

protected:
    Eigen::VectorXd measurements;
    Eigen::VectorXd setpoints;
    Eigen::VectorXd output;
    std::vector<min_max> boundaries;
    bool initialized = false;

public:
    virtual ~controller() = default; 
    virtual void compute() = 0; 
    
    bool init_system(
        const std::vector<double>& new_measurements,
        const std::vector<double>& new_setpoints,
        const std::vector<min_max>& new_boundaries
    ) {
        if (new_boundaries.size() != new_setpoints.size()) return false;
        
        size_t n = new_setpoints.size();
        measurements = Eigen::Map<const Eigen::VectorXd>(new_measurements.data(), n);
        setpoints = Eigen::Map<const Eigen::VectorXd>(new_setpoints.data(), n);
        boundaries = new_boundaries;
        output = Eigen::VectorXd::Zero(n);
        
        initialized = true;
        return true;
    }

    bool update_system(const std::vector<double>& new_measurements) {
        if (!initialized || new_measurements.size() != static_cast<size_t>(setpoints.size())) return false;
        measurements = Eigen::Map<const Eigen::VectorXd>(new_measurements.data(), new_measurements.size());
        return true;    
    }

    void update_setpoints(const std::vector<double>& new_setpoints) {
        setpoints = Eigen::Map<const Eigen::VectorXd>(new_setpoints.data(), new_setpoints.size());
    }

    double clamp(double val, min_max b) {
        return std::max(b.min, std::min(val, b.max));
    }

    std::vector<double> getOutputs() const { 
        std::vector<double> out_vec(output.data(), output.data() + output.size());
        return out_vec; 
    }
    
    bool is_initialized() const { return initialized; }
};

// ---------------------------------------------------------
// Multi-Dimensional PID Controller
// ---------------------------------------------------------
class pid_controller : public controller {
private:
    // Controller Parameters
    double K;  // Proportional Gain (K)
    double Ti; // Integral Time Constant
    double Td; // Derivative Time Constant
    double N;  // Derivative Filter Coefficient
    double b;  // Setpoint Weighting
    double Tt; // Tracking Time Constant for Anti-Windup (replaces 'timabil')
    double dt; // Time step (h)

    // State Variables
    Eigen::VectorXd I_term;
    Eigen::VectorXd prev_D;
    Eigen::VectorXd prev_y;

public:
    // Initialize with standard parameters mapping to your Python logic
    pid_controller(double K = 1.0, double Ti = 1.0, double Td = 0.1, 
                   double N = 10.0, double b = 1.0, double Tt = 0.5, double time_step = 0.1) 
        : K(K), Ti(Ti), Td(Td), N(N), b(b), Tt(Tt), dt(time_step) {}

    void compute() override {
        if (!initialized) return;

        size_t n = setpoints.size();

        // Initialize state vectors on the first run
        if (I_term.size() == 0) {
            I_term = Eigen::VectorXd::Zero(n);
            prev_D = Eigen::VectorXd::Zero(n);
            prev_y = measurements; // measurements == y
        }

        // 1. Proportional Term with Setpoint Weighting (b)
        // Python: P = K * (y - b * oskgildi) 
        // Note: For arm position control, standard error direction is (setpoint - y)
        Eigen::VectorXd P = K * ((b * setpoints) - measurements);

        // 2. Filtered Derivative on Measurement (Avoids Derivative Kick)
        // Python: D = (Td / (Td + N * h)) * (prev_D + K * N * (y - prev_y))
        Eigen::VectorXd y_diff = measurements - prev_y;
        Eigen::VectorXd D = (Td / (Td + N * dt)) * (prev_D - (K * N * y_diff)); 
        // Subtracted here to align with the standard (setpoint - y) error direction

        // 3. Calculate Unclamped PID Output
        Eigen::VectorXd PID_unclamped = P + I_term + D;

        // 4. Clamping (Output Saturation)
        Eigen::VectorXd PID_clamped = Eigen::VectorXd::Zero(n);
        for (size_t j = 0; j < n; ++j) {
            PID_clamped(j) = clamp(PID_unclamped(j), boundaries[j]);
        }

        // 5. Integral Update with Tracking Anti-Windup
        // Python: I += ((K * h) / Ti) * (y - oskgildi) + (h / timabil) * (duty_cycle - PID)
        Eigen::VectorXd error = setpoints - measurements;
        
        // Ensure Ti is not zero to avoid division by zero
        if (Ti > 0) {
            I_term += ((K * dt) / Ti) * error + (dt / Tt) * (PID_clamped - PID_unclamped);
        }

        // 6. Update states for the next loop (stak += 1)
        prev_D = D;
        prev_y = measurements;
        output = PID_clamped;
    }
};

// ---------------------------------------------------------
// Multi-Dimensional Secant Controller (Eigen-based)
// ---------------------------------------------------------
class secant : public controller {
private:
    double eta; // Learning rate
    Eigen::MatrixXd S_hat; 
    Eigen::VectorXd u_prev; 
    Eigen::VectorXd y_prev; 

public:
    // LOWERED learning rate from 1.0 to 0.1 to stop overreacting
    secant(double learning_rate = 0.1) 
        : eta(learning_rate) {}

    void compute() override {
        if (!initialized) return;

        size_t n = setpoints.size();

        if (S_hat.size() == 0) {
            // INITIALIZATION FIX: 
            // At Ts = 0.1s, a velocity of 1 rad/s results in 0.1 rad of movement.
            // Initializing to 0.1 matches the physical reality much better than 1.0.
            S_hat = Eigen::MatrixXd::Identity(n, n) * 0.1; 
            u_prev = Eigen::VectorXd::Zero(n);
            y_prev = measurements;
            return; 
        }

        Eigen::VectorXd delta_y = measurements - y_prev;
        Eigen::VectorXd expected_delta_y = S_hat * u_prev;
        Eigen::VectorXd z = u_prev; 
        
        double denominator = z.dot(u_prev);

        // Update System Matrix with a safety check against tiny denominators
        if (std::abs(denominator) > 1e-4) {
            Eigen::MatrixXd numerator = (delta_y - expected_delta_y) * z.transpose();
            S_hat = S_hat + (eta * numerator / denominator);
        }

        Eigen::VectorXd error = setpoints - measurements;

        // REGULARIZATION FIX (Damped Least Squares):
        // Solves the exploding inverse problem using: u = (S^T S + lambda I)^-1 S^T e
        double lambda = 0.05; // Damping factor (tune this: higher = more sluggish, lower = more aggressive)
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
        Eigen::MatrixXd S_T = S_hat.transpose();
        
        Eigen::MatrixXd damped_inverse = (S_T * S_hat + lambda * I).inverse() * S_T;
        Eigen::VectorXd u_curr = damped_inverse * error;

        // PROPORTIONAL GAIN FIX:
        // Softens the overall aggressiveness of the controller
        double K_p = 0.5; 
        u_curr = K_p * u_curr;

        for (size_t i = 0; i < n; ++i) {
            output(i) = clamp(u_curr(i), boundaries[i]);
        }

        y_prev = measurements;
        u_prev = output;
    }
};



#endif // NLC_CPP_LIB__CONTROLLERS_HPP_