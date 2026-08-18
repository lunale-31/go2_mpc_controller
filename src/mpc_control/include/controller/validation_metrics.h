#pragma once

#include <Eigen/Dense>

#include <cstddef>
#include <limits>

class ValidationMetrics {
public:
    using BodyState = Eigen::Matrix<double, 6, 1>;      // [px, py, pz, vx, vy, vz]
    using JointTorque = Eigen::Matrix<double, 12, 1>;   // 12 joint torques
    using GRFVector = Eigen::Matrix<double, 12, 1>;     // [Fx,Fy,Fz] x 4 legs
    using ResidualVector = Eigen::Matrix<double, 28, 1>;
    using InnovationVector = Eigen::Matrix<double, 28, 1>;
    using InnovationCovariance = Eigen::Matrix<double, 28, 28>;

    struct MPCSummary {
        // Tracking / stability
        double height_rmse{0.0};
        double steady_state_height_error{0.0};
        double roll_rms{0.0};                  // rad
        double pitch_rms{0.0};                 // rad
        double xy_drift{0.0};                  // m, displacement from first to latest XY sample

        // Control input metrics
        double control_effort{0.0};             // integral ||tau||^2 dt
        double torque_rms{0.0};                 // Nm, optional easier-to-interpret companion metric
        double control_smoothness{0.0};         // integral ||d(tau)/dt||^2 dt
        double torque_rate_rms{0.0};            // Nm/s, optional companion metric

        // GRF distribution: FR, FL, RR, RL
        Eigen::Vector4d mean_vertical_grf{Eigen::Vector4d::Zero()};
        Eigen::Vector4d mean_load_fraction{Eigen::Vector4d::Zero()};
        double grf_load_imbalance_rms{0.0};      // RMS deviation of load fraction from ideal 0.25

        // Constraint validation
        std::size_t constraint_rows_checked{0};
        std::size_t constraint_violations{0};
        double constraint_violation_rate{0.0};   // violating rows / checked rows
        std::size_t constraint_solves_checked{0};
        std::size_t constraint_solves_with_violation{0};
        double constraint_solve_violation_rate{0.0};
        double max_constraint_violation{0.0};
    };

    struct KFSummary {
        // [px, py, pz, vx, vy, vz]
        BodyState state_rmse{BodyState::Zero()};

        // RMS for each of the 28 residual channels
        ResidualVector residual_rms{ResidualVector::Zero()};

        // Grouped residual RMS values
        double foot_position_residual_rms{0.0}; // first 12 channels, m
        double foot_velocity_residual_rms{0.0}; // next 12 channels, m/s
        double foot_height_residual_rms{0.0};   // last 4 channels, m

        // NIS statistics
        double mean_nis{0.0};
        double max_nis{0.0};
        std::size_t nis_samples{0};

        // Seconds from first KF validation sample until convergence.
        // NaN means the convergence criterion was never satisfied.
        double convergence_time{std::numeric_limits<double>::quiet_NaN()};
    };

    explicit ValidationMetrics(
        double convergence_position_threshold = 0.02,
        double convergence_velocity_threshold = 0.05,
        double convergence_hold_time = 0.5);

    void reset();

    // ---------------- MPC / controller metrics ----------------

    // Call whenever a valid body state/reference sample is available.
    // current_position should preferably be simulator ground truth during simulation validation.
    void updateControllerState(
        const Eigen::Vector3d& current_position,
        double reference_height,
        double roll,
        double pitch,
        bool steady_state);

    // Call at the rate at which the torque being evaluated is actually updated/applied.
    void updateControlInput(const JointTorque& torque, double dt);

    // GRF layout: [FRx,FRy,FRz, FLx,FLy,FLz, RRx,RRy,RRz, RLx,RLy,RLz].
    void updateGRF(const GRFVector& ground_reaction_forces);

    // Validates l <= A*u <= ub for one MPC solve.
    // tolerance prevents tiny numerical solver error being counted as a violation.
    void updateConstraints(
        const Eigen::MatrixXd& A,
        const Eigen::VectorXd& lower_bound,
        const Eigen::VectorXd& upper_bound,
        const Eigen::VectorXd& solution,
        double tolerance = 1e-4);

    // ---------------- Kalman filter metrics ----------------

    // kf_state and simulator_state use [px, py, pz, vx, vy, vz].
    // residual can be your current post-update residual y - C*x_hat.
    // sim_time_seconds should be monotonic simulation/ROS time.
    void updateKalmanState(
        const BodyState& kf_state,
        const BodyState& simulator_state,
        const ResidualVector& residual,
        double sim_time_seconds);

    // NIS must use the PRE-UPDATE innovation nu = y - C*x_hat_minus
    // and S = C*P_minus*C' + R.
    void updateNIS(
        const InnovationVector& innovation,
        const InnovationCovariance& innovation_covariance);

    MPCSummary getMPCSummary() const;
    KFSummary getKFSummary() const;

private:
    // Convergence definition
    double convergence_position_threshold_;
    double convergence_velocity_threshold_;
    double convergence_hold_time_;

    // Controller tracking accumulators
    std::size_t controller_samples_{0};
    double height_sq_error_sum_{0.0};
    double roll_sq_sum_{0.0};
    double pitch_sq_sum_{0.0};

    std::size_t steady_state_samples_{0};
    double steady_state_height_error_sum_{0.0};

    bool have_initial_xy_{false};
    Eigen::Vector2d initial_xy_{Eigen::Vector2d::Zero()};
    Eigen::Vector2d latest_xy_{Eigen::Vector2d::Zero()};

    // Control input accumulators
    bool have_previous_torque_{false};
    JointTorque previous_torque_{JointTorque::Zero()};
    double torque_sq_integral_{0.0};
    double torque_rate_sq_integral_{0.0};
    double control_time_{0.0};
    double smoothness_time_{0.0};

    // GRF accumulators
    std::size_t grf_samples_{0};
    std::size_t grf_load_fraction_samples_{0};
    Eigen::Vector4d vertical_grf_sum_{Eigen::Vector4d::Zero()};
    Eigen::Vector4d load_fraction_sum_{Eigen::Vector4d::Zero()};
    double load_fraction_error_sq_sum_{0.0};

    // Constraint accumulators
    std::size_t constraint_rows_checked_{0};
    std::size_t constraint_violations_{0};
    std::size_t constraint_solves_checked_{0};
    std::size_t constraint_solves_with_violation_{0};
    double max_constraint_violation_{0.0};

    // KF accumulators
    std::size_t kf_samples_{0};
    BodyState kf_state_sq_error_sum_{BodyState::Zero()};
    ResidualVector residual_sq_sum_{ResidualVector::Zero()};

    double nis_sum_{0.0};
    double max_nis_{0.0};
    std::size_t nis_samples_{0};

    // KF convergence tracking
    bool have_kf_start_time_{false};
    double kf_start_time_{0.0};
    bool convergence_candidate_active_{false};
    double convergence_candidate_start_time_{0.0};
    bool converged_{false};
    double convergence_time_{std::numeric_limits<double>::quiet_NaN()};
};
