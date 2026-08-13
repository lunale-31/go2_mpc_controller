#include "controller/validation_metrics.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

ValidationMetrics::ValidationMetrics(
    double convergence_position_threshold,
    double convergence_velocity_threshold,
    double convergence_hold_time)
    : convergence_position_threshold_(convergence_position_threshold),
      convergence_velocity_threshold_(convergence_velocity_threshold),
      convergence_hold_time_(convergence_hold_time)
{
    if (convergence_position_threshold_ <= 0.0 ||
        convergence_velocity_threshold_ <= 0.0 ||
        convergence_hold_time_ < 0.0) {
        throw std::invalid_argument("Invalid ValidationMetrics convergence thresholds.");
    }
}

void ValidationMetrics::reset()
{
    controller_samples_ = 0;
    height_sq_error_sum_ = 0.0;
    roll_sq_sum_ = 0.0;
    pitch_sq_sum_ = 0.0;

    steady_state_samples_ = 0;
    steady_state_height_error_sum_ = 0.0;

    have_initial_xy_ = false;
    initial_xy_.setZero();
    latest_xy_.setZero();

    have_previous_torque_ = false;
    previous_torque_.setZero();
    torque_sq_integral_ = 0.0;
    torque_rate_sq_integral_ = 0.0;
    control_time_ = 0.0;
    smoothness_time_ = 0.0;

    grf_samples_ = 0;
    grf_load_fraction_samples_ = 0;
    vertical_grf_sum_.setZero();
    load_fraction_sum_.setZero();
    load_fraction_error_sq_sum_ = 0.0;

    constraint_rows_checked_ = 0;
    constraint_violations_ = 0;
    constraint_solves_checked_ = 0;
    constraint_solves_with_violation_ = 0;
    max_constraint_violation_ = 0.0;

    kf_samples_ = 0;
    kf_state_sq_error_sum_.setZero();
    residual_sq_sum_.setZero();

    nis_sum_ = 0.0;
    max_nis_ = 0.0;
    nis_samples_ = 0;

    have_kf_start_time_ = false;
    kf_start_time_ = 0.0;
    convergence_candidate_active_ = false;
    convergence_candidate_start_time_ = 0.0;
    converged_ = false;
    convergence_time_ = std::numeric_limits<double>::quiet_NaN();
}

void ValidationMetrics::updateControllerState(
    const Eigen::Vector3d& current_position,
    double reference_height,
    double roll,
    double pitch,
    bool steady_state)
{
    if (!current_position.allFinite() ||
        !std::isfinite(reference_height) ||
        !std::isfinite(roll) ||
        !std::isfinite(pitch)) {
        return;
    }

    const double height_error = current_position.z() - reference_height;

    height_sq_error_sum_ += height_error * height_error;
    roll_sq_sum_ += roll * roll;
    pitch_sq_sum_ += pitch * pitch;
    ++controller_samples_;

    if (steady_state) {
        steady_state_height_error_sum_ += height_error;
        ++steady_state_samples_;
    }

    const Eigen::Vector2d current_xy = current_position.head<2>();
    if (!have_initial_xy_) {
        initial_xy_ = current_xy;
        have_initial_xy_ = true;
    }
    latest_xy_ = current_xy;
}

void ValidationMetrics::updateControlInput(const JointTorque& torque, double dt)
{
    if (!torque.allFinite() || !std::isfinite(dt) || dt <= 0.0) {
        return;
    }

    torque_sq_integral_ += torque.squaredNorm() * dt;
    control_time_ += dt;

    if (have_previous_torque_) {
        const JointTorque torque_rate = (torque - previous_torque_) / dt;
        torque_rate_sq_integral_ += torque_rate.squaredNorm() * dt;
        smoothness_time_ += dt;
    }

    previous_torque_ = torque;
    have_previous_torque_ = true;
}

void ValidationMetrics::updateGRF(const GRFVector& ground_reaction_forces)
{
    if (!ground_reaction_forces.allFinite()) {
        return;
    }

    Eigen::Vector4d vertical_grf;
    for (int leg = 0; leg < 4; ++leg) {
        vertical_grf(leg) = ground_reaction_forces(leg * 3 + 2);
    }

    vertical_grf_sum_ += vertical_grf;
    ++grf_samples_;

    const double total_vertical_force = vertical_grf.sum();
    if (std::abs(total_vertical_force) > 1e-9) {
        const Eigen::Vector4d load_fraction = vertical_grf / total_vertical_force;
        load_fraction_sum_ += load_fraction;

        const Eigen::Vector4d ideal_fraction = Eigen::Vector4d::Constant(0.25);
        load_fraction_error_sq_sum_ +=
            (load_fraction - ideal_fraction).squaredNorm();

        ++grf_load_fraction_samples_;
    }
}

void ValidationMetrics::updateConstraints(
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& lower_bound,
    const Eigen::VectorXd& upper_bound,
    const Eigen::VectorXd& solution,
    double tolerance)
{
    if (A.cols() != solution.size() ||
        A.rows() != lower_bound.size() ||
        A.rows() != upper_bound.size()) {
        throw std::invalid_argument("Constraint matrix/vector dimensions do not match.");
    }

    if (tolerance < 0.0 || !std::isfinite(tolerance)) {
        throw std::invalid_argument("Constraint tolerance must be finite and non-negative.");
    }

    if (!A.allFinite() || !solution.allFinite()) {
        return;
    }

    const Eigen::VectorXd values = A * solution;
    ++constraint_solves_checked_;
    bool solve_has_violation = false;

    for (Eigen::Index i = 0; i < values.size(); ++i) {
        const double value = values(i);
        double violation = 0.0;
        bool row_has_bound = false;

        if (std::isfinite(lower_bound(i))) {
            row_has_bound = true;
            violation = std::max(violation, lower_bound(i) - value);
        }

        if (std::isfinite(upper_bound(i))) {
            row_has_bound = true;
            violation = std::max(violation, value - upper_bound(i));
        }

        if (!row_has_bound) {
            continue;
        }

        ++constraint_rows_checked_;

        if (violation > tolerance) {
            ++constraint_violations_;
            solve_has_violation = true;
            max_constraint_violation_ =
                std::max(max_constraint_violation_, violation);
        }
    }

    if (solve_has_violation) {
        ++constraint_solves_with_violation_;
    }
}

void ValidationMetrics::updateKalmanState(
    const BodyState& kf_state,
    const BodyState& simulator_state,
    const ResidualVector& residual,
    double sim_time_seconds)
{
    if (!kf_state.allFinite() ||
        !simulator_state.allFinite() ||
        !residual.allFinite() ||
        !std::isfinite(sim_time_seconds)) {
        return;
    }

    if (!have_kf_start_time_) {
        kf_start_time_ = sim_time_seconds;
        have_kf_start_time_ = true;
    }

    const BodyState error = kf_state - simulator_state;
    kf_state_sq_error_sum_ += error.array().square().matrix();
    residual_sq_sum_ += residual.array().square().matrix();
    ++kf_samples_;

    if (converged_) {
        return;
    }

    const double position_error = error.head<3>().norm();
    const double velocity_error = error.tail<3>().norm();

    const bool within_threshold =
        position_error <= convergence_position_threshold_ &&
        velocity_error <= convergence_velocity_threshold_;

    if (within_threshold) {
        if (!convergence_candidate_active_) {
            convergence_candidate_active_ = true;
            convergence_candidate_start_time_ = sim_time_seconds;
        }

        const double time_inside_threshold =
            sim_time_seconds - convergence_candidate_start_time_;

        if (time_inside_threshold >= convergence_hold_time_) {
            converged_ = true;
            convergence_time_ =
                convergence_candidate_start_time_ - kf_start_time_;
        }
    } else {
        convergence_candidate_active_ = false;
    }
}

void ValidationMetrics::updateNIS(
    const InnovationVector& innovation,
    const InnovationCovariance& innovation_covariance)
{
    if (!innovation.allFinite() || !innovation_covariance.allFinite()) {
        return;
    }

    Eigen::LDLT<InnovationCovariance> ldlt(innovation_covariance);
    if (ldlt.info() != Eigen::Success) {
        return;
    }

    const InnovationVector solved = ldlt.solve(innovation);
    if (ldlt.info() != Eigen::Success || !solved.allFinite()) {
        return;
    }

    double nis = innovation.dot(solved);

    // Tiny negative values can appear from floating-point roundoff.
    if (nis < 0.0 && nis > -1e-10) {
        nis = 0.0;
    }

    if (!std::isfinite(nis) || nis < 0.0) {
        return;
    }

    nis_sum_ += nis;
    max_nis_ = std::max(max_nis_, nis);
    ++nis_samples_;
}

ValidationMetrics::MPCSummary ValidationMetrics::getMPCSummary() const
{
    MPCSummary summary;

    if (controller_samples_ > 0) {
        const double n = static_cast<double>(controller_samples_);
        summary.height_rmse = std::sqrt(height_sq_error_sum_ / n);
        summary.roll_rms = std::sqrt(roll_sq_sum_ / n);
        summary.pitch_rms = std::sqrt(pitch_sq_sum_ / n);
    }

    if (steady_state_samples_ > 0) {
        summary.steady_state_height_error =
            steady_state_height_error_sum_ /
            static_cast<double>(steady_state_samples_);
    }

    if (have_initial_xy_) {
        summary.xy_drift = (latest_xy_ - initial_xy_).norm();
    }

    summary.control_effort = torque_sq_integral_;
    summary.control_smoothness = torque_rate_sq_integral_;

    if (control_time_ > 0.0) {
        summary.torque_rms =
            std::sqrt(torque_sq_integral_ / (12.0 * control_time_));
    }

    if (smoothness_time_ > 0.0) {
        summary.torque_rate_rms =
            std::sqrt(torque_rate_sq_integral_ / (12.0 * smoothness_time_));
    }

    if (grf_samples_ > 0) {
        const double n = static_cast<double>(grf_samples_);
        summary.mean_vertical_grf = vertical_grf_sum_ / n;
    }

    if (grf_load_fraction_samples_ > 0) {
        const double n = static_cast<double>(grf_load_fraction_samples_);
        summary.mean_load_fraction = load_fraction_sum_ / n;
        summary.grf_load_imbalance_rms =
            std::sqrt(load_fraction_error_sq_sum_ / (4.0 * n));
    }

    summary.constraint_rows_checked = constraint_rows_checked_;
    summary.constraint_violations = constraint_violations_;
    summary.constraint_solves_checked = constraint_solves_checked_;
    summary.constraint_solves_with_violation = constraint_solves_with_violation_;
    summary.max_constraint_violation = max_constraint_violation_;

    if (constraint_rows_checked_ > 0) {
        summary.constraint_violation_rate =
            static_cast<double>(constraint_violations_) /
            static_cast<double>(constraint_rows_checked_);
    }

    if (constraint_solves_checked_ > 0) {
        summary.constraint_solve_violation_rate =
            static_cast<double>(constraint_solves_with_violation_) /
            static_cast<double>(constraint_solves_checked_);
    }

    return summary;
}

ValidationMetrics::KFSummary ValidationMetrics::getKFSummary() const
{
    KFSummary summary;

    if (kf_samples_ > 0) {
        const double n = static_cast<double>(kf_samples_);

        summary.state_rmse =
            (kf_state_sq_error_sum_ / n).array().sqrt().matrix();

        summary.residual_rms =
            (residual_sq_sum_ / n).array().sqrt().matrix();

        // Group RMS: sqrt(mean of squared samples across time and channels).
        summary.foot_position_residual_rms = std::sqrt(
            residual_sq_sum_.segment<12>(0).sum() / (12.0 * n));

        summary.foot_velocity_residual_rms = std::sqrt(
            residual_sq_sum_.segment<12>(12).sum() / (12.0 * n));

        summary.foot_height_residual_rms = std::sqrt(
            residual_sq_sum_.segment<4>(24).sum() / (4.0 * n));
    }

    if (nis_samples_ > 0) {
        summary.mean_nis = nis_sum_ / static_cast<double>(nis_samples_);
        summary.max_nis = max_nis_;
        summary.nis_samples = nis_samples_;
    }

    summary.convergence_time = convergence_time_;

    return summary;
}
