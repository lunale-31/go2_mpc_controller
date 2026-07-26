#pragma once

#include "foot_index.h"

#include <Eigen/Dense>
#include <cmath>
#include <limits>

class KalmanFilter{
    
    private:
    
    // Initial x
    Eigen::Matrix<double, 18, 1> m_xi;

    // State vector of observer 
    Eigen::Matrix<double, 18, 1> m_xhat;  
    
    // Input vector of observer
    Eigen::Vector3d m_u; 

    // Model matrices
    Eigen::Matrix<double, 18, 18> m_A; 
    Eigen::Matrix<double, 18, 3> m_B; 
    Eigen::Matrix<double, 28,18> m_C; 
    
    // Measurement vector
    Eigen::Matrix<double, 28,1> m_y; 
    Eigen::Matrix<double, 28,1> m_yhat;

    // Innovation vector
    Eigen::Matrix<double, 28, 1> m_innovation; 

    // Covariance of state
    Eigen::Matrix<double, 18,18> P; 

    // Covariance of innovation vector 
    Eigen::Matrix<double, 28, 28> S;

    // Covariance of process and measurement noise
    Eigen::Matrix<double, 18, 18> R1; 
    Eigen::Matrix<double, 28, 28> R2; 

    // Kalman gain (Nr of states x Nr of measurements)
    Eigen::Matrix<double, 18, 28> L; 

    /* Validation Metrics */
    double NIS; 

    public:

    // Constructor for initialization
    KalmanFilter(double dt); 

    // Initialize 
    bool initializeKF(const std::vector<Eigen::Vector3d>& foot_positions_world);

    // Prediction step
    void predictionStep(const Eigen::Vector3d& u); 

    // Filtering step 
    void filteringStep(const std::vector<Eigen::Vector3d>& foot_positions_world, const std::vector<Eigen::Vector3d>& foot_velocities_world); 

    // Compute NIS
    double computeNIS(Eigen::Matrix<double, 28,1> innovation, Eigen::Matrix<double, 28,28> s);

    // Read-only member functions
    Eigen::Matrix<double, 18, 1> getXhat() const;
    Eigen::Matrix<double, 28, 1> getResidualError() const;
    Eigen::Matrix<double, 28, 1> getInnovation() const; 
    Eigen::Matrix<double, 28, 1> getMeasurement() const; 
    Eigen::Matrix<double, 28, 1> getMeasurementPrediction() const; 
    Eigen::Matrix<double, 18, 1> getStateCovarianceDiag() const;
    Eigen::Matrix<double, 28, 1> getInnovationCovarianceDiag() const; 

    double getNIS() const;
};