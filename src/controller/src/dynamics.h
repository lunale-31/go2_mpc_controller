#pragma once

#include <Eigen/Dense>
#include <iostream> 
#include <cmath>
#include <vector>
#include <unsupported/Eigen/MatrixFunctions>

class Dynamics{

    private:
        double mass{};
        Eigen::Matrix3d body_inertia; 
        Eigen::Matrix<double, 13,13> m_A; // 13 states including gravity
        Eigen::Matrix<double, 12,13> m_C; // 12 states and 1 gravity which we dont measure
        Eigen::MatrixXd m_B; 

        // Discrete A and B
        Eigen::Matrix<double,13,13>m_Ad;
        Eigen::MatrixXd m_Bd; 
        
    public: 
        Dynamics();

        void computeA(double yaw);
        void computeB(std::vector<Eigen::Vector3d>& foot_positions_world, double yaw);

        // Discretize function
        void discretize(double dt);

        // Common functions 
        Eigen::Matrix3d getRotationMatrix(double yaw); 
        Eigen::Matrix3d getSkewSymMatrix(Eigen::Vector3d& r); // 3d because we need r skewsym matrix, which is a position vector of xyz

        // Read-only member functions
        Eigen::Matrix<double,13,13> getA() const; 
        Eigen::Matrix<double, 12,13> getC() const;
        Eigen::MatrixXd getB() const;
};