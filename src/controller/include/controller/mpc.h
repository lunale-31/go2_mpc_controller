#pragma once

#include "dynamics.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <rclcpp/rclcpp.hpp>

#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <std_msgs/msg/bool.hpp>

#include <cmath>
#include <array>
#include <algorithm>
#include <optional>
#include <memory>
#include <chrono>
#include <limits>
#include <iostream>

#include "go2_interfaces/msg/mpc_command.hpp"
#include "go2_interfaces/msg/estimated_state.hpp"

#include "osqp++.h"

class MPC{
    public:
        // States and inputs number
        static constexpr int states_nr = 13; 
        static constexpr int inputs_nr = 12;  
        
        // Constructor to initialize params
        MPC(double dt); 

        // Dynamics matrices
        void computeDynamics(Dynamics& go2,const std::vector<Eigen::Vector3d>& foot_pos_com_world,double current_yaw,const Eigen::Matrix3d& inertia_body,double mass);
        void buildMatrix();

        // set Reference
        void setReference(Eigen::Matrix<double, 13,1>& x_ref);

        // Set Q1 and Q2 weights
        void setWeights(); 

        // Build cost
        void buildCost(Eigen::Matrix<double,13,1>& x_curr);

        // Build constraints
        void buildConstraints(std::vector<Eigen::Matrix3d>& leg_jacobians_world, std::array<double, 12>& gravity_tau);

        // solve QP to obtain optimal control signal 
        bool solve();

        // Read-only member functions
        Eigen::Matrix<double,13,1> getReference(); 
        Eigen::Matrix<double, 12, 1> getControlSignal();

        const Eigen::MatrixXd& getConstraintMatrix() const;
        const Eigen::VectorXd& getLowerBounds() const;
        const Eigen::VectorXd& getUpperBounds() const;
        const Eigen::VectorXd& getFullSolution() const;

    private:
        // Constants related to prediction and control horizon
        const int hp{10},hc{10};
        
        // Other constants related to sampling and constraints 
        double mpc_dt_{0.05};
        double mu_{0.4};
        double fz_min{5.0};
        double fz_max{100.0}; 
        double tau_min{-23.7}, tau_max{23.7}, tau_min_calf{-45.43}, tau_max_calf{45.43};
        double negInf = -std::numeric_limits<double>::infinity();
        
        // State, Reference state, constraint matrix
        Eigen::Matrix<double, 13, 1> m_xref;
        Eigen::Matrix<double, 13, 1> m_x; 
        Eigen::Matrix<double, 12, 1> m_u;
        
        Eigen::Matrix<double, 13, 13> m_Ad;
        Eigen::MatrixXd m_Bd;
        Eigen::Matrix<double, 12,13> m_C;

        Eigen::MatrixXd m_constraint_matrix; 
        Eigen::VectorXd m_lower_bounds;
        Eigen::VectorXd m_upper_bounds;

        // QP prediction matrices
        /* As hp = hc = 5, Aqp = (13 x hp) x 13, Bqp = (13 x hp) x (12 x hc), X = (13 x hp) x 1, U = (12 x hc) x 1 */
        Eigen::MatrixXd m_Aqp;
        Eigen::MatrixXd m_Bqp;
        Eigen::MatrixXd m_Xqp;
        Eigen::VectorXd m_Uqp;
        Eigen::MatrixXd m_Xqpref;
        Eigen::MatrixXd m_Q1qp; 
        Eigen::MatrixXd m_Q2qp;

        // QP cost vars
        Eigen::MatrixXd Hqp;
        Eigen::VectorXd gqp;
        double rho_qp; 

        // Weights
        Eigen::Matrix<double, states_nr, states_nr> Q1_; // State error weight
        Eigen::Matrix<double, inputs_nr, inputs_nr> Q2_; // Control effort weight 
        Eigen::Matrix<double, states_nr, states_nr> Qf_; // Terminal state weight

        // Dynamics object
        Dynamics go_2; 
};