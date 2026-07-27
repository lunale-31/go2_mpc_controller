#pragma once

#include "dynamics.h"

#include <Eigen/Dense>
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

#include "go2_interfaces/msg/mpc_command.hpp"
#include "go2_interfaces/msg/estimated_state.hpp"

class MPC : public rclcpp::Node{
    /* TODO:
    0) Read current robot state x0. (DONE)
    1) Create reference state vector Xref (time varying over prediction horizon Hp if needed or constant if not)
    --For every time step--  
    2) Pre-compute A0, A1, A2.. B0, B1, B2... matrices by using Xref (for stand-up, all the matrices are same as xref is constant)
    3) Unroll the dynamics for Hp prediction horizon to create a big Aqp and Bqp matrices (X = Aqp(X0) + Bqp(U)).
    4) Create the cost function J and use Xref for it. Then, compute H and g matrix from the paper to define simplified QP equation. 
    5) Create constraints related to friction and actuator limits, combine them as a single matrix inequality.
    6) Use solver to obtain the solution for the given QP equation and constraint matrix.
    7) Send the control signal u0 to the taw formula. */

    public:
        // States and inputs number
        static constexpr int states_nr = 13; 
        static constexpr int inputs_nr = 12;  
        
        // Constructor to initialize params
        MPC(double dt); 

        // Time varying matrices
        void computeLTVMatrices(double dt, Dynamics& go2);

        // set Reference
        void setReference(Eigen::Matrix<double, 13,1>& x_ref);

        // Set Q1 and Q2 weights
        void setWeights(); 

        // Formulate and solve QP to obtain optimal control signal 
        void solve(Eigen::Matrix<double, 13,1>& x_ref, Eigen::Matrix<double, 13,1>& x);

        // Read-only member functions
        Eigen::Matrix<double,13,1> getReference(); 
        
    private:
        // Constants related to prediction and control horizon
        const int hp{5},hc{5};
        
        // Other constants related to sampling and constraints 
        double mpc_dt_{0.05};
        double mu_{0.4};
        double fz_min;
        double fz_max; 

        // State, Reference state, constraint matrix
        Eigen::Matrix<double, 13, 1> m_xref;
        Eigen::Matrix<double, 13, 1> m_x; 
        Eigen::Matrix<double, 3,6> m_constraint_matrix; 
        Eigen::Matrix<double, 4,1> m_constraint_variable;
        Eigen::Matrix<double, 6,1> m_constraint_output;

        // Weights
        Eigen::Matrix<double, states_nr, states_nr> Q1_; // State error weight
        Eigen::Matrix<double, states_nr, inputs_nr> Q2_; // Control effort weight 

        // Dynamics object
        Dynamics go_2; 
};