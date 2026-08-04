#pragma once

#include "dynamics.h"
#include "mpc.h"

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

class MPCNode : public rclcpp::Node{
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
        // Constructor
        MPCNode();

        // Time varying matrices
        void computeLTVMatrices(double dt);

        // Compute Smooth Reference State
        std::pair<double,double>  computeSmoothHeight(Eigen::Matrix<double, 13,1> x_curr);

    private:
        // Load Params
        void loadParams(); 

        // Sampling time
        double mpc_dt_;

        // Dynamics and mpc object
        Dynamics go2; 
        std::unique_ptr<MPC> mpc_; 

        // ROS2 vars
        rclcpp::TimerBase::SharedPtr force_control_timer_;

        rclcpp::Publisher<go2_interfaces::msg::MpcCommand>::SharedPtr mpc_cmd_pub_; 
        rclcpp::Subscription<go2_interfaces::msg::EstimatedState>::SharedPtr x_sub_; 
        
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mpc_initialize_sub; 

        // ROS2 callbacks
        void stateCallback(go2_interfaces::msg::EstimatedState::SharedPtr msg);
        void mpcInitializeCallback(std_msgs::msg::Bool::SharedPtr msg); 
        void mpcControlLoop();

        // Latest vars
        go2_interfaces::msg::EstimatedState::SharedPtr latest_state_msg;

        // Smooth trajectory vars
        bool stand_initialized_{false};
        bool stand_complete_{false};

        std::optional<rclcpp::Time> stand_start_time_;
        double stand_transition_time_{5.0};

        double z_start{};
        double z_ref{0.3};
        Eigen::Matrix<double, 13,1> x_ref_;

        // Bool vars
        bool mpc_initialize_request{false};
        bool mpc_initialize_{false};
        bool hold_reference_initialized_{false};

        // Hold state
        double x_hold_{0.0};
        double y_hold_{0.0};
        double yaw_hold_{0.0};
};