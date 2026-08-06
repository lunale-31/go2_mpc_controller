#pragma once

#include "dynamics.h"
#include "kinematics.h"

#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <cmath>

class Go2MPC : public rclcpp::Node{

    public:
        Go2MPC(); 

    private:
        void loadParams();
        void controller_callback(const unitree_go::msg::LowState::SharedPtr msg); 

        // Ros2 communication variables initialization 
        rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr publisher_; 
        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr subscriber_;  

        // Initializations of member variables 
        std::vector<double> q_ref; 
        std::vector<double> x_ref;
        
        std::vector<common::LegSide> leg_sides;
        std::vector<Eigen::Vector3d> hip_offset; 
        
        double dt{}; 
        double kp_{};
        double kd_{};

        // MPC computation components 
        Dynamics go2; 
};