#pragma once

#include "dynamics.h"
#include "kinematics.h"
#include "state_estimator.h"
#include "foot_index.h"

#include <rclcpp/rclcpp.hpp>

#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <std_msgs/msg/bool.hpp>

#include <cmath>
#include <array>
#include <algorithm>
#include <optional>

#include "go2_interfaces/msg/estimator_debug.hpp"
#include "go2_interfaces/msg/estimated_state.hpp"

class StateEstimatorNode : public rclcpp::Node{

    public:
        StateEstimatorNode(); 

    private:
        void loadParams();
        void controller_callback(const unitree_go::msg::LowState::SharedPtr msg); 
        void initialize_callback(const std_msgs::msg::Bool::SharedPtr msg); 

        // Ros2 communication variables initialization 
        rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr publisher_; 
        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr subscriber_;  

        rclcpp::Publisher<go2_interfaces::msg::EstimatorDebug>::SharedPtr est_debug_; 
        rclcpp::Publisher<go2_interfaces::msg::EstimatedState>::SharedPtr est_state_pub;
        
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kf_initialize_sub;

        // Initializations of member variables 
        std::vector<double> q_ref; 
        std::vector<double> x_ref;
        
        std::vector<common::LegSide> leg_sides;
        std::vector<Eigen::Vector3d> hip_offset; 
        
        double obs_dt{};

        bool valid{false};

        bool kf_init_requested{false};
        bool kf_initialized{false};
        
        // Gait planner variables
        std::vector<bool> planned_contacts;

        // Hold kinematics
        std::vector<Eigen::Matrix3f> leg_jacobians_{
            Eigen::Matrix3f::Identity(), Eigen::Matrix3f::Identity(),
            Eigen::Matrix3f::Identity(), Eigen::Matrix3f::Identity()};
        std::vector<Eigen::Vector3d> foot_positions_body_{
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
        std::vector<Eigen::Vector3d> foot_positions_world_{
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
        std::vector<Eigen::Vector3d> est_foot_positions_world_{
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
 
        // MPC computation components 
        std::unique_ptr<KalmanFilter> filter_; // unique_ptr makes the controller class to exclusively own this object/member
        Dynamics go2; 
        Eigen::Matrix<double,13,1> x_curr; 
};