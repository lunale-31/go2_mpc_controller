#pragma once

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

#include "go2_interfaces/msg/estimator_debug.hpp"
#include "go2_interfaces/msg/estimated_state.hpp"
#include "go2_interfaces/msg/mpc_command.hpp"

enum class ControlState {
    BOOT, 
    IDLE,
    SMOOTH_RAISE,
    KF_INITIALIZE,
    MPC_INITIALIZE,
    MPC_RUNNING,
    STOP,
    EMERGENCY_STOP
};

class HighLevelControl : public rclcpp::Node{

    public:
        // Constructor
        HighLevelControl();

    private: 
        // Enum state member
        ControlState m_s{ControlState::BOOT};

        // Necessary params
        void loadParams(); 

        // Pub and Sub 
        rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr cmd_pub_;
        rclcpp::Subscription<go2_interfaces::msg::EstimatedState>::SharedPtr est_sub_;
        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
        rclcpp::Subscription<go2_interfaces::msg::MpcCommand>::SharedPtr mpc_command_sub_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kf_initialize_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mpc_initialize_pub_; 

        // Callbacks
        void estimatedStateCallback(const go2_interfaces::msg::EstimatedState::SharedPtr msg); 
        void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg); 
        void mpcCommandCallback(const go2_interfaces::msg::MpcCommand::SharedPtr msg);

        // Main control loop
        void controlLoop();

        // Change state
        void changeState(ControlState s);

        // State executions
        bool sensorsValid();
        void runSmoothRaise(unitree_go::msg::LowCmd &cmd);
        void runEmergencyStop(unitree_go::msg::LowCmd &cmd); 
        void runMpcCommand(unitree_go::msg::LowCmd &cmd, std::array<double,12> ground_force);

        // Time variables
        std::optional<rclcpp::Time> kf_start_time;
        std::optional<rclcpp::Time> mpc_start_time;

        double kf_warmup_time_{4.0};

        // Smooth raise variables 
        rclcpp::TimerBase::SharedPtr control_timer_;
        std::array<double, 12> q_start_{};
        std::vector<double> q_ref{};

        bool crouch_initialized_{false};
        bool crouch_complete_{false};

        std::optional<rclcpp::Time> crouch_start_time_;
        double crouch_transition_time_{3.0};

        double joint_dt_{0.002};

        double kp_{10.0};
        double kd_{1.0};

        // Latest msg variables
        unitree_go::msg::LowState::SharedPtr latest_low_state_;
        go2_interfaces::msg::EstimatedState::SharedPtr latest_est_state_;
        go2_interfaces::msg::MpcCommand::SharedPtr latest_mpc_cmd_;

        // Boolean flags
        bool low_state_received_{false};
        bool estimated_state_received_{false};
        bool mpc_command_received_{false};

        bool boot_complete_{false};
        bool idle_complete_{false};
        bool smooth_raise_complete_{false};
        bool kf_complete_{false};
        bool mpc_initialize_{false}; 

        // Read current state
        ControlState getCurrentState() const;
        std::string getStateName() const;
};