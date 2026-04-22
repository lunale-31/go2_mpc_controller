#pragma once

#include <rclcpp/node.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <unitree_go/msg/low_state.hpp>

#include "lowlevel/Leg.h"
#include "lowlevel/Joint.h"

namespace go2_utils::interface {
    class LowLevelControl {
    public:
        /**
         * Constructor.
         * @param node A reference to a ROS2 node used to interface with the robot.
         */
        explicit LowLevelControl(const rclcpp::Node::SharedPtr &node);

        /**
         * Gets the front-left leg.
         */
        [[nodiscard]] lowlevel::Leg::SharedPtr &frontLeft();

        /**
         * Gets the front-right leg.
         */
        [[nodiscard]] lowlevel::Leg::SharedPtr &frontRight();

        /**
         * Gets the back-left leg.
         */
        [[nodiscard]] lowlevel::Leg::SharedPtr &backLeft();

        /**
         * Gets the back-right leg.
         */
        [[nodiscard]] lowlevel::Leg::SharedPtr &backRight();

        /**
         * Gets a leg by index.
         */
        lowlevel::Leg::SharedPtr &leg(unsigned index);

        /**
         * Gets a joint by index.
         */
        lowlevel::Joint::SharedPtr &joint(unsigned index);

        /**
         * Gets the IMU state
         */
        unitree_go::msg::IMUState &imu_state();

        /**
         * Gets the BMS state
         */
        unitree_go::msg::BmsState &bms_state();

        /**
         * Returns whether the controller has received a low-level state from the robot.
         */
        bool was_state_received();

        /**
         * Sends all current joint commands to the robot. 
         */
        void publish();

        using SharedPtr = std::shared_ptr<LowLevelControl>;

    private:
        void initialize_motors();

        void initialize_legs();

        void update_state(const unitree_go::msg::LowState &state);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr publisher_;
        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr subscription_;

        // motors and locks
        lowlevel::Joint::SharedPtr motors_[12];
        lowlevel::Leg::SharedPtr legs_[4];

        // was state received?
        bool state_received_ = false;

        // IMU state
        unitree_go::msg::IMUState imu_state_;
        
        // BMS state
        unitree_go::msg::BmsState bms_state_;

        // command
        std::unique_ptr<unitree_go::msg::LowCmd> low_command_;
    };
} // namespace interface
